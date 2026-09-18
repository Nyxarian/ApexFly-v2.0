/**
 * @file flow_cxof.c
 * @brief CX-OF 光流串口帧解析实现。纯逻辑，不依赖 HAL、不依赖 FreeRTOS。
 *
 * 只做字节流 → 一帧数据的转换，其余（DMA、环形缓冲、任务、零点标定）在 flow.c。
 */

#include "flow_cxof.h"

#include <string.h>

/**
 * @brief 校验攒满的 9 个字节是不是一帧。
 *
 * 检查顺序是有讲究的：先便宜且区分度高的（长度字节），再累加和，最后包尾。
 * 每一处失败都单独计数——台架上"哪一步在失败"直接指向不同的原因：
 *   format_error 多 ⇒ 波特率不对或接线干扰（字节被切碎）
 *   checksum_error 多 ⇒ 数据在传，但对不齐（多半还是波特率）
 */
static bool FlowCxof_Validate(FlowCxofDevice_t *device, const uint8_t *frame)
{
    uint8_t sum;

    if (frame[1] != FLOW_CXOF_LENGTH)
    {
        device->format_error_count++;
        return false;
    }

    sum = (uint8_t)(frame[2] + frame[3] + frame[4] + frame[5]);
    if (sum != frame[6])
    {
        device->checksum_error_count++;
        return false;
    }

    if (frame[8] != FLOW_CXOF_TAIL)
    {
        device->format_error_count++;
        return false;
    }

    return true;
}

bool FlowCxof_Init(FlowCxofDevice_t *device)
{
    if (device == NULL) return false;

    memset(device, 0, sizeof(*device));
    return true;
}

bool FlowCxof_ProcessByte(FlowCxofDevice_t *device, uint8_t byte, FlowCxofFrame_t *frame)
{
    if ((device == NULL) || (frame == NULL)) return false;

    frame->frame_ready = false;
    frame->quality_ok = false;
    device->byte_count++;

    /* 还没找到包头：只认 0xFE，其余字节直接丢（它们不构成任何帧）。 */
    if (device->index == 0U)
    {
        if (byte == FLOW_CXOF_HEADER)
        {
            device->buffer[0] = byte;
            device->index = 1U;
        }
        return false;
    }

    device->buffer[device->index] = byte;
    device->index++;

    /* 还没攒满一个窗口，继续等。 */
    if (device->index < FLOW_CXOF_FRAME_SIZE) return false;

    if (FlowCxof_Validate(device, device->buffer))
    {
        /* 小端拼回 int16：DX = 低字节 + 高字节<<8。 */
        int16_t dx = (int16_t)((uint16_t)device->buffer[2] | ((uint16_t)device->buffer[3] << 8));
        int16_t dy = (int16_t)((uint16_t)device->buffer[4] | ((uint16_t)device->buffer[5] << 8));
        uint8_t quality = device->buffer[7];

        device->index = 0U;
        device->frame_count++;

        frame->dx = dx;
        frame->dy = dy;
        frame->quality = quality;
        frame->frame_ready = true;

        /* 质量不够只记账，不丢帧——上层要能区分"没收到帧"和"收到了但看不清地面"。 */
        if (quality < FLOW_CXOF_QUALITY_MIN)
        {
            device->quality_reject_count++;
            frame->quality_ok = false;
        }
        else
        {
            frame->quality_ok = true;
        }

        return true;
    }

    /* ── 校验失败：**只丢一个字节**重新同步 ────────────────────────
       ⚠️ 这里不能"整帧丢弃、跳满 9 字节"。那样一旦错位，就要等到下一个
          恰好落在窗口首位的 0xFE 才能对齐，中间会连丢好几帧。
       做法：把窗口左移一字节（原来的第 2~9 字节变成第 1~8 字节），
          下一次调用就正好补满第 9 字节、再校验一次。
       ⚠️ 左移后如果第 1 字节已经不是 0xFE，这个窗口再也不可能成立，
          直接回到"等包头"状态，省掉无意义的逐字节尝试。 */
    device->resync_count++;
    memmove(&device->buffer[0], &device->buffer[1], FLOW_CXOF_FRAME_SIZE - 1U);
    device->index = (device->buffer[0] == FLOW_CXOF_HEADER) ? (FLOW_CXOF_FRAME_SIZE - 1U) : 0U;

    return false;
}
