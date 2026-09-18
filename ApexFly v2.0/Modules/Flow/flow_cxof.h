/**
 * @file flow_cxof.h
 * @brief Cheerson CX-OF 光流模块的串口帧解析。**纯逻辑，不碰外设、不建任务、不读时钟。**
 *
 * 本模块只回答一个问题：喂进来一个字节，现在有没有凑出一帧合格的光流数据。
 * 不启动 DMA、不创建任务、不做零点标定、不做有效性判定——那些都在 flow.c 里。
 * 这样分层有一个实际好处：这一层可以直接在 PC 上喂合成字节流做仿真
 * （见 `_inbox_会话提取/提取脚本/sim_flow.c`），坏校验、半截帧、粘包都能提前验。
 *
 * ── 帧格式（出处：`技术文档/mini光流资料/光流串口数据格式.png`）──
 *   0xFE | 0x04 | DX_L | DX_H | DY_L | DY_H | SUM | Quality | 0xAA
 *   · DX / DY   = **int16 小端**，含义是"**距上次更新以来的位移计数**"。
 *                 注意它是"这一帧走了多少"，不是速度、也不是毫米。
 *   · SUM       = 中间 4 个字节相加取低 8 位。
 *   · Quality   = 0~100。协议图明确写着"30 以下数据较差建议考虑舍弃"。
 *   · 串口       = 19200 8N1；40ms 一帧（25Hz）；模块自己发，不用轮询。
 *
 * ── 为什么是"四重校验"，而不是商家 demo 的"找 0xFE + 查累加和" ──────
 * 商家 demo（`opticalflow_iic.c` 的 `flow_decode`）只查首字节和累加和。
 * 但一个 9 字节的随机窗口有约 1/256 的概率正好凑对累加和；一旦把错位的窗口
 * 当成有效帧，后面几帧会连着错位。这里把四个特征一起查
 * （包头 + 长度 + 累加和 + 包尾），误判概率降到可以忽略；
 * 而且校验失败时**只丢一个字节**重新同步，不会整帧跳过（见 FlowCxof_ProcessByte）。
 */

#ifndef FLOW_CXOF_H
#define FLOW_CXOF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一帧固定 9 字节。 */
#define FLOW_CXOF_FRAME_SIZE 9U
/* 包头固定值。 */
#define FLOW_CXOF_HEADER 0xFEU
/* 第 2 字节固定值：光流数据的字节数。 */
#define FLOW_CXOF_LENGTH 0x04U
/* 包尾固定值。 */
#define FLOW_CXOF_TAIL 0xAAU
/* 图像质量下限，低于它本帧判为不可用。取协议图建议值 30。 */
#define FLOW_CXOF_QUALITY_MIN 30U

/**
 * @brief 一次 ProcessByte 的结果。
 *
 * `frame_ready` 与 `quality_ok` 是**两个独立的判断**，故意分开：
 * "解析成功"（四重校验全过）回答"这串字节是不是一帧"，
 * "质量合格"回答"这一帧的数据能不能用"。混成一个标志的话，
 * 就没法统计"帧在进来、但地面纹理不行"这种情况——而那正是最需要看见的。
 */
typedef struct
{
    /* true 表示本次调用凑满并校验通过了一帧（与质量无关）。 */
    bool frame_ready;
    /* true 表示本帧 quality >= FLOW_CXOF_QUALITY_MIN。frame_ready 为 false 时无意义。 */
    bool quality_ok;
    /* X 轴位移计数，本帧值（有符号）。 */
    int16_t dx;
    /* Y 轴位移计数，本帧值（有符号）。 */
    int16_t dy;
    /* 图像质量，0~100。 */
    uint8_t quality;
} FlowCxofFrame_t;

/**
 * @brief 解析器状态。
 *
 * 统计量放在这里而不是 flow.c：它们都是"解析这一层"的事，
 * 分开会让"校验失败到底发生在哪一步"变得难查。
 */
typedef struct
{
    /* 正在拼装的 9 个字节。 */
    uint8_t buffer[FLOW_CXOF_FRAME_SIZE];
    /* 已经攒了几个字节，0 表示在等包头。 */
    uint8_t index;
    /* 累计喂进来的字节数。帧率、波特率是否对，看它与 frame_count 的比值。 */
    uint32_t byte_count;
    /* 四重校验全过、成功解出的帧数。 */
    uint32_t frame_count;
    /* 累加和（第 7 字节 SUM）不符的次数。 */
    uint32_t checksum_error_count;
    /* 长度字节（第 2 字节）或包尾（第 9 字节）不符的次数。 */
    uint32_t format_error_count;
    /* 质量低于门槛、被判为不可用的帧数。 */
    uint32_t quality_reject_count;
    /* 校验失败后"丢一个字节重新同步"的次数。持续增长说明波特率或接线有问题。 */
    uint32_t resync_count;
} FlowCxofDevice_t;

/**
 * @brief 清空解析状态与全部统计量。建任务时调用一次。
 * @param device 解析器对象。
 * @return 成功返回 true，device 为空返回 false。
 */
bool FlowCxof_Init(FlowCxofDevice_t *device);

/**
 * @brief 送入一个字节，推进解析状态机。
 *
 * @param device 已初始化的解析器。
 * @param byte 从串口收到的一个字节。
 * @param frame 输出。本次凑出一帧时填好并返回 true；否则只把 frame_ready 清 false。
 * @return true 表示本次调用解出了一帧（**不代表质量合格**，看 frame->quality_ok）。
 */
bool FlowCxof_ProcessByte(FlowCxofDevice_t *device, uint8_t byte, FlowCxofFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_CXOF_H */
