/**
 * @file flow.h
 * @brief CX-OF 光流的"管理层"：串口接收、有效性判定、零点标定、累积流量。
 *
 * ── 本模块**不做什么**（这是本阶段最重要的一句话）────────────────
 * **不接任何控制。** 不写姿态、不写电机、不碰油门、不碰定高。
 * 输出只有一份"给打印看的快照"。所以本阶段的炸机风险是零，
 * 拆桨或不拆桨都一样。位置环/速度环是 OF6 的事。
 *
 * ── 和 flow_cxof 的分工 ────────────────────────────────────────
 *   flow_cxof.c  = 纯逻辑：字节流 → 一帧（能在 PC 上仿真）
 *   flow.c（本文件）= 有状态：DMA + 环形缓冲 + 任务 + 有效性 + 标定 + 累积
 *
 * ── 为什么零点标定是**必须**的（不是可选优化）────────────────────
 * 位置环是**纯 P、目标恒 0**：它把"累积流量"压回 0，也就是把**读数**压回 0。
 * 而 PMW3901 这类芯片静止时会报出很小的非零流量（像素噪声、光照不均、装配公差）。
 * 如果静止时读数本来偏 +3，环路就会让飞机**真的朝反方向匀速移动**，
 * 直到读数回到 0 —— 这就是"恒速漂移机"的成因。
 * ⇒ 不标零点，定点要么漂、要么根本没有定点可言。
 * （商家 demo 就没做这一步，见 `优化一下/光流接入方案.md` §2.3。）
 *
 * ── 本阶段**不做**的两件事（别以为漏了）────────────────────────
 *   ① **高度门禁**（"低于多少厘米光流不可信"）：可信高度区间还没实测，
 *      现在拍一个数只会变成假安全感。等 OF4 手推测试之后按实测定，属 OF6 的范围。
 *   ② **标定命令 `ofc`**（不重新上电就重标）：本阶段的验收要求就是"换地面重新上电"，
 *      多加一条串口命令会多碰一处解析器，不值得。
 */

#ifndef FLOW_H
#define FLOW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 零点标定的状态。给打印看，也是"那个零偏到底可不可信"的唯一指示。
 */
typedef enum
{
    /* 还没开始（本阶段不会出现：上电即进入 RUNNING）。 */
    FLOW_CAL_IDLE = 0U,
    /* 正在采样：这一段时间里的数据**不累积**，只用来求平均。 */
    FLOW_CAL_RUNNING = 1U,
    /* 标定成功，零点已生效，之后的每一帧都减掉它。 */
    FLOW_CAL_DONE = 2U,
    /* 采样期间飞机在动（均值过大）⇒ 本次作废，等一会儿自动重来。 */
    FLOW_CAL_FAILED = 3U
} FlowCalState_t;

/**
 * @brief 一次 Flow_GetState 的完整快照。字段全部只读。
 *
 * 量纲说明（重要，别搞混）：
 *   dx / dy / sum_x / sum_y 的单位都是"**计数**"（模块自己的像素位移计数），
 *   不是毫米、不是米。要换成米必须先知道"1 计数 = 多少米"，而那需要高度
 *   （flow ∝ 位移 ÷ 高度）—— 那一步是 OF12 的事。
 *   位置环（OF6）的目标正是把 sum 压回 0，所以**不知道尺度也能闭环**。
 */
typedef struct
{
    /* true 表示光流数据当前可用。为 false 时 dx/dy/sum 全部是**冻结**的旧值。 */
    bool valid;
    /* 零点标定状态，取 FlowCalState_t。 */
    uint8_t cal;
    /* 最近一帧的图像质量，0~100。地面纹理/光照够不够，就看它。 */
    uint8_t quality;
    /* 最近一帧的 X 轴位移计数（原始值，未减零点）。 */
    int16_t dx;
    /* 最近一帧的 Y 轴位移计数（原始值，未减零点）。 */
    int16_t dy;
    /* 标定得到的 X 零点，单位计数/帧。正常应该远小于 1。 */
    float zero_x;
    /* 标定得到的 Y 零点，单位计数/帧。 */
    float zero_y;
    /* 累积流量（已减零点），单位计数。位置环的反馈量，目标 0。 */
    float sum_x;
    /* 同上，Y 轴。 */
    float sum_y;
    /* 瞬时流量率（已减零点），单位计数/秒。速度环（OF6）的反馈量。 */
    float vx_cps;
    /* 同上，Y 轴。 */
    float vy_cps;
    /* 距最近一帧**合格**数据的毫秒数。 */
    uint32_t age_ms;
    /* 标定尝试过的次数（>1 说明失败过，值得看一眼）。 */
    uint32_t cal_attempts;
    /* 四重校验全过、成功解出的帧数。 */
    uint32_t frame_count;
    /* 累加和校验失败次数。 */
    uint32_t checksum_error_count;
    /* 长度字节/包尾不符次数。 */
    uint32_t format_error_count;
    /* 质量低于门槛被拒的帧数。 */
    uint32_t quality_reject_count;
    /* 丢一字节重新同步的次数。持续增长 ⇒ 波特率或接线有问题。 */
    uint32_t resync_count;
    /* 累计收到的字节数。与 frame_count 的比值能看出帧率对不对。 */
    uint32_t byte_count;
    /* UART 错误中断次数（ORE/FE/NE/PE）。 */
    uint32_t uart_error_count;
    /* 环形缓冲溢出丢字节数。 */
    uint32_t dma_drop_count;
} FlowState_t;

/**
 * @brief 清空全部状态与统计量。光流任务启动时调用一次，**不要在中途调**。
 *
 * 为什么强调"不要中途调"：它会把零点标定一并打回 RUNNING，
 * 于是累积量从头开始、零点被丢掉。台架上想重标零点，规矩是**重新上电**。
 */
void Flow_Init(void);

/**
 * @brief 光流任务入口。由 app_freertos.c 创建，优先级与 tofTask 同级（BelowNormal）。
 *
 * 任务里做四件事：维持 DMA 接收、把环形缓冲里的字节喂给解析器、
 * 每帧更新统计与累积、每个轮询周期更新一次有效性。
 */
void Flow_Task(void *argument);

/**
 * @brief 取最近一次快照。只做一次结构拷贝，可在低频任务里调用（打印用）。
 * @param state 输出，不能为空。
 */
void Flow_GetState(FlowState_t *state);

/** @brief 发布机体角速度；快速旋转时冻结光流位置累计。 */
void Flow_SetMotionContext(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_H */
