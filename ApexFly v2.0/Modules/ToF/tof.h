/**
 * @file tof.h
 * @brief VL53L1X 激光测距任务对外接口。
 *
 * 任务层职责：定时读一次测距、低通滤波、处理失败（测不到时保持上一次读数）、发布快照、串口打印。
 * 不做定高，不参与任何控制。将来定高只调 ToF_GetSnapshot()，不碰驱动。
 *
 * ── 不做地面零点（已定案，不要再加回来）─────────────────────────
 * 输出的是"实测距离换算的厘米值"，不减任何地面零点，里面含着约 2cm 的安装偏置。
 * 曾用过"解锁采零点再相减"的方案，已废弃：零点必须绑定"飞机确实在地面"的时刻，
 * 一旦和控制环的解锁/上锁状态耦合，就会出现"上锁时零点被反复清掉、高度恒 0"的问题
 * （已实际发生过）。倾斜误差也不是零点能解决的，那要用 h = d·cosθ 单独补。
 */

#ifndef TOF_H
#define TOF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 激光测距快照。
 */
typedef struct
{
    /* 最近一次有效距离，单位毫米。从未测到过时为 0。 */
    uint16_t distance_mm;
    /* 实测距离换算成的厘米值（四舍五入），不做任何零点相减。
       里面含着约 2cm 的安装偏置：飞机停在地面上时读数约为 2cm 而不是 0。
       机身倾斜时该值会偏大（真实垂直高度 h = d·cosθ），倾斜补偿留到做定高时再加。 */
    int32_t altitude_cm;
    /* 上一次测量的测距状态码，0 表示有效。 */
    uint8_t range_status;
    /* 芯片是否已初始化成功。 */
    bool ready;
    /* 最近一次读数是否新鲜。false 表示当前用的是旧值（测不到）。 */
    bool fresh;
    /* 距上一次成功刷新过了多少毫秒。 */
    uint32_t age_ms;
    /* 累计通信错误次数。 */
    uint32_t error_count;
    /* 累计连续失败次数（用于判断传感器是否掉线）。 */
    uint32_t miss_streak;
} ToFSnapshot_t;

/**
 * @brief 激光测距任务入口。
 * @param argument 未使用。
 */
void ToF_Task(void *argument);

/**
 * @brief 复制最近一次快照。内部只拷贝一个结构体，不碰 I2C 总线，可在控制环里调用。
 * @param snapshot 输出。
 */
void ToF_GetSnapshot(ToFSnapshot_t *snapshot);

/**
 * @brief 只读实测高度，单位厘米。内部只做一次 32 位整数读取，不碰 I2C 总线。
 *
 * 专为 250Hz 控制环设计：控制环绝不能因为等 I2C2 锁而被阻塞，
 * 所以滤波和换算放在 ToF 任务里做，这里只取结果。
 */
int32_t ToF_GetAltitudeCm(void);

#ifdef __cplusplus
}
#endif

#endif
