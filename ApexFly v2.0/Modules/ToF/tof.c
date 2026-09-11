/**
 * @file tof.c
 * @brief VL53L1X 激光测距任务实现（单次测量循环）。
 *
 * ── 为什么要用单次测量循环 ────────────────────────────────────
 * 2026-09-11 实测：本芯片的连续模式写入成功但不再产生新结果；
 * 单次模式每次都能正常完成。因此这里改成"触发一次 → 等完成 → 读结果 → 再触发"。
 *
 * ── 节奏 ──────────────────────────────────────────────────
 * 每 20ms 走一步，一次测量占 3 步（触发 / 等 2 拍 / 读），
 * 约合 16Hz 一轮完整测量。比 20ms 的时间预算宽裕得多，不会追不上。
 *
 * ── 失败处理 ───────────────────────────────────────────────
 * 测不到时保持上一次的读数，只把 fresh 置 false 并累加 age，
 * 上层能一眼看出"这个数是新的还是旧的"。
 * 连续多轮测不到才判定掉线并重新初始化。
 *
 * ── 总线约定 ───────────────────────────────────────────────
 * 本任务与 magtask 同级（都是 BelowNormal），每步都让出 CPU，
 * 避免长时间占着 I2C2 把磁力计饿死。
 *
 * ── 为什么不做"地面零点" ─────────────────────────────────────
 * 曾用"解锁时采一次地面读数当零点，高度 = 距离 − 零点"的方案，已废弃，原因：
 *   1. 镜头装在机身底部，机械偏置只有约 2cm，为一个常数引入整套状态机不值；
 *   2. 零点必须绑定"飞机确实在地面"的时刻，一旦和控制环的解锁/上锁状态耦合，
 *      就会出现"上锁时零点被反复清掉、高度恒 0"这类问题（已实际发生过）；
 *   3. 机身倾斜时真正要补的是 h = d·cosθ（θ 取 MPU6050 的横滚/俯仰），
 *      零点相减并不能解决倾斜误差，减的只是一个常数。
 * 因此这里直接输出实测距离换算的厘米值，保留约 2cm 的安装偏置。
 * 倾斜补偿留到将来做定高时再加，那时由使用方拿姿态角做 d·cosθ。
 */

#include "tof.h"

#include "cmsis_os.h"
#include "debug.h"
#include "i2c.h"
#include "task.h"
#include "vl53l1x.h"

/* 步进周期。 */
#define TOF_STEP_MS 20U
/* 触发后等几拍再读结果（2 拍 = 40ms > 20ms 时间预算，留足余量）。 */
#define TOF_WAIT_STEPS 2U
/* 串口打印周期，单位毫秒。250Hz 的调用方，绝不能每拍都打。 */
#define TOF_PRINT_PERIOD_MS 500U
/* 连续多少轮拿不到有效数据才重新初始化（约 1.6 秒）。 */
#define TOF_MISS_REINIT 10U

/* 距离低通滤波：filtered += (raw - filtered) / ALPHA_SHIFT。
   实测原始抖动约 7mm 峰峰；α=1/4 时噪声约降到 1/2，
   时间常数约 4 拍（80ms）—— 对高度这种慢变量完全够，手举高不会觉得迟滞。
   取 2 的幂是为了用移位代替除法，避免在嵌入式里做浮点或整数除法。 */
#define TOF_LPF_ALPHA_SHIFT 2U

/* 单次测量状态机的阶段。 */
typedef enum
{
    TOF_PHASE_TRIGGER = 0,   /* 触发一次测量 */
    TOF_PHASE_WAIT,          /* 等测量完成 */
    TOF_PHASE_READ           /* 读结果 */
} ToF_Phase_t;

static VL53L1X_Device_t tof_device;
static ToFSnapshot_t tof_snapshot;

/* 低通滤波后的距离，单位是 1/4 毫米（定点）。
   为什么不用整数毫米：filtered += (raw-filtered)>>2 这种整数写法有 ~3mm 死区，
   差值不足 4mm 时移位结果为 0，滤波值会卡住不动，实测的 7mm 抖动就压不下去。
   存 1/4 毫米后死区只剩 0.25mm，等于可以忽略。 */
static uint32_t tof_distance_filtered_q2;

/* 滤波器是否已经用第一个真实读数起步过。
   低通是从 0 开始往上爬的，如果不先"起步"，前 8 拍会明显偏低。
   只在 ToF 初始化（含掉线后重新初始化）时复位一次。 */
static bool tof_distance_filter_primed;

void ToF_GetSnapshot(ToFSnapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    *snapshot = tof_snapshot;
}

int32_t ToF_GetAltitudeCm(void)
{
    /* 只做一次 32 位整数读取，不碰 I2C 总线。 */
    return tof_snapshot.altitude_cm;
}

void ToF_Task(void *argument)
{
    VL53L1X_Result_t result;
    ToF_Phase_t phase;
    uint32_t last_fresh_tick;
    uint32_t next_print_tick;
    uint8_t wait_count;
    uint8_t miss_rounds;

    (void)argument;

    phase = TOF_PHASE_TRIGGER;
    wait_count = 0U;
    miss_rounds = 0U;
    last_fresh_tick = osKernelGetTickCount();
    next_print_tick = osKernelGetTickCount() + TOF_PRINT_PERIOD_MS;

    for (;;)
    {
        /* ── 初始化直到成功为止，每 1 秒重试一次。 ── */
        if (!tof_device.initialized)
        {
            if (!VL53L1X_Init(&tof_device, &hi2c2))
            {
                tof_snapshot.ready = false;
                Debug_Printf("[TOF] init fail: %s (errors=%lu)\r\n",
                             VL53L1X_InitStatusString(VL53L1X_GetLastInitStatus(&tof_device)),
                             (unsigned long)tof_device.error_count);
                osDelay(1000U);
                continue;
            }

            tof_snapshot.ready = true;
            tof_snapshot.fresh = false;
            tof_snapshot.miss_streak = 0U;
            /* 重新初始化后滤波器也重新起步，避免带着旧距离或从 0 往上爬。 */
            tof_distance_filtered_q2 = 0U;
            tof_distance_filter_primed = false;
            last_fresh_tick = osKernelGetTickCount();
            /* 初始化末尾已经触发了第一次测量，直接从"等待"开始。 */
            phase = TOF_PHASE_WAIT;
            wait_count = 0U;
            miss_rounds = 0U;
            Debug_Printf("[TOF] init ok: id=0x%04X range=short step=%ums\r\n",
                         (unsigned)VL53L1X_ReadModelId(&tof_device), (unsigned)TOF_STEP_MS);
        }

        /* ── 状态机走一步。 ── */
        switch (phase)
        {
        case TOF_PHASE_TRIGGER:
            if (!VL53L1X_StartSingleShot(&tof_device))
            {
                Debug_Printf("[TOF] trigger fail, re-init.\r\n");
                tof_device.initialized = false;
                break;
            }
            wait_count = 0U;
            phase = TOF_PHASE_WAIT;
            break;

        case TOF_PHASE_WAIT:
            wait_count++;
            if (wait_count >= TOF_WAIT_STEPS)
            {
                phase = TOF_PHASE_READ;
            }
            break;

        case TOF_PHASE_READ:
            if (VL53L1X_IsMeasurementDone(&tof_device) &&
                VL53L1X_ReadMeasurement(&tof_device, &result))
            {
                tof_snapshot.range_status = result.range_status;
                tof_snapshot.error_count = result.error_count;

                if (result.valid)
                {
                    /* 有效新数据：先过低通滤波，再刷新距离，清零失败计数。
                       差值要用有符号算，否则距离变小时无符号下溢，滤波值会直接跳到天上。 */
                    if (!tof_distance_filter_primed)
                    {
                        /* 第一个读数直接作为滤波初值，不让滤波器从 0 慢慢爬。 */
                        tof_distance_filtered_q2 = (uint32_t)result.distance_mm << 2;
                        tof_distance_filter_primed = true;
                    }
                    else
                    {
                        int32_t diff_q2 = ((int32_t)result.distance_mm << 2) - (int32_t)tof_distance_filtered_q2;
                        tof_distance_filtered_q2 = (uint32_t)((int32_t)tof_distance_filtered_q2
                                                    + (diff_q2 >> TOF_LPF_ALPHA_SHIFT));
                    }
                    /* 1/4 毫米还原成毫米，四舍五入。 */
                    tof_snapshot.distance_mm = (uint16_t)((tof_distance_filtered_q2 + 2U) >> 2);
                    /* 直接换算成厘米输出，不减任何零点：镜头装在机身底部，
                       实测值里就含着约 2cm 的安装偏置，当作固定读数保留。
                       毫米 → 厘米四舍五入，直接整除会把 19mm 砍成 1cm。
                       距离是物理量，不可能为负，所以不需要负值分支。 */
                    tof_snapshot.altitude_cm = (int32_t)(((uint32_t)tof_snapshot.distance_mm + 5U) / 10U);
                    tof_snapshot.fresh = true;
                    tof_snapshot.miss_streak = 0U;
                    last_fresh_tick = osKernelGetTickCount();
                    miss_rounds = 0U;
                }
                else
                {
                    /* 测到了但结果无效（超量程/反射太弱/太强）：保持上一次读数。 */
                    tof_snapshot.fresh = false;
                    tof_snapshot.miss_streak++;
                }
            }
            else
            {
                /* 这一轮没读到有效结果。 */
                tof_snapshot.fresh = false;
                tof_snapshot.miss_streak++;
            }

            miss_rounds++;
            if (miss_rounds >= TOF_MISS_REINIT)
            {
                Debug_Printf("[TOF] too many misses, re-init.\r\n");
                tof_device.initialized = false;
                break;
            }

            /* 立刻触发下一轮。 */
            phase = TOF_PHASE_TRIGGER;
            break;

        default:
            phase = TOF_PHASE_TRIGGER;
            break;
        }

        tof_snapshot.age_ms = (uint32_t)(osKernelGetTickCount() - last_fresh_tick);

        /* ── 串口打印，按时间节流。 ── */
        if ((int32_t)(osKernelGetTickCount() - next_print_tick) >= 0)
        {
            next_print_tick = osKernelGetTickCount() + TOF_PRINT_PERIOD_MS;
            Debug_Printf("[TOF] d=%umm alt=%ldcm status=%u fresh=%u age=%lums err=%lu\r\n",
                         (unsigned)tof_snapshot.distance_mm,
                         (long)tof_snapshot.altitude_cm,
                         (unsigned)tof_snapshot.range_status,
                         tof_snapshot.fresh ? 1U : 0U,
                         (unsigned long)tof_snapshot.age_ms,
                         (unsigned long)tof_snapshot.error_count);
        }

        /* 每步都让出 CPU：本任务与 magtask 共用 I2C2。 */
        osDelay(TOF_STEP_MS);
    }
}
