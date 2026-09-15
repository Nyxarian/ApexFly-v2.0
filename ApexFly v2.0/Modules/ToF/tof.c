/**
 * @file tof.c
 * @brief VL53L1X 激光测距任务实现（单次测量循环）。
 *
 * ── 为什么要用单次测量循环 ────────────────────────────────────
 * 2026-09-11 实测：本芯片的连续模式写入成功但不再产生新结果；
 * 单次模式每次都能正常完成。因此这里改成"触发一次 → 等完成 → 读结果 → 再触发"。
 *
 * ── 节奏（2026-09-11 实测更正）────────────────────────────────
 * 每 20ms 走一步。一次完整测量占【4 步】：
 *     第1步 触发 → 第2步 等1拍 → 第3步 等2拍(转READ) → 第4步 读结果
 * 一轮约 83ms，即【约 12Hz】（串口 n= 每 500ms 增加 6）。
 * ⚠️ 这里原来写的是"3 步 / 约 16Hz"，是数错了步数，已按实测更正。
 * 结果读到手的时刻是触发后约 62ms，而测量本身 20ms 就完成了，
 * 也就是数据到手时已经放了约 42ms。这个 42ms 才是控制通道的滞后
 * （低通那 330ms 只影响显示/遥测，因为估计器吃的是 raw 值）。
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
 *   1. 镜头固定在机架最下方，机械偏置只有约 8mm（放在平面上就是 8mm），为一个常数引入整套状态机不值；
 *   2. 零点必须绑定"飞机确实在地面"的时刻，一旦和控制环的解锁/上锁状态耦合，
 *      就会出现"上锁时零点被反复清掉、高度恒 0"这类问题（已实际发生过）；
 *   3. 机身倾斜时真正要补的是 h = d·cosθ（θ 取 MPU6050 的横滚/俯仰），
 *      零点相减并不能解决倾斜误差，减的只是一个常数。
 * 因此这里直接输出实测距离换算的厘米值，保留约 8mm 的安装偏置。
 * 倾斜补偿留到将来做定高时再加，那时由使用方拿姿态角做 d·cosθ。
 */

#include "tof.h"

#include "cmsis_os.h"
#include "debug.h"
#include "i2c.h"
#include "motor_test.h"
#include "task.h"
#include "vertical_control.h"
#include "vertical_est.h"
#include "vl53l1x.h"

/* 步进周期。 */
#define TOF_STEP_MS 20U
/* 触发后等几拍再读结果（2 拍 = 40ms > 20ms 时间预算，留足余量）。
   ⚠️ 曾经评估过把它改成 1 拍来提速（一轮 4 步 → 3 步，12Hz → 约 16.7Hz），
   结论是【不做】：控制通道只吃约 42ms 的读取延迟、不吃低通的 330ms，
   省下 20ms 在互补滤波的交叉频率上只相当于约 0.6° 相位；
   而采样变快会让"相邻两拍差分求垂速"的噪声变大 34%，对 H3 是反作用。
   详见 优化一下/高度接入飞控方案.md 第 13.6 节。 */
#define TOF_WAIT_STEPS 2U
/* 串口打印周期，单位毫秒。250Hz 的调用方，绝不能每拍都打。 */
#define TOF_PRINT_PERIOD_MS 500U
/* ⚠️【临时】串口回显测试期间静音周期打印（[TOF] / [ALT] 两行）。
   原因：回显测试要的是"发什么回什么"，串口上混着周期日志就分不清哪条是回显。
   初始化与错误那几条保留（只在启动/异常时出现，正好能看出板子起来了）。
   测完改回 1U 即可。详见 优化一下/串口回显测试（临时）.md。 */
#define TOF_PERIODIC_PRINT_ENABLE 0U
/* 连续多少轮拿不到有效数据才重新初始化（约 0.83 秒，12Hz × 10 轮）。 */
#define TOF_MISS_REINIT 10U

/* 距离低通滤波：filtered += (raw - filtered) / ALPHA_SHIFT。
   2026-09-11 实测（飞机静置）：原始值的峰峰值多在 3~5mm（个别到 8~10mm），
   滤波后峰峰值多在 0~1mm（最大 2mm），约 3~4 倍降噪。
   时间常数约 4 拍 = 4 / 12Hz ≈ 330ms。
   ⚠️ 这个滞后只影响显示/遥测；状态估计器吃的是 distance_raw_mm，不走这个滤波。
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
                    /* ⭐ 顺序要求，不要打乱：
                       ① 先把原始距离写进快照；
                       ② 再更新低通滤波器；
                       ③ 最后才自增 valid_count。
                       上层靠 valid_count 判断"有没有新数据"，按这个顺序写，
                       只要计数器变了，distance_raw_mm 必然已经是这一条新数据，
                       不会出现"计数器是新的、距离还是旧的"这种错配。 */
                    tof_snapshot.distance_raw_mm = result.distance_mm;

                    /* 差值要用有符号算，否则距离变小时无符号下溢，滤波值会直接跳到天上。 */
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
                    tof_snapshot.distance_filtered_mm = (uint16_t)((tof_distance_filtered_q2 + 2U) >> 2);

                    /* 高度仍然由【滤波值】换算，所以显示与遥测和 H2 之前完全一致。
                       直接换算成厘米，不减任何零点：镜头装在机身底部，
                       实测值里就含着约 8mm 的安装偏置（放在平面上读数约 1cm），当作固定读数保留。
                       毫米 → 厘米四舍五入，直接整除会把 19mm 砍成 1cm。
                       距离是物理量，不可能为负，所以不需要负值分支。 */
                    tof_snapshot.altitude_cm = (int32_t)(((uint32_t)tof_snapshot.distance_filtered_mm + 5U) / 10U);

                    /* 最后自增计数器，见本分支开头的顺序要求。 */
                    tof_snapshot.valid_count++;

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

        /* ── 高度/垂速估计器已搬进控制环（H4'）─────────────────────
           原因：倾斜补偿要用机体系姿态角，而姿态角只有控制环算得出来。
           本任务不再驱动估计器，只在这里**读**它的状态来打印
           ——控制环里禁止打印（一行日志约 10ms，会吃掉两拍 4ms 预算），
           所以"谁驱动"和"谁打印"分开：驱动在控制环，打印留在这里。 */

        /* ── 串口打印，按时间节流。⚠️ 回显测试期间被 TOF_PERIODIC_PRINT_ENABLE 关掉。 ── */
        if ((TOF_PERIODIC_PRINT_ENABLE != 0U) && ((int32_t)(osKernelGetTickCount() - next_print_tick) >= 0))
        {
            VerticalEstState_t vertical_state;
            VerticalControlSnapshot_t vertical_control_state;

            next_print_tick = osKernelGetTickCount() + TOF_PRINT_PERIOD_MS;
            VerticalEst_GetState(&vertical_state);
            VerticalControl_GetSnapshot(&vertical_control_state);
            Debug_Printf("[TOF] raw=%umm filt=%umm alt=%ldcm status=%u n=%lu fresh=%u age=%lums err=%lu | h=%.1fcm vz=%+.1fcm/s v=%u sc=%lu | loop=%luus\r\n",
                         (unsigned)tof_snapshot.distance_raw_mm,
                         (unsigned)tof_snapshot.distance_filtered_mm,
                         (long)tof_snapshot.altitude_cm,
                         (unsigned)tof_snapshot.range_status,
                         (unsigned long)tof_snapshot.valid_count,
                         tof_snapshot.fresh ? 1U : 0U,
                         (unsigned long)tof_snapshot.age_ms,
                         (unsigned long)tof_snapshot.error_count,
                         (double)vertical_state.height_cm,
                         (double)vertical_state.vz_cms,
                         vertical_state.valid ? 1U : 0U,
                         (unsigned long)vertical_state.sample_count,
                         (unsigned long)MotorTest_TakeMaxFrameUs());
            /* 定高的模式 / 门禁 / 停手（H6）。单独一行，不并进 [TOF]：
               两行加起来会顶到 384 字节的格式化缓冲区上限。
               req=遥控请求  mode=实际模式  st=细分状态
               sw/ct=两层门禁位图（全通分别是 0xF）  gnd=停手
               stick=油门杆  rate=杆给的目标变化率  h_sp=目标高度
               gt=停手进入计时（到 500 就进停手） */
            Debug_Printf("[ALT] req=%u mode=%u st=%u sw=0x%X ct=0x%X gnd=%u stick=%u rate=%+.1f h_sp=%.1f h=%.1f v=%u gt=%lums\r\n",
                         (unsigned)vertical_control_state.requested_mode,
                         (unsigned)vertical_control_state.mode,
                         (unsigned)vertical_control_state.state,
                         (unsigned)vertical_control_state.switch_gates,
                         (unsigned)vertical_control_state.control_gates,
                         vertical_control_state.grounded ? 1U : 0U,
                         (unsigned)vertical_control_state.stick,
                         (double)vertical_control_state.stick_rate_cms,
                         (double)vertical_control_state.h_sp_cm,
                         (double)vertical_control_state.h_est_cm,
                         vertical_control_state.est_valid ? 1U : 0U,
                         (unsigned long)vertical_control_state.ground_timer_ms);
        }

        /* 每步都让出 CPU：本任务与 magtask 共用 I2C2。 */
        osDelay(TOF_STEP_MS);
    }
}
