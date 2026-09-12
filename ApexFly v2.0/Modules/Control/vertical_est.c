/**
 * @file vertical_est.c
 * @brief 高度/垂速状态估计（差分版）实现。
 *
 * ── 它在整条链路里的位置 ──────────────────────────────────────
 *   tofTask（激光，约 12Hz）
 *        │ distance_raw_mm + valid_count
 *        ▼
 *   VerticalEst_Update() ← 本文件，只做估计，不产生控制量
 *        │ height_cm / vz_cms / valid
 *        ▼
 *   将来由定高控制器消费（H9 起），本阶段只打印出来看
 *
 * ── 为什么吃原始值、不吃滤波值 ────────────────────────────────
 * tof.c 里的低通在 12Hz 下时间常数约 330ms。那个滞后对"屏幕上的数字"是好事，
 * 对估计器却是纯负担——差分求垂速最怕的就是滞后。所以这里用 distance_raw_mm。
 *
 * ── 为什么必须靠 valid_count 判断"是不是新数据" ────────────────
 * ToF 的 fresh 在两次测量之间一直是 true，光看它分不清"刚测的新数据"和
 * "83ms 前那一个"。若每拍都拿同一个数据去差分，会算出假的 0 速度
 * （分子恒为 0），而且还会把噪声一遍遍平均掉，看起来"很稳"其实是假的。
 *
 * ── 算法（差分 + 一阶低通）────────────────────────────────────
 *   来了新数据且可用：
 *       h_meas = 原始距离 / 10                        （毫米 → 厘米）
 *       vz_raw = (h_meas − h_est) / 距上次有效样本的秒数
 *       vz_est = vz_est + (vz_raw − vz_est) × ALPHA   （一阶低通）
 *       h_est  = h_meas                               （零阶保持，不再加滤波）
 *   读数不可用：h_est / vz_est 冻结不动，valid 置 false，不外推。
 *   判"不可用"用的是本模块自己累加的"距上次样本的时间"（超过 0.3 秒），
 *   不是 ToF 的 age_ms —— 单次丢数据不抖，连续丢三四次才判失效。
 *
 * ── 已知局限（都记在案，不要以为现在是最终形态）────────────────
 * 1. 差分噪声：原始值峰峰 3~5mm，差分出来约 2cm/s（1σ），低通后约 0.8cm/s。
 *    想再快就必须换"加速度计 + ToF 互补滤波"，那是 H16 的事，不是调参数能解决的。
 * 2. 倾斜没补偿：现在测到的是 d = h/cosθ，机身歪 20° 会偏大 6%（H4 加补偿）。
 * 3. 没有外推能力：数据一丢只能冻结；正确性靠 valid 标志告诉上层。
 * 4. 单上下文：本模块不加锁，H3 阶段只在 tofTask 里调用。
 *    将来搬进控制环时，Update 在控制环、GetState 在别处读，要先确认这一点。
 */

#include "vertical_est.h"

#include "tof.h"
#include <stddef.h>

/* 每拍 dt 的合理性区间，单位秒。
   正常一步约 20ms（tofTask 的步进周期）。超出这个范围说明任务被长时间抢占，
   累加值已经不可信，按标称值算，避免差分算出离谱的垂速。 */
#define VERTICAL_EST_DT_MIN_S 0.005f
#define VERTICAL_EST_DT_MAX_S 0.100f
#define VERTICAL_EST_DT_NOMINAL_S 0.020f

/* 差分时间分母的下限，单位秒。防止两条数据挨得太近时把噪声放大成巨大垂速。 */
#define VERTICAL_EST_MEAS_DT_MIN_S 0.020f

static VerticalEstState_t vertical_est;

/* 上一次见到的 ToF 版号。用来判断"这一拍有没有新数据"。 */
static uint32_t vertical_est_last_valid_count;
/* 是否已经用第一个有效测量值起步。起步前不出高度，避免从 0 往上爬。 */
static bool vertical_est_primed;
/* 距上次**有效**样本累积的时间，单位秒。作为差分的时间分母。 */
static float vertical_est_accum_dt_s;
/* 距上次用掉的样本过了多久，单位秒。超过 VERTICAL_EST_MAX_AGE_S 就判不可用。
   这是本模块自己累加的，不依赖 ToF 任务的 age_ms —— 两者不重复：
   任务万一被饿死，age_ms 会停在旧值不动，而这里的计时照样往前走。 */
static float vertical_est_since_sample_s;

/**
 * @brief 清空全部状态，等下一个有效样本重新起步。
 */
void VerticalEst_Reset(void)
{
    vertical_est.height_cm = 0.0f;
    vertical_est.vz_cms = 0.0f;
    vertical_est.valid = false;
    vertical_est.sample_count = 0U;
    /* 置 0 而不是读当前值：这样下一个样本一定会被当成"新数据"用上。 */
    vertical_est_last_valid_count = 0U;
    vertical_est_primed = false;
    vertical_est_accum_dt_s = 0.0f;
    vertical_est_since_sample_s = 0.0f;
}

void VerticalEst_Init(void)
{
    VerticalEst_Reset();
}

void VerticalEst_Update(float dt_s)
{
    ToFSnapshot_t snap;
    float h_meas_cm;
    float dt_meas_s;
    float vz_raw_cms;

    /* dt 兜底：离谱就按标称值算，宁可有点误差也不要算出垃圾。 */
    if ((dt_s < VERTICAL_EST_DT_MIN_S) || (dt_s > VERTICAL_EST_DT_MAX_S))
    {
        dt_s = VERTICAL_EST_DT_NOMINAL_S;
    }
    vertical_est_accum_dt_s += dt_s;
    vertical_est_since_sample_s += dt_s;

    /* 只拷贝一个结构体，不碰 I2C 总线，可以在任何任务里调用。 */
    ToF_GetSnapshot(&snap);

    /* valid_count 变了才是真的来了新数据。见文件头"为什么必须靠 valid_count"。
       注意：valid_count 只在"测到且结果有效"时才自增，所以它一变就说明这次读数有效，
       不需要再去判 fresh。 */
    if (snap.valid_count != vertical_est_last_valid_count)
    {
        vertical_est_last_valid_count = snap.valid_count;

        if (snap.ready && (snap.distance_raw_mm > 0U))
        {
            /* H4 的倾斜补偿就加在这一行：h_meas = 距离 × cos(roll) × cos(pitch) / 10。 */
            h_meas_cm = (float)snap.distance_raw_mm * 0.1f;

            if (!vertical_est_primed)
            {
                /* 第一个有效值直接当起点，垂速记 0，不从 0 厘米往上爬。 */
                vertical_est.height_cm = h_meas_cm;
                vertical_est.vz_cms = 0.0f;
                vertical_est_primed = true;
            }
            else
            {
                dt_meas_s = vertical_est_accum_dt_s;
                if (dt_meas_s < VERTICAL_EST_MEAS_DT_MIN_S)
                {
                    dt_meas_s = VERTICAL_EST_MEAS_DT_MIN_S;
                }

                vz_raw_cms = (h_meas_cm - vertical_est.height_cm) / dt_meas_s;
                vertical_est.vz_cms += (vz_raw_cms - vertical_est.vz_cms) * VERTICAL_EST_VZ_LPF_ALPHA;
                /* 高度不加滤波：原始值峰峰只有 3~5mm，对"稳住高度"够用，
                   再加一层只会白送滞后。真正的平滑留给 H16 的融合。 */
                vertical_est.height_cm = h_meas_cm;
            }

            vertical_est.sample_count++;
            /* 只有真正用掉的样本才清零两处计时：
               ① 差分分母 = 距上次**有效**测量的真实时间，中间丢几拍也不怕；
               ② 失效计时跟着复位，单次丢数据不会把 valid 抖成 false。 */
            vertical_est_accum_dt_s = 0.0f;
            vertical_est_since_sample_s = 0.0f;
        }
        /* 新数据但不可用（距离为 0 / 芯片没就绪）：什么都不动，
           让 since_sample 继续涨，涨过阈值自然判失效。 */
    }

    /* 冻结语义：一旦不可用，h/vz 保持最后可信值，valid 置 false，绝不外推。
       判据用"自己累加的、距上次样本的时间"，所以：
       - 单次丢数据（约 83ms）不会抖；
       - 连续丢三四次（约 330ms）才判失效；
       - ToF 任务被饿死、读数一直不刷新，同样会被判失效。 */
    vertical_est.valid = vertical_est_primed && snap.ready && (vertical_est_since_sample_s <= VERTICAL_EST_MAX_AGE_S);
}

void VerticalEst_GetState(VerticalEstState_t *state)
{
    if (state == NULL) return;
    *state = vertical_est;
}
