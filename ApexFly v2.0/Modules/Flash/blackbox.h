/**
 * @file blackbox.h
 * @brief 飞机端黑匣子：PID 双备份、解锁后场次日志、串口按场次导出 CSV。
 *
 * 只有本模块的任务碰 SPI3 / W25Q64。电机任务只 SubmitSample，电台任务只 RequestSave。
 * 不搬老 app_blackbox，不写水平零点。0x002000 给磁力计硬铁校准双备份，不再给六面校准。
 * 日志按 Session 切开，每场开头钉死旧 26 项姿态参数和新增高度参数。
 * 样本里 mag_heading_cdeg 记磁航向（百分之一度），CSV 列名为 mag_hdg。
 * 样本里 vz_target_cms 记目标垂速（厘米每秒），CSV 列名为 vz_target_cms；
 * 这一格原来是"本拍最高油门"（每拍重复的常数，无信息量），记录布局与偏移完全没变。
 * 样本仍用相对毫秒；解锁时刻的 UTC 写在高度参数记录里，上锁时刻的 UTC 写在场次尾。
 * 没收到合法 RMC 时这两处都写 0。导出时把 Unix 秒换成北京时间。
 * 日志区写满后从起点覆盖，写指针记在 0x003000；dump [N] 从当前写指针往回取最近 N 条，不是从 Flash 物理末尾取。
 * USART3：dump [N] 只出校验通过的新场次 CSV；dumpall [N] 原样打最近 N 条 64 字节十六进制。N=0 或 ALL 表示全部有效日志。
 */

#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一拍控制样本。电机任务在 100Hz 环里填，黑匣子任务写入 Flash。
 */
typedef struct
{
    /* 本场开始后的相对时间，单位毫秒。 */
    uint32_t t_ms;
    /* 机体系横滚，减过水平零点，单位百分之一度。 */
    int16_t roll_cdeg;
    /* 机体系俯仰，减过水平零点，单位百分之一度。 */
    int16_t pitch_cdeg;
    /* 机体系偏航，单位百分之一度。 */
    int16_t yaw_cdeg;
    /* 机体系横滚角速度，单位百分之一度每秒。 */
    int16_t roll_rate_cdps;
    /* 机体系俯仰角速度，单位百分之一度每秒。 */
    int16_t pitch_rate_cdps;
    /* 机体系偏航角速度，单位百分之一度每秒。 */
    int16_t yaw_rate_cdps;
    /* 横滚混控修正，单位微秒。 */
    int16_t roll_corr_us;
    /* 俯仰混控修正，单位微秒。 */
    int16_t pitch_corr_us;
    /* 偏航混控修正，单位微秒。 */
    int16_t yaw_corr_us;
    /* 基础油门，单位微秒。 */
    uint16_t throttle_us;
    /* 四路电机脉宽，单位微秒。 */
    uint16_t motor_us[4];
    /* 离地高度，单位厘米。当前由激光测距直接给出，含约 2cm 安装偏置。 */
    int16_t altitude_cm;
    /* 垂直速度，单位厘米每秒，向上为正。 */
    int16_t vz_cms;
    /* 磁航向，单位百分之一度，0～35999。无效填 -1。
       字段名和 CSV 列名 mag_hdg 保持一致。注意这一格不是"目标垂速"，
       target_vz_cms 那个旧名字是历史遗留，布局与偏移不变。 */
    int16_t mag_heading_cdeg;
    /* 垂速环对基准油门的修正，单位微秒。 */
    int16_t vz_correction_us;
    /* 目标垂直速度，单位厘米每秒，向上为正。
       这一格原来是"本拍使用的最高油门"（每拍重复的常数，没有信息量），
       定高接入后改记目标垂速，CSV 列名由 hover_us 改为 vz_target_cms。
       记录布局（偏移与宽度）完全不变；最高油门的值仍写在每场开头的参数行里。 */
    int16_t vz_target_cms;
    /* 高度控制模式：0 手动，1 定高，2 自动起飞，3 自动降落。 */
    uint8_t altitude_state;
    /* 0 上锁，1 解锁。 */
    uint8_t armed;
    /* 0 失联，1 链路通。 */
    uint8_t connected;
} BlackboxSample_t;

/**
 * @brief 黑匣子任务入口。初始化 W25Q64，上电加载 PID，之后处理保存、记录和 dump。
 * @param argument 未使用。
 */
void Blackbox_Task(void *argument);

/**
 * @brief Flash 已识别并完成上电加载后为 true。
 */
bool Blackbox_IsReady(void);

/**
 * @brief 当前正在记一场解锁日志。
 */
bool Blackbox_IsRecording(void);

/**
 * @brief 正在通过 USART3 导出 dump / dumpall。
 * @return true 表示姿态调试任务应暂停打印，避免插进 CSV。
 */
bool Blackbox_IsDumping(void);

/**
 * @brief 把当前 RAM PID 排队写入双备份槽。电台任务调用，本函数不擦 Flash。
 * @return 已入队返回 true；未就绪或上一次还在写返回 false。
 */
bool Blackbox_RequestSave(void);

/**
 * @brief 把当前磁力计硬铁偏移排队写入 0x002000 双备份。电台任务调用，本函数不擦 Flash。
 * @return 已入队返回 true；未就绪、正在记日志或上一次还在写返回 false。
 */
bool Blackbox_RequestMagCalibrationSave(void);

/**
 * @brief 把上电读出的磁力计硬铁偏移再灌进 RAM。磁力计任务芯片就绪后调用。
 * @return 有合法保存值并已应用返回 true。
 */
bool Blackbox_ApplySavedMagCalibration(void);

/**
 * @brief 从双备份槽读出合法表并灌进 RAM PID。上锁时由电台任务调用。
 * @return 至少一槽合法并已应用返回 true。
 */
bool Blackbox_LoadSavedParameters(void);

/**
 * @brief 电机任务在解锁边沿调用，开一场并写入带整张 PID 的场次头。
 */
void Blackbox_NotifyArmed(void);

/**
 * @brief 电机任务在上锁或失联边沿调用，结束当前场次。
 */
void Blackbox_NotifyDisarmed(void);

/**
 * @brief 把一拍样本丢进队列。队列满则丢弃并计数，不阻塞电机环。
 * @param sample 已经填好的控制样本。
 */
void Blackbox_SubmitSample(const BlackboxSample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
