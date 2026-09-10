# ApexFly V2.0 悬停稳定性分析与定高/定点/返航实施方案

> 分析对象：`F:\Zhuomian\ApexFly v2.0\ApexFly v2.0\`（STM32G474VCT6 四轴飞控固件，Keil MDK + CubeMX + FreeRTOS）
> 分析性质：**纯只读**，未修改任何源码/工程配置/链接脚本；本文档是本任务唯一写盘产物。
> 分析范围：仅 V2.0 工程及其配套数据；**不参考、不引用 ApexFly v3.0 / 遥控器工程**（工作区其余目录一律未纳入结论）。
> 参考数据：`BlackboxReader/exports/keep_v2/` 下 V2.0 自身黑匣子导出（session_001/016/164/244/361）与 `INDEX.md` 场次说明；`README.md`（V2.0 工程自己的说明）。
> 日期：本文基于当前工作区源码状态（FreeRTOS 10.3.1 / CMSIS-RTOS2 / HAL，协议版本 0x07）。

---

## 0. 一页纸结论（先结论，后依据）

1. **当前代码默认出厂状态 = 6 组姿态 PID 增益全部为 0**（`attitude_control.c` 的 `kDefaultPidParams`），解锁上电若不通过遥控 SET/LOAD_SAVED 灌参数，**姿态环完全不纠偏**，四电机只跟随共同油门——此时起飞即倾倒，必然炸机。
2. **黑匣子历史证据**（`INDEX.md` + 5 个场次 pid.csv）：V2.0 此前所有实测场次的内环（角速度环）增益**全部为 0**，横滚/俯仰只开过角度外环 Kp=0.3～1.0、无 Ki/Kd，偏航全 0。也就是说**实机从未在“带角速度阻尼”的状态下飞过**。
3. **"起飞震荡 + 单向漂移"在现有参数下的机理是确定的**：P-only 角度环（无阻尼内环、无积分、无偏航稳定）刚度低、阻尼为零，任何阵风/起转扰动都只能靠比例回正，被吹出角度后无速率阻尼吸收能量 → 姿态慢发散（震荡）、水平分力持续存在（单向漂移）。5～6 级大风只是把这个欠阻尼系统推到发散阈值以上。
4. 代码链路里另有 3 个【软件可改】的高风险点，必须先清除再谈调参：
   - **A-1（环路被串口打印阻塞）**：`motor_test.c` 主控循环 100Hz 内**每拍无条件 `Debug_Printf()` 打一行 IMU 文本**（约 65–85 字符 @115200，单拍阻塞约 6ms、串口占用 61–74%，详见 7.4.1），控制节拍被拖到 ~60Hz 且每拍抖动；这是姿态环相位裕度的大敌。
   - **A-2（参数无掉电记忆路径）**：上电默认全 0，`PidTable_Init()` 不自动 `LOAD_SAVED`，必须遥控手动 LOAD——一旦忘了灌参数，等于无自稳飞行。
   - **A-3（姿态环启用门槛与油门直通）**：解锁后"油门杆 ≤1050us 时姿态修正强制归零、四路等值"，起飞瞬间油门穿越 1050us 附近时姿态环从 0 突然切入（`AttitudeControl_Update` 的积分/微分历史是干净的，但修正量从 0 突变到全值），叠加电机从怠速到起转的非线性，易触发瞬时过冲。
5. **定高/定点/返航现状**：定高控制算法存在（`altitude_control.c`）但**已脱机**（气压计任务停用、高度/垂速硬编码 0、油门直通）；GPS 只解析数据不进控制；磁力计只供偏航外环参考与记录。README 亦明确"没有光流/测距之前不要做 XY 定点"——因此 Phase C/D 需要先补齐传感器与框架（详见第 6 章方案）。
6. 结论排序（详见第 5 章根因分析）：本次炸机最可能的直接原因是**实际飞行参数下系统欠阻尼 + 无航向/无阻尼结构 + 大风放大**，即"参数/结构"问题，而不是 IMU 方向、混控符号、协议丢帧这类"功能性问题"——后者的符号与映射在代码注释与黑匣子数据里互相自洽，且多次台架实测过。但"炸机当场实际 RAM 表是什么"必须用当场的黑匣子场次头回查确认，这是下一步第一动作。

---

## 1. 工程/芯片/工具链/链接与内存

（构建配置细节见第 7 章，本节给结论）

- 芯片：STM32G474VCT6（Cortex-M4F，170MHz，Flash 256KB，RAM 128KB；Keil 工程 Device 名 STM32G474VCTx）。
- 工具链：Keil MDK-ARM（uvprojx），ARMCC5（AC5），优化与宏见 7.2.4。CubeMX 6.18 生成 HAL + FreeRTOS CMSIS-RTOS2。
- HAL 时基：TIM6（见 `stm32g4xx_hal_timebase_tim.c`），FreeRTOS tick 1ms（`configTICK_RATE_HZ 1000`），`configTOTAL_HEAP_SIZE 30720`（30KB heap_4），`configCHECK_FOR_STACK_OVERFLOW 2`。
- 链接：`MDK-ARM/stm32g474xx_flash.sct`，FLASH 0x08000000 起、RAM 0x20000000 起（布局/占用见 7.2.4）。
- 外设总览：
  - I2C1：MPU6050（0x68）；I2C2：磁力计 GY-273（HMC5883L 0x1E / QMC 0x0D 探测），SPL06 气压计（0x76，已停用，可拔）。
  - SPI1：RA-01SC LoRa（SX126x）；SPI3：W25Q64 黑匣子；USART2：GPS ATGM336H（9600，DMA RX）；USART3：调试口 115200。
  - TIM8 CH1-4：四路电调 PWM（50Hz、1MHz 计数、1000–2000us），引脚 PC6-PC9。
  - ADC1 IN1(PA0)：电池分压。
- FreeRTOS 任务（app_freertos.c，优先级从高到低：radioTask AboveNormal > default/imuTest/motorTest/led Normal > blackbox/attitudeDebug/baro(停用)/gps/mag BelowNormal）：

| 任务 | 优先级 | 栈 | 实际内容 |
|---|---|---|---|
| radioTask | AboveNormal | 512×4 | RA-01SC 收发、control_link、PID 命令/应答 |
| motorTestTask | Normal | 640×4 | **主控任务**：IMU 读、零偏/水平采集、姿态解算、姿态 PID、混控、写 TIM8 |
| imuTestTask | Normal | 512×4 | 空转（避免抢 I2C1） |
| ledTask | Normal | 128×4 | 心跳灯 |
| blackboxTask | BelowNormal | 768×4 | W25Q64 记录/参数保存/dump |
| attitudeDebugTask | BelowNormal | 384×4 | USART3 打印 [MAG]（10Hz） |
| gpsTask | BelowNormal | 512×4 | USART2 NMEA 解析 |
| magTask | BelowNormal | 512×4 | I2C2 磁力计 ~50Hz |
| baroTask | BelowNormal | 768×4 | **未创建**（app_freertos.c L221-222） |

- 时间基准与中断：EXTI2(DIO1) 通知 radioTask；EXTI0(IMU_INT) 已配引脚但**没有任何任务消费**（轮询读 I2C1 代替中断驱动采样）；HAL 时基 TIM6 中断；各中断优先级、FreeRTOS 从 ISR 调用合规性见 7.2.2（已核实全部合规）。

### 1.1 可疑点

- [中] IMU 采样是"任务内阻塞轮询 + `osDelay(10)` 粗定时"，真实节拍（黑匣子证据）主峰 11ms、含 1/2/19ms 抖动 → 姿态解算 dt 不稳，见 A-1 与第 5 章 R5。
- [中] `baroTask` 未创建但 `app_freertos.c` 中仍保留创建代码与 StartBaroTask 空转实现；I2C2 引脚 PA8/PA9 与注释一致（gpio.c 亲核），当前仅磁力计一个活动用户，恢复 baro 后由 I2C2_Lock 事务级互斥保证不冲突（恢复注意点见 7.1.2）。
- [低] `imuTestTask`/`defaultTask` 长期空转占栈，属于开发残留，功能上无影响。

---

## 2. 传感器链

### 2.1 MPU6050（I2C1，0x68）—— `Modules/IMU/mpu6050.c`

- 配置：唤醒并选 X 陀螺 PLL；DLPF_CFG=3（陀螺 ~42Hz / 加计 ~44Hz 带宽）；SMPLRT_DIV=0（内部 1kHz 采样）；陀螺 ±2000dps（16.4 LSB/dps）；加计 ±8g（4096 LSB/g）；INT 高电平脉冲 + Data Ready 中断已使能（但**上层未用中断**）。
- 读取：`MPU6050_ReadData()` 阻塞 `HAL_I2C_Mem_Read` 一次 14 字节，超时 20ms；出错计数暴露给上层；单次失败会导致 motor_test 整轮"断电保护 → 重新初始化 IMU 并重采零偏"（`motor_test.c` L207-214），代价很大。
- 单位换算、符号：accel g、gyro dps 直接按量程换算，**未做任何符号翻转**（翻转全部放在上层映射）。

### 2.2 陀螺零偏与水平零点—— `imu_bias.c` / `imu_level.c`

- 零偏：上电后（电调先 1000us×3s）静置采 200 拍平均值，作为 gyro 零偏；只减陀螺，不减加计。
- 水平零点：解算启动后再静置采 200 拍机体系 roll/pitch 平均，之后每拍从测量角减去（仅 RAM，不保存 Flash）。
- 【疑点】水平零点把"静止时的姿态解算读数"直接当作"零"，它同时吸收了：(a) 安装倾角（想要）；(b) 姿态解算在静置时若使用加速度校正后的稳态误差（例如 Mahony 收敛偏差）；(c) 若采集时手抖/风晃则零点带噪声。零点误差会直接变成控制目标的常值偏置——单向漂移的一个软件侧来源（R4）。
- 【疑点】采集 200 拍期间如果 IMU 被打断重来（I2C 错误 → 重新初始化）或飞机被风吹动，零点就脏了且**无有效性检验**（没有"方差过大拒绝"逻辑）。

### 2.3 姿态解算（Mahony，芯片系四元数）—— `imu_attitude.c`

- 输入：减零偏后 gyro(dps)、accel(g)；dt 来自 `osKernelGetTickCount()` 差，**限幅 [5ms,30ms]**，越界返回 false 并丢拍。
- Mahony：Kp=0.8（rad/s 反馈），Ki 按"每样本 0.0003@100Hz"换算成 0.030/s；积分限幅 ±0.2 rad/s；加速度模长门限 0.8–1.2g 之外**只用陀螺预测并冻结积分**（大机动/振动时丢重力修正）。
- 初始四元数：首拍用静止加速度算 roll/pitch 置初值（yaw=0）。
- 机体系映射（芯片系→机体系，README/头文件与代码一致，属"实测过、勿改"约定）：
  - roll_deg = 芯片 pitch；pitch_deg = 芯片 roll；yaw_deg = −芯片 yaw
  - roll_rate = gyro_y；pitch_rate = gyro_x；yaw_rate = −gyro_z
  - 角度语义：右倾 +roll、抬头 +pitch、顺时针 +yaw。
- 【疑点】Mahony 的误差叉乘使用**归一化加计**与重力向量——当加计噪声大（电机振动、6 级风+机架共振）且 44Hz 加计带宽时，重力修正项会在 0.8–1.2g 门限边缘反复通断，角度输出可能带噪声方波（fusion_error 突跳）。加计未低通、DLPF 44Hz 直接进 Mahony，无额外软件低通。
- 【疑点】dt 若实测为 11ms 而代码按 10ms 标称调 Ki/Kp 的"每秒换算"，无碍（代码按真实 dt 积分），但 dt 抖动大时一阶欧拉积分误差累积在 yaw（无磁修正）上。

### 2.4 磁力计（I2C2）—— 运行中，只服务偏航外环

- GY-273 多芯片判型（HMC5883L 0x1E → QMC5883L 0x0D → QMC5883P/6308 0x2C），~50Hz 轮询 + 一阶低通（~2.5Hz）；机体系映射 mx=−x、my=y、mz=z（实测约定）。
- 航向 `atan2(−my, mx)` 顺时针为正，**无倾角补偿**（倾斜即失真，设计边界）；硬铁校准（≥100 样本、每轴跨度 ≥0.25Ga）存 RAM + Flash 0x002000（双槽同扇区，掉电互毁隐患见 7.4）。
- 消费：偏航外环目标（杆回中锁航向）、遥测槽（`home_distance_m` 字段名复用为航向，协议层历史遗留）。
- 软件疑点：HMC5883L CRB=0x20 与灵敏度 820 counts/Ga 档位不自洽（读数可能虚高 33%，只影响偏航，调 yaw 前须定案）。详见 7.1.3。

### 2.5 气压计 / 高度估计—— 确认停用（Phase B 恢复对象）

- SPL06(0x76/0x77, I2C2) 驱动、baro 任务（20ms 轮询、地面压力裁剪平均、相对高度公式）、altitude_est（三状态线性卡尔曼 [z,vz,ba]，R=0.09m²、创新 ±2m 门限、1s 超时判 unhealthy）**全部编入固件但零调用**：baroTask 未创建、StartBaroTask 是空转、altitude_est.h 无人 include。
- 恢复时两处必须同改（app_freertos.c:222 恢复 osThreadNew；StartBaroTask 接 Baro_Task 入口）；启用前需修"SPL06 无新样本被误计通信失败 → 每数秒整机软复位"的 BUG（B5）。详见 7.1.2。

### 2.6 GPS—— 数据全解析、不进控制（Phase C 坐标源现成）

- ATGM336H @USART2 9600 DMA 空闲中断 + 1024B 环形缓冲；解析 GGA/RMC：**经纬度(e7)、地速(cm/s)、航向(cdeg)、HDOP、星数、海拔、UTC Unix 秒全齐**；校验/长度防护到位。
- 消费者现状：只取 UTC 时间（黑匣子）与星数/地速/fix 位（遥测）；**经纬度无人消费**——Phase C-lite 的坐标输入零成本可得。
- 坑：GGA quality=0 / RMC 状态 V 会把有效坐标覆写成 0（未来消费者必须 fix_valid 门控，B9）。详见 7.1.4。

---

## 3. 姿态与控制链（核心精读结论）

### 3.1 主控任务时序（`motor_test.c` 的 `MotorTest_Task`）

上电流程：
1. `PidTable_Init()`（灌默认全 0 参数）+ `Battery_Init()`；`Motor_Init()` 失败则死循环。
2. 四路 1000us 保持 3s → 电调解锁。
3. 循环：`MPU6050_ReadData()` 阻塞读 → **每拍 `Debug_Printf` IMU 行（危险，见 A-1）** → 减陀螺零偏 → `IMU_Attitude_Update` → `MapToBody` → 采水平零点（200 拍）→ 之后每拍：
   - `altitude_cm=0; vz_cms=0`（气压停用硬编码）；
   - 1s 一次 `Battery_Update`；
   - `ControlLink_Update` + `GetSnapshot`；
   - `armed = connected && command_valid && FLAG_ARM_REQUESTED`；未解锁 → 四路 1000us、`AttitudeControl_Reset`；
   - 解锁：油门杆线性映射 1000→`PidTable_GetMaxThrottleUs()`（默认 1500）；**throttle≤1050us 时姿态修正强制为 0、四路等值**；>1050us 才 `AttitudeControl_Update()` 出混控 → `MotorTest_ApplyMixer`（每路再强制 ≥1050us）→ 黑匣子提交样本 → `Radio_PublishFlightState` → `osDelay(10)`。
4. 任何 I2C 错误 → 上锁保护 → 整个重来（重新初始化 + 重采零偏 + 重采水平零点）。

### 3.2 串级 PID（`attitude_control.c` + `cascade_pid.c` + `pid.c`）

- 每轴 = 外环（角度→目标角速度，输出单位名义 deg/s）+ 内环（角速度→混控修正 us）。
- **内环未启用判据**：内环 Kp/Ki/Kd 全 0 → 外环输出直接当修正量用（兼容旧的"角度开环测试通路"）。**这就是历史场次实际使用的模式**（外环 P 直通）。
- PID 公式：`out = Kp·e + Ki·∫e·dt + Kd·Δe/dt`，积分项在输出限幅外另有 i_limit；`dt∉[0.5ms,100ms]` 输出 0（丢拍保护）；首拍微分=0。
- 横滚/俯仰外环目标角：杆量 −1000..1000 中带 ±80 死区，映射满杆 ±10°；再加 Trim（默认 0）。
- 符号（已实测约定，勿改）：`roll_corr = −Kp·roll`、`pitch_corr = +Kp·pitch`，代码里 pitch 串级输出乘 `mixer_sign = −1` 实现（attitude_control.c L76 注释、L225）。
- 偏航：杆出死区(±80) → 外环让路、内环跟杆速率（rate_limit = 外环 out_max，默认 200）；杆回中 → 若磁航向有效且 |roll|,|pitch|<25° → 锁当前磁航向为 yaw_hold，外环吃"最短角误差"；磁不可用/倾角大 → 冻外环只留内环阻尼。**yaw 外环增益默认 0**。
- 黑匣子交叉验证（session_244/361，kp=1.0）：`roll_corr/(−roll)` 中位 ≈ 0.995、`pitch_corr/pitch` ≈ 0.97 → 角度→修正的符号与比例在记录层面**自洽正确**（排除符号反接为本次炸机主因，R1 降权）。

### 3.3 混控（`mixer.c`）

- X 机架；M1 后右 CW、M2 前右 CCW、M3 后左 CCW、M4 前左 CW（机头朝上俯视）。
- `M1 = −R + P − Y；M2 = −R − P + Y；M3 = +R + P + Y；M4 = +R − P − Y`（+油门）。
- 饱和保护：先按四路中"会最先撞限"的那一路统一缩放修正量（correction_scale），保证不出现单路先饱和把力矩打歪；输出限幅 1000–2000us。
- 手测现象（mixer.h L19-35）：右倾 → M1/M2 快 M3/M4 慢；前倾低头 → M2/M4 快；均与公式一致（若姿态映射正确）。

### 3.4 油门

- 杆 0–1000 线性映射：`1000 + stick·(max−1000)/1000`，max 默认 1500us（`APEX_PID_HOVER_PWM_US` 语义已被改为"满杆上限"）。
- 无垂速环、无起飞斜坡（旧 `altitude_control.c` 的 TAKEOFF_RAMP/AIRBORNE 状态机已脱机）；地面起飞全凭手推杆。
- 电调：50Hz PWM、1000us 锁、1050us 为"怠速门槛"（MotorTest_ApplyMixer 每路最低 1050us——注意这意味着"姿态修正最小值"可能把某电机抬到刚起转的区间）。

### 3.5 高度控制现状（`altitude_control.c`）

- 完整保留一套"垂速 PID + 起飞斜坡 + 离地确认 + 估计失效冻结油门"的状态机，`hover_pwm_us` 现被复用为"满杆上限"。
- 当前**无人调用**（motor_test 直通油门），`AltitudeControl_Update` 形同虚设；垂速 PID 参数 ID（APEX_PID_VERTICAL_SPEED_KP/KI/KD）仍可被遥控读写，但改的是脱机对象。
- 现状 = "定高只有骨架，闭环完全未启用"，与 README 一致。

---

## 4. 通信与协议

（本协议与遥控端为同一份 0x07 定义：A5 5A 头 + 小端 + CRC16-CCITT-FALSE）

- 帧：`A5 5A | ver(0x07) | type | seq(2) | len | payload | crc16(2)`，总长 = 9+len。CRC 覆盖 ver..payload。
- 下行（遥控→飞）：`0x01 CONTROL` payload=10B（throttle u16、roll/pitch/yaw i16、flags、flight_mode）→ 帧 19B；`0x20 PID_COMMAND` payload=6B；`0x10 PING`。
- 上行（飞→遥控）：`0x81 TELEMETRY` payload=31B（姿态 3×i16、高度 cm、垂速、地速、电池 mv、home_distance(现为磁航向 0–359/0xFFFF)、RSSI、SNR、星数、链路质量、flags、模式、油门 PWM、GPS UTC 秒）；`0xA0 PID_RESPONSE` payload=124B（30 参数 × q10000）→ 帧 133B；`0x90 PONG`。
- 收包流程（radio.c）：DIO1 中断 → radioTask 被唤醒 → `RA01SC_GetAndClearIrqStatus`（CRC/头错误 → 计数并重收）→ RX_DONE → 读包 → 协议解码（长度必须精确相等、CRC 过）→ CONTROL → `ControlLink_AcceptCommand`（杆量程检查 + 16bit 序号"新于"判定，防乱序/重放；失联/首包则任意序号可建链）→ **立刻回 TELEMETRY**；TX_DONE 后才重新 `StartReceive`。
- 失联：`CONTROL_LINK_TIMEOUT_MS=800ms`（实测一问一答约 300–320ms，放宽余量）；失联 → connected=false、油门 0、强制上锁。
- PID 命令：SET（解锁中拒绝）、SAVE（排队给 blackbox 写 Flash 双备份）、LOAD_SAVED（从 Flash 读回）、LOAD_DEFAULTS、MAG_CAL_START/STOP/CANCEL；SET 立刻 `AttitudeControl_ApplyParams`。
- 关于"8 字节固定帧"提示：V2.0 这里**不是**固定 8 字节帧（是 9+len 的可变帧，最短 PING 也要 9B 头尾），没有拆包/粘包问题——LoRa 每次收发都是完整单包。真正相关的是**一问一答半双工时序**：飞机回遥测期间（TX 空中时间 + 处理，SF7/BW125 下 19B 帧约 60–70ms，PID 响应 133B 帧约 150–250ms，空中时间估算见 7.3）无法接收新控制帧；若遥控按固定 100ms 节奏连发而不等应答，丢包率会高（这需要与遥控端实际节奏核对——仅顺带说明，遥控端不在本次分析范围内）。序号过滤只防乱序不重传——链路丢帧 = 杆量"旧值保持"，控制上是安全的（失联才会清零）。

---

## 5. 根因分析：起飞震荡 + 单向漂移

### 5.1 候因清单（按当前证据可能性从高到低排序）

| # | 候因 | 可能性 | 性质 | 证据/推理 |
|---|---|---|---|---|
| R1 | **实际增益配置 = 纯 P 角度环、无阻尼内环、无偏航控制 → 系统欠阻尼/低刚度，大风下姿态发散** | ★★★★★ | 混合：软件配置（可改）+ 需飞行复验 | 黑匣子 5 场次 pid 表：内环全 0、外环 Kp 0.3/1.0、无 Ki；attitude_control 默认全 0；INDEX.md 明确"没有一场带着 2.30/1.50 起飞"。P-only 无 D 无阻尼 = 姿态"弹性摆"，被扰动后靠比例缓慢回正且不回零（无积分），风持续吹 → 常值倾角 → 单向漂移；扰动大于恢复 → 发散震荡。 |
| R2 | **上电默认参数全 0 且无自动加载，飞行当天若未 LOAD_SAVED/SET 则完全无自稳** | ★★★★☆ | 软件（可改）/ 需回查当天黑匣子 | kDefaultPidParams 全 0；PidTable_Init 只灌默认；LOAD_SAVED 仅遥控手动触发。若炸机当场参数就是 0/缺参数 = 直接倾倒，无需风。**回查当天场次头即可定论。** |
| R3 | **炸机前有人改了参数但未 SAVE（RAM 与 Flash 不一致），重启后回到旧表** | ★★★☆☆ | 使用流程问题 | INDEX.md 已记录"你记得的最终 PID 与场次头对不上"的同类现象；SET 只改 RAM，SAVE 才写 Flash。 |
| R4 | **水平零点/姿态零偏在起转振动下被污染或静态零偏未除尽 → 常值假姿态 → 单向修正/漂移** | ★★★☆☆ | 软件可查（黑匣子可验） | imu_level 只减平均、无拒绝逻辑；电调起转后机架振动 → 加计门限 0.8–1.2g 内 Mahony 修正含振动泄漏；加计未滤波、DLPF 44Hz。若零点采集含安装面不水平误差=每度误差带来常值漂移速度。可在黑匣子样本看解锁初期 roll/pitch 均值是否离 0。 |
| R5 | **控制节拍抖动/打印阻塞（A-1）导致相位裕度不足，正常增益也震荡** | ★★★☆☆ | 软件（可改，立即修） | 主循环每拍 8ms 阻塞打印；历史 dt 主峰 11ms + 19ms 尖峰；100Hz 名义控制实际 50–90Hz 且非均匀 → 等效相位延迟加大。 |
| R6 | **链路：遥控包率低（实测 300ms 级一问一答）+ 半双工丢帧窗口 + 无重传** | ★★☆☆☆ | 需与遥控端核对节奏 | 手动模式打杆修正延迟 300ms 级，手感滞后易过量打杆诱发人为震荡；但姿态内环不依赖链路，故非主因。 |
| R7 | **IMU 方向/混控符号/电机方向错误** | ★☆☆☆☆（已排除为主因） | 需台架复核（低成本） | 代码注释（mixer.h/imu_attitude.h/attitude_control.h）与黑匣子数据自洽：kp=1.0 场次修正比例≈1.0 符号正确；README"实机联动已确认"。但仍建议拆桨手测复核一遍（见 7.5 清单 V2）以防"记录正确但实际机架接线不同"。 |
| R8 | **四电机/电调油门一致性、桨不平衡、重心偏移、机架刚性** | ★★☆☆☆ | 需台架/硬件验证 | 代码无能为力；电机 PWM 指令在记录里对称（均值偏差<10us），但实际转速/拉力一致性、桨动平衡、重心未知。 |
| R9 | **混控/油门结构缺陷：修正下限强制 1050us 把停转区电机推入起转非线性** | ★★☆☆☆ | 软件（可改，方案 B 处理） | MotorTest_ApplyMixer 每路 min 1050us：低油门大修正时可能让本应减速的电机进入"起转临界"导致反效果；无刷低速非线性区（油门<起转点）使力矩不可控。 |
| R10 | **缺位置/速度闭环，风中必然漂移** | ★★★★☆（方向性，非本次炸机直接原因） | 结构（Phase C/D 目标） | 现系统只控姿态不控水平速度/位置；风 5–6 级下即使姿态完美保持水平，机体会整体顺风平移（相当于常值气流攻角），需要速度/位置环或至少 GPS 地速阻尼才能稳住。 |

### 5.2 已确认/已排除的软件事实汇总（对照 7.5 验证清单使用）

- ✅ 姿态→机体系映射符号、混控公式、修正极性：黑匣子数据 + 注释自洽。
- ✅ 高度/垂速：确认全部硬编码 0、闭环脱机。
- ✅ GPS：确认不参与控制（RMC/GGA 字段已全解析，见 7.1.4）。
- ✅ 通信：帧长/序号/超时逻辑正确，无拆包粘包；半双工时序窗口待核对遥控节奏。
- ⚠ 未确认（需黑匣子/现场回查）：炸机当场实际 PID 表、实际控制节拍、解锁瞬间四路 PWM、Yaw 是否自旋、加计门限通断次数。
- 🐞 明确代码级问题（不改，只列出，修复方向见第 5.3）：
  1. motor_test 主循环每拍无条件阻塞 Debug_Printf（A-1）。
  2. PidTable_Init 不自动从 Flash 加载已保存参数（A-2）。
  3. 解锁后 I2C 单次错误 → 全链路重置 + 重采零偏/零点，飞行中极危险（motor_test.c L207-214：`was_armed` 时直接 `Motor_Disarm` 断桨保护? 复核：错误时是 disarm + break → 重采。飞行中断桨=炸机。虽属"安全设计"，但对振动引起的 I2C 偶发错误过于激进）。——需读 ra01sc/blackbox 无此问题。
  4. Debug_Vofa 系列 = 空函数返回 OK（调试期占位，不是 BUG）。
  5. `altitude_est`、`AltitudeControl_Update` 死代码 + 占 ID（无功能影响）。
  6. baroTask 未创建，但 spl06 驱动与 altitude_est 保留——Phase B 需接回 I2C2（与磁力计共用总线，需互斥正确）。

### 5.3 已发现 BUG 的修复方向（本任务不动手，仅给方向）

| # | 位置 | 问题 | 修复方向 |
|---|---|---|---|
| B1 | `motor_test.c` 主循环 | 100Hz 环内每拍阻塞 `Debug_Printf`（~8ms@115200）→ 控制节拍与相位裕度被毁 | 删除或 `armed==0` 才 10Hz 打印；解锁后只走黑匣子 |
| B2 | `pid_table.c` / `blackbox.c` | 上电默认全 0 且不自动加载 Flash 保存表 | `Blackbox_Task` 初始化成功后若表为默认值则自动 `LoadSavedParameters`（保留遥控手动 LOAD） |
| B3 | `motor_test.c` L207-214 | 单次 I2C 读错误 → 飞行中整机复位并重采零偏/零点（断桨级） | 改为"连续 N 次失败才保护；单次失败只丢拍计数"，解锁中禁重采 |
| B4 | `hmc5883l.c` L130/35 | CRB=0x20（±1.3Ga 档）与灵敏度 820 counts/Ga（±1.9Ga 档）不自洽，磁场读数可能虚高 33% | 对照手册定案：CRB→0x40 或系数→1090（二选一） |
| B5 | `spl06.c`+`baro.c`（启用后必现） | 8Hz 数据 vs 20ms 轮询，判重返回 false 被当通信失败累计 → 每数秒整机软复位重采地面压力 | baro.c 区分"暂无新样本"与"I2C 失败"两种返回 |
| B6 | `app_freertos.c` | Baro 恢复涉及两处（L222 不创建 + StartBaroTask 空转不调 Baro_Task），只改一处会静默失败 | 恢复时两处同改，先给 StartBaroTask 接真入口 |
| B7 | `attitude_debug.c` | [ATT] 姿态/[GPS] 观察量被 `#if 0` 关闭 | 需要时解除（先确认与 dump 互斥逻辑已生效） |
| B8 | `altitude_est`/`AltitudeControl_Update` | 死代码但占协议 ID；PidTable 仍可写垂速参数（写的是脱机对象） | Phase B 接线前保持现状，注明"改参不生效"避免误导；或临时在 SET 应答中区分模块离线 |
| B9 | `gps_atgm336h.c` GGA/RMC | 无 fix 时把有效坐标/地速覆写成 0 | Phase C 消费者一律 fix_valid 门控（当前无实害） |
| B10 | 混控下限语义 | `MotorTest_ApplyMixer` 每路强制 ≥1050us + "≤1050 旁路姿态环" → 起转非线性窗口 | Phase A 台架标定电调起转点后重设门槛（A0-3） |

> 修复实施必须走"方案 → 你同意 → 执行"流程，逐项小步验证；本报告不代改。

---

## 6. 分阶段实施方案书

### 路线概要（先结论）

- **Phase A（当前重点）**：先清零"环路阻塞/参数记忆/起转非线性"三个软件问题，再用**拆桨台架**逐步把三轴"角度外环 + 角速度内环"调到小扰动收敛；**飞行验收标准**：微风（<2 级），起飞后松杆，姿态角 95% 时间 ≤±3°，无持续单向漂移（1 分钟内水平位移目测 < 2m 或漂速 <0.3m/s 参考），杆回中后 2s 内收敛到 ±1°，无高频抖振（角速度纹波 <±10°/s 且无 5–15Hz 等幅振荡）。
- **Phase B**：恢复 SPL06 + altitude_est（I2C2 互斥）、垂速 PID 闭环与"油门杆中位=悬停"语义，先台架（限位绳）验证定高 30–60s 高度误差 ≤±0.3m、无慢周期振荡。
- **Phase C**：目标从"松杆回水平"升级为"水平速度阻尼 + GPS 位置环"。缺的反馈：水平速度/位置（现有：GPS 经纬度/地速 5Hz 级、无光流/气压高度差分质量差）→ 近期可行 = GPS 地速阻尼（抗漂移），远期 = 光流/激光测距 + 气压 + GPS 松耦合。README 现有约束："没有光流/测距之前不要做 XY 定点"，Phase C 应在该硬件约束内以"速度模式"推进。
- **Phase D（可后置）**：GPS 返航 = 航点→航向+距离控制 + RTH 开关，依托 Phase C 的框架；验收：100m 内返航落点误差 <10m。

（改动前一律先给方案经你确认，本文档只做分析与设计，不含任何代码改动。以下为第 6 章的分阶段详文。）

### Phase A 详细方案：小扰动自稳收敛（先做，唯一阻塞项）

> 原则：**先清软件障碍，再台架调参，最后微风飞行验收**；任何一步未过，不进入下一步。

#### A0. 软件清障（改动点精确到文件/函数）

| 项 | 文件/函数 | 改动方向 | 验收 |
|---|---|---|---|
| A0-1 | `Modules/Motor/motor_test.c` → `MotorTest_Task` 循环内 L236 的 `Debug_Printf("[IMU] ...")` | 解锁后禁止每拍阻塞打印：用 `armed` 门控（未解锁才允许 10Hz 打印），或整体注释并改走黑匣子。**禁止在 100Hz 控制环内做任何阻塞 UART 传输**。 | 台架看 dt 直方图：解锁段 100% 落在 9–11ms，无 19ms 尖峰（黑匣子 t_ms 列验证）。 |
| A0-2 | `Modules/Control/pid_table.c` → `PidTable_Init()` / `Modules/Flash/blackbox.c` → `Blackbox_Task` | 增加"上电自动加载已保存参数"：Flash 就绪且 RAM 仍为编译默认时，自动执行 `PidTable_ImportSnapshot`（黑匣子任务初始化末尾最合适，先于电机任务第一拍解锁即可）。保留遥控手动 LOAD_SAVED 不变。 | 上电后串口/遥测能读到非 0 的有效表；重启不丢参。 |
| A0-3 | `Modules/Motor/motor_test.c` → `MotorTest_ApplyMixer` / `MOTOR_ATTITUDE_ENABLE_PWM_US` | 台架确认后再定：1050us 下限与"≤1050 姿态环旁路"两处门槛要防起转非线性区；建议引入"解锁确认油门最低 1100us 且电调全部起转后，姿态修正才全程允许"。 | 地面小油门测试四路转速单调，无某路卡在起转临界。 |
| A0-4 | 调参入口核验 | 确认遥控 SET/SAVE/LOAD 链路与 `PidTable_HandleCommand` 的 armed 锁一致；建议 SET 后自动落一份到 Flash（或保留手动 SAVE，飞行前强制先 LOAD 并看回包确认）。 | 遥控回包 status=OK 且值 == 目标值。 |

#### A1. 拆桨台架调参（四轮迭代，每轮只动一个量）

前置：机体固定或系留，允许自由倾转 ±30° 且能安全回位；电池充满；**拆桨**。

1. **符号/方向复核（见 7.5 清单 V2）**：逐轴手倾验证修正方向与电机差速方向、四电机转向、磁航向与机头一致性。这一步错则后面全错，做完在 `motor_test.h`/`mixer.h` 文件头固化记录。
2. **只开内环**：SET roll/pitch 内环 Kp（建议起点 2.0–3.0 us/°/s，Ki=0，外环 Kp=0，各限幅 200）。手拨机体 ±20°/s 松手，观察是否"越快越顶"（阻尼方向对）与回中快慢。逐步加 Kp 到"轻拨快速衰减、无高频细抖"（目标：一次扰动 1–2 个摆动内停）。**若 200us 限幅不够用再放开限幅，上限 250–300us**。
3. **加内环 I（可选）**：仅当松杆悬停出现常值偏转（如持续缓慢低头）才加 Ki 0.01–0.03（限幅已同输出限幅），并确认无低频慢振荡（<2Hz 呼吸感即过大）。
4. **开外环 P**：roll/pitch 外环 Kp 从 1.0 起（用户记忆目标体系 2.3/1.5 作上限参考），Ki=0、Kd=0；外环输出限幅（目标角速度上限）先设 100°/s（注意 pid_table 里 ROLL_MAX_RATE_DPS 同时当 i_limit 用）。台架手倾 20° 松手：期望 1–2 次小幅超调内回到 ±1°，周期 0.5–1.5s，无持续振荡。
5. **偏航**：先 yaw_rate Kp（方向阻尼，起点 2–4），机头回中不摆；再 yaw_angle Kp（航向保持，起点 2–4，磁力计须已硬铁校准且 `[MAG]` 日志 calhdg 稳定）；倾角 >25° 时外环冻结属已知设计，别在 25° 外评估航向。
6. **联动 12–15s**：四路同油门(1050–1200us)用手模拟扰动（拨倾、拍击、轻推），观察 P、I 输出与四路 PWM 在示波器/黑匣子中的波形：修正应连续、无方波抖振、无饱和骑墙。
7. 每轮把表 SAVE 并**写进本报告附录"调参记录表"**（场次号 + pid.csv 备份），确保每次飞行的表可回查。

调参经验参数（单位体系：外环 kp=deg/s 输出 per deg；内环 kp=us per deg/s）：
角度环直通模式（内环=0）下的经验：外环 Kp 1.0 时 10° 误差只给 10us——修正太弱，这也是历史场次"回正慢、漂移"的原因之一；**打开内环后外环 P 语义才成立**，2.3 档才有意义。

#### A2. 微风（<2 级）飞行验收

1. 起飞：油门缓慢推到离地，**起飞阶段不要大杆量修正**（前 2s 观察）；
2. 松杆 10s：姿态角 95% 时间 ≤±3°；滚转/俯仰角速度无 5–15Hz 等幅细抖（黑匣子 rrate/prate 高频分量判断）；
3. 单向漂移检查：1 分钟内同方向漂移目测 <2m（或地速均值 <0.3m/s，若 GPS 有效）；
4. 打杆脉冲后回中：2s 内回到 ±1°，无 3 次以上往返摆动；
5. 偏航：松杆 30s 航向漂 <10°；
6. 全程链路 connected=1，无失联（黑匣子 conn 列）。

**验收通过标准**：上述 1–5 全部满足才算 Phase A 达标，进入 Phase B。不达标时回 A1 按"先阻尼后刚度、一次一个量"收敛，禁止跳步。

#### 风险与回退（Phase A）

- 内环 Kp 过大 → 高频抖振（电调/桨响应延迟 5–15Hz 共振）：立即减内环 P 或限幅；确认 A0-1 已做（打印抖动常被误判为共振）。
- 外环 P 过大 → 低频往复（倒摆模式）：减外环 P，加内环刚度后再回填。
- 任何符号怀疑 → 立刻回 A1.1 台架，禁止空中猜测。
- I2C 偶发错误触发"整机重置+重采零偏"（motor_test.c L207-214）：飞行中这是**断桨级风险**，建议改为"连续 N 次错误才保护；单次错误仅丢弃本拍并计数"——改动在 `MotorTest_Task` 错误分支，属低风险改造，进入 Phase A 前完成。
- 回退方案：所有参数表 SAVE 前保留上一版 pid.csv；固件改动逐项小步并可在 Keil 一键回编译。

### Phase B 详细方案：油门→垂向稳定→定高

#### 目标
把"油门杆直通"改回"油门杆 = 垂速指令，中位 = 悬停/定高"，让机体自动保持高度，为 Phase C/D 提供垂直稳定。

#### B0. 传感器恢复
1. `Core/Src/app_freertos.c`：恢复创建 `baroTask`（取消 L221-222 置 NULL）；确认 I2C2 上 SPL06(0x76) 与磁力计分时（`i2c2_lock.c` 已提供互斥，baro/mag 两个任务都要遵守"锁内做整段读写"）。
2. `Modules/Baro/baro.c/spl06.c`：核对采样率（建议 50–100Hz 原始）、开窗与超时；`altitude_est.c` 输出 `z_m / vz_mps / valid`（其内部若用 IMU 垂速修正，接 `IMU_Attitude_VerticalAccelUp` 或垂速积分——以 altitude_est 实际接口为准，见 7.1.2）。
3. 台架验证：静置与上下 30cm 手移，串口/黑匣子 z 与 vz 平滑无漂移（10 分钟漂移 <10cm），机体倾斜时高度不跳变。

#### B1. 控制语义切换（改动点）
1. `Modules/Motor/motor_test.c`：解锁后油门路径改为 `base_us = AltitudeControl_Update(stick, z_m, vz_mps, valid, dt, armed)`，姿态环照旧在 base 之上叠加（保持现有姿态→混控通路）。
2. `Modules/Control/altitude_control.c/h`：把 `hover_pwm_us` 语义恢复为"悬停油门"（去掉"满杆上限"复用）；杆区语义与现有死区(400/600)兼容，先按原有 0.30/0.20 m/s 爬降速率验收，后续再加满杆速率档。
3. 起飞斜坡/离地确认/失效冻结状态机已存在于该文件，恢复调用即可；把"估计失效"策略与失联保护串联（高度估计失效 → 冻结油门不突变，不 disarm——仅当链路同时失联才走现有保护）。
4. `apex_protocol.* / pid_table.c`：HOVER_PWM_US 的描述与注释同步改回；新增/复用的高度遥测字段（现有 altitude_cm/vz 已在遥测里，无需改协议）。

#### B2. 台架/系留调参
1. 系留绳限制垂直行程 ±1m；杆中位起飞后观察高度保持。
2. 垂速 PID 起点：vz_kp=100(us per m/s？按 altitude_control 旧单位 us/(m/s) 100 档)、ki 0、kd 0；先调 KP 消除"呼吸感"（0.2–0.5Hz 高度慢振荡 = 增益过高或传感器延迟大），再加小 I 消除静差。
3. hover_pwm 自动标定：悬停 10s 平均基础油门写入 hover_pwm（手算再 SET 亦可）。
4. 抗风扰动测试（小风扇斜吹）：高度恢复时间 <2s，过冲 <0.3m。

#### 验收标准
无风/微风：定高 60s，|z − z_ref| ≤ 0.3m，垂速纹波 <0.1m/s（黑匣子）；全程无"慢周期高度振荡"；杆中位松杆不升降。

#### 风险与回退
- 气压受桨下洗流/阵风影响：靠 altitude_est 的 IMU 修正与低通；验证阶段避免正对螺旋桨气流。
- 若 SPL06 硬件损坏/停用，B 阶段受阻 → 先回 Phase A 状态（一行宏/条件编译切回直通模式），不要带病推进。
- 协议注释与遥控显示字段的单位要保持一致（遥测已有 altitude_cm/vz_cms）。

### Phase C 详细方案：水平位置稳定→定点

#### 现状与缺口（先说清楚）
- 已有：GPS（ATGM336H，经度/纬度/地速/航向/UTC，串口 9600 DMA，解析见 7.1.4）、磁航向（偏航外环已用）、姿态环（Phase A 后可用）。
- 缺：水平位置/速度的低延迟高精度来源（GPS 5–10Hz、1–2σ 精度米级、更新延迟 100–200ms）、室内无 GPS 时无任何水平参考、无光流/激光（README 明示约束："没有光流/测距之前不要做 XY 定点"）。
- 风扰下的物理：要抗 5–6 级风定点，姿态环只是内层；外层必须闭环"水平速度→倾角"。只靠 GPS 的位置环带宽 <0.1Hz，只能做到"慢收敛的 GPS 圆点"（半径 3–5m），不足以在阵风里稳定悬停。

#### C-lite（现有传感器即可，先做）：速度阻尼 / 位置缓闭环
1. 坐标系：GPS NED（北东地），机体航向用磁航向（GPS course 仅作参考）；把 GPS 地速解到机体水平分量，做"目标 0 速"的 PI 速度环，输出目标倾角（限幅 ±10°）交给现有角度外环（即角度外环目标不再只来自杆量，速度环输出叠加在杆量上；松杆=0速）。
2. 结构改动点：新增 `Modules/Control/position_control.c`（或 gps_hold 模块）：
   - 输入：GPS 经纬度/地速（注意帧率低 → 样本时间戳 + 零阶保持）、磁航向、Phase A 的姿态环接口；
   - 输出：roll/pitch 目标角叠加；
   - 环参数：v_kp（°/s per m/s）、v_ki（积分带限幅）、位置环 kp/ki（输出 m/s 级）——全部进 pid_table/协议（新 ID 追加到 30 之后 = 协议版本 0x07 仍兼容则放 reserved，或升版）。
   - GPS 无效/速度跳变 → 立即切回纯姿态模式（松杆回平），不允许带坏数据积分。
3. 验收：GPS 有效开阔地，微风：松杆 60s 内漂移半径 ≤5m（GPS 圆内），阵风后 10s 内回到圆内；**不追求室内/高精度定点**。

#### C-full（需新增硬件，列为"需采购/后续"）
- 光流（如 OF 类模块 SPI/I2C，>50Hz 输出）+ 激光/超声波测距（高度与悬停油门校正）→ 近地定点半径 <0.5m；
- 把高度源从气压升级为"气压+测距融合"；位置源从 GPS 升级为"光流为主 GPS 纠偏"；
- 新增硬件后本方案再更新引脚/总线规划（I2C2/SPI3/USART 占用需重排，注意 SPI3 已给 W25Q64、SPI1 已给 LoRa——光流大概率走 I2C2 或新软件模拟口，需硬件评审）。

#### 验收（C-full 追加）
室内/无风：松杆 60s 位置漂移 <0.5m；人为推离 1m 后 5s 内归位；定高+定点同时启用不互相干扰。

#### 风险与回退
- GPS 不可用（室内/楼宇）：C-lite 自动退化为 Phase A 纯姿态（无水平参考时绝不进速度环积分）；
- 速度/位置环输出与手动杆量叠加的优先级要明确（打杆时速度环让路或目标叠加）；
- 全部新参数都要可在线调、可 SAVE/LOAD、入黑匣子场次头。

### Phase D 详细方案：GPS 返航（可后置，标注优先级最低）

#### 目标
任何时刻触发 RTH：自动回"家点"（解锁后首次 GPS fix 点或遥控设定点），到家后悬停并降落到安全高度，再交还手动。

#### 内容要点
1. 协议：RTH 触发建议用 flight_mode/新 flag（须两端同步，协议从 0x07 → 0x08 或占用保留位——**与遥控端共同评审**，禁止单端改动）；遥测的 home_distance 字段现被磁航向复用，返航距离需另开字段（协议改版一起做）。
2. 家点：解锁瞬间或遥控一键"设家"记录经纬度（Flash 保存可选）。
3. RTH 状态机：起飞阶段禁 RTH（高度 <2m 忽略或先爬升 5m）→ 爬升到安全高度（如 15m）→ 航向对准家点（用 GPS course 与磁航向协同）→ Phase C 位置环跟踪家点（速度限制）→ 距离 <3m 减速悬停 → 自动降落（垂速限制）或转手动。
4. 失联自动 RTH 可作为后续增强（现失联=油门 0 上锁，是最安全的保守策略，改自动 RTH 需要充分测试）。
5. 验收：开阔地，100m 直线，RTH 全程无人工干预，落点距家点 <10m，高度全程 >3m 安全曲线。

#### 风险与回退
- 磁航向/地磁干扰 → 航向优先用 GPS course（运动时），静止悬停用磁航向；
- GPS 掉星：RTH 中掉星 → 立即切回姿态+悬停并告警，绝不盲飞；
- 实现依赖 Phase C-lite 的坐标与位置环框架，未达 C 验收前不做 D。

## 7. 配套模块、外设与数据链代码解析

### 7.1 传感器簇：磁力计 / 气压 / 高度估计 / GPS

#### 7.1.1 任务接线事实（谁在跑）

| 任务 | 创建？ | 说明 |
|---|---|---|
| Mag_Task | ✅（app_freertos.c:228） | I2C2 读 GY-273，~50Hz，只发布快照 + 硬铁校准 |
| GPS_Task | ✅（app_freertos.c:225） | USART2+DMA 空闲中断收 NMEA，解析 GGA/RMC |
| AttitudeDebug_Task | ✅（app_freertos.c:207） | USART3 每 100ms 打一条 [MAG]（姿态/[GPS] 打印在 `#if 0` 中） |
| Baro_Task | ❌ 被 `baroTaskHandle = NULL` 取代（app_freertos.c:221-222） | 恢复时**必须同时改两处**：L222 恢复 osThreadNew，且 `StartBaroTask`（L362-371 现为空转）函数体改为 `Baro_Task(argument)` —— 只改一处会静默失败 |

#### 7.1.2 Baro（死代码，Phase B 恢复对象）

- `baro.c` 任务层：SPL06 失败每 1s 重试 → 预热 40 样本 → 32 个地面压力样本"去极值裁剪平均"（±4）→ 每 20ms 轮询；相对高度 `h=44330*(1-(p/p0)^0.19029495)`；地面压力慢跟踪 α=0.001（设计为解锁冻结，setter 现无调用者）；连续 150 次"miss"触发整机重新 Init（3s）。
- `spl06.c` 驱动：地址 0x76/0x77；压力 8Hz OSR32、温度 8Hz OSR8；**数据率 8Hz(125ms/样本) 与任务 20ms 轮询不匹配**：ready 位判重后同一 raw 返回 false，`baro.c` 不分"无新样本"与"I2C 失败"一律 miss_count++ → 每两个真样本间 5–6 次假 miss → **约 3–4s 就累计到 150 上限触发整机软复位 + 重采地面压力**（若启用必现，属待修 BUG：需区分通信失败与暂无新样本）。
- `altitude_est.c`：三状态线性卡尔曼 [z, vz, ba]（100Hz 预测 / 气压更新），Q_acc=0.08、Q_bias=0.0002、R_baro=0.09m²、dt∈[5,30]ms、创新 >±2m 拒绝、连续 3 坏样本气压重置、气压超时 1s → healthy=false、vz 限 ±2m/s、ba 限 ±1.5m/s²。**接口齐备但零调用者、无锁（单上下文前提）**；`Init(initial_accel_bias)` 需要"静置垂向加速度零偏"，恢复时可用 `IMU_Attitude_VerticalAccelUp` 的静置均值提供（这是现成可用的融合入口，Phase B 重点接线对象）。

#### 7.1.3 Mag（运行中，供偏航外环与遥测）

- 总线：I2C2(PA8/PA9)，与 SPL06 共用；事务级 `I2C2_Lock(timeout)` 互斥；总线挂死有 `I2C2_BusRecover`（GPIO 补时钟踢开死从机+重配外设），Mag 任务在未持锁时调用（当前仅 Mag 一个活动用户，无碍；若恢复 Baro 需复核恢复期间的互斥）。
- 判型顺序：HMC5883L(0x1E) → QMC5883L(0x0D) → QMC5883P/QMC6308(0x2C)。数据序注意：HMC5883L 是 X,Z,Y 大端须一次连续读 6B；QMC 系小端 XYZ。
- 机体系映射（实测约定）：mx=−chip_x、my=chip_y、mz=chip_z；heading = atan2(−my, mx)，顺时针为正，**无倾角补偿**（倾斜>10° 即失真，属设计边界，不进姿态融合）。
- 低通 α=0.25（≈2.5Hz）；饱和判据 |raw|=4096（HMC）；连续 I2C 失败 150 次重判型。
- 硬铁校准：min/max 在临界区内采集；门槛 100 样本 + 每轴跨度 ≥0.25Ga；偏移=min/max 中点存 RAM + Flash 0x002000 双备份（64B/槽），黑匣子任务上电加载，Mag 任务芯片就绪后再补灌防竞态；旧映射下的 Flash 偏移被"已有 RAM 偏移不覆盖"保护（换装后需重新标定）。
- 消费方：遥测把校准航向塞进 `home_distance_m` 槽（字段名与内容不符，仅协议层提示）；偏航外环 `mag_heading_deg` 输入；黑匣子 `target_vz_cms` 槽记录百分之一度磁航向；attitude_debug 打印。
- 【软件 BUG 候选（需对照 HMC5883L 手册实机确认）】CRB 写 0x20（GN[2:0]=001 → ±1.3Ga 档）却注释"±1.9Ga"，且灵敏度系数 820 counts/Ga 属 ±1.9Ga 档；若芯片真按 ±1.3Ga 运行，磁场读数虚高 ~1.33 倍 → 硬铁偏移/模长/heading 全部受影响。修复方向二选一：CRB 改 0x40（±1.9Ga）或系数改 ~1090。**注意：此疑点只影响偏航质量，不影响 roll/pitch 自稳**——不是本次炸机候选根因，但 Phase A 调 yaw 前必须定案。
- 【注释不一致】QMC5883L Control1=0x1D 注释"50Hz"实为 200Hz（若按常见位定义）；QMC5883P 0xC3 档位存疑——用真机打印 raw/数据率反证即可，不影响功能正确性。

#### 7.1.4 GPS（运行中：数据全解析、不进控制）

- USART2(PA2/PA3) 9600 + DMA 空闲中断 + 1024B 环形缓冲；解析全在 GPS_Task（BelowNormal，逐字节喂解析器，无字节 sleep 2ms）。
- 只认 GGA 与 RMC，$ 定界、128B 上限、*hex 异或校验、20 字段上限、坏句整句丢弃——防护到位。
- RMC：时/分/秒+日/月/年（不合法整句丢）→ UTC Unix 秒；A 状态 → fix_valid；**纬度/经度（度分→1e-7° 带符号）**；地速（节×51.444→cm/s）；航向（×100→cdeg）。
- GGA：经纬度、fix_quality、星数、HDOP、海拔。
- **坑（Phase C 必须用 fix_valid 门控）**：GGA quality=0 时把空坐标/海拔解析成 0 **覆写上次有效值**（gps_atgm336h.c:172-173/178）；RMC 状态 V 时 speed/course 同样被覆写成 0。当前消费者只看时间/星数无实害。
- 消费者现状：blackbox 取 UTC 时间戳；radio 遥测取星数/地速/fix 标志。**经纬度/航向已解析但无人消费**——Phase C-lite 坐标输入零成本可得。
- GPS 帧率依赖模块配置（README：ATGM336H，通常 1–10Hz 可配），Phase C 前需确认模块输出频率并统一"数据时间戳"语义（解析时无 GPS 周内秒，仅任务到达时间近似）。

#### 7.1.5 姿态调试输出

- AttitudeDebug_Task 现只打印 [MAG]（10Hz，含 raw/机体系/heading/校准状态/计数）；[ATT] 姿态角与 [GPS]/[GPS_TIME] 整块 `#if 0` 关闭。需要姿态角实时观察时解除该块（注意与黑匣子 dump 互斥已有 Blackbox_IsDumping 门控）。



### 7.2 时钟 / 外设 / 中断 / 构建配置明细（源码亲核）

#### 7.2.1 时钟树（main.c:144-183 亲核）
- HSE 8MHz（HSE_VALUE 见 hal_conf）→ PLL M÷2 × N=85 → PLLR÷2 → SYSCLK **170MHz**；AHB/APB1/APB2 均 ÷1 → HCLK=PCLK1=PCLK2=170MHz；Flash 4 等待；VOS BOOST。
- TIM8 计数时钟：APB2=170MHz（分频 1 → 定时器时钟=170MHz），`Prescaler=169` → 1MHz 计数；`Period=19999` → **50Hz PWM**、比较值=微秒（1000–2000），**AutoReloadPreload=DISABLE（比较值立即生效）**，PWM1 高有效（tim.c:99-145 亲核）。TIM8 高级定时器主输出 MOE 在 main.c:113 与 Motor_Init 均使能。
- TIM2：输入捕获模式（Period=0xFFFFFFFF，CH1 上升沿 DIRECTTI、无滤波），全工程业务代码无任何引用（grep 亲核）——CubeMX 预留的"遥控脉宽捕获"死配置。
- HAL 时基 TIM6（1ms 中断 → HAL_IncTick）。

#### 7.2.2 中断（stm32g4xx_it.c 亲核）
- EXTI0（PB0 IMU_INT）：已进 HAL_GPIO_EXTI_IRQHandler，但**回调（radio.c:344-353）只处理 DIO1，IMU_INT 被忽略**——IMU Data Ready 中断已配但无人消费，采样靠轮询。
- EXTI2（PB2 LoRa DIO1）：优先级 6，`xTaskNotifyFromISR` 唤醒 radioTask（gpio.c:114-115 亲核）。
- DMA1_CH1（USART2 RX 空闲中断收 GPS）、USART2_IRQHandler、TIM6_DAC（HAL tick）。
- FreeRTOS 中断安全边界（亲核，**全部合规**）：configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5（调用 FromISR API 的中断优先级数值须 ≥5）；实际配置 DMA1_CH1=5（dma.c:48）、EXTI0=6（gpio.c:111）、EXTI2=6（gpio.c:114）、USART2=6（usart.c:174）、TIM6=TickPriority（timebase）、PendSV=15。唯一调用 FreeRTOS API 的中断是 EXTI2（6 ≥ 5 ✓）；EXTI0(IMU_INT) 已使能但回调忽略（无 API 调用，无违规）。

#### 7.2.3 已核实外设参数（亲核 + 子代理交叉）
| 外设 | 关键参数 | 备注 |
|---|---|---|
| TIM8 | 50Hz、PWM1、高有效、比较值立即生效、1MHz 计数 | PC6–PC9 四路电调 |
| USART3 | 115200-8N1（debug） | PC10/PC11，阻塞发送 |
| USART2 | GPS 9600，DMA 空闲中断 | PA2/PA3 |
| SPI1 | 5.3125MHz（170/32）、Mode0、软件 NSS | RA-01SC |
| SPI3 | 10.625MHz（170/16）、软件 NSS（PB6） | W25Q64（GPIO 速度 LOW 待波形确认） |
| I2C1 | MPU6050，7 位地址 0x68；Timing=`0x40621236`（400kHz FM，2026-09-10 由旧值 `0x40B285C2` 改来；旧值实测约 98.8kHz） | 轮询阻塞读写，单次 14 字节读实测 405µs |
| I2C2 | 磁力计/气压共用 PA8/PA9 + 事务锁；Timing 同 I2C1（400kHz） | 设备地址见 7.1 |
| ADC1 | IN1(PA0) 电池分压 11:1 ×1.03 修正 | 1Hz |
| GPIO | 关键引脚全表见 main.h 与 gpio.c：LED PF9/PF10；LoRa NSS PA4/SCK PA5/MISO PA6/MOSI PA7（SPI1 AF）、DIO1 PB2(EXTI2 上升沿)、NRST PE7、TXEN PE8、RXEN PE9、BUSY PE10（速度 LOW）；IMU_INT PB0(EXTI0)；W25Q64 CS PB6；电调 PC6–PC9（TIM8 AF，速度待波形确认）；GPS USART2 PA2/PA3 | 详见 gpio.c |

#### 7.2.4 构建与内存（亲核）
- Keil AC5（`<uAC6>0`，ARMCC5）+ C99（`<uC99>1`），优化 **-O1**（`<Optim>1`），宏 `USE_HAL_DRIVER,STM32G474xx`；FreeRTOS portable = RVDS/ARM_CM4F；包含路径覆盖全部 Modules 业务目录。启动文件栈 1KB / 堆 512B（AC5 两区模式，实际业务堆用 FreeRTOS heap_4 30KB）。
- 链接 `stm32g474xx_flash.sct`：FLASH 0x08000000 + 256KB；RAM 0x20000000 + 128KB；单镜像无 Bootloader 分区。
- VSCode/EIDE 入口（.vscode/tasks.json）：`${command:eide.project.build}`（编译）、`buildAndFlash`/`uploadToDevice`（烧录——**由你在 VSCode/EIDE 内执行**，本分析不代跑）。
- map 实测占用（最近一次构建产物 ApexFly v2_0.map）：
  - Code+RO Data ≈ **79.27KB / 256KB（31%）**
  - RW+ZI ≈ **39.71KB / 128KB（31%）**（含 FreeRTOS heap_4 30KB + 任务栈 + blackbox 静态缓冲）
  - ROM 总计 ≈ 79.29KB。
- Flash/RAM 余量充足，Phase B–D 新增代码无内存压力（软件侧结论）。



### 7.3 RA-01SC（SX126x/LLCC68）驱动与空口时序

- 默认射频参数（`RA01SC_GetDefaultConfig`）：433.92MHz、SF7、BW125k、CR4/5、前导码 8、显式头、CRC 开、IQ 正相、+10dBm、同步字 0x3444；TX 超时 3s；RX 为 **0x00FFFFFF 连续接收**（非单包窗口）。
- **单包空中时间**（Tsym=1.024ms，前导 12.544ms）：
  - 控制帧 19B → **~51.5ms**；遥测 31B → ~66.8ms；PID 响应 131B → ~215ms。
- SPI1 实跑 **5.3125MHz**（170MHz APB2 ÷32）；Busy 等待为纯忙等（200ms 上限、不让出 CPU）。
- 收/发：任务上下文阻塞 SPI 原语（单条 <1ms），TX_DONE/RX_DONE 走 DIO1→EXTI2→`xTaskNotifyFromISR` 通知 radioTask；每次收到控制帧 → 立刻回遥测（51.5ms+66.8ms≈**每轮一问一答 ≥120ms 空中占用**，实测链路周期 300–320ms 与估算吻合），TX 完成后回连续 RX。
- **半双工窗口结论**：飞机在"回遥测 TX（~67ms）+ 模式切换间隙（~1–2ms）"内不监听空中；若遥控按固定 100ms 时隙连发会周期性撞窗口丢帧（序号过滤会丢旧帧但不会重传——链路丢帧 = 杆量保持旧值，控制安全但操控延迟增大）。丢帧统计已有 `rejected_count`；建议 V8 实测确认对端节奏。
- 驱动级问题：Busy 忙等不让出（异常时单次最长 200ms 独占任务）；DIO2 掩码被误写成与 DIO1 相同（未接线无实害）；芯片在位检查只判 status 全 0/全 1；SPI 错误直接上抛 → radioTask 整模块复位重来（能自愈、代价大）。

### 7.4 黑匣子 / W25Q64 / 串口导出细节

- 记录格式：Flash 内**64B 定长二进制**（union 4 类记录：SESSION_BEGIN 含 26 项 PID q10000、ALTITUDE_PARAMS 含 4 高度参数+解锁 UTC、SAMPLE、SESSION_END 含样本数+上锁 UTC），CRC16 在字节 60/61；CSV 只是导出视图（23 列，1Hz 相对毫秒时间戳由导出工具换算）。**场次头钉死解锁瞬间的整张 RAM PID 表** → 这是回查"炸机当天实际参数"的唯一权威来源（V1）。
- 分区：0x000000/0x001000 PID 槽 A/B（分扇区真双备份）；0x002000 磁校准（**A/B 共用同一扇区、擦除互毁**——擦完→A 写前的掉电窗口双槽全失，真双备份形同虚设，为隐患，建议改分扇区或先写 A 再整体搬移策略）；0x003000 写指针索引；0x004000 起日志环形区（130560 条，写满覆盖最老）。
- 任务：32 项静态队列非阻塞提交（满则丢样本并计数 drop），电机环不因黑匣子阻塞；每跨 4KB 扇区边界擦除一次（擦除期任务停摆 ~50–150ms，理论可丢样本，有 drop 计数可观测）；10s 状态行含 drop 数。
- dump：`dump [N]` 走 USART3 CSV 导出，样本行 ~200–260B（≈22ms/行 @115200），20000 条约需数分钟；dump 期间 [MAG]/[IMU] 打印会以整行混入 CSV（互斥保证不拆行但会插行）——**motor_test 的每拍 [IMU] 打印在 dump 期间照打不误**（未查 Blackbox_IsDumping），属需要清理的交互。
- W25Q64：硬件 SPI3 @10.625MHz（GPIO 速度却配 LOW，需波形确认）；读写擦均带超时；`W25Q64_Write` 不自动擦除（调用方负责），WaitReady 在调度器运行时会 vTaskDelay(1) 让出。
- LED：PF9 蓝/PF10 红 1s 心跳；battery：PA0 ADC1_IN1，`raw×3300/4095×11×1.03`、16 平均+IIR，1Hz 更新，供遥测。

#### 7.4.1 每拍阻塞打印的真实代价（100Hz 环路串口占用实测估算）

- 主控循环每拍无条件打印一行 [IMU]（motor_test.c:236），行长 65–85B，@115200 单拍阻塞 ~6ms；名义 100Hz 下串口占用 61–74%，真实控制节拍被拖到 ~60Hz 且每拍抖动。A0-1（清障）优先级高于任何调参。



### 7.5 下一步行动清单：把根因缩到 1–2 个的最小成本验证

> 执行顺序建议 V1 → V2 → V3，其余按需。所有步骤均**不修改固件**；烧录/上电/转桨动作由你在 VSCode+Keil 中完成。

| # | 验证步骤（操作交还用户） | 预期（正常） | 异常含义（异常=哪条候因成立） |
|---|---|---|---|
| V1 | **回查炸机当场的黑匣子**：把当时导出（`dump [N]`/`dumpall`）跑 `BlackboxReader`，看**炸机场次的场次头 PID 表**与解锁段样本（roll/pitch/rate/corr/thr/motor 列） | 场次头参数 = 你记忆中的"最终版"（如 roll/pitch 外环 2.3、内环 1.5）；解锁段姿态<±15°、修正 <±100us | 若参数仍是 0.3/1.0 档或全 0 → **R1/R2/R3 坐实**：飞的是无阻尼/无内环状态；若姿态记录里解锁即冲 >30° → 方向/增益异常（回 V2）。**这是成本最低、信息量最大的一步** |
| V2 | **拆桨台架符号复核**：上电解锁、基础油门 1250us，手倾机体：右倾→M1/M2 快 M3/M4 慢；前倾→M2/M4 快；左倾/后倾对称；拧动 yaw（若已开 yaw）看差速方向 | 与 mixer.h L19-35 完全一致 | 任一轴反 → 该轴反馈正反馈，炸机直接原因（R7）；方向对 → 排除符号问题 |
| V3 | **控制节拍与打印占用**：解锁（拆桨）跑 20s，抓黑匣子/示波器，统计 `t_ms` 差直方图 + USART3 是否每 10ms 有 [IMU] 行 | 解锁后 dt 集中在 10ms（±1ms），无 [IMU] 行刷屏（若当前固件保留打印则会刷屏） | dt 主峰 >15ms 或 19ms 尖峰多 → **R5 坐实**（A0-1 必须清） |
| V4 | **解锁瞬间四路 PWM 波形**：示波器/逻辑分析仪四通道同时抓解锁→推杆过程 | 四路同步从 1000us 平滑上升，修正差 <10us 量级 | 某路跳变/不同步 → 电调通道映射或 TIM8 通道错（R7/R8）；1050us 附近出现"某路先到 1050 而其他仍在 1000"的平台 → A0-3 起转非线性（R9） |
| V5 | **IMU 静态健康**：机体静置（不上电转桨），串口/黑匣子看 30s：accel 模长在 0.98–1.02g 比例、gyro 三轴 ±0.3dps 内、加计门限通过率、mahony fusion_error 幅度 | 模长 >0.99g 通过率 >95%，fusion_error <0.02 | 模长经常出 0.8–1.2 窗 → 振动/加计噪声大（R4/机械振动），需要考虑加计软件低通后再进 Mahony |
| V6 | **I2C 错误与重置**：观察解锁后是否出现过 `i2c_error_count` 增长或"重新采零偏"日志（红灯/蓝灯组合或串口） | 错误计数 0 或极少，无重置 | 频繁重置 → 飞行中"断桨保护/重采零点"风险坐实，必须先把错误分支改为"连续 N 次才保护" |
| V7 | **四电机一致性**：固定台架（锁机体），逐电机单独 PWM=1200/1300/1400us 用光电转速计读数，记录四路曲线 | 同 PWM 转速差 <5% | 差 >10% → 电调/电机/桨不一致（R8），需硬件处理；另注意 PWM 指令与实测转速的滞后（ESC 时间常数） |
| V8 | **链路实测**：遥控开、不飞，黑匣子 conn 列与 accepted/rejected 计数（或遥控显示）观察 60s；记录实际控制帧到达间隔 | connected 恒定 1，无 rejected 增长，帧间隔与遥控设计一致（若一问一答 ≈300ms） | 丢包多/乱序增长 → R6 需与遥控端核时序；若 300ms 级更新说明手动打杆延迟大，飞手要相应放慢动作 |
| V9 | **Trim/漂移方向相关性**：在 V2 台架上把机体朝 8 个方向各静置 30s，记录解锁后松杆漂移方向（黑匣子 roll/pitch 均值） | 均值 ≈0（<0.5°）与朝向无关 | 与朝向相关 → 零点/机械（R4/R8）；与朝向无关的常值 → 混控符号或配平（回 V2） |
| V10 | **油门-姿态耦合检查**：台架（拆桨）慢推油门 1050→1500us，观察姿态角与四路差速是否随油门突变 | 姿态不变，四路同步 | 油门段引起姿态跳变 → 混控/起转/机架共振（R9/R8） |

**本轮验证的目标结论**：做完 V1+V2+V3，即可把根因从 10 条候因收敛到 1–2 条（最可能 = "无阻尼/参数缺失/节拍抖动" 的组合），再按 Phase A 依次清除。

## 附录 A：调参记录表模板（每轮台架/飞行必须填并归档 pid.csv）

| 日期/场次 | 内环 P (R/P) | 内环 I | 外环 P | 外环 I | yaw R P/I | 限幅 | hover PWM | Trim | 结论/现象 | 下一步 |
|---|---|---|---|---|---|---|---|---|---|---|
| （例）2026-xx-xx s361 | 0/0 | 0/0 | 1.0/1.0 | 0 | 0 | 200 | 1300(实测油门) | 0 | 小油门可维持 ±2°，大风必漂 | 开内环 |

> 填表依据一律以黑匣子场次头 pid 表为准，不以记忆为准（INDEX.md 教训）。

## 附录 B：单位与量程速查表（协议/遥测/黑匣子/内部）

| 量 | 内部单位 | 协议/遥测 | 黑匣子列 | 备注 |
|---|---|---|---|---|
| 姿态角 | 度(float) | 百分之一度 int16 | °（float 文本） | 机体系：右倾/抬头/顺时针为正 |
| 角速度 | 度每秒 | 百分之一 cdeg/s | °/s | 机体系 |
| 混控修正 | us(float) | — | us | 三轴各一 |
| 油门 | us | 杆 0–1000（无单位） | us | 1000↔1500(默认满杆) |
| 高度 | m(float) | cm(int32) | cm | 卡尔曼离地高，上正 |
| 垂速 | m/s | cm/s int16 | cm/s | 上正 |
| 磁航向 | 度 | home_distance_m: 0–359 / 0xFFFF | target_vz_cms 列（mag_hdg，百分之一度） | 未做倾角补偿 |
| GPS | 度(e7)/cm/s/cdeg | 星数/地速/fix | — | RMC/GGA |
| PID 参数 | float | q10000(int32) | pid.csv q10000→float | 30 项，场次头钉死 |
| 电池 | — | mV | — | 1Hz |

## 附录 C：本文档使用的关键证据文件清单

- 源码（V2.0 工程内）：Core/Src/*.c，Modules/{Control,IMU,Motor,Communication,Baro,Mag,GPS,Flash,De_Bug,Power,LED}/*.c/.h
- 配置：MDK-ARM/ApexFly v2.0.uvprojx、stm32g474xx_flash.sct、startup_stm32g474xx.s、ApexFly v2_0.map、.vscode/tasks.json
- 数据：BlackboxReader/exports/keep_v2/{INDEX.md, session_001/016/164/244/361.csv + .pid.csv}
- 文档：README.md（V2.0 工程自身说明）
- 分析方法：只读 + 文本统计（采样节拍直方图、修正比例中位数等，均在本文第 5 章引用结论）

