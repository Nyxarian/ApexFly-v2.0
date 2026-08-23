# ApexFly v2.0 飞机端接手说明

这份文件给以后回来改飞机端的人看。先读「当前卡在哪」和「不要改的约定」，再按「下一步」往下做。不要凭记忆从老工程整包恢复 IMU 六面校准、Flash 水平基准或黑匣子。

- 飞机工程：`F:\Zhuomian\ApexFly v2.0\ApexFly v2.0\`
- 老飞机（只当对照，不要当主工程）：`F:\Zhuomian\ApexFly\ApexFly\`
- 遥控器（协议必须对齐）：`F:\Zhuomian\CoreFly F2 Remoto\`
- 黑匣子读取工具（PC）：`F:\Zhuomian\ApexFly v2.0\BlackboxReader\`（老 Python `BlackboxDump` 已停用）
- 业务代码只放 `Modules/`，不要再新建 Device / APP / Control / BSP 那套老目录
- CubeMX 6.18 + HAL + FreeRTOS CMSIS-V2，芯片 STM32G474VCT6，时基 TIM6
- 注释用中文；`Modules/` 里函数声明和调用写成一行，不要把参数列表自动折行

---

## 当前卡在哪（2026-03，电池电压刚接上）

PID GET/SET/SAVE 和场次日志已经能用。电池电压从 PA0 分压读取，填进遥测 `battery_mv`，遥控仪表盘直接显示。低压保护还没做。

现在上电实际会做的事：

1. `ledTask`：PF9 蓝 / PF10 红，约 1 秒翻转
2. `imuTestTask`：空转，避免和电机任务抢 I2C1
3. `radioTask`：SPI1 初始化 RA-01SC；收控制帧回遥测；收 PID 命令回整张 26 项表
4. `blackboxTask`：SPI3 初始化 W25Q64；上电加载已保存 PID；SAVE 写双备份；解锁记场次日志
5. `motorTestTask`：电调 1000us 保持 3 秒解锁 → 采陀螺零偏 200 点 → 平放采水平零点 200 点 → 停在 1000us 等遥控 → 已解锁则油门 0~1000 映射到 1000~2000us
6. 6 组 PID 的 Kp/Ki/Kd 编译默认是 0；若 Flash 里有已保存表，上电用保存值
7. 遥控没点解锁、油门杆在最低、或失联超过 800ms：四路保持 1000us
8. `motorTestTask` 每秒采一次电池：PA0 / ADC1_IN1，10k:1k 分压 ×11，毫伏进遥测

---

## 已经完成

### 硬件（CubeMX，与可飞老板对齐）

| 功能 | 引脚 / 外设 |
|---|---|
| 调试串口 | USART3，PC10/PC11，115200。USART2 是 GPS，不要拿来打日志 |
| MPU6050 | I2C1，地址 0x68，±8g，±2000 dps |
| 电调 PWM | TIM8 CH1–CH4 = PC6–PC9，1 tick = 1 µs，50 Hz，1000–2000 µs |
| LoRa RA-01SC | SPI1：NSS=PA4，SCK=PA5，MISO=PA6，MOSI=PA7；DIO1=PB2；NRST=PE7；TXEN=PE8；RXEN=PE9；BUSY=PE10 |
| W25Q64 | SPI3：SCK=PB3，MISO=PB4，MOSI=PB5；片选 W_CS=PB6 |
| LED | PF9 蓝，PF10 红 |

### IMU

- 文件：`Modules/IMU/`（`mpu6050`、`imu_bias`、`imu_attitude`、`imu_level`）
- 陀螺零偏：静置 200 点 @ 100 Hz
- 水平零点：陀螺零偏之后再静置 200 点，对映射后的机体系横滚/俯仰求平均并减掉；偏航和角速度不减；只放 RAM
- 姿态：互补滤波，芯片系内部不要改公式
- 对外用机体系，`IMU_Attitude_MapToBody(attitude, imu_data, body)`，再 `IMU_Level_Apply()`
- 角度和角速度同一套映射：

```text
Roll  = 芯片系 pitch_deg，roll_rate  = gyro_y
Pitch = 芯片系 roll_deg， pitch_rate = gyro_x
Yaw   = -芯片系 yaw_deg， yaw_rate   = -gyro_z
```

机体系习惯：右倾为正、抬头为正、顺时针偏航为正。

### 电机 / 混控 / PID

- 电机：`Modules/Motor/motor.c`，测试任务 `motor_test.c`
- 混控：`Modules/Control/mixer.c`，纯数学，不写 TIM8，也不做 PID
- PID：`pid.c` 算法+状态；`cascade_pid.c` 外环角度 + 内环角速度
- 胶水：`attitude_control.c`，三轴各一份 `CascadePid_t`，一共 6 组独立参数
- 在线调参：`pid_table.c` 把 0x06 的 26 个 ID 映射到这 6 组
- 编译默认仍是 `kDefaultPidParams`（全 0）；遥控 GET/SET 改 RAM 表并立刻 Apply
- 黑匣子：`Modules/Flash/blackbox.c`，独立 `blackboxTask`，独占 SPI3
- SAVE 写 0x000000 / 0x001000 双备份；上电读合法槽；日志从 0x004000 起，写满停记
- 解锁开一场：场次头钉死当时 26 项 PID；100Hz 记样本；上锁写场次尾
- USART3 发 `dump` 按场次导出 CSV；正在记录时拒绝 dump
- 电池：`Modules/Power/battery.c`，不单独开任务；10k:1k 分压比 1/11，再乘 103/100（对照万用表 12.11V）
- 当前每组：`kp=ki=kd=0`，`i_limit=200`，`out_max=200`，`out_min=-200`
- 内环三项增益全 0 时，外环输出直通混控，方便先调角度环
- 电机布局（机头朝上，从上往下看）：M1 后右 CW，M2 前右 CCW，M3 后左 CCW，M4 前左 CW
- 混控内部公式不要改：`M1=-R+P-Y`，`M2=-R-P+Y`，`M3=+R+P+Y`，`M4=+R-P-Y`
- 送进混控的符号（实机拆桨已测，改极性只改调用处）：

```text
roll_correction_us  = 串级输出      （目标角 0 时已是 -测量）
pitch_correction_us = -串级输出     （再取反变成实机需要的 +Kp*pitch）
yaw_correction_us   = 串级输出      （先按横滚同号；拧航向反了只改这里）
```

拆桨倾斜期望（油门 1250）：

- 右倾：M1/M2 加快
- 左倾：M3/M4 加快
- 低头前倾：M2/M4 加快
- 抬头后倾：M1/M3 加快

### 通讯

目录：`Modules/Communication/`，Keil 分组同名，include 已加 `..\Modules\Communication`。

| 文件 | 作用 | 谁在用 |
|---|---|---|
| `ra01sc.c/.h` | RA-01SC / LLCC68 底层，引脚由 `RA01SC_Hardware_t` 注入 | `radio.c` Init |
| `apex_protocol.c/.h` | 与遥控器同一份 0x06 编解码 | `radio.c` |
| `control_link.c/.h` | 最新杆量快照 + 800ms 失联 | 电台写、电机读 |
| `radio.c/.h` | 电台任务：DIO1 通知、收控制、回遥测 | `radioTask` |

空中必须和遥控器一致：

```text
LoRa：433.92 MHz，SF7，BW125，CR4/5，10 dBm，公共同步字 0x3444
协议：头 A5 5A，版本 0x06，小端，CRC16-CCITT-FALSE
遥控约 100ms 发控制帧；飞机收包后立刻回遥测；TX_DONE 后再进接收
```

控制负载：油门 0~1000，roll/pitch/yaw -1000~1000，`APEX_CONTROL_FLAG_ARM_REQUESTED` 表示请求解锁。

快照职责：无线任务写、电机任务读，无线任务不写 PWM。超时则 `connected=false`，油门和解锁标志清零。失联后允许任意序号重新建链。

### Flash（底层在，上层还没有）

目录：`Modules/Flash/`，Keil 分组同名，include 已加 `..\Modules\Flash`。

| 文件 | 作用 | 谁在用 |
|---|---|---|
| `w25q64.c/.h` | W25Q64 识别 / 读 / 按页写 / 4KB 擦除 | 还没 Init |

命令和时序沿用老飞机 `Device/w25q64.c`。本轮不创建黑匣子任务，也不写 PID 双备份。写之前必须先擦扇区。

### 任务（CubeMX / `app_freertos.c`）

| 任务 | 栈 | 优先级 | 现状 |
|---|---|---|---|
| `defaultTask` | 128 | Normal | 空转 |
| `imuTestTask` | 512 | Normal | 空转，I2C 让给电机任务 |
| `motorTestTask` | 512 | Normal | IMU + 读遥控快照 + 电机 |
| `ledTask` | 128 | Normal | 心跳 |
| `radioTask` | 512 | AboveNormal | 收控制、回遥测，不写 PWM |

VSCode 对 `Modules/` 的红线和 IntelliSense 有关，不是 Keil 编不过。不要把 `Modules/` 挪进 `MDK-ARM`。

---

## 不要改的约定

1. 不要改 `mixer.c` 内部加减号；要改极性，改 `attitude_control` 出口符号。
2. 不要改 IMU 芯片系互补滤波，也不要改机体系映射公式。
3. 6 组 PID 必须独立对象，禁止共用积分 / last_error。
4. 电调解锁：四路 1000us 保持 3000ms。
5. 调试口永远是 USART3；USART2 是 GPS。
6. 协议版本先保持 **0x06**，不要为飞机 v2 另做一版瘦协议。遥控器已在发这套。缺的遥测填 0；旧六面校准操作码回 `INVALID_OPERATION`，不要再实现校准。
7. 改 LoRa 默认参数或协议字节，必须飞机和遥控器一起改。
8. 不要从老工程搬回六面加速度校准、Flash 水平基准和老 `app_blackbox`。新黑匣子只在 `Modules/Flash/blackbox.c`。水平零点这轮仍只放 RAM，每次上电重采。
9. 任何电机 / PID 测试先拆桨。没说「装桨飞」就不要上桨。
10. 用户没说执行就不要改代码；说了「修改 / 写进去 / 继续 / 执行」再动手。

---

## 下一步（按这个顺序，不要跳）

### 1. 拆桨验收遥控跟杆（当前）

飞机先上电等电调解锁 + 陀螺零偏 + 水平零点（大约 6 秒），放平、别碰。USART3 上看 `[RADIO] rx ready`、`[ATT] level roll=...`，之后每秒一条 `[ATT] r=... p=...`，静置时应接近 0。

| 步骤 | 期望 |
|---|---|
| 飞机单独上电 | 3 秒 1000us，采陀螺零偏和水平零点，电机停 |
| 遥控开机，不点解锁，推油门 | 仍停 |
| 点解锁，油门最低 | 仍停（1000us） |
| 慢慢推油门 | 四路一起加快 |
| 收回油门 / 关掉遥控 | 一起停 |
| 用手倾斜（Kp=0） | 四路转速几乎一样 |

### 2. 拆桨验收遥控 PID 页（当前）

飞机上电、链路通了之后进 PID 页。不要先点解锁。

| 步骤 | 期望 |
|---|---|
| 进 PID 页 | 显示已读取，Roll 外环 Kp 为 0，不是遥控本地的 5 |
| 上锁时改一项并确定 | 显示已应用，USART3 有 `[PID] SET ...` |
| 解锁后再改 | 提示请先锁定 |
| 点保存 | USART3 有 `[PID] save queued` 和 `[BB] save gen=... ok`，再进 PID 页仍是刚保存的数 |
| 再上电 | GET 到的是 Flash 里的表，不是编译默认 0（除非从没保存过） |

### 3. 拆桨验收黑匣子日志

| 步骤 | 期望 |
|---|---|
| 上电 | `[BB] ready ...`，没有保存过则 `no saved pid` |
| 解锁几秒再上锁 | `[BB] session N begin` / `end samples=...` |
| 上锁后串口发 `dump` 回车 | 按场次打出 `# session=`、`# pid ...` 和 CSV |
| 正在解锁时发 dump | `disarm first` |

### 4. 拆桨验收电池电压

万用表量电池，对照 USART3 的 `bat=`（单位毫伏）和遥控仪表盘。

| 步骤 | 期望 |
|---|---|
| 插上健康 3S | `[ATT] ... bat=11xxx` 量级，遥控电池电压接近万用表（差 0.1～0.3V 正常） |
| 偏差很大 | 改 `BATTERY_SCALE_NUM/DEN`，不要改分压比 11 |

### 5. 台架调参（仍然拆桨）

改 `kDefaultPidParams`，建议顺序：

1. `roll_angle.kp` / `pitch_angle.kp`（可先试 10），确认倾斜方向仍对
2. 对应 `*_rate.kp`
3. 偏航先 `yaw_rate`；没磁力计时 `yaw_angle` 先保持 0
4. Ki 最后、很小

内环还是 0 时，外环会直通混控，和之前开环倾斜测试一样。

### 6. 更后面才做

- 把 `imuTestTask` 收成正式 IMU 任务，控制走 `IMU_GetAttitude()` 一类接口
- GPS 填场次 UTC、高度
- 低压自动上锁（遥控阈值已有，飞机端还没做）
- 遥控擦日志、环写策略
- 定高、失联返航

---

## 和遥控器怎么对齐

遥控器工程先独立改，飞机这轮不要去动它。飞机对齐现有 0x06 即可。

必须字节级一致的只有两样：

1. `apex_protocol.h/.c`
2. `RA01SC_GetDefaultConfig()` 的空中参数

不要统一的：MCU、SPI 编号、GPIO 名字、任务、UI、摇杆。驱动用硬件结构体注入引脚，这点已经对了。

遥控端无线在 `App_Radio_Task.c`，硬件是 F407 + SPI2 + `LoRa_*` 引脚。

---

## 回来接着干时建议先读的文件

1. 本 README
2. `Modules/Control/mixer.h`（混控符号和倾斜期望）
3. `Modules/IMU/imu_attitude.h`（机体系映射）
4. `Modules/Control/attitude_control.c`（6 组参数表和串级出口符号）
5. `Modules/Motor/motor_test.c`（当前唯一真正跑控制的任务）
6. `Modules/Control/pid_table.c`（26 个协议 ID 与 RAM 表）
7. `Modules/Flash/blackbox.c`（PID 双备份和场次日志）
8. `Modules/Power/battery.c`（PA0 分压 1/11，毫伏进遥测）
9. `Modules/Communication/radio.c`（收控制回遥测，收 PID 回整表）
10. `Modules/Communication/control_link.h`（快照和失联）
11. `Modules/Communication/apex_protocol.h`（0x06 帧）

下一步先拆桨验收：保存后重启仍能 GET 到，解锁一场后串口 `dump` 能导出 CSV。
