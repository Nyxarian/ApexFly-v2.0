# ApexFly v2.0

STM32G474 四轴飞机端固件，以及配套的 PC 黑匣子读取器。

遥控器工程不在本仓库，协议必须与 `CoreFly F2 Remoto` 对齐。

## 仓库里有什么

```text
ApexFly v2.0/          飞机端 Keil + CubeMX 工程
  Core/                CubeMX 生成：时钟、GPIO、UART、I2C、SPI、TIM、ADC
  Modules/             业务代码（IMU、电机、PID、电台、黑匣子、气压、GPS、磁力计）
  Drivers/             STM32G4 HAL / CMSIS
  Middlewares/         FreeRTOS
BlackboxReader/        Windows 黑匣子读取工具（C#）
```

业务代码只放 `Modules/`，不要再新建 Device / APP / Control / BSP 那套老目录。

## 当前状态

姿态环已经能飞。油门是直通映射，没有垂速环，也没有 XY 定点。GPS 只用来打时间和观察定位，不参与控制。黑匣子日志写满后从头覆盖。

四根杆已经进飞控：

- 油门：直通 PWM，满杆上限是 `APEX_PID_HOVER_PWM_US`
- 横滚 / 俯仰：杆量映射成外环目标角，满杆约 ±10°，再加上 Trim；回中回平
- 偏航：杆出死区跟角速度，回中锁当时的磁航向。外环测量是校准后的 `calhdg`，不是 IMU 积分偏航

气压计任务已停，I2C2 只给磁力计。高度遥测先填 0。磁力计硬铁校准写在 W25Q64 `0x002000`。遥测里原来的返航距离字段现在发磁航向 0～359°；黑匣子 `target_vz_cms` 同样记磁航向。

实际上电会跑这些任务：

| 任务 | 作用 |
|---|---|
| `ledTask` | PF9 蓝 / PF10 红心跳 |
| `imuTestTask` | 空转，避免和电机任务抢 I2C1 |
| `radioTask` | RA-01SC 收控制、回遥测和 PID 表 |
| `blackboxTask` | W25Q64 存 PID、磁力计校准、记场次、串口 dump |
| `baroTask` | 已停用，任务空转 |
| `gpsTask` | USART2 ATGM336H，解析 GGA/RMC |
| `magTask` | I2C2 读 GY-273，硬铁校准后给偏航外环 |
| `attitudeDebugTask` | USART3 打 `[ATT]` / `[GPS]` / `[GPS_TIME]` / `[MAG]` |
| `motorTestTask` | IMU 解算 + 姿态 PID + 混控 + 直通油门 |

## 硬件

芯片 STM32G474VCT6，CubeMX 6.18 + HAL + FreeRTOS CMSIS-V2，时基 TIM6。

| 功能 | 引脚 / 外设 |
|---|---|
| 调试串口 | USART3，PC10/PC11，115200。USART2 是 GPS |
| MPU6050 | I2C1，地址 0x68，±8g，±2000 dps |
| SPL06-001 | I2C2，地址 0x76（可拔掉，总线留给磁力计） |
| GY-273 磁力计 | I2C2，HMC5883L=0x1E 或 QMC5883L=0x0D；只接 VCC/GND/SCL/SDA，DRDY 不接 |
| GPS ATGM336H | USART2，PA2/PA3，9600，DMA RX |
| 电调 PWM | TIM8 CH1–CH4 = PC6–PC9，50 Hz，1000–2000 µs |
| LoRa RA-01SC | SPI1：NSS=PA4，SCK=PA5，MISO=PA6，MOSI=PA7；DIO1=PB2；NRST=PE7 |
| W25Q64 | SPI3：SCK=PB3，MISO=PB4，MOSI=PB5；片选 W_CS=PB6 |
| LED | PF9 蓝，PF10 红 |
| 电池 | PA0 / ADC1_IN1，10k:1k 分压 |

## 机体系与混控

机体系：右倾 Roll+，抬头 Pitch+，顺时针 Yaw+。

```text
Roll  = 芯片系 pitch，roll_rate  = gyro_y
Pitch = 芯片系 roll， pitch_rate = gyro_x
Yaw   = -芯片系 yaw， yaw_rate   = -gyro_z
```

电机（机头朝上俯视）：M1 后右 CW，M2 前右 CCW，M3 后左 CCW，M4 前左 CW。

混控内部公式不要改：

```text
M1 = -R + P - Y
M2 = -R - P + Y
M3 = +R + P + Y
M4 = +R - P - Y
```

油门：遥控杆 0–1000 映射到 `1000 + stick * (max-1000) / 1000`。`APEX_PID_HOVER_PWM_US` 现在是满杆 PWM 上限，默认 1500，可调 1200–2000。上锁 / 失联 = 1000 µs。

## 通讯

空中必须和遥控器一致：

```text
LoRa：433.92 MHz，SF7，BW125，CR4/5，10 dBm，同步字 0x3444
协议：头 A5 5A，版本 0x07，小端，CRC16-CCITT-FALSE
```

GPS 合法 RMC 后，遥测带 UTC Unix 秒和 `APEX_TELEMETRY_FLAG_GPS_TIME_VALID`。遥控器显示北京时间（UTC+8）。

## 黑匣子

- PID 双备份：`0x000000` / `0x001000`
- 写指针索引：`0x003000`
- 日志：`0x004000` 起，64 字节一条，写满后覆盖最老数据
- `dump [N]` 从当前写指针往回取最近 N 条（默认 20000）
- 场次头钉死当时 PID；解锁/上锁 UTC 分别写在高度参数记录和场次尾
- PC 工具：`BlackboxReader/ApexFlyBlackboxReader.exe`（源码在同目录，用 `compile.bat` 编）

## 不要改的约定

1. 不要改 `mixer.c` 内部加减号。
2. 不要改 IMU 芯片系公式和机体系映射。
3. 6 组 PID 必须独立对象。
4. 电调解锁：四路 1000 µs 保持 3 秒。
5. 调试口永远是 USART3；USART2 是 GPS。
6. 协议版本 **0x07**，飞机和遥控器必须同步。
7. 任何电机 / PID 测试先拆桨，没说装桨就不要上桨。
8. 磁力计机体系映射当前是 MAG4：`mx = -chip x`，`my = chip y`，`mz = chip z`。航向不要再额外加 90°。
9. 没有光流 / 测距之前不要做 XY 定点。

## 已知限制

- 松杆只会把横滚俯仰拉回水平，不会停在地面某一点。
- 偏航外环默认 Kp 为 0；空中若仍有净偏航力矩，先加 YAW RATE，不要先把 YAW ANGLE 拉大。
- 倾角大约超过 25° 时偏航外环会冻，目前没有倾角补偿。

## 编译

1. Keil uVision，工程 `ApexFly v2.0/MDK-ARM/ApexFly v2.0.uvprojx`
2. 目标芯片 STM32G474VCTx，ARMCC 5
3. 烧录 ST-Link / J-Link，SWD

读取器：

```text
BlackboxReader/compile.bat
```

需要本机 `C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe`。

## 回来先看这些文件

1. 本 README
2. `Modules/Control/mixer.h`
3. `Modules/IMU/imu_attitude.h`
4. `Modules/Motor/motor_test.c`
5. `Modules/Control/pid_table.c`
6. `Modules/Flash/blackbox.c`
7. `Modules/GPS/gps.c`
8. `Modules/Communication/apex_protocol.h`
9. `BlackboxReader/README.md`
10. `Modules/Mag/mag.c`
11. `Modules/Control/attitude_control.c`
