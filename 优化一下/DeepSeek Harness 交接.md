# ApexFly V2.0 · DeepSeek Harness 交接

> 更新时间：2026-09-28
>
> 用途：把当前 Codex 会话迁移到 DeepSeek Harness 后，先读本文件，再读项目根目录的 `AGENTS.md` 和 `优化一下/会话交接.md`。
>
> 本文件只描述当前工作状态，不代表代码已经编译、烧录或真机验证。

## 1. 新 Agent 开场提示词

```text
这是 ApexFly V2.0 四轴飞控项目。

工作区：F:\Zhuomian\ApexFly v2.0
机载端仓库：F:\Zhuomian\ApexFly v2.0\ApexFly v2.0
遥控端仓库：F:\Zhuomian\CoreFly F2 Remoto

开始前必须读取：
1. 项目根目录 AGENTS.md
2. 优化一下\会话交接.md
3. 优化一下\DeepSeek Harness 交接.md
4. 优化一下\代码学习计划.md

当前主要目标：
1. 先完成机载端参数版本黑匣子功能的真机验证；
2. 然后按照代码学习计划，从启动、PWM、电机、MPU6050 开始逐模块学习代码。

硬规则：
- 中文回答，先给结论，再给依据。
- 未经用户明确批准，不修改代码、工程文件或配置。
- 不烧录、不擦除、不占用串口、不假装硬件验证成功。
- 用户在 Keil 中编译、烧录和测试。
- 任何固件修改先说明方案，验证后再提交 Git。
- 不要清理、重置、覆盖当前未提交修改；先查看 git status。
```

## 2. 项目基本信息

- 机载端：STM32G474VCT6，自写四轴飞控，Keil MDK / ARM Compiler 5 / C99 / FreeRTOS。
- 控制环：`motorTestTask`，约 250Hz，由 IMU DRDY 信号驱动。
- 遥控端：另一个仓库 `F:\Zhuomian\CoreFly F2 Remoto`，STM32F407 + LVGL。
- 机载端调试口：USART3，115200 8N1；AI 不使用用户串口。
- 用户负责：Keil 编译、烧录、拆桨台架和真机飞行。

## 3. 当前代码状态

### 已完成并已经经过前期验证

- OF1~OF5：光流读取、质量门禁、零点标定、前后左右方向已真机通过。
- OF6/OF7：二维坐标旋转、位置环/速度环计算、门禁、锚定、手动飞行态、锁存和恢复；宿主机仿真已通过。
- OF8：光流、链路、姿态、高度和解锁许可门禁；故障锁存设计已真机取证。
- OF9：黑匣子光流扩展记录，定点样本包含 dx/dy、quality、累计量、速度、目标速度、raw/out 姿态角和状态。
- OF10：飞控和遥控端独立定点请求位、遥测定点状态和遥控器开关已联调。
- OF11：位置/速度环输出接入普通模式姿态目标，当前定点修正限幅为 `±8°`；定高模式不叠加定点修正。
- OF15：两端参数表扩展到 39 项，增加定点位置环/速度环 KP、KI、KD；遥控端支持 SET、SAVE、LOAD、上电加载。

### 当前刚完成、尚未真机验证的机载端功能

每次成功的参数 SET/LOAD 会写入一个参数版本：

- 记录完整 39 项 PID 参数；
- 记录修改操作和修改参数名称；
- 记录 revision 序号、场次号和时间；
- 每个版本物理上由 4 条 64 字节记录组成；
- SAVE 成功后另外写入 `pid_save` 标记；
- 导出时显示 `# pid_revision` 和 `# pid_save`。

这部分只改了机载端，遥控端协议不需要重新修改。机载端 ARMCC 语法检查已通过，64 字节记录静态检查已通过，但尚未由用户在 Keil 中完整编译、烧录和真机验证。

## 4. 当前工作区状态

### 机载端未提交修改

目前机载端仍有 OF15 和参数历史相关修改，主要文件包括：

- `ApexFly v2.0/Modules/Communication/apex_protocol.h`
- `ApexFly v2.0/Modules/Control/pid_table.c/.h`
- `ApexFly v2.0/Modules/Control/position_control.c`
- `ApexFly v2.0/Modules/Flash/blackbox.c/.h`
- `ApexFly v2.0/Modules/ToF/tof.c`
- `优化一下/会话交接.md`
- `优化一下/光流定点施工顺序.md`

还有未跟踪内容：

- `ApexFly v2.0/MDK-ARM/.clangd`
- `硬件/`
- `优化一下/代码学习计划.md`

不要把 `硬件/` 当作本次固件修改提交内容，也不要自动删除 `.clangd`。

### 遥控端未提交修改

遥控端当前有 OF15 配套修改：

- `CoreFly F2 Remoto/MDK-ARM/Algorithms/ui.c`
- `CoreFly F2 Remoto/MDK-ARM/Apps/App_Radio_Task.h`
- `CoreFly F2 Remoto/MDK-ARM/Communication/apex_protocol.h`

本次参数历史功能没有新增遥控端代码。

### 已有关键基线提交

- 机载端：`a2ac82e OF11: 接入定点姿态目标基线`
- 机载端：`84a26a5 OF9: 完成定点安全门禁与黑匣子记录`
- 机载端：`2540d41 OF10: 完成定点请求接入与双板联调基线`
- 遥控端：`aba4485 OF15: 在线调参扩展前基线`

当前最终修改尚未提交，因为用户要求先真机测试，再决定提交。

## 5. 当前第一件待办

先由用户在 Keil 中只重新编译、烧录机载端，然后按下面顺序验证参数版本历史：

1. 未解锁时 GET 当前 PID 参数；
2. 设置位置环 KP=`2.0`，确认遥控器显示“已应用”；不 SAVE；
3. 解锁并完成一次测试，再上锁；
4. 设置位置环 KP=`2.5`，完成第二次测试；
5. 设置位置环 KP=`3.0`，执行 SAVE，再完成第三次测试；
6. 断电重启后 GET，确认 `3.0` 仍然存在；
7. 用 `BB DUMP ALL` 导出完整日志。

期望导出中出现：

```text
# pid_revision ... changed=33(position_kp) ... part=1/4
# pid_revision ... changed=33(position_kp) ... part=2/4
# pid_revision ... changed=33(position_kp) ... part=3/4
# pid_revision ... changed=33(position_kp) ... part=4/4
...
# pid_save ...
```

不要用默认的短 dump 判断历史是否存在，因为短 dump 可能从场次中间开始，把前面的参数版本头截掉。

## 6. 最新飞行数据结论

数据文件：`F:\Zhuomian\ApexFly v2.0\apex_session_20260927_103954.csv`

- 这次记录的 `vel_sp_f / vel_sp_r` 全部为 0，说明该次飞行没有体现出有效的位置外环目标速度；需通过新的参数版本日志确认实际使用的 position KP。
- 第一次飞行约 5.3s 到 26.8s 处于定点 ACTIVE，之后进入 `state=3 / fault=4` 的高度故障锁存。
- 后续两次起飞仍处于锁存，位置修正输出为 0。
- 左前漂移不能只归因于 PID 增益，还要区分位置环参数是否真正进入飞控、ToF 有效性门禁是否锁存，以及偏航变化对光流累计坐标的影响。
- 起飞后的顺时针偏航是另一条链路，暂时不和定点参数整定混在一起。

## 7. 代码学习入口

学习计划文件：`优化一下\代码学习计划.md`

第一条主链路：

```text
Core/Src/main.c
  → Core/Src/app_freertos.c
  → MotorTest_Task()
  → PWM / 电机
  → MPU6050_Init()
  → MPU6050_ReadData()
  → IMU_GyroBias_Apply()
  → IMU_Attitude_Update()
  → IMU_Attitude_MapToBody()
  → AttitudeControl_Update()
  → PID / Mixer / Motor_Write()
```

学习时默认只读代码，不修改、不提交、不烧录。每个模块都按“头文件 → 初始化 → 原始读取 → 转换 → 滤波/校准 → 有效性 → 上层调用 → 输出 → 验收”顺序学习。

## 8. 当前明确限制

- 不要假设 Codex 的聊天记录会自动出现在 DeepSeek Harness 中。
- DeepSeek Harness 需要重新绑定本地工作区，并读取本文件、`AGENTS.md`、`会话交接.md` 和 `代码学习计划.md`。
- 不要自动清理未提交修改。
- 不要自动运行 Keil、烧录器、串口探测或擦除 Flash。
- 不要在用户验证前提交最终版本。
