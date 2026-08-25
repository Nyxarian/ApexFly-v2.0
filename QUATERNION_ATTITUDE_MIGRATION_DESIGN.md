# ApexFly v2.0 六轴四元数姿态融合迁移方案

## 1. 文档目的

本文把尚硅谷无人机教程中的四元数姿态算法整理成适用于 ApexFly v2.0 的数学和接口方案。

本阶段只确定设计，不修改固件代码、不构建、不烧录。后续实现必须经过无桨验证和影子运行，不直接替换带桨控制输入。

目标范围仅包括：

- MPU6050 六轴姿态融合；
- 四元数状态和 Roll/Pitch/Yaw 输出；
- 陀螺仪零偏、采样周期和加速度有效性；
- 与现有姿态串级 PID、混控和黑匣子接口的衔接。

暂不处理：

- 激光雷达或其他高度传感器；
- 气压高度状态机整改；
- 垂直速度控制参数；
- 磁力计和绝对航向；
- 姿态 PID 增益重新整定。

## 2. 现有 ApexFly 边界

当前相关代码：

- `ApexFly v2.0/Modules/IMU/mpu6050.c`
- `ApexFly v2.0/Modules/IMU/mpu6050.h`
- `ApexFly v2.0/Modules/IMU/imu_bias.c`
- `ApexFly v2.0/Modules/IMU/imu_level.c`
- `ApexFly v2.0/Modules/IMU/imu_attitude.c`
- `ApexFly v2.0/Modules/IMU/imu_attitude.h`
- `ApexFly v2.0/Modules/Control/attitude_control.c`
- `ApexFly v2.0/Modules/Control/cascade_pid.c`
- `ApexFly v2.0/Modules/Flash/blackbox.h`
- `ApexFly v2.0/Modules/Flash/blackbox.c`
- `ApexFly v2.0/Modules/Motor/motor_test.c`

当前输入单位：

- 加速度：`g`；
- 陀螺仪：`deg/s`；
- 采样时间戳：`HAL_GetTick()`，单位毫秒。

当前电机任务：

- 目标周期约 10 ms，即 100 Hz；
- `motor_test.c:216-218` 根据系统 tick 计算实际 `control_dt_s`；
- 当前将实际周期限制到 `0.005～0.030 s`；
- `motor_test.c:220` 先扣除陀螺仪零偏，再进入姿态更新；
- `motor_test.c:226` 把姿态映射成机体系输出；
- `motor_test.c:306` 将机体系角度和角速度交给现有姿态串级 PID。

因此四元数模块不能改变以下外部控制接口：

```c
IMU_BodyAttitude_t
AttitudeControl_Update(float throttle_us, const IMU_BodyAttitude_t *body, float dt_s, MixerOutput_t *output)
```

## 3. 教程算法的准确定位

教程最终版 `Com_imu.c` 不是纯四元数积分，也不是 EKF，而是六轴 Mahony 风格算法：

```text
陀螺仪角速度
    -> 四元数积分预测
加速度重力方向
    -> 与四元数预测重力方向做叉乘
    -> Kp 比例修正 + Ki 积分修正
修正后的角速度
    -> 更新并归一化四元数
```

可迁移的是结构，不可直接迁移的是：

- 固定的陀螺量程换算；
- 固定 6 ms 周期；
- `Kp=0.8`、`Ki=0.0003` 等参数；
- 教程的轴符号；
- 教程的无限积分和无数据有效性判断。

## 4. 数学约定

### 4.1 内部四元数

四元数统一存储为：

```text
q = [q0, q1, q2, q3]
```

其中：

```text
q0：标量部分
q1：X 轴虚部
q2：Y 轴虚部
q3：Z 轴虚部
```

单位约束：

```text
q0² + q1² + q2² + q3² = 1
```

每次更新后必须归一化。归一化前若范数非有限、过小或溢出，应拒绝本次更新并标记姿态数据无效。

### 4.2 角速度

输入陀螺仪数据为 `deg/s`，四元数计算使用 `rad/s`：

```text
omega_rad = omega_dps * pi / 180
```

不能使用教程中绑定 `±2000 deg/s` 原始计数的换算常数。ApexFly 驱动已经把原始值转换成 `deg/s`，姿态模块只负责单位转换。

### 4.3 误差反馈

定义归一化后的加速度观测向量为 `a`，定义当前四元数计算出的重力参考方向为 `v`。

```text
e = a × v
```

其中 `e` 是三轴方向误差。修正角速度：

```text
integral = integral + Ki * e * dt
omega_corrected = omega_rad + Kp * e + integral
```

当加速度不可信时，暂时令重力反馈权重为零：

```text
omega_corrected = omega_rad
```

推荐实现连续可信度权重 `w_accel`：

```text
omega_corrected = omega_rad + w_accel * (Kp * e + integral)
```

第一版可以先使用与当前代码一致的模长门限，验证通过后再增加连续权重。

### 4.4 四元数更新

采用一阶积分，形式为：

```text
q_dot = 0.5 * q ⊗ [0, omega_corrected]
q_new = q_old + q_dot * dt
q_new = normalize(q_new)
```

展开形式：

```text
q0_dot = 0.5 * (-q1*wx - q2*wy - q3*wz)
q1_dot = 0.5 * ( q0*wx - q3*wy + q2*wz)
q2_dot = 0.5 * ( q3*wx + q0*wy - q1*wz)
q3_dot = 0.5 * (-q2*wx + q1*wy + q0*wz)
```

这里的乘法顺序、误差叉乘方向和输出欧拉角公式必须作为一套整体验证，不能只替换其中一部分。

### 4.5 六轴限制

MPU6050 没有绝对航向参考：

- 加速度计只能约束重力方向；
- Roll/Pitch 可以被拉回；
- Yaw 仍然会因陀螺零偏而漂移；
- `e_z` 在纯重力观测下不能提供真正的航向校正。

所以本方案不承诺绝对 Yaw 锁定。四元数只解决姿态表示和三轴旋转传播问题，不创造新的航向信息。

## 5. 坐标轴和符号方案

### 5.1 当前已确认的控制映射

ApexFly 当前 `IMU_Attitude_MapToBody()` 使用：

```text
机体系 Roll  = 芯片系 pitch_deg
机体系 Pitch = 芯片系 roll_deg
机体系 Yaw   = -芯片系 yaw_deg

机体系 Roll rate  =  gyro_y_dps
机体系 Pitch rate =  gyro_x_dps
机体系 Yaw rate   = -gyro_z_dps
```

这是当前姿态 PID 和混控已经使用的接口约定，不能在算法替换时悄悄改变。

### 5.2 分阶段实施策略

为了避免“更换算法、重新轴变换、重新调 PID”同时发生，采用两阶段方案。

#### V1：芯片系四元数，保留原有输出映射

第一版四元数状态仍在 MPU6050 芯片坐标系中维护：

```text
q_sensor <- gyro_x/y/z 和 accel_x/y/z
```

更新完成后，从四元数得到芯片系 Roll/Pitch/Yaw，再沿用现有：

```text
IMU_Attitude_MapToBody()
```

优点：

- 不改变现有传感器轴映射；
- 不改变姿态 PID 输入的符号；
- 可以直接和当前互补滤波输出逐拍对比；
- 问题定位清晰。

这是一版验证迁移，不是最终坐标架构。

#### V2：统一机体系四元数

V1 经过无桨验证后，再考虑把传感器向量先转换到统一机体系，再维护机体系四元数。

需要特别说明：V1 中“由芯片系四元数导出芯片系欧拉角，再交换 Roll/Pitch 字段”的做法只用于保持旧接口和做小角度影子对比。它不是任意大姿态下严格的三维坐标变换，不能直接作为最终机体系四元数方案。

固定轴变换必须保持为一个合法的三维旋转矩阵，不能只交换欧拉角字段。初始候选映射为：

```text
fusion_x = sensor_y
fusion_y = sensor_x
fusion_z = -sensor_z
```

对应陀螺仪和加速度都必须使用同一矩阵。该候选映射需要通过六面静止和单轴旋转实测确认，确认前不作为固件常量提交。

V2 的优势是姿态内部和控制接口统一，但它不应与第一次算法替换同时引入。

### 5.3 水平零点

`IMU_Level` 当前在四元数输出之后、姿态控制之前减去 Roll/Pitch 平均值。

V1 保持这个边界：

```text
四元数原始姿态
    -> 芯片系到 body 的原有映射
    -> IMU_Level_Apply()
    -> AttitudeControl_Update()
```

不要把水平零点写回四元数，也不要在四元数内部修改零偏后的参考姿态。这样可以保持现有水平校准行为和日志含义。

## 6. 加速度有效性和初始化

### 6.1 加速度有效性

当前互补滤波使用：

```text
0.80g <= |a| <= 1.20g
```

四元数版本保留这项硬门限，后续可升级为软权重：

```text
硬无效：|a| < 0.70g 或 |a| > 1.30g
高可信：|a| 接近 1.00g
过渡区：按偏离 1g 的程度降低 w_accel
```

还必须检查：

- 三轴数据是否为有限值；
- 加速度范数是否大于最小值；
- 归一化结果是否为有限值；
- 当前 `dt` 是否在允许范围内。

### 6.2 初始化

姿态初始化应在收到第一组有效加速度后完成：

```text
Roll/Pitch：由重力方向确定
Yaw：置零或保留上次明确的相对航向
四元数：由初始 Roll/Pitch/Yaw 生成
```

初始化期间不调用姿态 PID，不输出带姿态修正的电机控制。

### 6.3 陀螺仪零偏

当前流程继续保持：

```text
静置采样 -> IMU_GyroBias_Update()
       -> IMU_GyroBias_Apply()
       -> 四元数更新
```

第一版不要同时启用较强的 Mahony `Ki` 自适应零偏：

- 上电零偏和 Mahony 积分同时工作，会增加调试变量；
- 先将 Mahony `Ki` 设为0或很小；
- 验证静态姿态和动态响应后，再按条件启用积分；
- 积分必须有三轴限幅；
- 加速度无效或飞机高动态时不得继续积累积分。

## 7. 采样周期方案

### 7.1 输入接口

四元数更新接口必须接收真实的 `dt_s`：

```c
bool IMU_Attitude_Update(IMU_Attitude_t *attitude, const MPU6050_Data_t *data, float dt_s);
```

V1 可以保留当前函数名，以减少调用面变化；内部结构从角度字段改为四元数字段。

### 7.2 `dt`处理

推荐策略：

```text
0.0005s <= dt <= 0.030s：正常更新
dt 太小：拒绝本拍，保持上一姿态
dt 太大：拒绝本拍并清理微分/积分历史，等待下一组连续样本
```

不建议把长时间任务阻塞简单压成最大值后继续积分，因为那会制造错误旋转量。

当前 `motor_test.c` 已经从 tick 计算 `control_dt_s`。后续实现可以继续使用这个值，但要把“异常 dt 的处理责任”固定在姿态模块内部，避免调用者和算法模块各自限幅。

### 7.3 更新频率

第一版保持当前约100Hz：

- 不同时修改任务周期；
- 不同时替换 MPU6050 总线方式；
- 不同时改变 DLPF 和量程；
- 先比较算法输出，再考虑提高到200Hz。

## 8. 建议的接口设计

### 8.1 内部状态

建议把 `IMU_Attitude_t` 调整为以下逻辑成员：

```c
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float integral_x;
    float integral_y;
    float integral_z;
    float last_accel_norm_g;
    float last_error_x;
    float last_error_y;
    float last_error_z;
    bool initialized;
    bool accel_correction_valid;
} IMU_Attitude_t;
```

具体字段名可以在实现阶段微调，但必须满足：

- 四元数是唯一姿态主状态；
- 不能再把 Roll/Pitch 作为积分主状态；
- 积分误差不能使用静态全局变量；
- 每个姿态对象拥有自己的状态；
- 可以向日志提供加速度有效性和融合误差。

### 8.2 公开输出

第一版继续输出现有结构：

```c
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
} IMU_BodyAttitude_t;
```

这样 `attitude_control.c`、`cascade_pid.c` 和 `mixer.c` 不需要因为姿态内部换成四元数而重写。

### 8.3 建议增加的诊断接口

实现阶段建议增加只读诊断函数：

```c
void IMU_Attitude_GetQuaternion(const IMU_Attitude_t *attitude, float q[4]);
float IMU_Attitude_GetAccelNormG(const IMU_Attitude_t *attitude);
void IMU_Attitude_GetFusionError(const IMU_Attitude_t *attitude, float *x, float *y, float *z);
bool IMU_Attitude_IsAccelCorrectionValid(const IMU_Attitude_t *attitude);
```

这些接口不参与控制决策，只服务于遥测、黑匣子和影子对比。

## 9. 与现有姿态 PID 的连接

连接顺序保持：

```text
MPU6050_ReadData()
    -> IMU_GyroBias_Apply()
    -> IMU_Attitude_Update()
    -> IMU_Attitude_MapToBody()
    -> IMU_Level_Apply()
    -> AttitudeControl_Update()
    -> Mixer_Calculate()
```

不得在这一步修改：

- `AttitudeControl_Update()` 的输入结构；
- `CascadePid_UpdateOuter()` 和 `CascadePid_UpdateInner()` 的顺序；
- `Mixer_Calculate()` 的加减号；
- 现有 Roll/Pitch/Yaw 控制符号；
- 现有 PID 增益。

原因是姿态算法替换与 PID 整定必须解耦。只有确认四元数输出的角度方向、零点、延迟和噪声都正确后，才讨论 PID 参数。

## 10. 与黑匣子的连接

### 10.1 现有日志继续兼容

现有 `BlackboxSample_t` 已记录：

- Roll/Pitch/Yaw；
- 三轴角速度；
- 三轴 PID 修正；
- 基础油门；
- 四路电机；
- 高度和垂速字段；
- 高度状态、解锁、连接状态。

这些字段继续保持原含义，不能把角度字段偷偷改成四元数分量。

### 10.2 四元数影子诊断字段

四元数上线前，建议增加一个新日志 schema 或诊断记录，不破坏旧 CSV：

```text
q0, q1, q2, q3
accel_norm_mg
fusion_error_x_mrad
fusion_error_y_mrad
fusion_error_z_mrad
accel_correction_valid
attitude_algorithm_id
dt_us
```

第一版至少需要记录：

```text
算法版本、dt、加速度模长、加速度修正是否有效、四元数或四元数范数
```

当前样本记录固定64字节且 schema 已被导出器使用，因此不应直接改变旧记录布局。实现时优先选择：

1. 新 schema 并同步更新导出器；或
2. 影子诊断通过串口单独输出；或
3. 使用明确标记的新记录类型。

不能把字段挤进旧布局后继续宣称旧 schema 不变。

### 10.3 黑匣子对比阶段

影子运行时记录：

```text
旧互补 Roll/Pitch/Yaw
新四元数 Roll/Pitch/Yaw
当前控制使用的 Roll/Pitch/Yaw
角速度
加速度模长
dt
融合有效标志
```

控制输入仍使用旧互补输出。只有对比结果稳定后，才切换控制输入到四元数输出。

## 11. 验证顺序

### 阶段 A：数学单元验证

使用离线或桌面测试向量验证：

- 静止水平：四元数保持单位姿态；
- 固定 Roll：输出符号正确；
- 固定 Pitch：输出符号正确；
- 固定 Yaw：Yaw 增量方向正确；
- 三轴组合旋转：四元数范数保持1；
- `dt`变化：相同角速度下总旋转角一致；
- 加速度无效：算法只做陀螺预测，不被异常加速度拉偏；
- 零向量和 NaN：不污染四元数。

### 阶段 B：无桨实机验证

- 六面静止，检查 Roll/Pitch 零点；
- 手动绕三个轴小角度旋转，检查方向；
- 连续旋转后回到水平，检查恢复；
- 静置数分钟，观察姿态漂移；
- 检查串口和黑匣子中的 dt、范数和有效标志。

### 阶段 C：影子运行

- 旧互补算法继续驱动姿态 PID；
- 新四元数算法只记录和遥测；
- 对比静态误差、动态延迟、抖动、恢复和偏航漂移；
- 确认旧接口和混控符号未改变。

### 阶段 D：控制切换前

- 将四元数输出接入姿态 PID，但继续禁止螺旋桨；
- 检查手动倾斜时四路修正方向；
- 检查 PID 输出限幅和混控缩放；
- 检查解锁、上锁、IMU读取失败时状态复位。

未经以上步骤通过，不进行带桨测试。

## 12. 实施边界和明确不做的事情

本次设计不允许顺手做以下改动：

- 不修改高度控制状态机；
- 不替换 SPL06；
- 不引入激光雷达；
- 不修改电机混控加减号；
- 不重调现有姿态 PID；
- 不改变 MPU6050 量程和 DLPF；
- 不改变黑匣子旧 schema 的含义；
- 不直接删除当前互补滤波实现，直到影子对比完成。

## 13. 推荐下一步

下一步可以进入实现，但只做第一阶段：

1. 在 `Modules/IMU` 内增加四元数状态和 Mahony 风格更新；
2. 保留当前 `IMU_BodyAttitude_t` 和 `IMU_Attitude_MapToBody()` 兼容边界；
3. 增加加速度有效性、四元数范数和融合误差诊断；
4. 增加编译期开关或影子调用，使旧算法仍然是控制输入；
5. 先编译验证和无桨运行，再决定是否把控制输入切换到四元数。

当前文档只完成数学和接口设计，尚未修改固件实现。
