# 扩展 MFOC

## 接入板级能力

板级实现静态 Port 绑定。ADC 帧的读取和应答、PWM 比较值生效点、单调时钟及硬件 Break 都由目标层负责。库接收一致的物理量和失败状态。[Port 契约](port-contract.md)

霍尔或编码器提供者由 Port 返回 BAM32 电角度和基于同一电频率基值的速度 PU。移植时用已知相序、正反转、计时器回绕和传感器超时检查输出，再核对整条 ISR 截止余量。[Hall 模块](modules/hall.md) · [Encoder 模块](modules/encoder.md)

## 增加反馈算法

算法实例保留自己的状态。初始化阶段处理参数和系数，采样阶段通过模块的 `Step` 消费本拍电流和上一个实际生效区间的模型电压。将输出形式与现有 Feedback 的电角度、速度、有效位及初始化/复位语义对齐。确定编译开关和主反馈选择，再在同一数值后端回放失效、恢复及换向。[Feedback 模块](modules/feedback.md) · [框架与数据流](framework.md)

发布前量化普通拍与最重分支的周期、栈和 Flash 增量。仅用影子观测采集结果时，分别记录其诊断配置与实际闭环配置。[发布支持矩阵](support-matrix.md)

## 迁移旧调用方

应用初始化后由 Motor 负责控制权。需要启动辨识或读取结果时，分别改用 `mfoc_motor_StartIdentification` 和 `mfoc_motor_GetIdentificationResult`。独立算法实验可直接使用辨识模块公开接口。关断失败时检查 Motor 故障状态，已完成的测量结果仍可单独查询。[Motor 公共入口](../motor/include/mfoc_motor.h)

需要来源于电机 profile 的应用包含 `mfoc_motor_config.h`。数学和控制入口使用 `mfoc.h`，Motor 调用者包含 `mfoc_motor.h`，板级实现包含 `mfoc_port.h`。更换 FLOAT/FIXED、反馈组合或辨识选项时，重新编译所有跨模块调用方和库。

使用 perfc 的工作区可保留 `MFOC_USE_PERFC_FSM=1`。新接入且无 perfc 依赖的应用使用默认 C11 PT。
