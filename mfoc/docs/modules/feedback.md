# Feedback 位置与速度反馈接口

Motor 在需要反馈的控制拍调用 `mfoc_feedback_Step()`。调用成功时，Feedback 一次交付电角度、电速度和下一拍的注入请求。**哪个位置源可以发布这份结果，由编译配置决定；这一拍 Core 实际用哪个角度，由 Motor 的运行阶段决定。** [公共头文件](../../motor/include/mfoc_feedback.h) · [框架中的位置](../framework.md#feedback)

## 选择反馈源

`MFOC_FEEDBACK_ENABLE_*` 决定编入哪些位置源。每次调用 Step，Feedback 读取已启用的物理位置，并根据输入推进或复位已启用的观测器。`MFOC_FEEDBACK_BACKEND` 从中指定唯一的角速输出。其他观测器可用于影子观察，主反馈失效时没有自动替补。编译开关会改变实例布局，各编译单元须使用相同配置。[配置定义](../../mfoc_config.h) · [输出选择实现](../../motor/src/mfoc_feedback.c)

| 选中的 backend | 本拍发布条件 | 角速来源 |
| --- | --- | --- |
| `MFOC_FEEDBACK_POSITION` | `mfoc_port_position()` 返回 `OK`，失败则原样返回 Port 状态 | 板级提供者。G431 使用 [Encoder](encoder.md)，霍尔板可接 [Hall](hall.md) |
| `MFOC_FEEDBACK_SMO` | 模型输入齐全，反电势矢量非零，且已有非零的窗口速度 | [SMO 滑模观测器](smo.md) |
| `MFOC_FEEDBACK_FLUX` | 模型输入齐全，校正后的转子磁链矢量非零；允许估算速度为零 | [Flux 非线性磁链观测器](flux.md) |
| `MFOC_FEEDBACK_EKF` | 模型输入齐全且叶子输出 `valid`，仅支持 FLOAT | EKF 后验估计 |
| `MFOC_FEEDBACK_HFI` | 有本拍电流且未进入 `FAILED`；获取位置期间也可返回 `OK` | HFI 输出，Motor 同时检查 `acquiring` |
| `MFOC_FEEDBACK_NONE` | 始终返回 `MFOC_RESULT_INVALID_STATE` | 无公共角速，可在强拖时运行影子算法 |

例如，要在编码器闭环下评估 Flux，启用 POSITION 与 FLUX，将 backend 设为 POSITION。即使启动首拍没有历史电压，编码器仍可给出本拍角速。Flux 随后在后台用相同的电流帧更新，诊断可读取 `motor.feedback.flux_output`。若反过来选 FLUX，编码器读数再新也不能代替无效的 Flux 输出。当前默认构建启用 POSITION 和 SMO，主反馈选 SMO。[反馈接线用例](../../tests/position/flux_test.c)

## 一拍的输入与发布

```c
mfoc_result_t mfoc_feedback_Step(mfoc_feedback_t *feedback,
    const mfoc_feedback_input_t *input, mfoc_feedback_output_t *output);
```

| 字段 | 单位与时刻 | 生产者与消费者 |
| --- | --- | --- |
| `input.i_ab_k` | 静止 αβ 电流，`I/Ibase`，采样时刻 k | Motor 读取新鲜三相帧，Clarke 变换后借给 Feedback |
| `input.previous_core_output` | 区间 [k−1, k) 的三相 duty 和当时的母线快照 `Vdc/Vbase` | Motor 在 PWM 提交成功后保存。首拍、Stop/Trip 后以及上一拍辨识电压直出时无此历史 |
| `output.angle`、`output.speed` | 时刻 k 的 BAM32 电角度（一电周为 2³²）与有符号电速度 `ωe/ωbase` | 主反馈提供，Motor 用于角度选择、接管和速度环 |
| `output.u_inject_ab` | 下一区间的静止 αβ 注入电压，`V/Vbase` | HFI 提出请求，Motor 随当前参考送入 Core 限幅与调制；未启用时为零 |
| `output.acquiring` | 仅在启用 HFI 的构建中有此字段 | 选 HFI 且正在获取位置时为真；常规控制路径令基波 dq 电流参考为零，暂停速度环，保留注入 |

模型观测器使用上一拍**已提交的占空比**重建理想平均电压。Feedback 调用 `mfoc_modulation_ModelVoltageAlphaBeta(&previous->duty, previous->bus, &voltage)`，这同时固定了限幅后的占空比和该区间的电压标度。本拍的电压指令要等 Core 算完才产生。模型电压未补偿死区和器件压降，台架对比时这两项会计入估算误差。[电压重建](../../control/src/mfoc_modulation.c) · [Motor 单拍顺序](../../motor/src/mfoc_motor.c)

**先看返回码，再读取输出。** `OK` 时上述字段一起发布。失败时调用方的整份 `output` 保留原值，即使某个影子观测器本拍已经更新。这个旧值不能当作本拍反馈。HFI 即使作为影子也会计算注入，只有主反馈成功发布时该请求才能进入 Core。[HFI 通道用例](../../tests/position/hfi_channel_test.c)

直接调用 Step 时，缺本拍电流或前拍电压会复位 SMO、Flux、EKF 的历史并清零各自缓存的输出。模型系数保留，恢复有效输入后从复位状态重新估算。POSITION 可独立工作；HFI 有电流就能以空的前拍电压开始注入，缺电流则重新初始化。正常的 Motor 路径若读电流帧失败，会在调用 Feedback 前锁存 `SAMPLE` 故障。[缺帧与输出保留用例](../../tests/position/smo_feedback_test.c)

## 实例与调用上下文

| 入口 | 调用时机 | 契约 |
| --- | --- | --- |
| `mfoc_feedback_Init` | 停止态前台，控制中断启用前 | 用配置与 PU 电机模型初始化所有使能观测器。全部接受后才写入目标实例，影子观测器配置有误也会导致初始化失败 |
| `mfoc_feedback_Step` | 控制 ISR，每个需要反馈的控制拍 | 调用方提供已初始化实例与可写输出。输入指针只在本次调用内有效；物理位置从 Port 取得，慢速采样留在前台 |
| `mfoc_feedback_SetMotorParamModel` | 停机后，由调用方排除并发 Step | 在原基值与极对数下，使用保留的配置重建 SMO、Flux、EKF。全部接受才替换并清历史，任一拒绝则保留整组旧状态。作用域是 Feedback，HFI 状态与 Port 的零位可继续使用 |
| `mfoc_feedback_HfiStart` | HFI 启动前，已初始化的停止态实例 | 复位 HFI 序列与缓存输出，关闭 HFI 的构建返回 `INVALID_STATE` |

Motor 持有 `mfoc_feedback_t`，板级 Port 持有 Encoder 或 Hall。接入物理传感器时，板级采样或捕获边沿，控制 ISR 的 `mfoc_port_position()` 交付同一时基上的电角度与电速度。接入新的模型观测器时，在 Feedback 内连接初始化、单拍推进、缺帧复位及停机重建，保留同一份公共输出。现有的影子观测结果是实例成员，公共头未提供独立的诊断读取函数。[板级位置契约](../port-contract.md) · [新增后端的框架清单](../framework.md#new-feedback-backend)

<a id="handover"></a>

## Motor 何时采用反馈角

纯固定角对齐和辨识期间，Motor 使用各自的角度或调制量，跳过 Feedback。强拖期间仍运行 Feedback，控制角取自 OpenLoop。无感启动则让选定观测器随强拖积累历史，由 Motor 完成以下接管。该入口支持 SMO、Flux 和 FLOAT 下的 EKF，要求关闭 HFI。[Motor 启动入口](../../motor/src/mfoc_motor.c)

1. **RAMP。** 保持结束后按 OpenLoop 角加速，到目标速度才进入 ACQUIRE 并开始接管超时计时。
2. **ACQUIRE，捕获前。** 观测器须返回 `OK`，强拖角与反馈角的最短角差须在捕获角窗内，且速度差不超过 `abs(speed_ref) × MFOC_START_SPEED_WINDOW`。例如当前配置下，目标为 0.5 PU 时，速度误差最多为 0.05 PU，角差最多为 10°。未满足时继续使用强拖角，直至捕获或超时。
3. **捕获后。** 记录强拖角相对反馈角的偏移，逐拍将偏移缩至零。偏移归零后 Motor 使速度 PI 跟踪当前 q 轴参考并进入速度模式。速度在窗口内连续满足稳定时长后，退出启动状态。捕获后再丢反馈、接管超时或常规闭环丢反馈，都走 `MFOC_MOTOR_FAULT_FEEDBACK` 关断路径。[接管状态机](../../motor/src/mfoc_sensorless_start.c) · [窗口与时限配置](../../mfoc_motor_config.h)

## 主机验证

从仓库根目录运行以下 PowerShell 命令。前两项通过真实观测器与 Feedback 检查选源、缺帧复位和模型更新。后两项向 Motor 注入可控的反馈角速，分别以 SMO（2）和 Flux（4）的配置检查正反转捕获、偏移消除和超时关断。两类用例都覆盖 FLOAT 和 FIXED；电机和采样链的实际误差需在台架上测量。[接管测试](../../tests/motor/motor_handover_test.c)

```powershell
.\mfoc\tests\scripts\run_smo_tests.ps1
.\mfoc\tests\scripts\run_flux_tests.ps1
.\mfoc\tests\scripts\run_core_tests.ps1 -Tests motor_handover -Hfi 0 -Backend 2
.\mfoc\tests\scripts\run_core_tests.ps1 -Tests motor_handover -Hfi 0 -Backend 4
```
