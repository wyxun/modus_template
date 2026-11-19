# Feedback 位置与速度反馈接口

Motor 在需要反馈的控制拍调用 `mfoc_feedback_Step()`。调用成功时，Feedback 一次交付电角度、电速度和下一拍的注入请求。**哪个位置源可以发布这份结果，由编译配置决定；这一拍 Core 实际用哪个角度，由 Motor 的运行阶段决定。** [公共头文件](../../motor/include/mfoc_feedback.h) · [框架中的位置](../framework.md#feedback)

## 选择反馈源

`MFOC_FEEDBACK_ENABLE_*` 决定编入哪些位置源。每次调用 Step，Feedback 读取已启用的物理位置，并根据输入推进或复位本拍运行的观测器。Motor 在本次启动前另行确定 HFI 许可。`MFOC_FEEDBACK_BACKEND` 从中指定唯一的角速输出。其他观测器可用于影子观察，主反馈失效时没有自动替补。编译开关会改变实例布局，各编译单元须使用相同配置。[配置定义](../../mfoc_config.h) · [输出选择实现](../../motor/src/mfoc_feedback.c)

| 选中的 backend | 本拍发布条件 | 角速来源 |
| --- | --- | --- |
| `MFOC_FEEDBACK_POSITION` | `mfoc_port_position()` 返回 `OK`，失败则原样返回 Port 状态 | 板级提供者。G431 使用 [Encoder](encoder.md)，霍尔板可接 [Hall](hall.md) |
| `MFOC_FEEDBACK_SMO` | 模型输入齐全，反电势矢量非零，且已有非零的窗口速度 | [SMO 滑模观测器](smo.md) |
| `MFOC_FEEDBACK_FLUX` | 模型输入齐全，校正后的转子磁链矢量非零；允许估算速度为零 | [Flux 非线性磁链观测器](flux.md) |
| `MFOC_FEEDBACK_EKF` | 模型输入齐全且叶子输出 `valid`，仅支持 FLOAT | EKF 后验估计 |
| `MFOC_FEEDBACK_HFI` | 获准注入、有本拍电流且未进入 `FAILED`，获取期间可返回 `OK` | HFI 输出，Motor 同时检查 `acquiring` |
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
| `input.u_ab_previous` | 区间 [k−1, k) 的静止 αβ 模型电压，`V/Vbase` | 借用 Motor 上次成功提交 PWM 后保存的电压。无完整区间时传 NULL，成功提交中性 duty 后传非空零向量 |
| `output.angle`、`output.speed` | 时刻 k 的 BAM32 电角度（一电周为 2³²）与有符号电速度 `ωe/ωbase` | 主反馈提供，Motor 用于角度选择、接管和速度环 |
| `output.u_inject_ab` | 下一区间的静止 αβ 注入电压，`V/Vbase` | HFI 提出请求，Motor 随当前参考送入 Core 限幅与调制；关闭注入或仅运行诊断时为零 |
| `output.acquiring` | 仅在启用 HFI 的构建中有此字段 | HFI 主反馈尚在获取时为真。Motor 使用本拍 HFI 估计角，令 dq 参考为零，冻结速度分频。Core 继续执行两轴电流 PI，在其电压上叠加载波 |

Motor 用本次成功提交的三相 duty 和对应母线快照重建理想平均电压。常规闭环、辨识虚拟角以及辨识电压直出共用这一发布点。下一拍 Feedback 将这份只读电压交给观测器，包括 HFI。调用方从旧接口迁移时，应将 Core 输出指针替换为对应区间的 αβ 电压指针。模型电压未补偿死区和器件压降，台架对比时这两项会计入估算误差。[电压重建](../../control/src/mfoc_modulation.c) · [Motor 单拍顺序](../../motor/src/mfoc_motor.c)

**先看返回码，再读取输出。** `OK` 时上述字段一起发布。失败时调用方的整份 `output` 保留原值，即使某个影子观测器本拍已经更新。这个旧值不能当作本拍反馈。影子 HFI 的原始请求保存在 `hfi_output` 中。只计算诊断时，公共输出的注入向量为零。[HFI 通道用例](../../tests/position/hfi_channel_test.c)

直接调用 Step 时，缺本拍电流或前拍电压会复位 SMO、Flux、EKF 的历史并清零各自缓存的输出。模型系数保留，恢复有效输入后从复位状态重新估算。POSITION 可独立工作；运行中的 HFI 缺电流时复位历史并清空输出。有电流且已获准注入时，可以在没有前拍电压的首拍发出载波。正常的 Motor 路径若读电流帧失败，会在调用 Feedback 前锁存 `SAMPLE` 故障。[缺帧与输出保留用例](../../tests/position/smo_feedback_test.c)

## 实例与调用上下文

| 入口 | 调用时机 | 契约 |
| --- | --- | --- |
| `mfoc_feedback_Init` | 停止态前台，控制中断启用前 | 用配置与 PU 电机模型初始化所有使能观测器。全部接受后才写入目标实例，影子观测器配置有误也会导致初始化失败 |
| `mfoc_feedback_Step` | 控制 ISR，每个需要反馈的控制拍 | 调用方提供已初始化实例与可写输出。输入指针只在本次调用内有效；物理位置从 Port 取得，慢速采样留在前台 |
| `mfoc_feedback_SetMotorParamModel` | 停机后，由调用方排除并发 Step | 在原基值与极对数下，使用保留的配置重建 SMO、Flux、EKF。全部接受才替换并清历史，任一拒绝则保留整组旧状态。作用域是 Feedback，HFI 状态与 Port 的零位可继续使用 |
| `mfoc_feedback_HfiStart` | HFI 启动前，已初始化的停止态实例 | 在 POSITION 或 HFI 主反馈下复位序列并授权注入，其他主反馈或关闭 HFI 时返回 `INVALID_STATE` |

Motor 持有 `mfoc_feedback_t`，板级 Port 持有 Encoder 或 Hall。接入物理传感器时，板级采样或捕获边沿，控制 ISR 的 `mfoc_port_position()` 交付同一时基上的电角度与电速度。接入新的模型观测器时，在 Feedback 内连接初始化、单拍推进、缺帧复位及停机重建，保留同一份公共输出。现有的影子观测结果是实例成员，公共头未提供独立的诊断读取函数。[板级位置契约](../port-contract.md) · [新增后端的框架清单](../framework.md#new-feedback-backend)

## 启动时的 HFI 许可

| 入口 | HFI 运算 | 控制角 | 允许发布注入 |
| --- | --- | --- | --- |
| 普通 `Start`，主反馈为 HFI | 运行 | HFI | 是，保留已选电流或速度模式 |
| 普通 `Start`，其他主反馈 | 默认跳过，`cfg.feedback.observe_hfi=true` 时运行诊断 | 所选主反馈 | 否 |
| 显式 `HfiStart` | 复位后运行 | POSITION 取物理角，HFI 取 HFI 角 | 是，须主反馈成功发布 |
| 固定角对齐、手动强拖 | 关闭 | 对齐角或 OpenLoop 角 | 否 |
| 辨识及其前台交接 | 关闭 | 辨识虚拟角或直接调制 | 否 |

普通启动每次从 Init 设置重新选择许可。显式注入只对当次运行生效。单独使用 Feedback 时，Init 采用表中的普通闭环设置。仅诊断时可读取原始 HFI 结果，其载波解调依赖与实际施加电压匹配的样本。[启停与交接用例](../../tests/motor/hfi_permission_test.c)

<a id="feedback-consumers"></a>

## 编码器和 SMO 的使用方

下表按 G431 已编入 POSITION 和 SMO 的情况列出采样 ISR 的使用方。调用 Feedback 时会读一次 Port 电角度和速度；SMO 有前区间模型电压时推进，无历史时复位。[Feedback 选源](../../motor/src/mfoc_feedback.c) · [G431 位置 Port](../../../peripheral/stm32g431/mfoc/mfoc_port.c)

| 控制阶段 | 编码器角速 | SMO 角速 |
| --- | --- | --- |
| 纯对齐、辨识及辨识交接 | 本拍控制跳过 Feedback。前台仍维护位置；纯对齐正常结束时用保持角标定零位 | 本拍跳过 SMO |
| 手动强拖 | Port 读数仅供观察，控制角来自 OpenLoop | 并行观测，强拖不依赖 SMO 有效性 |
| 无感启动，选 SMO 主反馈 | 编码器不参与接管判定 | RAMP 积累观测；ACQUIRE 用角差和速度差判定捕获。捕获后用于角度混合、速度闭环和稳定判定 |
| POSITION 闭环，含显式 HFI 注入 | 角度送 Core，速度供到期的速度 PI | 并行观测，可用作诊断对照 |
| SMO 闭环 | 读数供诊断对照；SMO 失效时 Motor 走反馈故障路径 | 有效角度送 Core，速度供到期的速度 PI |
| Flux、EKF 或 HFI 为主反馈 | 编码器不参与本拍控制 | 并行观测；所选主反馈提供角速。HFI 获取时冻结速度环 |

G431 的 FLOAT 角度波形使用本次 ADC 中断中 Feedback 读取的物理编码器角和有效状态。角度对应调用 `mfoc_port_position` 时的时钟，ADC 硬件触发到读取之间仍有延迟。PWM 写入后即使编码器有了新样本或刚过期，本拍波形也沿用这次读数。选 SMO 控制时，Motor 继续用 SMO 角，波形分别显示编码器角、观测角，以及两者都有效时的角差。纯对齐或辨识跳过 Feedback，接受波形样本时才单独读一次编码器，观测角和角差标为无效。没有就绪电流帧的中断不产生角度波形样本。[波形采集](../../../class/mfoc_debug.c) · [G431 开销记录](../../../docs/plans/2026-11-10-001-refactor-mfoc-library-v1-plan.md#p10-g431-position-waveform)

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
