# FOC 当前架构与模块边界

更新：2026-10-15。本文对照当前工作区源码，描述已实现的对象、数据契约和执行顺序。设计约束详见 [模块架构参考第 7 节](../../.agents/skills/foc-motor-control-methodology/references/foc-module-architecture.md)，数值约定详见 [标幺与离散时间参考第 8 节](../../.agents/skills/foc-motor-control-methodology/references/per-unit-fixed-point-and-discrete-time.md)。日期归档记录当次变化，本文件随实现更新。

## 1. 对象所有权与职责

`foc_app_t` 是 MODUS 组合入口，按值拥有 Motor、Position、Encoder 和 Identify。Position 直接按值拥有当前唯一算法 SMO 的状态及候选快照；Motor 拥有 Core、速度 PI 和 Startup 轨迹。可变状态属于各自实例，纯数据输入输出允许按值或 const 指针传递。

| 模块 / 源码 | 拥有的状态和职责 | 对外边界 |
|---|---|---|
| [App](../app/foc_app.c) | 配置绑定、前台调度、ISR 编排、辨识样本组装和请求应用 | 连接模块的类型明确接口 |
| [Motor](../motor/motor.c) | 生命周期、ADC 零偏/归一化、Clarke、速度环、Core、PWM 提交、ALIGN、安全输出 | 接收最终电角反馈，提供采样快照和控制结果 |
| [Startup](../motor/motor_startup.h) | Motor 拥有的强制电角轨迹、速度斜坡和步进历史 | 产生强拖候选；不判断观测器接管 |
| [Position](../motor/motor_position.h) | 机械角转电角、零位补偿、速度换算、候选快照、SMO 资格及交接进度 | 发布唯一电角反馈和接管/丢失事件 |
| [Encoder](../observer/foc_encoder.h) | 前台读取机械位置、机械速度滤波、带时间戳的缓存发布 | ISR 读取机械位置快照，不在 ISR 进行传感器总线读取 |
| [SMO](../observer/foc_smo.h) | PU 离散系数、估计电流、滑模/反电势历史、角差速度 | 自己的 input/cfg/state/output 类型，无父对象依赖 |
| [Core](../middleware/foc_core.h) | 电流 PI、Park/逆 Park、SVPWM 及限幅标志计算 | 数学输入输出；PWM 硬件提交归 Motor |
| [Identify](../identify/identify.h) | Rs/Ld 前台 PT、各自 ISR 采样状态、PU 结果 | ISR 接收独立样本并返回请求；前台用 Motor API 启停 |
| [端口](../hal/foc_port.h) / 目标适配 | 原始电流/母线 ADC、PWM、安全停机及传感器硬件绑定 | 原始数值和占空比；按目标编译期绑定 |

Motor 的 `tOutput` 是调用者拥有的 Core 输出数据，可以用于组装样本；Core 的 PI 状态不作为外部控制依据。算法状态在头文件可见是 C 按值组合的需要，不代表允许外部业务直接读写。隔离的波形诊断和白盒测试可观测内部状态，但不得据此驱动正式控制或安全迁移。

## 2. 有感、无感和强拖的位置边界

```mermaid
flowchart LR
    Enc["Encoder 机械位置快照"] --> Pos["Position：转换、选择、补偿、交接"]
    Start["Motor / Startup 强拖候选"] --> Pos
    Motor["Motor：电流与上一控制区间电压快照"] --> Pos
    Pos -->|独立算法输入| SMO["SMO 独立估算"]
    SMO -->|候选结果与专属证据| Pos
    Pos -->|唯一电角反馈与事件| Motor
    Motor --> Core["Core：电流环与调制"]
    Core -->|控制输出与限幅标志| Motor
    Motor --> Port["端口：PWM 提交 / 安全输出"]
```

图中 Motor→Position 数据由 App 接线，Position 直接调用 SMO；已删除只有转发职责的 `foc_observer.c/h`。传感器和强拖都属于 Position 管理的位置来源。强拖产生开环轨迹，SMO 根据电气输入估算位置；两者各自提供候选。

| 路径 | 当前反馈与策略 |
|---|---|
| SENSOR，有感 | Encoder 缓存 → Position 机械/电角转换与零位补偿 → Motor；可同时旁路运行 SMO |
| HARD_DRAG，纯强拖 | Position 采用 Motor/Startup 候选；关闭 SMO 时也能使用已有强拖路径，配置必须禁用接管 |
| HARD_DRAG，旁路观测 SMO | SMO 持续产出候选，关闭接管后反馈仍由强拖产生；SMO 无效不触发观测器接管丢失事件 |
| HARD_DRAG，强拖切 SMO | Position 资格计数 → 有界角度修正及速度融合 → SMO；丢失时报告事件，由 Motor 安全停机 |
| HFI+SMO | 扩展设计，尚未实现；需算法、注入、同步采样和 Position 双向切换共同落实 |

`motor_electrical_feedback_t` 含 BAM32 电角度、电角速度 PU、有效性及本拍瞬时 `eEvent`。`motor_IsrControlStep()` 在内部消费接管/丢失事件，再推进速度环与控制；Position 不锁存待取事件，也不提供 `TakeEvent` API。Motor/Core 不依据来源枚举选择反馈。当前 SMO 接管策略只支持 HARD_DRAG 到 SMO，不能解释为已支持传感器到无感切换或任意来源仲裁。

## 3. 输入、输出与数值边界

2026-10-11 配置边界收口：Rs 的激励、有效电流范围、采样数/频率及等待时限集中在 `app/motor_config.h`，由 Identify 在编译期直接使用；两档差分的算法结构仍由 Identify 固定。不新增 Rs 配置对象，各实例共用本次构建的宏配置。PI 增益表示、传递和换算保持原状。

Encoder 的静态 `FOC_ENCODER_PORT_READ` 和动态 `fnRead` 均输出 `foc_angle_t` 机械 BAM32 角度。原始计数、掩码、分辨率换算归传感器适配层；G431 在端口中将 AS5600 的 12 位计数转换为 BAM32。Encoder 只计算跨零位移、机械速度、方向和缓存时效；相邻采样的实际位移必须小于半圈。速度差分与滤波复用上一份已发布快照的角度、速度和采样完成 tick，不另存重复的上一角度/tick。自定义传感器适配须同步采用这个输出契约。

当前 SMO 只支持 `Ld == Lq`。其余参数有效但 `Ld != Lq` 时，初始化返回 `FOC_RESULT_DISABLED`，SMO 保持未初始化且不发布有效估计；不要求接管的 SENSOR/HARD_DRAG 可继续初始化。要求 SMO 接管时，该禁用结果仍使 App 初始化失败。非法参数和数值范围错误不作旁路豁免；此修改不增加凸极 SMO 能力。

此前验证记录：FLOAT/FIXED 编码器测试覆盖 12/14/20 位输入、跨零速度及方向反转；真实 App 组合回归覆盖静态/动态绑定、SENSOR/HARD_DRAG、NONE/SMO 和等轴/凸极模型，并检查必须接管时拒绝不支持模型。Rs/Ld、SMO、Observer、接管、命令及模块边界回归通过。G431 debug-rel FLOAT 与 release FIXED 构建通过；未烧录或进行真机验证。

| 边界 | 当前数据契约 | 单位 / 生命周期 |
|---|---|---|
| App → Motor 初始化 | 绑定后的 `motor_cfg_t` | 时间配置使用秒/Hz；App 内嵌 Motor 配置始终采用 PU。可选物理速度 profile 仅在 Init 绑定一次 |
| Position → SMO 初始化 | `foc_electrical_model_t`、`foc_smo_cfg_t`（配置成员 `tSmoCfg`） | Rs/Ld/Lq、Vbase/Ibase 等物理量仅用于 Init；生成执行系数，不保留借用指针 |
| Motor → Position | `motor_position_sample_t` | 当前电流 αβ、上一控制区间电压模型、强拖候选、运行代次与限幅信息 |
| Position → SMO | `foc_smo_input_t` | 借用电流与模型电压指针，仅在本次 Step 内有效 |
| SMO → Position | 扁平 `foc_smo_output_t`，保存于 Position 的 `tEstimate` | 直接包含角度、速度 PU、有效性及 SMO 自有证据 `qBemfStrengthPu`，无通用结果包装 |
| Position → Motor | `motor_electrical_feedback_t`（含 `motor_position_event_t eEvent`） | 本拍唯一反馈；瞬时事件在 `motor_IsrControlStep()` 内转换为运行模式或故障 |
| Motor → Core | `foc_core_command_t`、`foc_core_input_t` | 模式、参考、当前 αβ 电流、有效电角度、速度 PU |
| Core → Motor | `foc_core_output_t` | dq 电流、调制度命令、三相占空比、电压/PWM 边界标志 |
| App → Identify | Rs/Ld 各自的 sample 类型 | 电流 αβ PU、实际母线重构的模型 α 轴电压 PU、有效性、上一周期 PWM 饱和；不接收角度/速度 |
| Identify → App | `identify_isr_request_t` | 无量纲调制度命令或中止请求；调制度不是实际电压 PU |
| Identify → 前台消费者 | `identify_result_t` | `identify_Run()` 一次性发布带算法标签的 PU 结果；内部保留最终值 |

Core 输入保留 `tCurrentAlphaBeta`，输出只保留变换后的 dq 电流，不再复制 αβ 输入。Motor 原始 ADC 样本仅作本次采样局部量，运行时有效 Iq 上限统一保存在 `tLimits.qMaxIq`，不另存 raw 缓存或有效上限副本。

统一基值为电流 `Ib`、电压 `Vb` 和电频率 `fb`。机械速度由 Position 用极对数及 `fb` 转为电角速度 PU；算法输出角度沿用 BAM32。辨识采用 `Rb = Vb/Ib`、`Lb = Rb/(2πfb)`、`Ts_pu = 2πfb/fs`。系数可在 Init 代数合并推导，不要求为了形式额外保存一套全部 PU 参数。

FLOAT 和 FIXED 表达相同单位。Startup 的 Q16 步进余数、Q31 速度累加属于内部数值存储格式，输出恢复公共角度和速度类型；格式缩放不能混同物理单位换算。

Motor 在普通运行的 Prepare 内采集并归一化母线，用上一份提交占空比经 `foc_modulation_ModelVoltageAlphaBeta()` 重构理想 αβ 电压 PU。固定母线的 R/L 辨识在 `motor_StartIdentification()` 启动 PWM 前验证并锁存一次母线 PU，整次 ALIGN/注入会话保持该值，不依赖 ADC 均值更新窗口；Stop/Fault 清除缓存，下次辨识重新读取。此流程假定辨识期间供电稳定，不提供逐拍母线变化跟踪。SMO 和 Identify 消费 Motor 的共享 `u[k-1]` 快照，App 不再保存占空比或另采母线、重构模型。样本同时发布 `bVoltageModelValid` 和上一周期 `bPwmSaturated`；模型无效时观测历史与候选失效，辨识按自身采集阶段中止。纯强拖和有感控制不以模型有效性替代位置反馈有效性。

Core 的 `tVoltage/tVoltageAlphaBeta` 保留现行调制度命令语义，不是实测电压 PU；保留原来的 PI 单轴输出限制和 SVPWM 占空比裁剪。理想重构包含占空比裁剪，普通运行跟踪实测母线变化，固定母线辨识使用启动快照；不包含死区、器件压降或 PWM 实际生效点的硬件验证。电流归一化保留超过 1 pu 的可表达值，Clarke/Park 使用宽乘法；数值不可表达时锁存采样故障。电流基值、参考上限和硬件过流保护是不同边界。

`bVoltageLimited` 仅报告 PI 输出边界或调制度幅值判据；`bPwmSaturated` 表示调制达到占空比边界。Core 保留原来的控制路径：不做径向缩放，不把 PWM 裁剪反馈给两轴 PI。此前新增的径向限幅、快速路径和下游抗饱和状态已按用户要求撤回。HFI 等电压预算扩展须另行验证。

## 4. RUN 高频执行顺序

以下时序适用于 Prepare 返回 `MOTOR_ISR_CONTROL_READY` 的普通 RUN 分支，或 `MOTOR_ISR_IDENTIFICATION_READY` 的固定轴辨识分支。ALIGN、零偏校准和空闲状态由 Prepare 分支处理，不能都套用同一条 Core 调用链。

```mermaid
sequenceDiagram
    participant App as App HF ISR
    participant Motor as Motor
    participant Pos as Position
    participant Obs as SMO
    participant Core as Core
    participant Port as PWM port
    participant Id as Identify

    App->>Motor: motor_IsrPrepare(sample)
    Motor-->>App: i[k], 占空比/母线重构 u[k-1]、模型有效性、候选与代次
    alt MOTOR_ISR_CONTROL_READY
        App->>Pos: FOC_POSITION_GET(sample, feedback)
        Note over Pos: 采用传感器缓存、强拖或已发布的估计候选
        Pos-->>App: 最终反馈（含本拍 eEvent）
        App->>Motor: motor_IsrControlStep(feedback)
        Note over Motor: 内部消费 eEvent，接管或失效停机
    else MOTOR_ISR_IDENTIFICATION_READY
        App->>Motor: motor_IsrControlStep(NULL)，固定角度零
    end
    Motor->>Motor: 仅速度模式按分频推进速度 PI
    Motor->>Core: foc_core_step(command, input, output)
    Core-->>Motor: 电流、调制度命令、占空比与边界标志
    Motor->>Port: 提交本拍占空比
    Motor-->>App: eControl
    opt 普通运行、启用 SMO，位置与控制均成功
        App->>Pos: motor_position_ObserverStep(sample)
        Pos->>Obs: foc_smo_Step(input, output)
        Obs-->>Pos: 候选快照，供下一 ISR 使用
    end
    opt Identify 存在活动算法
        App->>App: 从 Prepare 快照组装独立算法样本
        App->>Id: 对应算法 IsrStep(sample, request)
        Id-->>App: 调制度请求或中止请求
        App->>Motor: 同 ISR 应用请求；中止优先
    end
```

关键拍关系：本拍 `i[k]` 对应前一控制区间电压，不能用刚计算的新电压配给已经采完的电流。SMO 在新控制计算之后运行，但输入仍来自 Prepare 保存的旧电压；候选供下一 ISR 选择，当前 SMO 策略还有一拍角度预测补偿。

Prepare/Control 配对守卫防止 RUN 中重复执行或漏配对。运行代次在重启时使 Position 清候选和观测历史。位置错误经无效反馈或丢失事件使 Motor 锁存位置故障；控制或 PWM 提交错误由 Motor 进入安全状态。

Identify 样本的 `bValid` 要求本拍固定轴 Core/PWM 控制成功、母线/模型有效。上一周期 PWM 饱和另作独立标志。`motor_IdentificationApplyIsr()` 当前只更新调制度参考，下一次控制步骤才计算并提交对应 PWM；“同 ISR 应用请求”不表示本拍再次运行 Core 或再次提交占空比。中止请求则在当前 ISR 内安全停止；请求应用失败同时锁存 Identify 错误并停止 Motor。

## 5. 初始化、前台与 ALIGN

App 绑定电流/电压/速度基值、控制周期及速度增益，再初始化 Motor、所选位置路径和 Identify。Encoder 仅在 SENSOR 路径初始化；HARD_DRAG 不依赖它。无感斜坡配置只允许配 HARD_DRAG；既无定频强拖又无斜坡的 HARD_DRAG 配置会被拒绝。

前台 PT 约每 1 ms 服务一次：检查 break 故障、处理调试电流阶跃、独立更新启用 Encoder 的缓存并推进 Identify；Encoder 失败不跳过 Identify，也不按辨识类型停止更新。速度 PI 属于 RUN 高频路径的分频步骤，当前配置为 1 kHz，不在前台 PT 中执行。

启用日志时，App ISR 从入口的 perf_counter tick 到波形步骤结束累计耗时；包含本次实际执行的校准/控制/观测/辨识及波形分支，不包含计时累加本身和外层中断处理。Run 每秒原子取走并清零累计值，以 T 类日志打印均值（0.1 μs 分辨率）。只要本窗口任一次耗时超过 `1 / FOC_HF_ISR_HZ`，改为打印本窗口最大值并标注 overrun；20 kHz 的预算为 50 μs。无样本时不打印，关闭 T 掩码仍每秒清零；`__NO_USE_LOG__` 构建移除统计状态和计时尾部。记录的是观测窗口数据，不等同最坏执行时间证明。

ALIGN 用固定 0 电角度和 D 轴电流励磁，Motor 在 Prepare 的 ALIGN 分支内部完成采样、Core 与 PWM，不需要编码器反馈。`motor align` 只执行转子定位，原编码器校零改用 `motor calibrate`，仅用于 SENSOR。重新 ALIGN 时旧零位有效标记先失效；保持结束分为四种完成动作：

两种手动命令使用同一个 `motor_RequestAlignment(motor, bCaptureZero)` 实现，分别传 false/true。Position 校零直接读取既有的机械位置快照并保存电零位，Encoder 不再提供重复的校零读取入口。

| 完成动作 | 后处理 |
|---|---|
| 独立转子定位 | Motor 关闭 PWM 并回到 IDLE；不请求 Position、不采集零位，不把转子定位标记成编码器校准成功 |
| 有感零位校准 | Prepare 返回 `MOTOR_ISR_CAPTURE_ZERO`；App 请求 Position 捕获机械零位，然后 Motor 完成 ALIGN、关闭 PWM 并回到 IDLE；捕获失败进入故障 |
| 无感启动 | Motor 保持 PWM，换为启动 Iq 参考并进入 RUN；后续 Startup 产生强拖轨迹，不要求捕获编码器零位 |
| Rs/Ld 辨识 | Motor 保持 PWM，切到固定零角度电压模式，初始调制度为零；不捕获零位，不推进 Startup、Position 或 SMO |

无感启动的接管事件使 Motor 跟踪速度 PI 初值、切到速度模式并停止强制轨迹；未在配置的最大启动周期内完成接管会停机。纯强拖实验与完整无感启动流程的退出策略需要区分。

HARD_DRAG 下 `motor speed <pu>` 调用 I/f 启动入口；未配置斜坡时拒绝命令。当前台架 profile 已切为 SENSOR，斜坡关闭：先执行 `motor calibrate` 捕获编码器电角零位，再通过 `motor speed <pu>` 进行编码器速度控制。SMO 持续影子估算，不接管控制。速度基值 400 电 Hz，`motor speed 0.1` 对应 40 电 Hz、7 极对下约 343 rpm 的目标。

## 6. Identify：内部 PU 与独立算法

Rs 和 Ld 分别拥有固定样本类型、ISR 入口和实例状态，新增磁链等辨识应增加自己的输入类型与入口。公共父层负责流程选择和最终结果，不能把新算法字段追加到既有 Rs/Ld 样本中，也不能改变其 α 轴电压的含义。

- Rs 对采集的电流与重构模型电压取平均，用两档差分得到 `r_pu`。激励宏只定义调制度，不用宏定义的 12000 mV 代替实际模型电压。
- Ld 使用 `r_pu`、PU 电压、电流变化和采样时间计算 `l_pu`；计划、ISR 采集、前台流程和结果分组保存在所属实例。
- `identify_Run()` 通过 `identify_result_t` 发布 PU 结果；`identify_ResultToPhysical()` 仅供前台显示转换为 mΩ/µH。实例保存显示尺度，不在 ISR 中转换物理单位。
- Rs 成功结果更新 Identify 内后续 Ld 使用的电阻 PU；没有自动改写运行中 SMO 或电流 PI 系数，模型更新应在明确的安全点重新推导系数。

ISR 子算法不接收 Motor 指针，只产出请求；前台 PT 仍通过 Motor API 查询状态、启停控制。Rs/Ld 的 `identify_Run()` 共用固定矢量准备 PT，通过 `motor_StartIdentification()` 自动对齐，超时 5 s 收敛到 ERROR 并停止 PWM。命令只要求 Motor 空闲，不要求电角零位；子算法在对齐成功后才开始采集。普通运行的无效位置反馈拒绝行为保留，后续磁链等旋转辨识采用自己的准备分支。

Ld 横轴电流阈值用于发现持续扰动，不能证明静止；采样期间转子必须保持静止，凸极电机需保持对齐，结果解释为沿该轴的增量电感。参考提交到相应电流样本至少相隔两拍，捕获延迟须不小于 2 + 母线采样延迟拍。

低频辨识日志以 `IDENTIFY_LOG_ENABLE` 控制，过程信息在模块前台消费，完成结果由 App 使用带标签结果显示。高频连续变量由波形观测；不为打印把过程数组重新加入对外结果结构。

## 7. 当前配置与可用能力

以下是 [motor_config.h](../app/motor_config.h) 当前值，属于配置快照而非框架默认承诺；目标构建覆盖以 [target.mk](../../target/stm32g431/target.mk) 和端口配置为准。

| 项目 | 当前配置 / 含义 |
|---|---|
| 位置来源 | SENSOR；编码器闭环，SMO 影子观测 |
| 定频强拖 | 20000 mHz 备用参数；当前 SENSOR 路径不使用 |
| 启动斜坡 | 0 拍，关闭无感启动；speed 使用编码器速度反馈 |
| SMO 接管策略 | `OBSERVER_TAKEOVER=false`，`AUTO_TAKEOVER=false`；可旁路估算，接管关闭 |
| 电流 / 电压 / 速度基值 | 3.5 A / 12 V / 400 电 Hz；G431 注入 ADC 保留 4×累加，标度为 476 sum counts/base（原 12 位格式为 119） |
| 执行频率 | G431 HF 20 kHz；速度环 1 kHz；前台服务约 1 kHz |
| 数值实现 | FLOAT / FIXED；同一 PU 和时序契约 |

`FOC_OBSERVER_BACKEND` 保留原构建参数名，按算法位集合组织。目前可运行 NONE 或 SMO；HFI 位以及 `FOC_ENABLE_HFI=1` 明确编译拒绝。启用 SMO 算法、启用接管策略、允许自动接管是三个独立选择，不能只改算法开关就认为电机进入无感闭环。

未实现能力包括 HFI 注入与同步采样、HFI+SMO 双向切换、Flux/EKF 后端、任意来源仲裁以及旋转磁链辨识流程。Core 当前直接调用 SVPWM，其他调制函数的存在不表示已经有运行时调制策略选择器。

当前 waveform 默认四路（流输出统一乘 1000）：`Enc_mT` 为本拍控制使用的编码器电角度，`SMO_mT` 为本拍更新后的原始估计电角度，`Err_mT` 为环绕处理的 SMO 减编码器角差，`Bemf_mpu` 为 `(abs(e_alpha)+abs(e_beta))/Vbase`。角度范围 0～1000 mT，误差范围约 ±500 mT；无效值输出 -1000。三路估计量取自同一次 SMO 输出；编码器参考为同一 ISR 的控制输入快照。编码器读取/外推与 ADC/PWM 的物理时间对齐仍待台架验证，原始角差不能直接等同于滤波器相移。

## 8. HFI+SMO 扩展约束

当前只有 Position 直接调用 SMO，不为未来算法保留嵌套 Observer 包装层。[Position](../motor/motor_position.h) 和 [Core](../middleware/foc_core.h) 的扩展注释与以下约束用于后续实际实现：

1. 新增 HFI/Flux/EKF 时先确定实际输入、输出及消费者；确需多算法模块时，将其放在 ISR 编排层，与 Position、Motor 同级接线，不重新嵌套在 Position 内。各算法独立拥有 cfg/input/output/state，不用 union 共用并发状态，也不预建通用结果或未使用字段。
2. 共用 PU 基值、坐标和符号，并对齐估计时刻。HFI 样本明确注入相位，SMO 对注入分量的处理有独立约定。
3. HFI 输出注入请求，经 Motor/Core 合成、共享电压预算和调制，再由 Motor 提交 PWM；算法不控制硬件。
4. Position 管理 HFI→资格确认 SMO→渐变→SMO，以及反向流程。切换阈值有滞回和驻留条件，目标提前运行并合格，角差按环绕处理。
5. 高速退出注入后，低速返回需重新安排注入和资格确认。回退只允许选择合格且符合运行条件的候选，否则报告丢失并安全停机。
6. 当前单 SMO Step 出错清空全部候选。双算法落地前必须补齐各算法失败状态，使一方失败不会抹掉仍有效的另一方结果。
7. 扩展保留已有 SENSOR、纯强拖、旁路 SMO、强拖切 SMO 行为，分别维护回归输入和预期结果。

## 9. 尚未闭合的边界与验收入口

| 当前限制 | 后续处理边界 |
|---|---|
| 统一理想电压模型尚无实际 PWM 生效点、死区及压降证据 | 对照硬件时序和波形验证模型，再独立更改电压语义 |
| Rs/Ld 固定轴会话已实现，横轴电流不能证明转子静止 | 在硬件上验证对齐、转子保持、注入时序和辨识精度 |

2026-10-09 边界修复已运行 FLOAT/FIXED 主机回归及 G431 debug-rel FLOAT / release FIXED 构建，未进行真机验证。详细记录见 [边界修复记录](2026-10-09-foc-boundary-fix.md)。当前验收入口如下，历史通过记录不能代替下一次变更后的验证。

| 范围 | 入口 |
|---|---|
| 统一电压模型、电流范围与极性 | `foc/tests/run_motor_sample_boundary_test.ps1` |
| 模块依赖与 ISR 顺序 | `foc/tests/run_foc_module_boundary_test.ps1` |
| SMO NONE/启用、独立递推、失败与复位（入口名沿用） | `foc/tests/run_observer_contract_test.ps1`、`run_smo_test.ps1` |
| Position 传感器/强拖与强拖接管 | `run_motor_position_boundary_test.ps1`、`run_sensorless_position_handoff_test.ps1` |
| Startup 与 Motor | `run_sensorless_startup_test.ps1`、`run_motor_alpha_beta_test.ps1` |
| Identify 类型边界与 Rs/Ld 数值 | `run_identify_pu_boundary_test.ps1`、`run_identify_resistance_test.ps1`、`run_identify_inductance_failure_test.ps1` |
| App 请求、命令及波形 | `run_encoder_command_test.ps1` |
| 目标构建 | 仓库根目录 `make.bat`，明确目标与 FLOAT/FIXED 配置 |

近期变更记录：[Motor](2026-10-09-motor-boundary-refactor.md)、[SMO](2026-10-09-smo-algorithm-template.md)、[Identify PU](2026-10-09-identify-pu-boundary.md)、[R/L 无编码器](2026-10-09-identify-stationary-boundary.md)、[Observer](2026-10-09-observer-composition-boundary.md)。

## 10. 本次同步修正

| 旧描述 | 对照源码后的修正 |
|---|---|
| 台架位置源切换 | 当前 SENSOR + SMO 影子观测；无感启动与接管关闭 |
| Core / PWM 作为一个模块提交硬件 | Core 输出数学结果，Motor 调用 PWM 端口 |
| Position → Observer → SMO 及嵌套结果 | 当前 Position → SMO，使用 `tSmoCfg` 和扁平 `tEstimate`；未来多算法按真实输入输出放在 ISR 编排同级 |
| Position 锁存事件并单独取出、应用 | 本拍反馈携带 `eEvent`，`motor_IsrControlStep()` 内消费 |
| Core、Motor、Encoder 保存重复数据 | Core 不复制 αβ 输入；Motor 原始样本局部化、统一 Iq 上限；Encoder 复用已发布历史快照 |
| Identify 只在图里泛称采样和电压请求 | 补齐实际母线重构、独立 Rs/Ld 类型、PU 结果与调制度请求 |
| 同 ISR 提交辨识请求容易理解为第二次 PWM 更新 | 明确当前调制度请求仅更新参考，下一次控制应用；中止当前 ISR 生效 |
| 通用运行图容易涵盖所有状态 | 明确 RUN 配对和 ALIGN 内部控制、零位捕获两条执行分支 |
| 系统电压模型分叉与残留状态访问 | Motor 统一模型，App 通过 ALIGN 快照及启动预算语义检查接线；保留静止/硬件时序验证限制 |

## 11. 本次边界收敛

App 使用 Prepare 发布的 `bAlignActive` 失效位置零位；初始化通过 `motor_position_CheckStartupBudget()` 校验交接预算，不读取对象内部交接计数。普通控制失败后复位观测历史和候选，不继续推进估算。Rs/Ld 的公开 ISR 入口保留在各自源文件，内部采集/均值/公式只接收自身子状态，父对象仅用于入口操作校验、前台协调与公共错误锁存。`identify.h` 只前向声明 Motor，不引入硬件头文件。LOG=0/1 共用安全判定。

当前实时路径仍超过 embedded-coding 的三层调用深度规定；本次没有私自修改规则或把公式合并回 Motor，此规范差距仍需单独处理。

## 12. V1 开发基线

执行频率唯一取自目标 FOC_HF_ISR_HZ，App 频率字段必须与其相等；SENSOR 缺少编码器能力时 Init 失败。Position 的静态绑定和回退路径共用 tSensor 上下文。Motor 的 ALIGN 电流受相电流限制，速度 PI 输出范围与实际 Iq 范围求交；无感接管后的改速更新斜坡目标。未完成 ADC 校准的故障恢复只重开 ADC 触发，功率 PWM 保持关闭。

主线只保留已接入能力；未接入算法移至 experimental，删除运行时 controller vtable 和无人使用的旧数学兼容层。冻结职责、单位、时序、安全行为和验收入口，内部布局和新增功能仍可在独立验证后演进。实际模块组合测试入口为 foc/tests/run_foc_app_combination_test.ps1，详细修改、集成及验证见 [V1 收敛记录](2026-10-09-foc-framework-v1.md)。
