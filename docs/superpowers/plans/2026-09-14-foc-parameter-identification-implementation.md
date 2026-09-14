# FOC 参数离线辨识与标幺化实施计划

> 本计划已确认实施。按 Gate 和 Task 顺序推进；PU 契约迁移与辨识算法保持为
> 可独立评审的改动，不混成一次大改。

**目标：** 建立 Identify 所需的 PU 输入契约，并提供可独立调用的离线电机参数辨识模块，
输出 Rs、Ld、Lq 的 PU 结果。SMO 参数迁移、优化和验证放在 Identify 及 Encoder PU 回归
通过后的后续独立阶段。

**架构：** 当前控制/Identify 路径的电流、电压及速度参考使用 PU；PositionPort 保持机械位置语义。Identify 是独立 C/H 模块，App 负责对齐、调度、安全监控和结果交付，不给 `motor_t` 增加辨识状态或专用回调。当前 SMO 尚未验证，本计划默认关闭其运行；不在 Identify 阶段改写或整定 SMO。

**技术栈：** Embedded C、现有 FOC FLOAT/FIXED 数值后端、PowerShell 主机测试、Mermaid。

---

**状态：** 已确认实施

**日期：** 2026-09-14

**参考资料：** [旧标幺化方案](../../foc-parameter-identification-and-per-unit-scheme.md)
仅作背景记录，已由本计划取代。两者冲突时以本计划为准；以下三点明确不采用：

1. 旧方案 §4.1 的电感公式 L_pu = L / Z_base 量纲错误，正确公式为 L_pu = L / L_base。
2. 旧方案 §4.2 的电流 PI / SMO 自动整定不在本次范围内。
3. 旧方案 §5.1/§5.2 将辨识状态、请求及参数应用接口加入 motor_t 的设计不采用；motor_t 不增加辨识状态、回调或专用 API。

SguanFOC v3.1.0 的 `Sguan_Identify.c/.h` 仅作为 Rs 两电平和 95% RL 阶跃公式
的参考。该源码声明 All Rights Reserved；本计划只独立实现公式和状态流程，不复制
源码。若后续确需直接借用代码，必须先确认授权并保留原始版权声明。

## 1. 目标与边界

本次建立唯一的算法参数单位契约，并实现固件受控激励的离线辨识，输出定子电阻 Rs、
电感 Ld 和 Lq 的 PU 值。辨识期间电机不运行常规控制闭环，但固件会输出受限测试电压，
因此这是需要机械固定和电流保护的主动台架测试，不是无激励的被动测量。

Identify 是独立可调用模块，源码仅新增 foc/identify/foc_identify.h 和
foc/identify/foc_identify.c。该模块只负责辨识阶段、采样处理和计算；不依赖 motor、
foc_app、HAL、ADC、PWM、Shell 或 Observer。foc_app_t 持有 Identify 实例，负责生命周期
编排、硬件安全检查及结果交付。此独立 C/H 模块要求是明确的模块边界，不再为其增加
通用插件、回调注册表或额外接口层。

本次不实现 Flash / EEPROM 持久化，不自动整定电流 PI 或速度 PI，也不自动调整 SMO 的
滑模增益、PLL 或滤波参数；不增加 MOTOR_STATE_IDENTIFY，也不修改第三方、MODUS、vendor
或 FOC 以外的业务代码。允许的硬件适配范围为 peripheral/<chip>/foc_port.c 中与 FOC
ADC/PWM 单位边界直接相关的实现；为让硬件 break/OCP 可被软件安全状态观察，必要时
可最小修改对应芯片的 PWM break 初始化/中断适配文件。不得修改 vendor、MODUS 或无关
外设初始化。

执行顺序固定为：PU 速度契约迁移 → Encoder 主机回归与限流台架速度闭环验收 → Identify
数学和 App 台架验证 → 后续独立 SMO 计划。Encoder PU Gate 未通过前，不得修改、启用或验证
SMO；Identify 期间 `FOC_ENABLE_SMO` 默认关闭，App 不初始化、不绑定 Observer，Motor 高频
路径不执行 SMO。辨识结果只交付/显示，不刷新未验证的 Observer。

## 2. PU 单位契约与唯一真相源

由 foc_app_cfg_t 提供且仅提供以下三个物理基准：

- U_base：αβ 电压矢量 1.0 PU 对应的实际电压幅值；定义必须与当前 foc_svpwm()
  输入到占空比的线性、未削顶范围一致。
- I_base：相电流 1.0 PU 对应的实际电流幅值；由 App 选定并传给 FOC ADC 硬件边界。
- f_base：电气频率基准，单位为电气 turns/s；它是归一化基准，不是电机额定转速或调参值。

Rs 的物理定义与 `motor_params_t` 的相模型一致：每相绕组的等效电阻，电压、电流采用
FOC 相量约定。线间测得的电阻不得直接当作 Rs；须记录电机星形/三角形接法和测量方式，
再换算成对应的每相等效值。计划不假定通用换算系数。

foc_app_cfg_t 同时持有 SI 铭牌/基准输入和唯一的高频采样周期；不另建通用参数管理子系统。
当前计划只为 Identify 建立其所需的基准转换，不迁移既有 SMO 电机模型参数。现有
`motor_params_t` 中 Rs/Ld/Lq 的 SI→PU 迁移留待后续 SMO 计划；Identify 不读取或修改这些
字段，也不让它们进入辨识计算。

铭牌参数的正式入口只有 App 配置，不提供第二套“操作员直接输入 PU”的正式加载路径。
foc_app_Init() 将 SI 铭牌值换算为 PU 一次；辨识结果则已经是 PU，运行时应用时不得转回
SI 再重复换算。FOC ADC 端只接收 App 传入的 I_base，不得另行硬编码不同的电流基准。
U_base 必须与调制器和台架母线条件一致；如果当前调制比例与 1.0 PU 的定义不一致，应在
Task 1 通过最小必要的 FOC 调制改动统一，并用测试证明线性范围，而不是只更改文档标称。

基准及派生量：

    Z_base = U_base / I_base
    ω_base = 2π × f_base       （仅作为 L_base 公式中的派生量）
    L_base = Z_base / ω_base
    qElectricalBaseTurnsPerSample = f_base × T_s

ω_base 不是第二个独立基准，不得另设或重复配置。T_s 在 foc_app_cfg_t 中只配置一次。
qElectricalBaseTurnsPerSample 是 f_base × T_s 导出的无量纲每拍量，可传给 SMO/Identify
初始化；不得在每拍重复传 dt，也不得在算法配置中再存一份 20 kHz / 50 μs 常量。

SI 铭牌参数在 App 边界只转换一次：

    Rs_pu = Rs_SI / Z_base
    Ld_pu = Ld_SI / L_base
    Lq_pu = Lq_SI / L_base

SI 铭牌到 PU 的转换由 foc/app/foc_app.c 内部一个 static 函数在 foc_app_Init() 中完成；
ADC 的安培到 PU 换算仍归 FOC peripheral 边界。App 在 motor_Init() 前调用
foc_adc_SetCurrentBaseMilliamp()，将唯一的 I_base 传给硬件边界；不得在 peripheral、
Motor、Core、Observer、Identify 各自重复配置或转换。

本计划当前阶段的接口契约：

- 相电流、αβ/dq 电压、电流/电压参考、速度反馈和速度参考均为 PU。
- 电气角、机械角继续使用归一化 BAM 角度。
- `foc_position_t` 是机械位置接口：角度为机械 BAM，`qMechanicalSpeed` 保持机械
  turns/s。Encoder 的 `encoder_age_turns()` 必须使用此机械速度外推机械角；不得先乘极对数
  或改成电气速度后再外推。
- Motor 在 `motor_BuildPositionInput()` 边界把机械 turns/s 转为控制环所需的电气速度 PU：
  `qElectricalSpeedPu = qMechanicalSpeed × pole_pairs / f_base`。转换增益在初始化时派生一次，
  高频路径只做一次有界乘法；不在 Encoder 中增加第二份机械速度状态。
- Motor/Core 的速度反馈与速度参考、Shell `motor speed` 命令均为电气速度 PU。
  Encoder 诊断命令仍报告机械 turns/s，标签必须明确标为机械速度。
- 现有 `tSpeedPiParams` 和最大速度配置按电气 turns/s 解释。为保持既有物理速度环响应，
  App 在 `motor_Init()` 前一次性执行 `qSpeedPu = qSpeedTurnsPerSecond / f_base`，并将
  `tKp`、`tKiTs`、`tKdOverTs` 乘以 `f_base` 转成 PU 输入增益；最大速度限值除以 `f_base`。
  所有换算须检查 FLOAT/FIXED 范围，高频路径不做单位转换。此为单位迁移，不是依据辨识
  结果自动整定 PI。
- 波形 `Speed`/`SpeedRef` 观测的是电气速度 PU；同步更新通道名称或文档单位，避免继续被
  理解为 turns/s。SMO 输出契约和 motor_params_t 的 Rs/Ld/Lq PU 迁移属于后续阶段。

所有 PU 参数只对产生它们的同一组 U_base、I_base、f_base 有效。本计划不实现参数持久化或基准版本管理；结果仅交付/显示，不自动应用到 Motor 或 Observer。若操作者在外部记录结果，必须同时记录三个基准，基准改变后不得直接加载旧结果。

## 3. 辨识数学与轴向定义

进入 Identify 的 foc_ab_t 电压、电流已经是算法数值域中的 PU 值，但其“1.0”是否对应约定的 SI 基准仍须通过 Gate 0 验证。Rs 使用两个稳定 D 轴电压等级的轴向变化量计算。每个有效样本将上一完整 PWM 区间的 `Vmodel[n-1]` 与本次 ADC 电流 `I[n]` 配对；低/高平台分别在稳定窗口内对配对的 D 轴投影求均值：

    ΔV_D_pu = mean(V_D_high) - mean(V_D_low)
    ΔI_D_pu = mean(I_D_high) - mean(I_D_low)
    Rs_pu = ΔV_D_pu / ΔI_D_pu

平台进入平均前，先经过配置的最小稳定等待拍数；随后要求连续平均窗口的电流峰峰值不
超过 Gate 0 校准的 `qCurrentStabilityTolerancePu`。超时仍未稳定即失败，不得把固定等待拍数
直接视作稳定。平均窗口长度、最小等待拍数和稳定阈值属于 Identify 配置；电压均值必须来自
与电流同区间配对的 `Vmodel`，不得用电压参考命令代替。

电感使用对齐后的固定转子 RL 阶跃响应。对齐后，Motor 的电角度参考为零，因此 D 轴对应
α 分量，Q 轴对应 β 分量。辨识器分别使用激励轴上的有符号投影电压与电流；不得用 αβ
向量模长替代单轴量，也不得混用 dq 与 αβ 分量。位移超过 Gate 0 规定阈值即中止。

每次电感阶跃开始前，记录该轴初始电流 I0。以两级稳态得到 Rs_pu 后，按本次阶跃电压计算最终电流 I∞_pu；交越阈值为：

    I∞_pu = mean(Vmodel_axis_pu) / Rs_pu
    I95_pu = I0_pu + 0.95 × (I∞_pu - I0_pu)

设测试电压完整生效后，经过 N 个完整高频采样区间首次达到 I95，则：

    Ld_pu 或 Lq_pu = Rs_pu × ω_base × (N × T_s) / ln(20)
                   = Rs_pu × 2π × qElectricalBaseTurnsPerSample × N / ln(20)

N 的定义是从测试电压在 PWM 更新点开始生效，到首次有效交越采样为止的完整区间数；
首个完整区间即 N=1，可作为最小可分辨结果；若交越在阶跃前已发生（N=0），或采样时已
无法确定首次交越区间，则判为快于采样分辨率。主机测试与固件状态机使用同一计数约定。
进入 D_STEP/Q_STEP 后收到的第一个配对样本仍携带阶跃命令生效前的 Vmodel；该样本只做安全
校验，不进入电压均值、电流交越判断或 N 计数。其后的首个完整阶跃区间才是 N=1。
用固定常量 1/ln(20) 和有界宽位中间量，不在高频路径调用 log()。

ΔI 低于门槛、平台未稳定、激励电压超出 Gate 0 验证的线性 SVPWM 范围、测试电流超限、样本无效、初始电流未复位到允许范围、交越快于采样分辨率、超时或参数超出 FLOAT/FIXED 可表达范围时，结果整体无效；不得发布部分 Rs/Ld/Lq。Init 必须拒绝超过线性范围的激励配置；Gate 0 还须证明获准的整个电压向量范围不会触发 SVPWM 占空比裁剪。

## 4. 物理边界与 Gate 0 安全条件

当前 ADC 计数和 PWM 电压指令在软件中已归一化，但现有硬件转换常量尚不足以证明它们与
实际安培/伏特基准一致。Gate 0 是任何通电辨识前的硬门槛，至少完成以下核验：

1. 用外部电流测量确认 ADC 原始计数到相电流的换算及 I_base；确认 App 设置的基准实际传入 peripheral FOC ADC 边界。
2. 确认 foc_svpwm() 的 αβ 指令到占空比线性范围、PWM 生效时刻及 U_base；将 Core 的上一拍电压命令与该基准下的实际绕组电压误差核对。
3. 本计划不新增母线电压采样。台架测试使用稳定、已外部测量的固定直流母线；若母线变化、死区或压降导致模型电压误差超出事先批准的辨识误差范围，则不得接受该结果。
4. 确认 f_base、T_s、极对数和 Encoder 速度 PU 转换一致；检查 FLOAT/FIXED 下的定点动态范围和 Q 格式溢出边界。
5. 明确首期支持的测试电流/电压范围、重复性及误差验收门槛。机械固定转子，验证硬件 OCP、软件相电流限幅、Encoder 位移中止条件和各阶段超时。
6. 确认 D/Q 轴正方向、绕组相序、每相 Rs 定义和零位与现有板级实现相符；不得为简化软件而改动现有 U/V/W、ADC 或 PWM 相序映射。
7. 为两块目标分别记录 ADC 触发相位、PWM CCR 预装载/更新生效点及二者间隔；证明 `Vmodel[n-1]` 对应本次 `I[n]` 采样前已完整生效的电压区间。跨越旧/新占空比的混合区间必须丢弃，`N` 只计完整区间。确定转子位移的最大允许电气角误差，并验证其对应的机械角门限。
8. 验证 PWM break/OCP 硬件关断之外，软件能读取锁存的 OCP/break 事件，并使 App/Identify 进入失败终态；不得仅依赖 Motor 状态推断硬件 OCP。

任何基准无法校准、模型电压与实际电压的误差不可接受、保护链路未经验证或测试条件无法复现时，停止硬件辨识，不得把结果标记为有效 PU 电机参数。高频中断禁止 I2C、日志和阻塞操作；AS5600 更新继续在现有前台低频路径运行。

## 5. 模块拓扑与调用时序


 ```mermaid
    classDiagram
        class foc_app_t {
            motor_t tMotor
            foc_encoder_t tEncoder
            foc_identify_t tIdentify
            one-byte pending command
            aligned mechanical angle baseline
        }
        class motor_t {
            existing lifecycle and Core state
        }
        class foc_identify_t {
            measurement state
            resistance level and active axis
            PU accumulators
            PU result Rs Ld Lq
            Init()
            Start()
            Step()
            Abort()
            GetResult()
        }
        foc_app_t *-- motor_t : owns
        foc_app_t *-- foc_encoder_t : owns
        foc_app_t *-- foc_identify_t : owns
        foc_app_t ..> foc_identify_t : schedules and reads result
        foc_app_t ..> motor_t : existing public API
 ```

此 UML 只描述 Identify 阶段：Observer/SMO 不在对象所有权或调用链中；不得因为代码中
保留旧结构而初始化、绑定或步进它们。

状态图表示完整的逻辑测量阶段。`foc_identify_output_t.eStatus` 使用粗粒度状态码：
`RESISTANCE_SETTLE`、`RESISTANCE_AVERAGE`、`AXIS_RESET` 和 `AXIS_STEP`；低/高电阻
平台及 D/Q 轴由 `foc_identify_t` 的实例状态区分。合并编码不改变电压参考切换、采样
边界、判稳、超时或结果计算。

App 是 Identify 与 Motor 之间唯一的编排边界。每次高频中断按以下顺序工作：

1. 在调用 motor_HighFrequencyStep() 前，保存 Core 中上一拍生成的 tVoltageAlphaBeta[n-1]，称为 Vmodel[n-1]。
2. 调用现有 motor_HighFrequencyStep()；Motor 完成 ADC 采样和电流处理，更新 tInput.tCurrentAlphaBeta[n]、控制状态及本拍电压命令。
3. 若 Motor 已进入故障或非辨识运行状态，停止 Identify 推进并走安全终止路径；否则将本拍 PU 电流和保存的 Vmodel[n-1] 传给 foc_identify_Step()。
4. 仅当 Identify 输出标记参考发生变化时，App 才调用 motor_SetVoltageReference()；该参考供下一控制周期使用。

对齐完成后 App 保存机械角基准。每拍先计算绕回安全的机械位移，再乘极对数得到电气位移；
Gate 0 按允许的 D/Q 串扰确定最大电气角误差，并换算成机械门限。超限时当前样本无效并
立即终止。只有门限内，才允许将 α/β 分量作为固定对齐 D/Q 轴的投影。

两块目标均须验证 ADC 采样点与 CCR 预装载/更新生效点的映射：只有完整施加在一个采样
区间内的电压才能和该区间结束时的电流配对。任何跨越 CCR 更新边界的混合区间均丢弃，
不进入 Rs 平均或电感 `N` 计数；主机测试和固件状态机使用同一完整区间规则。

tCore.tVoltageAlphaBeta 是 Core 经逆 Park 得到并送往 SVPWM 的模型电压指令，不是绕组端
电压实测值。文档、变量和日志统一称 Vmodel；只有 Gate 0 证明其标度及误差满足要求，
且测试处于线性未饱和区时，才能把它用于辨识计算。

辨识复用现有 MOTOR API：对齐通过 motor_RequestPositionCalibration() 完成；对齐结束且
Motor 回到 IDLE 后，调用 motor_Start(FOC_MODE_VOLTAGE)，再通过
motor_SetVoltageReference() 提交测试向量。motor_Start() 仍要求有效位置源；Encoder
必须持续运行，前台 foc_app_Run() 继续按 1 ms 更新位置，运动看门狗只读该缓存值。不得在
Identify 中直接访问 I2C 或另造无位置源的 PWM 注入通道。

当前阶段不初始化或绑定 Observer。关闭 `FOC_ENABLE_SMO` 时，App 将 Motor 的 Observer
依赖保持为 `NULL` 并跳过 `foc_observer_Init()`；Motor 已有的空指针分支确保 20 kHz 路径
不调用 SMO。SMO 源码可以暂留构建中以避免无关的构建拆改，但不得被初始化、步进或接收
Identify 参数。

首拍处理只采用一种方案：对齐完成后以零电压参考启动 VOLTAGE 模式，保持一个完整高频
周期；该周期清零/稳定 Core 电压历史并丢弃电流样本。预热结束后再调用
foc_identify_Start() 并开始第一段激励，电感计时从首个测试电压完整生效的区间开始。
不得在实现时自行改成“首拍补零后直接计入”。

## 6. App 与 Identify 状态转换

App 负责对齐和 Motor 生命周期；Identify 只负责测量阶段。状态图如下：


 ```mermaid
    stateDiagram-v2
        [*] --> Idle
        Idle --> Aligning: identify request accepted
        Aligning --> ZeroPreheat: alignment complete, voltage mode started
        Aligning --> Aborted: cancel
        Aligning --> Failed: Motor fault or OCP
        ZeroPreheat --> RsLowSettle: one zero-voltage period complete
        RsLowSettle --> RsLowAverage: minimum wait and stable window passed
        RsLowAverage --> RsHighSettle: low-level average complete
        RsHighSettle --> RsHighAverage: minimum wait and stable window passed
        RsHighAverage --> LdReset: resistance valid
        LdReset --> LdRise: D-axis current reset
        LdRise --> LqReset: Ld threshold reached
        LqReset --> LqRise: Q-axis current reset
        LqRise --> Complete: Lq threshold reached
        ZeroPreheat --> Aborted: cancel
        RsLowSettle --> Aborted: cancel
        RsLowAverage --> Aborted: cancel
        RsHighSettle --> Aborted: cancel
        RsHighAverage --> Aborted: cancel
        LdReset --> Aborted: cancel
        LdRise --> Aborted: cancel
        LqReset --> Aborted: cancel
        LqRise --> Aborted: cancel
        RsLowSettle --> Failed: timeout or safety guard
        RsLowAverage --> Failed: invalid delta or range
        RsHighSettle --> Failed: timeout or safety guard
        RsHighAverage --> Failed: invalid resistance
        LdReset --> Failed: timeout or movement
        LdRise --> Failed: timeout, limit, or poor resolution
        LqReset --> Failed: timeout or movement
        LqRise --> Failed: timeout, limit, or poor resolution
        ZeroPreheat --> Failed: Motor fault or OCP
        RsLowSettle --> Failed: Motor fault or OCP
        RsLowAverage --> Failed: Motor fault or OCP
        RsHighSettle --> Failed: Motor fault or OCP
        RsHighAverage --> Failed: Motor fault or OCP
        LdReset --> Failed: Motor fault or OCP
        LdRise --> Failed: Motor fault or OCP
        LqReset --> Failed: Motor fault or OCP
        LqRise --> Failed: Motor fault or OCP
        Complete --> Idle: PWM stopped, result consumed
        Failed --> Idle: PWM stopped, failure consumed
        Aborted --> Idle: PWM stopped, abort consumed
 ```

所有激励阶段共享样本有效性、电流限幅、位移监控、Motor fault、锁存 OCP/break 状态和阶段超时检查。每次 ISR 处理活动状态前优先消费取消请求；Motor fault/OCP 进入 Failed，操作者取消进入 Aborted。两条路径都先停止 PWM，再发布终态。motor identify 只允许在 App/Encoder 就绪、Motor 为无故障 IDLE 时启动。motor stop 在辨识期间转换为取消请求，由唯一所有者完成识别终止和 Motor 停止；不允许 Shell 前台与 ISR 并发直接写 Identify 状态。

Complete、Failed 或 Aborted 时先停 PWM，再发布终态。电流超限、位置移动、ADC/PWM/Motor 故障均进入安全终止；ISR 负责停止输出和发布结果状态，前台负责文字报告。失败和取消不返回部分结果。

## 7. 独立 Identify API 与上下文所有权

模块 API 保持小而直接：

    foc_result_t foc_identify_Init(
        foc_identify_t *ptIdentify,
        const foc_identify_cfg_t *ptConfig);

    foc_result_t foc_identify_Start(foc_identify_t *ptIdentify);

    foc_result_t foc_identify_Step(
        foc_identify_t *ptIdentify,
        const foc_identify_sample_t *ptSample,
        foc_identify_output_t *ptOutput);

    void foc_identify_Abort(foc_identify_t *ptIdentify);

    foc_result_t foc_identify_GetResult(
        const foc_identify_t *ptIdentify,
        foc_identify_result_t *ptResult);

接口数据约定：

- foc_identify_cfg_t 只含 PU 激励/电流限值、`qCurrentStabilityTolerancePu`、稳定等待/平均窗口采样数、阶段超时数和 qElectricalBaseTurnsPerSample。
- foc_identify_sample_t 含本拍 foc_ab_t PU 电流、上一周期 foc_ab_t PU 模型电压 Vmodel，以及有效标志。
- foc_identify_output_t 含 D/Q PU 电压参考、参考变化标志和粗粒度 Identify 状态。低/高电阻平台及 D/Q 轴不另设公开状态码；API 错误只由函数返回的 foc_result_t 表示，不在 output 中重复存结果码。
- foc_identify_result_t 只含 qResistancePu、qInductanceDPu、qInductanceQPu，不含 SI 值或重复基准。
- Identify 无堆分配、硬件依赖、Shell 依赖和 motor_t 回调。

上下文所有权固定如下：初始化在启用中断前完成；开始后，20 kHz ISR 独占调用
foc_identify_Start()/foc_identify_Step()/foc_identify_Abort()，并独占写辨识状态与结果。
前台只写一个目标平台原子读写的 volatile 单字节命令值（NONE/START/CANCEL）；ISR 读取并
清零后处理，不增加锁、队列或多字段 mailbox。

`foc_identify_Start()` 仅在 IDLE 或已消费的终态接受；活动期间再次 Start 必须拒绝。每次
被接受的 Start 清空阶段计数、累加器、旧结果、参考输出和终态。所有可变运行状态必须属于
`foc_identify_t`，禁止共享的函数级/文件级可变静态状态；测试覆盖重复运行和两个实例交错
运行时互不影响。

结果发布采用单写者、终态提交约定：ISR 先写完全部 PU 结果字段，再执行项目现有的目标平台
release/compiler barrier，最后写入 volatile COMPLETE/FAILED/ABORTED 状态。前台先读取终态，
再执行对应 acquire/compiler barrier 后调用 `foc_identify_GetResult()`。终态结果在下一次
START 被接受前保持只读；不得把 `volatile` 单独当作内存屏障。FLOAT/FIXED 的标量分别是
`float`/`int32_t`，实现须验证目标上 32 位对齐访问及 barrier 契约；不新增通用锁或同步 API。

硬件 OCP、Motor fault、Encoder 位移和 PWM 停止属于 App 集成测试，不是独立 Identify
算法 API 的输入。Identify 单测只验证模块实际拥有的行为，如输入无效、电流限值、阶段
超时、复位/取消和不发布部分结果。

## 8. 加载与应用 PU 参数

Identify 完成后只交付并显示同一组基准下的 PU 结果。本计划不把结果转回 SI，不写入
`motor_params_t`，也不初始化/刷新 Observer；这样可先取得可靠电机参数，再在后续 SMO
计划中完成 PU 模型迁移和算法精简。电流 PI、速度 PI、SMO 滑模增益、PLL 增益和滤波参数
均不由本次辨识自动整定。

复用 foc/foc_config.h 已有的 FOC_ENABLE_EXPERIMENTAL_IDENTIFY（默认 0）门控 Identify，
不新增平行的 Identify 宏。当前代码不存在 SMO 开关，因此新增 `FOC_ENABLE_SMO`，默认 **0**。
关闭时 App 跳过 Observer 初始化和绑定，Motor 高频路径不得执行 SMO；本阶段不做 SMO
运行测试或参数刷新。后续 SMO 计划通过独立评审、Encoder PU Gate 和 Identify Gate 后，才允许
启用并测试。后续 SMO 计划必须先统一 Observer 速度输出为电气速度 PU，或明确唯一的 Motor
边界换算点；不得让 turns/s 与 PU 两种契约直接进入同一速度环。不得只关 Observer 输出而
仍执行 SMO Step。

本计划不做 Flash/EEPROM 持久化。仅 `foc_identify_t` 持有本次辨识的 PU 结果；Motor 和
Observer 不保存、不加载也不应用该结果。任何后续外部持久化必须将参数与其 U_base、I_base、
f_base 配置一起版本化，但不在本次添加存储层。

Flux 不属于本次固定转子 Rs/Ld/Lq 流程。Sguan 的 Flux 路径依赖旋转工况和非零电气速度；
若未来消费者需要 Flux，须另行设计旋转辨识、机械/电气安全保护和验收 Gate，不作为本计划
的隐含扩展。

## 9. 实施任务与顺序

### Gate 0：确认 PU 基准和可测条件

- 定义 U_base、I_base、f_base、T_s 和每个量的物理单位及来源。
- 实测确认 ADC counts→安培→PU、SVPWM 线性范围与 PWM 生效延迟；确认 Vmodel 与实际绕组电压误差满足预先批准的辨识精度。
- 选定稳定母线、机械固定方式、激励和电流限值、位移/故障中止条件、阶段超时及重复性验收标准。
- 对每次台架结果记录实际 D/Q 激励、电流范围、母线电压、绕组/可追溯温度条件和重复测量结果；这些是测试记录，不进入 Identify API。
- 确认 D/Q 对齐及 U/V/W/ADC/PWM 相序；核验 FLOAT/FIXED 的中间量范围。
- 任一条件未通过时，只允许做主机数学测试，不允许硬件激励或应用结果。

### Task 1：迁移 Encoder/Motor 速度 PU 契约

此任务是独立的速度单位迁移，须先单独审核其 UML、调用接口和改动边界，不得与 Identify
模块实现合并。现有速度 PI/速度限值配置按电气 turns/s 解释，由 App 初始化时按 §2 公式
一次换算为 PU 等价参数；主机回归须证明相同物理速度误差在换算前后产生相同 PI 输出。
完成后必须先通过 Encoder PU Gate，才能进行 Identify 硬件集成；SMO 参数
迁移与算法修改不在此任务中。修改后同步更新 foc/docs/foc-architecture.md 的单位/速度契约。

文件范围：

- 修改 foc/foc_types.h、foc/hal/foc_position.h；`qMechanicalSpeed` 保持机械 turns/s。
- 修改 foc/motor/motor.h、foc/motor/motor.c。
- 修改 foc/observer/foc_encoder.h、foc/observer/foc_encoder.c；角度外推继续使用机械速度。
- 修改 foc/app/foc_app.h、foc/app/foc_app.c：配置唯一的 f_base 转换增益，速度命令/API/波形统一为电气 PU；`encoder` 诊断仍显示机械 turns/s。
- 修改 foc/hal/foc_port.h，新增 `foc_adc_SetCurrentBaseMilliamp()`；App 在 ADC
  校准/采样开始前传入 I_base。OCP/break 软件可观察接口属于 Task 3，不扩大本任务范围。
- 修改 foc/foc_config.h：复用 FOC_ENABLE_EXPERIMENTAL_IDENTIFY；新增默认关闭的 FOC_ENABLE_SMO 运行门控。
- 修改 peripheral/stm32g431/foc_port.c、peripheral/at32f413/foc_port.c，使 ADC 单位缩放使用 App 传入的 I_base；不得另存互相矛盾的基准。count-to-amp 标定常量只保留在对应硬件适配中。
- 更新 foc/docs/foc-architecture.md、foc/README.md、foc/docs/foc-test-guide.md、Shell `encoder`/`speed` 标签、Speed/SpeedRef 波形单位及相关 FOC 测试。

完成条件：

- Encoder 的 `encoder_age_turns()` 使用机械 turns/s，外推结果与机械角一致，且不受极对数影响。
- Motor 只在 `motor_BuildPositionInput()` 把机械 turns/s 转成电气速度 PU；固定后端使用经范围验证的初始化派生增益。
- 速度 PI 增益与最大速度限值按 §2 一次性单位换算；同一物理速度误差在换算前后得到一致 PI 输出。
- Shell/API 的 speed reference 与 Motor/Core 速度反馈为电气 PU；Encoder 诊断标签明确为机械 turns/s；Speed/SpeedRef 波形单位同步为 PU。
- `FOC_ENABLE_SMO=0` 时 App 不初始化/绑定 Observer，20 kHz 路径不执行 SMO。
- 不向 motor_t 增加辨识字段、状态或回调；不改硬件 U/V/W、ADC 或 PWM 相序。
- 先完成 FLOAT/FIXED Core、Encoder、Motor 速度映射回归，再进行 Identify 台架集成；本任务不运行 SMO 算法测试。

### Task 2：实现独立 Identify 模块

Task 2 可与 Task 1 并行；它只使用合成 PU 序列实现和测试数学核心，不接硬件、不访问
Motor、不应用 Observer 参数。硬件采样及 App 集成必须等待 Task 1 和 Gate 0 均通过。

文件范围：

- 新建 foc/identify/foc_identify.h、foc/identify/foc_identify.c。
- 修改 foc/foc.mk，仅在 FOC_ENABLE_EXPERIMENTAL_IDENTIFY 开启时加入 Identify 源码。
- 新建 foc/tests/foc_identify_test.c、foc/tests/run_identify_test.ps1。

测试及实现要求：

- 实现两电平稳态 Rs_pu 和 D/Q 轴 95% 阶跃 Ld_pu/Lq_pu；按 §3 轴向、初始电流、PWM 生效区间定义计算。
- T_s 固定在 Init 配置；用有界宽中间量、显式范围检查，不动态分配，不在 Step 做 transcendental log 运算。
- FLOAT 和 FIXED 分别覆盖已知合成 R/L、初始电流偏差、正负激励、采样边界、量化分辨率、范围溢出、无效样本、电流超限、平台不稳定、超时、重复 Start、复位、取消、双实例隔离和禁止部分结果。
- Task 1 完成后，Encoder PU Gate 必须先通过：主机测试覆盖非零样本年龄、机械速度正反向外推、多个极对数下机械角外推不变，以及 Motor 输出速度 PU 换算；随后在限流台架跑通 Encoder 速度闭环。Gate 未通过不得进入 Identify 硬件集成；尤其不得开始 SMO 源码修改、参数迁移、启用或算法验证。
- Identify 单测不伪造测试硬件 OCP、Encoder 移动或 Motor fault；这些属于 Task 3。
- 验证测试参考只在阶段边界变化，终止结果不保留部分参数。

### Task 3：App 调度、安全和结果交付

依赖：Gate 0、Task 1、Task 2 和 Encoder PU Gate 全部通过后才能进行硬件集成。

文件范围：

- 修改 foc/app/foc_app.h、foc/app/foc_app.c。
- 必要时修改 `foc/hal/foc_port.h` 和对应目标的 PWM break 适配文件，为 App 提供最小
  OCP/break 锁存状态查询；不得增加 Motor 专用 Identify API。
- 提供 `bool foc_pwm_GetFaultStatus(void)` 与
  `foc_result_t foc_pwm_ClearFaultStatus(void)`；App 在电机清故障流程中仅当硬件 break
  源已解除且底层清锁存成功后才调用 `motor_ClearFault()`，失败时保持 Motor 故障状态。
  适配层必须保留 OCP/break 锁存，直至显式清除，不得由 ISR 自动清除。
- 扩展 foc/tests/foc_app_encoder_command_test.c；若其替身无法覆盖完整次序，则新增一个聚焦的 App Identify 集成测试。
- 更新 foc/README.md 与 foc/docs/foc-test-guide.md，文档只说明通用 API/调试流程；板级实例与框架职责分开。

实施约束：

- App 持有一个 Identify 对象和最少量的辨识请求/运行状态；Motor 生命周期不新增辨识状态；Observer 保持未初始化且未绑定。
- 复用位置校准、FOC_MODE_VOLTAGE、motor_SetVoltageReference；不增加 Identify 专用 Motor API。
- ISR 按 §5 的 Vmodel[n-1]/I[n] 次序调用 Identify，只在参考变化时提交 Motor 电压参考。
- 前台继续以 1 ms 周期更新 Encoder；Shell 仅提交单字节 start/cancel 请求并在终态后读取、显示 PU 结果。
- App 集成测试覆盖对齐成功/失败、首个零电压预热周期、ISR 与前台命令交接、Motor fault、OCP 锁存/终止、各活动状态的取消、Encoder 位移取消、阶段超时、PWM 线性范围拒绝、PWM 停止及完整 PU 结果交付；不测试 Observer/SMO 参数应用。
- 所有异常路径保持 PWM 关闭；无 I2C、日志或阻塞工作进入高频 ISR。

### Gate 2：数值、构建与受控台架验收

- 运行 Identify、Core、Encoder、Motor 速度映射的 FLOAT/FIXED 主机测试；`FOC_ENABLE_SMO=0` 构建与 App 集成中确认无 Observer 初始化/高频调用。
- 构建当前支持的 STM32G431 与 AT32F413 FOC 目标；确认硬件 U/V/W、ADC/PWM 映射没有变化。
- 通过现有 `perf_counter` tick 统计并按已验证 tick 频率换算时长，不使用 DWT。分别记录两块目标上 ADC ISR 入口到下一 CCR 生效截止点的最坏执行时间和裕量；正常与 Identify 路径均须留有 Gate 0 批准的余量。另记录相对正常路径的 Identify 增量。SMO 开启后的性能验收属于后续计划。
- 在机械固定转子、稳定且外部测量的母线、限流电源和独立电流/电压仪表条件下重复辨识；按 Gate 0 事先批准的误差与重复性标准验收。
- 在两个目标上确认 break/OCP 会硬件关断 PWM、软件状态保持锁存且可观测；清除故障前不得重新使能 PWM。
- 完整结果交付后记录对应 U_base、I_base、f_base；基准改变后旧 PU 结果无效。本计划不自动应用到 Motor/Observer。

### 后续独立阶段：SMO 参数迁移、精简与验证

本阶段不属于当前 Identify 实施范围。只有 Encoder PU Gate（含限流台架速度闭环）和
Identify Gate 均通过后，才允许开始任何 SMO 源码修改；之后另行审核 SMO 模块图、状态/数据流
与精简方案。此前文档中提到的 Observer 接管不属于本计划的 Gate 2；后续 SMO 计划必须定义
独立的传感器接管 Gate，至少满足以下条件后才允许实施：

- SMO 的电气角度/电气速度先以 Shadow 方式与 Encoder 对照；未达到已定义的连续有效性和
  误差门槛时，显式 Encoder→Observer 接管请求必须被拒绝。
- 接管后若 Observer 失效或失锁，Motor 必须锁存故障并停止 PWM；不得静默退回 Encoder，
  也不得继续使用无效估算角度闭环。
- 只有一个算法时不保留 `foc_observer` 透传包装和绑定具体 SMO 的函数指针层；Motor 直接
  持有并调度 SMO 实例。不得为了假设中的多算法扩展增加通用抽象层。
- SMO PLL 直接跟踪电气量并输出电气角度和电气速度 PU；算法内部不保存机械 PLL 状态、
  重复极对数格式或执行机械/电气往返换算。机械速度到电气速度的唯一转换仍在 Motor 输入边界。
- 每拍只计算一次 SMO 电气角度并复用该值；静态依赖在 Init/接管时校验，避免在高频路径
  增加冗余的多层空指针检查和透传调用。
- 只保留已由 App 配置且有实机依据的必要有效性门槛；Init 拒绝缺失/零值配置，不允许
  “所有门槛为零”时绕过资格检查。不得恢复未配置的五参数窗口/门禁套件。

届时使用已确认的 PU Rs/Ld/Lq 迁移 `motor_params_t` 和 SMO 参数契约，移除重复配置/派生
状态，先做 FLOAT/FIXED 主机测试，再启用 SMO Shadow 并测量 20 kHz 预算。旧 SMO 计划中的
“Gate 1”应按该独立接管 Gate 定义，不与本计划的 Gate 2 混用。完成独立评审前，
`FOC_ENABLE_SMO` 始终保持关闭。

## 10. 验收标准

- 本计划交付的 Identify 输入与结果、以及 Encoder 控制路径的速度反馈/参考均为 PU；机械位置接口保留机械角与机械 turns/s。
- ADC 电流物理缩放由 peripheral FOC 边界使用同一个 I_base；不在多个算法模块重复配置电流基准。
- foc_identify_result_t 中 Rs/Ld/Lq 均为 PU；不自动整定 PI/SMO 参数，也不将结果应用到未验证的 Observer。
- Identify 是有独立 C/H 和 FLOAT/FIXED 主机测试的模块，不依赖 Motor/HAL/Shell。
- 机械固定、轴向定义、Vmodel 电压误差、线性调制和保护条件通过 Gate 0 后，才允许台架激励。
- 所有结束、失败和取消路径都停止 PWM；不发布不完整或超范围结果。
- Observer/SMO 在当前计划中保持未初始化、未绑定、未运行；其 PU 参数迁移与优化由后续独立计划完成。
- motor_t 不增加 Identify 对象、回调、状态或专用 API；当前硬件相序保持不变。
- 非辨识时中断额外开销有界；辨识路径满足经审核的 20 kHz 周期预算。
- 不修改第三方/MODUS/vendor 或 FOC/peripheral 范围以外的源码。
