# Motor 电角反馈边界 Implementation Plan

> **实施状态：** Prepare → Position → Control、Encoder 反馈、Motor 固定频率开环候选和初始化选源已接入。当前 `foc_config.h` 启用 SMO 影子估算；运行中切换、融合与 HFI 注入仍未实现。

**Goal:** 将位置源读取、机械到电角换算和角度源融合从 Motor 的 RUN 控制步骤移出，让 Motor 的 FOC 控制只消费统一电角反馈。

**Architecture:** `foc_app_HighFrequencyISR()` 依次调用 Motor Prepare、App 持有的 Position Step、Motor Control。Position 从 Encoder 或 Motor 开环候选中按初始化配置输出唯一电角反馈；启用 SMO 后可额外运行 Observer 影子估算，但不切换控制源。Motor 拥有功率级、FOC Core、ALIGN 和故障生命周期。注入请求尚未实现。

**Tech Stack:** C11、STM32G431 编译期 MDI 绑定、20 kHz ISR、FLOAT/FIXED、现有 `foc/tests` 主机测试。

---

## 1. 改造前代码的硬约束

- `foc/app/foc_app.c::foc_app_HighFrequencyISR()` 目前只有一次 `motor_IsrStep()`；`foc/motor/motor.c::_motor_RunControlStep()` 在内部依次采样电流、Clarke、取编码器位置、运行 Observer、速度环、Core、PWM。
- Encoder 可在 Motor 控制步前读取；SMO 需要本周期 `iα/iβ`，必须等 Motor 采样与 Clarke 后才能运行。若把 `FOC_POSITION_GET()` 直接放在现有 `motor_IsrStep()` 前，SMO 只能用旧电流。若放在其后，本周期 Core 已算完。**要同周期统一，必须拆分 ISR 控制入口；否则明确接受一周期延迟并重新校准电压/电流时序。**
- `foc_core_input_t` 和 Motor 速度环都需要电角速度 PU；统一接口不能只含电角度。所有来源必须交付相同电角零位、旋转方向、BAM32 角度、PU 速度和本周期有效性语义。
- ALIGN 当前在 Motor 内捕获 Encoder 机械角并写电零位；若位置读取搬到 App，零位捕获也必须搬出，并让 Motor 与 App 完成一次有结果的交接。无感硬拖没有机械零位可捕获，不能冒用同一个校准完成标志。
- 当前 `foc_app.bReady` 与 Encoder 初始化成功绑定，`motor_Init()` 也要求位置 Provider。要允许纯无感构建，二者都要改为“所选来源已就绪”，否则统一接口有了仍无法启动。

## 2. 统一值接口与放置位置

推荐把运行态值定义在 `foc/motor/motor_position.h`：

```c
typedef struct {
    foc_angle_t tElectricalAngle;     /* BAM32，按控制零位定义 */
    foc_scalar_t qElectricalSpeedPu;  /* 电转/秒 ÷ 电速度基值 */
    bool bValid;                      /* 本周期可交给 Core/速度环 */
} motor_electrical_feedback_t;
```

传给位置源的实时样本只包含确实需要的同步数据：本周期 `tCurrentAlphaBeta` 和与其对应的上一周期 `tVoltageModelAlphaBeta`。位置对象自身运行 Observer 并保存输出。配置阶段校验极对数、电速度基值、采样周期、来源绑定和算法参数，并预计算速度增益；ISR 不重复校验配置或除法。原始 Encoder `foc_position_t` 继续表示机械量，不直接进入 Motor 控制步骤。

| 来源 | 状态所有者 | 统一反馈如何产生 |
|---|---|---|
| 编码器 | App 持有 Encoder；有感适配层持有零位/换算执行参数 | 机械角 × 极对数 − 电零位；机械速度 × 极对数 / 电速度基值 |
| 霍尔等有感 | 外部模块 | 模块自行采集/插值，其适配层输出电角度、速度 PU、有效位 |
| Motor 硬拖 | Motor 内部启动状态 | Motor 生成开环电角度/速度，通过只读 ISR API 交给位置策略 |
| SMO 等无感 | Observer 算法状态 | 本周期 αβ 电流及对齐电压模型更新估计，输出转为电角反馈 |

`FOC_POSITION_GET(position_state, nowTick, sample, feedback)` 是 App 中的**编译期绑定调用宏**，统一返回 `foc_result_t` 并进入 `motor_position_Step()`。G431 原始机械读取已独立命名为 `FOC_SENSOR_POSITION_GET()`，避免将机械值接口与电角反馈接口混用。宏本身不保存状态，也不实现切换或融合。

## 3. 推荐的 App ISR 顺序

```c
motor_position_sample_t sample = {0};
motor_electrical_feedback_t feedback = {0};
motor_isr_phase_t phase = motor_IsrPrepare(&app->tMotor, nowTick, &sample);

if (phase == MOTOR_ISR_CONTROL_READY) {
    foc_result_t result = FOC_POSITION_GET(&app->tPosition,
                                            nowTick, &sample, &feedback);
    motor_IsrControlStep(&app->tMotor,
                         result == FOC_RESULT_OK ? &feedback : NULL);
} else if (phase == MOTOR_ISR_CAPTURE_ZERO) {
    foc_result_t result = motor_position_CaptureZero(&app->tPosition, nowTick);
    motor_CompleteAlignIsr(&app->tMotor, result);
}
/* Identify 仍在本周期 Motor Core 后运行。 */
```

以上是边界伪代码。`motor_IsrPrepare()` 在 RUN 中只做 ADC/Clarke 并提供同步快照；位置对象随后运行 Observer，影子源错误不影响 Encoder 控制源。`motor_IsrControlStep()` 只消费最终电角反馈、执行速度环/Core/PWM；反馈无效则锁存位置故障并安全停机。两个入口在同一 ISR 中连续调用，一次采样、一次 Core、一次 PWM 提交。ADC_CAL、IDLE、FAULT 和 ALIGN 的功率级动作继续由 Motor 处理。位置源、融合与 Motor Control 均不重采 ADC。

已选定的所有权是 App 按值持有 `motor_position_t`，位置对象按值持有 `foc_observer_t`。Motor 的 Prepare 输出本周期电流、上一周期电压模型和固定频率开环候选；位置对象在同一 ISR 内调用 Observer，再按初始化配置选取传感器或开环反馈。App 不直接读写 `tMotor.tInput`。以后扩展开环启动轨迹时，轨迹状态仍由 Motor 持有，继续通过 Prepare 样本交给同一个选源点。

## 4. 无法绕开的选择

| 冲突 | 方案 A | 方案 B | 建议 |
|---|---|---|---|
| 同周期 SMO 要当前电流，App 又要在 Core 前取角度 | 拆 `Prepare → Position → Control` | 现有 `motor_IsrStep` 前取上一周期样本，接受一周期延迟 | A；B 需要重新验证电压/电流对齐与动态性能 |
| Motor 管 ALIGN，Encoder 拥有机械零位 | Motor 发捕获事件，App 执行所选有感源 `CaptureZero` 并回告结果 | 保留 Motor 内 Encoder 调用 | A；B 使 Motor 继续依赖有感源 |
| 位置策略需要同时获得同步样本和 SMO 结果 | 位置对象按值拥有 Observer，并在 Prepare 后调用 | Motor 内调用 Observer，再向 App 输出快照 | 已选 A；Motor 不依赖具体估计器 |
| 硬拖由 Motor 内生成，位置策略在外 | Motor 暴露只读开环电角反馈 | Motor 内部直接决定控制角 | A；B 破坏单一角度入口 |
| 宏静态绑定与运行时融合 | 宏调用一个持有状态的策略函数 | 宏用 `#if` 固定单一来源 | 当前单源可 B；需要切换/融合时必须 A |
| 无效源的处理 | 位置策略尝试已定义的备用源，最终仍无效则 Motor 故障 | 任一源无效立即故障 | 影子源不影响控制；活动源的回退策略须在选源处明定 |
| 角度/速度来源一致性 | 策略同时选择或融合一对角度与速度 | 各自独立选择 | A；独立拼接会扰动速度环 |

已选定 App 按值持有 `motor_position_t` 策略状态。当前实现按初始化配置从 Encoder 和 Motor 开环候选中选择一个，交给 Motor/Core；配置启用 Observer 时，可在同一 Position Step 中计算影子估计，但它不参与控制角选择。将来添加 Hall、SMO 接管或融合时，由 Position 汇总候选并仍只向 Core 输出一组角度和速度。当前没有注入请求接口。

`FOC_POSITION_GET(...)` 是稳定的编译期绑定入口，展开后调用 `motor_position_Step(...)` 这类策略函数；不是用宏体写融合算法。策略对象按值存储源的执行状态，直接调用静态已知的算法函数，不引入每周期函数指针。若算法源由板级宏选择，宏只绑定具体实现。App 只调度该入口，融合仍封装在位置模块内部。改由 Motor 持有策略也能做到同样的多源计算，但会让 Motor 的 API 和状态随来源扩张，因此优先 App 所有。

已认可并固定：若启用 `foc_observer`，它是估计算法的**编译期抽象层**，后端分派不使用运行时函数指针。当前 `FOC_OBSERVER_BACKEND` 单选一个 Observer 后端；SMO 当前只提供影子估算。多个估计器并行、Hall 接入、融合和活动源切换均未实现；届时应由 Position 持有状态并显式调用算法，再按统一 BAM32 参考选择或融合角度与速度。首版多源验证应只对比，不应在缺少可信度条件时接管控制。

### SMO + HFI 的额外边界

SMO 与 HFI 可以共同向位置策略提供候选电角度，位置策略再输出唯一的电角反馈；不过 HFI 包含**注入执行**和**电流响应解调**两部分，不能只把它当作第二个被动 Observer。HFI 解调可放在 Observer/Estimator 算法对象中，注入电压则必须由 Motor/FOC 控制路径按周期合成并实际施加，且采样要与注入相位同步。接口需要表达注入请求/注入状态、同步电流样本及对应的施加电压，最后再交付角度、速度和有效性。

当前参数宏配置 `Ld=Lq=1000 µH`。若电机实际也近似无凸极，基于差分电感的常规 HFI 缺少主要位置线索；HFI 是否可用要看实测磁饱和凸极或其他可观测的角度相关响应，不能因为软件加了 HFI 后端就认定可低速运行。信号注入方法依赖几何或磁饱和凸极的电流响应来提取位置，[相关信号注入研究](https://ieeexplore.ieee.org/document/6426887/)与[SPM 机器的 HFI 研究](https://www.sciencedirect.com/science/article/abs/pii/S037847542030063X)都讨论了这一条件/非理想凸极影响。

现有 `foc_observer_input_t` 虽预留实际施加电压和母线电压字段，但 `foc_observer_Step()` 当前只向 SMO 传当前 αβ 电流和电压模型；没有注入命令、注入相位或 HFI 的控制交互。因此**现有 Observer 抽象不足以直接承载完整 HFI 闭环**。适合的边界是保留 Observer 作为估计算法编译期抽象，同时新增一条窄的 HFI 注入请求/反馈通道，由 Motor 合成注入电压；SMO 与 HFI 估计器可以在同一周期各自产出候选值，位置策略在低速选择/融合 HFI，在反电动势足够时选择/融合 SMO。具体切换门槛和角度连续性仍属位置策略。

若 HFI 只用于离线影子验证、控制链不施加注入，则无需注入请求通道，但也不能称作完整可工作的 HFI 闭环。若决定将来实现 SMO+HFI，需把“多估计器编译期组合”和“Motor 接受 HFI 注入请求并限幅合成”单列实施任务；不通过运行时函数指针实现。

切换/融合真正落地时还必须确定：启动来源和退出硬拖的速度门槛、SMO 有效与**可信**的区别、BAM32 最短弧角差、切换偏移或渐变、速度方向/连续性、活动源失效后的回退与故障条件。零速 SMO 不应因 `bValid` 为真就直接接管 Core。这些属于策略配置和运行状态，由选源模块持有，不由 App ISR 临时拼接。

## 5. 实施顺序

### Task 1：固定当前行为的验证点

**Files:** `foc/tests/motor_alpha_beta_test.c`、`foc/tests/motor_speed_pu_test.c`、`foc/tests/foc_app_encoder_command_test.c`。

- [x] 覆盖 Encoder 角度的极对数、零位与 BAM32 回绕；机械速度 `±10 turn/s × 7 / 100 = ±0.7 PU`；位置失效停 PWM；ALIGN 状态交接。Identify 调度保持在 Core 后。
- [x] 本边界相关 FLOAT/FIXED 主机测试通过；此前以关闭 SMO 的配置完成过 G431 全量构建。`service.c` 已包含 HAL 总头以取得 `HAL_RCC_GetPCLK1Freq` 声明。

### Task 2：拆分 Motor RUN ISR 数据与控制

**Files:** `foc/motor/motor.h/.c`、`foc/app/foc_app.c`、对应 Motor/App 测试。

- [x] 定义 `motor_position_sample_t`、`motor_isr_phase_t`、`motor_IsrPrepare()`、`motor_IsrControlStep()`；把现有 RUN 中的采样/Clarke 与速度环/Core/PWM 分开。非 RUN 状态仍由 Motor 自己处理。
- [x] App 调用顺序改为第 3 节；Identify 仍在本周期 Core 后。Prepare 失败和 Prepare/Control 错序均安全停机。

### Task 3：有感统一电角接口与 ALIGN 交接

**Files:** `foc/motor/motor_position.h`、新建 `foc/motor/motor_position.c`、`foc/app/foc_app.h/.c`、`peripheral/stm32g431/mdi/foc_adapter.h`、`foc/foc.mk`、对应测试。

- [x] Encoder 适配层持有零位和预计算换算增益，产出统一反馈；宏只绑定具体有感读取函数。重复 ALIGN 开始使旧零位失效，捕获失败不得保留旧有效状态。
- [x] Motor 发 `MOTOR_ISR_CAPTURE_ZERO`，App 调用有感捕获并回告；`motor_GetStatus().bElectricalZeroValid` 继续维持现有 Identify 所需的有感校准语义。
- [x] Encoder-only 的角度、速度和故障行为已有 FLOAT/FIXED 主机验证；Identify 的 ISR 调度仍在 Core 后。

### Task 4：纯无感与硬拖来源，另行验收

**Files:** `foc/motor/motor_position.h/.c`、`foc/motor/motor.h/.c`、`foc/app/foc_app.h/.c`、`foc/observer/foc_observer.h/.c`、对应测试。

- [x] 将 App 的 ready 门槛与 Position 的传感器 Provider 必填校验改为“所选来源已就绪”；默认 Encoder 行为保持不变。
- [x] Motor 在 Prepare 样本中发布固定频率开环候选；频率在 Init 换算为 BAM32 步长和速度 PU，Position 可静态选用，不反向依赖 Motor。
- [ ] 完整硬拖启动轨迹、Observer 输出统一 PU、实际 PWM 对齐验证和无感接管仍待实现。
- [ ] 依实机数据决定是否实现开环→无感切换/融合；覆盖无效源、角度连续性、速度方向和 ISR 预算。未验证可信度时 SMO 继续影子运行。

当前默认以 Encoder 电角反馈控制；显式配置可选 Motor 固定频率开环候选。Observer 启用时仍作为影子源运行。纯无感闭环启动、SMO 接管、SMO+HFI 和角度融合仍属于 Task 4 后续；`foc/docs/smo-fixed-parameter-integration-plan.md` 中关于角度源接入的后续阶段以这里确定的边界为准。
