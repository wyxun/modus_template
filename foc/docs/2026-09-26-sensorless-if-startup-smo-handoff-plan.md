# 无编码器 I/f 启动与 SMO 接管 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 电机从静止出发，靠限流对齐、内部电频率斜坡和闭环电流起转；SMO 可靠后连续接管电角反馈，最终用 SMO 速度运行闭环，全程不依赖编码器。

**Architecture:** 沿用既有边界：Motor 只持有强制角/频率轨迹、功率级生命周期和电流/速度环；Position 持有 Encoder/SMO 等角度候选及选源、资格和交接融合，始终只向 Motor 输出一个 `motor_electrical_feedback_t`。App 只编排同周期采样、Position、Motor，不实现融合数学。接管时 Position 发布一次状态事件，Motor 据此跟踪速度 PI 并切入速度模式。纯编码器、纯强拖、强拖+SMO 共用同一条 Motor 输入边界；后续 HFI、磁链、EKF 作为 Position 候选接入，组合策略仍归 Position 管理。HFI 将来的注入/同步采样是独立执行能力，不能假装现有采样接口已经支持。

**Tech Stack:** STM32G431、C11、BAM32 电角度、FLOAT/FIXED 双后端、20 kHz FOC ISR、PowerShell/GCC 主机测试、现有波形与 RTT 数据。

---

## 当前落实状态（2026-09-28）

软件能力已按上述所有权边界落地，但**未完成带电台架验收**。默认配置仍是 SENSOR、启动斜坡为 0、自动接管关闭；本次代码不会让现有固件自动改用无感。新路径需显式将 `MOTOR_CONFIG_POSITION_SOURCE` 设为 `MOTOR_POSITION_SOURCE_HARD_DRAG`、将 `MOTOR_CONFIG_STARTUP_RAMP_STEPS` 设为非零，再编译、烧录、限流验证。`motor speed <pu>` 此时执行连续 PWM 的限流对齐和 I/f 斜坡，0.8 pu 是代码上界，不是实机已验证上界。

Position 已实现 Observer 资格、连续窗口、角度 BLEND、接管和丢失事件；Motor 只生成强制角并在事件到达时跟踪速度 PI。`MOTOR_CONFIG_OBSERVER_AUTO_TAKEOVER=false` 保持观察模式，最大强拖期限后故障停机；无台架重复数据前不得改为 true。BEMF/角差/速差阈值均为待实测候选。主机 FLOAT/FIXED 测试仅证明软件状态与安全边界，不证明电机稳定接管。

| 组合 | 当前归属/状态 |
|---|---|
| 纯编码器 | 原有 Position 传感器路径，Motor/Core 不变 |
| 纯强制角 | Motor 生成候选，Position 直接选择；新 I/f 启动入口有运行限时保护 |
| 强制角 + SMO | Position 资格/融合/接管；自动接管默认关闭，待实测 |
| 编码器 + SMO | 现有 SMO shadow 作台架对照；运行选角仍是编码器 |
| HFI + SMO、强拖 + 磁链、EKF | 未来在 Position 增加候选和策略，统一反馈边界不改。HFI 注入/同步采样需另行设计执行接口 |

## 范围和当前基线

本计划聚焦静止无感启动与 SMO 接管，衔接 `foc/docs/2026-09-25-stable-smo-closed-loop-plan.md` 的观测器验证阶段。它不覆盖零速无感转矩控制、飞车重启、反向不停机切换或弱磁；这些需要独立设计。现有主计划仍写 100 Hz 基值且以编码器开发基线为主，执行本计划时以当前代码和实测为准，不复制旧数值。

当前代码证据：

- `target/stm32g431/target.mk` 配置 FOC ISR 为 **20 kHz**；`foc/app/motor_config.h` 当前电速度基值为 **200 electrical Hz**。因此 1 pu 时每次 ISR 的角度增量是 **3.6 电角度**。10 kHz 是此前波形输出频率，不能代替 FOC 周期。
- `foc/motor/motor.c` 已有 `wHardDragAngleStepBam32` 与 `tHardDragAngle`，由初始化时给定的固定电频率逐 ISR 积分；默认配置频率为 0，位置源仍是 SENSOR。现有固定频率候选没有加速度、超时或自动接管。
- `foc/motor/motor_position.c::motor_position_CaptureZero()` 对 HARD_DRAG 返回 `FOC_RESULT_DISABLED`；`foc/app/foc_app.c` 在 ALIGN 到期后仍调用它，随后 `motor_CompleteAlignIsr()` 会将失败转成 ALIGN fault。必须修复纯无感对齐完成路径。
- `foc/app/foc_app.c` 在 `motor_IsrControlStep()` **之后**运行 Observer，以保留 ADC 触发到 PWM CCR 提交的既有时序余量。Position 下一次 ISR 消费上一周期 SMO 候选，按估计速度预测一个控制周期；台架需实测最坏 ISR 时序。
- `foc/observer/foc_smo.c` 的 `bValid` 只代表估计反电势非零；估算速度是相邻角度差。二者都不是可直接接管的可靠性证明。`foc/control/foc_pid.c` 已提供 `foc_pid_Track()`，用于接管时避免速度 PI 的 Iq 输出跳变。
- 实施时保留用户修改，不重置、覆盖或顺带整理。本文是计划，不代表已通过实机验证。

阶段门槛：先证明 ADC 电流标度/极性、限流、Break、`motor stop` 和有感电流环可用。此前 `0.9 pu` 实测速约 25.7 turn/s，符合 `0.9×200/7`；`0.95 pu` 曾测到约 49 turn/s，远高于期望。超速原因未清前，任何无感实验不进入 0.9 pu 以上区域。台架需允许自由转动、提供可用限流和断电手段。只有用户完成烧录、确认上电及该次实验后才发电机运行命令。

## 文件边界

| 文件 | 职责 |
|---|---|
| `foc/motor/motor_startup.h/.c`（新） | 每实例强制电角度/电频率斜坡和期限；不读 SMO、不选源、不融合角度；无隐藏静态运行态 |
| `foc/motor/motor.h/.c` | 按值持有启动子状态；限流对齐、PWM/故障生命周期、电流模式至速度模式交接 |
| `foc/motor/motor_position.h/.c` | Encoder/强制角/SMO 候选、同一速度 pu 量纲、SMO 资格与选源/融合状态；输出唯一反馈及一次性接管/失败事件；不决定启动斜率和功率级动作 |
| `foc/observer/foc_smo.c`、`foc/foc_types.h` | 输出可用于资格判断的反电势强度指标，同时保留 `bValid` 的原语义 |
| `foc/app/foc_app.c/.h`、`foc/app/motor_config.h` | 配置和 ISR 调度；App 不直接计算角度差或操作 SMO 内部状态 |
| `foc/app/foc_debug.c` | 使 `motor speed <pu>` 在无感配置下提交目标后触发完整启动，并只打印一次状态迁移/故障 |
| `foc/tests/` | 独立测试斜坡、对齐、资格、交接、故障、编排与 FLOAT/FIXED 构建 |

## Task 1：冻结现状，建立无编码器失败测试

**Files:** `foc/tests/sensorless_startup_test.c`（新）、`foc/tests/run_sensorless_startup_test.ps1`（新）、`foc/tests/foc_app_encoder_command_test.c`（扩展）、`foc/tests/motor_position_boundary_test.c`（扩展）。

- [ ] 写主机测试重现三个当前缺口：HARD_DRAG 对齐到期会因 `CaptureZero` 返回 DISABLED 而故障；固定频率不会从零斜坡；Observer `bValid` 即使为真也没有资格与接管状态。测试使用 fake ADC/PWM，不访问真实外设。
- [ ] 给新 runner 沿用 `foc/tests/run_motor_speed_pu_test.ps1` 的 GCC include、链接和 FLOAT/FIXED 两次运行方式。执行 `powershell -ExecutionPolicy Bypass -File foc/tests/run_sensorless_startup_test.ps1`，预期至少上述三个断言在实现前失败；旧 `run_motor_position_boundary_test.ps1` 和 `run_motor_speed_pu_test.ps1` 仍通过。
- [ ] 记录无感配置能否在不初始化 Encoder 的情况下达到 `foc_app_t.bReady`，并添加反例：无感启动配置选择 `FOC_OBSERVER_BACKEND_NONE` 时初始化必须拒绝，不允许假成功。单独的固定频率开环实验继续使用现有 HARD_DRAG 测试配置。

## Task 2：实现限流、连续 PWM 的 I/f 启动

**Files:** 新建 `foc/motor/motor_startup.h/.c`；修改 `foc/motor/motor.h/.c`、`foc/app/foc_app.h/.c`、`foc/app/motor_config.h`、`foc/foc.mk`；测试 `foc/tests/sensorless_startup_test.c`。

- [x] Motor 的 `motor_startup_t` 只保存强制 BAM32 角及频率斜坡；Motor 负责对齐、电流指令和期限。Position 独立保存 `PRIMARY → BLEND → OBSERVER/FAILED` 与资格计数；两个对象只以候选、状态事件和统一反馈交互。
- [x] 用 Q16 BAM 增量和 Q31 速度累加实现有界斜坡；Start 时预计算除法，ISR 只做有界加法、常量缩放和环绕。新入口只允许正向 `0 < target ≤ 0.8 pu`。旧固定频率路径暂留兼容已有命令和测试，确认无使用者后再删。
- [ ] 斜坡累加器使用 `int64_t` 保存带小数位的 BAM32 每 ISR 增量，而不是逐 ISR 对 Q 格式 pu 速度相加；后者在小加速度下可能被量化成零。控制角仍是 `foc_angle_t`，更新只做有界加法与位移。启动模块向外提供以下语义接口，具体头文件同时写清频率和加速度单位：

```c
foc_result_t motor_startup_Init(motor_startup_t *ptStartup,
                                const motor_startup_cfg_t *ptConfig);
foc_result_t motor_startup_Start(motor_startup_t *ptStartup,
                                 foc_scalar_t qTargetSpeedPu);
foc_result_t motor_startup_IsrStep(
    motor_startup_t *ptStartup,
    motor_electrical_feedback_t *ptForcedCandidate);
void motor_startup_Stop(motor_startup_t *ptStartup);
```
- [ ] 增加单次原子提交目标的无感启动入口，例如 `motor_StartSensorlessSpeed(ptMotor, qTargetSpeedPu)`。先按现有电流 PI 在固定电角度施加受限 `Id_ref`；对齐完成后**保持 PWM 有效**，切到 `Id_ref=0` 与受限 `Iq_ref`，再启动频率斜坡。现有有感 `motor_RequestPositionCalibration()` 继续走 Encoder CaptureZero；无感流程不调用它。
- [ ] 强拖时 Motor 命令模式保持 `FOC_MODE_CURRENT`，速度 PI 不以指令频率充当真实速度闭环反馈。对齐和强拖 Iq 上限单独受实际电流标度校验；PWM、ADC、数学、Break、超时任一失败立即锁存故障并安全停 PWM。
- [ ] 运行 `run_sensorless_startup_test.ps1`、`run_motor_speed_pu_test.ps1`、`run_motor_position_boundary_test.ps1`；预期 FLOAT/FIXED 全过。使用 `make BUILD=debug TARGET_CHIP=stm32g431 -j4` 编译，预期无错误且 ISR 路径无日志/阻塞等待。

## Task 3：同周期 SMO 候选和“可靠”资格

**Files:** `foc/app/foc_app.c`、`foc/motor/motor_position.h/.c`、`foc/observer/foc_smo.c`、`foc/foc_types.h`、`foc/motor/motor_startup.h/.c`；测试 `foc/tests/sensorless_handoff_test.c`（新）、`foc/tests/foc_smo_test.c`、`foc/tests/foc_app_encoder_command_test.c`。

- [ ] RUN ISR 固定为 `Motor Prepare → Position 选源/融合 → Motor Control/PWM → Position 更新 Observer`。Observer 候选延迟一个 ISR，Position 用估计速度补偿该延迟。Observer 失败在强拖阶段清资格；接管后安全停机，不伪装成编码器回退。
- [ ] App 只编排，数据流按以下调用顺序写在一个 ISR 分支中；Position 将 SMO 原始电转/秒转换为反馈 pu 并决定最终反馈：

```c
ePhase = motor_IsrPrepare(&app->tMotor, &sample);
if (ePhase == MOTOR_ISR_CONTROL_READY) {
    eResult = motor_position_Step(
        &app->tPosition, nowTick, &sample, &controlFeedback);
    /* Position 状态事件仅通知 Motor 跟踪速度 PI/切换模式。 */
    motor_ApplyPositionEventIsr(&app->tMotor,
        motor_position_TakeEvent(&app->tPosition),
        controlFeedback.qElectricalSpeedPu);
    motor_IsrControlStep(&app->tMotor,
        eResult == FOC_RESULT_OK ? &controlFeedback : NULL);
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    /* 所有模式的 Observer 都在 PWM 提交后更新下一周期候选。 */
    motor_position_ObserverStep(&app->tPosition, &sample);
#endif
}
```
- [ ] Position 内部维护 `motor_electrical_feedback_t` 形式的 SMO 候选，`qElectricalSpeedPu = qElectricalSpeedTurnsPerSecond / qElectricalSpeedBaseTurnsPerSecond`。保留 `bValid` 表示算法输入/输出有效，另由 Position 维护可靠性，不能把非零反电势等同于锁定。
- [ ] `motor_position_Step()` 继续是唯一选角边界；仅新增一次性状态事件接口，所有无效输出先清零。Motor 只消费统一反馈和 `OBSERVER_ACTIVE/LOST` 等状态事件，不读取 SMO 内部成员，也不接受两路候选：

```c
motor_position_event_t motor_position_TakeEvent(
    motor_position_t *ptPosition);
void motor_ApplyPositionEventIsr(motor_t *ptMotor,
                                motor_position_event_t eEvent,
                                foc_scalar_t qObservedSpeedPu);
```
- [ ] 在 SMO 输出中增加轻量反电势强度指标，可用两轴绝对值之和，明确 pu 量纲并采用饱和算术；资格器同时检查非有限值、方向、滤波后的估算速度与强制频率差、圆周角差、角度连续性和电流/电压限幅。`foc_angle_diff(θ_smo, θ_force)` 使用最短弧；**带载 I/f 下两角可存在稳定转矩角，不要求差值为零**。
- [ ] 资格须连续保持一个配置窗口，任一条件失效就清计数。目前配置候选为最小 0.15 pu、滤波速度相对强制速度误差 ≤10%、原始速度误差 ≤15%、角差 ≤45 电角度、保持 200 ms；这些是**待实测候选**，不是本电机已经通过的接管值。自动接管配置默认关闭，待台架三次重复窗口证明可靠后才允许打开。此策略完全位于 Position；Motor 不因将来更换为 HFI、磁链或 EKF 而修改。
- [ ] 主机测试覆盖错误 `bValid`、低反电势、反向估速、速度尖峰、角度跨 0/1 圈、持续合格、一次坏样清资格、超时、运行代次更替，以及 Observer 当前电流/上一电压的时间顺序。运行 `run_smo_test.ps1`、`run_observer_contract_test.ps1`、新 `run_sensorless_handoff_test.ps1`，预期 FLOAT/FIXED 通过。

## Task 4：角度连续交接和速度 PI 无扰进入

**Files:** `foc/motor/motor_position.h/.c`、`foc/motor/motor.c`、`foc/app/foc_app.c`、`foc/app/foc_debug.c`、`foc/tests/sensorless_handoff_test.c`。

- [ ] Position 的 `BLEND` 期间 Motor 强制角继续积分。Position 每次对 `θ_smo` 与当前控制角计算 BAM32 最短弧角差，对修正量限速；同时让混合比例按固定 ISR 计数从 0 增至 1。输出角及速度作为同一反馈对，不拼接来自不同采样周期的值。测试环绕附近的角度连续性、正反号和单步最大修正量。
- [ ] 交接窗口内锁定失效、当前限流/调制饱和、角差突变或等待超时，立即安全停 PWM 并记录可区分的启动故障码。无编码器版本不尝试回退到不存在的传感器。
- [ ] 达到 SMO_ACTIVE 时，把目标速度先置为**当前可信 SMO 速度**，以当前 Iq 指令调用已有 `foc_pid_Track()`；随后按闭环速度斜坡走向用户目标，才执行 `FOC_MODE_SPEED` 的周期 PI。主机测试首个速度环周期的 Iq 跳变量、随后限流、SMO 丢失停机和再次启动清状态。
- [ ] 无扰进入时先跟踪当前 Iq，再变更控制模式；主机断言首次速度 PI 输出与交接前 Iq 的差不超过一个允许的离散量化步：

```c
ptMotor->tCommand.qSpeedReferencePu = qTrustedSmoSpeedPu;
foc_pid_Track(&ptMotor->tSpeedPi,
              ptMotor->tCommand.tCurrentReference.qQ,
              qTrustedSmoSpeedPu, qTrustedSmoSpeedPu);
ptMotor->tCommand.eMode = FOC_MODE_SPEED;
```
- [ ] 无感配置的 `motor speed <pu>` 调用原子启动入口，拒绝在 RUN 中重启；`motor status` 可读取 `ALIGN/RAMP/WAIT_LOCK/BLEND/SMO_ACTIVE/ERROR`、强制/SMO 电频、资格计数和故障。状态迁移只打印一次 `I`，周期统计用 `T`，ISR 中不写日志。旧有感命令和识别命令回归不得退化。
- [ ] 重跑启动、接管、Encoder 和 PID 主机测试，再构建 debug/release；检查 20 kHz ISR 的最大执行时间、ADC 触发到 CCR 延时以及 `bottomLate/bad` 计数。若 ISR 预算变差，先优化资格与角度融合计算，不牺牲保护。

## Task 5：逐级台架验收并确定实际接管门槛

**Files:** `foc/docs/foc-tuning-results.md`（记录）、`foc/app/motor_config.h`（只在单变量实测后更新）、波形原始 CSV（各次单独保存）。

- [ ] 在编码器仅作**外部对照**的台架先检查无感固件初始化、`Id_align=0.005 pu` 与 `Iq_start=0.02 pu` 的真实电流，确认实际相电流标度、限流、停机、Break、母线与故障。初次短时启动仍以用户已接受的 0.02 pu 电流指令为上限；电源显示的是输入电流，不用它直接替代相电流标定。
- [ ] 先保持自动接管关闭，用 0.2 pu 作为**测试目标而非接管结论**；缓慢斜坡、短时运行，记录强制角/频、SMO 角/估速、反电势强度、Id/Iq、Vq、母线和 fault。用采样序号对齐同一 ISR；每档至少三次，取最差的 200 ms 连续窗口。若 0.2 pu 时 SMO 不合格，在已验证的安全范围内逐档提高目标，不能因为反电势变平滑就跳过角差检查。
- [ ] 只有某一实测速档反复满足 Task 3 的资格条件、且阶跃/负载下无明显失步，才把门槛写回配置并启用自动接管。首轮接管仍使用短运行时间和保守电流上限；观察接管前后 `θ_control`、Iq/Vq、机械转速和故障。出现反向冲击、失控加速、过流、明显振动、温升或角差扩大，立即停机，保留 CSV，不继续提速。
- [ ] 通过冷启动、热启动、不同起始转子角、轻载与目标负载、低/额定母线、停机/再次启动和故障注入。无编码器配置必须从上电至速度闭环不调用 Encoder 读取；编码器断开仍能正常初始化。`0.95 pu` 异常排清前只验收其下方已证明安全的范围，绝不把 0.9 pu 以下成功外推到全速域。
- [ ] 每轮报告固件 commit/工作区状态、配置、负载、母线、相电流、电频、原始 CSV、资格/接管计数、丢样、ISR 最坏时延和“通过/失败/证据不足”。最终门槛仅以本机重复数据定稿；无感接管成功不等于全速域或零速无感已通过。

## 实施顺序与停止条件

依次完成 Task 1 → 2 → 3 → 4 → 5。Task 2 结束时已有可单独测试的无编码器限流 I/f 启动；Task 3 的资格器默认只观察；Task 4 才允许自动接管；Task 5 决定实际阈值。一个阶段失败时停在该阶段分析，不同时改 PI、SMO 参数、启动斜率和 ADC/PWM 时序。

每次代码改动先补能失败的主机测试，再用最小改动通过测试。Motor/Position 的对象运行状态放入对象成员，主循环和 ISR 不使用隐藏文件静态状态；热路径保持固定上界和直接调用。实机启动、烧录和带电切换按用户当次现场授权与台架状态执行。
