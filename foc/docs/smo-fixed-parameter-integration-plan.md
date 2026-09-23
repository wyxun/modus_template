# 固定辨识参数接入 SMO Implementation Plan

> **当前状态：影子路径已接入，验收未完成。** 当前 `foc_config.h` 设定 `FOC_ENABLE_SMO=1`，Position 会用同步 αβ 样本更新 SMO 影子结果；SMO 仍不参与控制源选择。当前待办是按现有执行参数接口恢复 FLOAT/FIXED 测试、核实电压时序并完成实机验证。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用已辨识并回写的固定电机参数运行可诊断的 SMO，并为将来的可信角度闭环建立可验收边界。

**Architecture:** 当前 `foc_app_t` 按值组合 Motor、Position、Encoder 和 Identify。Motor Prepare 输出同步 αβ 样本，App 调用 Position 取得唯一电角反馈，再调用 Motor Control 执行速度环/Core/PWM。Position 已按值持有 Observer/SMO 状态并可计算影子结果；当前 `FOC_ENABLE_SMO=1`，但 SMO 不参与控制源选择。

**Tech Stack:** C11、STM32G431、20 kHz FOC ISR、FLOAT/FIXED 双数值后端、PowerShell 主机测试。

---

> 验收条件：使用已回写的 `Rs=2740 mΩ`、`Ld=Lq=1000 µH`，按当前 Position 边界验证 SMO 影子路径；保持 Encoder 控制源、Motor 生命周期和 Identify 辨识流程。无感接管另行验收。

## 1. 现状与选择

此前列出的路径是：① 直接用 SMO 角度替换编码器角度；② 由 Motor 拥有并影子运行；③ 提升为独立 MODUS Class。当前已确认由 App 按值持有 Position，估算器作为 Position 子状态；不让 Motor 拥有 Observer，也不为 SMO 增加独立 Class。直接接管仍需可信度与启动策略，当前阶段不做。

`identify` 与 SMO 的共同点是调用者拥有状态、初始化配置按值传入、ISR 与前台职责明确。两者的运行形式不同：Identify 有非阻塞 PT、启动/停止和结果消费；SMO 是每个 20 kHz 电流周期执行一次的纯算法状态机，不需要 PT、Shell 启动命令、硬件 ops 或单独 `MODUS_DECLARE_OBJECT`。

## 2. 明确接口边界

### 2.1 初始化参数

调用链：`motor_config.h` 宏 → `foc_app_cfg_t.tMotorCfg.tParams` → `foc_app_BindMotorConfig()` → `motor_Init()` → `foc_observer_Init()` → `foc_smo_Init()`。辨识结果不在运行时写回 SMO；要更新值，改宏并重新构建。

| 参数 | 本次值 | 作用 |
|---|---:|---|
| `wResistanceMilliohm` | 2740 mΩ | 电流模型 `Rs/Ld × Ts` |
| `wInductanceDMicroHenry` | 1000 µH | 主电流模型电感 |
| `wInductanceQMicroHenry` | 1000 µH | 当前 SMO 要求 `Ld=Lq`；用于验证该等电感模型的参数约束，不计算凸极交叉项 |
| `wVoltageBaseMillivolt` | 12000 mV | 电压 PU 基准，由 App 绑定 |
| `wCurrentBaseMilliamp` | 3500 mA | 电流 PU 基准，`motor_Init()` 取 `FOC_CURRENT_BASE_MILLIAMP` |
| `wHighFrequencyIsrHz` | 20000 Hz | 来自目标配置 `FOC_HF_ISR_HZ`，传给 `foc_smo_cfg_t.wSampleFrequencyHz`；SMO 初始化时据此换算周期和执行系数 |

`chPolePairs=7` 属于 Motor 的机械/电角速度换算，不参与当前 SMO 系数。现有 Identify 只辨识 `Ld`，不能据此声称 `Lq=1000 µH` 也已实测；当前配置将 `Lq` 设为与 `Ld` 相等，是等电感 αβ 模型的前提，不代表已经验证电机没有凸极。SMO 自身还需要截止角频率 `10000 rad/s`、滑模电压 `3500 mV` 和电流估计限幅 `1 PU`；这些是调参值，不是 Identify 的输出。

以当前参数计算，FLOAT/FIXED 后端的执行系数预期值为：`qVoltageCurrentGain≈0.1714286`、`qResistanceGain≈0.137`、`qBemfFilterNumerator≈0.2`、`qBemfFilterDenominator≈-0.6`、`qSlidingGain≈0.2916667 PU`、`qSpeedConversionGain≈20000`。这些系数只在初始化时计算并保存在 `foc_smo_exec_t`；SMO 单步不读取物理参数或配置。

### 2.2 每次 ISR 输入与输出

当前头文件定义了以下单步接口；SMO 已列入 FOC 源文件，但这不代表算法已通过主机测试或板级验证。

调用 `foc_smo_Step(smo, i_ab, u_ab_previous, output)`：

| 方向 | 值 | 单位与时序 |
|---|---|---|
| 输入 | `iα`,`iβ` | 本次 ADC 电流经偏移校准、归一化及 Clarke 变换后的 PU |
| 输入 | `uα`,`uβ` | 上一控制周期的电压模型命令 PU，当前来自 `tCore.tVoltageAlphaBeta`；必须验证它与本次电流样本对应的实际 PWM 周期 |
| 内部状态 | `êα`,`êβ` | SMO 根据电流误差和滑模低通计算的反电动势估计；**不由调用者传入** |
| 输出 | `tElectricalAngle` | 由 `atan2(-êα, êβ)` 得到的 BAM32 电角度 |
| 输出 | `qElectricalSpeedTurnsPerSecond` | 电角度差分得到的电转/秒；首个样本为 0 |
| 输出 | `bValid` | 现有实现仅表示反电动势非零，尚不是可用于闭环的可信标志 |

SMO 内部速度输出为电转/秒；若后续将 SMO 候选用于统一反馈，Position 负责按配置的电速度基值转换为 Motor 消费的 PU 速度，不改变 Motor/Core 的反馈单位。

算法单步不读取编码器、极对数、母线 ADC 或电机宏；可选的实际施加电压和母线输入目前在 `foc_observer_input_t` 中预留，但 SMO 后端未使用。当前 `uαβ` 是调制命令；当实际母线偏离 12 V、PWM 饱和或死区影响明显时，它与实际电机端电压不同。第一阶段要把这种限制写入诊断和验收，不能把命令电压称为已测端电压。

坐标轴核对：`Rs` 在理想 PMSM 模型中是各轴共用的定子电阻。`dq` 到 `αβ` 后的电感矩阵为 `R(θ)·diag(Ld,Lq)·Rᵀ(θ)`；当前 SMO 只实现 `Ld=Lq=Ls` 时的等电感简化模型，并在初始化时拒绝不相等的 `Ld/Lq`。它不支持一般凸极电机的角度相关电感矩阵。

### 2.3 所有权与安全边界

当前所有权为 `foc_app_t` 组合 `motor_t`、`motor_position_t`、`foc_encoder_t` 和 `identify_t`。`motor_position_t` 持有 Observer/SMO 状态；Motor 不拥有估算器，也不直接调用 Observer。App ISR 在 Motor Prepare 后调用 Position，再将唯一反馈交给 Motor Control。SMO 不提交 PWM、不访问 HAL、不持有 Motor 指针、不改变 Motor 故障状态。

## 3. 对照 `template_driver` 的结论

SMO 使用强类型的 `foc_smo_Init/Reset/Step` 算法接口，配置在 Init 时转换成 `foc_smo_exec_t`，Step 只消费 αβ 输入和执行状态。状态由 Position 按值拥有，固定速率入口类型明确，热路径没有函数指针或隐藏静态 runtime。`template_driver.h/.c` 里的硬件 ops、PT、Clock、Stop、GetStatus 是示例的可选能力，不应机械复制给 SMO。现有 `foc_observer_t` 是按 `FOC_OBSERVER_BACKEND` 编译期选择单个后端的薄封装；当前不需要再为 SMO 引入 `_Generic`、运行时注册表或额外分派层。未来并行运行 SMO 与 HFI、融合多个候选属于 Position 的扩展，不在本计划范围内。

需要完成的契约有三项：① SMO 采样频率来自目标 `FOC_HF_ISR_HZ`；② 电压与电流周期对齐且说明电压基准；③ 输出可信度及失效规则。若前台诊断需要读取影子输出，优先复用已有 waveform；只有现有通道不足时才增加最小只读快照，不把它作为 SMO 算法接口的前置条件。现有 `foc_observer_input_t` 的未使用字段可在实际施加电压接入时启用，不继续扩充推测性字段。

## 4. 可执行任务

### 阶段 A：固定参数影子运行

- [ ] **A1 配置链核对。** 保持不同电机可调参数集中在 `foc/app/motor_config.h`，`foc_app.c` 的 `FOC_APP_INIT_OBSERVER_CONFIG` 只组装 `MOTOR_CONFIG_SMO_*` 配置。确认 App 把 `FOC_HF_ISR_HZ` 传给 `foc_smo_cfg_t.wSampleFrequencyHz`；不要另加一个可与目标频率分叉的 `MOTOR_HF_PERIOD_NANOSECONDS` 宏。验证频率为零或超出 SMO 支持范围时，初始化会失败。
- [ ] **A2 参数与执行状态回归。** 重写 `foc/tests/foc_smo_test.c` 及相关运行脚本，以 `2740/1000/1000/12000/3500/20000` 初始化，FLOAT/FIXED 都检查 `tSmo.tExec` 中的电压电流增益、电阻增益、滤波系数、滑模增益和速度换算增益，以及 Reset 保留执行参数并清零历史状态。检查零电阻、电感或 PU 基准，以及 `Ld != Lq` 被拒绝。现有旧字段断言须删除或迁移，不保留当前模型不支持的异电感通用算法用例；测试通过后再从 `SKIP` 恢复脚本。
- [ ] **A3 确认电压时序。** 在 `foc/tests/motor_alpha_beta_test.c` 用连续两次 ISR 的不同电压命令证明第 `n` 次 SMO 所读的是先前提交的 `tCore.tVoltageAlphaBeta`，再依据 STM32G431 ADC 触发/PWM preload 实测确认它对应的实际施加周期。若存在额外一个周期延迟，只在 Motor 对象中保存所需的上一周期电压快照；不在 SMO 内猜测硬件延迟。
- [ ] **A4 输入失效与影子隔离。** 在 `foc_smo.c`/`foc_observer.c` 明确非有限输入和算法失效时输出清零或标无效的行为；确认 `motor_position.c` 在 Observer 出错时只撤销影子输出有效性，不影响 Encoder 活动源及 Motor 控制。Prepare 未返回 `MOTOR_ISR_CONTROL_READY` 时 App 不调用 Position/SMO；不为此额外增加 SMO 输入状态字段。
- [ ] **A5 生命周期重置验证。** `motor_position_Step()` 已在 `wRunGeneration` 改变时调用 `motor_position_ResetObserver()`，Reset 清除估计历史和公共输出，同时保留已校验的执行系数。增加回归检查，确认运行代次变化后首个样本不会沿用上一周期历史。若后续增加前台诊断快照，再定义 Stop/Fault 时快照的过期规则。

阶段 A 验收：将 `FOC_ENABLE_SMO` 设为 `0` 时 Encoder 控制构建和行为不变；设为 `1` 时运行影子估算，Core 继续使用 Encoder 反馈。确认现有 Position 路径正确传递样本，恢复 FLOAT/FIXED 测试，并在板上观察角度差、速度方向和 ISR 周期预算。角度差和当前 `bValid` 只能用于诊断。

### 阶段 B：角度可信度与实测电压

- [ ] **B1 定义可闭环的有效条件。** 在 SMO 对象内增加最小必要的资格状态：反电动势幅值下限、连续合格样本数、速度范围及角度跳变/失步拒绝；参数放在 `foc_smo_cfg_t`，在 `Init` 校验，`Reset` 清零。低速/静止、输入缺失或无效时输出 `bValid=false`。阈值必须根据阶段 A 的实测噪声和目标最低电频确定，不能用“反电动势非零”替代。PWM 饱和和母线异常只有在同步状态实际接入 Position/Observer 输入后，才纳入有效性判定。
- [ ] **B2 校准电压模型。** 对照当前 `foc_svpwm()` 的调制 PU、STM32G431 PWM preload 和母线采样，确定 `uαβ` 的实际电压系数与延迟。仅在板级母线采样的单位、更新时刻和有效期已验证后，把有效的施加电压/母线信息送入 `foc_observer_input_t`；无效采样时保持影子输出无效。用主机向量覆盖 12 V 基准、母线偏差和饱和情形。
- [ ] **B3 角度补偿。** 使用编码器影子对照测量 SMO 低通和 PWM 延迟造成的角度滞后，按电角速度补偿，并分别验证正反转和跨 BAM32 零点。补偿系数与适用速度范围写成显式配置，不在热路径临时估计。

阶段 B 验收：在指定电频区间内，稳定运行时 SMO 与编码器的角度误差和速度误差满足台架约定阈值；启停、反转、母线波动、限幅和缺样都会撤销可信标志。阈值需先由实际电机和负载的对照数据定稿。

### 阶段 C：需要无感闭环时单独实施

- [ ] **C1 启动策略（单独范围）。** 当前 Motor 已能发布固定频率硬拖候选，但没有加速曲线、限流启动或自动切换。若要无感闭环，另行定义从静止到 SMO 最低可信电频的启动轨迹、最大电流、超时和失败停机；复用 Motor 的候选通路，不把启动轨迹复制进 SMO。SMO 在静止时不能提供可信角度，不能直接替代 `FOC_POSITION_GET()`。
- [ ] **C2 位置策略切换。** 在 App 持有的 `motor_position_t` 中增加明确的编码器/SMO 来源状态与切换条件，保持 `foc_core_input_t` 不变；只有阶段 B 连续合格且相位差、速度一致时切换，失效时按既定安全策略停止或回退。App ISR 只调度统一接口，不直接写 `tMotor.tInput.tElectricalAngle`。
- [ ] **C3 闭环验证。** 分别测试编码器源、SMO 源和切换瞬间的电角度连续性、正反转、低速失效、Break/Stop/故障复位及最坏 ISR 周期。未完成 C1/C2 前不开放无编码器构建。

## 5. 验证命令与当前基线

Windows 主机测试使用 `D:/0_software/msys64/mingw64/bin/gcc.exe`。先把该目录加到当前 PowerShell 的 `PATH`，再运行：

```powershell
$env:PATH='D:/0_software/msys64/mingw64/bin;'+$env:PATH
powershell -ExecutionPolicy Bypass -File foc/tests/run_smo_test.ps1
powershell -ExecutionPolicy Bypass -File foc/tests/run_observer_contract_test.ps1
powershell -ExecutionPolicy Bypass -File foc/tests/run_motor_alpha_beta_test.ps1
```

FOC 源文件列表固定包含 `foc_smo.c`，`FOC_ENABLE_SMO` 在 `foc_config.h` 选择是否启用后端；关闭时链接器回收未引用的代码段。`run_smo_test.ps1` 与 `run_smo_hotpath_contract_test.ps1` 仍针对旧运行态字段并输出 `SKIP`；按当前 `foc_smo_exec_t` 重写测试后再恢复。其余主机测试不能替代实机相位、电压比例、角度可信度或无感闭环验收。

## 6. 变更范围

阶段 A 的主要工作是恢复 FLOAT/FIXED SMO 主机测试并验证当前 App→Position→Observer→SMO 配置与样本路径，涉及 `foc/tests/foc_smo_test.c`、相关 SMO 测试脚本及 `foc/tests/motor_alpha_beta_test.c`。仅在验证发现缺口时修改 `foc/app/foc_app.h/.c`、`foc/motor/motor_position.c` 或 `foc/motor/motor.c`；只有实测证明电压命令与采样周期错位时，才增加 Motor 侧的周期快照。不要为本阶段改动 Identify 算法或已回写的电机参数宏。

可以进入台架实机测试，但目前只算“有条件进入”，还不能认为电流、速度和辨识都已实机验证。
开始前建议先补齐或确认这三项：
1. 确认 SMO 不会拖垮 20 kHz 中断。 SMO 现在每次 FOC ISR 都运行一次；固件虽已构建、相关软件测试也通过，但还没有实测 ISR 执行时间。20 kHz 周期是 50 μs，需在板上测量最坏执行时间并确认留有余量。
2. 先不要用当前路径做 Ld 辨识。 它使用 100 Hz 发布的母线电压均值，没有样本年龄校验；母线波动时可能影响辨识结果。应先恢复同步采样或补上年龄校验。
3. 确认实机电机参数满足 SMO 初始化条件。 当前 SMO 要求 Ld == Lq；不满足时可能导致 App 初始化失败，连编码器控制也无法启动。现在配置中的两个值相等，实机参数则需要核实。
以上确认后，可以先做不带负载、低压限流的台架验证：检查电流零偏和采样、PWM 安全停机、编码器方向与对齐，再低速启动并逐步检查电流环和速度环。电阻辨识虽有软件测试覆盖，仍需实机核对结果；Ld 辨识暂缓。
