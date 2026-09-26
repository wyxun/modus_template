# 稳定 SMO 闭环 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在保留编码器安全基准的前提下，依次验收电流环、速度环、SMO 影子估算、带保护的角度接管，最终得到规定速度范围内可重复、可退回的 SMO 闭环。

**Architecture:** `foc_app_t` 编排 Motor、Position 与编码器；Motor 只消费统一的电角反馈，SMO 仍由 Position 持有。先以编码器闭环建立控制与测量基线，SMO 影子数据达到门槛后才允许 Position 选择估算角度；静止启动和低速不可观测区独立处理。每阶段只改一个主要变量，并在同一母线、电机、负载和采样条件下与前一版本比较。

**Tech Stack:** STM32G431、C11、20 kHz FOC ISR、10 kHz RTT 波形、Python CSV 分析、PowerShell 主机测试、编码器对照台架。

---

## 0. 范围、当前证据与安全边界

本文件是跨电流环、速度环、观测器和来源切换的**主验证计划**。各阶段有独立交付和停/走门槛；尚未拿到实测值的门槛不伪装成已证明的设计指标。现有未提交的 FOC 和工具修改属于工作基线，不重置、不顺手清理。实施每一阶段时再按本阶段可测契约写小范围测试和代码；本文件不授权烧录、启动电机或自动切换角度。

旧固件的 `motor step 0.05 100`、IqRef/Iq 10 kHz CSV 时间戳和样本序号可作历史记录，但 `Iq` pu 受 ADC 对齐/计数基准不一致影响，不能用于 PI 结论。源码已修正 MDI 样本为 12 位右对齐值；新固件烧录并验证标度前不重复 0.05 pu 阶跃，旧 `Iq`、αβ 和 SMO 物理量均需重采。

每次上电实验的前置条件：电机固定在可靠台架、可自由转动且无人手扶转子；12 V 稳压电源具有限流和紧急断电手段；确认驱动板、电流极性、编码器方向、对齐、故障与 `motor stop` 有效。所有阶段保留过流/Break/位置故障保护。只有用户明确表示已上电并同意该次试验，才执行电机命令；先小电流、短脉冲，异常噪声、振动、温升或 fault 立即停止。调试日志仅按该阶段所需打开，避免 RTT 丢波形；记录固件版本、参数、母线、负载和 CSV 路径。

## 1. 先统一单位和参数，而不是先改 PI

代码证据：`foc/app/motor_config.h` 配置 `MOTOR_CONFIG_BASE_ELECTRICAL_HZ=100.0f`；`foc/app/foc_app.c` 把它传入 `qElectricalSpeedBaseTurnsPerSecond`；`foc/motor/motor_position.c` 用机械转/秒乘 `7/100` 得电速度 pu。因此 **1 pu = 100 电周期/秒 = 100 Hz = 628.3 电 rad/s = 36000 电角度/秒，约 857 机械 rpm（7 极对）**。0.5 pu 约 429 rpm；规格书 2600 rpm 约 3.03 pu。用户此前所说“1 pu 是 100 电角度/s”与当前代码定义不同；先以编码器读数和短时实验确认，不以口述单位设计速度 PI 或 SMO 门槛。

物理量来源分开登记：规格书相电阻 2.55 Ω、相电感 0.86 mH、7 极对、额定 0.5 A、最大 2 A、惯量 370 g·mm²、磁链 0.0035 Wb；现用工程值 Rs=2.74 Ω、Ld=Lq=0.8 mH、电压基值 12 V、电流基值 3.5 A、FOC 20 kHz，速度环 1 kHz。规格书的线间电阻 5.1 Ω 不直接代入相电阻公式；磁链/Ke/电流定义需要交叉核对。0.05 pu 电流按工程基值是 0.175 A；0.1 pu 是 0.35 A。注意当前宏名 `MOTOR_CONFIG_MAX_SPEED_REFERENCE_PU=100` 容易误读：`foc_app_BindMotorConfig()` 把这个 100 除以 100 Hz 基值，Motor 实际收到的上限是 1 pu（100 电 Hz）。计划中的单位检查要同时记录这个换算，不能把 100 当成 100 pu。

### Task 1：建立可复现的量纲表和台架记录

**Files:** `foc/app/motor_config.h`、`foc/hal/foc_port_config.h`、`target/stm32g431/target.mk`、`foc/motor/motor_position.c`（只读）；新建 `foc/docs/foc-tuning-results.md` 记录每轮结果。

- [x] 核对编码器机械速度单位、`motor speed` 参数、速度 PI 的 pu 输入/输出以及 20 kHz/1 kHz 周期；在结果文档写出 `1 pu=100 electrical Hz=857.14 mechanical rpm` 的换算和规格书值/工程值的区别。
- [x] 为现有两份 CSV 记录脉冲命令、`sample_index` 局部连续性、真实采样间隔和实际可得的控制量；速度、fault 和 motor status 未包含在这些历史 CSV/记录里，继续测试时补齐。全文件早期缓冲区缺口没有误判为阶跃窗口丢点。
- [x] 用 `powershell -ExecutionPolicy Bypass -File foc/tests/run_motor_speed_pu_test.ps1` 验证速度换算回归；FLOAT 与 FIXED 均 PASS。后续硬件读数若与定义不符，暂停后续调参，先修单位链。

## 2. 电流环：先确认反馈，再用物理模型作参数候选

用户给出的连续域经验式 `Kp=ωc Lq`、`Ki=ωc Rs` 对理想电压输出/安培输入的 RL 电流对象适用。当前代码中的无量纲比例增益和**每个 50 µs 电流周期的积分步长**应按以下式换算，不能把连续域 `Ki` 直接填入 `MOTOR_CONFIG_CURRENT_PI_KI_TS_PU`：

```text
Kp_pu   = ωc · Lq · Ibase / Vbase
KiTs_pu = ωc · Rs · Ibase · (1/20000) / Vbase
```

以现值 `Kp=.20, KiTs=.005` 及 `Rs=2.74Ω, Lq=.8mH` 反推，比例项对应约 857 rad/s，积分项只对应约 125 rad/s；两者不匹配是可检验假设，不等于已证明根因。若保留 Kp=.20 且采用理想零极点抵消，候选 `KiTs≈.034`。实机还受采样滤波、PWM 延迟、反电势、量化和限幅影响，先试较小步进，不能直接宣称 .034 为最终值。现有电流 PI 限幅 ±.55 pu；用户观察 Vq 约 .1 pu，但须在脉冲全过程核对，而非依据停机快照断言从未饱和。

### Task 2：修正和验收阶跃采集工具

**Files:** `tools/foc_current_step_test.py`、`tools/tests/test_foc_current_step_test.py`、`foc/app/foc_debug.c`；原始 CSV 存 `build/` 且不覆盖。

- [x] 用当前两通道 CSV 加入分析器回归：读取 `sample_index`、逐样本时间、`IqRef` 与 `Iq`；对阶跃窗口丢样失败，不从文件行数推断时间。时间倒退、必需字段不全也会失败。
- [x] 修改分析工具支持实际两通道 CSV 和可选速度列；将 10–90% 定义为相对实测脉冲末段平台值，使用 CSV 时间计算脉宽/上升时间，并在采集基线计数时忽略空波形行。默认选择相邻 MStudio 工程中写 `sample_index` 的新版 AITrace；缺失时回退至仓库副本。`python -m unittest tools.tests.test_foc_current_step_test -v` 通过 9 项测试。三份完整 CSV 已用新分析器重算。
- [x] 按对齐 → `motor current 0 0` → 先启动 CSV 采集 → `motor step 0.05 100` 的顺序完成本轮；脉冲后固件自动停 PWM，最终 `motor state=2`、`fault=0`、`pwm=0`。已累计三份同条件原始 CSV 并附分析结果；三次平台/上升响应存在差异，不能判为 PI 已验收。

### Task 3：区分电流测量噪声、机械变化和 PI 跟踪

**Files:** `foc/app/foc_debug.c`、`foc/app/foc_app.c`、`foc/tests/run_foc_waveform_channels_test.ps1`、`tools/foc_current_step_test.py`；变更仅限必要的可观测量，测完恢复精简通道。

- [x] 已计算三轮 Iq 基线的零参考交流 RMS、峰峰值，阶跃末段中值、真实时间轴 10–90% 上升时间及指令过冲/平台波动。三轮均未超过 0.05 pu 指令，但平台仅 0.036–0.045 pu；速度/Vq 不在 CSV，响应差异的归因仍待补证。
- [ ] 仅在需要区分饱和时临时增加 `VqPI` 一通道，先运行 `powershell -ExecutionPolicy Bypass -File foc/tests/run_foc_waveform_channels_test.ps1`，再做同一短脉冲；记录动态峰值与贴限持续时间，完成后恢复两通道，避免 RTT 丢帧。
- [x] 已采集旧固件下 Clarke 后、Park 前的 `Ialpha/Ibeta` 和原始三相 ADC：捕获时间、样本序号和 ADC 原始码仍可追溯，但旧 `Iq`/αβ pu 因 ADC 标度不一致而无效。左对齐 JDR 与 1390 右对齐计数基准已在 MDI 读入处修正；详情见 `foc-tuning-results.md`。
- [ ] 烧录修正固件后，以不超过 0.01 pu 的短电流给定核对方向、反馈标度与 fault，再重新采集精简电流波形。通过前不运行 0.05 pu 阶跃，不调 PI/SMO。
- [ ] ADC rank 时差暂列次要假设；只有标度验证后波形仍显示相位/扇区相关误差，才做 U rank 对照。不先改 ADC 时序。

### Task 4：一次只改一个电流 PI 变量

**Files:** `foc/app/motor_config.h`；回归 `foc/tests/run_core_step_test.ps1`、`foc/tests/run_motor_alpha_beta_test.ps1`、`foc/tests/run_motor_reference_limit_test.ps1`。

- [ ] 冻结 `Kp=.20`、电机参数和电流限幅，保存 `.005` 基线；在相同条件下只试 `KiTs=.010`，对比至少三次短脉冲的局部连续 CSV、噪声、上升时间、最后 10 ms 跟踪偏差、Vq 峰值、振荡和 fault。
- [ ] 只有 `.010` 比基线改善且不增加持续振荡/饱和，才依次试 `.020`，必要时试 `.030`；每档重建、主机测试、由用户烧录并确认上电后实机复测。出现电流/电压限幅、持续振荡、异常温升或故障，立即停止并回到上一个已验证参数。
- [ ] 积分步长定下后再评估是否有必要单独改变 Kp；正负 Iq 小阶跃、起止转速不同的 100 ms 窗口均复测。阶段通过标准：三次采集的局部缺样为零、无 fault/持续饱和、零参考噪声已解释、实际 Iq 在短窗口内可重复地接近指令；记录上升时间、偏差和误差带，不以一张图或一次平滑峰值定参数。

## 3. 速度环：只有电流环通过后才调

用户公式 `Kp=2Jωs/(3Npφ)` 是基于 `Kt=(3/2)Npφ` 的物理域比例候选；图片中的 `Ki=ωs/10` 需确认是积分/比例比值、积分零点还是某一实现的离散参数，量纲不明时**不直接填入**工程 `SPEED_PI_KI_TS_PU`。先核对 `J=370 g·mm²=3.7×10⁻⁷ kg·m²` 是否是转子还是含负载惯量，确认磁链 0.0035 Wb 与电流幅值定义，按机械/电角速度换算到 `IqRef pu / speed-error pu`，再换算 1 kHz 的 KiTs。速度环目标带宽必须显著低于实测电流环、编码器速度测量带宽和机械共振范围。

### Task 5：建速度环基线与单位换算测试

**Files:** `foc/motor/motor.c`、`foc/app/motor_config.h`、`foc/tests/motor_speed_pu_test.c`、`foc/docs/foc-tuning-results.md`。

- [ ] 在不改速度 PI 的情况下，用 `motor speed` 的小阶跃核对指令 pu、编码器实际电速度 pu、IqRef 和 Vq；先在低于 0.5 pu 的安全速度段试，逐步增加，记录稳态误差、上升/稳定时间、振荡与限流。观察当前 `IqRef` ±.10 pu 钳位是否长期命中。
- [ ] 将速度公式的物理输出 A 和机械 rad/s 输入显式换成工程 pu，并在 `motor_speed_pu_test.c` 添加正反向、1 pu=100 电 Hz、输出限幅的断言；运行 `powershell -ExecutionPolicy Bypass -File foc/tests/run_motor_speed_pu_test.ps1`，预期 FLOAT/FIXED PASS。
- [ ] 若速度跟踪受 `.10 pu` 电流限幅限制，先核对实际电机额定 0.5 A、母线与负载，再制定新的电流上限；不能只增大速度 PI 或把软件 `MAX_SPEED_REFERENCE_PU=100` 当作许可。只有确认非限流导致的动态不足，才按模型候选一次改 Kp 或 KiTs 并复测。
- [ ] 无载与规定负载分别验收速度阶跃、扰动恢复、正反转、降速与停机；始终保留编码器反馈。速度环未通过前，不让 SMO 控制 PWM。

## 4. SMO 影子验证：先证明估算可用区间

当前 `FOC_ENABLE_SMO=1` 仅运行影子路径；`motor_position.c` 仍使用编码器控制。`MOTOR_CONFIG_SMO_BEMF_CUTOFF_RADIANS_PER_SECOND=4000`、滑模增益 5000 mV；先保持不变。现有 `bValid` 或非零反电势不足以证明角度可闭环，尤其静止与低速反电势不可观测。既有 `SMO rms/cond/bin/sec` 统计有价值，但“某扇区坏点多”必须除以该扇区总采样数，且要在相同反电势幅值档内比较，不能只看坏点个数。

### Task 6：观测器模型与时序契约

**Files:** `foc/observer/foc_smo.c`、`foc/motor/motor.c`、`foc/motor/motor_position.c`、`foc/tests/foc_smo_test.c`、`foc/tests/motor_alpha_beta_test.c`。

- [ ] 检查 SMO 消费的电流 αβ 与电压 αβ 对应的 PWM 周期、母线标幺、调制/死区及钳位；用两周期不同命令的主机向量证明时序，板上以 ADC 触发→CCR 最坏延迟、底点裕量和 late/bad 计数验证。若电压模型错位，先修时序，暂不调 SMO 增益。
- [ ] 恢复 `run_smo_test.ps1` 与 `run_smo_hotpath_contract_test.ps1` 中仍为 `SKIP` 的旧契约，覆盖 FLOAT/FIXED、初始化、Reset、方向、跨周、失效、非有限值与限幅；运行 `powershell -ExecutionPolicy Bypass -File foc/tests/run_observer_contract_test.ps1` 和相应 SMO 脚本，预期非 SKIP 的 PASS。
- [ ] 以编码器控制记录 0.5、1.0 pu 及继续提升到安全速度段时的正反转影子数据：环绕角误差、误差均值/RMS/95 分位、连续超过 25° 的最长时间、反电势幅值和电流估计误差。每档至少三个稳定窗口；出入加速段另记，不混入稳态统计。

### Task 7：设定最低可信速度、相位补偿与资格门槛

**Files:** `foc/motor/motor_position.c`、`foc/motor/motor_position.h`、`foc/observer/foc_smo.c`、`foc/observer/foc_smo.h`、`foc/app/foc_app.c`、对应主机测试。

- [ ] 用同一幅值档的扇区坏点率、低速 BEMF 噪声底和角误差分布定位最低可信电频；把最低电频、BEMF 幅值、连续合格周期数、最大角跳、速度方向一致性设成候选有效条件。跨门槛及失效时要有迟滞，不能以一次 `bValid` 翻转直接接管。
- [ ] 按速度测得的固定相位偏差与滤波/PWM 延迟建立角度补偿，只在已测速度范围内启用；正反转、BAM32 环绕和母线变化回归。补偿不得掩盖大幅随机跳角。
- [ ] 门槛通过的定义：在计划接管的速度/负载/母线范围内，角误差 95 分位和连续超限时间均低于先与台架风险相容的角度限值，且 SMO 资格信号无高频抖动；一旦失效能在规定 ISR 数内撤销资格。验收数值应由本阶段数据和可接受转矩扰动共同定稿，并写入结果文档，在此之前保持影子模式。

## 5. 接管与启动：独立于调参实施

### Task 8：先在编码器在场条件下验证来源切换

**Files:** `foc/motor/motor_position.c`、`foc/motor/motor_position.h`、`foc/app/foc_app.c`、`foc/tests/motor_position_boundary_test.c`。

- [ ] 为 Position 增加明确的 `ENCODER → SMO_CANDIDATE → SMO_ACTIVE` 状态；只有 Task 7 连续合格且两角度差/速度方向在限值内才允许切换。切换前后电角度、Iq 和 Vq 连续；Motor/Core 不读取来源类型。
- [ ] 在编码器仍在线的台架先做短时、可立即停机的 SMO 接管；编码器只作独立对照/保护。失去 SMO 资格、角度分歧扩大、Break 或母线异常时按显式策略退回编码器或停 PWM；没有编码器时不得假装可安全退回。主机测试覆盖来源状态、跨周、Stop/Fault/Reset 和回退，再做实机限流试验。
- [ ] 逐级扩展稳态速度、加减速和规定负载；每级记录真实控制来源、故障、Iq/Vq、角差与 ISR 最坏时延。任何一级未通过，都退回影子阶段定位，不继续扩大范围。

### Task 9：无编码器启动与低速边界

**Files:** `foc/motor/motor.c`、`foc/motor/motor_position.c`、`foc/app/motor_config.h`、启动/切换主机测试。

- [ ] 先定义“仅中高速无感运行”的交付范围：静止使用 ALIGN 与受限开环加速，达到 Task 7 的可信电频并连续合格后接管；设置启动限流、加速斜率、超时、反转与失败安全停机。现有固定频率 hard-drag 候选没有加速/限流策略，不能直接作为完整启动方案。
- [ ] 若产品要求零速保持或全速域高可靠无感闭环，反电势 SMO 单独不具可观测性；需另立 HFI/其他位置获取方案及硬件可行性验收。不能通过提高滑模增益或降低滤波带宽承诺解决静止估角。
- [ ] 最终交付必须分别通过冷启动、热启动、负载启动、低压、额定电压、加减速、方向切换、丢采样、SMO 失效、堵转/过流和安全停机测试；全部留原始数据、固件版本与回滚配置。

## 6. 执行顺序与报告格式

严格按 `Task 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9` 推进。允许 Task 6 的纯主机代码审查与 Task 2–4 并行，但 SMO 参数和控制源不与电流 PI 同时改。每一轮仅提交：改动参数、构建/主机测试结果、原始 CSV、局部样本完整性、量化指标、结论（通过/未通过/证据不足）、下一轮唯一变量。未通过时先查测量链与限幅，再查模型/时序，最后调增益。

Task 1 已完成；Task 2 的工具修正和三轮定时阶跃已完成。Task 3 已确认 Ialpha/Ibeta 的交流 RMS 与 Iq 同量级，暂时定位为 Park 之前的采样/电流纹波路径。下一版三相原始 ADC 计数通道已通过主机测试及目标构建，等待用户烧录和原始相电流采集；在证据区分 ADC/采样问题与真实电流纹波前，不调 KiTs。速度和 Vq 仍未连续记录，不能把阶跃响应差异归因于 PI。
