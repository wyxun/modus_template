# FOC 调试与标定记录

## 速度单位核查：2026-09-25

代码链为 `MOTOR_CONFIG_BASE_ELECTRICAL_HZ=100` → `foc_app_BindMotorConfig()` → `motor_position_Init()` → 编码器机械速度换算：

```text
electrical_speed_pu
  = mechanical_turns_per_second × pole_pairs / 100 electrical turns_per_second
```

对当前 7 极对电机：1 pu = 100 电 Hz = 628.32 电 rad/s = 36000 电角度/秒；机械速度约 14.286 转/秒 = 857.14 rpm。0.5 pu 约 428.57 rpm。规格书 2600 rpm 若适用，可换算为约 303.3 电 Hz = 3.03 pu。

配置宏 `MOTOR_CONFIG_MAX_SPEED_REFERENCE_PU=100` 的名称与绑定实现不一致：`foc_app_BindMotorConfig()` 将该值除以 100 Hz 基值，因此 Motor 收到的最大参考是 1 pu（100 电 Hz），而不是 100 pu。调试命令 `motor speed` 的值和 Motor PI 反馈都使用电速度 pu；Encoder `motor encoder` 日志打印的却是机械转/秒。后续配置重命名或语义整理应单独做并加测试，不能混淆这两种读数。

## 电流阶跃采集基线：2026-09-25

**标度审计提示：** 以下电流环阶跃和 Iq/αβ 的 pu 数值均来自旧固件，不能作为物理电流或 PI 调参基线。旧配置把 12 位 ADC 设为左对齐，却把完整 JDR 原值交给 Motor，并使用 1390 counts/base；在已确认 20 mΩ、PGA×16、3.3 V 假设下，1390 对应右对齐码，旧反馈会高估约 16 倍。新固件已在 MDI 采样绑定处右移 4 位；用户烧录并复核前，不运行 0.05 pu 电流阶跃。旧 Iq、Ialpha/Ibeta 与 SMO 物理量应在新固件上重采，历史波形形状/时间戳可留作参考，但旧 pu 数值作废。

固件波形通道为 `IqRef`、`Iq`，CSV 元数据为 `time`、`sample_index`。阶跃命令 `motor step 0.05 100`，名义波形率 10 kHz，滑动平均 64 点（约 6.4 ms）。三份原始文件保存在 `build/`：

| CSV | 脉冲有效时长 | 局部丢样 | 采样率（由本地时间戳估计） | 零参考交流 RMS / 峰峰值 | 脉冲末段中值 | 10–90% 上升时间 | 指令过冲 / 平台上方峰值 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `current_step_2ch_20260925.csv` | 100.7 ms | 0 | 10.000 kHz | 0.0139 / 0.080 pu | 0.0400 pu | 16.7 ms | 0% / 19.7% |
| `current_step_2ch_repeat_20260925.csv` | 100.1 ms | 0 | 10.000 kHz | 0.0126 / 0.071 pu | 0.0450 pu | 23.7 ms | 0% / 4.5% |
| `current_step_20260925_161831_238776.csv` | 100.7 ms | 0 | 10.000 kHz | 0.01355 / 0.073 pu | 0.0360 pu | 16.8 ms | 0% / 28.3% |

第一份文件在阶跃前约 82.1 秒处有一次大缓冲缺口，重复文件在阶跃前约 1.09 秒处有一次缺口；都位于局部基线/阶跃窗口之外。分析器现在按 `sample_index` 检查局部连续性，并用 CSV `time` 计算脉冲时长和上升时间。上升时间按 10–90% 的**实测脉冲末段中值**计算；过冲也相对该平台值计算。平台末段取脉冲最后 10% 的原始采样中值，时间响应用 64 点滑动平均。

这些旧固件数据曾显示末段低于 0.05 pu 目标，但由于上述约 16 倍反馈尺度疑点，所有 Iq 实测 pu、噪声比例和稳态误差均不能解释为物理电流；此前据此评价 PI 的结论撤销。新固件标度复核并重采前，不从这些值推出 PI 参数。

## 当前实现进展

- `tools/foc_current_step_test.py` 接受当前两通道 CSV，不再要求 `ElecSpeed`；缺少速度通道时明确报告速度不可用。
- 工程目录下的 `tools/aitrace.exe` 是旧版（2026-08-04），其 CSV 没有 `sample_index`；采集脚本现在优先选择相邻 `mstudio/aitrace/aitrace.exe`，缺少该文件时才回退。
- 上升时间/脉宽使用文件时间戳，采样率从局部时间间隔估算；拒绝阶跃及其 50 ms 基线窗口内的样本序号断档。
- 指令过冲（峰值高于 0.05 pu 目标）与相对测量平台的峰值偏差分开报告。
- 分析器测试覆盖两通道格式、窗口外旧缓冲缺口、局部丢样拒绝、真实时间戳、可选速度列与空行计数。
- 验证命令：`python -m unittest tools.tests.test_foc_current_step_test -v`；截至本记录，9 项测试通过；`run_motor_speed_pu_test.ps1` 的 FLOAT/FIXED 均通过。

板卡被动检查：OpenOCD 正常、ELF 存在，`motor status` 为 `state=2 fault=0 pwm=0`。PWM 关闭时 1 秒 `wave capture` 得到 0 个样本；该波形由 FOC 高频 ISR 取样，下一次应在电流模式 PWM 启动后开始采集。两次空闲探测生成的只有 CSV 表头/无效数据文件已删除。

## SMO 影子角度模式对比：2026-09-25

### 观察到的现象

用户提供两组波形：图一为 `motor voltage 0.0 0.3`，图二为
`motor current 0.0 0.02`。两图的 Encoder 与 SMO 电角度均呈连续、重复的
0～1 turn 环绕斜坡；环绕处的竖线是角度回绕，不应直接判作丢步或电流尖峰。
图一中两角度目测较接近；图二中 SMO 相对 Encoder 有更明显的正向偏差，
即 SMO 看起来领先，且 SMO 曲线抖动更显眼。图二误差量级仅凭截图粗估，
不作为测量结果；需要原始 CSV 才能统计均值、RMS、峰峰值和最大值。

### 当前判断与限制

- 两个命令不是等价激励：`voltage 0.0 0.3` 给定 D/Q 电压；
  `current 0.0 0.02` 给定 D/Q 电流，电流 PI 决定实际电压。
- 两张截图没有证明电角速度、实际 `Iq`、母线和负载相同，因此不能把误差
  差异单独归因于控制模式，也不能据此调整 PI 或 SMO 参数。
- 波形 `Err_mT` 是 `SMO角度 - Encoder角度` 的环绕差，值以电角周为单位，
  显示缩放为 1000；正误差表示 SMO 领先。`Iq_mpu` 是实际 q 轴电流反馈 pu，
  并非 q 轴电压。
- 当前 SMO 是影子估算，电机控制仍使用 Encoder 反馈；角度跟随良好不等于
  已验证 SMO 闭环能力。
- SMO 使用 αβ 电流和电压模型输入。Motor 在 RUN 样本中交给 Position 的
  `tVoltageModelAlphaBeta` 是当时保存的 `tCore.tVoltageAlphaBeta`；电流闭环下
  电压会随 PI 动态变化。因此电流测量噪声、实际电压模型/更新延迟以及运行点
  差异都属于待检验因素，现阶段没有证据选定其中某一项为根因。

### 下一步验证

在不改 PI、ADC 或 SMO 参数的前提下，分别记录两种模式的原始 CSV；尽量匹配
母线、负载和 Encoder 电角速度，并保留现有 `Enc_mT`、`SMO_mT`、`Err_mT`、
`Iq_mpu` 通道。由 Encoder 角度斜率计算实际电角速度，再比较环绕误差的有符号
均值、标准差、RMS、峰峰值及其随速度的变化。若同速条件下仍只有电流模式明显
偏差，再围绕电流模式下的电压模型与电流噪声做下一项单变量验证；在此之前，
不把截图目测值当成调参依据，也不进行 SMO 无感接管。

## 下一个实验

**流程更新：** 本节后文中“烧录这版后……原始 Ialpha/Ibeta”的旧步骤因 ADC 标度审计已作废。新固件先以不超过 0.01 pu 的短电流给定核对反馈方向、ADC 标度与 fault；通过后再重建 0.05 pu 阶跃基线，期间不调 `KiTs`、不切 SMO 闭环。

**状态更新：** Ialpha/Ibeta 与三相原始 ADC 测量均已完成，但都使用旧 ADC 标度；旧 pu 数值不能作为物理电流证据。新固件重采前，暂停 PI/SMO 量化分析。

已完成的三相测量流程为：`motor align` 并等到 idle / PWM=0；排空旧样本；执行 `motor current 0 0` 并捕获 0.32 s；再 `motor stop`。两份有效采集都连续且 0 丢样。后续不再重复同一采集，先验证下方列出的 ADC rank 时差假设；在此之前不改 `KiTs`。

### Clarke / Park 前电流对照：2026-09-25

在 `motor align` 后等待回到 idle，并排空对齐阶段留在 RTT 缓冲中的旧波形，再以 `motor current 0 0` 运行并捕获 0.32 s。有效 CSV 为 `build/ialpha_ibeta_zero_20260925_clean.csv`：3201 个样本、10.000 kHz、样本序号连续、丢样 0。

| 通道 | 旧固件报告均值 (pu，不可作物理量) | 旧固件报告交流 RMS (pu，不可作物理量) | 峰峰值 (旧标度 pu) |
|---|---:|---:|---:|
| Ialpha | -0.00083 | 0.01334 | 0.103 |
| Ibeta | +0.00015 | 0.01381 | 0.085 |

这些值由旧标度算出，需在修正后的固件上重采；不能再据此断言 αβ 噪声与 Iq 噪声同量级或排除 Park 变换。

### 三相 ADC 原始值对照：2026-09-25

在对齐后、PWM 已启用且 `IqRef=0` 时采集两次有效 0.32 s 原始三相 ADC 偏置差；两份 CSV 均为 10 kHz、样本序号连续、0 丢样。诊断值定义为 `ADC raw - calibrated offset`，符号与 Motor 内部使用的 `offset - raw` 相反。旧 CSV 是左对齐 JDR 原始差值；换成 12 位 ADC code 需除以 16，不能直接按 1390 作 pu 换算。

| 文件 | U/V/W 均值 (ADC code) | U/V/W 交流 RMS (ADC code) | U/V/W 峰峰值 (ADC code) | 三相和均值 (ADC code) | 三相和交流 RMS (ADC code) |
|---|---:|---:|---:|---:|---:|
| `iphase_adc_zero_20260925.csv` | .084 / 1.443 / 1.499 | 1.20 / 1.23 / 1.68 | 8 / 9 / 13 | 3.026 | 2.55 |
| `iphase_adc_zero_repeat_clean_20260925.csv` | .096 / 2.196 / 2.237 | 1.20 / 1.23 / 1.75 | 9 / 10 / 14 | 4.529 | 2.56 |

两份有效采集复现了约 1.2/1.2/1.7 ADC code 的交流 RMS；V/W 均值在两次采集中变化，不能把它简单解释成固定校准偏置。原始码值结论仍有效，但旧 pu 换算无效；PWM 零给定下仍无法区分真实开关电流纹波、模拟/ADC 耦合和采样时差。

源码检查到一个待验证的采样时差：ADC1 rank 1 采 U、rank 2 重复采 U；ADC2 rank 1 采 W、rank 2 采 V。所有序列由同一 TIM1 CH4 上升沿触发，但被 FOC 消费的 U/W 是 rank 1、V 是 rank 2，因此 V 晚一个转换槽。它可能污染瞬时三相和，并通过 `Ibeta=(Iv-Iw)/sqrt(3)` 影响 β；目前尚无同步 rank 对照，不能将其定为根因，也不应先改触发点或 rank 顺序。

两份无效重复采集 `iphase_adc_zero_repeat1_20260925.csv`、`iphase_adc_zero_repeat2_20260925.csv` 有数千个 RTT 丢样，已排除统计并删除。旧固件下板卡最终状态为 `state=2 fault=0 pwm=0`。当前首要步骤是烧录并验证 ADC 右对齐码与 1390 counts/base 的标度；通过后以低电流给定重做短采集。rank 时差暂列次要，不改 ADC 时序、PI 或 SMO。

烧录这版后，按相同条件先运行 `motor align`，再 `motor current 0 0`；启动 100–150 ms CSV 波形采集并在 PWM 运行期间保存原始 `Ialpha/Ibeta`。由于零参考模式仍有 PWM 电流纹波，不要把“零给定”理解为功率级关闭。分析两轴各自的均值、交流 RMS、峰峰值、偏置与频谱/周期相关性，并和此前 Iq 零参考交流 RMS 0.01355 pu 对照。若 αβ 已同量级波动，优先检查 ADC 偏置、采样触发点、相电流重构与真实 PWM 纹波；若 αβ 较平稳而 Iq 起伏明显，再检查编码器角度抖动及 Park 变换。测量归因前不调 `KiTs`。之后若发现 IqRef/Iq 基线缺少速度/Vq 证据，再单独加一个所需通道，避免一次加满导致 RTT 丢样。
