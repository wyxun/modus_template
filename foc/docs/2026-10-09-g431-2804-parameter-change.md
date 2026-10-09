# G431 / 2804 电机参数变更存档

日期：2026-10-09。用户已确认空轴，按参数表中的转子惯量整定，并授权存档后落实参数。适用配置为 G431、12 V 母线、20 kHz 电流环、1 kHz 速度环、浮点数值后端。本次不烧录、不启动电机。

## 实施顺序

1. 保存修改前后参数和单位换算依据。
2. 用主机测试复现电阻辨识在超流时没有立即退出的问题。
3. 修改参数；通过已有 Motor ISR 停机接口锁存辨识错误。
4. 运行浮点/定点回归测试和 G431 目标构建，记录结果。
5. 上板校准和闭环性能验证另行进行；本次不把编译通过视为真机验证。

## 电机模型与硬件换算

| 项目 | 本次采用值 | 依据与限制 |
|---|---:|---|
| 接法 / 极对数 | Y / 7 | 用户提供的 2804 电机参数图 |
| 相电阻 Rs | 2.55 Ω | 图中线间电阻 5.1 Ω；FOC 使用相电阻 |
| Ld / Lq | 0.86 / 0.86 mH | 采用图中相电感，暂按非凸极模型；不使用绕组电感 2.8 mH |
| 永磁磁链 | 0.0035 Wb | 暂按 dq 模型的相磁链口径；厂家未明确峰值/RMS 定义 |
| 转子惯量 J | 3.7e-6 kg·m² | 3700 g·mm² × 1e-9；空轴，不含外部负载 |
| Kt | 0.03675 N·m/A | 1.5 × 7 × 0.0035；电流为相峰值口径 |
| 额定 / 最大电流 | 0.5 / 2 A | 厂家未明确 RMS/峰值和最大电流持续时间；本次给定限制低于 0.5 A 相峰值 |
| 电流基值 / ADC 差码 | 3.5 A / 119 code | 保留用户先前的 119U 改动，并在 G431 target.mk 显式绑定 |
| 电流分辨率 | 29.41 mA/code | 3500 / 119；0.5 A 约 17 code，改变软件基值不能提升硬件分辨率 |

所给采样图中，OPA 正输入节点还通过 10 kΩ 接入 VShunt+，不能仅用 976/(22000+976) 作为信号分压比。假设 OPA 输入电流可忽略、VShunt− 为采样参考地、分流电阻 20 mΩ、PGA 为 16、ADC 为 12 位且 VDDA=3.3 V：

```text
Vp = (VShunt+/10000 + 3.3/22000) /
     (1/10000 + 1/22000 + 1/976)
ΔVp/ΔVShunt+ = 0.0854668
ΔVout/ΔVShunt+ = 16 × 0.0854668 = 1.367469
counts/A ≈ 1.367469 × 0.020 × 4095 / 3.3 = 33.95
counts/base ≈ 33.95 × 3.5 = 118.8 → 119
```

这是电路模型推算，仍需已知相电流校准 ΔADC/ΔI 和极性。零偏继续按 PWM 关闭时每相实测值标定，不固定为 2048。

## 修改前后参数

下表除 FOC_* 与 IDENTIFY_RESISTANCE_* 外，宏统一带 MOTOR_CONFIG_ 前缀，位于 foc/app/motor_config.h。电阻宏位于 foc/identify/identify.h；板级绑定位于 target/stm32g431/target.mk。

| 宏 | 修改前 | 修改后 | 原因 / 物理含义 |
|---|---|---|---|
| FOC_CURRENT_COUNTS_PER_BASE | 工作区已为 119U，Git 原值 1390U；G431 无显式覆盖 | 119U，新增 G431 显式覆盖 | 约 34 code/A × 3.5 A；不把先前用户的改动算作本次新改动 |
| FOC_CURRENT_BASE_MILLIAMP | 3500U，G431 无显式覆盖 | 3500U，新增 G431 显式覆盖 | 与 119U 成对绑定；不是电机额定电流 |
| RESISTANCE_MILLIOHM | 2740U | 2550U | 图中相电阻 |
| INDUCTANCE_D_MICROHENRY | 800U | 860U | 图中相电感 |
| INDUCTANCE_Q_MICROHENRY | 800U | 860U | 暂按 Lq=Ld |
| BASE_ELECTRICAL_HZ | 200.0f | 400.0f | 速度基值从 1714.3 rpm 增为 3428.6 rpm，使 2600 rpm 小于无感启动 0.8 pu 门槛 |
| MAX_SPEED_REFERENCE_PU | 200.0f | 303.333333f | 此宏实际先按电频率 Hz 输入；初始化后为 0.758333 pu，即 2600 rpm |
| CURRENT_PI_KP_PU | 0.20f | 0.472810f | 12 V 下电流环目标带宽 300 Hz |
| CURRENT_PI_KI_TS_PU | 0.005f | 0.070097f | 已乘 50 μs；并联 PI 积分增量 |
| SPEED_PI_KP_PU | 0.20f | 0.001622326f | 输入宏系数按电 Hz 误差；初始化乘 400 后为约 0.6489304 |
| SPEED_PI_KI_TS_PU | 0.005f | 0.00002548343f | 已乘 1 ms；初始化乘 400 后为约 0.010193372 |
| MAX_PHASE_CURRENT_PU | 1.0f | 0.14f | 给定上限从 3.5 A 降为 490 mA；不等同于全局实测过流保护 |
| ALIGN_CURRENT_PU | 0.005f | 0.06f | 17.5 mA → 210 mA，约 0.595 → 7.14 code |
| STARTUP_IQ_PU | 0.02f | 0.06f | 70 mA → 210 mA，空轴启动初值 |
| STARTUP_CLOSED_SPEED_STEP_PU | 0.001f | 0.0005f | 基值翻倍，保持原来的物理速度斜率 |
| OBSERVER_MIN_SPEED_PU | 0.15f | 0.075f | 保持 30 电 Hz，即约 257 rpm 的门槛 |
| IDENTIFY_LD_MAX_SPEED_PU | 0.01f | 0.005f | 保持 2 电 Hz，即约 17.14 rpm 的运动门槛 |
| IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU | 0.05f | 0.025f | 12 V 下理想稳态约 118 mA |
| IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU | 0.1f | 0.085f | 12 V 下理想稳态约 400 mA，低于额定电流 |
| IDENTIFY_RESISTANCE_MIN_CURRENT_PU | 0.001f | 0.025f | 87.5 mA，约 2.975 code，拒绝过小电流与电流差 |
| IDENTIFY_RESISTANCE_MAX_CURRENT_PU | 无 | 0.14f | 新增 490 mA 的 dq 电流矢量退出阈值，每次 ISR 检查 |
| IDENTIFY_LD_MODULATION_PU | 0.10f | 0.06f | 12 V 下调制激励约 1.2 V → 0.72 V |
| IDENTIFY_LD_MAX_CURRENT_PU | 0.02f | 0.10f | 70 mA → 350 mA；这是退出阈值，不是电流目标 |
| IDENTIFY_LD_MIN_DELTA_PU | 0.001f | 0.025f | 最小变化提高到约 3 code，避免接受远小于一个码的斜率 |
| IDENTIFY_LD_CAPTURE_SAMPLES | 4U | 8U | 首末采样跨度从 150 μs 增为 350 μs；延迟 1 拍 + 8 样本小于半周期 10 拍 |
| IDENTIFY_LD_HALF_CYCLES | 4U | 20U | 增加正负脉冲平均次数；总激励约 10 ms |

## 保留参数及原因

| 参数 | 保留值 | 原因 |
|---|---|---|
| POLE_PAIRS | 7U | 与规格一致 |
| BASE_VOLTAGE_MILLIVOLT | 12000U | 本次固定 12 V 工况 |
| FOC_CURRENT_SAMPLE_INVERTED | 1 | 无新极性测量证据 |
| CURRENT_PI_OUTPUT_MIN/MAX_PU | ±0.55 | 输出为电压调制量，不随 ADC 标度成比例调整 |
| CURRENT_PI_INTEGRATOR_MIN/MAX_PU | ±0.50 | 同上 |
| SPEED_PI_OUTPUT_MIN/MAX_PU | ±0.10 | 最大 Iq=350 mA，保留空轴初调余量 |
| SPEED_PI_INTEGRATOR_MIN/MAX_PU | ±0.10 | 与速度 PI 输出一致 |
| 两环 KD_OVER_TS_PU | 0 | 避免放大量化噪声 |
| SPEED_LOOP_FREQUENCY_HZ / FOC_HF_ISR_HZ | 1000U / 20000U | PI 按现有周期推算 |
| IDENTIFY_LD_FREQUENCY_HZ / CAPTURE_DELAY | 1000U / 1U | 与 8 样本窗口兼容 |
| 电阻采集 | 100 Hz × 100 样本，稳定等待 100 ms | 保留平均与有界等待 |
| POSITION_SOURCE / HARD_DRAG_ELECTRICAL_MILLIHZ | HARD_DRAG / 0 | 不假设编码器已连接；速度 PI 必须用真实速度反馈验证 |
| OBSERVER_AUTO_TAKEOVER | false | 尚无自动无感接管可靠性证据 |
| SMO 参数 | 原值 | 不把观测器估计限幅等同于电流给定上限 |
| ADC 对齐、4 次平均、零偏、母线分压 | 原值 | 无证据需要修改外设；平均不消除系统误差 |
| AT32F413 的 2048U | 原值 | 属于另一块板子 |

## PI 推导与速度单位

当前 PID 是并联形式，积分宏已经包含控制周期。电流环输出直接进入 SVPWM，未看到根据实测母线自动归一化的环节，故以 12 V 推算：

```text
ωc = 2π × 300
Kp_current = ωc × 0.00086 × 3.5 / 12 = 0.472809694
KiTs_current = ωc × 2.55 × 3.5 / 12 / 20000 = 0.070096786
```

速度环按空轴、忽略摩擦与电流环延时、自然频率 5 Hz、阻尼比 1 计算。5 Hz 是整定自然频率，不是已经测得的闭环带宽。

```text
ωn = 2π × 5
Kt = 1.5 × 7 × 0.0035 = 0.03675 N·m/A
ωm_base = 2π × 400 / 7
Kp_speed_effective = 2 × ωn × J × ωm_base / (Kt × 3.5)
                   = 0.648930257
KiTs_speed_effective = ωn² × J × ωm_base / (Kt × 3.5) / 1000
                     = 0.010193373
```

foc_app_BindMotorConfig() 会把速度 PI 宏乘电频率基值，所以宏填上述有效增益 / 400。不能把有效增益直接填入宏，否则会再放大 400 倍。MAX_SPEED_REFERENCE_PU 则先除以基值：303.333333/400=0.75833333 pu。该阶段暂时保留了旧宏名；后续审核已统一改为 `MAX_SPEED_ELECTRICAL_HZ` 与 `SPEED_PI_*_PER_ELECTRICAL_HZ`，数值和绑定公式不变，见下方收尾说明。

旧配置的最大参考为 200/200=1 pu，即 1714.3 rpm。motor_StartSensorlessSpeed() 还限制目标不超过 0.8 pu，即旧基值下 1371.4 rpm。新基值下 2600 rpm 对应 0.758333 pu，可通过这两个范围检查；能否达到该转速仍取决于电压余量、真实角度反馈和电机损耗。运行态 PU 给定不能直接沿用：旧给定 × 0.5 才保持相同物理转速；新 0.5 pu 为 1714.3 rpm。

G431 使用 float。本次很小的速度 KiTs 宏在 Q15 后端会先量化再乘基值，有效增益会偏离上述浮点初值；定点回归测试通过不代表这组参数已经完成定点性能整定。

## 辨识退出与测量限制

新增电阻辨识最大电流宏，在电机启动后的每次辨识 ISR 中检查 dq 电流矢量幅值，不等待 100 Hz 降采样，不跳过已采满但尚未处理的批次。超过 0.14 pu 或电流非有限值时，通过 motor_IdentificationAbortIsr() 关闭输出、锁存 Motor 辨识故障，同时锁存 Identify ERROR/SAFETY 并取消结果发布。前台不得恢复激励；需安全停机并按已有故障清除/复位流程恢复。该检查依赖采样标度正确，不替代硬件保护。

新增回归覆盖等待稳定、采集、批次处理中的正负 d/q 超流、合成矢量超流和错误锁存。正常两档电阻测试按 12 V、3.5 A 基值、2.55 Ω 模型生成采样，要求恢复约 2550 mΩ。

主机编译另发现已有日志字段缺失：identify_resistance.c 已使用 aawAverageAd 和 atAverageDuty，但 identify_resistance_t 未声明。当时补齐日志路径需要的两个字段，均受 FOC_APP_LOG_RESISTANCE_ID 条件编译保护；不改变采样或 PWM 时序。随后日志清理已删除这些过程诊断字段与开关，本段仅记录阶段历史，不描述当前接口。

12 V 下两档电流理想值约 118/400 mA，差值约 282 mA，即 9.6 code。实际端电压还受死区、开关压降和母线变化影响。现有电阻公式使用命令差值和固定 12 V 基值；未增加端电压测量或母线补偿，所以不能声称已经消除电阻误差。上档在 16 V 下理论电流约 533 mA，超出本次初调条件；不要把该配置当作全供电范围已验证方案。

电感采用 350 μs 窗口时，电流并非严格线性；现有算法减去 Rs × 平均电流，但平均积分近似、采样延时和量化误差仍需波形核对。增加平均次数只能改善有随机扰动的噪声，不能保证消除重复的量化偏差。电感辨识需要按现有接口约定机械锁定转子。

## 验证记录

实施时先运行新增超流测试，旧实现未调用停机接口，`s_wAbortCount == wAbortBefore + 1U` 断言失败，确认复现缺失的退出行为。修复后结果如下；当前尚未完成硬件验证。

| 验证 | 结果 |
|---|---|
| run_identify_resistance_test.ps1 | FLOAT / FIXED 通过；正常 2.55 Ω 模型、零电流超时、等待/采集/批次处理中正负 d/q 与合成矢量超流、错误锁存；FLOAT 另覆盖 NaN/Inf |
| run_identify_inductance_diagnostic_test.ps1（随后删除） | 当时 FLOAT / FIXED 通过；按 119-code 基值修正小电流诊断断言，0.004 pu 取整为 0 code |
| run_encoder_command_test.ps1 | FLOAT / FIXED 通过；补齐旧测试缺失的 motor_SensorlessStartConfigured 桩 |
| run_motor_speed_pu_test.ps1 | FLOAT / FIXED 通过 |
| run_sensorless_startup_test.ps1 | FLOAT / FIXED 通过 |
| run_motor_reference_limit_test.ps1 | FLOAT / FIXED 通过 |
| run_diagnostic_log_groups_test.ps1（随后删除） | 当时 FLOAT / FIXED 的 6 组日志开关组合均编译通过 |
| 当前宏的数值复核 | 最大参考 2600 rpm / 0.758333 pu；速度 PI 初始化后 Kp=0.6489304、KiTs=0.010193372；新增/修改 C 行均不超过 82 列 |
| G431 debug-rel 构建 | 通过，无编译警告；text=70116、data=588、bss=18344 bytes |
| G431 release 构建 | 通过，无编译警告；text=40388、data=416、bss=6024 bytes |
| 差异检查 | git diff --check 通过；原有 .agents/ 文件与先前 119U 改动保留 |

本机工具链实际位于 D:/0_software/，包含可运行本机程序的 MinGW GCC；此前只检查 D:/software 与 PATH 的结论不能继续沿用。主机测试临时将 D:/0_software/msys64/mingw64/bin 加入 PATH。目标构建仍通过仓库 make.bat，显式指定 SW_ROOT 与 OS，分别使用独立输出目录，避免覆盖原 build/template 固件：

```powershell
New-Item -ItemType Directory -Force build\g431-2804-audit-debug-rel, build\g431-2804-audit-release | Out-Null
.\make.bat TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software OS=Windows_NT BUILD=debug-rel BUILD_DIR=build\g431-2804-audit-debug-rel
.\make.bat TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software OS=Windows_NT BUILD=release BUILD_DIR=build\g431-2804-audit-release
```

除返回值外，也检查构建日志和 ELF/BIN/HEX 产物；本机 make.bat 在 make 出错时可能仍返回 0，不能只凭批处理退出码判断成功。release 关闭 Shell/波形入口；符号检查显示原有 foc_debug_CurrentStepRun 前台服务仍被保留，本次未修改这一既有裁剪边界，不声称所有调试服务已消失。构建与单元测试均不证明实板标度、环路稳定性或高转速性能。

## 上板验收

1. PWM 关闭时核对三相零偏、12 位右对齐格式和母线读数；用独立相电流测量校准 ΔADC/ΔI 与方向，不能用电源输入电流代替相电流。
2. 用真实编码器角度/速度建立电流环基线；观察 Id/Iq、调制输出和饱和，出现持续振荡、超流或角度失效时停止。
3. 再做空轴速度阶跃，观察真实速度、Iq 给定/反馈、超调、稳态振荡和输出饱和；强拖的给定频率不能当作真实转速。
4. 在 12 V、静止转子和有效电流校准条件下重复电阻/电感辨识，保留 ADC 差码、母线、激励、电流差和重复结果，不要求仅凭宏变化就贴合规格。

方法参考：[TI InstaSPIN Projects and Labs User’s Guide，Lab 5a/5b](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/8880.instaspin_5F00_labs.pdf)。本次已换算为仓库并联 PI、控制周期和实际速度初始化口径，未照抄其串联积分系数。

## 后续日志清理与审核收尾（2026-10-09）

上方参数表保留参数修改时的宏名和验证记录。当前宏名映射如下，数值未再整定：

| 历史宏名（均带 MOTOR_CONFIG_） | 当前宏名 | 口径 |
|---|---|---|
| MAX_SPEED_REFERENCE_PU | MAX_SPEED_ELECTRICAL_HZ | 电 Hz，绑定时除以 f_base |
| SPEED_PI_KP_PU | SPEED_PI_KP_PER_ELECTRICAL_HZ | 每电 Hz，绑定时乘 f_base |
| SPEED_PI_KI_TS_PU | SPEED_PI_KI_TS_PER_ELECTRICAL_HZ | 每电 Hz，已经含速度周期 |
| SPEED_PI_KD_OVER_TS_PU | SPEED_PI_KD_OVER_TS_PER_ELECTRICAL_HZ | 每电 Hz，已经除以速度周期 |

两环 D 项仍为零；当前速度 D 宏也按每电 Hz 命名。删除的电感诊断、日志开关测试不再是验收入口；`aawAverageAd/atAverageDuty` 和 CCR 计时日志不再是公共契约。零偏与 PWM 时序的替代观测步骤见 [电流采样审计](g431-current-adc-and-identify-audit.md)。当前完整测试、构建和未完成边界以 [Motor 审核收尾记录](2026-10-09-motor-boundary-refactor.md#审核收尾2026-10-09) 为准，不能把上方历史构建大小当作当前产物。
