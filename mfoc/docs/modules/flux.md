# Flux 非线性磁链观测器

Flux 用本拍定子电流和上一区间的模型电压估算转子磁链，再用锁相环（PLL）给出电角度与电速度。Feedback 持有 `mfoc_flux_t` 实例。在控制中断中，每次调用 Feedback 且两份输入齐备时，Flux 推进一步。它可以在编码器闭环中并行运行，也可以作为无感启动后的闭环角速来源。[Flux 公共接口](../../position/include/mfoc_flux.h) · [Feedback 接口](feedback.md)

## 每一拍的输入与输出

| 量 | 单位与时刻 | 来源或用途 |
| --- | --- | --- |
| `i[k]` | αβ 电流，除以相电流峰值基值 `Ib` | 本拍采样，经幅值不变 Clarke 变换 |
| `u[k-1]` | αβ 电压，除以电压基值 `Vb` | Feedback 用上一拍成功提交的三相 duty 和该拍母线快照重建，覆盖从上次到本次采样的区间 |
| `out.angle` | 本拍电角度，BAM32 每电周回绕一次 | 当前采样时刻的 PLL 角度 |
| `out.speed` | 带符号的电速度 PU，`ωe/ωb` | PLL 在本拍更新的速度 |
| `out.valid` | 布尔值 | 修正后的转子磁链至少有一个分量非零时为真，也可能同时给出零速度 |

## 磁链递推与 PLL

设 `fs = sample_hz`、`ωb = 2π × speed_base_hz`，归一化步长 `Δτ = ωb/fs`。以下磁链用 `ψb = Vb/ωb` 归一化，`ψm` 为 `model.flux`，`Ls` 为等值的 `model.ld` 和 `model.lq`。每拍依次执行下列运算。[递推实现](../../position/src/mfoc_flux.c) · [电机模型基值](../../mfoc_motor_config.h)

1. 从上拍定子磁链积分电压，得到 `ψs_pred = ψs[k-1] + Δτ × (u[k-1] − Rs × i[k])`。扣去电感磁链后，`r = ψs_pred − Ls × i[k]` 是转子磁链预测值。
2. 比较 `r` 的模长平方与永磁体磁链平方。修正量沿 `r` 方向，`ψr = r + gain × Δτ × (ψm² − |r|²) × r`。磁链偏小时修正量朝外，偏大时朝内。将 `ψr + Ls × i[k]` 保存为下一拍积分所需的定子磁链。这里的电机模型采用常量电感和永磁磁链。FIXED 中间运算到达数值载体边界时会饱和，模型未描述铁芯随电流变化的磁饱和特性。
3. 以本拍角度 `θ[k]` 投影转子磁链，得到归一化相位误差 `e = (ψrβ cos θ − ψrα sin θ)/ψm`。PLL 积分项增加 `pll_ki × e/fs`，积分项与 `pll_kp × e` 之和即 `out.speed`。发布 `θ[k]` 后，再把角度推进 `out.speed × speed_base_hz/fs` 电周供下拍使用。

## 参数与使用边界

`mfoc_flux_Init()` 接受正值的 `Rs`、`Ld = Lq`、`ψm`、电频率基值及三个增益。采样率范围为 1～1,000,000 Hz，接入 Motor 时应配置为实际控制频率。FIXED 下，预计算正系数舍入为零或超出载体范围时返回 `MFOC_RESULT_OUT_OF_RANGE`。初始化失败保留原实例。[初始化与定点系数计算](../../position/src/mfoc_flux.c)

| 应用配置 | 默认值 | 调整的环节 |
| --- | --- | --- |
| `MFOC_FLUX_GAIN` | 0.1 | 磁链模长修正的快慢，离散修正量随 `gain × Δτ` 变化 |
| `MFOC_FLUX_PLL_KP` | 0.5 | 相位误差对本拍速度的比例修正 |
| `MFOC_FLUX_PLL_KI` | 100 | 相位误差在每秒内累积的速度修正，Init 按采样率折算到每拍 |

这组值是影子观察的起始配置。磁链幅值接近 `ψm`、相差很小时，`e` 近似为弧度相差。由上式的连续近似，PLL 自然角频率为 `ωn ≈ √(ωb × pll_ki)`，阻尼比为 `ζ ≈ ωb × pll_kp/(2ωn)`。这两个量可用来选择响应速度与阻尼，实际跟踪带宽还受采样率和磁链估计误差影响。[增益默认值](../../mfoc_config.h) · [应用配置](../../../class/mfoc_app.c)

低速时，永磁磁链转动产生的电压减小，电阻压降的估算误差、逆变器死区和器件压降对 `u − Rs × i` 的相对影响增大。模长修正只沿已有的磁链方向作用，静止且初始磁链为零时，算法无法仅靠模长约束确定转子方位。即使 `valid` 已置位，接管仍需经过 Motor 的速度窗与角度捕获判据。[无感接管](feedback.md#handover)

缺少本拍电流、缺少上一拍输出或电压重建失败时，Feedback 调用 `mfoc_flux_Reset()` 并清零缓存的 `flux_output`。Reset 保留系数，清除定子磁链、PLL 积分和角度历史。输入齐备后重新估算。若仅是本拍算出的转子磁链为零，Flux 清零本拍输出和 PLL 历史，保留本拍更新后的定子磁链。[Feedback 调度](../../motor/src/mfoc_feedback.c) · [Flux 实现](../../position/src/mfoc_flux.c)

## 接线与验证

Flux 默认关闭。编译时设 `MFOC_FEEDBACK_ENABLE_FLUX=1` 即可运行观测器。做编码器对照时，同时启用物理位置并选 `MFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_POSITION`。若选 `MFOC_FEEDBACK_FLUX`，Feedback 在电压重建成功且 `flux_output.valid` 为真时发布 Flux 的角速。[反馈配置](../../mfoc_config.h)

从仓库根目录运行 `./mfoc/tests/scripts/run_flux_tests.ps1`。FLOAT、FIXED 都用等电感 PMSM 合成波形回放 ±0.05、±0.3 和 ±0.7 PU 的恒速段，以及 +0.3 到 −0.3 PU 的换向。测试在预热后检查最大角差和速度误差，另行覆盖零磁链、模长修正、复位、后端选择和停机更新模型。[主机脚本](../../tests/scripts/run_flux_tests.ps1) · [数值回放](../../tests/position/flux_algorithm_test.c) · [Feedback 接线用例](../../tests/position/flux_test.c)

台架对照可先用编码器闭环驱动同一台电机，用 `motor flux status` 读取估计角速和配置磁链。命令的 `angle_mT` 单位为千分之一电周，`speed_mpu` 单位为千分之一速度 PU，`psi_mpu` 打印的是模型永磁磁链。FLOAT 构建启用调试波形、选用角度通道（`MFOC_DEBUG_HFI_WAVEFORM=0`）、关闭 EKF 并启用 Flux 时，可比较 `Enc_mT`、`Flux_mT` 和最短角差 `Err_mT`。重点检查低速、换向及带载时的角差和速度偏差，主机合成波形未覆盖实际逆变器压降与板上采样误差。[状态命令](../../../class/mfoc_app.c) · [对照波形](../../../class/mfoc_debug.c)
