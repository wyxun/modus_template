# SMO 滑模反电势观测器

SMO 用等电感电机模型估计 αβ 电流，再从电流估计误差提取反电势，求出电角度和带方向的电速度。`MFOC_FEEDBACK_ENABLE_SMO=1` 将它接入 Feedback。默认 `MFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_SMO` 用其角速闭环。选择 `MFOC_FEEDBACK_POSITION` 并同时启用物理位置和 SMO 时，控制使用板级位置，SMO 作为影子观测器运行。[反馈接口](feedback.md) · [SMO 公共接口](../../position/include/mfoc_smo.h)

## 输入和参数

每次 `mfoc_smo_Step()` 接收本拍测得的 `iαβ[k]`（I/Ibase）和上一控制区间施加的模型电压 `uαβ[k-1]`（V/Vbase）。Feedback 用上次成功提交给 PWM 的占空比和与其一同保存的母线快照重建该电压。它反映调制后的理想平均电压，仍有死区、器件压降等误差。[Feedback 电压准备](../../motor/src/mfoc_feedback.c) · [模型电压重建](../../control/src/mfoc_modulation.c)

| `mfoc_smo_Init()` 输入 | 含义与约束 | 当前应用取值 |
| --- | --- | --- |
| `model.rs`、`model.ld`、`model.lq` | 相电阻和 d/q 轴电感，均为正 PU，要求 `ld == lq` | 从 2.55 Ω、0.86 mH、0.86 mH 换算 |
| `sample_hz`、`speed_base_hz` | 实际控制频率 `fs`（1 至 1,000,000 Hz）和正的电频率基值 `fb`（Hz） | 默认 20 kHz、400 Hz。Motor 统一传入电频率基值 |
| `h`、`boundary` | 滑模校正电压限幅（电压 PU）、电流误差边界层半宽（电流 PU），均在 `(0, 1]` | 5 V / 12 V、0.1 PU |
| `i_limit` | 每轴估计电流的幅值上限，范围 `(0, 1]`，独立于 Motor 电流保护 | 1 PU |
| `cutoff_rad_s` | 滑模电压低通的截止角频率，单位 rad/s，须大于零 | 4000 rad/s（约 637 Hz） |

Init 预计算离散系数，失败保留原实例。停机更新电机模型时，Feedback 用保留的 SMO 配置重算系数。更新成功后清估计历史，重新积累速度窗。[应用配置](../../../class/mfoc_app.c) · [电机基值与参数](../../mfoc_motor_config.h)

## 从电流误差到角速

1. **边界层。** 两轴分别用前一拍的滑模校正电压 `z` 推进电流模型，按梯形积分求 `i_hat` 并限幅。用更新后的误差 `i_hat - i` 求 `z = clamp((h / boundary) × (i_hat - i), -h, h)`。例如误差为半个 `boundary`，校正量为 `h/2`。达到或超过正边界，校正量保持 `h`。`z` 同时进入一阶低通，得到两轴反电势估计 `eα`、`eβ`。[逐轴递推实现](../../position/src/mfoc_smo.c)
2. **速度窗。** 原始电角度为 `atan2(-eα, eβ)`。累计相邻两拍原始角度的最短有符号差（电周），每 `N = clamp(round(fs / 1000), 1, 127)` 个角差更新一次 `speed = 累计电周 × fs / (N × fb)`。窗内各拍沿用最近完整窗的速度。20 kHz 时，首个反电势矢量非零的样本建立角度起点，还需 20 个连续角差才有首个速度估计。
3. **相位补偿。** 每次更新速度时，计算低通相位超前量 `atan(2π × fb × speed / cutoff_rad_s)`，加到各拍原始角度上。负速度再加半个电周，以修正反转时反电势极性翻转。补偿针对低通的低频相位滞后，采样与逆变器引起的延迟仍会反映在输出角差中。[正反转与跨零用例](../../tests/position/smo_test.c)

## 有效、复位与接管

`angle` 是 BAM32 电角度（一电周为 `2^32`），`speed` 是带方向的电频率除以 `fb`。诊断量 `bemf = |eα| + |eβ|` 为两轴绝对值之和，单位为电压 PU。两轴反电势同时为零时，本拍输出清零、速度窗重新开始。有反电势时，`valid` 取决于最近完整窗的速度是否非零。此判据没有反电势幅值门限，低速噪声也可能产生非零估计。[输出定义](../../position/include/mfoc_smo.h)

缺少本拍电流或上拍电压时，Feedback 调用 `mfoc_smo_Reset()` 清除两轴电流、低通和速度窗历史，保留系数。当 SMO 是 backend 时，只有电压准备成功且 `valid` 的角速才会发布。无感启动先强拖，Motor 再按开环角与观测角之差、目标速度误差和持续时间接管。影子运行可用同一控制拍的编码器电角度核对估计误差。[接管条件与故障处置](feedback.md#handover) · [角度对照波形](../../../class/mfoc_debug.c)

## 主机验证

在仓库根目录运行 `./mfoc/tests/scripts/run_smo_tests.ps1`。算法用例在 FLOAT、FIXED 两种后端检查系数、边界层限幅、正反转速度、相位补偿和复位。其中相位用例注入已知反电势，单独检验角速提取。Feedback 用例核对电压重建、影子运行、缺帧复位与模型更新。实际电机上的低速可用范围和带载接管仍需用编码器对照验证。[测试脚本](../../tests/scripts/run_smo_tests.ps1) · [Feedback 回归用例](../../tests/position/smo_feedback_test.c)
