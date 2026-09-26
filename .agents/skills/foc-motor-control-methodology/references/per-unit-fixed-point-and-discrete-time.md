
# 标幺值、定点和离散时间

## 1. 标幺值的目的

标幺值不是为了隐藏单位，而是把不同功率、电压和速度等级的电机映射到同一套数值范围，降低中间计算溢出风险，方便仿真、定点实现和平台移植。每个标幺量都必须有明确基值，不能只把数值除以一个“看起来合适”的常数。

基本定义：

$$
x_{\mathrm{pu}}=\frac{x_{\mathrm{real}}}{x_{\mathrm{base}}},
\qquad
x_{\mathrm{real}}=x_{\mathrm{pu}}x_{\mathrm{base}}
$$

建议至少定义并记录：

$$
U_{\mathrm{base}},\ I_{\mathrm{base}},\ \omega_{\mathrm{base}},\
\Psi_{\mathrm{base}},\ T_{\mathrm{base}},\ t_{\mathrm{base}}
$$

常见且容易自洽的一组选择是：

$$
\begin{aligned}
R_{\mathrm{base}}&=\frac{U_{\mathrm{base}}}{I_{\mathrm{base}}},\\
L_{\mathrm{base}}&=\frac{U_{\mathrm{base}}}
{\omega_{\mathrm{base}}I_{\mathrm{base}}},\\
\Psi_{\mathrm{base}}&=\frac{U_{\mathrm{base}}}{\omega_{\mathrm{base}}},\\
t_{\mathrm{pu}}&=\frac{t_{\mathrm{real}}}{t_{\mathrm{base}}},\\
T_{s,\mathrm{pu}}&=\frac{T_{s,\mathrm{real}}}{t_{\mathrm{base}}}
\end{aligned}
$$

若以电角频率基值定义时间，通常取 `t_base = 1 / omega_base`，因此 `Ts_pu = omega_base * Ts_real`。也可以使用其他时间基，但必须从同一套定义推导所有积分和微分项。

转矩基值必须和转矩公式的归一化一致。例如在常见幅值不变约定下可从 `T_base = 3/2 * p * Psi_base * I_base` 推出；如果项目采用功率不变变换，比例因子应随项目公式变化。

## 2. dq 电压方程的标幺推导

从真实值模型开始：

$$
\begin{aligned}
u_d &= R_s i_d+L_d\frac{di_d}{dt}-\omega_e L_q i_q,\\
u_q &= R_s i_q+L_q\frac{di_q}{dt}
      +\omega_e(L_d i_d+\Psi_r)
\end{aligned}
$$

把每个量替换为 `x_real = x_pu * x_base`，再除以 `U_base`。由基值定义可得：

$$
\begin{aligned}
u_{d,\mathrm{pu}}={}&R_{s,\mathrm{pu}}i_{d,\mathrm{pu}}
 +L_{d,\mathrm{pu}}\frac{\Delta i_{d,\mathrm{pu}}}
 {T_{s,\mathrm{pu}}}
 -\omega_{e,\mathrm{pu}}L_{q,\mathrm{pu}}i_{q,\mathrm{pu}},\\
u_{q,\mathrm{pu}}={}&R_{s,\mathrm{pu}}i_{q,\mathrm{pu}}
 +L_{q,\mathrm{pu}}\frac{\Delta i_{q,\mathrm{pu}}}
 {T_{s,\mathrm{pu}}}
 +\omega_{e,\mathrm{pu}}
 (L_{d,\mathrm{pu}}i_{d,\mathrm{pu}}+\Psi_{r,\mathrm{pu}})
\end{aligned}
$$

这不是另一套物理模型；它必须在相同基值、相同采样周期和相同坐标约定下与真实值模型得到相同的波形。验证方法是同时运行真实值和 PU 模型，比较电流、速度、转矩和饱和点，而不是只比较某个中间变量。

## 3. 基值选择

基值的目标是给预期最大范围留出裕量，同时保留足够的定点分辨率：

- `I_base` 应覆盖正常最大相电流和瞬态余量，不能刚好等于额定值；
- `U_base` 应与模型使用的相电压定义一致，不能把线电压 RMS 直接当相电压峰值；
- `omega_base` 要说明是机械还是电角速度，并覆盖最高运行点；
- `Psi_base`、`T_base` 和时间基由前述方程推导，避免独立拍脑袋设置；
- 调整基值后，要重新检查所有限幅、PI 参数、查表轴和通信显示换算。

标幺值的优先级是“自洽和可验证”，不是追求所有量都恰好等于 1。若电机参数未知，可以用铭牌、功率平衡和低风险测量估算初值，再用实际波形修正。

## 4. Q 格式与定点实现

先在浮点或 Python/Simulink 中验证范围，再选择 Q 格式：

1. 列出每个输入、状态、中间乘积和输出的最大正负范围；
2. 为正常运行和瞬态各保留余量，不能把满量程当成可用量程；
3. 乘法先扩展到更宽类型，做显式缩放和饱和，再缩回目标 Q 格式；
4. 所有除法检查除数为零或接近零；
5. 对积分器、角度累加器、速度和观测器状态分别检查溢出；
6. 用真实值、PU 和定点三套仿真结果交叉比较。

定点实现的常见故障不是“算法不对”，而是移位、符号扩展、乘积宽度或限幅顺序错误。溢出导致的极性翻转尤其容易伪装成坐标、极性或观测器问题。

## 5. 离散滤波器

一阶低通可以按连续截止角频率 `omega_c` 和执行周期 `Ts` 推导：

$$
\begin{aligned}
a&=\frac{1}{1+\omega_cT_s},\\
b&=\frac{\omega_cT_s}{1+\omega_cT_s},\\
y[n]&=a\,y[n-1]+b\,x[n]
\end{aligned}
$$

定点化时把 `a`、`b` 映射到同一 Q 格式，检查量化后 `a+b` 是否仍接近该格式的 1。`Ts` 是滤波器实际执行间隔，不是输入信号刷新间隔；如果速度环比电流环慢，必须使用速度环的执行周期。

量化误差、截止频率、相位滞后和状态初值都要在仿真中检查。不要只替换两个系数就假定滤波器行为不变。

## 6. PWM 更新延时和角度补偿

使用影子寄存器时，本周期计算的比较值可能在下一周期才生效。因此“当前采样的电流”和“当前计算的电压”并不一定属于同一电角度。补偿前先画出：

```text
ADC trigger -> DMA complete -> FOC calculation -> CCR write -> PWM effective
```

要分别审计以下角度用途：

- 电流坐标变换角度；
- 电压重构/观测器使用的电压角度；
- 电流重构角度；
- 上一拍电压角度；
- 电流环反变换和 SVPWM 使用的角度。

不同硬件和计数方式可能对应 `Ts`、`1.5Ts` 或其他延时。课件中的 `1Ts/1.5Ts` 是分析起点，不是固定补偿常数。通常电压重构角度对高速无感性能更敏感，但必须用实际更新点和仿真波形决定补偿极性与幅值。

## 7. 不要混淆这些频率

分别记录并命名：

| 量 | 含义 |
|---|---|
| PWM/开关频率 | 功率器件或调制周期频率 |
| ADC 采样频率 | 采样触发和有效样本产生频率 |
| FOC 计算频率 | 快速控制算法执行频率 |
| 电流/电气频率 | 电机电流或电角度变化频率 |
| 电流环带宽 | 闭环对电流参考的动态响应 |
| 速度环带宽 | 外环对速度参考的动态响应 |

例如 `20 kHz` 的 PWM、采样和 FOC 计算，不等于电流频率 `300 Hz`，也不等于电流环带宽 `500 Hz`。所有带宽公式必须使用对应环路的执行周期。
