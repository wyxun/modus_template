# FOC 当前电感辨识算法说明

本文说明当前代码已经实现的 Phase 1 d 轴增量电感 Ld 辨识。内容以
foc/identify/identify_inductance.c 的实际执行路径为准，用于后续扩展 Lq
和磁链辨识时保持同一套时序、数据交接和安全边界。

当前版本只做 Ld：转子必须在机械上锁住，注入 Vd，保持 Vq = 0，根据
dI/dt 计算电感。Lq、磁链和自动锁轴尚未属于当前实现。

## 1. 辨识前必须知道的量

这些量决定公式中的物理单位、采样时序和结果是否可信。

| 量 | 用途 | 当前来源或要求 |
| --- | --- | --- |
| 高频 ISR 频率 FOC_HF_ISR_HZ | 把采样周期换算成秒，计算半周期和电流斜率 | 编译期固定量 |
| 注入频率 f_inj | 决定正、负电压半周期长度 | identify_inductance_cfg_t |
| 调制度幅值 A | 产生 +Vd/-Vd，不是直接的伏特值 | qModulationAmplitude |
| 母线电压 Vbus | 把调制度换算成实际施加电压 | 标称值或 ADC 实测值 |
| 调制度到电压比例 | 修正调制度与母线电压的对应关系 | FOC_DCBUS_MODULATION_NUM/DEN |
| 电流基准 Ibase | 把电流标幺值换算成 mA | Motor 运行时绑定 |
| 电阻 Rs | 扣除绕组电阻压降 | 已完成的电阻辨识结果，单位 mΩ |
| 命令管线延迟 | 确定新命令何时真正作用到 PWM | FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES |
| 母线采样延迟和数据年龄 | 确认 Vbus 与当前 PWM 属于同一测量窗口 | FOC_DCBUS_SAMPLE_DELAY_CYCLES、FOC_DCBUS_MAX_AGE_CYCLES |
| 电角度有效性和速度 | 确认 FOC 坐标系有效且转子没有转动 | bAngleValid、qElectricalSpeedPu |
| 电流、速度、电压上限 | 防止注入造成过流、转动或过调制 | 配置和编译期上限 |
| 机械锁止条件 | Vq = 0 不能替代机械固定 | 台架/产品测试前置条件 |

母线电压不是调制度。若使用 ADC，板级换算为：

~~~text
Vbus_mV = ADC_count × FOC_DCBUS_MV_PER_COUNT_NUM
          / FOC_DCBUS_MV_PER_COUNT_DEN
          + FOC_DCBUS_OFFSET_MILLIVOLT
~~~

如果没有母线 ADC，可以使用 FOC_DCBUS_SOURCE_NOMINAL 和
FOC_DCBUS_NOMINAL_MILLIVOLT，但结果只能解释为“标称母线电压条件下的
增量电感”。FOC_DCBUS_SOURCE_NONE 或标称电压为零时，辨识保持禁用。

## 2. 配置量与启动时预计算

配置输入为：

~~~text
f_inj   = wInjectionFrequencyHz
D       = hwCaptureDelayCycles
M       = hwCaptureSampleCount
H       = hwHalfCycleCount
A       = qModulationAmplitude
Imax    = qMaxIdentificationCurrent
ΔImin   = qMinCurrentDelta
ωmax    = qMaxElectricalSpeedPu
Nmotion = hwMotionFaultCycles
~~~

启动时一次性计算并保存：

~~~text
T_half_cycles = FOC_HF_ISR_HZ / (2 × f_inj)
capture_start = D
timeout_ms    = ceil(T_half_cycles × H × 1000 / FOC_HF_ISR_HZ) + 500
~~~

两组命令在启动时生成：

~~~text
command[0] = { +A, 0 }
command[1] = { -A, 0 }
~~~

ISR 只根据半周期奇偶读取命令，不重新构造正负命令。

启动校验还必须满足：

~~~text
f_inj <= FOC_HF_ISR_HZ / 2
FOC_HF_ISR_HZ % (2 × f_inj) == 0
D >= FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES
    + FOC_DCBUS_SAMPLE_DELAY_CYCLES
D + M <= T_half_cycles
H >= 4，且 H 为偶数
0 < A <= 1
0 < ΔImin < Imax <= 1
0 < ωmax <= 1
~~~

D 是从半周期开始到捕获窗口开始的总延迟，必须覆盖命令从 Identify
发布到 Motor/PWM 生效的延迟，以及母线采样相对 PWM 的延迟，不能随意填成零。

## 3. 整体时序

前台和 20 kHz 高频 ISR 的职责分开：

~~~text
前台 Start
    │ 校验配置、检查母线源、预计算半周期/命令/超时
    ▼
发布 IDENTIFY_OPERATION_INDUCTANCE
    │
    ├── 每个 HF ISR：Motor 先运行，Identify 后运行
    │
    │   phase=0：发布本半周期的 +Vd 或 -Vd 命令
    │   0..D-1：等待命令管线和母线采样延迟
    │   D..D+M-1：采集 M 个 Id 和 Vbus 样本
    │   半周期结束：保存首末电流差，切换极性
    │
    └── 完成 H 个半周期：发布零电压命令，发布 batch ready
    │
    ▼
前台复制批次并计算正、负极性结果
    │
    ▼
取正负结果平均，停止 Motor，发布最终 Ld
~~~

当前 App 的顺序是 Motor → Identify。因此 Identify 在周期 N 发布的命令，
最早由下一周期的 Motor 消费。FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES 默认
为 1，捕获延迟必须把这个事实计入。

高频 ISR 不做最终电感除法，也不做日志输出；它只做有界的安全检查、注入切换
和采样统计。批次完成后，前台 PT 执行最终换算。

## 4. 每个 ISR 周期保留的安全检查

辨识运行期间以下条件每个 HF ISR 都检查一次：

~~~text
Motor 有故障                       -> 失败并安全停机
电角度无效                         -> 失败并安全停机
母线样本无效或超过最大母线电压     -> 失败并安全停机
PWM 饱和                           -> 失败并安全停机
|Id| 或 |Iq| > Imax                 -> 失败并安全停机
|电角速度| > ωmax                  -> 累计运动故障
~~~

速度越限达到 Nmotion 个连续周期才失败；速度恢复正常时计数清零。这样既不
忽略转动，也不会因为一个瞬时异常立刻误判。

失败路径由 motor_IdentificationAbortIsr() 进入 Motor 的统一故障收敛路径，
不会只停硬件 PWM 而留下 Motor 状态仍为运行中的不一致状态。

## 5. 采样窗口和统计量

对每个半周期，在 D..D+M-1 窗口内保存：

~~~text
I_first = 窗口第一个 Id 样本
I_last  = 窗口最后一个 Id 样本
ΣI      = M 个 Id 样本之和
ΣVbus   = M 个实际母线电压样本之和
~~~

半周期结束时只合并有限统计量：

~~~text
ΔI_half = I_last - I_first
~~~

正、负极性分别累加 ΔI_half、ΣI 和 ΣVbus。设某一极性有 K 个半周期，
则该极性的样本总数为：

~~~text
N = K × M
~~~

最终正负极性各自至少有两个半周期；当前配置要求总半周期数 H >= 4 且为偶数。

## 6. 电感计算公式

下面公式对正、负电压极性分别计算。令 s=+1 表示 +Vd 半周期，
s=-1 表示 -Vd 半周期。

### 6.1 平均母线电压和有效注入电压

~~~text
Vbus_mean = ΣVbus / N

Veff = Vbus_mean × A
       × FOC_DCBUS_MODULATION_NUM
       / FOC_DCBUS_MODULATION_DEN
~~~

Veff 是注入电压幅值，单位 mV。正半周期的命令电压为 +Veff，负半周期
的命令电压为 -Veff。

### 6.2 平均电流

~~~text
Imean_pu = ΣI / N
Imean_mA = Imean_pu × Ibase
~~~

Imean_mA 保留符号，单位 mA。

### 6.3 电流斜率

首末样本之间有 M-1 个采样间隔：

~~~text
dt    = (M - 1) / FOC_HF_ISR_HZ
ΔI_pu = Σ(ΔI_half) / K
ΔI_mA = ΔI_pu × Ibase

di_dt = ΔI_mA / dt
      = ΔI_pu × Ibase × FOC_HF_ISR_HZ / (M - 1)
~~~

di_dt 单位为 mA/s。代码使用首末点差分，不引入回归器或动态拟合器。

### 6.4 扣除电阻压降

电流变化由净电压驱动，不是由完整注入电压直接驱动。因此必须先扣除
Rs 压降：

~~~text
Vnet_mV = s × Veff - Rs_mΩ × Imean_mA / 1000
~~~

Rs_mΩ × Imean_mA / 1000 的结果单位为 mV。

### 6.5 单极性电感和最终结果

~~~text
Ld_s_μH = Vnet_mV × 1,000,000 / di_dt_mA_per_s
~~~

乘以 1,000,000 是因为 mV/(mA/s) 的数值对应 H，需要转换成 μH。

必须满足：

~~~text
abs(ΔI_pu) >= ΔImin
Vnet_mV × di_dt_mA_per_s > 0
Ld_s_μH > 0 且在可表示范围内
~~~

正、负极性都有效后，最终结果为：

~~~text
Ld = (Ld_positive + Ld_negative) / 2
~~~

同时输出两极性的平均有效电压、平均电流、注入频率、捕获样本数和半周期数。

## 7. FLOAT 和 FIXED 的差异

算法相同，只是中间数值表示不同。

### FLOAT

- ISR 直接累加 foc_scalar_t 电流值，不做每样本的 foc_to_float() 转换。
- 前台用 float 完成平均值、Rs 压降、斜率和电感换算。
- fVoltageScale 在启动时预计算为：

~~~text
A × FOC_DCBUS_MODULATION_NUM / FOC_DCBUS_MODULATION_DEN
~~~

### FIXED

- 电流以 FOC_Q_SCALE 表示。
- 前台使用整数比例计算平均电流、有效电压和斜率。
- 同样应用 FOC_DCBUS_MODULATION_NUM/DEN，不能把调制度误当成实际电压。

最终计算只在前台发生；ISR 只保留采样、累加和有限状态切换。

## 8. 数值例子

假设：

~~~text
FOC_HF_ISR_HZ = 20,000 Hz
f_inj         = 1,000 Hz
M             = 4
H             = 4
A             = 0.1
Vbus          = 12,000 mV
调制度比例    = 1/2
Ibase         = 1,000 mA
Rs            = 100 mΩ
ΔI_pu         = 0.15
~~~

则：

~~~text
T_half_cycles = 20,000 / (2 × 1,000) = 10
Veff          = 12,000 × 0.1 × 1/2 = 600 mV
di_dt         = 0.15 × 1,000 × 20,000 / (4 - 1)
              = 1,000,000 mA/s
Rs_drop       = 100 × Imean_mA / 1000
~~~

如果该极性的平均电流为 75 mA：

~~~text
Vnet = 600 - 100 × 75 / 1000 = 592.5 mV
Ld   = 592.5 × 1,000,000 / 1,000,000
     = 592.5 μH
~~~

实际代码会对正、负两个极性分别计算，再取平均。

## 9. 当前实现的边界和后续扩展

当前实现明确保持以下边界：

1. 只辨识锁轴条件下的 d 轴增量电感 Ld。
2. 不在 20 kHz ISR 中执行最终电感计算、日志或浮点转换。
3. Motor 负责 PWM、运行状态和故障收敛，Identify 不能直接绕过 Motor 操作硬件。
4. identify_inductance_cfg_t 只用于启动校验和预计算，不作为长期运行配置镜像。
5. 电阻辨识保持独立，不改变其已验证的流程和文件结构。

后续增加 Lq 时，复用“配置校验 → ISR 注入/采样 → 前台批次换算”框架，
只改变注入轴和锁轴准入条件；增加磁链辨识时，单独定义反电动势/角度相关的
采样窗口和公式，不把这些物理量硬塞进当前 Ld 公式。

## 10. 验收时必须记录的测试条件

不能只记录一个“电感值”。每次结果至少记录：

~~~text
电机型号和接线方式
机械是否锁轴
母线来源：ADC 实测或标称值
实际/标称母线电压
注入频率 f_inj
调制度 A 和调制度比例
Rs
平均电流
捕获样本数 M 和半周期数 H
Ld 结果及正、负极性单独结果
~~~

结果应表述为“指定电压、频率和电流工作点下的增量 Ld”，而不是脱离测试
条件的唯一电感常数。

