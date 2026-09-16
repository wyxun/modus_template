# FOC Encoder / PU / 参数识别 / SMO 旁观验证操作说明

## 1. 本次验证目标

本次以已验证的基线提交为对照：

```text
a11c186443cbbd9767938add0c9e59bf502cd37b
```

基线已经验证过 Encoder 下的：

- `speed` 模式；
- `current` 模式；
- `voltage` 模式；
- `align` 电气零位对齐。

本次只按以下顺序验证，不跨阶段判断：

```text
PU 改造后的 Encoder 三种模式
        ↓
离线参数识别
        ↓
识别参数用于 SMO 初始化
        ↓
Encoder 主控 + SMO 旁观比较
        ↓
后续再做强拖启动和无感接管
```

当前代码中：

- Encoder 仍然是 Motor 的实际角度和速度来源；
- SMO 打开时只在 Motor 内部旁观运行，不接管 Core；
- HFI 只有宏预留，没有注入、解调和角度估算；
- 强拖开环启动、SMO 接管和融合尚未实现；
- 离线识别结果目前不会自动回写 `motor_params_t`，需要手动更新配置后再构建 SMO 固件。

## 2. 当前板卡和单位

目标芯片：STM32G431，170 MHz，PWM/FOC 高频周期 20 kHz，即 50 μs。

当前示例电机参数：

| 项目 | 当前值 |
| --- | ---: |
| 极对数 | 7 |
| 电压基准 | 12000 mV |
| 电流基准 | 7000 mA |
| 电速度基准 | 100 electrical turns/s |
| PWM 频率 | 20000 Hz |

`motor current`、`motor voltage` 和 `motor speed` 的输入都是 PU，不是安培、伏特或 rpm。

```text
电角速度 turns/s = speed_pu × 100
机械 rpm = 电角速度 turns/s × 60 / 7
```

例如：

```text
speed 0.10 pu = 10 electrical turns/s ≈ 85.7 mechanical rpm
```

第一次上电不要直接使用 1.0 pu，先使用此前已经确认安全的低参考值。

## 3. 上电前安全检查

1. 电机固定可靠，轴上无可能卷入的物体。
2. 使用限流电源，硬件过流保护和急停可用。
3. 确认三相 ADC 采样和 PWM 输出仍按 U/V/W 对应。
4. 确认编码器供电、通信、方向和机械角读数正常。
5. 软件 `motor stop` 不能代替硬件急停。
6. 首次测试时手放在急停位置，不要接触旋转中的电机。

## 4. Host 回归测试

在仓库根目录执行：

```powershell
$env:Path = 'D:\software\msys64\mingw64\bin;' + $env:Path
Get-ChildItem .\foc\tests\run_*.ps1 |
    Sort-Object Name |
    ForEach-Object {
        & $_.FullName
        if ($LASTEXITCODE -ne 0) {
            throw "$($_.Name) failed with exit code $LASTEXITCODE"
        }
    }
```

判定：所有测试脚本返回 0。当前已验证的脚本包括 Core、App、Identify、Motor PU、Observer 和 SMO 的 FLOAT/FIXED 测试。

## 5. 固件构建命令

工程使用 LLVM Embedded Toolchain for Arm，构建工具和 OpenOCD 位于 `D:/0_software`。

建议每次切换 `FOC_NUMERIC`、`FOC_ENABLE_SMO` 或 `FOC_EXPERIMENTAL_IDENTIFY` 时都显式写出参数。构建配置签名会根据这些宏重新编译相关目标文件。

### 5.1 第一阶段：Encoder-only

先构建浮点调试固件：

```powershell
& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' `
    SW_ROOT=D:/0_software `
    TARGET_CHIP=stm32g431 `
    BUILD=debug-rel `
    FOC_NUMERIC=float `
    FOC_ENABLE_SMO=0 `
    FOC_ENABLE_HFI=0 `
    FOC_EXPERIMENTAL_IDENTIFY=0
```

如需验证固定点编译，再构建：

```powershell
& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' `
    SW_ROOT=D:/0_software `
    TARGET_CHIP=stm32g431 `
    BUILD=debug-rel `
    FOC_NUMERIC=fixed `
    FOC_ENABLE_SMO=0 `
    FOC_ENABLE_HFI=0 `
    FOC_EXPERIMENTAL_IDENTIFY=0
```

确认生成：

```text
build/template.elf
build/template.hex
```

### 5.2 第二阶段：Encoder + 离线参数识别

参数识别使用 Encoder 主控，关闭 SMO：

```powershell
& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' `
    SW_ROOT=D:/software `
    TARGET_CHIP=stm32g431 `
    BUILD=debug-rel `
    FOC_NUMERIC=float `
    FOC_ENABLE_SMO=0 `
    FOC_ENABLE_HFI=0 `
    FOC_EXPERIMENTAL_IDENTIFY=1
```

### 5.3 第三阶段：Encoder 主控 + SMO 旁观

先使用浮点构建旁观版本：

```powershell
& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' `
    SW_ROOT=D:/software `
    TARGET_CHIP=stm32g431 `
    BUILD=debug-rel `
    FOC_NUMERIC=float `
    FOC_ENABLE_SMO=1 `
    FOC_ENABLE_HFI=0 `
    FOC_EXPERIMENTAL_IDENTIFY=0
```

此版本中 SMO 会在 `motor_Init()` 内实例化，在每个 Motor 高频周期运行，但 SMO 输出不会用于 Park、速度环或 PWM 控制。

## 6. 烧录和启动检查

确认调试器连接后烧录当前构建：

```powershell
& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' `
    SW_ROOT=D:/0_software `
    TARGET_CHIP=stm32g431 `
    flash
```

烧录后先不要启动电机，执行：

```text
motor status
```

等待 Motor 从 `ADC_CAL` 进入 `IDLE`。

初始状态应满足：

```text
state = IDLE (2)
fault = 0
pwm = 0
```

如果进入 FAULT：

1. 立即确认功率级和电机状态；
2. 读取 `motor status` 的故障位；
3. 排除硬件原因后再执行 `motor clear`；
4. 不要在故障原因未确认时反复 clear/start。

## 7. 第一阶段：验证 Encoder 下三种 PU 命令

### 7.1 验证编码器读数

PWM 关闭时缓慢转动电机轴，执行：

```text
motor encoder
```

检查：

- `valid=1`；
- 机械角度连续变化；
- 正反转方向符合约定；
- 机械速度符号正确；
- 没有 `encoder data unavailable`。

### 7.2 执行 Align

```text
motor align
```

等待再次回到：

```text
motor status
```

确认状态为 `IDLE`、故障为 0、PWM 为 0。Align 完成后才继续三种控制模式。

### 7.3 Current 模式

使用低电流 PU 参考值。下面只是操作格式示例，实际值以硬件允许范围和此前安全值为准：

```text
motor current 0.0 0.02
motor status
motor stop
```

记录：

- 是否正常进入 RUNNING；
- 实际 `Iq` 是否随 `IqRef` 变化；
- 三相电流方向和幅值是否合理；
- 是否有振动、异常噪声、快速发热或过流；
- 停止后是否回到 IDLE、PWM 是否关闭。

### 7.4 Voltage 模式

Voltage 模式没有软件电流闭环保护，必须使用更保守的低电压 PU 值：

```text
motor voltage 0.0 0.01
motor status
motor stop
```

检查同样的电流、方向、振动、故障和停止结果。任何异常立即执行：

```text
motor stop
```

### 7.5 Speed 模式

从低电速度 PU 开始：

```text
motor speed 0.05
motor status
motor stop
```

检查：

- 速度方向是否正确；
- 速度是否能跟随参考；
- `Iq` 是否没有持续饱和；
- 是否有明显振荡或过流；
- 停止后是否回到 IDLE。

第一阶段通过条件：

- 三种命令都能正常启动和停止；
- PU 参考值没有被错误当作 SI 单位；
- Align 后电气角度正确；
- 无 ADC、位置、数学或 PWM 故障；
- Encoder 仍然是实际控制角度和速度来源。

## 8. 第二阶段：离线参数识别

使用 `FOC_EXPERIMENTAL_IDENTIFY=1` 的固件重新烧录。上电后依次执行：

```text
motor status
motor encoder
motor align
```

确认 Align 完成并回到 IDLE 后执行：

```text
motor identify start
```

识别过程中不要执行 speed/current/voltage/align 命令。使用下面命令查看阶段：

```text
motor identify status
```

等待状态变为 `COMPLETE`。完成后记录：

- `identify PU: Rs=..., Ld=..., Lq=...`；
- `identify SI: Rs=... ohm, Ld=... uH, Lq=... uH`；
- 识别期间是否有移动、过流、PWM 故障或 Encoder 失效；
- 重复识别结果是否稳定。

识别完成或失败后再次执行 `motor identify status`，还会输出 RS、Ld、Lq
三个阶段的诊断快照，包括 `Istart`、`Ilast`、`deltaI`、`sumV`、使用的
`R`、阶段结果和采样周期数。`valid=0` 表示该阶段没有得到有效完成结果；
失败状态还会输出失败阶段编号。

Rs、Ld、Lq 各自执行 3 次独立试验，使用三次结果的中位数，并要求最大最小
差不超过中位数的 20%。Ld/Lq 每次使用 64 个采样点、每 4 点一组，对离散
RL 响应做线性拟合；`sumV` 记录实际施加的脉冲电压，仅用于正值和有限值
检查，不直接代替斜率计算电感。这样可降低恒定电流采样偏置和单次采样噪声
对结果的影响；FOC 周期内只做定长标量累加，拟合和中位数判断在阶段边界
执行，不包含日志、排序循环或阻塞等待。

电机动作 API 与识别采样是异步交接的：当前 ADC 电流样本必须配对“上一拍
已经提交给 Motor 的电压命令”，不能配对刚计算出的下一拍参考值。应用层用
内部缓存保存该上一拍命令，保持现有 Motor API 和识别接口不变。Lq 完成后
会施加一个等时反向 Q 轴脉冲抵消平均转矩；这只能减小偏移，不能替代机械
固定，识别 Lq 时仍应固定转子。

若 `sumV<=0`、拟合斜率方向错误、响应不足或三次结果离散过大，识别会安全
失败而不会输出假参数。

当前识别结果不会自动写回 Motor 配置。SMO 使用的 Motor 参数字段是物理单位：

```text
wResistanceMilliohm
wInductanceDMicroHenry
wInductanceQMicroHenry
```

不要把识别输出的 PU 数值直接填入这些字段；应填入对应的 mΩ 和 μH 物理值，再重新构建 SMO 固件。

## 9. 第三阶段：SMO 旁观比较

先将识别得到的物理参数手动更新到 App 的 Motor 参数配置，然后构建 `FOC_ENABLE_SMO=1` 的固件并烧录。

启动后仍然执行：

```text
motor status
motor encoder
motor align
```

然后用 Encoder 运行低速到中高速测试，例如：

```text
motor speed 0.05
motor speed 0.10
motor speed 0.20
motor stop
```

SMO 适合中高速，不要以静止或极低速时的 SMO 角度作为最终结论。强拖和无感启动尚未实现，本阶段不能断开 Encoder，也不能让 SMO 接管 Core。

### 9.1 需要观察的变量

在调试器中观察 Motor 内部：

```text
tMotor.tInput.tElectricalAngle
tMotor.tObserver.tOutput.tElectricalAngle
tMotor.tObserver.tOutput.qElectricalSpeedTurnsPerSecond
tMotor.tObserver.tOutput.bValid
```

编码器电角度是 `tInput.tElectricalAngle`，SMO 电角度是 `tObserver.tOutput.tElectricalAngle`。角度差必须按 BAM32 回绕后的最短差值计算，不能直接做无符号减法。

当前 SMO 速度输出字段是 electrical turns/s，不是 PU。与 Encoder 速度比较时先换算：

```text
SMO speed PU = SMO electrical turns/s / 100
```

### 9.2 旁观阶段记录表

| 速度参考 PU | Encoder 电角度 | SMO 电角度 | 最短角度差 | SMO valid | ISR avg cycles | 故障 |
| ---: | --- | --- | ---: | --- | ---: | --- |
| 0.05 |  |  |  |  |  |  |
| 0.10 |  |  |  |  |  |  |
| 0.20 |  |  |  |  |  |  |

旁观阶段通过条件：

- `motor status` 无故障；
- Encoder 控制行为与 SMO=0 时一致；
- SMO `bValid` 在有足够反电动势后有效；
- SMO 角度没有持续跳变；
- 随速度上升，SMO 与 Encoder 的角度差进入稳定范围；
- 旁观 SMO 没有导致 PWM、速度环或电流环异常；
- 高频 ISR 平均耗时有记录，并确认没有超过 50 μs 周期预算。

注意：当前代码只输出 ISR 平均耗时，不输出最大耗时；平均值不能替代最坏耗时检查。首次打开 SMO 时建议同时用 DWT、逻辑分析仪或调试器记录最大周期。

## 10. 失败处理

### 编码器无效

```text
motor stop
motor status
motor encoder
```

检查 Encoder 供电、通信、前台更新频率、方向配置和样本超时。

### 启动后过流或剧烈振动

```text
motor stop
```

优先检查：

1. Align 电气零位；
2. 编码器方向；
3. U/V/W 相序；
4. 电流采样极性；
5. 电流 PI；
6. PU 参考值是否过大。

### PWM Break 故障

先排除硬件过流、母线异常、驱动器保护或死区问题，再执行：

```text
motor clear
motor status
```

### SMO 无效

不要立即判断算法失败。先确认：

- `FOC_ENABLE_SMO=1` 确实参与了构建；
- Motor 参数基准和 Rs/Ld/Lq 已正确配置；
- 电流和模型电压输入有效；
- 电机已经运行到有足够反电动势的速度；
- CORDIC 和 ISR 没有异常。

## 11. 本次验证结束标准

只有完成以下项目后，才进入下一轮无感功能开发：

```text
[ ] Encoder + PU 的 current 通过
[ ] Encoder + PU 的 voltage 通过
[ ] Encoder + PU 的 speed 通过
[ ] Align 重复执行正常
[ ] 离线 Rs/Ld/Lq 识别完成且结果稳定
[ ] 识别物理参数已正确填入 Motor 配置
[ ] Encoder 主控 + SMO 旁观构建通过
[ ] SMO 角度与 Encoder 差异已记录
[ ] SMO 旁观没有破坏 Encoder 控制和 ISR 周期
```

本文件完成的最后一项不是“无感运行成功”。强拖启动、低速 HFI、SMO 中高速接管、HFI/SMO 融合和失锁回退属于后续独立验证阶段。
