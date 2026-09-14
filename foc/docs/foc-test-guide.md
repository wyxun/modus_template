# FOC 使用与电机调试指南

本文面向 FOC 框架调用者、首次适配电机的开发者和台架调试人员。模块职责见
[FOC 当前架构](../foc/docs/foc-architecture.md)。文中的 STM32G431 表格是当前板级实例，不是框架对所有硬件的要求。

## 1. 当前能力与安全边界

当前产品组合面向单台电机，使用位置传感器提供角度。已实现 VOLTAGE、
CURRENT 和 SPEED 模式；位置闭环与无感观测/接管尚未接入。
当前 `motor_Start()` 对所有运行模式都要求位置回调可用，因此没有位置源的
目标不能通过当前 App 启动电机。

开始调试前先牢记几个边界：

- `motor_limits_t` 目前只声明，运行代码不读取其中的限值；不能把它当作有效的电流、速度或调制度保护。
- `motor_params_t` 的极对数参与机械量到电气量的换算。电阻和 D/Q 轴电感当前还用于 SMO 标幺模型，不用于自动计算 PI 参数。电压、电流基准决定 SMO 的 pu 换算；产品示例 12 V / 7 A 是理论初值，应按实机校准。
- 电流与电压接口使用 FOC 归一化值，不是固定的安培或伏特。实际电流基值取决于采样电阻、模拟增益和板级归一化实现；换板时必须重新核对。
- VOLTAGE 模式直接使用电压参考，不提供由 `motor_limits_t` 实现的软件电流限幅。首次上电必须使用硬件过流保护、限流电源和保守参考值。

## 2. 初始化与控制调用流程

当前 MODUS 产品的启动顺序如下：

~~~text
peripheral_Init() → perfc_init() → modus_Init()
                                  │
                                  └─ 自动初始化 foc_app
                                       ├─ 初始化 Encoder 与 Motor
                                       └─ Motor 进入 ADC_CAL

ADC 电流采样完成中断 → foc_app_HighFrequencyISR()
                    → motor_HighFrequencyStep()

modus_Run() 前台调度 → 约每 1 ms 更新一次编码器位置样本
~~~

ADC 校准完成后 Motor 才进入 IDLE。只有 IDLE 且故障已清除时才能启动或对齐。高频步进由目标中断调用；应用层不要从普通业务代码重复调用它。

Motor 控制 API 位于 `foc/motor/motor.h`：

| API | 用途与约束 |
| --- | --- |
| `motor_Init(ptMotor, &config)` | 组合层初始化 Motor 并开始 ADC 校准；当前由 foc_app 自动调用 |
| `motor_Start(ptMotor, mode)` | 从 IDLE 启动 VOLTAGE、CURRENT 或 SPEED 模式 |
| `motor_SetVoltageReference(ptMotor, d, q)` | 更新当前电压模式的 D/Q 参考 |
| `motor_SetCurrentReference(ptMotor, d, q)` | 更新当前电流模式的 D/Q 参考 |
| `motor_SetSpeedReference(ptMotor, speed)` | 更新当前速度模式的电速度参考 |
| `motor_RequestPositionCalibration(ptMotor)` | 请求非阻塞 ALIGN；这不是位置闭环 |
| `motor_Stop(ptMotor)` | 先关闭 PWM，再收敛运行状态 |
| `motor_ClearFault(ptMotor)` | PWM 关闭后清除锁存故障；校准故障会重新校准 |
| `motor_HighFrequencyStep(ptMotor, nowTick)` | 每个目标高频控制中断调用一次；由 App/目标调度负责 |
| `motor_GetStatus(ptMotor, &status)` | 读取状态、故障、模式和 PWM 使能状态 |

调用顺序是先启动指定模式，再设置同一模式的参考；设置函数本身不会切换模式：

~~~c
static void app_SetSpeed(motor_t *ptMotor)
{
    foc_result_t eResult = motor_Start(ptMotor, FOC_MODE_SPEED);

    if (eResult != FOC_RESULT_OK) {
        return;
    }
    eResult = motor_SetSpeedReference(ptMotor, foc_from_float(1.0f));
    if (eResult != FOC_RESULT_OK) {
        motor_Stop(ptMotor);
    }
}
~~~

`ptMotor` 必须是已经由产品组合层初始化并拥有的 Motor 对象。
当前产品把 Motor 放在 `foc_app_t` 内，但 `foc_app.h` 尚未提供公开的
Motor 获取接口；现成固件提供的上层操作入口是下面的 `motor` Shell。
不要在其他模块硬编码访问 `tFocApp.tMotor`。
若业务代码需要直接操作 Motor，应由组合层显式持有或传递 `motor_t *`，
并负责保证初始化和高频调度契约。

## 3. 当前 Shell 命令与单位

启用 MShell 的调试固件提供以下 motor 命令：

| 命令 | 含义 |
| --- | --- |
| `motor status` | 显示生命周期状态、故障位、控制模式和 PWM 状态 |
| `motor encoder` | 显示有效性、机械角度（度）和机械速度（机械圈/秒） |
| `motor align` | 执行固定电角度 D 轴电流对齐并捕获电气零位 |
| `motor current <d> <q>` | 启动电流模式并设置 D/Q 电流参考 |
| `motor speed <e-turn/s>` | 启动速度模式并设置电气圈/秒参考 |
| `motor voltage <d> <q>` | 启动电压模式并设置 D/Q 电压参考 |
| `motor stop` | 立即停止功率级 |
| `motor clear` | 清除可恢复的锁存故障 |

current 和 voltage 的数值是归一化 FOC 值，不是 SI 单位；不同板卡上相同数值可能对应不同的实际电流或电压。speed 使用电气圈/秒（electrical turn/s），机械转速与电速度的关系为：

~~~text
electrical turns/s = mechanical RPM × pole_pairs / 60
mechanical RPM     = electrical turns/s × 60 / pole_pairs
~~~

例如，7 对极电机的 motor speed 1.0 对应约 8.57 rpm 的机械转速目标。实际响应还取决于速度 PI、电流环、负载和电压余量。

## 4. 新电机适配

### 4.1 把新电机接到现有控制板

先确认电机铭牌/数据表和位置传感器配置，再调整 `foc/app/foc_app.c` 中 `MODUS_DECLARE_OBJECT` 的配置。至少核对下表中的参数：

| 配置 | 用途 |
| --- | --- |
| `tParams.chPolePairs` | 必须准确；参与机械角度和速度到电气量的换算 |
| `tParams.wResistanceMilliohm` | 当前必须非零，但不是当前 PI 的自动整定输入 |
| `tParams.wInductanceDMicroHenry` / `wInductanceQMicroHenry` | 当前必须非零；保留为电机元数据，暂不自动整定 PI |
| `tControl.tCurrentPiParams` | D/Q 共用的电流 PI 配置；换电机后需要重新验证 |
| `tControl.tSpeedPiParams` | 速度 PI 配置；输出为 Q 轴电流参考 |
| `tControl.qAlignCurrent` | ALIGN 的 D 轴归一化电流参考，须符合电机与硬件能力 |
| `tControl.wAlignSteps` | ALIGN 高频步数；当前按 20 kHz 调度时 30000 步约为 1.5 秒 |
| `tControl.chSpeedLoopDiv` | 速度 PI 分频；当前为 20，20 kHz 高频步进下速度环为 1 kHz |
| `tEncoderCfg.bDirectionInvert` | 只有实测编码器方向与控制定义相反时才调整 |
| `tEncoderCfg.qSpeedFilterAlpha` | 机械速度滤波系数；调整后检查噪声和延迟 |
| `tEncoderCfg.wInvalidTimeoutUs` | 位置样本有效超时，需大于正常采样间隔并满足安全要求 |

现有数值仅是当前电机的起点，不代表新电机可以直接安全运行。调试前确认电流采样基值、硬件过流阈值、驱动桥能力和电机允许电流，再从低电流、低速度逐步验证。PI 输出与积分器上下限会参与 PI 运算，但 `motor_limits_t` 不会自动覆盖或限制参考值。

### 4.2 换控制板或重新适配功率级

FOC 数学层要求三相输入和输出始终按相同的 U、V、W 顺序解释。适配新板时，在目标硬件层实现 `foc/hal/foc_port.h` 的直接接口，并逐项确认：

1. `foc_adc_Sample()` 返回的 U/V/W 电流与实际驱动桥相位对应，且电流正方向一致。
2. `foc_pwm_SetDuty()` 按同一 U/V/W 顺序更新对应桥臂。
3. 三相采样触发与 `foc_app_HighFrequencyISR()` 的调度匹配；ADC、PWM 更新和中断入口属于目标平台。
4. `foc_pwm_Enable()` 只在提交安全初始占空比后使能，`foc_pwm_Stop()` 能可靠关闭功率输出。
5. 位置源初始化与原始机械角读取在产品/板级组合处接入 Encoder；慢速传感器事务不进入高频中断。

驱动器、放大器、采样电阻、ADC 通道、极性、定时器与引脚均属于板级适配内容，应由对应目标的原理图和外设代码确认，不写成跨硬件通用的 FOC 架构假设。

### 4.3 当前 STM32G431 板级实例

以下映射来自 `peripheral/stm32g431` 的当前实现，只用于核对现有板卡；其他目标不需要采用相同的 ADC 通道、定时器或引脚。

| 电机相位 | 当前电流采样 | 当前 PWM 输出 |
| --- | --- | --- |
| U | `ADC1 injected index 0` | `TIM1 CH1`：PA8 / PC13 互补输出 |
| V | `ADC2 injected index 1` | `TIM1 CH2`：PA9 / PA12 互补输出 |
| W | `ADC2 injected index 0` | `TIM1 CH3`：PA10 / PB15 互补输出 |

该板的电流偏置和归一化位于 `peripheral/stm32g431/foc_port.c`，
PWM 与采样触发由对应 TIM/MDI 适配负责。当前电流标幺换算使用
1390 ADC counts/pu，采样方向为 offset − raw；偏置校准有效范围为
20000–60000 counts，每相累计 512 个样本。
实际安培值仍取决于分流电阻、放大器增益与模拟链路，必须通过原理图和
实测电流确认。若更换相线、放大器或 PCB，应先追踪三相电流与桥臂的
真实对应关系，再同步调整板级采样和 PWM 映射；不要只交换其中一侧的
U/V/W。

## 5. 建议的首次上电调试顺序

所有带电测试都应配备可触及的硬件急停、可靠的硬件过流保护和限流电源。固定好电机并移除可能卷入的机械负载。motor stop 是软件停止指令，不能替代硬件急停。

### 第一步：静态检查与构建

检查电源、驱动桥、采样电阻/放大器、编码器供电与接线，并确认 U/V/W 采样和 PWM 通道逐相一致。先构建调试版：

~~~powershell
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=float
~~~

切换到定点后端时构建：

~~~powershell
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed
~~~

不接电机也可运行 App/Encoder 的 float 与 fixed 主机回归测试：

~~~powershell
powershell -ExecutionPolicy Bypass -File foc/tests/run_encoder_command_test.ps1
~~~

### 第二步：确认启动和 ADC 校准

通过板上 Shell 执行 motor status。上电后 Motor 暂处于 ADC_CAL，
校准完成后进入 IDLE。当前 STM32G431 配置累计 512 组采样；在 20 kHz
下约 25.6 ms。确认 fault=0、pwm=0 后再继续。
state 是 `motor_state_e` 的数值：ADC_CAL=1、IDLE=2、ALIGN=3、
RUNNING=4、FAULT=5。

若状态为 FAULT，先根据故障位查明原因，再使用 motor clear。ADC 校准类故障清除后会重新开始校准，不会直接启动 PWM。

### 第三步：验证编码器方向

保持功率输出关闭，手动缓慢转轴并重复执行 motor encoder。机械角度应随轴连续变化，机械速度方向应与约定一致。若方向相反，先检查传感器安装和 bDirectionInvert 配置，不要通过随意重排 ADC/PWM 通道来修正编码器方向。

### 第四步：ALIGN 与电气零位

确认限流电源、硬件保护和 qAlignCurrent 适合该电机后，再执行 motor align。
ALIGN 使用固定电角度注入 D 轴电流，完成后捕获当前位置对应的电气零位、
关闭 PWM 并回到 IDLE。该过程会对转子施加力矩，测试时应确保轴安全且无人接触。

### 第五步：小参考验证电流环与相序

确认电流 PI 已针对当前硬件设置后，从很小的 D/Q 参考开始，例如：

~~~text
motor current 0.0 0.02
motor status
motor stop
~~~

0.02 只是归一化参考示例，不是通用安全电流值。观察实际 Iq 是否按预期变化、相电流符号是否一致，以及电机有无异常振动或发热；异常时立即 motor stop，再检查相序、采样极性、电气零位和电流环参数。

确认电流环稳定后，再用较低电气速度试运行：

~~~text
motor speed 1.0
motor status
motor stop
~~~

1.0 表示 1 电气圈/秒，不是 1 rpm。结合极对数、母线电压和负载确定合理范围，不要直接沿用旧测试报告中的速度或验收数字。

## 6. 波形、日志与故障定位

当前 float 且启用波形的构建注册以下通道：

| 通道 | 观察内容 | 采样率 |
| --- | --- | ---: |
| `Speed` / `Iq` | 实际电速度 / 实测 Q 轴电流 | 10 kHz |
| `SpeedRef` / `IqRef` | 电速度目标 / Q 轴电流目标 | 1 kHz |
| `WaveSeq` | 高频采样序号，用于观察丢样 | 10 kHz |
| `Sine500` | 波形链路测试信号 | 10 kHz |

fixed 构建不注册这些 float 波形。前台日志 FOC HF ISR avg=... 表示统计窗口内的平均耗时，不是最坏耗时或 P99，不能单独作为高频余量结论。

被动日志和波形采集使用仓库的 AITrace 入口：

~~~powershell
.\tools\aitrace.exe shell log -E -W -I -D
.\tools\aitrace.exe wave capture 5 --output foc.csv
~~~

先看日志和波形，再决定是否需要侵入式定位。CPU halt 或 GDB 会打断实时控制，只能在电机输出确认安全并获得操作确认后使用。

常见现象及优先检查项：

| 现象 | 优先检查 |
| --- | --- |
| motor start 被拒绝 | 是否完成 ADC_CAL、处于 IDLE 且位置源有效 |
| motor encoder 无有效数据 | 传感器供电/接线、初始化、前台更新和样本年龄 |
| 启动后立即进入 FAULT | motor status 故障位；检查 ADC、位置、数学结果和 PWM 提交 |
| 电流符号或转矩方向不符 | U/V/W 逐相映射、电流采样极性、编码器方向与电气零位 |
| 速度环振荡或过流 | 先停机确认电流环稳定，再降低速度 PI 增益和参考值 |
| 波形没有数据 | 是否为 float + MWAVEFORM_ENABLE 构建，以及 RTT/MStudio 通道状态 |

当前故障位定义如下：

| 故障 | 掩码 |
| --- | ---: |
| `MOTOR_FAULT_ADC_CAL` | `0x01` |
| `MOTOR_FAULT_ADC_SAMPLE` | `0x02` |
| `MOTOR_FAULT_POSITION` | `0x04` |
| `MOTOR_FAULT_MATH` | `0x08` |
| `MOTOR_FAULT_PWM` | `0x10` |
| `MOTOR_FAULT_ALIGN` | `0x20` |

## 7. 当前板级配置位置

| 配置内容 | 当前代码位置 |
| --- | --- |
| Motor 参数、PI、ALIGN 和速度环分频 | foc/app/foc_app.c 的 App 对象配置 |
| 编码器滤波、方向和失效超时 | 同一 App 配置中的 tEncoderCfg |
| 通用 ADC/PWM 函数契约 | foc/hal/foc_port.h |
| STM32G431 的 ADC/PWM/位置源实现 | peripheral/stm32g431/foc_port.c |
| STM32G431 定时器、引脚与 ADC 触发 | peripheral/stm32g431/haltim1.c、haladc.c |

修改控制代码前先判断变更属于 Motor/Core 算法、产品组合还是板级适配，再保持这三个边界清楚；不要为了某块板的接线把目标专属信息写进通用架构层。
