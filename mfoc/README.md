# MFOC

MFOC 是面向电机控制的嵌入式 C 库，包含电流环、速度环、位置反馈、强拖启动和参数辨识所需的模块。本仓库的 STM32G431 应用位于 [`class/mfoc_app.c`](../class/mfoc_app.c)，可作为接入参考。

## 文档入口

| 要做的事 | 阅读入口 |
| --- | --- |
| 理解核心架构、单拍数据流、状态所有权（持续完善） | [框架总览](docs/framework.md) |
| 阅读模块接口 | [Feedback 位置与速度反馈接口](docs/modules/feedback.md) · [Core](docs/framework.md#core) · [Identification](docs/framework.md#identification) |
| 配置无感观测器，核对估算角速和有效条件 | [Flux 非线性磁链观测器](docs/modules/flux.md) · [SMO 滑模反电势观测器](docs/modules/smo.md) |
| 接入有感位置源，理解角速如何进入 Feedback | [Encoder 位置组件](docs/modules/encoder.md) · [Hall 位置组件](docs/modules/hall.md) |
| 选择反馈源、设置基值或修改板级换算 | [Port 相关配置和头文件](docs/configuration.md) |
| 把库接到新的功率板 | [硬件接口移植指南](docs/port-contract.md) · [可复制的 C 模板](port/template/mfoc_port_template.c) |
| 核对 G431 的主机测试、构建和验证范围 | [Port 迁移验证记录](docs/validation/2026-11-10-port-migration.md) |

## 目录与接口

| 目录或文件 | 用途 |
| --- | --- |
| `mfoc_config.h` | 库功能开关和部分算法配置 |
| `mfoc_motor_config.h` | 电机规格、标幺基值和启动参数 |
| `math/`、`control/` | 数值、坐标变换、PI 和调制 |
| `core/` | 电流环及电压调制计算 |
| `motor/` | 电机运行、反馈选择与启动流程 |
| `position/` | 编码器及无感位置算法 |
| `identification/` | 电机参数辨识 |
| `port/` | 电流、母线、统一电角度/速度、PWM 和时钟的接入契约 |
| `tests/`、`tools/` | 测试和代码生成工具 |

各模块的公共头文件位于其 `include/`，实现位于 `src/`。`mfoc.h` 汇集基础数学及控制接口；使用 Motor 时还需包含 `mfoc_motor.h`。`mfoc.mk` 提供本工程使用的源码与头文件路径清单。

## 配置 MFOC

### 数值和三角函数

构建时选择 FLOAT 或 FIXED，不能同时选择。当前工程与 G431 目标均通过 make 参数 `MFOC_NUMERIC=float` 或 `MFOC_NUMERIC=fixed` 选择。

`MFOC_TRIG_BACKEND` 选择三角函数实现：`0` 为 LIBM、`1` 为 LUT、`2` 为外部实现；默认 LUT。FIXED 不支持 LIBM，外部实现需按 [`mfoc_trig.h`](math/include/mfoc_trig.h) 提供函数。

### 库功能

在 [`mfoc_config.h`](mfoc_config.h) 中配置：

| 配置 | 含义 | 当前默认值 |
| --- | --- | --- |
| `MFOC_CORE_M_MAX` | dq 电压矢量相对母线的调制上限 | `0.55f` |
| `MFOC_MOTOR_RUN_FREQ_HZ` | 控制中断、电流环、采样、HFI 与辨识共用的频率 | `20000U`，须与 PWM/ADC 触发一致 |
| `MFOC_SPEED_RUN_FREQ_HZ` | 速度环频率，`MFOC_SPEED_LOOP_DIVIDER` 由它派生 | `1000U`，`0` 关闭速度模式 |
| `MFOC_ISR_MEASURE` | 中断耗时统计开关 | `0`，G431 调试构建覆盖为 `1` |
| `MFOC_ISR_BUDGET_TICKS` | 中断耗时预算，按计时器频率设置 | `8500` |
| `MFOC_IDENTIFICATION_ENABLE` | Motor 参数辨识流程 | `0` |
| `MFOC_FEEDBACK_ENABLE_POSITION` | 板级提供的统一位置反馈；旧 `ENABLE_ENCODER` 名称兼容 | `1` |
| `MFOC_FEEDBACK_ENABLE_SMO` | SMO 观测器 | `1` |
| `MFOC_FEEDBACK_ENABLE_HFI` | 高频注入 | `0` |
| `MFOC_FEEDBACK_ENABLE_FLUX` | 磁链观测器 | `0` |
| `MFOC_FEEDBACK_ENABLE_EKF` | EKF 观测器 | `0`，仅 FLOAT |
| `MFOC_FEEDBACK_BACKEND` | 选作控制输出的位置反馈 | `MFOC_FEEDBACK_SMO` |

使能算法与选择控制输出是两项配置：所选 backend 对应的算法必须使能。未选作 backend 的已使能算法可用于影子观察。HFI 会产生注入电压；Flux 和 EKF 的增益、噪声参数等需要针对实际电机配置及验证。文件中还包含 Rs/L 辨识、Flux 和 EKF 的参数，具体定义以配置头为准。

### 电机与标幺值

在 [`mfoc_motor_config.h`](mfoc_motor_config.h) 中填写相电阻、Ld、Lq、磁链、转动惯量和极对数，并设置电流、电压、电角速度基值。其余电阻、电感、磁链等基值由这三项基值推导。当前文件中的 2804 电机规格及启动阈值是本工程配置，换电机时需重新核对。

电流使用 `I/Ibase`，母线使用 `Vdc/Vbase`，速度使用电角速度与电角速度基值之比。当前电流基值是相电流峰值；例如 `0.02 pu` 对应 `0.02 × 3.5 A = 0.07 A`。极对数用于机械角与电角的换算。

当前示例应用的电流 PI、速度 PI、运行限值、采样频率和速度环分频在 [`class/mfoc_app.c`](../class/mfoc_app.c) 的 Motor 配置中设置；修改电机参数或控制频率后应重新整定并核对这些值。

## 硬件端口

新板实现 `mfoc_port.h` 声明的操作。电流、母线及速度使用共同基值，电角度使用 BAM32，三相 duty 取值为 0 到 1。G431 示例在板级持有 Encoder，前台发布机械采样，ISR 读取推算的电角度和速度。接入步骤和完整接口清单见 [硬件接口移植指南](docs/port-contract.md)。

## 接入与运行

1. 完成上述库、电机及板级端口配置，将 `mfoc.mk` 的头文件路径和库源码加入工程，并为目标选入 Port 实现和中断入口。
2. 初始化时钟、ADC 触发、PWM 和所需传感器；相输出保持关闭。调用 `mfoc_motor_Init()`，由其初始化控制对象和端口零偏。
3. 前台周期调用 `mfoc_motor_Run()`，处理反馈读取和对齐等前台流程。控制中断每个有效采样周期调用一次 `mfoc_motor_Step()`。
4. 根据需要配置或更新母线值。`mfoc_motor_SetBus()` 接收正的 `Vdc/Vbase`；`mfoc_motor_UpdateBus()` 可读取已发布的母线采样并更新。动态母线异常的处理由应用结合硬件保护策略决定。
5. 根据控制模式设置指令并启动；停止使用 `mfoc_motor_Stop()`，故障清除使用 `mfoc_motor_ClearFault()`。调用条件和返回值见 [`mfoc_motor.h`](motor/include/mfoc_motor.h)。

本工程 G431 的应用和中断接线见 [应用](../class/mfoc_app.c) 与 [控制中断](../target/stm32g431/mfoc_motor_irq.c)。示例控制周期为 20 kHz，移植时应以实际采样和 PWM 时序配置为准。

### G431 示例命令

构建 `stm32g431` 目标并连接 RTT、启用 mshell 后，可使用：

```text
motor status
motor encoder
motor align
motor current <id_pu> <iq_pu>
motor speed <speed_pu>
motor openloop <speed_pu> <id_pu>
motor sensorless <speed_pu> <current_pu>
motor stop
motor clear
```

`align` 执行固定磁场对齐和编码器零位采集；`current`、`speed` 为对应闭环指令；`openloop` 是强拖；`sensorless` 使用所选 SMO/Flux 后端进行无感启动。无感接管及参数仍需针对实际电机和负载验证。`motor status` 当前打印应用状态和最近一次结果，不包含完整 Motor 故障详情；`clear` 调用故障清除接口，不会自动启动电机。更多命令以 `motor help` 为准。

在仓库根目录可使用 `mingw32-make TARGET_CHIP=stm32g431` 构建；`make.bat auto` 会清理、构建、烧录并启动 RTT，请在目标和硬件配置确认后使用。

## 测试入口

主机测试入口为 `tests/scripts/run_tests.ps1`，ARM 编译检查入口为 `tests/scripts/check_arm.ps1`；各算法也有独立脚本。测试是软件验证，实际采样时序、电流极性、中断耗时和无感接管仍需台架验证。
