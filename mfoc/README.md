# MFOC

MFOC 是面向电机控制的嵌入式 C 库，包含电流环、速度环、位置反馈、强拖启动和参数辨识所需的模块。首次使用可从 [主机回放示例](examples/host_replay/README.md) 开始。

## 文档入口

| 要做的事 | 阅读入口 |
| --- | --- |
| 运行主机示例并接线到自己的板 | [快速接入](docs/getting-started.md) · [扩展与迁移](docs/extension-guide.md) |
| 确认实机支持、源码包身份及复验步骤 | [发布支持矩阵](docs/support-matrix.md) · [交付核对](docs/release-v1-checklist.md) · [变更记录](CHANGELOG.md) |
| 理解核心架构、单拍数据流、状态所有权（持续完善） | [框架总览](docs/framework.md) |
| 核对 PU 单位、Q15 范围、舍入与积分精度 | [数值契约](docs/numeric-contract.md) |
| 阅读模块接口 | [Feedback 位置与速度反馈接口](docs/modules/feedback.md) · [Core](docs/framework.md#core) · [Identification](docs/framework.md#identification) |
| 配置无感观测器，核对估算角速和有效条件 | [Flux 非线性磁链观测器](docs/modules/flux.md) · [SMO 滑模反电势观测器](docs/modules/smo.md) |
| 接入有感位置源，理解角速如何进入 Feedback | [Encoder 位置组件](docs/modules/encoder.md) · [Hall 位置组件](docs/modules/hall.md) |
| 选择反馈源、设置基值或修改板级换算 | [Port 相关配置和头文件](docs/configuration.md) |
| 把库接到新的功率板 | [硬件接口移植指南](docs/port-contract.md) · [可复制的 C 模板](port/template/mfoc_port_template.c) |
| 核对 G431 的主机测试、构建和验证范围 | [Port 迁移验证记录](docs/validation/2026-11-10-port-migration.md) |

## 目录与接口

| 目录或文件 | 用途 |
| --- | --- |
| `mfoc_config.h` | 项目常用设置，包括速度环、反馈选择和功能开关 |
| `include/` | 公共入口、电机配置选择入口及库内共用的配置派生头 |
| `config/board_template.h` | 板级配置模板和独立构建的默认值 |
| `config/motors/` | 2804 默认配置和可复制的电机配置模板 |
| `math/`、`control/` | 数值、坐标变换、PI 和调制 |
| `core/` | 电流环及电压调制计算 |
| `motor/` | 电机运行、反馈选择与启动流程 |
| `position/` | 编码器及无感位置算法 |
| `identification/` | 电机参数辨识 |
| `port/` | 电流、母线、统一电角度/速度、PWM 和时钟的接入契约 |
| `tests/`、`tools/` | 主工程的测试和代码生成工具 |

各模块的公共头文件位于其 `include/`，实现位于 `src/`。[`mfoc.h`](include/mfoc.h) 汇集基础数学及控制接口。使用 Motor 时还需包含 `mfoc_motor.h`。`mfoc.mk` 提供源码与头文件路径清单。

独立工程可使用导出的 `mfoc/` 源码包，在自己的 Makefile 中包含 `mfoc/mfoc.mk`，用 `MFOC_SOURCES`、`MFOC_INCLUDES` 和自行实现的 Port 编译链接。所有编译单元使用同一组数值后端、位置源和辨识开关。使用自定义电机配置时，添加 `MFOC_MOTOR_CONFIG_FLAGS` 到库和应用的编译参数。主机示例使用 C11、GNU Make 和标准数学库，直接链接库内 Motor、Feedback 与辨识实现。[主机回放示例](examples/host_replay/README.md) · [Port 接入指南](docs/port-contract.md)

## 配置 MFOC

集成 perfc 的工程可为库和调用方统一设置 `-DMFOC_USE_PERFC_FSM=1`，并提供 perfc 的头文件路径和移植配置。G431 已启用此选项，`mfoc_fsm_rt_t` 是 `fsm_rt_t` 的别名，辨识前台使用 `perfc_task_pt.h` 的原生 PT 宏。独立工程默认使用 MFOC 的返回类型和 C11 PT 实现。[公共类型](math/include/mfoc_types.h) · [PT 接口](include/mfoc_pt.h)

### 数值和三角函数

构建时选择 FLOAT 或 FIXED，不能同时选择。当前工程与 G431 目标均通过 make 参数 `MFOC_NUMERIC=float` 或 `MFOC_NUMERIC=fixed` 选择。

`MFOC_TRIG_BACKEND` 选择三角函数实现：`0` 为 LIBM、`1` 为 LUT、`2` 为外部实现；默认 LUT。FIXED 不支持 LIBM，外部实现需按 [`mfoc_trig.h`](math/include/mfoc_trig.h) 提供函数。

### 库功能

在 [`mfoc_config.h`](mfoc_config.h) 中配置：

| 配置 | 含义 | 当前默认值 |
| --- | --- | --- |
| `MFOC_SPEED_RUN_FREQ_HZ` | 速度环频率，按板级控制频率自动计算分频 | `1000U`，`0` 关闭速度模式 |
| `MFOC_IDENTIFICATION_ENABLE` | Motor 参数辨识流程 | `0` |
| `MFOC_MOTOR_MODEL_UPDATE_ENABLE` | 停机后显式应用 PU 电机参数 | `0` |
| `MFOC_FEEDBACK_ENABLE_POSITION` | 板级提供的统一位置反馈；旧 `ENABLE_ENCODER` 名称兼容 | `1` |
| `MFOC_FEEDBACK_ENABLE_SMO` | SMO 观测器 | `0` |
| `MFOC_FEEDBACK_ENABLE_HFI` | 高频注入 | `0` |
| `MFOC_FEEDBACK_ENABLE_FLUX` | 磁链观测器 | `1` |
| `MFOC_FEEDBACK_ENABLE_EKF` | EKF 观测器 | `0`，仅 FLOAT |
| `MFOC_FEEDBACK_BACKEND` | 选作控制输出的位置反馈 | `MFOC_FEEDBACK_FLUX` |

所选控制反馈对应的算法必须使能。其余已使能算法可用于影子观察。HFI 主反馈闭环允许注入，POSITION 仅在显式 `HfiStart` 时允许注入。普通非 HFI 闭环的 `feedback.observe_hfi` 可启用无注入诊断。Flux、EKF 和参数辨识的调参默认值由各模块提供，项目可按需用编译宏覆盖。[高级参数](docs/configuration.md#advanced-settings)

### 板级设置

G431 和 AT32F413 的控制频率默认都是 20 kHz，由各自的 `target.mk` 同时提供给 PWM/ADC 和 MFOC。计时器频率和允许的调制上限也在板级设置。移植新板时可从库内模板取得控制频率和调制上限的写法，环路分频由库在编译期计算。中断耗时统计及其计时配置由目标工程设置。[板级模板](config/board_template.h) · [板级配置方法](docs/configuration.md#board-settings)

### 电机与标幺值

默认使用 2804 的电机配置。换电机时复制模板，按结构体字段顺序填写电机参数、基值及启动参数，再用 `MFOC_MOTOR_CONFIG` 指定新头文件。应用、HFI 和辨识在同一次构建中使用同一份配置，单位换算和其余 PU 基值在编译期推导。[默认配置](config/motors/motor_2804.h) · [电机配置模板](config/motors/motor_template.h) · [配置选择](docs/configuration.md#motor-profile)

电流使用 `I/Ibase`，母线使用 `Vdc/Vbase`，速度使用电角速度与电角速度基值之比。当前电流基值是相电流峰值；例如 `0.02 pu` 对应 `0.02 × 3.5 A = 0.07 A`。极对数用于机械角与电角的换算。

主工程示例应用的电流 PI、速度 PI、运行限值、采样频率和速度环分频在 `class/mfoc_app.c` 的 Motor 配置中设置；修改电机参数或控制频率后应重新整定并核对这些值。

电机参数辨识支持单测 Rs、连续测 Rs/Ld/Lq、测磁链和测空轴惯量，四种操作共用一套启停和结果接口，均可关闭物理位置反馈。静止实验要求轴保持在对齐位置；磁链和惯量使用电压、电流估算转子运动。[实验操作与适用条件](docs/framework.md#identification) · [双后端软件回放](docs/validation/2026-11-19-p09-identification.md)

需要停机更新时，库与调用方统一定义 `MFOC_MOTOR_MODEL_UPDATE_ENABLE=1`。同一台 Motor 的辨识完成并停机后，用 `mfoc_motor_GetIdentificationResult(&motor, &params)` 读取结果，成功后调用 `mfoc_motor_ApplyModel(&motor, &params)`。应用从文件或其他电机导入参数时，须先按目标 Motor 的 Init 基值换算为 PU。更改基值或极对数需要重新初始化。[接口契约](motor/include/mfoc_motor.h)

## 硬件端口

新板实现 `mfoc_port.h` 声明的操作。电流、母线及速度使用共同基值，电角度使用 BAM32，三相 duty 取值为 0 到 1。G431 示例在板级持有 Encoder，前台发布机械采样，ISR 读取推算的电角度和速度。接入步骤和完整接口清单见 [硬件接口移植指南](docs/port-contract.md)。

## 接入与运行

1. 完成上述库、电机及板级端口配置，将 `mfoc.mk` 的头文件路径和库源码加入工程，并为目标选入 Port 实现和中断入口。
2. 初始化时钟、ADC 触发、PWM 和所需传感器；相输出保持关闭。调用 `mfoc_motor_Init()`，由其初始化控制对象和端口零偏。
3. 前台周期调用 `mfoc_motor_Run()`，处理反馈读取和对齐等前台流程。控制中断每个有效采样周期调用一次 `mfoc_motor_Step()`。
4. 根据需要配置或更新母线值。`mfoc_motor_SetBus()` 接收正的 `Vdc/Vbase`；`mfoc_motor_UpdateBus()` 可读取已发布的母线采样并更新。动态母线异常的处理由应用结合硬件保护策略决定。
5. 根据控制模式设置指令并启动；停止使用 `mfoc_motor_Stop()`，故障清除使用 `mfoc_motor_ClearFault()`。调用条件和返回值见 [`mfoc_motor.h`](motor/include/mfoc_motor.h)。

主工程 G431 的应用和中断接线位于 `class/mfoc_app.c` 和 `target/stm32g431/mfoc_motor_irq.c`。示例控制周期为 20 kHz，移植时应以实际采样和 PWM 时序配置为准。

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

### AT32F413 第二功率板

仓库现有的 `at32f413` 目标接入 AT-MOTOR-EVB V1.1 和 AT32F413RCT7。调试构建可用 `.\make.bat TARGET_CHIP=at32f413 BUILD=debug-rel MFOC_NUMERIC=fixed`。基线采用 Flux，Hall 配置需提供电机实测扇区表。门极和采样时序须在这块板上验证。[AT32F413 接入与验收](docs/validation/2026-11-20-p12-at32f413.md)

## 测试入口

从主工程根目录运行 `.\mfoc\tests\scripts\check_standalone.ps1`。脚本导出源码包、生成 SHA-256 清单，再从包的独立副本构建、链接并执行反馈与辨识配置。开启辨识时，模拟功率级按上一拍实际提交的 duty 产生电流，经 Motor 完成 Rs 辨识并校验结果。[导出和复验命令](docs/release-v1-checklist.md)

主机回归入口 `tests/scripts/run_tests.ps1` 包含各算法、启动交接、PU 换算和独立构建。ARM 入口 `tests/scripts/check_arm.ps1` 负责链接和符号审计。实际采样时序、电流极性、中断耗时和无感接管在台架上验收。
