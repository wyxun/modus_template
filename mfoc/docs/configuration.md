# 配置与头文件

项目选择环路与反馈功能，电机配置定义模型与基值，板级实现按同一控制频率采样并将测量值转换成相应单位。

| 谁设置 | 设置位置 | 内容 |
| --- | --- | --- |
| 项目构建者 | 构建参数和 `mfoc_config.h` | 设置速度环频率，选择位置反馈及库功能。`MFOC_NUMERIC=float/fixed` 选择数值后端，`MFOC_MOTOR_CONFIG` 选择电机配置。所有参与链接的编译单元使用一致配置。 |
| 电机应用作者 | 电机 profile 和 `mfoc_motor_cfg_t` | 示例 profile 提供电流基值 mA、电压基值 mV、电频率基值 Hz，以及电机固有的极对数。应用填写三个基值字段和 `params.pole_pairs`，Motor_Init 在初始化 Port 时转交。 |
| 板级移植者 | 目标构建配置、`mfoc_port.c` 和板级驱动 | 统一设置控制频率、计时器频率和调制上限。配置 ADC 通道、满量程、极性、母线分压、PWM 更新点和传感器参数。 |
| 算法调试者 | 对应模块的默认值和项目编译宏 | 按需覆盖 Flux、HFI 增益、EKF 噪声参数和辨识实验参数。 |

例如，使用 G431 编码器闭环时，让目标中所有 MFOC 源文件看到以下构建选项。

```text
-DMFOC_FEEDBACK_ENABLE_POSITION=1
-DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_POSITION
```

同一个位置提供者也可用于观察。保留 `ENABLE_POSITION=1`，将 backend 设为已启用的 SMO，Encoder 仍会更新，常规闭环取 SMO 的输出。纯无感板可设 `ENABLE_POSITION=0` 并选择相应的已启用无感后端，此时公共 Port 头只声明采样、时钟、初始化和 PWM 接口。旧项目的 `MFOC_FEEDBACK_ENABLE_ENCODER`、`MFOC_FEEDBACK_ENCODER` 是兼容名称。

启动、SMO、Flux 和 EKF 可在同一构建里使用各自 Init 配置中的采样率和基值。启用 HFI 或 Motor 辨识时，两者在编译期使用选定电机配置的电频率基值，采样率取 `MFOC_MOTOR_RUN_FREQ_HZ`。示例应用用同一份电机配置初始化 Motor。

<a id="board-settings"></a>

## 板级配置与控制频率

| 项目 | G431 | AT32F413 |
| --- | --- | --- |
| 控制频率 `MFOC_MOTOR_RUN_FREQ_HZ` | `STM32G431_CONTROL_HZ ?= 20000` | `AT32F413_CONTROL_HZ ?= 20000` |
| perfc 计时频率 `MFOC_PERFC_HZ` | 170 MHz | 200 MHz |
| 调制上限 `MFOC_CORE_M_MAX` | `0.55f` | `0.35f` |

运行 `make.bat` 时可以通过对应目标的 `*_CONTROL_HZ` 变量设置控制频率。例如 `make.bat TARGET_CHIP=stm32g431 STM32G431_CONTROL_HZ=20000`。更改频率时，确认定时器能产生该频率，且 PWM、ADC 触发和采样 ISR 步长一致，再核对 PI 的离散系数。perfc 计时频率应与移植实际使用的 tick 源一致。[G431 目标](../../target/stm32g431/target.mk) · [AT32F413 目标](../../target/at32f413/target.mk)

接入独立工程时，为应用和库统一提供板级控制频率与调制上限。例如，目标可提供 `-DMFOC_MOTOR_RUN_FREQ_HZ=20000U` 和 `-DMFOC_CORE_M_MAX=0.35f`。库内的回退值是 20 kHz 和 `0.55f`。耗时诊断的计时频率、开关和预算由应用及目标设定。[移植模板](../port/template/mfoc_port_template.c)

项目设置的速度环频率须为实际控制频率的整数分频，默认 1 kHz。20 kHz 控制频率下，每 20 拍运行一次速度环。填 `0` 会禁用示例应用的速度模式。控制与速度周期换算、速度环分频和编译期检查由库内头文件提供。[库内配置](../include/mfoc_config_internal.h)

<a id="motor-profile"></a>

## 编译时选择电机配置

从模板复制一份电机参数头到自己的项目，例如 `mfoc/port/projects/my_project/motor_my_motor.h`。按 `mfoc_motor_params_t` 的字段顺序填写极对数、相电阻、Ld、Lq、磁链和惯量，按 `mfoc_motor_cfg_t` 的基值字段顺序填写相电流峰值 mA、母线电压 mV 和电频率 Hz。项目可根据功率板提供适用基值及电流上限。启动接管和模型参数范围由模块提供默认值，可用编译宏覆盖。[电机配置模板](../port/template/motor_template.h) · [无感启动默认值](../motor/include/mfoc_sensorless_start_config.h) · [模型参数范围](../motor/include/mfoc_motor_config_internal.h)

在仓库根目录运行下列命令，路径相对于构建工作目录，也可填写绝对路径。

```powershell
.\make.bat TARGET_CHIP=stm32g431 MFOC_NUMERIC=float MFOC_MOTOR_CONFIG=mfoc/port/projects/my_project/motor_my_motor.h
```

仓库的主 Makefile 为参考构建选择 2804。独立库可以省略 `MFOC_MOTOR_CONFIG`，通过 `mfoc_motor_Init()` 传入实际电机参数。若启用 HFI 或辨识，电频率基值还要与同一构建的编译期设置一致（默认 400 Hz）。自己的 Makefile 可使用 `mfoc.mk` 提供的 `MFOC_MOTOR_CONFIG_FLAGS`。使用其他构建系统时，给库和应用统一定义 C 宏 `MFOC_MOTOR_CONFIG`，宏值为所选头文件的路径字符串。[选择入口](../include/mfoc_motor_config.h)

## 仓库内的板与电机项目

在主工程根目录选择项目，例如：

| 功率板与电机 | Make 参数 |
| --- | --- |
| STM32G431 + 2804 | `TARGET_CHIP=stm32g431 MFOC_PROJECT=g431_2804` |
| AT-MOTOR-EVB V1.1 + BLDC5562D048，24 V Hall | `TARGET_CHIP=at32f413 MFOC_PROJECT=f413_5562` |

`port/projects/` 选择功率板、电机、反馈方式和项目运行配置。
项目入口选择本目录的电机参数头。Hall 扇区表随项目中的电机头保存，
GPIO 和计时器通道由项目内的 `board/` 绑定。MODUS 共用的 ADC
控制 IRQ 位于 `port/modus/`。独立源码包提供
Port 契约和模板。随包的主机示例显式选用电机参数模板。
[AT32 项目说明](validation/2026-11-20-p12-at32f413.md#bldc5562d048-的-24-v-hall-构建)

G431 参考项目通过 `MFOC_G431_FEEDBACK` 选择 Flux（默认）、SMO、HFI
或 FLOAT EKF。项目可用 C 宏 `MFOC_PROJECT_CONFIG` 指定自己的整定头，
宏值为头文件路径字符串。库和应用在读取模块回退值前加载这份配置。
[构建示例](../port/README.md#构建现有项目)

<a id="advanced-settings"></a>

## 模块高级参数

Flux 的观测器和 PLL 增益、EKF 的噪声和协方差、辨识的激励、限值和采样数由对应模块提供默认值。项目可在构建中覆盖已有定义，例如在 Make 目标中追加 `C_DEFS += -DMFOC_FLUX_PLL_KP=0.45f`，让库和应用看到同一值。根据实际电机与采样率验证调参结果。[Flux 默认值](../position/include/mfoc_flux.h) · [EKF 默认值](../position/include/mfoc_ekf.h) · [辨识默认值](../identification/include/mfoc_identification_algorithms.h)

HFI 注入电压可覆盖 `MFOC_HFI_VOLTAGE_PU`。阈值和 PLL 增益使用后端标量，例如在项目配置中定义 `MFOC_HFI_PLL_KP` 为 `MFOC_SCALAR(0.05f)`。载波由 HFI 模块固定为八拍。启用极性辨识时，用统一的构建选项 `MFOC_HFI_ENABLE_POLARITY=1`。[HFI 默认值](../position/include/mfoc_hfi.h)

## 链接配置检查

链接检查覆盖 FLOAT/FIXED、反馈主后端及各项使能、辨识、模型更新、HFI 极性、PWM 预装载延迟、perfc FSM 和三角函数后端。目标和应用依然要统一采样率、标幺基值、电机参数、增益和 C ABI 编译选项。

GNU/Clang ELF 构建每个匹配的配置合并为一个指针宽度的只读检查项（ARM Cortex-M 为 4 字节），保留原有 API 符号名称。链接应包含 `mfoc_numeric.c`。链接器须支持 GNU retain 和 COMDAT，检查在静态库与段裁剪下生效。Windows MinGW 用带配置签名的编译后符号绑定库调用与 Port，C 源码中的函数名不变。仅内联访问结构字段的 MinGW 编译单元依赖相同的构建配置。两种方式均不增加执行指令。[检查实现](../include/mfoc_abi.h)

## 使用头文件

| 头文件 | 使用方式 |
| --- | --- |
| [mfoc_config.h](../mfoc_config.h) | 配置项目常用的速度环、反馈和功能开关。项目也可用构建宏覆盖这些默认值。 |
| [mfoc.h](../include/mfoc.h) | 基础数值和控制公共入口。需要电机参数时，显式包含电机配置选择头。 |
| [mfoc_motor_config.h](../include/mfoc_motor_config.h) | 电机配置选择入口。应用和库从这里读取同一份配置。 |
| [mfoc_port.h](../port/include/mfoc_port.h) | 板级实现与库共用的函数签名、上下文、单位和失败语义。新板实现这里声明的操作。 |
| [mfoc_encoder.h](../position/include/mfoc_encoder.h) | 使用库 Encoder 的位置提供者包含此头，创建实例，通过 Init、Update、Step、CalibrateZero 操作。 |
| [mfoc_hall.h](../position/include/mfoc_hall.h) | 三路数字霍尔板填写六扇区电角度表和超时，创建实例，通过 Init、OnEdge、Step 提供物理位置。 |

G431 的 ADC 增益、分压和采样间隔随硬件或传感器调整。板级换算所用的 Q30 增益精度、已测零偏及发布槽等是实现细节。相同电机换板时，通常重配前一组板级量，继续沿用原有控制单位。实际接线见 [硬件接口移植指南](port-contract.md)。
