# Port 相关配置和头文件

移植一块板时，应用定义控制系统采用的单位，构建配置选择位置源，板级实现负责把测量值转换到这些单位。下面列出这条数据流用到的设置。

| 谁设置 | 设置位置 | 内容 |
| --- | --- | --- |
| 项目构建者 | 构建参数、`mfoc_config.h` 中的用户选项 | `MFOC_NUMERIC=float/fixed` 是本工程的 make 参数。位置功能用 `MFOC_FEEDBACK_ENABLE_POSITION`，控制反馈来源用 `MFOC_FEEDBACK_BACKEND`。所有参与链接的编译单元使用一致配置。 |
| 电机应用作者 | 电机 profile 和 `mfoc_motor_cfg_t` | 示例 profile 提供电流基值 mA、电压基值 mV、电频率基值 Hz，以及电机固有的极对数。应用填写三个基值字段和 `params.pole_pairs`，Motor_Init 在初始化 Port 时转交。 |
| 板级移植者 | 新板的 `mfoc_port.c` 和板级驱动 | 配置 ADC 通道、满量程、极性、母线分压、PWM 周期与更新点、tick 来源。使用 Encoder 时在这里选择采样周期、滤波、方向、零位和超时。 |
| 库维护者 | 选择常量、派生量和数值实现 | `MFOC_FEEDBACK_POSITION`、`MFOC_FEEDBACK_SMO` 等是固定编号的公开选择值。`MFOC_SPEED_LOOP_DIVIDER`、`MFOC_MOTOR_RUN_PERIOD_US` 由用户填写的频率推导。`MFOC_Q_FRACTION_BITS` 和 `MFOC_Q_SCALE` 描述 FIXED 数值格式。应用按定义使用这些值。 |

例如，使用 G431 编码器闭环时，让目标中所有 MFOC 源文件看到以下构建选项。

```text
-DMFOC_FEEDBACK_ENABLE_POSITION=1
-DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_POSITION
```

同一个位置提供者也可用于观察。保留 `ENABLE_POSITION=1`，将 backend 设为已启用的 SMO，Encoder 仍会更新，常规闭环取 SMO 的输出。纯无感板可设 `ENABLE_POSITION=0` 并选择相应的已启用无感后端，此时公共 Port 头只声明采样、时钟、初始化和 PWM 接口。旧项目的 `MFOC_FEEDBACK_ENABLE_ENCODER`、`MFOC_FEEDBACK_ENCODER` 是兼容名称。

| 头文件 | 使用方式 |
| --- | --- |
| [mfoc_config.h](../mfoc_config.h) | 查功能开关及默认值。移植项目可用构建宏覆盖提供默认值的用户选项。 |
| [mfoc_motor_config.h](../mfoc_motor_config.h) | 本工程的电机 profile。换电机时更新实测或规格值与选定基值，其余 PU 换算由公式导出。 |
| [mfoc_port.h](../port/include/mfoc_port.h) | 板级实现与库共用的函数签名、上下文、单位和失败语义。新板实现这里声明的操作。 |
| [mfoc_encoder.h](../position/include/mfoc_encoder.h) | 使用库 Encoder 的位置提供者包含此头，创建实例，通过 Init、Update、Step、CalibrateZero 操作。 |
| [mfoc_hall.h](../position/include/mfoc_hall.h) | 三路数字霍尔板填写六扇区电角度表和超时，创建实例，通过 Init、OnEdge、Step 提供物理位置。 |

G431 的 ADC 增益、分压和采样间隔随硬件或传感器调整。板级换算所用的 Q30 增益精度、已测零偏及发布槽等是实现细节。相同电机换板时，通常重配前一组板级量，继续沿用原有控制单位。实际接线见 [硬件接口移植指南](port-contract.md)。
