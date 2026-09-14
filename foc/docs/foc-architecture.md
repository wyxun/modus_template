# FOC 当前架构

本文说明当前 FOC 源码的模块职责、控制流程和边界，不绑定具体 MCU、ADC 通道、PWM 引脚或位置传感器。具体目标的硬件映射和台架调试方法见[使用与电机调试指南](../docs/foc-test-guide.md)中的板级实例。

台架性能数字应以与源码版本、构建选项和目标硬件相对应的独立测试记录为准。

## 1. 范围与分层

当前实现由一个产品组合对象管理一个 Motor 和一个位置观测对象。职责按实际调用关系划分：

~~~text
产品入口 / 高频采样中断 / 产品命令
                 │
                 ▼
foc_app_t ─── 调度、命令适配、持有 Motor、Encoder 与 Observer
   ├── motor_t       生命周期、参考值、电气量换算与控制编排
   └── foc_encoder_t 机械位置样本、速度估算与当前角度外推
                 │
                 ├── foc_core：Clarke / Park / 电流 PI / 反 Park / SVPWM
                 ├── foc_pid：速度 PI
                 └── foc_port.h：直接 ADC/PWM 函数边界
                                      │
                                      ▼
                              目标硬件适配层
~~~

`foc_port.h` 是直接函数契约，不是 ops 表或运行时插件注册表。Motor 通过该边界访问 ADC 与 PWM，不包含目标 HAL、MDI 或具体传感器类型。App 持有 Encoder，并将统一的位置读取函数和上下文绑定给 Motor；
Motor 不拥有传感器驱动对象。

## 2. 模块职责

| 模块 | 当前职责 |
| --- | --- |
| `foc_app_t` | MODUS 生命周期入口、前台调度、产品命令、波形和高频统计；持有 Motor、Encoder 与 Observer |
| `motor_t` | 单电机状态、参数与控制配置、命令参考、ADC 校准状态、Core/PID 状态、电气零位和高频控制步骤 |
| `foc_encoder_t` | 调用已绑定的位置源、缓存机械位置样本、滤波机械速度并提供带时间戳的位置读数 |
| `foc_core` | 数值后端无关的 Clarke/Park 变换、电流 PI、反变换和 SVPWM 编排 |
| `foc_pid` | 电流环和速度环使用的 PI 算法实现 |
| `foc_port` | ADC 零偏校准与三相电流采样；三相 PWM 占空比提交、使能和停止 |
| 目标硬件适配 | 将直接端口契约映射到具体板级外设，并提供所选位置传感器的原始读数 |

Motor API 位于 `foc/motor/motor.h`，负责启动、停止、清故障、设置电压/电流/速度参考、请求位置对齐、高频步进和读取 Motor 状态。产品命令层只负责把用户输入转换为这些 API 调用；算法模块不依赖 Shell。

## 3. Motor 生命周期

~~~text
INITIALIZING → ADC_CAL → IDLE ── Start ──→ RUNNING
                    │      │                  │
                    │      └─ Align request → ALIGN
                    │                         │
                    └──────────────┐          ├─ 捕获电气零位 → IDLE
                                   ▼          │
                                  FAULT ←─────┘
~~~

- 初始化时关闭 PWM，并由高频步进推进 ADC 零偏校准；校准成功后进入 `IDLE`。
- `IDLE` 才接受运行和 ALIGN 请求。当前支持 VOLTAGE、CURRENT、SPEED；POSITION 尚未实现，启动时返回 disabled。
- 当前 `motor_Start()` 对所有运行模式都要求位置读取函数可用。
- ALIGN 以固定电角度和 D 轴电流运行，完成后从机械位置捕获电气零位、停止 PWM 并回到 `IDLE`。
- 运行中 ADC、位置、数学运算或 PWM 失败时进入 `FAULT` 并停止 PWM。
- 清除普通故障后回到 `IDLE`；ADC 校准故障清除后重新进入校准。
- STOP 先关闭 PWM；停止操作不会把未完成的 ADC 校准伪装成 `IDLE`。

## 4. 调度与控制数据流

### 高频路径

目标平台在每个电流采样周期完成后调用 `foc_app_HighFrequencyISR()`。当前产品按 20 kHz 运行；该频率与触发机制属于目标配置，FOC 数学层不直接配置定时器。

~~~text
目标采样完成中断
  → `foc_app_HighFrequencyISR()`
  → `motor_HighFrequencyStep(motor, now_tick)`
      ├─ ADC_CAL：累计偏置样本
      ├─ ALIGN：采样三相电流 → 固定角度 Core → PWM
      └─ RUNNING：采样三相电流 + 读取位置缓存
                   → 机械角 × 极对数 − 电气零位
                   → 速度 PI（按配置分频）
                   → `foc_core_step()` → 三相 PWM 提交
~~~

高频路径只消费位置观测对象已发布的缓存，不执行慢速传感器事务。Motor 使用机械角与自身极对数换算电角度，使用机械速度与极对数换算电速度。速度 PI 的输出更新 Q 轴电流参考；当前产品的分频值为 20，高频路径为 20 kHz 时对应 1 kHz 速度环。

### 前台位置更新

`foc_app_Run()` 在前台以约 1 ms 周期调用 `foc_encoder_Update()`。
读取失败后当前实现等待 100 ms 再重试；传感器 I/O 不在 MODUS Clock
中断或高频控制中断内执行。Encoder 发布机械角、机械速度和采样 tick。
高频读取时检查样本年龄，并使用滤波速度将角度外推至当前 tick；
超时样本会被标记为无效。

### 三相语义契约

框架的数据类型和调用顺序使用 U/V/W 三相值。目标适配必须保证 ADC
返回的每相电流与 PWM 提交的对应相一致，且电流正方向定义一致。
ADC 单元/通道、定时器、引脚、采样拓扑和驱动器连接属于板级实现，
不属于架构层；本契约不规定任何目标的物理映射。

## 5. 配置、模式与数值后端

- `foc_app_cfg_t` 组合 Motor 控制配置与 Encoder 配置；当前产品配置在 `foc/app/foc_app.c` 的 MODUS 对象声明中。
- `motor_params_t` 保存极对数、定子电阻和 D/Q 轴电感。极对数用于机械量到电气量的换算；电阻和电感当前要求非零并保留，不参与当前 Core 的控制参数计算。
- `motor_params_t` 另保存观测器使用的电压、电流 pu 基准。当前示例为 12 V / 7 A；7 A 由 `0.1 pu ≈ 0.7 A` 推估，属于待台架确认的初值，不是电流采样标定结果。
- App 持有单实例 `foc_observer_t`，Motor 借用它并在 Encoder 控制时每拍运行 SMO Shadow。Observer 使用本拍 `Iαβ` 和 Core 上一采样区间的 `Vmodel`；当前产品没有配置质量门限，因此输出保持 `valid=false`，不会切换 FOC 反馈源。
- SMO 使用标幺化模型：电压、电流分别除以 Motor 的基准；时间基准取固定 `Ts`，因此模型的电阻、电感和 PLL 系数在 Init 时换算。SMO 不把物理大增益作为 Q15 普通 pu 乘数使用。
- 电流 PI、速度 PI、ADC 校准超时、ALIGN 步数、电流参考和速度环分频由 Motor 控制配置提供。`motor_limits_t` 当前只声明，运行路径不读取。
- `FOC_NUMERIC_FLOAT` 与 `FOC_NUMERIC_FIXED` 编译期二选一；Core、Motor 和 Encoder 共用相同的控制逻辑。
- `foc/foc.mk` 编译数值/角度数学、Core、PID、调制、Encoder、SMO Observer、Motor 和 App。NLFO、HFI 等其它算法仍不进入当前构建。

## 6. 命令与观测边界

产品命令目前支持 `speed`、`current`、`voltage`、`align`、`stop`、
`clear`、`status` 和 `encoder`。设置参考只更新当前已选模式对应的参考，
不会隐式切换模式。Motor 状态 API 提供生命周期、故障、模式和 PWM
使能状态；传感器机械位置由 Encoder 读取接口提供。

float 且启用 `MWAVEFORM_ENABLE` 时，App 注册 `Sine500`、`WaveSeq`、
`Speed`、`SpeedRef`、`Iq` 和 `IqRef` 波形通道。实际量按 10 kHz 采样，
参考值按 1 kHz 采样；fixed 或关闭波形的构建不执行该采样路径。
高频统计报告平均耗时，不代表最大值或 P99。

## 7. 当前实现边界

- 当前运行角度源是位置传感器；无感 shadow、无感接管和融合控制均未接入。
- 当前不实现位置闭环、多电机管理或运行时算法切换。
- 本文只维护模块职责和架构关系；具体板卡配置与电机调试由独立使用指南维护。
- 若增加控制模式或改变对象边界，应先更新模块图/状态机并审核，再修改实现；不能把未来设计提案误当成当前架构。
