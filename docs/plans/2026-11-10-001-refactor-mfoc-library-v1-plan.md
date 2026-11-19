---
title: MFOC 第一版独立库实施与发布计划
type: refactor
status: active
date: 2026-11-10
---

# MFOC 第一版独立库实施与发布计划

本计划的终点是交付可以脱离当前工程、移植到 Cortex-M、选择 FLOAT 或 FIXED 后端、内部使用统一标幺体系的第一版 MFOC 库。后续 FOC 功能和内部算法优化以这套骨架为基础。

**发布条件：P00—P13 全部完成，发布门禁 G1—G8 全部通过，形成可复现的发布包与验收记录。** 不能用“某块板已能转动”“文件可以编译”或“算法分别验证过”代替库级验收。本文完成只表示计划已落地，所有实施项初始均为未开始。

目标仓库为 `modus_template`；下文代码路径均相对于仓库根目录。依据为本次需求、当前工作区源码及 `mfoc/docs/framework.md`。现状文档描述已实现行为；本文描述计划目标，两者在实施完成前不能混用。保留现有工作区改动，实施中按对应工作包更新现状文档。

## 1. 第一版交付范围

| 编号 | 需求 | 发布时的判定 |
| --- | --- | --- |
| R1 | 可以独立交付 | 发布包离开本仓库仍能构建、链接和运行主机示例；核心不依赖 MODUS、perfc、MDI、厂商 HAL、RTOS 或应用对象 |
| R2 | 硬软隔离与实例隔离 | 可移植控制层只消费明确的数据与配置；硬件副作用全部位于适配层；两套实例交错执行互不污染 |
| R3 | 数据流有明确周期关系 | 固定采样/PWM 时序下，电流、前区间电压与控制角对应；时序由适配保证，不向高频入口逐拍附加通用时间戳、序号或有效性包 |
| R4 | FLOAT/FIXED 与统一 PU | 对外单位、范围、限幅、舍入和精度要求可查；局部高精度状态不泄漏为跨模块格式 |
| R5 | 保留已验证能力 | Encoder、SMO、Flux、Core、Motor、开环与无感启动经过迁移回归；HFI 完成接口及实时预算验证 |
| R6 | 能继续扩展功能 | 电流参考生成、电压合成、反馈选择、激励授权和参数应用各有唯一职责位置 |
| R7 | 四组辨识完整可用 | Rs、Ld/Lq、磁链、惯量均有公开启动入口、完成/失败/取消路径、结果读取和验证证据 |
| R8 | 发布结论有证据 | 支持矩阵、独立构建、数值回归、实机时序、第二芯片移植、接入文档和版本包一致 |

首版功能范围：电流/速度控制、已实现的反馈与启动路径、HFI、四组离线辨识、停机后的模型应用。EKF 保持 FLOAT 可选能力；若随首版对外公开，必须进入对应测试和资源矩阵，不能以“可编译”代替验证。FLOAT/FIXED 是编译期选择，不要求同一二进制同时运行两套数值 ABI。

磁链和惯量辨识首版从实测电流和前区间 PWM 模型电压估计转子运动，四种操作均支持无编码器构建。虚拟旋转角只用于激励。惯量测量条件为轴上无外加载荷，结果是该条件下的轴侧等效惯量。真实电机精度与重复性留待台架测量。

后续版本工作：MTPA、弱磁、过调制、在线辨识、未知任意负载下的惯量辨识、HFI 与反电势观测器全速域双向融合、FIXED EKF、单电阻采样重构、更多芯片认证。首版确定这些功能的接入位置，但不提前实现通用插件系统或所有算法。

核心支持独立多实例；首版不要求同一 MCU 的多电机同步硬件系统。基础采样范围为板级能够提供完整、有效的三相电流帧；两/单电阻的重构策略在声明支持并通过验收前不得冒充已有能力。

## 2. 当前基础与需要解决的问题

| 方面 | 可以保留的基础 | 本计划补齐的内容 |
| --- | --- | --- |
| 主框架 | Motor 编排，Core 电流环和调制，Feedback 管算法组合，叶算法拥有状态 | 保留直接调用，统一模块输入输出，硬件细节收在 Port 实现内 |
| 电压历史 | 成功提交 PWM 后才发布前拍输出 | 明确前拍模型电压与实际采样区间的固定关系；反馈与辨识共用一种契约 |
| 数值 | int32 载体的 15 位小数、BAM32、局部宽乘积、初始化预计算 | 运算范围语义、小信号积分精度、后端误差和无 FPU 覆盖 |
| Port | 直接调用、无热路径函数指针、目标负责采样和 PWM | 校准/PU 换算保留在物理边界；器件驱动、位置处理和板级准备分清，公共接口不绑定 AS5600/编码器名称 |
| HFI | 算法请求注入，Core 合成并限幅 | 运行/注入/控制角选择分开；测量八拍周期峰值及完整 ISR 预算 |
| 辨识 | Rs 流程、L 捕获/前台求值、独立磁链估计叶 | L 调度、磁链接线、惯量实现、模型事务应用、集成测试同步 |

具体依据：

- `mfoc/motor/src/mfoc_motor.c` 的 Step 同时采样、计算、提交 PWM；直接引用 perfc 进行前台计时。
- `peripheral/stm32g431/mfoc/mfoc_port.c` 同时处理 AS5600 配置、约 100 ms 校零、ADC 转 PU、母线标定、PWM 和时间；含板级单例状态。
- `mfoc/motor/include/mfoc_feedback.h` 消费完整 Core 输出；辨识主入口消费前拍请求电压；独立磁链叶又依赖 Core 输出。
- `mfoc/math/include/mfoc_numeric.h` 的 `mfoc_mul_pu` 在 FIXED 下限制输入到 ±1，FLOAT 下直接相乘；PU 并不意味着数值必须在 ±1 内。
- `mfoc/control/src/mfoc_pi.c` 的 FIXED 每拍积分增量先量化再累加，小误差可能始终无法积累；HFI 已有局部宽积分实践。
- `mfoc/identification/src/mfoc_identification.c` 的公开启动当前只接受 Rs；Ld/Lq、磁链、旋转准备仍有未接通阶段；惯量叶为占位实现。
- `mfoc/motor/src/mfoc_feedback.c` 已有“全部观测器接受参数后再提交”的模式，但未形成 Motor 层统一模型应用操作。

已有检查作为基线记录，不代表发布验收：Core、Motor 初始化、控制节拍、启停故障路径的主机 FLOAT/FIXED 回归在 HFI 关闭配置下通过；Motor 辨识测试默认配置在 `feedback_reads == 1U` 处失败，关闭 HFI 后还存在引用条件字段 `acquiring` 的编译失配。用户报告 HFI 开启后整段 step 超过 50 μs；该数值尚未在本计划中按各段和计时范围复测。

## 3. 目标架构与 Port 改造

### 3.1 选定方向

**Port 约定物理世界向 mfoc 提供的量，以及 mfoc 向硬件输出的量和动作。** 输入在边界完成标幺化，库内按统一单位计算，输出到 duty 后由板级换成 PWM 比较值。Motor 保留编排，Core 保留电流控制与调制，已经验证的算法边界不因 Port 整理而重建。

保留有类型、静态绑定的直接调用。Motor 可以调用窄 Port 接口；库的独立性要求不依赖 HAL/MDI/板级对象，主机可以提供 fake/replay Port，并不要求所有 Port 符号都从 Motor 消失。Motor_Init 继续主动调用 Port 初始化，硬件实现留在集成侧；应用无需另行调用 Port 初始化。首版不引入运行时 ops 表、通用采样消息或提交回执框架。第 3.6 节记录已实施的 Port 迁移对照；P01 的完整配置、生命周期与支持矩阵仍按工作包验收。

### 3.2 物理输入量清单

下表按物理能力整理；“可选”不表示首版必须新增该接口，更不表示每拍都要读取。由实际启用的功能决定是否接入。

| 输入量 | 控制侧约定 | 使用范围与提供者责任 |
| --- | --- | --- |
| 三相电流 `i_u/i_v/i_w` | `I/Ibase`，相序与正方向固定 | 基本电流闭环必需；板级负责 ADC 获取、零偏、增益、极性和必要的相电流重构，提供完整三相量；库内做 Clarke/Park |
| 母线电压 `v_bus` | `Vdc/Vbase` | 电压限幅、调制及模型电压所需；有采样时由板级更新，没有采样时可使用明确配置的固定值并限定适用条件，不伪称实测 |
| 实测转子角度、速度 `theta_e/omega_e` | 电角度 BAM32；电角速度按统一电角速度基值标幺化 | 有感路径按需提供；控制入口不区分编码器或霍尔，由位置源完成零位、方向、极对数、测速及必要的插值/滤波 |
| 实测定子电压 | 去共模后的三相定子电压 `V/Vbase`，线电压测量由适配转换 | 可选，用于确实需要实测电压的观测或辨识；与 duty、母线推算的模型电压分清，不要求无电压传感器的板提供 |
| 母线电流 `i_bus` | `I/Ibase`，正方向约定为电源向功率级供电 | 可选，供功率、电源侧限流等功能；不能直接代替三相电流或 `i_q` |
| 绕组、功率级温度 | 监测接口明确单位，如 °C | 可选，归低频监测/应用；需要温度补偿时才进入相应模块，不扩充高频 FOC 必需输入 |

电流和电压换算系数在准备阶段预计算，热路径只做必要换算。原始 ADC 码、传感器寄存器、计数器值不进入 Motor/Core。PU 不等于强制限制在 ±1，采样不按运行限额静默裁剪。温度等监测量不为形式统一而强行套用电流/电压基值。

**角度、速度的统一接口与硬件 Port 是两条相接的边界。** 有感位置源、SMO/Flux/HFI、强拖对象都向控制侧提供相同口径的 `theta_e/omega_e`；只有有感来源需要硬件测量。无感估计和强拖生成仍归库内对象，不放进板级 Port。统一输出不要求内部实现相同，也不要求增加函数指针。强拖的角速是命令轨迹，不能因字段相同就当成旋转辨识的实际测量。

已确定：位置算法仍归库，有感位置组件实例由集成侧持有，通过 Port 提供统一电角度/电角速度，再经 Feedback 到 Motor。现有 Encoder 的测速、滤波、角度转换等计算继续复用；器件配置/读取留在驱动。原始机械角、霍尔边沿及组件内部采样时间不提升为所有 Motor 每拍必传的字段。公共位置接口不接收父对象或具体传感器对象，不引入通用位置管理器。新增霍尔组件属于后续功能，不在本轮借接口整理扩大实现范围。

### 3.3 物理输出与辅助操作清单

| 输出或操作 | Port 边界 | 调用阶段 |
| --- | --- | --- |
| 三相占空比 `d_u/d_v/d_w` | 库输出 `[0,1]` 的 duty；板级换成定时器比较值，处理计数方式、极性和更新点 | 高频；`mfoc_port_pwm(duty)` 只写本拍输出，取消 `start` 参数 |
| 输出使能 | `mfoc_port_pwm_start(initial_duty)` 内部完成初值装载、生效和输出使能 | 启动；Motor 不展开硬件更新顺序，正常 PWM 更新没有启停含义 |
| 输出关闭 | 关闭功率输出，按板级约定保留电流采样触发 | 停止/故障；关闭与清故障分开 |
| 硬件准备、电流校零 | Motor_Init 调用 Port 初始化；板级实现准备、校准及位置提供者初始化，库算法不引用硬件实现 | 运行前；外部先准备必要时钟/触发等基础条件，无需增加独立的应用 Port 初始化步骤 |
| 有感位置前台更新 | Motor_Run 调用 `mfoc_port_position_run()`；提供者按自身周期采集，位置组件保存测速/滤波结果 | 停机和运行期间按需服务；不在高频入口等待总线 |
| 有感位置零位标定 | 根据已保持的已知电角度调用所绑定位置组件的标定操作 | 对齐前台流程；Motor 管激励和保持，位置组件管零位 |
| 物理故障读取/清除 | 硬件或驱动拥有物理故障；Motor 拥有软件运行状态。沿用明确的故障通知/停机入口，清物理故障不自动启动 | 按板能力及应用需求；不逐拍打包故障快照 |
| 单调时基及频率 | 基础 Port 契约：64 位无符号 tick，32 位 Hz；隐藏 perfc 等平台实现 | 实际需要测速、插值/外推、超时的对象使用；固定控制步长仍为配置，不给每次 Core 计算附加时间戳 |

采样率、PWM 频率、PU 基值、极对数和零位是配置；ADC 触发位置、死区、引脚与通道是硬件配置。它们不作为每拍 Port 数据重复传递。日志、波形和性能计数归应用诊断，不成为必需 Port。

### 3.4 从采样到 PWM 的职责与数据流

```mermaid
flowchart LR
    ADC[ADC 原始值] --> PORTIN[板级 Port：校准与 PU 换算]
    PORTIN -->|三相电流、母线| MOTOR[Motor：编排与来源选择]
    SENSOR[驱动 + 集成侧持有的库位置组件] --> POSITION[Port 统一角速入口]
    POSITION --> FB[库内 Feedback]
    MOTOR --> FB
    FB -->|统一角度、速度及所需注入| MOTOR
    START[库内强拖与启动策略] -->|控制角与参考| MOTOR
    MOTOR --> CORE[Core：坐标变换、电流环、限幅、调制]
    CORE -->|三相 duty| PORTOUT[板级 Port：比较值换算与写入]
    PORTOUT --> PWM[PWM 硬件]
```

1. 板级配置好 ADC 触发与 PWM 更新关系；中断到来时，Port 读取已完成的采样，完成相序、零偏、比例换算，提供 PU 电流和所需母线值。
2. Motor 按已选控制模式编排。Feedback 用本拍电流和前区间模型电压估计角速；有感分支调用 Port 获取位置组件的同口径角速；强拖对象生成命令角速。Feedback 输出保持扁平的 `angle`、`speed` 字段，Motor 不新增一套 SetAngle/SetSpeedFeedback 状态。来源选择、启动交接仍在库内。辨识按实验需要消费电流、电压及实际角速，不访问器件。
3. Core 用电流、选定电角度、dq 参考和必要注入完成电流控制、合成限幅及调制，产出 duty；速度由 Motor 的速度环等消费者使用，不给不需要它的 Core 增加输入。
4. Port 把 duty 转为硬件比较值并写入。成功后按固定时序契约保留下一拍需要的输出历史；原有明确的读写失败处理继续保留，不增加通用回执或整机状态回滚。

相比当前边界，主要改善是：公共位置接口不再绑定编码器器件；硬件实现与库算法分离，Motor_Init 仍统一组织初始化；PU 基值与板级增益归属明确；库不直接引用 perfc/MDI/HAL；各对象只取得自己需要的量。直接 Port 调用、现有 Motor 编排及已验证算法可以保留。

### 3.5 固定时序与状态归属

高频路径依赖接口前提，不在每层重复做有限值、范围、新鲜度检查。采样完成及相间一致性由 ADC 触发与 Port 实现保证；控制限幅、观测器自身收敛判断及必要的故障退出仍由原对象负责。合理数值本身不能证明采样没有停更，但这不要求向 Motor 添加通用样本有效性包。

采样点、PWM 更新点与模型电压采用固定、可测的周期约定。反馈、HFI、辨识使用相同的前区间电压含义；duty 和母线推算的是模型电压，不能冒充端电压实测。适配应满足该约定；若某硬件时序带来额外延迟或区间内多次更新，应在适配配置与验证中解决，不能静默当成“上一拍”。不要求通用 pending/active 管理器、逐拍序号、时间戳或提交确认协议。

零偏/增益和硬件资源归适配绑定，位置处理状态归位置组件，观测与控制状态归各库对象，模型和运行阶段归 Motor。两套软件实例不能共享隐藏可变算法状态；首版单功率级的静态 Port 绑定不能被误称为已支持同芯片多电机硬件。

命令与 ISR 的并发、前台位置更新与 ISR 读取的一致性由集成侧和对应组件解决，不将未处理的总线事务带进高频中断。启停和硬件 Break 行为继续明确测试；故障处理不要求逐拍向所有算法传递故障结构。平台看门狗等系统机制归板级系统设计，不在本节新增为库的统一接口或强制工作包。

### 3.6 Port 接口迁移与原头文件对照

2026-11-10：本节直接函数接口已落实到 `mfoc_port.h`、Motor/Feedback、纯 Encoder 组件与 G431 适配；不增加运行时分派。具体实现与调用前提见 [Port 契约](../../mfoc/docs/port-contract.md)，基线及回归证据见 [迁移验证](../../mfoc/docs/validation/2026-11-10-port-migration.md)。这是 P01/P03/P04 中的 Port 迁移子集，不代表整项工作包或发布门禁完成。

| 当前接口 | 目标处理 | 责任与调用方 |
| --- | --- | --- |
| `mfoc_port_current(current)` | 保留 | Motor 高频采样；三相 PU |
| `mfoc_port_bus(bus)` | 保留 | Motor 按约定频率更新母线 PU |
| `mfoc_port_pwm(duty, start)` | 取消 `start`，另增 `mfoc_port_pwm_start(initial_duty)` | Step 更新，Start 启动；板级实现初值生效及使能顺序 |
| `mfoc_port_pwm_stop()` | 保留 | Stop/Trip 关闭输出 |
| `mfoc_port_init()` | 保留由 Motor_Init 调用；接收 `mfoc_port_cfg_t` | 统一提供电流/电压/电角速度基值与极对数；参数不含 Motor 或 HAL 对象 |
| `mfoc_port_encoder(angle)` | 公共控制入口改为 `mfoc_port_position(angle, speed)` | Feedback 获取电角度和电角速度；机械角读取留在具体位置源接线中 |
| `Feedback_Run → Encoder_Run` | Motor_Run 直接调用新增的 `mfoc_port_position_run()` | 集成侧更新其持有的位置实例；Feedback 不保留纯转发前台入口 |
| 无通用有感零位入口 | 增加 `mfoc_port_position_calibrate_zero(held_angle)` | 保持已知电角度期间，由 Motor 对齐流程直接调用 |
| `mfoc_port_tick()` 返回 `uint32_t` | 改为公共 `mfoc_tick_t`，定义为 `uint64_t` | 位置插值/外推及库内超时；不直接依赖 perfc |
| `mfoc_port_tick_frequency()` | 保留 `uint32_t` Hz | 初始化读取，频率与 tick 同源且运行期间稳定 |

```c
typedef uint64_t mfoc_tick_t;

mfoc_result_t mfoc_port_current(mfoc_uvw_t *current);
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus);

mfoc_result_t mfoc_port_position(mfoc_angle_t *angle,
                                mfoc_scalar_t *speed);
mfoc_result_t mfoc_port_position_run(void);
mfoc_result_t mfoc_port_position_calibrate_zero(mfoc_angle_t held_angle);

mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty);
mfoc_result_t mfoc_port_pwm_start(const mfoc_uvw_t *initial_duty);
mfoc_result_t mfoc_port_pwm_stop(void);

mfoc_tick_t mfoc_port_tick(void);
uint32_t mfoc_port_tick_frequency(void);
```

`mfoc_port_position` 一次调用取得同一位置处理结果的两个量，不拆成独立读角度/读速度操作，不在 ISR 发起等待式传感器事务。返回值沿用 `mfoc_result_t`，不增加逐拍有效性数据包。公开位置入口不携带 HFI 注入或获取状态。

Feedback 保持当前扁平输出，不嵌入 `angle_speed` 成员：

```c
typedef struct {
    mfoc_angle_t angle;
    mfoc_scalar_t speed;
    mfoc_ab_t u_inject_ab;
#if MFOC_FEEDBACK_ENABLE_HFI
    bool acquiring;
#endif
} mfoc_feedback_output_t;
```

有感分支可直接调用 `mfoc_port_position(&next.angle, &next.speed)`；Motor 继续使用 `feedback.angle`、`feedback.speed`。统一角速的单位、方向和含义即可，暂不新增 `mfoc_angle_speed_t`。嵌套结构不增加函数调用深度，但会增加成员访问层级；这里不为复用两个字段引入嵌套、额外转换函数或布局强制转换。

位置算法归库、位置实例归集成侧，不等于将测速/滤波算法复制进各板 Port。具体驱动与库组件在集成侧组合；高频 Port 提供计算好的角速或执行已有的短时外推。前台采集、零位标定和初始化都通过对象操作完成，不由 Motor 读写位置对象成员。保留当前对齐结束前捕获零位、随后停止的行为；不支持该标定的来源不启用自动零位流程。

初始化保持 `应用 → Motor_Init → port_init`。已落实的统一初始化配置为：

```c
typedef struct {
    uint32_t current_base_milliamp;
    uint32_t voltage_base_millivolt;
    mfoc_scalar_t electrical_speed_base_hz;
    uint16_t pole_pairs;
} mfoc_port_cfg_t;

mfoc_result_t mfoc_port_init(const mfoc_port_cfg_t *cfg);
```

四个字段分别服务于电流换算、电压换算、速度归一化和机械角到电角转换。`electrical_speed_base_hz` 在初始化边界表示物理电频率 Hz，不是速度 PU；其类型沿用当前位置组件配置，不进入逐拍输入。Motor 从统一配置生成该初始化参数，极对数来自同一电机模型，Port 不另配一套基值。无需把整个 Motor 配置或运行对象传入 Port。

ADC 通道、分压、采样增益与 PWM 设置仍归板级；位置源的采集周期、滤波参数、方向和初始零位归相应位置配置，运行标定结果归位置实例。Port 初始化将公共配置与所绑定位置组件的专用配置组合，调用组件 Init；不把 Encoder 配置类型引入公共 Port ABI。输入配置只在初始化中读取，需要的状态/系数由相应对象保存，不借用临时配置指针。

时基宽度的源码依据：perfc 的 `get_system_ticks()` 返回 `int64_t`，MDI 的 `mdi_tick_t` 已是 `uint64_t`，当前 G431 mfoc Port 才强制截成低 32 位。采用 64 位公共计数，保留完整时间跨度；频率表示每秒 tick 数，仍用 32 位。库使用自身 typedef，不引入 perfc/MDI 公共类型。

32 位无符号差值在间隔严格小于一周回绕且相关比较约束成立时可正确计时，不能据此认定现有实现必然错误；但不将这一限制强加给整个库。移植侧负责提供前台/ISR 均可稳定读取的计数，运行期间不重置或切换时基。仅有窄硬件计数器的目标在 Port 内完成扩展，单纯强转为 64 位不构成完整时间源。

64 位类型不保证 Cortex-M 上共享读写原子性。Port 保证读时钟一致，位置组件保证发布样本一致，不依靠 `volatile` 解决跨上下文访问。公共时间戳加宽不要求所有短间隔运算都加宽：在规定范围内可保留局部 32 位差值，但不能先截断再判断过期；Encoder 原有乘法/除法的输入上界也须随迁移复核。时基读取消耗及必要临界区计入 P10，无测量证据不宣称零开销。

可选扩展仍按消费者启用：`mfoc_port_voltage(mfoc_uvw_t *)` 提供约定三相定子电压，`mfoc_port_bus_current(mfoc_scalar_t *)` 提供母线电流。温度监测、物理故障清除先留板级接口；现有故障通知可接 Motor_Trip。以上所有无实例参数接口仍按当前单功率级静态绑定理解，不代表已经实现同芯片多硬件实例绑定。

### 3.7 位置提供者的生命周期与数据接线

以集成侧持有的一个 Encoder 实例为例，前台更新与高频读取必须绑定同一个实例。Port 不再保存一份重复的角速缓存，Motor 不增加 SetFeedback 或位置样本邮箱；位置组件已有的样本发布机制就是前后台之间的数据连接。

| 阶段 | 调用关系 | 该阶段完成的工作 |
| --- | --- | --- |
| 初始化 | 应用 → Motor_Init → port_init(cfg) | 先验证并准备库内控制对象；Port 完成硬件准备、校准及位置组件初始化；全部成功后 Motor 才进入可启动状态，保持输出关闭 |
| 前台更新 | 应用 → Motor_Run → port_position_run | 提供者按自身周期获取测量，位置组件计算速度、滤波并发布样本；正常停机时也服务位置源 |
| 高频读取 | Motor_Step → Feedback_Step → port_position(angle, speed) | 从同一位置实例取得一致样本，执行必要外推与电角转换，直接填写 Feedback 本拍 angle/speed |
| 对齐标定 | Motor 对齐前台流程 → port_position_calibrate_zero(held_angle) | Motor 保持已知电角度；位置提供者取得所需测量，组件更新零位与速度历史，随后 Motor 按原流程停止 |
| 停止与再启动 | Motor_Stop / Motor_Start | Motor 管控制器和功率输出；正常停机保留位置零位，位置更新继续推进，不因重新启动重复执行硬件初始化 |

Motor_Init 完成对象与硬件准备，不自动开启 PWM。沿用当前先通过前台 Run 建立位置测量再进入有感闭环的用法；没有测量时不凭空发布零角度作为成功结果。不同位置源建立初始角速的具体方式仍由对应组件负责，不增加公共样本就绪数据包。

```text
前台：
Motor_Run → port_position_run → 驱动采集 → 同一位置组件更新/发布
                                                   │
                                      组件自身样本与发布索引
                                                   │
高频：                                             ↓
Motor_Step → Feedback_Step → port_position → 位置组件读取/外推
                                │
                  直接写 next.angle、next.speed
                                ↓
                   Feedback 本拍输出 → Motor → Core
```

以现有 Encoder 为例，Run 发布的是机械角、速度和内部采样时间，Step 获取一致样本并外推电角度；速度 PU 可以复用前台已算结果。位置采样 1 kHz、FOC 20 kHz 时，两次采样间仍可逐拍外推控制角，无需将前台电角度原样保持 20 拍。这只是时序示例，不规定所有传感器的采样率。

有感分支的接入就是 `rc = mfoc_port_position(&next.angle, &next.speed)`。`next` 是 Feedback 的本拍局部输出，不是新增长期状态；后续按当前后端选择和返回处理交给 Motor。有感输入与 SMO/Flux/HFI 输出同口径，Feedback 不反向拥有或修改板级位置实例。强拖轨迹继续由库内启动路径产生。

前台写入组件未发布的样本槽，完整写入后切换发布索引；ISR 读取一份已发布样本。沿用当前单核、单前台写者、高优先级非重入 ISR 读者的并发约定。64 位 tick 同样纳入该完整样本，不单独修改已发布字段；若新平台改变写者或优先级关系，需在该组件/适配内落实一致性，不能只沿用 volatile 声明。

`port_position_run` 调用频率不强制等于采样频率，采集间隔归提供者。霍尔等事件采集由驱动中断保存测量，前台入口只推进实际需要的工作；无前台工作时无需制造一次计算。未启用有感能力的构建裁掉位置服务及读取调用，不要求无感路径实现假位置数据。

源码走查发现当前 Motor_Run 的辨识分支先于 Feedback_Run 且会提前返回。迁移时，有感位置前台服务应在可能跳过它的模式分支之前得到调用；启用的物理位置源可持续发布诊断数据。P09 旋转辨识独立使用电压、电流观测转子运动。采集错误也不能让 Motor 提前跳过应执行的对齐超时、停止或故障收尾。组件继续负责自身测量失败/超时行为，是否退出控制由当前消费者决定，不额外新增多层检查。

零位标定由 Motor 在固定角保持期间直接调用 Port，位置组件通过自身操作完成变更，遵守现有避免 ISR 并发读取被修改状态的条件。取消对齐不提交零位，标定读取失败不覆盖原零位。位置业务移出 Feedback 后，当前只转发 Encoder 操作的 Feedback_Run/CalibrateZero 无其他职责时移除，不保留空包装层。

实例迁移不能只改调用名字：当前 Encoder_Run/CalibrateZero 内还直接读取旧的机械角 Port，实施时须按新的驱动/位置组件接线调整原始测量入口。复用测速、滤波、外推与样本发布算法，不让统一的电角度 Port 回调自身造成递归或被当作原始机械角。高频 Port 的静态绑定按实际编译调用链检查开销和深度，不将“薄包装”当作已经零成本的证据。

## 4. 总计划表

所有项均为首版发布必做项。“完成”须同时有实现、测试和记录。各项验收记录随实施更新。

| 编号 | 工作包与主要交付物 | 前置 | 完成判据 | 状态 |
| --- | --- | --- | --- | --- |
| P00 | 冻结现状基线、目标板与验收范围；形成 baseline 文档 | 无 | 已验证/未实现能力分清，数值与实机测量准则预先写定 | 进行中（[已有 G431 软件基线，待冻结发布范围与台架指标](#p00-progress)） |
| P01 | 定义公共数据、单位、时序、生命周期与 Port 契约 | P00 | 本文第 3 节落为接口规范和支持矩阵，无硬件动作归属空白 | 进行中（[Port 和生命周期已有契约，待核齐支持矩阵](#p01-progress)） |
| P02 | Motor/位置组件解除平台依赖，统一角速与生命周期边界 | P01 | 可接主机 Port 独立运行；两实例隔离；原有失败处理保持 | 完成（[2026-11-12 验收](#p02-verification)） |
| P03 | 采样标定、PU 换算与物理量输入边界 | P01 | 标定不占用高频计算；状态归属明确；FLOAT/FIXED 口径一致 | 完成（[2026-11-12 软件验收](#p03-verification)） |
| P04 | G431 参考适配迁移，落实采样与 PWM 固定时序 | P02、P03 | 原板功能保持，硬件初始化/关断/输出时序符合契约 | 进行中（[Port 已迁移，待 G431 实机复验](#p04-progress)） |
| P05 | 统一前区间电压和 Core 合成/限幅边界 | P02、P03 | Feedback/辨识共用输入语义，限幅与回授无分叉 | 待验收（[软件回归通过，HFI 区间待台架核对](#p05-verification)） |
| P06 | 数值后端契约、局部精度与固定点覆盖 | P05 | 范围、舍入、积分小信号及边界误差通过回归 | 进行中，已验收数值边界、PI 及当前 FIXED 库采样路径。HFI、Rs 实例化暂缓。[分项状态](#p06-status) |
| P07 | 参考生成、反馈选择与激励控制权收口 | P02、P05 | 每拍唯一最终命令；HFI 运行/注入/选择独立可解释 | 完成（[2026-11-17 软件验收](#p07-verification)）。HFI 主反馈获取期保留零 dq 电流闭环；纯注入待台架比较锁定成功率后再议。 |
| P08 | Motor 层停机模型事务应用 | P02、P06 | 成功统一生效；失败整组保持旧模型和系数 | 同实例停机应用已通过[软件验收](#p08-verification)。外部跨基值导入见该节的接口边界。 |
| P09 | 四组离线辨识接通与验证 | P03、P05、P07、P08 | Rs/Ld/Lq/磁链/惯量完整公开流程和验证，取消/失败可靠 | 软件完成，实机待验收。[四项软件结果与计算量](../../mfoc/docs/validation/2026-11-19-p09-identification.md) |
| P10 | HFI 与完整实时链性能优化 | P04、P06、P07、P09 | 生产配置全部关键路径满足实际 PWM 截止预算 | 进行中。[G431 采样](#p10-g431-adc)、[FIXED 限幅与模型电压](#p10-fixed-limit-model)、[同拍编码器波形](#p10-g431-position-waveform)已验证。已有 [Flux、SMO 台架读数](#p10-g431-bench)，生产配置的 PWM 截止余量待测。 |
| P11 | 独立构建、配置组合和系统回归 | P02—P10 | 独立目录可链接运行；全部发布组合通过；无隐藏外部依赖 | 未开始 |
| P12 | 第二芯片迁移与最终实机验收 | P04、P11 | 换芯片只改适配/配置；发布矩阵中的板上能力均有证据 | 未开始 |
| P13 | 发布包、文档、版本及交付核验 | P12 | G1—G8 全通过；干净环境消费发布包可复现 | 未开始 |

依赖允许 P02/P03 并行，也允许 P06/P07 在 P05 后分开推进；接线和实机回归按共同接口版本收敛。

```mermaid
flowchart TB
    P00 --> P01
    P01 --> P02
    P01 --> P03
    P02 --> P04
    P03 --> P04
    P02 --> P05
    P03 --> P05
    P05 --> P06
    P05 --> P07
    P06 --> P08
    P07 --> P09
    P08 --> P09
    P04 --> P10
    P06 --> P10
    P07 --> P10
    P09 --> P10
    P09 --> P11
    P10 --> P11
    P11 --> P12
    P12 --> P13
```

## 5. 工作包实施与验收说明

路径标注“新增”表示拟创建的位置；实施时可以调整文件组织，但必须保持职责、测试及证据映射。接口迁移先保留算法递推和采样顺序，性能或数值行为调整单独验证。

### P00 基线与发布范围

- [ ] 交付基线、验收指标和第一版支持矩阵。对应 R5、R8。

读取现有 `mfoc/tests/`、G431 应用配置和板级绑定，保存可回放输入及配置；辨识旧测试先标记失配原因，不能机械修改断言来制造绿灯。确定第二款不同于 G431 的 Cortex-M 板，优先核对现有 AT32 目标及硬件条件；其 FIXED 运行验证不能用假 Port 替代。

新增 `mfoc/docs/validation-baseline.md`、`mfoc/docs/support-matrix.md`、`mfoc/tests/fixtures/` 的必要基线数据。测量前冻结各算法误差/稳态/收敛、正反转与负载工况，并记录参考测量的不确定度。四项辨识的精度与重复性先在台架上取得数据，再确定发布验收门限。每项实验还须定义允许电流、转速、激励幅值、持续时间及越限退出期限。无可靠参考值不能声称辨识精度通过。发布指标不得在看到结果后无记录地放宽。

验收：基线可重复播放；每个后续行为变更能指出比较对象和容限；第二板型号、采样/PWM时序、可用工具和试验条件已记录。P00 完成不代表第二板已适配。

<a id="p00-progress"></a>

**2026-11-13 进度核对。** G431 的构建配置、Port 寄存器模拟、编码器逐拍回放和已知的实机验证边界都有记录。发布范围仍需列明第二块实际功率板及其试验条件，汇总全库支持矩阵，并冻结算法误差门限。按 11 月 19 日决定，四项辨识在首轮台架测量后确定发布精度与重复性门限。现有 AT32 构建目标是候选，第二块板尚未定型。[G431 迁移基线](../../mfoc/docs/validation/2026-11-10-port-migration.md)

### P01 公共契约

- [ ] 将第 3 节落为接口规范。对应 R2、R3、R4、R6。

涉及 `mfoc/port/include/mfoc_port.h`、`mfoc/motor/include/mfoc_motor.h`、`mfoc/motor/include/mfoc_feedback.h`、`mfoc/core/include/mfoc_core.h` 及辨识头；新增 `mfoc/docs/port-contract.md`。配置按库裁剪、PU 基值/模型、控制参数、板级电气参数、硬件时序分类。修改电机 profile 不需要编辑库源码；同一构建中不同实例可以有独立模型和基值，换算系数在 Init 生成。

按第 3.2/3.3/3.6/3.7 节冻结必需接口及实际使用的可选接口，列出单位、范围、坐标、正方向、写者/读者和初始化要求。有感、无感、强拖的控制侧输出统一为电角度/电角速度，保持 Feedback 扁平字段；保留实际测量与命令轨迹的用途区别。明确采样和 PWM 更新的固定关系，不增加每拍有效性、时间或故障包。Init 继续由 Motor 发起，公共配置提供三项基值和极对数；位置实例由集成侧持有，Motor_Run 主动推进其前台服务；PWM 更新与启动分开。确认完整生命周期表与源码迁移清单一致。

公共时基采用 64 位无符号 tick 与 32 位频率，约定稳定读取及运行期间不重置，局部间隔计算有明确范围。位置组件自行处理测量延迟及外推，不能把总线完成时刻无条件视为真实测量时刻。验收走查正常运行、首次无历史、已有读写失败、Break、启动取消和重启，确保各有所有者；同步项目规范的库/应用适用范围。纯文档阶段无需新增行为测试，情景在 P02—P05 兑现。

<a id="p01-progress"></a>

**2026-11-13 进度核对。** Port 已列出控制量的单位、写者和读者、调用时机、失败语义以及 PWM 启停顺序。Motor 已记录初始化、运行、故障和停机的归属。待按发布支持矩阵核对各配置下首拍缺历史、读写失败、Break、启动取消和重启的契约，明确单功率级绑定以及 HFI、辨识仍依赖编译期基值的组合。P02、P03 已有的软件验收使用了这些接口。[Port 契约](../../mfoc/docs/port-contract.md) · [配置归属](../../mfoc/docs/configuration.md) · [Motor 生命周期](../../mfoc/docs/framework.md)

### P02 可移植 Motor 与生命周期

- [x] 移除库内平台依赖，统一控制侧角速，保留 Motor 主动调用 Port 的生命周期编排。对应 R1、R2、R3、R5。

修改 `mfoc/motor/src/mfoc_motor.c`、`mfoc/position/src/mfoc_encoder.c` 及对应头、`mfoc/motor/src/mfoc_sensorless_start.c`、辨识调度源文件与公开头；移除公共库中的 perfc 类型和直接平台调用，以公共 tick/frequency 完成所需计时。Motor 保留调用窄 Port 的编排，Core/反馈/辨识继续消费明确的数据。有感位置实例移到集成侧，Feedback 通过 Port 获取角速，保留扁平输出；位置组件不配置具体传感器。统一角速口径，不为了统一字段改写已验证的估计与启动递推。

子模块状态迁移通过其语义操作完成，不让适配层直接改 PI、启动或观测器成员。Motor_Init 调用 Port 初始化，Motor_Run 直接推进位置前台服务，Motor_Start/Step/Stop 分别使用启动、更新、关闭接口；对齐流程直接经有感零位 Port 操作接到位置组件。移除已无业务的 Feedback 前台/标定转发，避免额外调用层。准备、校准和物理使能的具体实现仍在集成侧。扩展 `mfoc/tests/motor/motor_test.c`、`motor_run_test.c`、`motor_control_test.c`、`mfoc/tests/position/encoder_test.c`，使用有类型的 fake/replay Port 在无硬件条件下测试完整生命周期。

测试：Motor_Init 正确传入基值/极对数并传播 Port 失败；两个不同配置软件实例在对应测试绑定下交错 Step 与 Stop；角速方向/零位/单位一致；前台更新与高频读取绑定同一实例；停机及需要实际位置的辨识期间仍更新；采集失败不跳过对齐超时或停止收尾；对齐标定成功/失败/取消；PWM 更新不使能输出；PWM 拒绝后不发布正常输出历史；清故障不自启；首次样本和重启；位置组件前台发布与 ISR 一致读取。时间覆盖跨低 32 位回绕、长时间停顿后的现有超时行为和有界间隔换算，不能只改返回类型而保留截断状态。验收为链接仅需声明的标准库及 Port 实现，无 HAL/MDI/perfc/应用对象，已有功能基线未被接口迁移改变。

<a id="p02-verification"></a>

**2026-11-12 验收记录**（基于 `4a8514e` 的 P02 改动）。原版本在仓库外编译辨识头时缺少 `perf_counter.h`。MFOC 在公共头 `mfoc_types.h` 定义进度返回类型 `mfoc_fsm_rt_t`。辨识模块管理自己的阶段推进，库构建清单已移除 MODUS 路径。主机消费者只用导出目录内的生产源码、自己的 Port 和标准库。

辨识前台通过实例内的 `run_pt` 保存等待位置。独立构建使用 C11 PT，G431 通过 `MFOC_USE_PERFC_FSM=1` 使用原生 perfc PT。运行 `.\mfoc\tests\scripts\run_core_tests.ps1 -Tests motor_identification` 可回放 Rs、超时和取消重启；追加 `-Perfc` 可用 perfc 执行同一组断言。

| 仓库外配置 | 运行结果 |
| --- | --- |
| FLOAT、FIXED，辨识关闭 | 两个配置不同的 Motor 交错运行。先停 A，B 继续执行，其三相 duty 与单独回放 B 的结果逐分量相同 |
| FLOAT、FIXED，辨识开启 | 同一双实例用例通过。另以已知 1.2 PU 电阻模型驱动真实 Motor 与 Rs 流程，两种配置均测得 1.2000 PU，实验完成后 PWM 关闭 |

复跑使用 [独立构建脚本](../../mfoc/tests/scripts/run_standalone_tests.ps1) 和 [主机回归](../../mfoc/tests/scripts/run_tests.ps1)。回归覆盖延迟首次轮询、完成事件与超时同拍、下一阶段独立计时、取消后重启和故障关断。Cortex-M0 FIXED、Cortex-M4 FLOAT 通过开启辨识的库对象编译，G431 FIXED 开启辨识的固件完成链接。Rs 数值来自主机电阻模型，台架辨识精度和 PWM/ADC 实际时序按后续工作包验收。

### P03 采样标定与数据准备

- [x] 约定校准、PU 换算及物理量准备的所有者。对应 R2、R3、R4。

整理 `peripheral/stm32g431/mfoc/mfoc_port.c` 及配置：硬件驱动获取原始值，Port 完成零偏、相序、极性与 PU 换算后提供控制量。板级绑定持有零偏、预计算增益和准备状态。重复的纯换算数学可以复用，但不预设新增 `sampling` 对象、目录或逐拍数据包装层。

物理量和 PU 换算只在边界发生，ADC 量程不等于运行电流限值，测量不被静默裁成 ±1 PU。校准在 Motor_Init 调用的 Port 准备过程中完成，明确采样数、终止和失败行为，不占用高频计算；需要异步准备的目标另行明确完成契约，不能未准备好却返回初始化成功。母线刷新率及有感位置处理由各自提供者落实，不要求所有量每拍重采或随数据附带时间。

扩展现有 Port 测试，覆盖正负电流、极性/相序、ADC 边界、过零、零偏与校准失败、不同 PU 基值、母线换算和 FLOAT/FIXED 一致性。成功校准后再启用控制，不能让 ISR 读取半更新的系数。主机 replay Port 直接提供约定 PU 数据，绕开硬件采集。

<a id="p03-verification"></a>

**2026-11-12 软件验收。** 在 `928e111` 上从仓库根目录运行 `.\mfoc\tests\scripts\run_port_tests.ps1`，FLOAT 与 FIXED 均通过。测试运行实际 G431 Port 实现，以主机寄存器模型提供 ADC 帧、PWM 状态和时钟。[实现](../../peripheral/stm32g431/mfoc/mfoc_port.c) · [验收用例](../../mfoc/tests/port/port_test.c)

| 检查 | 验证结果 |
| --- | --- |
| 零偏及所有权 | Port 在输出关闭的初始化阶段采集 10 帧，保存三相各自的零偏与增益。缺帧或初始化参数错误时不发布新配置。控制 ISR 只读取新鲜帧并换算。 |
| 三相电流 | 以两种极性和五种增益逐码扫描 14 位 ADC，按相同物理公式核对两个数值后端。另验相序、过零和 2 PU 输入，保留超出 1 PU 的有效测量。 |
| 共同基值及母线 | 将电流基值从 3500 改为 7000 mA、电压基值从 12000 改为 24000 mV，核对相同输入的 PU 缩放。运行期间由集成侧调度母线更新。 |

板上 ADC 触发与 PWM 生效窗口、真实电流极性和测量误差归入 P04 实机验收。[时序及校准边界](../../mfoc/docs/port-contract.md)

### P04 G431 参考适配

- [ ] 当前应用迁移到新契约并保住实机基线。对应 R2、R3、R5。

修改 `peripheral/stm32g431/mfoc/mfoc_port.c`、`target/stm32g431/mfoc_motor_irq.c`、`target/stm32g431/mfoc_motor_irq_binding.h`、`class/mfoc_app.c`。库外保留目标驱动与直接调用；Port 初始化组合公共换算配置与专用位置配置，调用驱动完成 AS5600 配置并初始化位置组件。位置实例由集成侧持有，port_position_run 与 port_position 绑定该同一实例，分别完成前台更新及高频结果输出；IRQ/波形仍归目标集成。原机械角诊断转接相应驱动/位置源诊断，不把新电角度接口当作机械角接口使用。沿用经整理的 Motor Init/Run/Step 编排，应用不额外调用一套位置调度，不复制控制算法。

修改 `mfoc/tests/port/port_test.c`、`mfoc/tests/motor/motor_irq_test.c`；新增 `mfoc/tests/port/port_contract_test.c`。

测试：启动接口使中性 duty 生效后才开门极，普通 PWM 接口只更新输出；ADC 数据与触发对应；duty 到比较值的换算、量化及更新方式；现有读写失败与关断行为；Break；停止保留采样触发；前台 I2C 不进入 ISR；64 位时钟在前台/ISR 的一致读取。板上核对 ADC 触发、计算和比较值生效点，不用单元测试代替时序测量，也不为验证加入生产路径逐拍校验层。

<a id="p04-progress"></a>

**2026-11-13 进度核对。** G431 已接入新 Port 与 IRQ 契约，FLOAT、FIXED 固件均已链接。寄存器模拟覆盖初始 duty 先于门极使能、正常运行时的 PWM 更新，以及停止后保留 ADC 触发。迁移后的台架记录仍缺 ADC 触发和电流采样、控制计算、CCR 写入到 PWM 实际生效的先后关系，以及电流极性和测量误差。P04 与 P05 均依赖 P02、P03，可并行推进。[Port 迁移及验证范围](../../mfoc/docs/validation/2026-11-10-port-migration.md)

### P05 统一前区间模型与 Core 合成

- [ ] 反馈、HFI、辨识共用相同前区间电压语义。对应 R3、R5、R6。

修改 `mfoc/control/src/mfoc_modulation.c`、`mfoc/core/src/mfoc_core.c`、Feedback 与辨识输入。Motor 保留必要前拍输出，使用库内纯数学形成统一模型电压，Feedback/HFI/辨识不各自解释一套历史。模型与实际采样区间的对应由固定 Port 时序保证；若硬件有额外延迟，须明确适配方式并验证，不给所有控制对象增加通用时间字段。模型可包含已实现补偿，不能把命令值标为实测值；直接电压辨识遵守相同周期关系。

Core 保留 PI、注入及将来 dq 前馈的明确合成位置，合成后统一限幅和抗饱和。首版不实现尚未需要的前馈算法；保留电压 `V/Vbase` 与调制 `V/Vdc` 的区别。

扩展 `mfoc/tests/core/core_test.c`、Feedback 算法集成测试；新增 `mfoc/tests/motor/voltage_interval_test.c`。测试首次无历史、母线变化、限幅、PWM 失败后停止与重启、直接电压辨识，以及已声明支持的采样/更新关系。量化误差有容限；硬件不得静默进行契约外的脉宽修正。相同区间的消费者收到同口径电压，HFI 激励与对应电流增量经板上验证。

<a id="p05-verification"></a>

**2026-11-12 软件验收。** Motor 在普通闭环、辨识虚拟角和辨识直出三条路径提交 PWM 后，按最终 duty 与该区间的母线快照重建一次模型电压。`u_ab_previous` 为 NULL 表示没有前区间，非空零向量表示提交了零电压。Core 遥测有独立的有效位。首拍、母线切换、合成限幅、直接电压和虚拟角路由、失败关断与重启均已在 FLOAT/FIXED 主机回放中校验。[Motor 区间测试](../../mfoc/tests/motor/voltage_interval_test.c) · [Core 注入限幅与积分回写](../../mfoc/tests/core/core_test.c)

Rs 的低、高档分别使用 0.8 和 1.2 PU 母线，测试用前拍提交的 duty 驱动 1.2 PU 电阻模型，两种数值后端的辨识误差均小于 0.01 PU。另覆盖 HFI 缺压时提出首个注入、有效零电压和非零脉冲，以及磁链辨识叶在正反转下的区间中点投影。复跑入口为 `run_core_tests.ps1 -Tests voltage_interval,core,motor_identification,identification_rs`、`run_hfi_tests.ps1` 和 `run_flux_tests.ps1`，位于 [tests/scripts](../../mfoc/tests/scripts)。[变母线 Rs 回放](../../mfoc/tests/motor/motor_identification_test.c) · [磁链辨识叶测试](../../mfoc/tests/identification/identification_flux_test.c)

G431 的 FLOAT/FIXED 固件均已在开启辨识的配置下链接。实际 HFI 注入电压生效点与 ADC 电流增量是否相隔约定的区间，仍需按 P04/P05 在台架上测量。

### P06 数值契约与精度

- [ ] 完成数值范围说明、局部精度修复与两后端回归。对应 R4、R5。

修改 `mfoc/math/include/mfoc_numeric.h`、`mfoc/math/src/mfoc_numeric.c`、`mfoc/control/src/mfoc_pi.c` 及必要的算法配置；新增 `mfoc/docs/numeric-contract.md`。保留公共 int32/15 位小数和 BAM32 的基础，限定范围乘法必须有显式语义，普通 PU 运算不默认为 ±1。按证据为 PI 等必要积分状态保留余量；不机械统一所有算法内部 Q 格式。

按模块确定采样频率与基值的归属。启动、SMO、Flux 观测器和 EKF 使用实例配置，电感实验可在前台选择实例时间系数。HFI、Rs 和磁链辨识沿用编译期配置，后续复核节点见下表。各模块单独验收其支持频率下的离散行为。

扩展 `mfoc/tests/math/math_test.c`、`mfoc/tests/control/pi_test.c`；新增 `mfoc/tests/control/pi_precision_test.c`。测试包括超过 1 PU 的合法输入、正负舍入、载体极值、向零截断、小误差持续积分、限幅及恢复、复位、模型参数可表示范围、后端边界一致性。纯接口整理的 FIXED 逐拍结果保持一致；精度改善单列行为差异及容限，不能混入等价重构报告。

明确 FIXED 热路径禁用运行时软浮点；常量表达式、边界配置及 Init 可有经审查的浮点转换，并记录是否带来库依赖。EKF 的 FLOAT-only 编译拒绝属于支持矩阵，不强行定点化。数字接口与 CMSIS 等目标加速接口的格式转换须有误差证明。

<a id="p06-status"></a>

| 分项 | 当前验收与后续条件 |
| --- | --- |
| 标量、角度、乘法、PI 与模型范围 | 已验收数值边界与 PI 跨拍精度。真实 Feedback 模型更新已覆盖 profile 端点、相邻越界值，以及 Flux 平方系数的后端差异。[模型范围与用例](../../mfoc/docs/numeric-contract.md#feedback-model-range) |
| FIXED 采样与三角后端 | 已检查 M0 上 Motor 采样可达的库函数及独立采样入口。G431 当前使用 LUT。若接入 CMSIS-DSP 等外部实现，届时按实际适配层补格式转换误差；整机 ISR 周期与栈峰值归 P10。[数值与 ARM 验收](../../mfoc/docs/numeric-contract.md#fixed-sampling-check) |
| 启动、观测器与电感 | 已验收启动、SMO、Flux 的双后端双频率用例、FLOAT EKF，以及 Ld/Lq 共用的可选前台时间系数。 |
| HFI、Rs、磁链与惯量辨识 | HFI 与 Rs 实例化暂缓。磁链辨识窗长与半拍角度系数在 P09c 接通旋转采样时复核。惯量待 P09d 有实际算法再评估。 |

<a id="p06-pi-verification"></a>

**2026-11-13 FIXED PI 小信号积分验收。** 输入、输出和积分读数 `ui` 使用 int32 Q15。实例用一个有符号 16 位余数保留积分低位，完整 Q30 积分等于 `ui × 32768 + remainder`。`ui` 向零截断，余数范围为 −32767 至 32767。FLOAT、FIXED 都通过了正负小误差累积、跨零、限幅恢复、Track 首拍、ApplyLimit 和 Reset 回归。每拍仅增加一个 Q30 计数时，连续 32768 拍恰好累积一个 Q15 计数，负方向对称。复跑入口为 `mfoc/tests/scripts/run_pi_tests.ps1`。[PI 实现](../../mfoc/control/src/mfoc_pi.c) · [精度用例](../../mfoc/tests/control/pi_precision_test.c)

主机差分使用相同配置和输入驱动优化后的 int64 Q30 基准与有符号余数实现。固定种子 `0x051C0F15`，执行 3,358,923 次公开调用，其中 Step 2,325,812 次，另覆盖 Init、Track、ApplyLimit、Reset。数据覆盖全 int32 范围、极值、窄限幅及微小增量，并注入 12,000 个合法积分初态。返回码、每拍输出、`ui` 和重建的完整积分均逐次相同。

Cortex-M0 使用 Clang 19.1.5 编译 FIXED，链接整数乘法辅助函数。`-Os` 与 `-O2` 生成相同的 Step 机器码。指令数由 Unicorn 对实际执行路径逐条统计，栈峰值由入口 SP 到最低 SP 的差值取得。未饱和用例采用 `kp = 1/4`、`ki_ts = 1/8` 和 ±1/8 PU 误差。指令数栏依次列出正、负方向。

| M0 单次调用或资源 | int64 Q30 基准 | Q15 加有符号余数 |
| --- | ---: | ---: |
| Step 未饱和，指令数 | 227 / 231 | 201 / 206 |
| Step 在输出限幅处冻结，指令数 | 181 / 191 | 158 / 162 |
| Step 从正 / 负限幅反向恢复，指令数 | 231 / 227 | 207 / 200 |
| ApplyLimit 常规修正，指令数 | 48 / 48 | 44 / 43 |
| Step 自身栈帧 / 含乘法辅助函数的栈峰值 | 56 / 72 B | 48 / 64 B |
| ApplyLimit 栈峰值 | 20 B | 20 B |
| PI 实例大小 | 40 B | 32 B |
| Step 函数体积 | 424 B | 440 B |

上述栈值覆盖所测 PI 路径。完整 ISR 从采样完成到 PWM 最晚写入的最坏响应时间与栈峰值，仍须在目标板的生产配置下按 P10 验收。

<a id="p06-conversion-verification"></a>

**2026-11-13 运行时 Q15 转换验收。** `mfoc_from_float` 在 FIXED 下取最近的 Q15 计数，恰好半个计数时向远离零的方向舍入。过去先加减 `0.5f` 还会触发一次 binary32 舍入，导致部分靠近分界的输入或原本精确的奇数计数被多舍入一位。现在先截取整数，再比较带符号的小数部分。FLOAT 直接返回输入。[转换实现](../../mfoc/math/src/mfoc_numeric.c)

主机回归覆盖正负 0.5、1.5 个 Q15 计数及各自相邻的两个可表示 float；精确的 ±8,388,609 个计数；±65536 边界的内侧、端点和外侧；以及有符号零、NaN、无穷大。饱和边界内侧的相邻值转换为 ±2,147,483,520，端点和外侧分别返回 `INT32_MAX`、`INT32_MIN`。数学测试覆盖 FLOAT 的 LUT、libm、外部三种配置和 FIXED 的 LUT、外部两种配置。另用双精度 `llround` 加饱和规则对照 4,000,000 个 FIXED 结果，全部一致。[边界用例](../../mfoc/tests/math/math_test.c) · [主机回归入口](../../mfoc/tests/scripts/run_tests.ps1)

**2026-11-13 数值契约。** 已按现有实现汇总 PU 单位与载体范围、两类乘法的输入约束、常量宏与运行时边界转换的区别、BAM32 角差及 PI 余数语义。将编译期常数改成实例系数前，先核对热路径的装载、运算和峰值拍，按第 7 节确认可能的开销。[数值契约](../../mfoc/docs/numeric-contract.md)

**2026-11-13 HFI 与辨识系数审查。** 已分清控制采样频率、电频率基值和 Port 超时时基，并按 Init、前台、每拍和载波第八拍列出实际用途。临时候选在默认参数下保持所测路径逐拍输出。HFI 有效第八拍和 Rs 采样在 M0 FIXED 上有指令增量；仅实例化 Rs 仍使采样路径栈增加 8 B。本轮暂缓 HFI 和 Rs 实例化，生产代码保留编译期系数。[系数清单与前后开销](../../mfoc/docs/validation/2026-11-13-p06-instance-coefficients.md)

**Rs 重复启动已修复。** 前台进入 Rs 时重建该轮的采样工作区。同一实例连续辨识 1 PU 和 2 PU，FLOAT、FIXED 的两轮低档与高档均各收满 6,000 帧，第二轮结果随阻值更新。M0 FIXED 与 M4F FLOAT 在 `-Os`、`-O2` 下的辨识采样分派及 Rs 采样叶机器码、重定位均与修复前一致。[两轮结果与前台、ISR 开销](../../mfoc/docs/validation/2026-11-13-p06-instance-coefficients.md#同一实例重复启动-rs-的修复与验收)

**Ld/Lq 可选时间系数已落地。** 默认三参数 Finish 保留编译期系数。运行时实验调用 InitTiming 一次，两个轴在前台共用一个 4 B 标量。M0 FIXED 和 M4F FLOAT/FIXED 在 `-Os`、`-O2` 下的 Step 和默认 Finish 机器码及重定位均与改前一致。M0 FIXED 默认电感入口独立链接无软浮点；可选初始化的前台开销已确认。[两轴回归、链接与数值验收](../../mfoc/docs/validation/2026-11-13-p06-instance-coefficients.md#p06-inductance-dt)

**2026-11-15 启动与观测器的非默认配置已验收。** 同一构建以 20 kHz／400 电 Hz 和 16 kHz／250 电 Hz 回放启动接管、SMO 测速窗、Flux 正反转和 EKF 模型预测。启动、SMO、Flux 的 FLOAT/FIXED 及 EKF 的 FLOAT 均通过。HFI、Rs 实例化继续暂缓，磁链辨识叶仍使用编译期窗长与半拍系数，惯量系数待 P09d 有实际算法后再评估。[双配置用例、容限与复跑入口](../../mfoc/docs/validation/2026-11-13-p06-instance-coefficients.md#p06-instance-rates)

**磁链辨识保留编译期配置。** P06 已完成窗长与半拍角度系数的开销对照。实例化推迟到 P09c 接通旋转采样时，再根据实际多频率需求和届时的采样链成本决定。候选每实验多占 12 B，增加一次前台准备，M0 在 `-Os` 下满窗后重调多 1 条指令。[采样、Finish、Prepare 与链接对照](../../mfoc/docs/validation/2026-11-13-p06-instance-coefficients.md#p06-flux-window)

### P07 参考生成与控制权

- [x] Motor 确定本次运行的注入许可，各控制路径选定本拍角度与 dq 参考。对应 R5、R6。
- [x] HFI 主反馈获取期采用零 dq 电流参考，两轴电流 PI 逐拍运行并叠加 HFI 注入。速度 PI 和分频暂停。首个有效反馈拍沿用电流积分，恢复当前模式的参考与速度环调度，HFI 继续发布载波。

- [x] 命令入口拒绝禁用的速度环、辨识占用或故障时的参考命令，以及当前主反馈不支持的显式 HFI 启动。
- [x] ISR 按同一阶段快照判断一次辨识采样许可。直压阶段完成后，前台交接前每拍提交三相 50% 占空比。虚拟阶段等待期间保持辨识给出的 dq 参考和角度。
- [x] 辨识模块从同一次阶段读取选定直压、虚拟 dq 或常规控制。Motor 的中断执行所选控制，前台与命令入口只查询辨识是否仍占用输出。
- [x] 选参考时确定本拍速度分频，PWM 和模型电压提交成功后写回计数。到期拍、中间拍写 PWM 失败都保留原计数。

- [x] 外部电流命令、跨拍 Iq 和本拍 Core 的 dq 参考分开。虚拟辨识保留自己的参考与角度，对齐与强拖只持有 Id。

G431 release，Clang 19.1.5、Cortex-M4F、`-Os`。以下是 `mfoc_motor_Step` 函数体的静态 Thumb 指令数，包含所有分支。ADC IRQ 入口在各配置下均为 29 条，机器码一致。

| 辨识 | HFI | FLOAT 改前 → 改后 | FIXED 改前 → 改后 |
|---|---|---:|---:|
| 关 | 关 | 208 → 198 | 206 → 192 |
| 开 | 关 | 267 → 252 | 266 → 250 |
| 关 | 开 | 216 → 201 | 215 → 199 |
| 开 | 开 | 276 → 259 | 275 → 257 |

以相同采样和 PWM 返回值回放目标机器码，按拍计数。默认配置的速度 PI 到期拍，Motor 调度减少 12 条（FLOAT）和 14 条（FIXED），分频中间拍分别减少 11 和 13 条。只开辨识时，Rs 采样拍两种后端均减少 14 条，直压等待拍均减少 7 条。回放使用测试 Port 与 Feedback，指令数不代表板上周期。

本次控制方式归属调整以上表的改后值为基线。G431 八个构建的 ADC IRQ 入口和 `mfoc_motor_Step` 机器码、调用重定位及静态栈帧均与基线一致。只开辨识时，FLOAT、FIXED 的直压等待拍调度均为 75 → 75 条，虚拟 dq 等待拍均为 89 → 89 条。回归将 SPINUP、FLUX、INERTIA 各暂停三拍，逐相对照以保留参考和非零辨识角计算的 Core 占空比，并验证 Ld、Lq 等待时 PWM 回中性。[交接用例](../../mfoc/tests/motor/motor_identification_test.c)

本次 dq 参考拆分以本轮开始时的工作区为基线。两种数值后端均用非零外部 dq 命令验证速度分频、正反转无感交接和辨识等待，逐相核对分频到期拍、中间拍与虚拟等待拍的 PWM。[速度环](../../mfoc/tests/motor/motor_control_test.c) · [无感交接](../../mfoc/tests/motor/motor_handover_test.c) · [辨识等待](../../mfoc/tests/motor/motor_identification_test.c)

| 辨识 | HFI | G431 Step 静态指令 FLOAT | G431 Step 静态指令 FIXED | Step 栈帧 FLOAT / FIXED（字节） |
|---|---|---:|---:|---|
| 关 | 关 | 198 → 182 | 192 → 182 | 152 → 144 / 144 → 144 |
| 开 | 关 | 252 → 234 | 250 → 234 | 152 → 152 / 152 → 152 |
| 关 | 开 | 201 → 189 | 199 → 189 | 152 → 144 / 152 → 144 |
| 开 | 开 | 259 → 242 | 257 → 242 | 152 → 152 / 152 → 152 |

G431 八种构建的 IRQ 入口均为 29 条，只有写回 `irq_result` 的成员偏移改变。Motor 实例字节数与基线相同。默认配置普通电流拍的调度指令 FLOAT 为 99 → 84，FIXED 为 97 → 84。速度 PI 到期拍分别为 116 → 95、113 → 95，中间拍为 103 → 89、100 → 89。只开辨识时，两种后端的直压等待均为 75 → 71，虚拟 dq 等待均为 89 → 80。ARM 机器码回放逐字节比对 Core 输入和 PWM 提交值。上述计数使用测试 Port 和 Feedback，板上周期未测。

Cortex-M0 默认 FIXED 的 `-Os` 复核中，Step 静态指令为 292 → 255，局部栈帧为 168 → 160 字节。G431 共 236 组、M0 FIXED 共 118 组同输入回放，所测路径的执行指令数均持平或减少。

**HFI 注入许可验收。** 启动前按本次控制确定 HFI 是否运行、是否发布注入。POSITION 普通启动和显式启动后停机再普通重启均无注入，`observe_hfi` 仅供诊断。HFI 主反馈接受 `SetSpeed + Start`，获取结束后按原分频恢复速度 PI。对齐、强拖及辨识等待均由各自参考独占输出。测试使用真实 Motor、Feedback、HFI 和 Core，硬件访问使用测试 Port。[集成用例](../../mfoc/tests/motor/hfi_permission_test.c)

G431 Cortex-M4F、Clang 19.1.5、`-Os`，下表为 `mfoc_motor_Step` 从入口到返回的执行指令数，包含子调用。开启辨识和 HFI，分别选 POSITION 与 HFI 主反馈，关闭 SMO。普通拍取启动后的第二拍，轴解算拍使用足以触发 `atan2` 的载波窗口。两组授权路径回放均逐字节核对 Core 输入和 PWM 输出，包含获取首拍有非零实测电流的情况。板上周期未测。

| 路径 | FLOAT 改前 → 改后 | FIXED 改前 → 改后 |
|---|---:|---:|
| POSITION 普通电流拍 | 913 → 697 | 1434 → 1121 |
| POSITION 强拖旋转拍 | 923 → 708 | 1442 → 1129 |
| POSITION 显式 HfiStart | 913 → 804 | 1434 → 1325 |
| HFI 主反馈 SetSpeed + Start | 914 → 803 | 1434 → 1323 |
| HFI 主反馈轴解算拍 | 1227 → 1116 | 2919 → 2808 |

纯对齐、Rs 直压和虚拟 dq 等待的执行指令数均持平。两种数值后端的 Step 局部栈帧均为 152 字节。POSITION 的 Feedback 静态指令为 46 → 47，栈帧为 56 字节。HFI 主反馈为 52 → 51，栈帧为 56 → 48 字节。默认关闭 HFI 的 G431 release 完整固件分别与该后端本批改前的 `.bin` 逐字节一致。另测 HFI 关闭、辨识开关两档的 18 种合法组合，覆盖 NONE、POSITION、SMO、Flux 和 FLOAT EKF，Motor、Feedback、Core 的指令字节和重定位目标均等价。

**2026-11-15 HFI 主反馈获取期电压路径。** 本版保留零 dq 参考的电流闭环。使用 G431 应用的电流 PI 增益（`Kp = 0.472810`、`Ki·Ts = 0.070097`），向采样端口回放 `Id = +0.25 PU`、`Iq = 0`，初始角度与积分为零。首拍 HFI 请求 `Uα = +0.04 PU`，PWM 提交后由 duty 重建的 `Uα` 为 −0.095727（FLOAT）和 −0.095703（FIXED）。把 Id 改为 −0.25，分别得到 +0.175727 和 +0.175720。暂停电流 PI 时，两种电流方向都得到 +0.040000 和 +0.040009。现有闭环会抑制残余电流，也可能使本拍总电压与载波请求反向。[应用增益](../../class/mfoc_app.c) · [电压合成](../../mfoc/core/src/mfoc_core.c)

下表按 **FLOAT / FIXED** 列出 G431 Cortex-M4F 目标机器码从 `mfoc_motor_Step` 入口到返回的动态指令数，包含子调用。Clang 19.1.5、`-Os`、LUT，开启 HFI 主反馈、POSITION 和辨识，关闭 SMO 和极性脉冲。使用真实 Motor、Feedback、HFI、Core 与测试 Port。两种纯注入实现均为临时对比，沿用圆形限幅、SVPWM 和提交后的电压重建。稳态行对齐两边的 PI、HFI 和分频状态，参考选择函数保持内联。板上周期和目标电机的锁定成功率待测。

| 采样拍 | 现有零 dq 闭环 | 纯注入，直接恢复 PI | 纯注入，锁定首拍 Track |
|---|---:|---:|---:|
| 获取普通拍 | 804 / 1355 | 682 / 1080 | 684 / 1082 |
| 获取期八拍解调、提取轴角 | 1069 / 2359 | 947 / 2084 | 949 / 2086 |
| 锁定首拍，电流模式 | 1081 / 2372 | 1079 / 2378 | 1152 / 2555 |
| 锁定首拍，速度 PI 同拍到期 | 1138 / 2464 | 1137 / 2471 | 1210 / 2639 |
| 锁定后稳态，速度 PI 未到期 | 812 / 1377 | 811 / 1384 | 814 / 1387 |
| 锁定后稳态，速度 PI 到期 | 864 / 1451 | 863 / 1458 | 866 / 1461 |

用相同输入保持 16 拍，再在受控的锁定边界给出 `Iq_ref = +0.10 PU`，首个有效拍的总 `(Ud, Uq)` 依次约为 `(−0.3761, +0.0543)`、`(−0.0957, +0.0543)` 和 `(+0.0400, 0)`。第三种方案在积分限幅有余量时，对两轴各调用一次 `mfoc_pi_Track`，令首拍基波电压为零。它在普通获取拍省 120 / 273 条，在速度 PI 同拍锁定时增加 72 / 175 条，速度模式稳态每拍增加 2 / 10 条。待台架比较两条电压路径的锁定成功率后，再评估是否采用纯注入。[PI 跟踪实现](../../mfoc/control/src/mfoc_pi.c)

<a id="p07-verification"></a>

**2026-11-17 启动命令仲裁与软件验收**

- [x] Rs 已登记且 Motor 仍为 IDLE 时，修复前 `AlignStart`、`OpenLoopStart` 和受支持的 `SensorlessStart` 都返回成功、使能 PWM，首拍实际执行 Rs。现在三个前台入口在写状态或开启 PWM 前返回 `INVALID_STATE`。回归逐字节核对 Motor，核对 PWM 写入和使能次数，并用普通 `Start` 接着运行原辨识。[前台入口](../../mfoc/motor/src/mfoc_motor.c) · [冲突回归](../../mfoc/tests/motor/motor_identification_test.c)
- [x] HFI 主反馈在电流、速度两种模式下，用非零三相采样逐拍核对由 duty 重建的 αβ 电压。前两拍零 dq 参考的积分延续到受控的锁定首拍，速度环在该拍恢复分频。独立 HFI 算法用例覆盖完整锁定序列。[Motor 集成](../../mfoc/tests/motor/hfi_permission_test.c) · [HFI 算法](../../mfoc/tests/position/hfi_algorithm_test.c)
- [x] 无感启动捕获前丢反馈继续强拖等待；捕获后丢反馈在本拍关断 PWM。捕获后主动停止，以及故障清除后，在同一实例上反向启动都从保持阶段开始，旧角度偏移和前区间电压已清除。SMO、Flux 在 FLOAT 和 FIXED 通过，EKF 在 FLOAT 通过。[接管回归](../../mfoc/tests/motor/motor_handover_test.c)
- [x] 框架已列出各状态的控制角、最终 dq 参考及 MTPA/弱磁后续接入点。Feedback 文档分别列出编码器和 SMO 的闭环、接管、并行观测及波形用途。诊断能否复用同拍外推值由 P10 评估。[参考选择](../../mfoc/docs/framework.md) · [编码器与 SMO 使用方](../../mfoc/docs/modules/feedback.md#feedback-consumers) · [P10 性能候选](#p10-g431-current-loop)

从仓库根运行 `mfoc/tests/scripts/run_tests.ps1`，FLOAT、FIXED 主机回归通过。脚本包含 SMO 主反馈、关闭 HFI、开启辨识的启动冲突和接管用例。另以同一组 `motor_identification`、`motor_handover` 用例运行 Flux（backend 4）与 FLOAT EKF（backend 5）。[主机回归入口](../../mfoc/tests/scripts/run_tests.ps1) · [配置矩阵入口](../../mfoc/tests/scripts/run_core_tests.ps1)

与本次前台入口修复前的工作区比较，G431 release、Clang 19.1.5、Cortex-M4F、`-Os` 下，40 组配置（FLOAT 22 组、FIXED 18 组，覆盖各合法主反馈与 HFI、辨识开关）中，ADC 采样 IRQ 和 `mfoc_motor_Step` 的指令字节、重定位目标与局部栈帧全部一致，采样 ISR 新增 **0 条指令**。关闭辨识的 20 组中，整个 Motor 目标文件的可执行代码和重定位也一致。G431 默认 release 的 FLOAT、FIXED 固件均已重新链接，各自的 `.bin` 与本批前保存的固件逐字节一致。

启用辨识后的新增判断在前台启动入口执行。以 SMO 主反馈、HFI 关闭为例，`AlignStart`、`OpenLoopStart`、`SensorlessStart` 三个函数体的静态指令数分别增加 1 / 2、1 / 2、4 / 3 条（FLOAT / FIXED）。HFI 锁定成功率和完整实时链的板上周期留待台架验证。

### P08 停机模型事务

- [x] 通过 Motor 统一应用本实例基值上的辨识结果或配置参数。对应 R2、R4、R7。

开启 `MFOC_MOTOR_MODEL_UPDATE_ENABLE=1` 后，前台在 Motor 处于 IDLE、没有实验占用输出且已排除并发访问时调用 `mfoc_motor_ApplyModel(motor, &params)`。入口核对极对数，复用 Feedback 现有的系数准备与提交逻辑。所有观测器接受后才写入 `motor->params`。更新 Rs、L 或磁链后，电流 PI 与速度 PI 的增益沿用原整定值，应用按电机特性复核。[Motor 入口](../../mfoc/motor/src/mfoc_motor.c) · [Feedback 模型重建](../../mfoc/motor/src/mfoc_feedback.c)

**接口边界。** 同一 Motor 运行期间使用 Init 选定的基值。该实例的 `GetResult` 直接返回 `mfoc_motor_params_t`，应用显式决定是否生效。更改基值或极对数需要完整重新初始化。按 2026-11-17 的接口决定，取消为每台 Motor 保存三项基值、在每份参数里重复携带基值的候选实现。裸 PU 值无法识别其物理来源。以后若支持跨设备文件导入，来源校验、拒绝和换算应由能读取来源信息的导入层负责。

<a id="p08-verification"></a>

**2026-11-17 软件验收。** FLOAT、FIXED 均用真实 SMO 与 Flux，FLOAT 另启用 EKF。覆盖未初始化、运行、故障、IDLE 已预备 Rs、极对数变化、参数越界，以及后一个观测器拒绝时整个 Motor 和假 Port 逐字节保持原值。重复应用后，重启的首拍提交电压，第二拍使用该电压估算，结果与按新参数直接 Init 的实例一致。启用辨识时，用 2 PU 电阻模型跑通 Rs、自动停机、读结果及显式应用，确认读出后旧参数仍然有效，应用成功才更新 Rs 和 SMO 系数。[永久用例](../../mfoc/tests/motor/motor_model_update_test.c) · [主机回归](../../mfoc/tests/scripts/run_tests.ps1)

G431 release、Clang 19.1.5、Cortex-M4F、`-Os`。对比 P08 前工作区的 28 组配置，覆盖 FLOAT/FIXED、HFI 与辨识裁剪、POSITION/SMO/HFI 主反馈和多观测器组合。打开更新开关后，所有原有 Motor、Feedback 和采样 IRQ 函数的指令字节及重定位一致。Motor 实例 RAM、Init 局部栈与采样 ISR 增量均为零。关闭开关时，G431 默认 FLOAT、FIXED 的 release 固件各自与改前的 `.bin` 逐字节一致。

| G431 前台成本 | 带基值描述的候选 FLOAT / FIXED | 直接传 `mfoc_motor_params_t` 的 FLOAT / FIXED |
| --- | ---: | ---: |
| 每实例新增 RAM | 16 / 16 B | 0 / 0 B |
| Init 局部栈增量 | 16 / 16 B | 0 / 0 B |
| ApplyModel 函数体静态指令，辨识关闭 | 45 / 43 | 32 / 32 |
| ApplyModel 函数体静态指令，辨识开启 | 47 / 46 | 34 / 34 |
| ApplyModel 局部栈 | 24 / 24 B | 24 / 24 B |

静态指令数和局部栈只计新增入口。成功路径还会调用原有 Feedback 和已启用观测器的系数重算。上述数据来自反汇编与编译器栈报告，板上周期未测。

### P09 四组离线辨识

按 2026-11-19 确认，本批统一交付四种无编码器操作的软件实现及回归。真实电机的精度与重复性根据台架实测评估，库中只保留实验成立、数值可求和运行保护所需的条件。[验证配置、测量结果与指令对照](../../mfoc/docs/validation/2026-11-19-p09-identification.md)

- [x] P09a，Rs 两档配对、重复运行、实例限流、停机及结果读取。
- [x] P09b，对齐后保持静止，连接 d/q 两轴短脉冲、采样与前台求值。
- [x] P09c，无编码器强拖起转，以电压和电流观测反电动势，确认稳定后测磁链。保留编译期采样频率和速度基值。
- [x] P09d，空轴在同一速度范围内制动和加速，用观测速度变化和实测转矩求惯量。
- [x] P09e 软件，统一限流、旋转限速、取消、超时、故障和结果应用语义，覆盖 FLOAT、FIXED。
- [ ] 实机精度、重复性及板上时序留待实机验收，本批未上电。

<a id="p09a-rs"></a>

#### P09a Rs 流程与运行边界（2026-11-18）

11 月 18 日的 20 kHz 基线中，ALIGN 在 6,000 拍内请求 0.04 V/Vdc 的 α 轴调制。Rs 低、高档分别请求 0.04、0.09 V/Vdc，每档等待 4,000 个有效电压帧，再累计 2,000 对 `u[k-1]` 与 `i[k]`。缺少前区间电压时计数保持。低档最后一对样本仍归低档，本拍开始提交高档。高档采满当拍请求中性。ALIGN 采满后的等待拍同样提交中性。每阶段 5 s 超时从首次前台轮询该阶段开始计算，已采满的阶段优先交接。ADC 中断停止时，物理关断依赖仍能运行的前台或板级保护。[采样与求值](../../mfoc/identification/src/mfoc_identification_rs.c) · [阶段与超时](../../mfoc/identification/src/mfoc_identification.c)

FLOAT、FIXED 均通过同实例 1 PU、2 PU 两轮辨识，各轮两档均采满 6,000 帧。Motor 回放另覆盖母线变化、交接中性、取消、超时、采样与 PWM 写入失败。新增用例先显式应用一次成功的 Rs，再用同一实例取消一次并用零电流测到失败。两次 `GetResult` 都拒绝覆盖调用方保存的上次结果，Motor 模型及已应用的 SMO 系数保持原值。辨识内部工作区随新实验复用，上次成功结果由调用方保存。[Rs 采样用例](../../mfoc/tests/identification/identification_rs_test.c) · [Motor 集成回放](../../mfoc/tests/motor/motor_identification_test.c) · [应用后重试](../../mfoc/tests/motor/motor_model_update_test.c)

本批在两个后端复现了三处运行边界。

- ALIGN、Rs 低档和高档在采样前共用同一个实验电流上限。两轴电流绝对值之和等于 0.12 PU 时继续测量，超过时丢弃该拍，标记本轮失败，并在该拍提交三相 50% 占空比以撤掉测量电压。下一次 Motor Run 请求关闭 PWM，`GetResult` 拒绝返回本轮阻值。FLOAT、FIXED 从公开启动入口验证了两档的正负边界、仅超出一个可表示值，以及两轴各自低于上限但合计超限的情况。[采样判断](../../mfoc/identification/src/mfoc_identification.c) · [超限回放](../../mfoc/tests/motor/motor_identification_test.c)
- 已确定保留成功求出的 Rs，由 Motor 单独处理后续关断故障。自动关闭 PWM 返回 I/O 错误时，`Run` 返回该错误，Motor 锁存 PWM 故障并拒绝重启，`GetResult` 仍可读取约 1.2 PU 的新测量，当前模型仍为 0.5 PU。[正常及失败关断回放](../../mfoc/tests/motor/motor_identification_test.c)
- 2026-11-18 的 Rs 直压路径尚未落实 Motor 实例的运行限额。配置 `i_max=0.03`、`speed_max=1.0`，送入 α 电流 0.0625 PU 及位置源速度 1.25 PU 后，两后端均继续驱动，U 相 duty 为 0.53，位置快照读取次数为 0。该电流低于本次落实的 0.12 PU 实验上限。该问题已在 11 月 19 日以实例限流和新的无编码器实验约定处理；静止实验机械固定，旋转实验从电气采样估速。

下表为本地实现与加入两档超限判断前的 G431 指令对比。Cortex-M4F、Clang 19.1.5、release `-Os`，SMO 主反馈，HFI 关闭，辨识开启。使用测试 Port，从 Motor Step 入口计到返回，电流为 `iα=0.0625, iβ=0` PU。表中数字为改前 → 改后的模拟执行指令数，板上周期待测。

| 路径 | FLOAT | FIXED |
| --- | ---: | ---: |
| ALIGN 采样 | 240 → 237 | 397 → 392 |
| Rs 等待稳定 | 229 → 247 | 384 → 400 |
| Rs 窗内累加 | 244 → 262 | 407 → 423 |
| 低档切高档 | 248 → 266 | 413 → 429 |
| 高档采满 | 239 → 257 | 408 → 424 |
| Rs 缺前区间电压 | 223 → 241 | 378 → 394 |
| 等待前台的中性拍 | 185 → 185 | 346 → 346 |

上述路径的栈峰值增量均为 0。Rs 窗内整拍为 176 / 288 B（FLOAT / FIXED，测试 Port）。Motor 实例为 568 / 584 B，其中辨识实例为 96 / 104 B，额外 RAM 为 0 B。变更的机器码位于辨识采样分派函数，普通电流环每拍新增 0 条指令。

### P10 实时预算与 HFI 优化

- [ ] 用目标测量定位热点并满足完整实时链预算。对应 R5、R8。

涉及 `mfoc/position/src/mfoc_hfi.c`、`mfoc/math/src/mfoc_angle.c`、`mfoc/math/include/mfoc_trig.h`、目标 IRQ 与诊断。先分解采样/换算、各启用反馈、Core、提交、诊断开销，记录 HFI 普通拍与每八拍重计算拍、获取/跟踪/极性分支以及限幅/故障路径；四组辨识的 ISR 激励、捕获、阶段切换和失败路径同样纳入。热点定位可与 P09 并行，P10 最终验收必须基于辨识已全部集成的版本。FLOAT+LUT 并不自动替换 atan2；目标外部数学后端沿用直接绑定，优化位置由测量决定。

扩展 HFI 算法与通道测试；新增 `mfoc/tests/scripts/check_realtime_symbols.ps1` 和 `mfoc/docs/performance.md`。任何近似、除法替换、分步运算必须比较角度/速度误差、锁定与失效行为，防止以改变时序语义换取平均速度。

验收采用实际“采样完成至 PWM 最晚写入”的可用窗口，计划保留至少 20% 余量。响应时间包括 IRQ 入口延迟、允许的屏蔽/抢占、计算和比较值写入；不能只量 Step 函数。若窗口确为 50 μs，则完整响应目标不超过 40 μs；若有效窗口更短，按更短窗口计算。冻结允许的中断负载，记录最坏分支分析与测得最大值，测量最大值不能单独当成严格 WCET 证明。生产配置关闭诊断后验收，诊断增量单列；HFI 发布配置不能通过关闭 HFI 来过关。

<a id="p10-g431-current-loop"></a>

#### G431 电流环高频路径候选

2026-11-15 按仓库 Makefile 用 Clang 19.1.5 重编 Cortex-M4F 的 FLOAT、FIXED release 目标文件，核对函数反汇编和调用重定位，另核对 FLOAT debug-rel 的波形调用链。G431 默认 FLOAT，保留 sin/cos LUT。Core 每拍已共用一组 sin/cos，母线倒数由 SetBus 预计算。以下是板测前的候选，调用点和循环次数均来自源码与目标文件，周期收益待测。

| 路径 | 已核实的开销 | 候选与验收 |
| --- | --- | --- |
| 三相采样，PWM 写入前 | 优化前 FLOAT 每个成功帧调用 3 次 `__aeabi_l2f`。MDI 将三相掩码为 14 位，十帧校准均值也在同一范围，有符号差值在 [−16383, 16383]。 | 已完成。两个操作数分别转 `int32_t` 后求差，FIXED 保留 64 位增益乘积。[验收记录](#p10-g431-adc) · [MDI 帧](../../peripheral/stm32g431/mdi/instance.h) |
| FIXED 矢量限幅，PWM 写入前 | 原实现超限后固定做 32 轮整数开方，d、q 缩放各做一次宽除。 | 已跳过开方的前导空轮，保留向上取整和两轴缩放。[Core](../../mfoc/core/src/mfoc_core.c) · [验收记录](#p10-fixed-limit-model) |
| FIXED 模型电压重建，PWM 成功写入后 | 原实现用有符号宽除法计算 α 分量。合法 duty 原始计数为 [0, 32768]，分子 `2du−dv−dw` 在 [−65536, 65536]。 | α 的分子和除 3 已改为 32 位，两个分量的母线缩放保留宽乘。[重建公式](../../mfoc/control/src/mfoc_modulation.c) · [验收记录](#p10-fixed-limit-model) |
| 编码器外推与角度波形 | 优化前，Feedback 和角度波形分别取当前 tick 外推编码器角，接受采集时读两次。 | 已完成同拍复用，关闭波形的发布固件逐字节一致。外推除以实例 `tick_hz` 的优化仍待单独评估。[验收记录](#p10-g431-position-waveform) |
| SMO 角度提取，PWM 写入前 | 有前区间电压且反电势非零时提取一次角度。默认 20 kHz 下每累计 20 个连续角差再求一次相位补偿。FLOAT 使用 `atan2f`，FIXED 的非轴向输入使用 14 轮 CORDIC。 | 分测普通拍与测速窗更新拍，再决定是否优化。若采用近似，回放低速、正反转和接管，核对角度、速度、有效状态及接管时刻。[SMO](../../mfoc/position/src/mfoc_smo.c) · [角度实现](../../mfoc/math/src/mfoc_angle.c) |
| 高频对象的编译选项 | release、debug-rel 的 MFOC 对象当前为 `-Os`，单独追加 `-O2` 的规则匹配 `motor.o`。 | 对实测热点逐个比较 `-Os` / `-O2`，保持 `-ffp-contract=off`，记录前后数值、最坏拍周期、栈峰值与链接后 Flash 差额。[构建规则](../../Makefile) |

分别记录关诊断时从采样完成到最后一个 PWM 比较寄存器写入、以及到 ISR 退出的开销，再按发布后端和最坏分支排后续优化顺序。FIXED 将写入前的限幅峰值与写入后的重建开销分开。改动共享 FIXED 算法时还要复核 M0 的指令数与栈。任何候选若可能增加普通拍、最坏分支或前台的计算量，按第 7 节先报差额、取得确认后再改生产实现。

<a id="p10-g431-adc"></a>

#### G431 三相 ADC 换算验收（2026-11-19）

`mfoc_port_current` 的三相 ADC 与校准偏置改用有符号 32 位求差，沿用已有 14 位范围保证。FLOAT 每相由一条 `vcvt.f32.s32` 转为浮点再乘原增益。FIXED 每相保留有符号 64 位乘积，除以 32768 时仍向零截断。[采样实现](../../peripheral/stm32g431/mfoc/mfoc_port.c)

| G431 后端 | 函数体静态指令 | 成功帧路径指令 | 帧未就绪路径指令 | 未初始化路径指令 | 函数自身最大栈帧 |
| --- | ---: | ---: | ---: | ---: | ---: |
| FLOAT | 62 → 47 | 60 → 45 | 22 → 11 | 17 → 7 | 64 → 16 B |
| FIXED | 84 → 63 | 82 → 61 | 13 → 13 | 9 → 9 | 52 → 24 B |

两版均用 G431 Makefile 的 release 参数、Clang 19.1.5、Cortex-M4F、`-Os -ffp-contract=off`。用 `llvm-objdump -dr --section=.text.mfoc_port_current` 数函数体指令并沿三个分支分别计数，栈帧取 `-fstack-usage` 输出。旧 FLOAT 成功路径的三次转换调用各计一条，未计入被调函数内部指令。完整 ISR 的周期与栈峰值仍待板测。

主机回归用原 64 位求差公式作基准。偏置取两端，三个通道分别遍历全部 32,767 种差值，每组都测正反极性和五档增益（每 PU 对应 1、119、476、16384、1048576 个计数）。FLOAT 按位比较含负零，FIXED 按位比较含超出 32 位的中间乘积，两者均覆盖正负超 1 PU。未初始化或 ADC1 帧未就绪时，三相输出保持原值且帧未被确认；随后首个有效帧正常发布。[端口回归](../../mfoc/tests/port/port_test.c) · [运行脚本](../../mfoc/tests/scripts/run_port_tests.ps1)

FLOAT、FIXED 主机回归和 G431 release 固件构建均通过。两个 release 端口目标文件中，采样函数的指令及重定位与上表所用目标文件一致。[主回归脚本](../../mfoc/tests/scripts/run_tests.ps1)

<a id="p10-fixed-limit-model"></a>

#### FIXED 矢量限幅与上一拍模型电压（2026-11-19）

基线使用 `928e111fff76` 中的 `mfoc_core.c` 和 `mfoc_modulation.c`，其余源码和配置均取同一工作区版本。

PWM 写入前，`mfoc_core_Step` 在电流 PI 电压叠加注入后做圆形限幅。超限时，`_mfoc_core_CeilSqrt` 在模长平方的首个非零 32 位字中，找到不超过该字的最大 4 的幂（1、4、16、…）再开始开方，只在平方超过 32 位时再处理低字的 16 对位。非零余数仍使根向上取整。PWM 写入成功后，Motor 用本次 duty 和母线快照重建下一拍可用的电压。α 的有符号除 3 向零截断，母线缩放继续使用 64 位乘积。[电流环](../../mfoc/core/src/mfoc_core.c) · [PWM 提交和电压发布](../../mfoc/motor/src/mfoc_motor.c)

| FIXED 构建 | Core Step 静态指令 / 自身栈 B | 重建函数静态指令 / 自身栈 B | 普通拍 Core 指令 | 刚超限 Core 指令 | 192 个用例中 Core 最大指令 / 调用链最大栈 B |
| --- | --- | --- | ---: | ---: | --- |
| G431 release `-Os` | 208 → 223 / 80 → 80 | 57 → 49 / 40 → 24 | 601 → 599 | 1373 → 1064 | 2002 → 1923 / 176 → 176 |
| Cortex-M0 `-Os` | 298 → 296 / 96 → 88 | 68 → 62 / 32 → 32 | 1255 → 1255 | 2848 → 2126 | 4288 → 3918 / 216 → 208 |
| Cortex-M0 `-O2` | 540 → 390 / 104 → 88 | 70 → 64 / 32 → 32 | 1257 → 1255 | 2719 → 2126 | 4196 → 3843 / 224 → 208 |

用 Clang 19.1.5、LUT、`-ffp-contract=off` 编译，G431 取 Makefile release 参数，M0 取 Thumb/soft ABI。静态指令来自 `llvm-objdump -dr`，自身栈来自 `-fstack-usage`。Unicorn 对实际链接的函数及整数辅助函数逐条计数，从入口 SP 至最低 SP 取调用链峰值。普通拍为零电流、零命令、零注入；刚超限拍的 Q15 半径为 18022，请求为 (18022, 1)。其余覆盖临界等号、微小半径、极值和变角度的电流/参考/注入。192 组同输入对比中，Core 每组执行指令都持平或减少。完整中断的板上周期和栈峰值待测。

重建函数的六组有效输入中，G431 每次 109 → 48 条，调用链栈 136 → 24 B；M0 每次少 55 条，调用链栈 136 → 48 B。以最大母线和 duty (1, 0, 0) PU 为例，M0 `-Os` 为 313 → 258 条，`-O2` 为 315 → 260 条。M0 `-O2` 的两个空指针返回各多 1 条（9 → 10、11 → 12），Motor ISR 的实参是有效对象地址。G431 的 `mfoc_core_Step` 代码由 610 增至 632 B（开方被内联），重建函数由 174 降至 148 B。两项合计后链接的 FIXED 固件 `text/data/bss` 仍为 27564/296/5136 B。这两处局部增加已确认。

FLOAT、FIXED 各用新旧实现连续回放 65,536 拍，逐拍比较 PI 状态、限幅标记、dq 电压、调制、duty 和重建电压，全部逐位一致。FIXED 有 25,274 个限幅拍，10,274 次退出限幅。独立用例核对完全平方及两侧、最小半径和两轴同时取 INT32_MIN；重建覆盖全部 131,073 种 α 分子与四档母线（含 INT32_MAX）。Motor 回归检查上一拍电压的使用、PWM 提交失败后的历史失效及重启。G431 和 M0 的 FLOAT 两个函数指令与重定位均与原版一致。[限幅边界](../../mfoc/tests/core/core_test.c) · [重建穷举](../../mfoc/tests/math/math_test.c) · [区间回放](../../mfoc/tests/motor/voltage_interval_test.c)

FLOAT、FIXED 主机回归、独立库回归、Arm 目标检查和 G431 两种 release 固件构建均通过。[主回归](../../mfoc/tests/scripts/run_tests.ps1) · [Arm 检查](../../mfoc/tests/scripts/check_arm.ps1)

<a id="p10-g431-position-waveform"></a>

#### G431 编码器角与波形共用同拍读数（2026-11-19）

FLOAT 角度波形使用 Feedback 在当前 ADC 帧调用 `mfoc_port_position` 时取得的 BAM32 电角和有效状态。缓存属于 G431 的采样中断，每个就绪帧结束时作废，波形停采、抽取跳过和缓冲区满也一样。纯对齐、辨识等跳过 Feedback 的拍，只有实际采集时才读编码器，并将缺少同拍来源的观测角和角差标为无效。无 ADC 帧不推进角度波形。SMO 主反馈仍使用 SMO 控制角。[G431 位置入口](../../peripheral/stm32g431/mfoc/mfoc_port.c) · [波形](../../class/mfoc_debug.c) · [采样 IRQ](../../target/stm32g431/mfoc_motor_irq.c)

用 Makefile 和 Clang 19.1.5 `-Os` 构建前后两套 G431 固件。下表用 Cortex-M4 指令模拟从 `mfoc_motor_IRQHandler` 入口执行至返回，“写 PWM”计至最后一次写入 TIM1 比较寄存器（含该指令）。模拟 ADC 就绪和 SysTick 寄存器，执行固件自己的时钟、编码器、SMO、PWM 和波形代码。普通拍为 SMO 有效、测速窗未更新的代表输入。栈为该路径入口 SP 至最低 SP 的差，不含硬件异常入栈和抢占。指令数用于同输入前后比较，板上周期待测。

| G431 配置与路径 | ISR 执行指令，前 → 后 | 至写 PWM，前 → 后 | 栈 B，前 → 后 |
| --- | ---: | ---: | ---: |
| release FLOAT，普通拍 | 939 → 939 | 878 → 878 | 344 → 344 |
| release FIXED，普通拍 | 1552 → 1552 | 1465 → 1465 | 408 → 408 |
| FLOAT 角度波形，普通采集拍 | 1612 → 1495 | 917 → 924 | 344 → 344 |
| 同配置，抽取跳过 | 1053 → 1067 | 917 → 924 | 344 → 344 |
| 同配置，缓冲区满 | 1175 → 1189 | 917 → 924 | 344 → 344 |
| 同配置，运行时停止采集 | 1046 → 1060 | 917 → 924 | 344 → 344 |
| 同配置，对齐采集拍 | 1161 → 1180 | 496 → 496 | 240 → 240 |
| 同配置，无 ADC 帧且原波形到采集拍 | 656 → 95 | 无写入 | 224 → 48 |

G431 默认以 20 kHz 运行电流环、10 kHz 采集波形。上述普通输入连续两拍合计少 103 条指令，写 PWM 前每拍多 7 条。FLOAT 角度波形固件增加 96 B Flash、8 B 静态 RAM。这些调试开销已确认接受。编译关闭波形时，FLOAT、FIXED 的 release 固件各自与改前的 `.bin` 逐字节相同。另比较了三个 debug-rel 配置，FIXED、FLOAT 只采 HFI Di/Du、FLOAT 关闭物理位置反馈，前后固件也分别逐字节相同。

主机回归用真实 G431 Port、Encoder、Feedback 与波形回调覆盖了正转、反转、超时边界与恢复：读时恰好到超时阈值仍有效，波形晚一个 tick 依然使用原角；读时已过期，即使采集前发布了新位置，本拍仍无效。跳采后进入对齐只读一次。G431 默认 SMO 固件回放中，控制时刻编码器为 0.438900 电圈，晚读为 0.439600 电圈，SMO 角为 0；改后波形显示 0.438900，角差为 −0.438900 电圈。FLOAT、FIXED 主机、波形配置、独立库、Flux/EKF 和 Arm 目标检查通过。[G431 用例](../../mfoc/tests/port/port_test.c) · [波形用例](../../mfoc/tests/debug/debug_waveform_test.c)

<a id="p10-g431-bench"></a>

#### G431 台架耗时与观测器运行数据

2026-11-19，操作者反馈了以下 G431 台架读数。启动列为观察到的最高值，运行列为反馈的运行读数。SMO 使用上一轮 FLOAT 调试候选固件。

| 观测器 | 启动最高 | 运行 |
| --- | ---: | ---: |
| Flux | 约 25 µs | 22.7 µs |
| SMO | 29.4 µs | 26.6 µs |

本轮记录未附原始轨迹、计时起止点或同条件的改前读数。生产配置从采样完成到最后写 PWM 的余量仍需单独测量。

核对了以下几组每拍写入的数据。

- SMO 每拍先清空输出，有反电势时又写入全部字段。试编了仅在两轴反电势同时为零时清空输出的写法。按 G431 Clang `-Os` 反汇编，FLOAT、FIXED 的普通拍和测速窗更新拍各少执行 4 条指令，零反电势拍持平。`mfoc_smo_Step` 的局部栈均为 32 B。相同改法在 Cortex-M0 FIXED 上使函数体从 412 B 增至 420 B，局部栈均为 56 B。已确认保留原实现，候选仅作记录。输出里的 `bemf` 占 4 B，是由两轴反电势计算的公开诊断值，当前板级控制和波形均未读取。[SMO](../../mfoc/position/src/mfoc_smo.c) · [输出用例](../../mfoc/tests/position/smo_test.c)
- Flux 输出角对应当前采样，实例中的 PLL 角已推进到下一拍。也试编了仅在零磁链时清空输出的写法，FIXED 的 `mfoc_flux_Step` 静态函数体从 523 条增至 535 条，故保留现有实现。[Flux](../../mfoc/position/src/mfoc_flux.c)
- Motor 的 `previous_core_output` 保存整份 Core 遥测，G431 的 FLOAT、FIXED 均占 44 B。板上波形只从中取 4 B 的 Iq，库用例还读取 dq 电压、占空比和限幅状态。`previous_voltage_ab` 记录成功提交 PWM 后重建的上一拍电压，辨识直压模式也会更新，此时 Core 遥测无效。两者有各自的有效标志。[Motor](../../mfoc/motor/include/mfoc_motor.h) · [调试读取](../../class/mfoc_debug.c) · [区间电压用例](../../mfoc/tests/motor/voltage_interval_test.c)

### P11 独立构建与系统回归

- [ ] 所有发布组合在独立消费环境中通过。对应 R1、R2、R4、R5、R8。

修改 `mfoc/mfoc.mk`、测试脚本与 `mfoc/mfoc.h` 的公共入口；新增最小 `mfoc/examples/host_replay/`、芯片无关示例配置和 `mfoc/tests/scripts/check_standalone.ps1`。整理可选算法源码与配置，使用 C11 基础及必要的明示编译器扩展；库无堆分配要求，静态表允许存在，各编译单元必须共享一致裁剪配置。

导出一个只含库、公共配置、示例和必要许可证信息的临时目录；主机示例提供 fake/replay Port，实现声明的物理量与输出接口。在无 `../modus`、无厂商目录、无原工程应用对象的环境编译、完整链接并运行，不能以原仓库 include 路径兜底。允许使用有明确清单的标准 C 内存/数学函数及编译器运行库，独立链接示例须提供对应设置。原工程 Make 集成与独立消费共用同一源码。是否增加 CMake 不作为发布必要条件，提供可靠源码清单和可运行构建示例即可。

扩展 `mfoc/tests/scripts/run_tests.ps1`、`check_arm.ps1`；主入口必须执行全部公布算法的测试，包含现有独立 Flux/EKF 脚本及启动交接回归。测试矩阵包括主机 FLOAT/FIXED、Cortex-M0 FIXED 完整链接、Cortex-M4F FLOAT/FIXED、EKF FIXED 拒绝、各公布 backend、HFI/辨识开关。检查软浮点热路径、未定义符号、隐藏全局、资源用量及两实例差异配置回放。

新增 `mfoc/tests/motor/per_unit_invariance_test.c`：对同一物理电机、激励与采样轨迹，使用两套合法 PU 基值运行采样、控制、观测和四组辨识，换回物理量后满足预设容限；验证各实例频率系数独立生成。分别测试基值缩放等价性和不同采样率下的模型误差，不能要求不同采样率逐拍完全一致。M0 编译/链接通过不等于该芯片已达到 20 kHz。

### P12 第二芯片与最终板上验收

- [ ] 在第二款芯片实施新适配，并完成最终发布配置的实机矩阵。对应 R2、R3、R5、R7、R8。

在 P00 选定的 `peripheral/<chip>/mfoc/` 与 `target/<chip>/` 实现板级接线和配置；路径占位在 P00 换成实际目标。新增该目标的 Port 契约测试及 `mfoc/docs/validation/` 下记录。只为适配需要修改算法文件意味着边界未通过，应回到对应工作包解决，不把芯片判断加入算法。

第二芯片至少验证 FIXED 基础闭环、启停/故障、采样/PWM 区间以及一种有效反馈路径；每款板明确公布可运行功能与频率。G431 参考板完成首版全部必做功能，包括 HFI 与四组辨识。算法级 FLOAT/FIXED 回归覆盖四组辨识；发布矩阵中额外声称实机支持的组合必须补对应板上记录。

验收包含实际控制周期、资源、温升/负载试验条件、方向/极性、启动重复性、观测器交接、异常关断和长时间运行；量化目标使用 P00 预先冻结值。保留 MCU/板型、时钟、工具链、优化选项、库版本、配置、波形/计时和参考测量。第二板未完成时 G6 保持未通过。

### P13 发布交付

- [ ] 生成并复核版本包和全部接入资料。对应 R1—R8。

更新 `mfoc/README.md`、`mfoc/docs/framework.md`、Port/数值/支持矩阵文档；新增 `mfoc/docs/getting-started.md`、`mfoc/docs/extension-guide.md`、`mfoc/CHANGELOG.md`、`mfoc/docs/release-v1-checklist.md`。给出初始化、校准、周期接线、命令提交、停机、错误处理、参数应用和增加算法的最小示例，记录旧 API 的迁移方式。

明确版本标识、源码分发布局、公共 API 稳定范围、编译配置一致性、许可证与第三方来源。许可证选择和对外发布渠道由项目所有者在本工作包确定，不能由工具擅自选定；这是对外交付信息，不能缺失后仍称发布完成。无需为首版重建仓库或设计动态二进制插件 ABI。

从待发布包重新执行 P11 的独立消费验证，核对文档、头文件、支持矩阵及实测版本一致。发布候选准备完成后再执行实际版本标记与分发；本文不代表这些动作已经执行。

## 6. 发布门禁

| 门禁 | 必须提供的证据 | 阻止发布的情况 |
| --- | --- | --- |
| G1 独立性 | 导出包独立编译、链接、运行；依赖与符号检查记录 | 仍需相邻 MODUS/perfc/应用对象，或只完成逐文件编译 |
| G2 隔离性 | 两实例不同配置交错运行；状态与调用者所有权审计 | 核心隐藏可变全局、实例串扰、通过直接改子对象成员完成生命周期 |
| G3 Port 与周期契约 | PU/duty 换算测试；采样/前区间电压/PWM 更新对应图与板上测量 | 单位/相序不一致、前拍模型与实际区间错位、输出失败仍继续正常控制 |
| G4 数值 | 后端支持表、范围/舍入文档、小信号与饱和回归、误差证据 | 只保证 FLOAT 正常；FIXED 静默改变单位/范围；未知热路径软浮点 |
| G5 功能 | 已验证功能迁移回归与四组辨识完整证据 | 以 TODO/占位/不可启动流程宣称支持；丢失原有启动或保护行为 |
| G6 移植 | G431 与第二芯片接入记录、平台功能/频率表 | 换芯片需修改算法；第二板只有 fake 或编译记录 |
| G7 实时性 | 实际截止窗口、最坏分支与最大耗时、资源余量 | HFI 周期峰值越界；只报均值或只报算法局部耗时 |
| G8 可交付性 | 文档、示例、许可证、版本、变更记录、发布包复现 | 文档支持范围超过证据，或消费者无法按步骤接入 |

状态只允许“未开始 / 进行中 / 待验收 / 完成”。每个工作包完成时填写实现版本、验证配置、测试/实机证据与剩余限制。发布必做项不存在默认豁免；若后续调整范围，应修改范围和支持矩阵并重新审查，不能把未通过项直接勾选完成。

## 7. 执行顺序与变更控制

**计算量与变更确认。** 结构整理以改善可读性和维护性为目标。凡改动可能增加目标端运行时计算量，先向项目负责人说明模块、后端、调用频率、收益及代价，取得明确确认后再改生产实现。对 FOC 控制中断，按相同目标与优化配置比较改前和候选实现，区分普通拍、限幅拍或观测器周期峰值；有实测周期时报告周期，只有指令、代码大小或栈数据时注明测量范围。初始化、前台操作和每拍调用分别核算。后续按模块优化时沿用同一基线。

模块校验按数据归属核对。对同一份值，若所有入口已经建立同一前提，且到消费点之间数值和状态都不会改变，就合并重复判断，并验证原返回码、输出及失效恢复。可独立调用的公开接口、异步更新后的数据、物理保护和观测器收敛各有不同的失败条件；要改变这些行为时，先说明受影响的调用者并取得确认。

第一阶段完成 P00/P01，先冻结物理量、PU 口径、Port 操作和固定周期关系；第二阶段完成 P02—P05，把同一已验证算法迁到新骨架并恢复 G431 基线；第三阶段完成 P06—P10，解决数值、控制权、参数应用、辨识和性能；第四阶段完成 P11—P13，验证独立交付、第二芯片和最终发布包。

每次改动以一个可验证边界为单位。接口整理保持算法递推顺序，数值精度改善和算法性能优化分别记录行为差异。测试失败先判断契约失配还是实现错误，不删除有效验证来适配实现。只有相关实现或配置变化才重跑受影响的板上试验，最终发布候选完成完整门禁复核。

实施前必须确定的事项已归入 P00/P01：第二板型号、实际 PWM 截止窗口、性能生产配置、算法和辨识验收阈值、Port 必需/可选物理量及其单位和周期关系。实现中才能得出的事项包括热点归因、具体数学优化和最终 RAM/栈占用，由 P06/P10/P11 产生证据。它们是有交付节点的任务，不是可以带进发布包的未定项。

所有实机动作按项目调试规范执行，台架上电、烧录及运行验证与主机测试区分记录。本轮只维护发布计划，不更改控制代码、现有工作区配置或已验证电机行为。


## 2026-11-10 Port 实施记录

- 已实施：显式共同基值，Motor_Init 继续调用 Port；PWM 更新/启动/停止拆开；统一电角度/速度接口；板级持有 Encoder；Motor_Run 先服务位置再推进辨识/对齐；64 位 tick，Motor/辨识的超时改用 Port 时钟；Feedback 保持平铺输出。
- 保持原算法：Core、PI、SMO、Flux、HFI、SensorlessStart 和辨识叶子未改递推。Encoder 保留滤波、速度换算、外推，只拆出硬件采集和采样节拍，并防止长于 32 位周期的停顿被截短。HFI 仍使用编译期速度基值，Motor_Init 在启用 HFI 时拒绝不一致配置。
- G431 构建的数值后端由顶层 MFOC_NUMERIC 唯一选择；具体测试、既有失败及构建结果以验证文档为准。
- 后续仍需完成：辨识 PT 公共类型的平台依赖移除、独立库打包、未实现辨识调度、第二芯片和台架性能/时序验收。当前单功率级静态绑定，不宣称支持同时绑定多个硬件电机。
