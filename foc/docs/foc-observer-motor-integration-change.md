# FOC Observer 内聚到 Motor 的变更设计与手动迁移说明

> 本文对应 2026-09-15 工作树中的 Observer 所有权迁移。本文只描述设计和迁移步骤，不直接修改控制算法实现。

## 1. 目标

将位置估算器从 App 级组合对象收敛为 Motor 的内部控制部件。Motor 自己拥有、初始化、复位和调度 Observer；App 只负责提供配置并调用 Motor 的生命周期接口。

最终需要支持以下编译期组合：

| 组合 | 典型用途 | 运行策略 |
| --- | --- | --- |
| Encoder | 有位置传感器 | 编码器角度和速度直接进入 Core |
| SMO | 无感中高速 | 对齐后强拖/开环升速，SMO 锁定后接管 |
| HFI | 预留 | 只保留 `FOC_ENABLE_HFI` 宏，不实现注入、解调或反馈 |
| HFI + SMO | 后续扩展 | 本次不实现 HFI，也不实现 HFI/SMO 融合 |
| Encoder + Observer | 调试或验证 | 编码器闭环，Observer 旁路运行并输出质量信息 |

这里的“注册 Observer”不是运行时插件注册表，而是由编译宏确定 Motor 内部包含哪些算法，再由统一 Observer 接口调用。这样可以避免 App 了解 SMO/HFI 的内部结构，也避免在高频路径上做不必要的运行时遍历。

## 2. 本次迁移后的状态

当前迁移完成后，结构边界应为：

- `foc/app/foc_app.h` 不再包含 `foc_app_t.tObserver` 或重复的 Observer 配置；
- `motor_cfg_t` 保存 `foc_observer_cfg_t tObserverCfg`；
- `motor_t` 保存内嵌的 `foc_observer_t tObserver`；
- `motor_Init()` 是 Observer 的唯一初始化入口；
- `motor_RunStep()` 通过 `foc_observer_Step()` 运行 SMO 验证路径。

本次不包含以下行为：

- SMO 接管 Core 的角度和速度；
- HFI 注入、解调和角度估算；
- HFI/SMO 融合以及无感失锁回退；
- 完整的强拖开环启动器。

因此，本次所有权迁移已经完成；无感启动、反馈接管和融合属于后续阶段，不能与本次迁移混为一谈。

## 3. 最终对象边界

### 3.1 App 的职责

App 只做三件事：

1. 构造 `motor_cfg_t`；
2. 调用 `motor_Init()`、`motor_Start()` 和 `motor_HighFrequencyStep()`；
3. 如需读取 Observer 诊断结果，后续通过 Motor 的诊断接口暴露；App 不直接访问 Observer 对象。

App 不应再：

- 直接调用 `foc_observer_Init()`；
- 直接调用 `foc_smo_Step()` 或 `foc_hfi_Step()`；
- 直接访问 `motor.tObserver` 的算法状态；
- 决定本拍使用编码器、HFI 还是 SMO。

### 3.2 Motor 的职责

Motor 拥有完整的控制上下文：

```text
motor_t
 ├─ Core
 ├─ 电流环和速度环
 ├─ Encoder 位置读取回调
 ├─ Observer 状态（当前为 SMO；HFI 仅预留）
 ├─ 当前反馈源和融合状态（后续无感接管时增加）
 └─ 启动、强拖、接管、降级和故障状态（后续无感启动时增加）
```

Motor 负责保证同一个高频周期内的以下数据来自同一个反馈快照：

```text
电角度 → Park/反 Park
电速度 → 速度环
角度质量 → 是否允许接管或继续运行
```

不能由上层分别调用一次 HFI、一次 SMO，再把结果拼接给 Core；这样容易造成角度和速度不一致。

## 4. 建议的配置和宏模型

### 4.1 宏只选择编译期能力

建议保留能力宏：

```c
#define FOC_ENABLE_ENCODER          1
#define FOC_ENABLE_SMO              1
#define FOC_ENABLE_HFI              0
```

再用一个编译期 Profile 选择 Motor 内部的组合：

```c
#define FOC_FEEDBACK_PROFILE_ENCODER       0
#define FOC_FEEDBACK_PROFILE_SMO           1
#define FOC_FEEDBACK_PROFILE_HFI_SMO       2
#define FOC_FEEDBACK_PROFILE_ENCODER_SMO   3

#ifndef FOC_MOTOR_FEEDBACK_PROFILE
#define FOC_MOTOR_FEEDBACK_PROFILE FOC_FEEDBACK_PROFILE_ENCODER
#endif
```

实际工程中只保留一种默认 Profile，测试时用编译参数切换。不要为每个组合再创建一组互相独立的 `FOC_ENABLE_xxx_AND_yyy` 宏，否则宏组合会快速失控。

### 4.2 配置归属

Observer 的配置应属于 `motor_cfg_t`，而不是 `foc_app_cfg_t`：

```c
typedef struct {
    motor_params_t tParams;
    motor_limits_t tLimits;
    foc_scalar_t qElectricalSpeedBaseTurnsPerSecond;
    motor_get_position_fn fnGetPosition;
    void *pPositionContext;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    foc_observer_cfg_t tObserverCfg;
#endif
    motor_control_cfg_t tControl;
} motor_cfg_t;
```

HFI 当前不增加配置结构、状态成员或 Step 函数。未来开始实现 HFI 时，再在明确的 HFI 设计中增加对应配置和状态，不在本次迁移中放置空的占位字段。

如果未来 HFI 和 SMO 的配置完全不同，`foc_observer_cfg_t` 内部可以按宏包含：

```c
typedef struct {
#if FOC_ENABLE_SMO
    foc_smo_cfg_t tSmo;
#endif
#if FOC_ENABLE_HFI
    foc_hfi_cfg_t tHfi;
#endif
} foc_observer_cfg_t;
```

`foc_app_cfg_t` 只保留 `motor_cfg_t` 和编码器硬件配置，App 不再单独保存 Observer 配置。

### 4.3 Motor 内部状态

```c
typedef enum {
    MOTOR_FEEDBACK_ENCODER = 0,
    MOTOR_FEEDBACK_HFI,
    MOTOR_FEEDBACK_SMO,
    MOTOR_FEEDBACK_BLEND,
    MOTOR_FEEDBACK_INVALID,
} motor_feedback_source_e;

typedef struct {
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeedPu;
    foc_scalar_t qQuality;
    bool bValid;
} motor_feedback_t;
```

`motor_t` 至少需要保存：

- 内嵌 `foc_observer_t tObserver`；
- 当前 `motor_feedback_source_e`；
- 当前有效反馈快照；
- 融合权重或融合进度；
- SMO/HFI 接管计数和失锁计数。

这些字段属于控制状态，不应该放进 App。

## 5. Observer 接口收敛

旧接口的回调参数是 `foc_smo_t *`，这会把 SMO 类型泄漏到 Motor。本次删除 `fnSelectedStep`，改为由编译期 `FOC_OBSERVER_BACKEND` 在 Observer 内部选择实现：

```c
#define FOC_OBSERVER_BACKEND_NONE 0
#define FOC_OBSERVER_BACKEND_SMO  1
```

Motor 只调用统一接口，不知道当前后端是 SMO、磁链观测器还是未来的 EKF：

```c
foc_result_t foc_observer_Step(
    foc_observer_t *ptObserver,
    const foc_observer_input_t *ptInput);
```

输入使用结构体，避免未来增加母线电压、实际施加电压或注入信息时再次修改函数签名：

```c
typedef struct {
    const foc_ab_t *ptCurrentAlphaBeta;
    const foc_ab_t *ptVoltageModelAlphaBeta;
    const foc_ab_t *ptVoltageAppliedAlphaBeta;
    foc_scalar_t qDcBusVoltagePu;
    bool bVoltageAppliedValid;
    bool bDcBusVoltageValid;
} foc_observer_input_t;
```

其中：

- `ptVoltageModelAlphaBeta` 用于 SMO、磁链或 EKF 的基础电机模型；
- `ptVoltageAppliedAlphaBeta` 预留给未来实际施加电压和 HFI 注入分量；
- `qDcBusVoltagePu` 预留给未来基于 PWM/母线电压重构实际电压；
- 两个 `Valid` 字段区分当前是否真的提供了实际电压和母线电压；
- 当前 SMO 只校验和读取电流与模型电压，其他字段不参与运算。

## 6. Motor 初始化和运行时序

### 6.1 初始化

将 Observer 初始化从 App 移到 `motor_Init()`：

```text
motor_Init()
  ├─ 保存 motor_cfg_t
  ├─ 初始化 Core/PID
  ├─ 初始化 Observer/HFI/SMO
  ├─ 清空反馈切换状态
  └─ 开始 ADC 校准
```

如果某个 Profile 没有 Observer，相关代码由宏裁掉；不能在运行时通过空指针模拟不存在的算法。

### 6.2 高频运行顺序

目标顺序如下：

```text
1. ADC 采样三相电流
2. Clarke 得到 Iαβ
3. Observer 读取 Iαβ 和上一拍 Vmodelαβ
4. HFI 执行注入/解调（若启用）
5. Motor 根据当前状态选择或融合角度、速度
6. 速度环使用选中的速度，生成 Iq ref
7. Core 使用选中的电角度执行 Park 和电流环
8. 反 Park 得到本拍 Vmodelαβ
9. 生成 PWM
```

SMO 必须使用上一拍的模型电压，因为本拍 Core 的电压要到步骤 8 才生成。当前 `tCore.tVoltageAlphaBeta` 的“上一拍输入”语义应保留。

如果 HFI 修改了实际电压指令，必须保留独立的 `Vmodelαβ` 和 `Vappliedαβ`。SMO 模型不能直接把高频注入分量当作普通电机模型电压，否则会把注入扰动解释成反电动势。

## 7. SMO、HFI 和强拖接管策略

### 7.1 SMO 的适用区间

SMO 依赖反电动势，适合中高速。启动时不能把 SMO 的输出直接当作有效角度。SMO 路径至少需要：

```text
转子对齐
  ↓
强拖/开环角度推进
  ↓
达到最低电气速度
  ↓
等待 SMO 的反电动势、相位误差和速度质量满足条件
  ↓
进入融合
  ↓
SMO 接管
```

“强拖”是启动策略，不应混入 `foc_smo.c` 的数学模型。SMO 只负责估算；强拖的角度发生器、速度斜坡和接管状态属于 Motor。

### 7.2 HFI 的预留边界

本次只在 `foc_config.h` 中保留 `FOC_ENABLE_HFI` 宏，默认值为零。Motor 不实例化 HFI，不调用 HFI API，也不改变当前 Core 电压命令。未来实现 HFI 时，需要处理：

- 注入电压/电流幅值限制；
- 解调滤波和相位延迟；
- 角度质量门限；
- 初始极性或 180 度歧义；
- 注入对电流环和 SMO 模型的干扰。

HFI 同样不应直接改变 Core 的基础电压模型。注入量应由 Motor/调制层明确叠加，并向 Observer 提供分离后的模型电压。

### 7.3 HFI + SMO 的后续方向

HFI/SMO 的三段式接管仅作为后续方向，本次不实现：

```text
低速区：       未来由 HFI 负责
过渡区：       未来进行 HFI/SMO 融合
中高速区：     未来由 SMO 负责
```

未来 SMO 接管时，建议同时满足：

- 反电动势幅值达到最小值；
- SMO 相位误差小于上限；
- 电气速度处于允许范围；
- 连续满足若干采样周期；
- HFI 与 SMO 的角度差已完成极性/相位校正。

未来融合实现后，SMO 失锁时反向处理：

```text
SMO 质量下降
  ↓
短暂保持/降低融合权重
  ↓
回退 HFI（若仍在 HFI 能力范围）
  ↓
否则进入安全停止或重新强拖
```

角度融合必须按最短角度差进行，不能直接对 BAM32 数值做普通加权：

```text
error = wrap(theta_smo - theta_hfi)
theta = theta_hfi + alpha * error
```

`alpha` 应以采样周期为单位渐变，避免接管瞬间造成 Park 角度跳变。速度也要使用同一套接管状态进行渐变或一致选择。

## 8. 手动迁移步骤

### 步骤 1：收口 Motor 配置

修改 `foc/motor/motor.h`：

- 保留 `motor_cfg_t.tObserverCfg`；
- 删除 `ptObserver`；
- 将未来 HFI 配置放入同一个 Observer 配置或 Motor 的无感配置中；
- 保证 Observer 配置只在对应能力宏启用时出现。

修改 `foc/app/foc_app.h`：

- 保留已完成的 `tObserver` 删除；
- 从 `foc_app_cfg_t` 删除重复的 `tObserverCfg`；
- 把 Observer 配置迁移到 `tMotorCfg.tObserverCfg`。

### 步骤 2：收口 App 初始化

修改 `foc/app/foc_app.c`：

- 删除 `ptMotorConfig->ptObserver = &ptApp->tObserver`；
- 删除 `ptApp->tObserver` 的初始化变量和 `foc_observer_Init()` 调用；
- 不再让 `foc_app_BindMotorConfig()` 绑定 Observer 指针；
- 把原 `tObserverCfg` 初始化内容放进默认 `tMotorCfg`；
- `motor_Init()` 成为唯一的 Observer 初始化入口。

### 步骤 3：在 Motor 初始化内建立对象

修改 `foc/motor/motor.c`：

```text
motor_Init()
  → foc_core_Reset()
  → foc_observer_Init(&ptMotor->tObserver,
                      &ptMotor->tCfg.tParams,
                      &ptMotor->tCfg.tObserverCfg)
  → foc_observer_Reset()
```

初始化失败必须关闭 PWM 并返回错误。不能让 Motor 在 Observer 未初始化时进入 RUNNING。

将所有：

```c
ptMotor->tCfg.ptObserver
```

替换为 Motor 自身的内嵌对象或统一的内部辅助函数。复位、故障、停止、重新启动和 ALIGN 完成时都要复位同一个内嵌 Observer。

### 步骤 4：拆分输入读取和反馈选择

当前 `motor_ReadInput()` 将 ADC、Clarke、编码器读取和 Core 输入构造绑定在一起。需要拆成三个逻辑阶段：

```text
motor_ReadCurrent()
motor_UpdateObservers()
motor_SelectFeedback()
```

Encoder Profile 继续要求编码器有效；SMO/HFI Profile 不得因为编码器回调为空而在 `motor_Start()` 或 `motor_ReadInput()` 中失败。

### 步骤 5：保留后续启动和接管边界

本次不加入强拖状态机和反馈源切换。保留 Motor 是未来强拖、SMO 接管和 HFI 融合的唯一编排位置，当前 `MOTOR_STATE_RUNNING` 仍使用现有编码器 Core 反馈。

未来实现无感接管时至少需要覆盖：

```text
ALIGN
FORCED_START
HFI_ACTIVE
OBSERVER_QUALIFYING
BLENDING
SMO_ACTIVE
FEEDBACK_LOST
```

状态切换必须有连续采样计数、超时和回退路径，不能只判断一次 `bValid`。

### 步骤 6：更新测试

迁移现有测试中的外部 Observer：

- 不再创建独立的 `foc_observer_t tObserver` 传入 Motor；
- 将配置放进 `motor_cfg_t.tObserverCfg`；
- 通过 `tMotor.tObserver` 或公开的诊断读取接口验证状态；
- 删除 `ptObserver` 字段断言；
- 保留独立的 `foc_smo_test`，继续验证 SMO 数学模块本身。

新增测试应覆盖：

  1. Motor 初始化时 Observer 成功初始化；
  2. Motor Stop、Fault、ClearFault 和 ALIGN 后 Observer 被复位；
  3. Encoder Profile 的角度/速度来源不变；
  4. Motor 高频路径能够调用 SMO 并更新其内部状态；
  5. `FOC_OBSERVER_BACKEND` 非 SMO 时不会调用 SMO；
  6. HFI 宏打开或关闭都不会引入未实现的 HFI 符号；
  7. FLOAT 和 FIXED 两种数值后端都能编译；
  8. 各 Profile 未启用的算法不会进入最终对象或高频路径。

## 9. 推荐的迁移顺序

```text
第一阶段：所有权收口
  App 删除 Observer
  Motor 初始化和复位 Observer
  清除旧指针引用

第二阶段：Observer 接口收口
  Motor 不再直接看到 foc_smo_t
  引入统一 observer input/output

第三阶段：SMO 接入反馈
  拆分电流读取和位置选择
  增加强拖、资格判断和 SMO 接管

第四阶段：HFI 后续设计
  单独设计注入、解调和 HFI 状态
  再决定是否与 SMO 融合
```

每个阶段都应能单独编译和测试。不要在所有权迁移尚未收口时同时加入 HFI，否则旧的 App/Motor 边界问题会和无感接管问题混在一起。

## 10. 验收标准

完成后应满足：

- 全工程不存在 `ptObserver` 旧字段引用；
- `foc_app_t` 不包含 `foc_observer_t`；
- `motor_t` 是 Observer 的唯一拥有者；
- `foc_observer_Init()` 只从 `motor_Init()` 调用；
- App 不调用任何 SMO/HFI Step 函数；
- Motor 高频路径能按 `FOC_OBSERVER_BACKEND` 选择当前已实现的 Observer 后端；
- 当前 Core 仍使用编码器反馈，SMO 只用于 Motor 内实例化验证；
- HFI 只保留 `FOC_ENABLE_HFI` 宏，没有 HFI 对象或算法调用；
- 强拖、无感接管和 Observer 失锁回退作为后续实现边界；
- FLOAT/FIXED 测试入口已保留；当前环境缺少 GCC，因此尚未执行编译测试。新增 FIXED 初始化源码契约检查已通过。
