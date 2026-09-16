# FOC Class/Driver 分层重构设计

**日期：** 2026-09-16

## 目标

按照更新后的 Class/Driver 框架重构 `foc_app`、`motor`、`encoder` 和可选的
`identify`，但不改变已经验证的编码器、速度/电流控制、对齐、故障和识别行为。

本轮范围已经确认：

- `foc_app` 按 `class/template_class` 作为 MODUS Class 重构。
- `motor` 按 `class/template_driver` 的对象所有权和依赖注入方式重构。
- `encoder` 成为由 App Class 按值拥有、并按照 `template_driver` 模板实现的独立
  FOC Driver。
- `identify` 也按照 `class/template_driver` 整理为显式算法 Driver；
  `class/foc_identify_minimal.*` 只作结构参考。
- `template_observer.*` 仅保留为后续扩展接口。
- `foc_smo.*` 不修改、不接入。
- 本轮只验证 STM32G431，AT32F413 暂不纳入。

## 对象所有权

`foc_app_t` 是唯一注册到 MODUS 的 Class 对象，按值拥有 Motor、Encoder 以及
启用时的 Identify 实例。App 负责组合策略、调度、Shell、波形和诊断。

Motor 负责生命周期、命令、PI、ADC 校准、电流输入、电气零位、故障和 PWM
状态。`motor_cfg_t` 只保存初始化输入和注入依赖。

Encoder 负责原始传感器绑定、双缓冲机械位置、速度滤波、样本年龄和传感器错误。
App 在前台调用慢速传感器更新，Motor 在高频路径只读取 Encoder 已发布的快速位置。

Identify 负责自身状态机、输入输出快照、拟合累加器、试验状态和诊断，不拥有硬件，
也不调用 ADC、PWM、Encoder 或板级函数。

Motor、Encoder 和 Identify 都按照 `template_driver` 的生命周期、对象所有权、配置
绑定、状态快照和 `ops/context` 规则实现各自的领域 Driver；不在具体对象内再次嵌套
通用 `template_driver_t`，避免重复状态和实时路径上的二次分发。Encoder 明确区分
前台 Run/Update、快速缓存读取、Stop/Reset 和状态查询；Identify 选择高频 IsrStep
加前台状态查询/终态处理的能力档位，不强行增加通用 PT Run。

## 依赖边界

`motor_cfg_t` 注入 ADC、PWM 和位置三类语义化 `ops/context`。Motor 直接调用已绑定
的操作。`motor.c` 不包含 AS5600、MCU、Vendor HAL，也不调用旧的全局
`foc_adc_*`/`foc_pwm_*` 接口。

`foc/hal/foc_port.h` 收缩为 ADC/PWM 语义接口。STM32G431 的
`peripheral/stm32g431/foc_port.c` 实现并绑定这些接口。break 锁存和电流缩放等板级
单例状态必须通过显式 board context 绑定，不能作为 Motor 的隐式全局行为。

AS5600 仍属于 `peripheral/driver` 的物理外设 Driver；FOC Encoder 本轮继续放在
`foc/observer`，但对象角色和接口按 Driver 实现，避免无关目录迁移。

## 调用和调度模型

高频实时路径限定为：

```text
ADC ISR -> foc_app_HighFrequencyISR -> motor_IsrStep
         -> 当前状态处理 -> 直接调用 ADC/position/Core/PWM 操作
```

当前状态处理顺序保持为：

```text
ADC 采样 -> 位置缓存读取 -> 现有 observer
          -> speed 分频/PI -> foc_core_step -> PWM duty 提交
```

初始化失败、Motor 停止和运行时故障都由 Motor 调用绑定的 PWM stop 操作；第一安全
动作仍是关闭 PWM。App 不直接执行硬件清理。

前台调度保持分离：

```text
  MODUS Run   -> foc_app_Run   -> Encoder 慢速 Run
MODUS Clock -> foc_app_Clock -> 短周期服务/Identify 前台处理/诊断
```

## 兼容性和明确不做的内容

如新所有权模型确实需要，允许调整公开名称；但行为是兼容性标准。现有测试中的数值
结果、状态迁移、命令行为和故障顺序必须在更新 fake binding 后保持不变。

明确不做：

- 修改或接入 SMO、`template_observer`。
- 运行时 observer 选择或全局 Driver 注册表。
- 修改识别激励算法和测量模型。
- 自动识别持久化、Flash 存储和 AS5600 协议改造。
- 多电机 Manager。
- AT32F413 适配和目标构建。

## 必须验证的风险

- STM32G431 ADC 通道映射、偏置极性、电流缩放和校准触发行为不变。
- break 锁存的 ISR/前台访问仍然安全。
- Motor 绑定依赖后初始化失败时自行执行安全停机。
- Encoder 双缓冲在前台写入和 ISR 读取时保持一致性。
- Float 和 Fixed 后端的可观察行为保持一致。
- 工作区现有 Identify 诊断改动在 API 重组后完整保留。
