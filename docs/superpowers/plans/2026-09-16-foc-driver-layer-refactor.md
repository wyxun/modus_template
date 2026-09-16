# FOC Class/Driver 分层重构实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标：** 按新的 Class/Driver 依赖注入框架重构 `foc_app`、`motor`、`encoder` 以及
可选的 `identify`，同时保持已经验证的控制功能不变。

**架构：** `foc_app_t` 继续作为 MODUS Class，按值拥有 Motor、Encoder 和可选的
Identify 实例。Motor、Encoder 和 Identify 都按照 `template_driver` 的对象模型实现，
并使用领域明确的 `ops + context`；Motor 在浅层高频路径中直接调用 ADC/PWM/position
操作，App 只负责装配和调度。

**技术栈：** C11/GNU11、MODUS、perfc、现有 Float/Q16.15 FOC 后端、GCC 主机测试、
`make.bat` 目标构建。本轮只验证 STM32G431，AT32F413 暂不处理。

---

## 执行前提

当前工作区包含模板、App、Identify、构建文件和测试数据的用户改动。必须保留这些
改动以及其他无关改动；不得 reset、checkout、stage、commit 或 flash。

### 任务 1：建立当前行为基线

**文件：** 只读以下文件，不创建实现文件。

- `class/template_class.h`
- `class/template_class.c`
- `class/template_driver.h`
- `class/template_driver.c`
- `foc/app/foc_app.h`
- `foc/app/foc_app.c`
- `foc/motor/motor.h`
- `foc/motor/motor.c`
- `foc/observer/foc_encoder.h`
- `foc/observer/foc_encoder.c`
- `foc/hal/foc_port.h`
- `peripheral/stm32g431/foc_port.c`
- `foc/tests/*.c`
- `foc/tests/run_*.ps1`

- [ ] **步骤 1：运行现有 Motor、Encoder/App 和 Identify 测试。**

```powershell
& .\foc\tests\run_motor_alpha_beta_test.ps1
& .\foc\tests\run_motor_speed_pu_test.ps1
& .\foc\tests\run_motor_reference_limit_test.ps1
& .\foc\tests\run_motor_break_fault_test.ps1
& .\foc\tests\run_encoder_command_test.ps1
& .\foc\tests\run_identify_test.ps1
```

分别记录 FLOAT 和 FIXED 结果。已有失败记录为基线，不作为本次回归。

- [ ] **步骤 2：只构建当前 STM32G431 目标，不烧录。**

```powershell
.\make.bat TARGET_CHIP=stm32g431
```

记录工作区当前是否能解析 `class/template_driver.c`，并区分工具链问题和代码问题。

- [ ] **步骤 3：记录当前直接调用关系。**

```powershell
rg -n "foc_(adc|pwm)_|foc_port_|foc_encoder_|motor_" foc class peripheral src --glob '!vendor/**' --glob '!third_party/**' --glob '*.c' --glob '*.h'
```

列出 App/Motor 中所有硬件调用、板级绑定和测试中的链接桩，最后再次核对。

### 任务 2：定义语义化注入接口

**文件：**

- 修改：`foc/hal/foc_port.h`
- 新建：`foc/motor/motor_position.h`
- 修改：`foc/motor/motor.h`
- 修改：`foc/observer/foc_encoder.h`
- 修改：`foc/app/foc_app.h`

- [ ] **步骤 1：在 `foc_port.h` 增加 ADC/PWM 的 `ops/context` 类型。**

使用带 context 的语义化回调，接口形状如下：

```c
typedef struct {
    foc_result_t (*fnSetCurrentBase)(void *pContext,
                                     uint32_t wCurrentBaseMilliamp);
    foc_result_t (*fnCalibrationBegin)(void *pContext,
                                       foc_adc_calib_t *ptCalibration);
    foc_calibration_state_e (*fnCalibrationStep)(
        void *pContext,
        foc_adc_calib_t *ptCalibration);
    foc_result_t (*fnSample)(void *pContext,
                             const foc_adc_calib_t *ptCalibration,
                             foc_current_abc_t *ptCurrent);
} foc_adc_ops_t;

typedef struct {
    foc_result_t (*fnSetDuty)(void *pContext,
                              const foc_duty_abc_t *ptDuty);
    foc_result_t (*fnEnable)(void *pContext);
    foc_result_t (*fnStop)(void *pContext);
    bool (*fnGetFaultStatus)(void *pContext);
    foc_result_t (*fnClearFaultStatus)(void *pContext);
} foc_pwm_ops_t;

typedef struct {
    const foc_adc_ops_t *ptOps;
    void *pContext;
} foc_adc_if_t;

typedef struct {
    const foc_pwm_ops_t *ptOps;
    void *pContext;
} foc_pwm_if_t;
```

`motor_Init` 和 `foc_encoder_Init` 必须校验各自所有必需回调；不增加注册表、能力位
或第二套调度器。

- [ ] **步骤 2：在独立头文件中定义快速位置接口。**

在 `foc/motor/motor_position.h` 中定义以下内容，并由 `motor.h`、
`foc_encoder.h` 同时包含。两个具体头文件之间不得互相包含。

```c
typedef struct {
    foc_result_t (*fnGetPosition)(void *pContext,
                                  uint32_t wNowTick,
                                  foc_position_t *ptPosition);
    foc_result_t (*fnCaptureZero)(void *pContext,
                                  uint32_t wNowTick,
                                  foc_position_t *ptPosition);
} motor_position_ops_t;

typedef struct {
    const motor_position_ops_t *ptOps;
    void *pContext;
} motor_position_if_t;
```

该接口不得暴露 AS5600、SMO 或具体 Encoder 结构体给 `motor.h`。

- [ ] **步骤 3：调整 `motor_cfg_t` 和 `motor_t`。**

在配置中加入 `foc_adc_if_t tAdc`、`foc_pwm_if_t tPwm`、
`motor_position_if_t tPosition`。Motor 对象按值保存绑定后的接口；删除旧的
`fnGetPosition`/`pPositionContext` 双字段，确保位置依赖只有一份。

保留现有电机参数、限制、PI、校准、命令、电气零位、生命周期、故障和 PWM 状态。
Motor 按模板实现自己的 `cfg/object/status/Init/Run/Stop/Reset/Get` 语义，不嵌套通用
`template_driver_t`，避免重复状态和 ISR 二次分发。

- [ ] **步骤 4：调整 Encoder 配置。**

Encoder 必须完整采用 `template_driver` 的 Driver 结构：配置只用于初始化绑定，
对象拥有全部可变运行态，状态快照通过查询 API 输出，支持显式 Init、前台 Run/Update、
Stop/Reset 和 Get 操作。把原始传感器回调改成显式 sensor `ops/context`，保留速度
滤波、无效超时、方向反转、双缓冲和所有运行态字段。Encoder 不嵌套通用
`template_driver_t`，而是专门化同一模板规则，避免重复调度层。

声明：

```c
extern const motor_position_ops_t g_tFocEncoderPositionOps;
```

### 任务 3：把 STM32G431 板级硬件改成注入 ops

**文件：**

- 修改：`peripheral/stm32g431/foc_port.c`
- 修改：`peripheral/stm32g431/foc_port_config.h`
- 修改：STM32G431 目标中负责 break 通知的中断适配文件

- [ ] **步骤 1：在不改变数学行为的前提下封装 ADC。**

把当前原始通道映射、偏置校验、电流缩放、校准触发和采样转换放到
`foc_adc_ops_t` 回调后面。保留 STM32G431 当前的通道、极性、缩放和触发行为。

- [ ] **步骤 2：在不改变安全行为的前提下封装 PWM。**

把 duty 转换、duty 提交、Enable、Stop、break 查询和 break 清除放到
`foc_pwm_ops_t` 回调后面。`fnStop` 必须支持 Motor 部分初始化期间调用；硬件停机失败
时仍锁存 Motor 故障，不能报告恢复成功。

- [ ] **步骤 3：明确 break 锁存的所有权。**

把当前 break 锁存放进显式 PWM board context，并让目标中断通过该 context 通知板级
实现。App/Motor 不得重新调用无 context 的 `foc_pwm_NotifyBreak()`。

- [ ] **步骤 4：只导出绑定对象，不导出全局硬件包装函数。**

导出常量 ADC/PWM interface binding 和独立的 AS5600 sensor binding。Vendor HAL 调用
只能留在 `peripheral/stm32g431`，不得移动到 `foc/app` 或 `foc/motor`。

### 任务 4：按照 `template_driver` 实现 Encoder Driver

**文件：**

- 修改：`foc/observer/foc_encoder.h`
- 修改：`foc/observer/foc_encoder.c`
- 修改：`foc/app/foc_app.h`
- 修改：`foc/app/foc_app.c`
- 修改：`foc/foc.mk`
- 修改：Encoder/App 测试及对应运行脚本

- [ ] **步骤 1：实现模板化 Driver 初始化和原始传感器绑定校验。**

`foc_encoder_Init` 清零调用者提供的对象，将状态置为
`FOC_ENCODER_STATE_UNINITIALIZED`，校验 sensor ops/context、滤波范围、超时和系统
tick 频率，复制初始化配置，并且只调用一次注入的 sensor init。初始化失败时锁存
明确错误，调用安全 stop（如传感器 Driver 提供），返回现有错误类别并保持对象无效。

- [ ] **步骤 2：保留现有 Encoder 算法。**

保留 raw-to-BAM32、环绕差值、速度滤波、方向反转、年龄外推、超时失效和双槽发布。
只把旧回调字段替换为注入的 sensor interface 调用。

- [ ] **步骤 3：实现模板化 Driver 的 Run/Stop/Reset/Status 语义。**

    `foc_encoder_Run` 作为 Driver 的前台 Run 服务；传感器读取失败时更新
Encoder 自己的状态和错误快照，但不能隐藏错误或使用文件静态状态。Stop/Reset 必须
清理前台服务状态、保留或清除错误按明确 API 语义执行；状态查询必须复制只读快照，
不能把内部结构地址暴露给 App。高频位置读取不经过 Run/Update 调度器。

- [ ] **步骤 4：保持慢速和快速 API 分离。**

`foc_encoder_Run` 继续执行前台传感器事务；`foc_encoder_GetPosition` 继续作为
Motor 高频路径的缓存读取；`foc_encoder_CaptureZero` 继续使用缓存并返回原有安全结果。
三者通过 typed position interface 绑定。

- [ ] **步骤 5：新增 Encoder Driver fake 测试。**

使用带 init/read 计数器和预设角度/错误序列的 fake sensor context，断言：Update 才会
调用原始传感器；GetPosition 不会调用传感器；方向、滤波、环绕和超时结果与基线一致；
两个 Encoder 对象的槽位和滤波状态互不共享。

### 任务 5：按注入硬件重构 Motor，并压平高频调用链

**文件：**

- 修改：`foc/motor/motor.h`
- 修改：`foc/motor/motor.c`
- 修改：`foc/tests/motor_*.c`

- [ ] **步骤 1：删除 Motor 中所有直接 port 调用。**

以下符号必须从 `motor.c` 消失，改为调用对应的 `tAdc`/`tPwm` 成员：

```text
foc_adc_SetCurrentBaseMilliamp
foc_adc_CalibBegin
foc_adc_CalibStep
foc_adc_Sample
foc_pwm_SetDuty
foc_pwm_Enable
foc_pwm_Stop
foc_pwm_GetFaultStatus
foc_pwm_ClearFaultStatus
```

- [ ] **步骤 2：保持 Motor 初始化和安全状态。**

在复制配置前校验所有必需操作；绑定接口、初始化控制器、调用 ADC base setup、复位
现有 observer、启动校准，均遵循当前状态契约。绑定 PWM 后的每个失败分支都调用注入
的 stop，返回前不得 Enable PWM。

- [ ] **步骤 3：压平高频控制路径。**

`motor_IsrStep` 只分发到一个当前状态处理函数。RUNNING 处理函数直接按顺序
执行 ADC sample、position read、现有 observer、speed 分频/PI、`foc_core_step` 和
PWM duty commit。

不得重新形成以下包装链：

```text
motor_IsrStep -> _motor_RunControlStep -> hardware operation
```

辅助函数只能承担叶子计算或状态更新，不能再增加硬件分发层。最终调用图必须符合项目
规定的实时三层深度限制。

- [ ] **步骤 4：保持故障优先顺序。**

ADC、position、Core、observer 或 duty 失败时，先调用绑定的 PWM stop，再让 Motor 故障
状态对外可见。break 检测继续通过前台轮询进入故障；ClearFault 在现有中断保护内调用
绑定的 clear 操作。

- [ ] **步骤 5：保持控制 API 语义。**

保持 Start、Stop、ClearFault、电压/电流/速度参考、对齐、状态快照、速度 PU 换算、
限制、PI 参数、速度分频、电气角度换算和状态准入规则不变。只允许调整硬件依赖访问
及 Driver 结构所必需的名称。

- [ ] **步骤 6：将 Motor 测试改为 fake interface。**

用 fake ADC/PWM/position context 替换链接桩，并使用以下事件枚举记录调用顺序：

```c
enum {
    TEST_EVENT_ADC_SAMPLE = 1,
    TEST_EVENT_POSITION_READ,
    TEST_EVENT_CORE_STEP,
    TEST_EVENT_PWM_DUTY,
    TEST_EVENT_PWM_STOP,
};
```

保留所有数值断言；增加采样、位置、duty 和 break 失败时 PWM stop 先于故障可见的断言。

### 任务 6：把 `foc_app` 收敛为 Class 组合层

**文件：**

- 修改：`foc/app/foc_app.h`
- 修改：`foc/app/foc_app.c`
- 修改：`foc/app/motor_config.h`
- 修改：`foc/foc.mk`
- 修改：`makefile`
- 修改：`foc/tests/foc_app_encoder_command_test.c`

- [ ] **步骤 1：按 Class 模板顺序初始化。**

`foc_app_Init` 按以下顺序执行：

```text
检查 object/config 地址
清零 App 对象并初始化 App 状态
绑定 MODUS parent
绑定 STM32G431 ADC/PWM 和 Encoder sensor interface
初始化 Encoder Driver
把 Encoder position interface 绑定到 Motor 配置
初始化 Motor Driver
初始化可选 Identify Driver
最后调用 mbase_Init
```

Motor 依赖绑定后，如果子模块初始化失败，调用 Motor 的安全停机路径；App 不调用原始
全局 PWM stop。

- [ ] **步骤 2：删除 App 硬件胶水。**

删除 `foc_app_GetPosition` 以及 `foc_app.c` 中所有直接的 `foc_adc_*`、`foc_pwm_*` 和
`foc_port_PositionContext` 调用。App 配置只在组合阶段提供显式板级绑定。

- [ ] **步骤 3：保持 App 调度和命令行为。**

保持前台周期、Encoder 错误退避、HF 统计、Shell 命令、Identify mailbox 和 waveform
通道。App 只能通过公开查询 API 读取子 Driver 状态，不直接修改子对象内部状态。

- [ ] **步骤 4：修正构建源文件路径。**

根 `makefile` 使用实际路径：

```make
CLASS_SOURCES = class/template_class.c class/template_driver.c
```

`foc/observer/foc_encoder.c` 保留在 `FOC_SOURCES`；`class/foc_identify_minimal.c` 不加入
固件，它只是结构参考，不是当前 Identify 实现。

- [ ] **步骤 5：更新 App 组合测试。**

显式绑定 fake interface，断言 App 初始化把 Encoder context 连接到 Motor，前台 Run 更新
Encoder，HF ISR 调用 Motor，且 App 失败清理不直接产生 PWM 事件。

### 任务 7：按照 `template_driver` 实现 Identify 算法 Driver

**文件：**

- 修改：`foc/identify/foc_identify.h`
- 修改：`foc/identify/foc_identify.c`
- 修改：`foc/app/foc_app.h`
- 修改：`foc/app/foc_app.c`
- 修改：`foc/tests/foc_identify_test.c`
- 只读：`class/foc_identify_minimal.h`、`class/foc_identify_minimal.c`

- [ ] **步骤 1：隔离两套 Identify 类型模型，并确定模板能力档位。**

同一编译单元不得同时包含两个定义 `foc_identify_t` 的头文件。当前数值/诊断模型是
行为基准；minimal 模板只提供显式对象、配置、状态、命令、结果和生命周期结构参考。
Identify 必须完整遵循 `template_driver` 的 `cfg/object/status/lifecycle` 规则，
采用“高频 `IsrStep` + 前台状态查询/终态处理”的能力档位；不嵌套通用
`template_driver_t`，也不为了形式一致增加无意义的 PT `Run`。

- [ ] **步骤 2：使 Identify 初始化自洽。**

先校验配置，再清零调用者提供的对象，将状态置为
`FOC_IDENTIFY_STATE_UNINITIALIZED`，初始化 IDLE/错误状态和零命令，绑定所有输入输出
依赖，返回现有 `foc_result_t` 契约。禁止文件静态可变运行时状态。

- [ ] **步骤 3：分离 ISR 与前台职责。**

Identify 高频 `IsrStep` 继续消费现有电流、上一拍电压命令、机械角、有效标志和故障
输入；App 前台处理 Start/Abort 请求、终态消费、Reset/状态/结果/诊断查询。Identify
不调用 ADC、PWM、Encoder 或板级函数。所有 Driver 状态和错误都留在 Identify 对象
及其输出快照中。

- [ ] **步骤 4：保留算法和诊断行为。**

保留电阻试验、零电压停留、Ld/Lq 拟合、反向脉冲、电流/位移/故障检查、终态处理和
当前诊断快照。不得用更简单的 float-only minimal 实现替换当前算法。

- [ ] **步骤 5：增加 Identify Driver 生命周期断言。**

覆盖空指针/配置失败、初始化后 IDLE、Start/Abort、失败后的零命令、结果/诊断读取和
终态消费，同时保留现有数值识别断言。

### 任务 8：最终验证和架构审查

**文件：** 阅读所有修改文件；仅在源文件列表确实需要时修改测试脚本或文档。

- [ ] **步骤 1：每完成一个 Driver 就运行对应专项测试。**

任务 4 后运行 Encoder Driver；任务 5 后运行 Motor；任务 6 后运行 App；任务 7 后运行
Identify。现有脚本支持的地方必须同时运行 FLOAT 和 FIXED。

- [ ] **步骤 2：证明领域层边界已经清理。**

以下命令在 App 和 Motor 中必须无匹配：

```powershell
rg -n "foc_(adc|pwm)_|foc_port_PositionContext|foc_port_PositionInit|foc_port_PositionRead" foc/app/foc_app.c foc/motor/motor.c
```

以下搜索在 FOC 领域代码中必须无 Vendor/HAL 调用：

```powershell
rg -n "haladc_|halpwm_|MDI_|haltim1_|HAL_|LL_" foc/app foc/motor foc/observer
```

发现匹配时必须逐项审查，不能用编译器选项掩盖。

- [ ] **步骤 3：审查高频调用深度和禁用操作。**

从 `foc_app_HighFrequencyISR` 跟踪到每个注入硬件操作，确认调用链浅、无 PT/日志/总线/
动态内存/阻塞延时，并保持 ADC → position → observer → speed PI → Core → PWM 顺序。

- [ ] **步骤 4：运行完整主机测试和 STM32G431 构建。**

```powershell
& .\foc\tests\run_motor_alpha_beta_test.ps1
& .\foc\tests\run_motor_speed_pu_test.ps1
& .\foc\tests\run_motor_reference_limit_test.ps1
& .\foc\tests\run_motor_break_fault_test.ps1
& .\foc\tests\run_encoder_command_test.ps1
& .\foc\tests\run_observer_contract_test.ps1
& .\foc\tests\run_identify_test.ps1
.\make.bat TARGET_CHIP=stm32g431
```

本计划不烧录硬件。工具链不可用时，报告具体不可用命令，不得声称真机行为已验证。

- [ ] **步骤 5：审查所有静态对象和所有权。**

列出修改后 C 文件的每个 `static` 对象，并归类为 MODUS 基础对象、只读数据、显式拥有
的板级资源或测试 fake。任何位于对象外部的 Motor、Encoder、Identify、PT、定时器、缓存、
命令或故障可变状态都必须在完成前移回其拥有对象。

## 完成标准

- `foc_app` 只承担 Class 组合和调度边界。
- Motor、Encoder、Identify 的可变状态都由调用者提供的对象拥有。
- ADC/PWM 行为由初始化时的 `ops/context` 注入决定。
- `motor.c` 和 `foc_app.c` 不再直接调用全局 FOC 硬件接口。
- 高频调用深度和操作顺序符合已确认约束。
- Encoder、speed、current、align、fault、Identify 现有测试保持通过。
- STM32G431 构建通过，或明确报告工具链阻塞。
- `template_observer` 和 SMO 保持不变。
