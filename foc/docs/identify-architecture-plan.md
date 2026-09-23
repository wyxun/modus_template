# FOC Identify 参数辨识架构规划

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不污染 Motor 热路径、不复制安全逻辑的前提下，为已经存在的电阻辨识和第一阶段 `Ld` 辨识提供最小父调度、对象所有权及 ISR 边界。

**Architecture:** `identify_t` 只拥有公共生命周期、当前 operation、首个错误和各辨识子对象。每个已实现的辨识类型独立拥有业务状态、PT 游标、ISR 统计和结果；前台通过 `identify_Run()` 推进流程，20 kHz ISR 通过唯一的 `identify_IsrStep()` 分发同步数据和有界操作。Motor 继续拥有 PWM、故障状态和安全停机，Identify 不直接访问底层端口。

**Tech Stack:** C11、FOC `motor_t`、perfc-PT、MDI/perfc tick、FLOAT/FIXED 双数值后端。

---

## 1. 当前范围与非目标

当前只规划两个真实子模块：

1. 已实现的两电平定子电阻辨识；
2. [电感辨识专项计划](identify-inductance-plan.md)中的 Phase 1 `Ld`。

磁链辨识和 `Lq` 不属于当前架构实施范围：

- `Lq` 必须在 `Ld` 真机稳定、机械锁止准入和 ISR 安全停机通过后再开启；
- 磁链辨识必须先选定物理方法，当前不创建 `tFlux`、配置、积分器、状态或 API；
- 不建设通用算法基类、driver、回调表、运行时函数指针或算法注册表；
- 不为节省尚未证明存在的 RAM 压力，把子对象改造成 union 或动态分配。

算法公式、注入频率、采样窗口、物理量换算和台架验收只在各专项计划中维护，本文不复制，避免两份文档漂移。

## 2. 对象所有权

### 2.1 父对象

父状态只表达公共生命周期，不包含任何电阻或电感步骤：

```c
typedef enum {
    IDENTIFY_STATE_UNINITIALIZED = 0,
    IDENTIFY_STATE_IDLE,
    IDENTIFY_STATE_RUNNING,
    IDENTIFY_STATE_ERROR
} identify_state_t;

typedef enum {
    IDENTIFY_OPERATION_NONE = 0,
    IDENTIFY_OPERATION_RESISTANCE,
    IDENTIFY_OPERATION_INDUCTANCE
} identify_operation_t;

typedef struct {
    volatile identify_state_t eState;
    volatile identify_operation_t eOperation;
    foc_result_t eLastResult;
    identify_resistance_t tResistance;
    identify_inductance_t tInductance;
} identify_t;
```

父对象负责：

- 保证同一时刻只有一个 operation 活动；
- 分发 `Start/Run/IsrStep/Stop/Reset`；
- 保存公共状态和第一个错误；
- 在完成、停止或失败时要求 Motor 进入一致的安全状态。

父对象不保存子 PT 游标、采样累加器、算法结果或硬件句柄，也不保存全局 `motor_t *`。

### 2.2 子对象

每个子对象独立拥有：

- 自己的业务状态枚举和 `chRunPt`；
- 配置快照、计数器、超时和 ISR/前台交接标志；
- 采样统计、结果和结果待取标志；
- 自己的启动、运行、ISR、停止、复位和结果读取私有函数。

电阻状态迁移到 `identify_resistance_t`，不再把
`IDENTIFY_STATE_RESISTANCE_*` 放进父 `identify_state_t`。迁移只改变状态所有权和父调度，不改变已经验证的电阻测量算法。

所有可变状态必须属于 `identify_t` 或其子对象，禁止文件静态 runtime。

## 3. 公共 API 与状态语义

```c
foc_result_t identify_Init(identify_t *ptThis);

foc_result_t identify_StartResistance(identify_t *ptThis);

foc_result_t identify_StartInductance(
    identify_t *ptThis,
    const identify_inductance_cfg_t *ptConfig);

foc_result_t identify_Run(identify_t *ptThis, motor_t *ptMotor);

void identify_IsrStep(
    identify_t *ptThis,
    motor_t *ptMotor,
    const identify_isr_sample_t *ptSample);

void identify_Stop(identify_t *ptThis, motor_t *ptMotor);

foc_result_t identify_Reset(
    identify_t *ptThis,
    const motor_t *ptMotor);
```

结果继续使用类型明确的接口，不增加无类型通用结果容器：

```c
foc_result_t identify_GetResistance(
    identify_t *ptThis,
    uint32_t *pwResistanceMilliohm);

foc_result_t identify_GetInductance(
    identify_t *ptThis,
    identify_inductance_result_t *ptResult);
```

公共状态只报告父层事实：

```c
typedef struct {
    identify_state_t eState;
    identify_operation_t eOperation;
    foc_result_t eLastResult;
    bool bResultPending;
} identify_status_t;
```

电压级、样本数和半周期等算法进度不进入公共状态。确有 Shell 或测试需求时，再为对应子模块增加类型明确的状态接口。

状态语义固定为：

- `Start*()` 只在父状态为 `IDLE` 且没有未消费结果时成功；
- 启动成功后先完整准备子对象，再在中断保护内发布 operation 和 `RUNNING`；
- `identify_Run()` 只分发活动子 PT，并映射完成、继续或失败；
- `Stop()` 在同一个中断保护区内先撤销 active/operation，再调用可在该上下文执行的 Motor 停止路径，最后停止当前子对象并回到 `IDLE`；
- `Reset()` 通过传入的 Motor 状态确认 PWM 已关闭后，才清除锁存错误、PT 游标和结果；
- Identify 的第一个阶段错误不得被后续停止错误覆盖；Motor 保留原始故障位，后续安全停机错误可以追加 PWM 故障位。

## 4. 前台与 ISR 数据流

```text
foc_app_Run()                         foreground, about 1 ms
    `-- identify_Run(identify, motor)
            `-- active child RunPt()

foc_app_HighFrequencyISR()            one configured HF rate
    |-- motor_IsrStep(motor)
    `-- identify_IsrStep(identify, motor, sample)
            `-- active child IsrStep()
```

App 只有一个 Identify ISR 调用点。父 ISR 使用直接 `switch` 分发当前 operation，不使用函数指针或通用命令分派框架。

`identify_isr_sample_t` 只包含当前已实现电阻和 `Ld` 所需的同步快照：

```c
typedef struct {
    foc_scalar_t qCurrentD;
    foc_scalar_t qCurrentQ;
    foc_scalar_t qElectricalSpeedPu;
    uint32_t wDcBusMillivolt;
    bool bDcBusValid;
    bool bPwmSaturated;
    bool bMotorFault;
    bool bAngleValid;
} identify_isr_sample_t;
```

不为未来磁链辨识预加磁链、电压积分或观察器字段。后续真实算法需要新数据时再扩展快照。

## 5. Identify 与 Motor 边界

Motor 始终拥有模式、注入命令、PWM 状态、Break 和故障生命周期。Identify 不得直接写 `motor_t` 私有成员，不得调用 `FOC_PORT_PWM_SAFE_STOP()`、PWM 寄存器或 vendor HAL。

电感辨识只增加两个类型化、固定边界的 Motor ISR 能力：

```c
foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor,
    const foc_dq_t *ptVoltageCommand);

void motor_IdentificationAbortIsr(
    motor_t *ptMotor,
    motor_fault_e eFault);
```

- `ApplyIsr()` 只接收前台已经校验的 d/q 调制度，并保持固定执行时间；
- `AbortIsr()` 原子清零注入、执行 PWM safe-stop、更新 `bPwmEnabled`、Motor 状态和故障位；
- `motor_Stop()`、Break 和普通 Motor fault 同样清除辨识注入；
- Identify 同时保存自己的首个阶段错误；Motor 保留原始故障位，后续故障只能追加位，不能清除原始原因。

电角度对齐仍由 Motor 生命周期负责。Identify 只检查有效的电角度零点；未对齐时拒绝启动，不在子 PT 中复制一套固定 alpha/beta 对齐流程。

母线电压采样、PWM 饱和产生方式和 `Ld` 注入保护以专项计划为准。硬件没有有效母线采样或安全停机能力时，电感辨识保持禁用，不能使用成功 stub。

## 6. ISR 与前台并发契约

配置和批次交接遵守单生产者/单消费者规则：

1. 前台在中断保护内复制配置、清空统计，最后发布 `bActive`、operation 和父 `RUNNING`；
2. 活动期间只有 ISR 写计数器、累加器、首末样本、批次错误和 `bBatchReady`；
3. ISR 写完完整批次和首个错误后，最后设置 `bBatchReady`；
4. 前台发现 ready 后，在中断保护内复制完整批次并清除 ready，退出临界区后再计算结果；
5. 32 位 MCU 上禁止无保护读取 ISR 写入的 64 位累加器；
6. `volatile` 只用于跨上下文可见性，不替代临界区；
7. `Stop/Reset` 必须先撤销 active/operation，确认 ISR 不再写入；`Stop` 随后停止 Motor，`Reset` 则必须先确认 Motor 已停止，才能清除子对象。

ISR 每次调用必须有界，不执行日志、动态内存、阻塞等待、无界循环或最终物理结果计算。生产热路径调用深度不超过 3 层，并测量最坏执行时间。

## 7. 单一时基

FOC 高频频率必须只有一个板级权威来源：由目标配置提供 `FOC_HF_ISR_HZ`，并通过 `foc_config.h` 暴露给 Motor、Identify 和测试。PWM 配置、Resistance 分频、Inductance 分频、超时换算和主机模型全部引用该值。

删除或映射以下重复来源，禁止彼此独立配置：

```text
MOTOR_PWM_FREQ_HZ
IDENTIFY_RESISTANCE_HF_ISR_HZ
各辨识子模块私有的 20000U
```

若 PWM 频率与 ADC/控制 ISR 频率未来不相等，应分别定义有物理语义的常量，并在板级配置中显式说明二者关系；当前不为这种未出现的需求增加配置层。

## 8. 文件职责

```text
foc/identify/
|-- identify.h                  public parent API and complete child objects
|-- identify.c                  parent lifecycle and direct dispatch only
|-- identify_resistance.h       private resistance declarations
|-- identify_resistance.c       resistance PT, ISR capture and calculation
|-- identify_inductance.h       private inductance declarations
`-- identify_inductance.c       Phase 1 Ld PT, ISR capture and calculation
```

测试入口保持独立：

```text
foc/tests/identify_resistance_test.c
foc/tests/identify_inductance_test.c
```

`identify.c` 不包含子算法计算。只有出现第三个已经确认且独立实现的算法后，才重新评估父 dispatch 是否需要调整；默认仍保留直接 `switch`。

## 9. 实施顺序

### Task 1：纠正父子状态所有权

**Files:** `foc/identify/identify.h`, `foc/identify/identify.c`, `foc/identify/identify_resistance.c`, `foc/tests/identify_resistance_test.c`

- [x] 将父状态收敛为 `UNINITIALIZED/IDLE/RUNNING/ERROR`；
- [x] 将电阻步骤状态移入 `identify_resistance_t`；
- [x] 公共状态增加 operation，移除电阻专用字段；
- [x] 保持电阻算法、结果和外部启动接口行为不变；
- [x] 运行电阻 FLOAT/FIXED 测试，确认状态迁移和结果无回归。

### Task 2：统一时基和 ISR 契约

**Files:** `foc/foc_config.h`, `foc/app/motor_config.h`, `foc/identify/identify.h`, `foc/identify/identify.c`, `foc/app/foc_app.c`, `foc/tests/identify_resistance_test.c`, `foc/tests/foc_app_encoder_command_test.c`

- [x] 建立唯一的 FOC 高频频率定义并迁移电阻分频；
- [x] 将 `identify_IsrStep()` 扩展为显式接收 `motor_t *` 和同步快照；
- [x] 将 `identify_Reset()` 扩展为接收只读 Motor，并拒绝在 PWM 未关闭时复位；
- [x] 保持 App 只有一个 Identify ISR 调用点；
- [x] 定义配置发布、批次 ready、首错锁存和 Stop/Reset 的临界区顺序；
- [x] 用交错 ISR/前台测试验证批次不会撕裂。

### Task 3：实现电感前置能力和 Phase 1 `Ld`

严格执行 [电感辨识专项计划](identify-inductance-plan.md)的 Task 0～Task 5。该计划负责母线电压、PWM 饱和、Motor 注入/停机 API、时序验证、RL 模型和真机验收；本架构文档不重复任务细节。

### Task 4：延期功能准入

- [ ] 只有 `Ld` 真机验收完成后，才单独评审 `Lq`；
- [ ] 只有磁链辨识物理方法、输入、运行条件和安全边界确认后，才创建独立专项计划；
- [ ] 不因“以后可能需要”提前修改公共对象或 ISR 快照。

## 10. 架构验收标准

- [ ] 父状态不包含任何 Resistance/Inductance 专用步骤；
- [ ] 每个子对象独立拥有 PT、状态、统计、超时和结果；
- [ ] `identify.c` 只包含公共生命周期和直接 dispatch；
- [ ] App 只有一个 Identify ISR 调用点；
- [ ] Identify 只能通过类型化 Motor API 提交注入和触发安全停机；
- [ ] Motor fault、Break、Stop、超时和辨识保护均清除注入并保持对象状态与硬件一致；
- [ ] ISR/前台交接不会读取撕裂的 64 位统计，首个错误不会被覆盖；
- [ ] 全工程只有一个有效的 FOC 高频时基来源；
- [ ] 电阻 FLOAT/FIXED 测试无回归；
- [ ] `Ld` 按专项计划完成主机模型、故障注入、时序边缘、ISR 周期预算和真机验证；
- [ ] 没有 `tFlux`、通用算法基类、回调表或运行时函数指针等提前抽象。

## 11. 最终确认

1. 父对象只承担生命周期、operation、首错和直接调度；
2. 电阻和电感状态属于各自子对象；
3. Motor 独占功率级、安全停机和故障状态；
4. 高频路径使用显式对象参数和编译期绑定，不保存隐藏全局依赖；
5. 总规划只维护公共架构，算法细节由专项计划维护；
6. 当前只交付 Resistance 和 Phase 1 `Ld`，`Lq` 与 Flux 按证据延期。
