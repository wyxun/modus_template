# FOC 电感辨识实施计划

> **当前状态：Phase 1 `Ld` 的主机实现已完成，默认仍由母线配置门控。**
> 目标板的 ADC 标定、采样年龄和机械锁轴仍需在台架确认后才能启用；第一阶段只实现机械锁轴条件下的 `Ld`，`Lq` 继续延期。

**目标：** 在 [identify-architecture-plan.md](identify-architecture-plan.md) 约定的父对象、子 PT 和 20 kHz ISR 框架下，先得到指定测试条件下可复现的 d 轴增量电感 `Ld`，再复用同一采样逻辑扩展 `Lq`。

**最小架构：** `identify_Run()` 调度电感子 PT，现有 `identify_IsrStep()` 分发同步采样。电感代码从第一版起放在 `identify_inductance.c/.h`，与 `identify_resistance.c/.h` 同级；父模块直接调用私有函数，不增加 driver、函数指针或通用辨识接口。

**技术约束：** C11、`motor_t`、perfc-PT、MDI/perfc tick、20 kHz FOC ISR、FLOAT/FIXED 双后端。

---

## 1. 实施门槛和阶段范围

以下门槛未全部满足前，不编写方波注入和电感计算代码：

- [ ] **G1：实际电压有效。** 能取得有效、未过期的母线电压，并明确调制度到实际 d/q 轴电压的板级标定关系；不能只把 `qVoltageAmplitude` 当作伏特。
- [ ] **G2：ISR 过流停机有效。** 每个高频周期检查 `|Id|` 和 `|Iq|`，超限后在当前 ISR 内撤销注入、触发 PWM 安全停止并锁存错误。
- [ ] **G3：采样时序已验证。** 明确命令发布、PWM 更新和 ADC 采样的周期关系，通过带周期标签的测试证明切换边缘样本不会参与斜率计算。
- [ ] **G4：机械锁轴条件成立。** 第一阶段 `Ld` 仅在受控台架和机械锁轴条件下运行。位置闭环当前不可用，不能作为锁轴替代方案。

阶段范围固定为：

1. **Phase 1：只实现 `Ld`。** 机械锁轴，d 轴双极性注入，`Vq = 0`。
2. **Phase 2：真机验证 `Ld`。** 使用已知参考值，在相同频率、幅值、电流和温度条件下比较。
3. **Phase 3：再实现 `Lq`。** 必须先提供可执行的机械锁止准入条件；检测到转动后停止只能作为故障保护，不能作为防止首次运动的措施。

当前工程未实现位置闭环，`motor_Start()` 也拒绝 `FOC_MODE_POSITION`，因此计划中不再保留“位置闭环锁轴”的备选路径。

## 2. 代码组织和职责

### 2.1 文件布局

电感辨识开始实现时直接创建：

```text
foc/identify/
├── identify.c                 # 父状态、公共 API、子流程调度
├── identify.h                 # 公共配置、结果和父对象
├── identify_resistance.c/.h   # 已有电阻子 PT
└── identify_inductance.c/.h   # 电感子 PT、ISR 采样和计算
```

`identify_inductance.h` 只暴露给父模块所需的内部函数。父模块直接调用，不为电阻、电感和后续磁链辨识设计统一虚接口。

### 2.2 现有入口

父级 `identify_IsrStep()` 和 App 统一入口已经存在。本计划不再重构入口，只扩展它传递的同步快照，并在 `IDENTIFY_OPERATION_INDUCTANCE` 活动时调用电感子 ISR。App 在同一个 ISR 中显式传入当前 `motor_t *`；Identify 不保存全局 Motor 指针。

职责保持如下：

- `identify_Run()`：前台 PT 调度、超时、状态收敛和结果发布；
- `identify_IsrStep(identify_t *, motor_t *, const identify_isr_sample_t *)`：父级 ISR 分发；
- `_identify_inductance_RunPt()`：电感前台子 PT；
- `_identify_inductance_IsrStep()`：有界的高频保护、注入切换和采样；
- Motor：消费已校验的注入命令，执行 PWM，并提供同步电流、母线电压及饱和状态。

内部函数统一使用 `_identify_...` 前缀。

### 2.3 Identify 到 Motor 的类型化控制通道

Identify 不得直接调用 `FOC_PORT_PWM_SAFE_STOP()`、PWM 寄存器或其他底层端口。Task 2 在 `motor.h/.c` 增加两个仅供同一 FOC ISR 使用的类型化 API：

```c
foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor,
    const foc_dq_t *ptVoltageCommand);

void motor_IdentificationAbortIsr(
    motor_t *ptMotor,
    motor_fault_e eFault);
```

- `motor_IdentificationApplyIsr()` 只接受已经在前台校验过的 d/q 调制度，保持固定执行时间；
- `motor_IdentificationAbortIsr()` 必须原子地清零辨识注入、停 PWM、锁存 Motor 故障并更新 `bPwmEnabled`、Motor 状态及故障位；其内部复用 Motor 已有的故障状态收敛路径；
- Identify 自身同时锁存辨识阶段错误。Motor 的第一个故障和 Identify 的第一个阶段错误均不得被后续错误覆盖；
- `motor_Stop()`、Break 和普通 Motor fault 也必须清除已准备的辨识注入。

名称可按现有 Motor 命名风格微调，但“提交注入”和“原子故障停机”必须是两个语义明确的 Motor API，不能通过裸端口绕过 Motor 对象状态。

## 3. 配置、同步快照和结果

### 3.1 最小配置

```c
typedef struct {
    uint32_t wInjectionFrequencyHz;
    uint16_t hwCaptureDelayCycles;
    uint16_t hwCaptureSampleCount;
    uint16_t hwHalfCycleCount;
    foc_scalar_t qModulationAmplitude;
    foc_scalar_t qMaxIdentificationCurrent;
    foc_scalar_t qMinCurrentDelta;
    foc_scalar_t qMaxElectricalSpeedPu;
    uint16_t hwMotionFaultCycles;
} identify_inductance_cfg_t;
```

字段语义：

- `qModulationAmplitude` 是 FOC 调制度，不是物理电压；
- `qMaxIdentificationCurrent` 是辨识专用的软件限流值，必须大于噪声阈值且不高于 Motor/硬件允许值；
- `qMinCurrentDelta` 是有效窗口所需的最小电流变化量，应高于电流采样噪声；
- `qMaxElectricalSpeedPu` 和 `hwMotionFaultCycles` 定义转动故障阈值；速度连续超限指定周期数即中止，二者复用 Motor 现有的 `qElectricalSpeedPu` 和 `bAngleValid`；
- `hwHalfCycleCount` 必须为偶数且至少为 4，即至少采集 2 个正半周期和 2 个负半周期；
- 每个半周期至少保留 2 个有效采样点。

### 3.2 ISR 同步快照

Task 1 是扩展现有同步快照，而不是新建第二个 App ISR 入口。快照至少包含：

```c
typedef struct {
    foc_scalar_t qCurrentD;
    foc_scalar_t qCurrentQ;
    uint32_t wDcBusMillivolt;
    bool bDcBusValid;
    bool bPwmSaturated;
    bool bMotorFault;
    foc_scalar_t qElectricalSpeedPu;
    bool bAngleValid;
} identify_isr_sample_t;
```

如果母线电压无效、过期，或 PWM 已限幅/饱和，本批次不得计算 `µH`。实现时可复用工程现有类型和标志，不要求机械复制上述字段名。

### 3.3 母线电压和 PWM 饱和的来源

第一版允许先使用配置宏占位，默认仍然禁用：

```c
#define FOC_DCBUS_SOURCE_NONE       (0U)
#define FOC_DCBUS_SOURCE_NOMINAL    (1U)
#define FOC_DCBUS_SOURCE_ADC        (2U)

#define FOC_DCBUS_SOURCE             FOC_DCBUS_SOURCE_NONE
#define FOC_DCBUS_NOMINAL_MILLIVOLT (0U)
#define FOC_DCBUS_MV_PER_COUNT_NUM  (1U)
#define FOC_DCBUS_MV_PER_COUNT_DEN  (1U)
#define FOC_DCBUS_OFFSET_MILLIVOLT  (0)
#define FOC_DCBUS_SAMPLE_DELAY_CYCLES (0U)
#define FOC_DCBUS_MAX_AGE_CYCLES    (0U)
#define FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES (1U)
#define FOC_DCBUS_MODULATION_NUM   (1U)
#define FOC_DCBUS_MODULATION_DEN   (1U)
```

`FOC_DCBUS_SOURCE_NOMINAL` 只表示受控电源下的标称母线电压估算，不等价于 ADC 实测；其结果必须记录为标称母线条件下的增量电感。`FOC_DCBUS_SOURCE_ADC` 才使用以下 ADC 换算：

```ini
Vbus_mV = ADC_count × FOC_DCBUS_MV_PER_COUNT_NUM /
          FOC_DCBUS_MV_PER_COUNT_DEN + FOC_DCBUS_OFFSET_MILLIVOLT
```

在 `FOC_DCBUS_SOURCE_NONE` 或标称电压为零时，辨识保持禁用。后续只需修改目标板配置宏，不需要修改电阻辨识代码。

Task 1 同时扩展 `foc/hal/foc_port.h`、目标板 `foc_port.c` 适配和测试 fake：

- 新增单位明确的直接能力 `foc_result_t foc_SampleDcBusMillivolt(uint32_t *pwMillivolt)`；目标板返回经过缩放的 ADC 结果，fake 可按 ISR 周期提供指定值；不支持时返回明确错误，不能成功 stub；
- 母线电压样本携带取得周期或年龄。`identify_isr_sample_t` 只在电压年龄不大于板级常量 `FOC_DCBUS_MAX_AGE_CYCLES` 时置 `bDcBusValid`；默认要求同一 PWM/ADC 同步点取得，放宽该常量前必须有时序和误差验证；
- PWM 饱和在 `foc_modulation.c` 产生：调制器比较请求 duty 与裁剪后的实际 duty，返回明确的饱和标志；Motor 在提交 PWM 后把该标志填入同步快照；
- 端口没有母线采样能力、结果缩放未知，或调制器不能报告裁剪时，G1 保持失败，禁止启动电感辨识。

母线采样和饱和状态均由 Motor 汇总；Identify 只读取快照，不直接读取 ADC 或调制器内部状态。

### 3.4 ISR 与前台 PT 的批次交接

配置、状态转换和批次复位只由前台 PT 写入；前台在中断保护内完成配置复制、清空 ISR 统计和发布 `bActive`。`bActive` 发布完成前，ISR 不得读取半配置对象。

活动批次期间，ISR 是下列成员的唯一写者：计数器、`Ifirst/Ilast`、电流/电压累加器、首个错误和 `bBatchReady`。累加器可为 64 位，32 位 MCU 上前台不得在未保护的情况下直接读取。

ISR 完成一个完整批次后，按如下顺序操作：写完全部统计与首个错误 → 编译器/中断边界屏障 → 最后置 `bBatchReady`。`bBatchReady`、`bActive`、错误锁存和 ISR/前台共享状态声明为 `volatile`，但 `volatile` 不替代临界区。

前台发现 `bBatchReady` 后，在中断保护内复制完整批次到前台局部快照，清除 `bBatchReady`，再退出临界区计算 `Ld` 或收敛错误。前台不得在 ISR 活动期间原地复位累加器。错误一旦锁存，ISR 不再覆盖它，前台只在安全停止后的下一次启动准备阶段清除。

### 3.5 结果和测试条件

```c
typedef struct {
    uint32_t wInductanceDMicroHenry;
    uint32_t wInjectionFrequencyHz;
    uint32_t wEffectiveVoltageMillivolt;
    int32_t lMeanCurrentMilliamp;
    uint16_t hwCaptureSampleCount;
    uint16_t hwHalfCycleCount;
} identify_inductance_result_t;
```

结果定义为：**指定注入频率、电压幅值、电流工作点和温度下的增量电感**，不是不随测试条件变化的唯一电机常数。若系统有温度采样，测试记录同时保存绕组温度；没有温度采样时由台架记录环境和冷/热机状态。

## 4. 安全准入和 ISR 故障处理

### 4.1 启动前检查

前台子 PT 在开启 PWM 前检查：

- Motor 空闲、无故障，PWM 关闭；
- 电角度对齐有效；
- 台架机械锁止已经按测试流程确认；
- `Rs` 有效，或配置中存在明确的电阻参数；
- 电流基值、母线电压测量和调制度换算标定有效；
- 频率、调制度、限流值和采样窗口全部通过边界校验。
- `hwCaptureDelayCycles` 不得小于
  `FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES + FOC_DCBUS_SAMPLE_DELAY_CYCLES`；
  ISR 还必须拒绝超过 `FOC_DCBUS_MAX_MILLIVOLT` 的实际母线样本，只有这样
  启动阶段的 32 位累加上限证明才成立。

当前硬件若没有可验证的锁轴输入，`Lq` 保持未实现/禁用；一个普通命令行布尔参数不能充当安全联锁。

### 4.2 每周期保护顺序

`_identify_inductance_IsrStep()` 每次进入时，先执行保护，再执行注入和采样：

1. 检查 Motor 故障、母线电压有效性、PWM 饱和状态、电角度有效性和连续转动超限；
2. 计算 `max(|Id|, |Iq|)`，与 `qMaxIdentificationCurrent` 比较；
3. 任一条件异常时，立即调用 `motor_IdentificationAbortIsr()`，并锁存首个 Identify 错误原因；
4. 错误锁存后不再恢复注入，由前台 PT 收敛到 `ERROR`；
5. 只有保护通过后，才更新方波相位或记录样本。

硬件 Break/过流保护保持独立，是最后防线；软件限流不能替代硬件保护。ISR 内禁止日志、动态内存、阻塞、无界循环和最终 `µH` 计算。

## 5. 物理量换算和电感计算

### 5.1 单位关系

调制度必须先经过板级标定关系换算为实际施加电压：

```ini
Vphysical = modulation × measured_dc_bus × modulation_gain
            - calibrated_deadtime_and_device_drop
Iphysical = current_pu × current_base
dt        = (capture_sample_count - 1) / HF_ISR_HZ
slope     = (Ilast - Ifirst) / dt
L         = (Vphysical - Rs × Imean) / slope
```

其中：

- `modulation_gain` 必须与当前 SVPWM/电压标幺定义一致，不能猜测为 1；
- `measured_dc_bus` 必须来自有效、经过缩放的母线电压采样；
- `current_base` 使用 Motor 已配置的电流基值；
- 样本逐 ISR 取得时，`M` 个样本的首末时间跨度是 `(M - 1) / HF_ISR_HZ`，不是 `M / HF_ISR_HZ`；
- `Rs` 使用与当前温度条件一致的辨识值或配置值。

如果母线电压从 12 V 变化到 11 V，计算必须使用对应窗口的实测电压；否则结果会随母线电压同比漂移。

### 5.2 死区、器件压降和饱和

第一版只采用板级校准得到的最小有效电压模型，不实现通用逆变器模型：

- 根据电流方向修正死区和功率器件压降；
- 正、负半周期分别计算，利用对称结果检查残余偏差，但不能用正负平均代替标定；
- PWM 限幅、过调制或调制度被裁剪的半周期直接判无效；
- 有效电压过小、压降修正后符号异常时不输出结果。

在上述换算未标定前，只允许输出原始调试统计量，不允许把结果标为 `µH`。

## 6. 20 kHz 命令和采样时序契约

当前 App ISR 顺序是先执行 Motor，再执行 Identify，因此 Identify 发布的命令不能作用于当前周期：

| 周期 | Motor 阶段 | Identify 阶段 | 样本含义 |
|---|---|---|---|
| `N` | 采集旧 PWM 产生的电流并执行 Motor | 发布新的注入命令 | 当前样本仍属于旧命令 |
| `N+1` | 消费新命令并提交 PWM 更新 | 等待管线延迟 | 不得默认有效 |
| `N+1+PWM_UPDATE_DELAY+ADC_DELAY` | 取得新 PWM 对应的 ADC 电流 | 可开始有效窗口 | 第一份允许进入斜率计算的样本 |

因此：

```ini
pipeline_delay_cycles = 1 + PWM_UPDATE_DELAY + ADC_DELAY
hwCaptureDelayCycles >= pipeline_delay_cycles
hwCaptureDelayCycles + hwCaptureSampleCount <= half_period_cycles
half_period_cycles = HF_ISR_HZ / (2 × injection_frequency_hz)
```

`PWM_UPDATE_DELAY` 取决于 PWM 影子寄存器的装载点，`ADC_DELAY` 取决于 ADC 触发和结果进入 Motor/Identify 的路径，必须按板级实现测量或验证，不能使用含糊的固定猜值。

当前 App 的 Motor→Identify 顺序由 `FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES`
明确声明；目标板若 PWM/ADC 路径改变，必须同步修改该宏和采样延迟宏，并重新
验证首个有效样本。

主机测试使用周期标签模拟上述管线，断言命令切换前、切换周期和延迟窗口内的样本均未进入斜率计算。真机使用示波器/采样跟踪确认 PWM 边缘、ADC 触发和首个有效样本的位置。

## 7. 最小斜率估算器

第一版使用固定窗口首末点差分，不引入通用拟合器。

每个正/负半周期只保存：

- 窗口首个电流 `Ifirst`；
- 窗口最后一个电流 `Ilast`；
- 窗口电流和、实际电压和及样本数，用于计算 `Imean` 和 `Vmean`；
- 注入极性和故障/饱和标志。

每个半周期的有效性条件：

1. 捕获样本数与配置一致，且窗口没有跨越方波边界；
2. `|Ilast - Ifirst| >= qMinCurrentDelta`；
3. 斜率符号与净电感电压 `Vmean - Rs × Imean` 的极性一致，且 `|slope| >= qMinCurrentDelta / dt`；
4. 窗口内没有过流、Motor 故障、母线无效、PWM 饱和、角度无效或转动故障；
5. 计算结果位于项目预先定义的物理范围内。

至少取得 2 个有效正半周期和 2 个有效负半周期。先分别平均同极性结果，再比较正负结果；差异超过测试配置允许值时返回测量不稳定，不发布 `Ld`。

只有真机证据证明首末点差分受噪声影响无法满足重复性要求时，才升级为整数累加的固定窗口线性回归；该升级不属于第一阶段。

## 8. 前台子 PT 和 ISR 分工

### 8.1 Phase 1 子 PT

```text
IDLE
  → CHECK_PREREQUISITES
  → PREPARE_D_AXIS
  → START_MOTOR
  → WAIT_D_CAPTURE
  → CALCULATE_LD
  → SAFE_STOP
  → COMPLETE / ERROR
```

`_identify_inductance_RunPt()` 负责准入校验、Motor 启停、等待 ISR 批次、超时、最终换算和状态收敛。所有等待使用 perfc/MDI tick；超时值由测量总周期和固定余量计算，不使用与注入频率无关的魔法常量。

### 8.2 ISR 工作

`_identify_inductance_IsrStep()` 只负责：

- 每周期安全检查；
- 20 kHz 整数分频产生 d 轴双极性方波；
- 按已验证的管线延迟跳过无效样本；
- 固定窗口统计；
- 设置批次完成或故障锁存标志。

方波不能由 1 ms 前台 `Run()` 产生。第一阶段不包含 q 轴分支；后续 `Lq` 直接复用相同计数器和窗口统计，仅替换注入轴和采样轴。

## 9. 分步实施任务

### Task 0：冻结测量契约

**Files:** `foc/docs/identify-inductance-plan.md`、板级配置文档

- [ ] 明确电流基值、SVPWM 调制度定义和母线电压缩放；
- [ ] 测得/确认 `PWM_UPDATE_DELAY` 与 `ADC_DELAY`；
- [ ] 明确机械锁轴测试流程、硬件 Break 和辨识软件限流值；
- [ ] 在开始编码前通过 G1～G4 评审。

### Task 1：扩展现有同步快照

**Files:** `foc/identify/identify.h`, `foc/identify/identify.c`, `foc/app/foc_app.c`, `foc/motor/motor.c`, `foc/hal/foc_port.h`, 目标板 `foc_port.c`、相关 fake/tests、`foc/modulation/foc_modulation.c`

- [x] 在现有父级 `identify_IsrStep()` 快照中增加 `Id/Iq`、有效母线电压、PWM 饱和、Motor fault、电角速度和角度有效性；
- [ ] 增加 `foc_SampleDcBusMillivolt()`、板级 ADC 缩放、样本周期/年龄及 `FOC_DCBUS_MAX_AGE_CYCLES` 校验；无硬件能力时明确返回不支持；
- [x] 由调制器报告请求 duty 与实际裁剪 duty 是否不同，Motor 将饱和状态写入快照；
- [x] 保持 App 只有一个 Identify ISR 调用点；
- [ ] 增加周期标签/假管线测试，验证一周期命令延迟；
- [ ] 用 fake 覆盖母线采样缺失、过期和 PWM 裁剪；
- [x] 运行现有 FLOAT/FIXED 电阻辨识测试，确认无回归。

### Task 2：最小注入和安全停机能力

**Files:** `foc/motor/motor.h`, `foc/motor/motor.c`, Motor tests

- [x] 前台校验并准备固定边界的辨识注入参数；
- [x] `identify_IsrStep()` 显式接收同一 ISR 内的 `motor_t *`，并只通过 `motor_IdentificationApplyIsr()` 提交已校验的 d/q 命令；
- [x] 提供 `motor_IdentificationAbortIsr()`，原子清零注入、停 PWM、锁存 Motor fault 并更新 Motor 对象状态；
- [ ] 禁止 Identify 直接调用 `FOC_PORT_PWM_SAFE_STOP()` 或其他底层端口；
- [ ] 验证 Stop、Break、Motor fault 和过流均清除注入并锁存错误；
- [ ] 测量 20 kHz 最坏执行时间。

### Task 3：创建电感子文件并实现 Phase 1 `Ld`

**Files:** `foc/identify/identify_inductance.c`, `foc/identify/identify_inductance.h`, `foc/identify/identify.h`, `foc/identify/identify.c`

- [x] 增加 `IDENTIFY_OPERATION_INDUCTANCE` 和对象自持有的 `tInductance`；
- [x] 实现配置校验、d 轴方波、窗口采样、首末点差分和电阻压降补偿；
- [x] 实现 PT 超时、错误锁存、统一 safe-stop 和 `Ld` 结果发布；
- [x] 不加入 `Lq` 分支、通用拟合器或磁链辨识抽象。

### Task 4：已知 RL 模型验证

**Files:** `foc/tests/identify_inductance_test.c`

- [x] 用已知 `R/L`、母线电压、调制度和管线延迟生成电流样本；
- [x] FLOAT/FIXED 均验证正负半周期和 `Rs × Imean` 补偿；
- [ ] 覆盖母线无效/过期、PWM 饱和、电流超限、Motor fault、角度无效、连续转动超限、超时和净电感电压符号异常；
- [ ] 断言电流超限后当前 ISR 内通过 Motor API 清零注入、执行 safe-stop，且 Motor 状态、PWM 标志和故障位一致；
- [ ] 断言切换边缘和 delay 窗口样本不会进入结果；
- [ ] 断言不满足最小 `ΔI`、窗口越界和半周期数量不足时拒绝结果。
- [ ] 用交错的 ISR/前台 fake 验证 64 位累加器只能通过批次复制读取，且首个错误不会被后续错误覆盖。

### Task 5：受控台架验证 `Ld`

- [ ] 使用机械夹具锁住转子，并确认硬件 Break 可用；
- [ ] 从低调制度、低限流开始，逐级验证波形和结果；
- [ ] 与相同频率、幅值、电流工作点和温度下的 LCR/参考测试比较；
- [ ] 连续重复测量并保存实际母线电压、有效注入电压、平均电流和温度条件；
- [ ] 结果稳定后再评审 Phase 3 `Lq`。

### Task 6：延期的 `Lq`

- [ ] 先提供机械夹具及其可执行准入方式；没有锁止条件时软件保持 `Lq` 禁用；
- [ ] 复用 Phase 1 的注入、保护、时序和窗口统计，不复制第二套采样器；
- [ ] q 轴注入时仍每周期同时监测 `Id/Iq`；
- [ ] 按与 `Ld` 相同的 RL 模型、故障注入和台架流程验收。

## 10. 可复现验收标准

### 10.1 主机模型

- [ ] 测试固定记录 `HF_ISR_HZ`、`R`、`L`、母线电压、调制度、管线延迟、采样窗口和半周期数；
- [ ] 对无量化噪声的已知 RL 模型，FLOAT 结果误差不超过 3%，FIXED 结果误差不超过 5%；
- [ ] 改变母线电压但保持模型参数不变时，换算后的 `Ld` 不随母线电压同比漂移；
- [ ] 正负半周期、边缘样本排除、错误锁存和 safe-stop 结果完全可重复；
- [ ] 现有电阻辨识 FLOAT/FIXED 测试全部通过。

### 10.2 真机 Phase 1

- [ ] 测试前写明参考仪器、参考值、允许误差、注入频率、实际有效电压、电流范围、母线范围和温度条件；
- [ ] 初始项目验收目标：同条件连续 5 次 `Ld` 极差不超过均值的 5%，与同条件参考值偏差不超过 10%；若电机或仪器不适用，必须在测试前改写并批准阈值，不能测试后放宽；
- [ ] 全程转子不动、无 PWM 饱和、无软件/硬件过流；
- [ ] 任一故障或超时均不发布新结果，PWM 安全停止，首个错误原因保留；
- [ ] 输出明确标注为对应测试条件下的增量 `Ld`。

### 10.3 `Lq` 准入

`Lq` 不属于 Phase 1 验收。只有机械锁止准入、`Ld` 真机稳定性和 ISR 安全停机全部通过后，才能开启 Task 6。

## 11. 明确不做

- 不使用未实现的位置闭环锁轴；
- 不把“检测到转动后停止”当成 `Lq` 的防转措施；
- 不用调制度直接计算 `µH`；
- 不在前台 `Run()` 产生 100～500 Hz 方波；
- 不新增通用辨识 driver、算法基类或运行时函数指针；
- 不提前实现 `Lq`、磁链辨识或通用线性回归；
- 不修改已经验证的电阻辨识流程逻辑。

## 12. 参考资料

- [ST：PMSM 电机参数测量（测量期间保持转子不动，Lq 测量需锁住转子）](https://www.st.com/content/dam/technology-tour-2017/session-2_track-6_advanced-bldc-motor-drive.pdf)
- [TI InstaSPIN-FOC/FAST User's Guide（参数辨识的电流、频率及缩放要求）](https://www.ti.com/lit/ug/spruhj1g/spruhj1g.pdf)
- [TI：电感结果会随测试电流、频率和磁饱和状态变化](https://e2e.ti.com/support/microcontrollers/c2000-microcontrollers-group/c2000/f/c2000-microcontrollers-forum/252953/i-m-seeing-variation-in-ls-measurements)

## 13. Phase 1 实现优化计划（必须执行）

本节针对当前 Phase 1 实现做一次最小化优化，不改变测量公式、保护条件、
父子 PT 结构或已经验证的电阻辨识流程。

### 13.1 运行参数与对象布局

`identify_inductance_cfg_t` 只作为 `StartInductance()` 的输入，不保存在运行对象
中。`_identify_inductance_Start()` 在未激活阶段完成校验、派生参数计算和 Motor
无关参数准备；`_identify_inductance_RunPt()` 在启动 Motor 前绑定当前 Motor 的
电流基值和 Rs；全部准备完成后才在短临界区发布 `operation/RUNNING/bActive`。

运行对象直接保存 ISR 和前台真正需要的执行参数；不额外嵌套
`prepared_t`：

```c
typedef struct {
    uint32_t wHalfPeriodCycles;
    uint32_t wCaptureStartCycle;
    uint32_t wTimeoutMs;
    foc_dq_t atCommand[2];
    foc_scalar_t qMaxCurrent;
    foc_scalar_t qMaxSpeed;
    foc_scalar_t qMinDelta;
    uint16_t hwCaptureTarget;
    uint16_t hwHalfCycleTarget;
    uint16_t hwMotionFaultLimit;
    uint32_t wCurrentBaseMilliamp;
    uint32_t wResistanceMilliohm;
    float fVoltageScale;
    float fSlopeScale;
} identify_inductance_t;
```

`fVoltageScale/fSlopeScale` 只供 FLOAT 前台换算使用；FIXED 前台使用同一批次
快照的 64 位整数比例路径。不能把最终 `Ld` 计算放回 ISR。

### 13.2 ISR 最小化

启动时一次计算并保存：

- `wHalfPeriodCycles = FOC_HF_ISR_HZ / (2 * frequency)`；
- `wCaptureStartCycle = delay`；
- `wTimeoutMs`；
- 正负两个 `atCommand[2]`。

ISR 只读取这些字段，不再执行上述除法、窗口端点加法或正负命令构造。窗口使用
`(phaseCycle - captureStart) < captureTarget` 的无符号区间判断。父级入口已经
完成空指针检查，私有 `_identify_inductance_IsrStep()` 删除重复的三指针检查。

以下工作必须保留：Motor fault、角度、母线、PWM 饱和、Id/Iq 限流、电角速度
连续超限、实际电流/母线累积、首末点记录、半周期统计合并和错误后的立即
safe-stop。窗口计数和运动计数均已在配置校验中有界，删除无意义的
`UINT16_MAX` 饱和分支。

### 13.3 前台结果计算

将 `_Calculate()` 拆成三个有界函数：

1. `_identify_inductance_CopyBatch()`：ready 发布后直接复制完整批次并清除
   ready 标志；ISR 此后不再写统计量，发布和读取分别用内存屏障保证顺序；
2. `_identify_inductance_CalculatePolarity()`：使用一个共同函数处理正、负极性；
3. `_identify_inductance_PublishResult()`：检查两极性结果并发布最终 `Ld`。

FLOAT 后端使用 `float`，不调用 `foc_to_float()` 做逐样本转换，也不使用 `double`。
FIXED 后端使用 `int64_t/uint64_t` 计算平均电流、净电压、斜率和微亨结果；禁止
为了方便把固定点路径提升为 `double`。最终公式保持：  

```text
Vnet = Vpolarity - Rs * Imean / 1000
slope = ΔI * current_base / dt
Ld_uH = Vnet_mV * 1000000 / slope_mA_per_s
```

FIXED 与 FLOAT 都应用 `FOC_DCBUS_MODULATION_NUM/DEN`；完成结果中的注入频率由
`FOC_HF_ISR_HZ / (2 * wHalfPeriodCycles)` 恢复。启动时还须证明
`captureSampleCount * FOC_DCBUS_MAX_MILLIVOLT` 不会超过 32 位窗口累加器。

### 13.4 执行上下文边界

`_identify_inductance_SetError()` 不再同时承担 ISR 和前台职责，拆成：

- `_identify_inductance_FailIsr()`：锁存首错、撤销 active、调用
  `motor_IdentificationAbortIsr()`；
- `_identify_inductance_FailForeground()`：锁存首错、调用正常 Motor 停止和父级
  状态收敛，不调用 ISR-only API。

`Reset()` 先在临界区撤销 `operation/active`，退出临界区后再清零完整电感对象。
启动和 Reset 的大对象赋值不得放在关中断区；临界区只用于发布/撤销跨上下文状态。

App 按 `operation` 构造 ISR 快照：NONE 不构造；RESISTANCE 只传 `Id`；只有
INDUCTANCE 读取母线 ADC 并构造完整快照。`motor_IdentificationApplyIsr()` 接收
已在前台验证的固定命令，只检查动态 Motor 状态后赋值；最终零命令提交失败必须
立即转入 ISR safe-stop。

### 13.5 验证

- FLOAT/FIXED 电阻测试必须继续通过；
- FLOAT/FIXED 已知 RL 电感测试必须继续通过；
- 增加前台失败路径测试，确认不会调用 ISR-only Motor API；
- 增加母线无效、PWM 饱和、过流、运动超限和窗口边界测试；
- 目标板编译后检查电感 ISR 反汇编/符号，确认不再出现半周期整数除法或
  `double` helper；
- 不宣称真实 20 kHz 最坏周期，直到完成目标板周期测量。
