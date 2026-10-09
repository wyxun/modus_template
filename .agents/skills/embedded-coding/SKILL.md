---
name: embedded-coding
description: Use when 编写、修改或审查本仓库 src/ class/ peripheral/ foc/ target/ 下的嵌入式 C 代码，涉及 MODUS Class/Driver、Mdriver、perfc-PT 状态机、MDI 硬件接口、FOC 实时路径、ISR 或 grblhal 集成时
---

# Embedded Coding Rules (modus)

## 0. 核心理念（遇到本文未覆盖的情况，按这里推理）

1. **状态有主**：所有可变状态都属于某个对象实例（`xxx_t` 或其按值拥有的成员），所有权、生命周期和并发访问可从类型定义直接看出。文件静态变量不是状态的藏身处。
2. **语义优先于形式**：接口表达"做什么"（`MDI_PWM_SetDuty`、`motor_Start`），带单位、范围和类型；不为统一形式发明 `read/write/control`、`void *` 参数、getter/setter 或空壳模板 API。
3. **编译期绑定，热路径零抽象成本**：能在编译期确定的依赖就不在运行时分派；实时路径没有函数指针链、命令 `switch`、日志或阻塞。
4. **非阻塞且必收敛**：前台流程用对象内的 perfc-PT 推进；每个等待都有超时，每个失败都有安全输出并收敛到 `ERROR`。
5. **简单优先，按需拆分**：只在出现独立生命周期、独立测试入口或独立变化原因时才拆模块；不为"可能复用"预先抽象。
6. **证据驱动**：改行为先有失败测试或可复现步骤；无根因证据不改无关外设；无硬件证据不宣称真机已验证。

## 1. 适用范围与优先级

- 冲突时优先级：L0 MISRA > L1 对象与接口设计 > L2 状态机 > L3 库映射 > TDD。
- 82 字符硬限制**只适用于嵌入式 C/C++ 源码**。Markdown、SKILL、设计文档、报告按语义分段排版，不为凑行宽切断句子。
- `third_party/`（含 grblhal）、厂商代码和生成文件不受 L1 风格约束，但业务代码调用它们时仍受本规则约束。

## L0: MISRA C:2012 安全子集（硬规则）

| 规则 | 要求 | bad | good |
|---|---|---|---|
| 10.3 | 禁止隐式窄化或跨本质类型赋值 | `uint8_t b = a;` | `uint8_t b = (uint8_t)(a & 0xFFU);` |
| 10.4 | 运算/比较两侧本质类型类别一致 | `if (s < u)` | `if (s < (int32_t)u)` |
| 13.4 | 不使用赋值表达式的结果 | `if ((ret = f()) == OK)` | `ret = f(); if (ret == OK) {...}` |
| 13.2 | 表达式结果和持久副作用不得依赖求值顺序（不限于 `++/--`） | `a[i] = i++;`、`x = f() + g();`（f/g 改同一状态） | 拆成按顺序执行的独立语句 |
| 13.3 | `++/--` 不与其他副作用混在同一完整表达式 | `d = buf[i++];` | `d = buf[i]; i++;` |
| 15.7 | if-else-if 链以 else 收尾 | 只有 if/else-if | 增加最终 else |
| 16.4 | switch 必须有 default | 无 default | default 进入错误分支 |
| 16.3 | case 不得 fall-through | case 末尾无 break | 每个 case 显式 break |
| 8.2 | 原型显式声明参数 | `void process();` | `void process(void);` |
| 8.13 | 只读指针加 const | `void read(uint8_t *buf)` | `void read(const uint8_t *buf)` |
| 9.1（项目加强） | 局部变量声明即初始化 | `uint32_t cnt;` | `uint32_t cnt = 0U;` |
| 17.7（项目加强） | 返回值必须检查；错误/状态码不得 `(void)` 丢弃 | `hal_init();`、`(void)hal_init();` | 保存返回值并处理错误；仅非状态类返回值（如 `memset` 的指针）可写 `(void)` |
| 20.1 | include 位于文件开头 | 宏定义后 include | include 先于宏和代码 |
| 20.5 | 不使用 `#undef` | `#undef SIZE` | 独立命名或缩小作用域 |

- 15.5 单一出口默认**不启用**；错误处理允许提前 return。
- 允许的偏差：`memset/memcpy/memcmp/strlen`；DMA 缓冲区所需的 `void *` 指针运算；`third_party/` 内部代码。偏差不适用于 `src/`、`class/`、`peripheral/driver/`、`foc/`。

## L1: 代码与对象设计

### L1.1 源码风格

- C/C++ 源码 82 字符硬限制；超长语句在运算符或逗号后按语义换行。
- 头文件、C 源码预处理宏和 Makefile 的反斜杠续行：反斜杠对齐第 81 列，续行内容缩进 4 空格，不额外补格。
- Makefile 带引号的宏值不得在 `=` 后直接拆行（GNU Make 会把续行展开为空格改变编译参数）；先定义辅助变量再在 `C_DEFS` 中引用，并在 Windows 和 POSIX 目标各验证一次。
- 每个 C 文件有 Doxygen 文件头：`@file`、`@brief`、`@author`、`@date`。
- 每个函数有 Doxygen 说明：`@brief`、`@param`、`@return`；涉及 ISR、并发或错误边界时加 `@note`。
- 注释说明硬件限制、时序/性能考量、副作用、失败条件；算法允许注明公式、单位、离散系数和历史拍关系。不写"给变量赋值""循环遍历"等机械描述。

### L1.2 命名

- 函数 `module_Action()`（如 `motor_Start`）；文件内私有函数加前导下划线 `_module_Action()`；类型（含枚举）用 `_t` 后缀；宏和枚举值全大写。
- 变量前缀是**推荐的可读性约定**，以周边代码为准，不作为接口硬性要求：

| 前缀 | 含义 | 示例 |
|---|---|---|
| `pt` / `p` | 结构体指针 / 其他指针（可与类型前缀组合） | `ptThis`、`ptCfg`、`pchBuf`、`pwTick` |
| `ch` / `hw` / `w` / `l` | 8 / 16 / 32 / 64 位整数 | `chRunPt`、`hwSize`、`wNowTick`、`lTimestamp` |
| `e` / `t` / `b` | 枚举 / 按值结构体 / bool | `eState`、`tDriver`、`bEnable` |
| `s_` | 文件静态对象 | `s_tTemplateClassBase` |

### L1.3 复杂度

- 函数体 ≤ 80 行（大括号内非空、非纯注释行）；一个函数只有一个主要动作和一个变化原因。采样、数学计算、状态迁移、硬件提交、日志、参数解析按职责拆分。
- 参数默认 ≤ 4 个；超出需说明必要性，优先把稳定配置放入配置结构体，不为凑数随意打包。
- 实际调用深度：实时路径 ≤ 3 层；普通非实时路径 ≤ 5 层；仅由非 release 条件编译保护的调试路径 ≤ 4 层（位于实时路径时仍 ≤ 3）。函数指针、回调、宏、inline 展开都计入，不得以"测试没走到"豁免。
- 超限时拆成"入口编排 → 业务步骤 → 叶子操作"；不得只搬代码或用宏规避。

### L1.4 结构体、对象与访问边界

- **对象 vs 纯数据**：具有生命周期、资源所有权、状态转换、不变量、硬件副作用、并发或中断访问约束的结构体是对象，必须提供操作 API 维护不变量和错误返回。坐标、采样值、abc/dq 量、命令、配置快照是纯数据，可公开定义并按值或 const 指针传递，需要时只提供 validate/normalize。
- 结构体单一职责：配置、运行状态、硬件接口、统计、错误上下文不为减少类型数而合并。直接成员 ≤ 16 推荐；17～24 需说明共同职责；> 24 默认拆分（寄存器映射、协议报文、厂商生成结构体可例外并注明）。
- 拆分不得破坏实时路径、增加无意义调用层级或额外拷贝；FOC/ISR 数据还要检查大小、对齐、访问原子性、`volatile` 和缓存布局。
- **访问边界（唯一表述）**：正式业务代码不得读写其他对象或子对象的运行时成员（如 `ptApp->tMotor.eState`、`ptThis->tDriver.eState`），即使 C 头文件中可见。改变状态用 `Start/Stop/Reset/Step` 等语义操作；查询用 `GetStatus()` 等只读快照。配置和采样等纯数据不受限。
- 临时诊断、波形和白盒测试可直接读内部成员，但必须与正式控制路径隔离，不得据此驱动控制命令或安全状态迁移，也不为它们新增 getter。
- 禁止机械 getter/setter；正式业务确需数据时，由对象提供恰好满足该业务的快照或操作接口。
- `.c` 按模块职责和变化原因组织；只有独立生命周期、独立测试入口或独立变化原因才拆成独立模块。

### L1.5 MODUS Class 与 Mdriver（硬规则）

动手前完整阅读 [template_class.h](../../../class/template_class.h)、[template_class.c](../../../class/template_class.c)、[template_driver.h](../../../class/template_driver.h)，以模板为准。

**对象所有权**

- `xxx_t` 是运行时状态的唯一拥有者；`xxx_cfg_t` 是初始化参数和外部依赖的唯一入口，每个字段都必须在 `xxx_Init()` 中被校验、复制或绑定。禁止只有 `chReserved` 等占位字段的配置结构体。
- `xxx.h` 完整声明 `xxx_cfg_t`、`xxx_t`、直接组合的成员类型和公开 API，使组合与 `MODUS_DECLARE_OBJECT` 实例化显式可见；`xxx.c` 只放实现和私有辅助函数。更深层私有实现可放 `Mdriver/<class_name>/` 的私有头文件（`Mdriver/` 目前是**尚未落地的设计约定**，首次使用时需同时接入构建）。
- 对象只能由 `MODUS_DECLARE_OBJECT` 创建。单实例也必须是它生成的 `xxx_t`，不能是"空 MODUS 对象 + 隐藏 `static xxx_runtime_t`"。
- 父 Class 按值拥有其 Driver/Mdriver 成员；每个实例拥有独立子模块状态。多个 Class 共享物理资源时必须用显式 Manager/client/接口注入表达所有权和仲裁。

**静态变量边界** —— Class `.c` 中只允许：

1. MODUS 模板要求的基础对象和基础配置（`modus_base_t` 等）；
2. `const` 查表、只读映射、只读默认参数/ops；
3. 明确声明所有权、生命周期和并发保护的共享资源管理对象；
4. 测试中的 fake、stub 和统计对象。

运行时会变化的任何东西（状态、命令、定时器、时间戳、缓存、PID 积分、编码器状态、错误标志、ISR 统计、Waveform 句柄）都必须在对象内。基类配置若含随实例变化的 `wParent`、缓冲区、依赖或回调，只有严格单实例时才能静态保存；多实例必须每实例独立配置。

> [!WARNING]
> perf_counter 有两个宏在省略指针时使用**调用点级隐藏 `static int64_t`**，多实例共用调用点会互相踩踏（实测第二个实例 300 ms 就收到 500 ms 超时）：`PERFC_PT_DELAY_MS(ms)` 和单参数 `perfc_is_time_out_ms/us(t)`。Class/Driver 中必须显式传入对象成员指针。

两种槽语义不同，不得混用或互相赋值：

| 槽 | 写法 | 保存内容 | 启动方式 |
|---|---|---|---|
| PT 延时槽 | `PERFC_PT_DELAY_MS(ms, &ptThis->lXxxStartMs)` | **毫秒起点**（`get_system_ms()`） | 宏进入时自动写入 |
| 超时槽 | `perfc_is_time_out_ms(ms, &ptThis->lXxxDeadline, false)` | **tick 截止时间**，0 表示未启动 | 进入等待前清零，再调用一次以启动；首次调用只设截止时间并返回 `false` |

**Mdriver 子模块**

- 简单功能直接作为 Class 成员和内部逻辑，不为形式拆分。
- 同时满足"职责独立、较复杂、需要多个操作 API"时才抽成 Mdriver：只服务一个 Class 放 `Mdriver/<class_name>/<submodule>/`；再满足"不依赖父 Class、API 不接收父指针、配置可独立描述、可多实例、可独立测试"时才提升到 `Mdriver/<submodule>/`。相似函数名或预期复用不构成理由。
- Mdriver 是父对象的成员，不是独立 MODUS Class：父 `xxx_t` 含 `sub_t`，父 `xxx_cfg_t` 含 `sub_cfg_t`，父 `Init` 初始化它，父 `Run`/PT 调度它，父 Class 负责停止和处理其错误。只有确需独立注册、生命周期或调度时才用 `MODUS_DECLARE_OBJECT`。
- 子模块 API 以自身指针为上下文（`sub_Run(sub_t *ptThis)`），不保存父指针、不 `extern` 父变量、不承载父 Class 的业务决策。

**初始化契约** —— `xxx_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)` 顺序：

1. 转换并检查对象/配置地址；
2. 校验配置内容；
3. 初始化对象自身状态（`UNINITIALIZED`）；
4. 设置 `ptThis->ptBase` 和基础配置 `wParent`；
5. 绑定配置依赖到对象和子模块；
6. 初始化 PT 游标、业务状态、定时器、事件和子模块；
7. 通过 MDI 初始化硬件依赖，检查每个返回值；
8. 最后 `mbase_Init()` 注册并检查返回值。

- 失败路径：对象保持安全状态，已启用硬件资源关闭，详细错误被锁存。
- 启用 MODUS 时不得用 `(void)wObjectAddr` / `(void)wObjectCfgAddr` 跳过初始化（仅关闭 MODUS 的纯算法分支例外）。`Run`/`Clock` 必须从 `wObjectAddr` 取得真实 `xxx_t` 工作。

### L1.6 Driver 能力模型

`template_driver_*` 是能力模板，不是固定 API。具体 Driver 先选能力，删掉没有语义的示例接口：

| 能力 | 用途 |
|---|---|
| `Init` | 初始化对象和外部依赖（必需） |
| `Run` | 前台非阻塞流程：校准、启动、异步通信 |
| `Start` / `Stop` / `Reset` | 启停、安全输出、故障恢复 |
| `Clock` | 短小固定周期服务、置事件/时间标志；不是第二个 `Run` |
| `IsrStep` | 有严格时间边界的硬件或控制环服务 |
| `GetStatus` | 只读状态快照 |

- 只在 ISR 中执行的固定步长算法（SMO、Flux、EKF）默认只提供参数类型固定的 `xxx_IsrStep()`，不保留 PT、`Run()` 或通用 `sample/apply`。
- `Stop()` 请求依赖进入安全状态并停止本对象服务计时；`Reset()` 只能在安全停止后清除锁存故障和 PT 游标；失败时父 Class 锁存详细错误并进入 `ERROR`。
- Class/Driver 状态至少表达 `UNINITIALIZED`、`IDLE`、`RUNNING`、`ERROR`；状态、错误码、故障位、定时器、PT 游标各自存于所属对象，父子不共享流程状态。
- 复制模板后必须：替换 `template_*` 名称和占位类型；用真实语义替换占位依赖和 ops；删除不需要的回调、状态和示例能力（模板中的 `sample/apply`、通用 ops 和 no-op 依赖只是可运行示例）；为每个配置字段定义校验/复制/绑定；审计所有文件静态对象；提供独立验证入口。

### L1.7 接口、分派与 MDI（硬规则）

- 接口描述一个语义能力族（observer、ADC、PWM、storage），不是通用基类或第二套 Driver 框架。输入、输出和操作函数使用带单位、范围、生命周期说明的类型。
- MDI 使用语义能力接口：`MDI_PWM_SetDuty`、`MDI_PWM_SetFrequency`、`MDI_PWM_Enable`、`MDI_PWM_SafeStop`、`MDI_ADC_Sample`、`MDI_I2C_Transfer` 等。只有 Stream 和 Flash 因数据流/存储语义保留 Read/Write；GPIO、ADC、PWM、I2C 不伪装成字节流。不把 MDI 做成运行时 POSIX 设备系统。
- **分派二选一，同一调用链不得叠加**：
  - `_Generic`：编译期按具体设备类型选择，目标必须是直接函数或 `static inline`；热路径默认用它。
  - `ops/context`：仅用于确需运行时替换的非实时依赖。
- `void *` 只能作为绑定一个明确依赖实例的 `pContext`；禁止用它传递物理量、通道号、地址或命令参数。
- MDI 设备实例保留具体类型；板级 `mdi_hw.h` 静态绑定实现；业务代码不 include vendor HAL、不访问寄存器。
- 返回值只表示状态；读取值和传输长度通过输出参数返回，不复用无定义的 `int32_t`。未实现能力返回 `MDI_STATUS_ENOTSUP`，不得用成功 stub。

### L1.8 FOC 硬件边界（项目当前约定）

- 电流/PWM 硬件契约使用直接语义 API：`foc_SampleCurrent()`（原始三相 ADC）、`foc_SetDuty()`（三相占空比）和 `foc_SampleDcBusRaw()`（原始母线 ADC）。目标 `foc_port.c` 直接调用类型化 MDI，编译期绑定、零运行时分派。
- `motor_t` 不保存 ADC/PWM `ops/context`；Motor 热路径不经过函数指针、通用 `read/write/control`、转发函数、日志或阻塞总线。
- 电流偏移校准、电流归一化和母线 ADC 标定/PU 归一化属于 Motor：`motor_t.tParams.wCurrentBaseMilliamp` 由 `FOC_CURRENT_BASE_MILLIAMP` 初始化；`motor_SampleDcBusPuIsr()` 用板级 ADC 标定和 Motor 电压基值输出母线 PU。App 只编排采样与算法，不重复做物理换算。
- PWM 安全停机、Break 锁存与故障恢复属于非热路径安全生命周期，保留显式安全 API，直到单独设计安全能力接口；不得回流到电流环分派链。
- 新增母线电压、过流输入等能力时，继续新增单位明确的直接 API。

### L1.9 FOC 算法数值边界与可读性

本节是实现和审核约束；基值推导、边界表、特殊格式和 SMO 模板详见 [标幺值、定点和离散时间](../foc-motor-control-methodology/references/per-unit-fixed-point-and-discrete-time.md) 第 8 节。它约束后续整理，不代表现有实现已经全部满足。

1. **统一标幺契约**：控制链使用同一套电压、电流、电角速度及派生基值；明确相量/线量、峰值/RMS、坐标、方向和采样拍。算法公共标量默认用 `foc_scalar_t` 表示约定基值下的 pu，角度统一用 `foc_angle_t`；计数、状态和布尔值不标幺化。Hz、rad/s、turn/s 不得冒充速度 pu。
2. **物理换算集中在边界**：电流 ADC 原始码在 Motor 采样入口扣每相零偏并归一化；母线 ADC 原始码由 Motor 的母线采样 API 按板级标定换算并归一化；用户物理指令在指令入口转换；电机物理参数在初始化统一转成标幺模型并生成离散系数。模型参数安全更新时重算相关系数，运行递推不重新读取物理参数或反复换算基值。可以代数合并初始化计算，不为保留中间结果重复存储或增加调用层。
3. **调制与硬件边界**：调制依据电压 pu 和 Motor 提供的实时母线 pu 求占空比；电压 pu 不等于占空比。硬件端口只把占空比转换为定时器计数。显示/诊断出口的物理换算不得回流驱动控制。
4. **格式统一，例外有据**：算法内部默认沿用公共标量格式；乘积、积分或低速步进确需精度/范围时允许局部加宽或特殊 Q 格式。注明用途、存储缩放关系、有效范围、舍入、溢出和转换位置，禁止特殊格式跨模块传播。必要的数值缩放不属于重复物理换算；不能仅凭格式数量删除精度状态。浮点后端默认 `float`，初始化可按精度需要使用 `double`，ISR 使用 double 必须有必要性和目标时序依据。
5. **公式命名**：算法局部量和明确限定在算法内的状态/系数使用数学名，如 `i`、`u`、`i_hat`、`e_hat`、`z_prev`、`err`、`ku`、`kr`、`b0`、`a1`；不强制类型前缀。定义旁说明符号和单位；边界参数用清楚的语义/单位名。禁止用 `part0/part1` 等无语义短名，特殊格式状态不能仅靠 Q 后缀表达含义。
6. **按递推顺序阅读**：核心函数按模型、积分/限幅、切换/滤波、历史更新组织，源码可直接对照离散公式；明确当前拍和上一拍，历史值在最后按原时序更新。初始化校验和系数生成与每拍计算分开，限幅与故障处理就近可见，不为每个乘加拆函数。
7. **依赖保持窄小**：算法只接收自身配置及有单位约定的纯数据，状态属于实例；不依赖父对象、硬件、日志或启动接管策略。拆分以独立职责为依据，不为解耦增加 getter、函数指针、宏包装或通用算法框架。
8. **后端表达数值语义**：浮点核心优先直接表达公式，定点核心显式保留宽乘、缩放、饱和和舍入；共用公式与接口契约，不为去重强行遮住公式。初始化预计算稳定系数，每拍仅做必要递推；内联、FMA 或运算重排的收益以目标编译/时序证据为准。
9. **重构验收**：看核心函数能对照公式，看头文件能确定单位、状态归属和拍关系，热路径没有无关依赖。保持递推时序、限幅、异常收敛和数值结果；定点检查逐拍一致性，浮点事先定义误差容限并检查边界。模型、反馈路径、滤波方法或限幅行为改变须作为独立算法变更验证，不混入可读性整理。

### L1.10 Motor / Position / Observer / Core 边界（硬规则）

接口字段和本仓库拍关系详见 [FOC 模块架构](../foc-motor-control-methodology/references/foc-module-architecture.md) 第 7 节；修改这些模块时必须同时核对该契约。

1. **职责单向**：Motor 管生命周期、ADC 归一化、运行参考及 PWM 提交；Position 管传感器/强拖/估计角度的选择、补偿与接管；Observer 选择具体估计算法；SMO、磁链观测器、EKF 等只计算模型。Motor 不 include、调用或检查具体观测算法；算法和 Observer 不依赖 `motor.h`、App、端口、驱动、日志或启动状态。
2. **最小数据接口**：初始化只传实际需要的电气模型及算法配置，安全更新时重算系数；Step 只传同步的标幺采样，返回角度、速度、有效性及有明确含义的质量证据。禁止传父对象指针、保留临时输入指针、从父对象取参数或新增未使用的通用槽。算法内部状态、增益、协方差、历史量不得成为 Motor 的控制依据。
3. **Core 输入输出契约**：Core 接收命令、当前 alpha-beta 电流、有效电角度及速度 pu；输出电流、电压模型、占空比和自身限幅结果。输入输出是纯数据，Core 的 PI/积分/历史状态归 Core，Motor 不访问其内部成员。Core 不做采样、选择角度源、控制硬件或迁移 Motor 状态；错误返回由 Motor 收敛到安全状态。
4. **拍与单位不可隐式改变**：采样快照必须明确 `i[k]`、控制区间模型电压 `u[k-1]` 及运行代次；当前 ISR 先 Prepare、取反馈、执行 Core，再推进 Observer，估计用于下一拍。禁止把本拍新算的电压配给本拍已采完的电流。请求电压、模型电压、实际施加电压和占空比不得混称；所有来源共用电流、电压、速度基值。
5. **算法可替换，策略有归属**：后端在 Observer 编译期选择，未实现后端返回禁用/错误，不返回假成功。接管资格、启动超时、失效停机由 Position/Motor 处理；算法只报告自身估计有效性。SMO 的 BEMF 阈值不能直接套用到 EKF 协方差或磁链置信度；不同质量量必须明确类型、单位和消费者，再扩展接口，不能异义复用字段或下放到 Motor。
6. **边界验收是变更要求**：相关改动必须运行 `foc/tests/run_foc_module_boundary_test.ps1`、受影响模块测试及目标构建。源码守卫检查直接依赖和调用顺序；类型检查、逐拍对照、失效/复位和接管测试补足语义验证，不能以守卫通过代替数值或硬件验证。共享类型只能承载数据，不能经此引入算法或硬件依赖。

## L2: 状态机

选择顺序：**perfc-PT > 简单叶子 `switch` > PLOOC**。PLOOC（`modus/lib/plooc`，`__IMPLEMENT`、`__new` 等宏）仅用于确需封装、继承和多实例的领域对象，如电机或传感器 Manager：Manager 结构体第一个成员是基类，实例用 `__new` 创建。

### 通用约束（唯一表述）

1. 前台业务由 `Run()` 驱动，PT 游标（`uint8_t chXxxPt`）和业务状态（`eState`）分开存放在所属对象中；父子各自拥有游标。
2. 禁止 `HAL_Delay`、`perfc_delay_ms`、空循环和阻塞轮询；PT 中用 `PERFC_PT_WAIT_UNTIL` / `PERFC_PT_DELAY_MS(ms, &成员)`，超时判断用 `perfc_is_time_out_ms(ms, &成员, false)`，槽规则见 L1.5 警告。
3. 每个 `PERFC_PT_WAIT_UNTIL` 的条件都必须包含对象内超时槽，超时后进入 `ERROR` 并返回 `fsm_rt_err`；`PERFC_PT_DELAY_MS` 本身有界，不需要额外超时。默认超时 500 ms；状态枚举至少含 `IDLE` 和 `ERROR`。
4. 明确流程语义：一次性流程以 `PERFC_PT_END()` 结束并在完成时返回 `fsm_rt_cpl`；循环流程用 `while (1)` 包住，每轮 `PERFC_PT_YIELD(fsm_rt_cpl)` 一次。不要在 `YIELD(cpl)` 后紧接 `END()`，否则下一次调用会重复报告完成。
5. 状态迁移只在状态机内部发生；与 ISR 共享的迁移加中断保护或等效调度保护。Shell 只提交命令，不改对象成员。
6. `Clock()` 不承载流程。高频 ISR 内禁止任何日志。

### perfc-PT 写法（以 template_class.c 为准）

PT 由 `switch/case` 宏实现，因此：**局部变量在 yield/wait 之后失效**（需跨越的值放对象成员或在恢复后重新读取）；PT 体内**不得再写 `switch`** 包住 yield/wait；函数返回值须兼容 `fsm_rt_t`（等待宏返回 `fsm_rt_on_going`）；`PERFC_PT_BEGIN` 之前的守卫可以提前 return。

以下示例为循环流程：等待计数就绪（500 ms 超时进 `ERROR`）→ 稳定 200 ms → 每轮返回一次 `fsm_rt_cpl` → 下一次调用开始新一轮。示例已用真实 perf_counter 头文件做过编译检查（Doxygen 从略）。

```c
#include <stdbool.h>
#include <stdint.h>
#include "perf_counter.h"
#include "perfc_task_pt.h"

#define PUMP_READY_COUNT        10U
#define PUMP_WAIT_TIMEOUT_MS    500U
#define PUMP_SETTLE_MS          200

typedef enum {
    PUMP_STATE_UNINITIALIZED = 0,
    PUMP_STATE_IDLE,
    PUMP_STATE_RUNNING,
    PUMP_STATE_ERROR
} pump_state_t;

typedef struct {
    pump_state_t eState;
    uint8_t chRunPt;            /* PT cursor, 0 = flow start */
    uint8_t chCount;            /* written by pump_AddCount() */
    int64_t lWaitDeadline;      /* timeout slot: tick deadline, 0 = disarmed */
    int64_t lSettleStartMs;     /* PERFC_PT_DELAY_MS slot: start time in ms */
} pump_t;

void pump_Init(pump_t *ptThis)
{
    ptThis->eState = PUMP_STATE_IDLE;
    ptThis->chRunPt = 0U;
    ptThis->chCount = 0U;
    ptThis->lWaitDeadline = 0;
    ptThis->lSettleStartMs = 0;
}

static void _pump_ArmWaitTimeout(pump_t *ptThis)
{
    ptThis->lWaitDeadline = 0;
    /* First call on a zero slot only stores the deadline and returns false. */
    (void)perfc_is_time_out_ms(PUMP_WAIT_TIMEOUT_MS,
                               &ptThis->lWaitDeadline, false);
}

static bool _pump_IsReadyOrTimedOut(pump_t *ptThis)
{
    bool bReady = (ptThis->chCount >= PUMP_READY_COUNT);
    bool bTimedOut = perfc_is_time_out_ms(PUMP_WAIT_TIMEOUT_MS,
                                          &ptThis->lWaitDeadline, false);

    return (bReady || bTimedOut);
}

fsm_rt_t pump_Run(pump_t *ptThis)
{
    if (ptThis->eState == PUMP_STATE_ERROR) {
        return fsm_rt_err;
    }

    PERFC_PT_BEGIN(ptThis->chRunPt)

    while (1) {
        ptThis->eState = PUMP_STATE_RUNNING;
        _pump_ArmWaitTimeout(ptThis);
        PERFC_PT_WAIT_UNTIL(_pump_IsReadyOrTimedOut(ptThis))

        if (ptThis->chCount < PUMP_READY_COUNT) {
            ptThis->eState = PUMP_STATE_ERROR;
            ptThis->chRunPt = 0U;
            return fsm_rt_err;
        }

        PERFC_PT_DELAY_MS(PUMP_SETTLE_MS, &ptThis->lSettleStartMs)
        ptThis->chCount = 0U;
        ptThis->eState = PUMP_STATE_IDLE;
        PERFC_PT_YIELD(fsm_rt_cpl)
    }

    PERFC_PT_END()

    return fsm_rt_cpl;
}
```

要点：`_pump_ArmWaitTimeout()` 位于 WAIT 入口之前，因此只在每轮进入等待时执行一次；超时后清零游标并锁存 `ERROR`，由 `Reset()` 类接口负责恢复。

### 简单叶子 switch

只用于没有等待层次的叶状态机，不得用来绕开 Class 的 PT 约束。进入等待状态时清零超时槽并立即调用一次以启动计时。

```c
#define SM_RUN_TIMEOUT_MS   500U

typedef enum {
    SM_STATE_IDLE = 0,
    SM_STATE_RUN,
    SM_STATE_ERROR
} sm_state_t;

typedef struct {
    sm_state_t eState;
    int64_t lDeadline;          /* timeout slot: tick deadline, 0 = disarmed */
} sm_t;

static void _sm_Run(sm_t *ptThis, bool bStart, bool bDone)
{
    switch (ptThis->eState) {
    case SM_STATE_IDLE:
        if (bStart) {
            ptThis->lDeadline = 0;
            (void)perfc_is_time_out_ms(SM_RUN_TIMEOUT_MS,
                                       &ptThis->lDeadline, false);
            ptThis->eState = SM_STATE_RUN;
        }
        break;
    case SM_STATE_RUN:
        if (bDone) {
            ptThis->eState = SM_STATE_IDLE;
        } else if (perfc_is_time_out_ms(SM_RUN_TIMEOUT_MS,
                                        &ptThis->lDeadline, false)) {
            ptThis->eState = SM_STATE_ERROR;
        } else {
            /* keep waiting */
        }
        break;
    case SM_STATE_ERROR:
        break;
    default:
        ptThis->eState = SM_STATE_ERROR;
        break;
    }
}
```

### 硬实时 ISR 边界

FOC 电流环等固定时限路径必须满足明确的执行时间、重入和上下文安全约束。ISR 中使用 PT 不一刀切禁止，但必须给出证据：调用链与循环上界、最坏执行时间或时序预算、PT 游标的并发所有权、依赖回调的上下文安全。ISR 不得有无界循环、等待硬件/总线完成、时延不确定的回调、日志或 I2C；所需状态仍属于对象，不得转移到隐藏静态 runtime。

## L3: 库映射（禁止重复造轮子）

| 场景 | 必须使用 | 禁止 |
|---|---|---|
| 对象注册 | `MODUS_DECLARE_OBJECT` | 手写注册表或自造对象框架 |
| Shell/日志 | `MODUS_SHELL_CMD`、mdebug `MLOG/MLOGF` | 裸 printf 自造日志协议 |
| 等待/延时 | `PERFC_PT_WAIT_UNTIL`、`PERFC_PT_DELAY_MS`；必须忙等的硬件微秒时序用 `perfc_delay_us` | `HAL_Delay`、`vTaskDelay`、空循环 |
| 超时判断 | `perfc_is_time_out_ms/us()` | 手写毫秒计数比较 |
| 硬件访问 | MDI：`peripheral/<chip>/` 适配，`peripheral/driver/` 芯片无关驱动 | 业务代码 include vendor HAL 或寄存器 |
| 调试 | `tools/aitrace.exe` + RTT | 裸串口自造 AI 调试协议 |
| 运动控制 | grblhal `mc_`、`protocol_`、`settings_` API；板级适配在 `target/<chip>/` | 业务代码调用 grblhal 内部实现 |
| 共享物理资源 | 显式 Manager、client 或接口注入 | 多个 Class 偷偷共用静态资源 |

### mdebug 日志级别

| 级别 | 场景 | 示例 |
|---|---|---|
| `T` | **周期性/重复性**输出，运行期可单独开关（`log` 命令 bit4） | 每秒 ISR 计时报告、心跳、波形状态轮询 |
| `I` | **触发式一次性**事件 | `motor start accepted`、`encoder cal done` |
| `W` | 可恢复异常 | `motor start rejected (not IDLE)` |
| `E` | 不可恢复错误 | `init failed`、`HardFault` |

周期性输出写成 `I` 会淹没日志且无法关闭；一次性事件写成 `T` 会默认不可见。

## TDD：独立验证

- 每个业务模块有可单独编译运行的测试入口（参考 `foc/tests/`、`peripheral/template/tests/`）；硬件依赖可注入 fake/stub，测试不依赖真实硬件。无法自动化时提供可执行验证脚本。
- 改行为先补/改测试再改实现（红 → 绿 → 重构）；测试接口保持简单，不为测试过度设计。
- 每个 Class 至少覆盖：空指针与配置校验、基类绑定、依赖注入、PT 推进、超时与错误收敛、初始化失败后的安全状态。
- 每个 Mdriver 有独立测试；父 Class 有组合测试验证子模块实例互不污染；多 Class 共用 Mdriver 时覆盖两个独立实例；共享资源时覆盖 Manager 的所有权与仲裁。

## Agent 执行流程

1. 读 `.agents/AGENTS.md` 确认构建、调试、硬件和提交权限。
2. 读模板（`class/template_*`），再读目标模块 `.h`、`.c` 和测试。
3. 画对象所有权：`xxx_t`、`xxx_cfg_t`、每个子模块成员、每个外部依赖；列出 `.c` 中全部 `static` 对象并归入 L1.5 允许类别，无法归类的可变静态对象即设计缺陷。
4. 判定子功能归属：Class 成员 / Class 私有 Mdriver / 可复用 Mdriver / 独立 MODUS Class。
5. 选择状态机方式和库映射；前台默认 perfc-PT，ISR 按硬实时边界处理。
6. 改行为前建立最小失败测试或可执行复现。
7. 自查：MISRA、82 字符、Doxygen、返回值、函数长度、调用深度、结构体职责、访问边界、静态状态、PT 游标、延时槽与超时槽所有权。
8. 运行 Class 和 Mdriver 的独立测试，并用 `.\make.bat` 完成目标构建；release 构建确认调试代码已被预处理移除。
9. 按风险做硬件验证；无硬件证据不宣称真机行为已验证。

## 拒绝清单（出现任一项，停止堆叠修改，回到对象设计）

**状态与所有权**
- 为快速通过编译把运行状态、定时器、时间戳或统计放进文件 `static`，或省略对象成员指针调用 `PERFC_PT_DELAY_MS` / `perfc_is_time_out_ms/us`；
- PT 等待没有对象内超时槽和到 `ERROR` 的收敛路径；
- `xxx_t` 只有 `ptBase`/占位字段而真正状态在隐藏 runtime；单实例借口下 MODUS 对象成为空壳；
- `xxx_cfg_t` 只有 `chReserved`，或存在未被 `Init` 使用的配置字段；
- `Init` 只做 `ptBase` + `mbase_Init()`，没有初始化对象成员、依赖和子模块；启用 MODUS 时 `(void)wObjectAddr` 跳过初始化；
- 父对象或全局变量保存多个子模块的 PT 进度；多个 Class 通过隐式全局对象共享资源。

**接口与分派**
- 正式代码直接读写子对象运行时成员；
- 模板占位 API、no-op 依赖或不适用能力在具体模块中残留；
- 算法通过 `void *` 或无类型参数传递物理量、输入或输出；
- 同一调用链同时使用 `_Generic` 和 `ops/context`；热路径出现函数指针链、命令 `switch` 或日志。

**流程与范围**
- 多步骤业务没有用对象内 PT；`Clock()` 承载复杂或阻塞流程；
- `Stop()`、`Reset()` 或初始化失败路径没有定义安全输出和错误收敛；
- 简单功能被过度拆成 Mdriver，或复杂可复用功能散落在 Class `.c`；Class 专属子模块被注册成独立 MODUS 对象；
- 为修 Class 业务问题而无根因证据修改 ADC、PWM、定时器或其他无关外设初始化。
