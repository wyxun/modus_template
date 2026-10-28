# MFOC 强拖到 SMO / FLUX 接管设计与实施计划

**状态：原设计记录。2026-10-28 已按审核修正并实现 Motor 接管流程。**

实现采用 `WAIT → REDUCE → BLEND → SPEED`：恒定强拖频率下降低电流，
在控制 d 轴与观察器转子角接近后才捕获剩余偏差。下面原稿的“固定电流
直接消除大角差”和“严格 ±90° 接管判据”已被替代，不再作为实现依据。
当前参数、入口限制、SMO 方向/相位修正及验证边界见 `mfoc/README.md`。
主机检查不代表机械动态、观察器接管及 20 kHz 最坏耗时已经过台架验证。

**Goal:** 在当前 20 kHz 控制流程中实现可验证的强拖启动与观察器接管，并保留以后 HFI 与 SMO / FLUX 配合的位置。

**Architecture:** 独立观察器仍由 Feedback 调度，Motor 在 Feedback 与 Core 之间选择控制角度。采用“捕获一次角度偏差，再逐步归零”，Core 每拍只运行一次；不新建通用融合框架。

**Tech Stack:** C、BAM32、FLOAT / FIXED、现有 OpenLoop / Feedback / Core / PI、mshell、四路 waveform、AITrace。

实施时按下面任务顺序推进；不自动提交代码，不操作电机或烧录。本文伪代码表达设计，新增 API 和状态均是提案。

## 1. 查到的外部做法及选择

| 来源 | 核实到的做法 | 对 mfoc 的取舍 |
|---|---|---|
| TI AM13E230X SDK 启动文档 | 固定 q 轴启动电流、强制角度爬升；进入过渡时记录角差，按斜率消除偏差，最后使用观察器角度 | 采用这个过渡思路。这里查阅的是官方设计文档，不把它说成已审计的开源实现 |
| VESC `mcpwm_foc.c` | 启动阶段覆盖控制角度，并设置磁链观察器内部状态；结束覆盖后使用观察器。HFI 与观察器也有按速度选源逻辑 | 借鉴分阶段角度选择；不照搬向观察器内部写状态的耦合方式 |
| MESC `MESChfi.c` | HFI 有运行时注入开关，并按反电势/速度阈值控制，包含滞回 | 借鉴运行开关与滞回；不照搬其数值 |

来源：[TI 启动流程](https://software-dl.ti.com/mcu-sdk/esd/AM13E2X-SDK/26_01_00_03/exports/docs/docs/am13e230x/user_guide/chapters/solutions/am13e230x/motor_control/control_start_sequence.html)、[VESC 源码](https://github.com/vedderb/bldc/blob/master/motor/mcpwm_foc.c)、[MESC HFI 源码](https://github.com/davidmolony/MESC_Firmware/blob/master/MESC_Common/Src/MESChfi.c)。查阅日期：2026-10-27；master 链接会随仓库更新。

另外，所查 MESC `MESCfoc.c` 的 `OPEN_LOOP_TRANSITION` 分支只有注释和 `break`，不能把这个分支当作已完成的接管实现。[对应源码](https://github.com/davidmolony/MESC_Firmware/blob/master/MESC_Common/Src/MESCfoc.c)

三个候选方案中：直接切角度最短，但有阶跃；同时插值角度、Id、Iq 的调参项较多；本方案只渐变一个角度偏差，电流参考在过渡期间保持不变。**简单依赖明确的坐标约定和经过验证的运行区间，不等于到时间就无条件切换。**

## 2. 当前代码决定了哪些边界

当前主路径位于 `mfoc/motor/src/mfoc_motor.c::mfoc_motor_Step()`：

```text
采三相电流 → Clarke
    → Feedback.Step(i[k], previous_core_output[k-1])
    → _mfoc_motor_SelectReference()
    → Core.Step() → PWM
    → 保存本拍成功提交的 Core 输出
```

- Feedback 已经统一重建上一拍电压；SMO、FLUX 不需要各自再做一次。
- OpenLoop 已有整数角度累加及速度爬坡，继续复用。
- Core 负责 Park、双 PI、限幅、反 Park、SVPWM，继续只调用一次。
- `mfoc_motor_Start()` 会复位电流 PI，不能在运行中的接管点再次调用。
- 现有 `motor openloop <speed> <id>` 使用强拖坐标系的 **Id**；它不是转子坐标系的励磁电流结论。
- 现有 SMO / FLUX 的 `valid` 不是严格的锁定证明，不能单独用它触发接管。
- 当前 HFI 宏开启后，即便没有选择 HFI 后端，也可能执行并输出注入。第一阶段必须关闭 HFI，避免把注入与接管混成一次实验；运行时开关列入后续 HFI 任务。

## 3. 推荐的启动电流约定：从起点消除坐标切换

保留现有 `motor openloop` 的行为。新增自动启动入口：

```c
mfoc_result_t mfoc_motor_SensorlessStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t current);
```

`speed` 是有符号电速度 PU，也是第一版接管后的速度目标；`current` 是正的启动电流幅值。第一版目标速度须处于所选观察器已验证的接管区间。

新流程从静止开始，设 `sign = sign(speed)`：

```text
电流参考：Id = 0，Iq = sign × current
初始控制角度：theta_control = theta_align - sign × 90°
```

按当前 Park / 反 Park 约定，这与 `theta_align` 下的 `Id=current, Iq=0` 产生同方向、同幅值的定子电流参考。例如正转：控制角度 -90°，正 Iq，实际电流矢量指向 0°。

因此新流程可以用这套坐标完成整个固定磁场保持、强拖、接管：

1. 保持初始角度，复用当前对齐时长。
2. 保持同一个 Iq，OpenLoop 从这个角度爬升。
3. 接管期间保持这个 Iq，只改变控制角度的来源。
4. 角差归零后，进入观察器速度闭环。

**这里不是在带电运行中突然旋转坐标 90°。** 初始坐标在 PWM 启动前就选好，期间无需交换 PI 积分、无需增加 Core 的坐标变换 API。

代价：新自动启动的电流环轨迹与旧 Id 强拖不同，需要单独验证。角度连续也不保证整个过渡的实际转矩恒定；过渡速度必须让电流环跟得上。不能声称“数学等价”就免除台架验证。

## 4. 插入位置与最小状态

### 4.1 Motor 只增加启动过渡状态

用枚举替代 `openloop_active`，不要同时保留两套真假状态：

```c
typedef enum {
    MFOC_START_NONE = 0,  /* 正常反馈控制 */
    MFOC_START_FORCED,    /* 原有 openloop 调试，不自动接管 */
    MFOC_START_WAIT,      /* 自动启动：对齐、爬升、等待可接管 */
    MFOC_START_BLEND,     /* 角度偏差逐步归零 */
} mfoc_start_state_t;

typedef struct {
    mfoc_start_state_t state;
    int32_t offset;      /* 有符号 BAM32 角度偏差 */
    uint32_t ticks;      /* 对齐结束后的启动超时计数 */
    uint16_t stable;     /* 连续满足接管条件的拍数 */
} mfoc_start_t;
```

状态属于 Motor；`hold_active` 保留原有固定磁场保持语义。复用已有 `speed_ref`、`command.i_ref`、OpenLoop，不重复存目标角度、目标电流、观察器输出。实际结构尺寸由目标编译确认，不能仅凭字段相加估算 ISR 开销。

稳定配置放在外层配置头：观察器最低接管速度、连续稳定时间、最长启动时间、角差消除速度。常数转 BAM32 步长和计数上限在初始化/启动前完成，不在 ISR 算除法。

### 4.2 ISR 插入点

在 `mfoc_motor_Step()` 中 **Feedback 返回之后、Core 调用之前**，扩展现有 `_mfoc_motor_SelectReference()`。它返回 `mfoc_result_t`，由 Motor 统一执行已有失败停机路径。

```text
Feedback.Step 一次
    ↓
SelectReference：选强拖 / 过渡 / 反馈角度，必要时执行速度 PI
    ↓
Core.Step 一次
    ↓
PWM 一次，成功后更新 previous_core_output
```

观察器不知道强拖状态，不调用 Motor，也不调用另一个观察器。OpenLoop 只生成角度，不接收 SMO / FLUX 对象。

## 5. 最少的接管条件与伪代码

### 5.1 不堆叠判断，但保留必要的失败出口

在 OpenLoop 达到设定速度后，要求以下条件连续保持一小段时间：

- 所选观察器本拍返回有效输出。
- 观察器速度与强拖目标同向且接近：使用一个速度误差窗口即可同时排除反转和明显未跟上。
- 观察器与强拖 **q 坐标角度** 的最短角差小于 90°，排除明显反向/半周错误；不要求几度内一致。

“到达最低速度”由启动入口和爬坡完成条件保证，不再多建一层资格判断。连续计数失败就清零；超过总等待时限，走现有故障停机，不无限强拖、不自动重启。

这些是本方案的工程约束，不是上述项目共同采用的固定判据。速度一致仍不能证明绝对角度正确，所以台架必须先用编码器验证目标观察器。

可用于首轮设计计算的值：稳定时间 20 ms、速度误差窗口为目标的 10%、角差消除速度 1 电周/s、对齐结束后超时 2 s。它们是待验证的起点，不是已验收参数；超时必须大于配置爬坡耗时、稳定时间和最大过渡时间之和，否则初始化拒绝。

SMO 的 160 电 Hz（400 Hz 基值时 0.4 PU，7 极对时约 1371 rpm）来自此前候选验证点，须在当前 mfoc 重测；不能把它直接当成 FLUX 的已验证阈值。

### 5.2 捕获偏差，再归零

以下为算法伪代码。`shortest_bam(a,b)` 表示 a−b 的最短有符号 BAM32 差，`toward_zero(x,step)` 每拍向零移动不超过 step。实际实现要避免有符号溢出；半周边界拒绝接管。

```text
SelectReference(motor, feedback, feedback_rc, input):// 改用swtich
    if state == FORCED:
        if not hold_active: OpenLoop.Step()
        input.angle = openloop.angle
        return OK

    if state == WAIT:
        input.angle = openloop.angle
        if hold_active: return OK
        OpenLoop.Step()
        input.angle = openloop.angle
        ticks += 1
        if ticks > timeout_ticks: return SAFETY

        if ramp_not_finished or feedback_rc != OK:
            stable = 0
            return OK

        error = shortest_bam(input.angle, feedback.angle)
        ready = speed_lower <= feedback.speed <= speed_upper
                and -quarter_turn < error < quarter_turn
        stable = ready ? stable + 1 : 0
        if stable < stable_ticks: return OK

        offset = error
        state = BLEND
        return OK                 // 捕获当拍不削减 offset

    if state == BLEND:
        if feedback_rc != OK: return SAFETY
        ticks += 1
        if ticks > timeout_ticks: return SAFETY
        offset = toward_zero(offset, offset_step)
        input.angle = feedback.angle + offset  // 无符号模一周相加
        if offset != 0: return OK

        rc = PI.Track(speed_pi, command.i_ref.qQ,
                      speed_ref, feedback.speed)
        if rc != OK: return rc
        state = NONE
        mode = SPEED
        speed_count = 0
        // 继续进入下方现有速度环分频逻辑

    if feedback_rc != OK: return feedback_rc
    input.angle = feedback.angle
    if mode == SPEED and speed_count == 0:
        PI.Step(speed_pi, speed_ref, feedback.speed, input.ref.qQ)
        input.ref.qD = 0
        command.i_ref = input.ref
        speed_count = speed_divider
    return OK
```

正常 align 和 HFI acquiring 分支仍按原有职责处理，不能被上述启动分支吞掉。识别/辨识分支的编译边界保持不动。

捕获点满足：`observer_angle + offset == forced_angle`，因此没有人为切角阶跃。之后控制角增量包含观察器本身的增量与偏差消除量；**该方法不会滤掉观察器的跳角**，有毛刺的观察器仍须先修复。

接管过程中保留原 d/q PI 状态，禁止重新 Start、Reset Core、重置观察器或清空上一拍电压。角度只小步变化，PI 依靠现有电流反馈跟踪，不宣称电压矢量每拍严格不变。

### 5.3 速度环衔接

调用顺序与现有 API 一致：`mfoc_pi_Track(&motor->speed, iq, speed_ref, feedback.speed)`。该拍再运行一次正常速度 PI，避免等到下一次分频时输入已变化。

Track 在积分限幅/数值范围允许时复现原 Iq；若预置所需积分被裁剪，不能声称无扰。测试必须覆盖此情况，配置启动电流与速度 PI 限幅时确保接管误差窗口内可预置。必要时缩小速度接管窗口，不能默默放大电流限幅。

第一版不支持运行中反向重启、失锁回退强拖或自动重试。成功后 `motor speed` 只接受已验证的观察器速度范围，同向更新；低于下限或反向请求返回范围/状态错误，停机使用 `motor stop`。

## 6. 需要修改的文件及明确的 API 影响

| 文件 | 计划改动 |
|---|---|
| `mfoc/motor/include/mfoc_motor.h` | 增加 SensorlessStart；用 start 状态替代 openloop_active；更新注释 |
| `mfoc/motor/src/mfoc_motor.c` | 新启动准备、SelectReference 中的过渡；Stop/Trip/Clear 清理启动状态；保持单一 Core 路径 |
| `mfoc/motor/src/mfoc_openloop.c` | 算法不改；用既有 Reset 设初始角，再 SetSpeed；不得顺序颠倒而清掉目标 |
| `mfoc/motor/src/mfoc_feedback.c` | 观察器 Step 和输入输出结构不改；诊断编码器的前台读错不能被误当作当前无感源失败 |
| `mfoc/mfoc_motor_config.h` | 当前电机的接管速度/爬坡/等待/角差斜率参数 |
| `mfoc/mfoc_config.h` | 可选功能宏、选定 SMO 或 FLUX；首轮关闭 HFI 与另一个非目标观察器 |
| `class/mfoc_app.c` | mshell 解析、前台打印状态；复用现有命令串行化，不新增中断屏蔽体系 |
| `class/mfoc_debug.c` | 复用四路通道，按本轮验证目的选量 |
| `mfoc/README.md` | 说明新命令、参数单位、启动与旧 openloop 的区别 |

Core、PI、SMO、FLUX 的公共 API 不变；Feedback.Step 的输入/输出结构不变。当前 Motor/OpenLoop 已直接读取角度等成员，本轮沿用现有用法，不另加机械 getter。

建议命令：

```text
motor openloop 0.4 0.04       原有 Id 强拖调试，保持原行为
motor sensorless 0.4 0.04     新的自动启动并接管，后端由宏选定
motor status                 前台显示启动阶段、当前控制来源和故障
motor stop                   所有阶段统一停止
```

命令仅为设计示例，不代表已执行或已获运行授权。新入口只接受 IDLE、所选后端 SMO/FLUX、HFI 关闭、速度 PI 已配置且参数合规的配置。拒绝时不改变旧命令/PWM。

## 7. 后期 HFI + SMO / FLUX 怎么接

第一版不提前实现动态融合，但约定三个不变点：观察器同层独立计算；只向 Core 提交一个控制角度；只有 HFI 提供高频注入。

```text
电流与上一拍电压
    ├─ HFI.Step            低速启用/过渡期启用
    └─ SMO.Step 或 FLUX.Step   独立计算
             ↓
    Feedback 内的少量选源逻辑
             ↓
    Motor 原有控制参考选择 → Core 一次
```

后续真正接 HFI 时，在 Feedback 内增加运行时关闭状态，Init 不自动注入，HfiStart 才开启，Stop/失败关闭。不能仅清零输出却继续跑完整解调；也不能每拍反复 Init。对上仍返回既有 angle、speed、u_inject_ab、acquiring，不往 Motor 塞两个算法对象的判断。

HFI→观察器同样捕获角差并消除，使用转子 dq 电流参考，不再有旧 Id 强拖的坐标问题。过渡过程中保留 HFI，结束后在完整载波组边界撤销注入并停止 HFI 计算。接管首轮先做单向；确认可靠后才补高低两个阈值的滞回往返。

后续降速返回 HFI 时必须先恢复注入并获得有效角度，再移交角度，不能同一拍开启 HFI 就把角度交给它。此时状态属于 Feedback 的源选择，Motor 不复制一套 HFI 状态机。

有两个当前尚未解决的前提：

1. 极性关闭时的 HFI 可能只确定磁轴而不是完整电角度；必须先解决半周歧义。对齐可以提供初始极性依据，但要在算法边界明确支持，不能默认已有。
2. 当前 `MFOC_HFI_SPEED_MAX=0.05 PU`，400 Hz 基值对应 20 电 Hz；此前 SMO 候选接管点是 160 电 Hz。**尚未证明两者存在共同可靠区间。** 这个空档不能靠拉长角度渐变填补；应先验证 HFI 的速度范围或 FLUX 的低速能力，必要时保留强拖桥接。

因此第一版只实现强拖→单观察器；HFI 的这部分是后续设计约束，不加入本轮状态成员和 API。

## 8. 分步实施及验证

### 任务 1：锁定基线与启动坐标

- [ ] 在当前提交与工作区上记录 FLOAT/FIXED 的现有测试结果，区分原有失败。
- [ ] 在 `mfoc/tests/motor_control_test.c` 增加正反向初始矢量测试：新 q 约定的反 Park 结果应等于旧 d 对齐矢量。
- [ ] 增加拒绝用例：非 IDLE、零速、低于观察器下限、电流越界、HFI 开启、未配置速度 PI；验证无 PWM 副作用。
- [ ] 实现启动入口与状态替换，旧 openloop/align/stop 测试必须保持原语义。

测试核心关系：

```text
for sign in [+1, -1]:
    old = inverse_park({I, 0}, align_angle)
    new = inverse_park({0, sign*I}, align_angle-sign*quarter_turn)
    assert old ≈ new             // FLOAT 容差、FIXED LSB 容差分开
```

### 任务 2：接管逻辑

- [ ] 增加环绕测试：强拖 0.99 周、观察器 0.01 周，偏差取 -0.02 周，不绕远路。
- [ ] 注入间歇 invalid、反向速度、半周错误、未完成爬坡，确认 stable 清零，绝不读失败输出。
- [ ] 验证捕获当拍控制角不变、offset 单调归零且不越零，Core/PWM 每拍恰好一次。
- [ ] 验证过渡 invalid 与超时走现有停机，Stop 在每个阶段都有效，重启无残留。
- [ ] 实现 SelectReference 扩展，保留 `i[k]` 与上一拍成功 PWM 输出的关系。
- [ ] 用现有 PI.Track 接入速度环，并覆盖正反转、积分裁剪、分频首拍。

### 任务 3：接入真实 Feedback 与工具入口

- [ ] 在 `mfoc/tests` 增加 `motor_handover_test.c`，覆盖真实 Feedback 与 Motor 的联接；原 Motor fake 只能证明编排，不能证明观察器有效性。
- [ ] 修改 `run_core_tests.ps1` 收录新测试；组合分别为 encoder 基线、SMO 后端、FLUX 后端，FLOAT/FIXED 都覆盖。
- [ ] 补 mshell 帮助与状态；非目标观察器编译关闭；确认普通 current/speed/align 回归。
- [ ] 检查无效历史电压复位只发生在预定启动边界，接管不触发观察器重新初始化。

现有验证入口（不包含烧录）：

```powershell
powershell -File mfoc/tests/run_pi_tests.ps1
powershell -File mfoc/tests/run_core_tests.ps1
powershell -File mfoc/tests/run_smo_tests.ps1
powershell -File mfoc/tests/run_flux_tests.ps1
powershell -File mfoc/tests/run_app_tests.ps1
powershell -File mfoc/tests/run_debug_tests.ps1
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel
```

预期：新增测试通过、相关回归无新增失败；宏矩阵要以每份 ELF 的实际编译宏为准。文档本轮未运行上述测试，不声称通过。

### 任务 4：20 kHz 耗时与台架

- [ ] 目标反汇编确认热路径没有新增 sin/cos、atan2、sqrt、除法及库调用；偏差归零仅整数比较/加减，速度条件使用预计算上下界。
- [ ] 观察首拍、等待转过渡、过渡结束及速度 PI 同拍的最大耗时；复用当前测量，禁止叠加第二套常驻探针。
- [ ] 用户烧录并明确授权后，分别验证 SMO、FLUX；先编码器闭环影子观测确认候选接管速度，再做短时自动接管。
- [ ] 四路先用编码器角、观察器角、控制角、实际控制坐标 Iq。控制角从该拍局部输入交给已有调试点，不仅为波形新增长期缓存。
- [ ] 再用独立一轮替换辅助通道为相电流/电压限幅状态；不增加通道或采样率。编码器零位与时刻必须明确，不能将强拖角误标成编码器角。
- [ ] 检查 CSV 序号连续、丢帧计数，丢样记录不用于定量比较。比较相同方向/速度/负载的角误差、速度跌落、电流峰值、限幅与成功次数。

当前约 49.2 µs 的启动日志已经接近 50 µs，不可把后续稳态降低当作通过。建议验收目标为最坏控制路径不超过 40 µs，给外围 IRQ 和其他响应保留余量；这是工程目标，不是已测结果。现有测量未覆盖的 IRQ 入口/出口成本须单独说明，不能把函数体最大值当成全 IRQ WCET。

## 9. 本方案的取舍

本轮只增加一个自动启动入口、一组小状态、现有选参考位置的一段过渡逻辑。采用统一启动坐标避免运行中重构电流 PI；采用一次角差捕获避免双角插值与每拍额外三角函数。

先把强拖→SMO 和强拖→FLUX 分别做可靠，再接 HFI 的运行开关与低高速选源。没有已经验证的观察器工作区间时，任何简短接管代码都不能保证成功；本方案不靠新增复杂判断替代这些验证。
