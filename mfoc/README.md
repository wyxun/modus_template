# mfoc

mfoc 是 `dev-focv2` 的新 FOC 库，目录、文件和符号统一使用 `mfoc` 名称。

G431 默认固件已接入停止状态的 mfoc 应用，旧 `foc/` 保留作为参考和对照；
base/Core 不依赖硬件；Motor 通过端口读取电流和提交 PWM。
算法和控制模块不包含旧库、MODUS、MDI 或厂商寄存器头文件，
`port/` 通过 MDI 实现硬件迁移。

计算基座统一归入 `base/` 模块，公开头文件位于 `base/include/`，实现位于 `base/src/`。通用 PI 也属于这个基座；电流环、速度环通过各自的 PI 实例和参数使用同一套控制器接口。

## 首批边界

| 文件 | 内容 | 来源 |
| --- | --- | --- |
| `mfoc_numeric.*` | FLOAT / int32 Q15 标量、饱和、乘除、转换 | `foc/math/foc_numeric.*`，不带旧 PID 的分拆增益类型 |
| `mfoc_angle.*` | BAM32 角度、环绕、角差、atan2、sincos | `foc/math/foc_angle.*` |
| `mfoc_trig.*`、`mfoc_sin_table.inc` | 四分之一波表、插值、编译期后端 | `foc/math/foc_trig*` |
| `mfoc_math.*` | 标量平方根 | `foc/math/foc_math.*` |
| `mfoc_transform.*` | Clarke、Park、逆 Park | 从旧 `foc_core.c` 提取纯变换 |
| `mfoc_modulation.*` | SVPWM、SPWM、理想占空比电压重建 | `foc/modulation/foc_modulation.*` |
| `mfoc_pi.*` | 通用并联 PI、积分/输出限幅、条件积分、状态跟踪 | 独立实现，FLOAT/Q15 共用接口 |
| `mfoc_types.h` | 标量、统一结果码、uvw、alpha-beta、dq 公共类型 | 不依赖数值运算头，三相量统一为 `mfoc_uvw_t` |

电流环计算编排位于独立的 `core/`，位置算法位于 `position/`，当前已实现编码器。
SMO、硬拖、识别和接管留到对应流程审核后迁移；它们不进入基础类型和配置头。

## Motor 基础电流控制

`motor/include/mfoc_motor.h` 定义电机本体参数、限值、指令、配置和运行对象。
`mfoc_motor_Init()` 校验并复制参数、初始化 Core，成功后进入 IDLE，
电流指令和故障位清零；失败保留原对象。零初始化对象为 UNINITIALIZED，
不能启用 PWM。初始化必须在硬件未运行时执行，不能用 Init 绕过故障恢复。

| API | 行为 |
| --- | --- |
| `Init` | 校验本体参数、限值及 PI 配置，初始化到 IDLE |
| `Start` | 复位积分，通过合并 PWM 接口写入 50% 中性占空比并启动 |
| `Stop` | 关断 PWM、清零指令和积分；已有故障继续保留 |
| `SetCurrent` | 检查 dq 参考矢量幅值不超过 i_max，整体更新两个分量 |
| `SetBus` | 更新 Core 母线系数；拒绝时保留原值，不自动停机 |
| `Step` | 读取电流、Clarke、获取反馈、分频速度 PI、Core、提交 PWM |
| `Trip` | 在控制 ISR 或停机状态下立即关断并锁存故障 |
| `ClearFault` | 关断后复位控制器，恢复软件状态到 IDLE，不自动启动 |
| `GetStatus` | 直接读取状态及故障位；跨 ISR 更新时不保证二者同拍 |

`Start` 保留已接受的电流指令，`Stop/Trip/ClearFault` 清零指令。
Motor 负责调用电流端口和 Clarke；端口保证归一化结果有限且可表示。
`i_max` 约束 SetCurrent 接收的 dq 参考；当前 Motor Step 的实测电流圆检查已关闭。
写 PWM、使能或关断失败均锁存 PWM 故障；关断失败不宣称硬件已安全。
硬件过流 Break 仍是独立保护，供电、温度等系统监测由外层调用 Trip。
无效的动态母线测量也必须由外层按安全策略处理。

`port/include/mfoc_port.h` 声明直接绑定的电流与 PWM 数据接口：

```c
mfoc_result_t mfoc_port_current(mfoc_uvw_t *current);
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty, bool start);
mfoc_result_t mfoc_port_pwm_stop(void);
```

`mfoc_port_current()` 读取新鲜且完整的三相 ADC 完成帧，处理 ADC 数据格式、
零偏和极性，用预计算系数换算成有限且可表示的 I/Ibase，结果仅在返回 OK 时有效。
采样不可等待转换完成，不把测量值裁剪到运行限值；顶层控制编排负责
执行 Clarke，再将 αβ 电流交给 Motor 执行电流环。
`mfoc_port_pwm()` 接收有限的 [0,1] 三相占空比，处理极性并转换成硬件比较值后写入。
Core 在有效输入下保证占空比在范围内，端口直接转换，不重复做 finite/范围检查。
`start=true` 先写占空比再由 MDI 使能；`start=false` 仅更新占空比，不改变门控。
启动写入失败不会继续使能。`mfoc_port_pwm_stop()` 通过 MDI 关闭三相输出，保留
ADC 触发。目标板负责确保启动占空比在使能前生效，MDI 错误向上传递。

迁移实现位于 `port/src/mfoc_port.c`，通过目标提供的 `mdi/instance.h` 绑定
`phase_current_completed`、`adc_bus_voltage`、`bridge`、`encoder_angle` 和
`raw_tick`，所有硬件操作均走 MDI。编码器地址由 `encoder_angle` 绑定；
Port 将 AS5600 原始 12 位机械角度转换成 BAM32。
采样使用 `MDI_Sample_ReadCompleted()`，母线使用现有 `MDI_ADC_Read()`
读取稳定的已发布均值；未发布或正在发布时返回失败，不等待、不覆盖输出。
PWM 提交使用 `MDI_PWM_SetDuty()`。校零前通过现有 `MDI_OP` 调用 G431
bridge 的 `_pwm_AnyOutputEnabled` 板级能力，不扩展 modus 公共接口。
该检查覆盖部分相门开启、计数器停止及 Break 生效的情况；
采样触发专用输出不计入相输出。port 不直接访问芯片寄存器。
换芯片时保持上述资源名和能力契约，只需修改 MDI 实现及资源绑定；
电流标度与极性在 `mfoc_port.h` 中配置，零偏由端口初始化采集。
板级完成 ADC 触发配置后、Motor 停止且三相输出门控关闭时，Motor Init 会调用
`mfoc_port_init()`。它阻塞读取十个完成帧，
每次读取前通过 `perfc_delay_ms(10)` 等待 10 ms，然后分别对 U/V/W 求均值作为零偏；
这段时间 ADC 触发必须持续工作，且不能有 ISR/其他调用方读取同一采样资源。
G431 使用注入 ADC 右对齐四倍累加值，`mfoc_port.h` 中的
`MFOC_PORT_COUNTS_PER_BASE` 定义为 476 个累加计数，对应当前 Motor/Core 的
3.5 A Ibase。换板或更改 Ibase 时应调整该宏；`MFOC_PORT_CURRENT_INVERTED`
配置固定电流极性，不通过运行时配置传入。
母线通道按 `20570/2457` mV/count 换算，并以 12 V 为 Vbase 归一化；
`mfoc_port.h` 中的参数需与电压采样分压、Vbase 和测量范围一致。
Init 不会使能桥臂，完成均值计算后
才发布新的偏置和换算系数。
`mfoc_port_bus()` 读取 MDI 最近一次完整发布的母线均值；它不等待新 ADC 转换，
允许多通道均值正在更新时读取前一份已发布值。

未初始化或没有完成帧时，电流端口返回 INVALID_STATE，输出保持不变。
FIXED 使用私有 Q30 倒数，将 ADC 差值转换成公共 Q15，避免每拍除以板级标度；
G431 的 14 位采样下，倒数量化与向零截断的合计误差小于两个 Q15 计数。
测量值不裁剪到 ±1 pu；G431 的 ADC 计数范围及标度保证结果可由标量表示。
PWM 先转换为 MDI 的 Q16 占空比，再由 MDI 换算成 TIM1 比较值。
G431 当前关闭 CCR 预装载，由 MDI 写 CCR1-3，不产生 UG 或改变输出门控。

PWM 写入、启动和停止统一声明在 `port/include/mfoc_port.h`，
由 `port/src/mfoc_port.c` 通过 MDI 实现。校零由 Port Init 完成；启动不再读取
母线或重复校验初始化状态。硬件使能条件由 MDI Enable 负责。
AT32 尚无新库 MDI 绑定。
`mfoc.mk` 已包含此 MDI port 源文件；目标工程需提供相应 MDI 包含路径和
`MFOC_NUMERIC_FLOAT` 或 `MFOC_NUMERIC_FIXED`。G431 已接入停止状态的 mfoc 应用。
一个编译绑定的功率级由一个应用持有的 Motor 实例独占；不支持两个
活动 Motor 共用这些端口。配置和计算对象可独立实例化。

端口返回已校准、调整极性并归一化为 I/Ibase 的三相完成帧；
ADC 零偏、count-to-PU 系数和帧新鲜度由端口负责。使能前必须保证
中性占空比已真正生效。ISR 使用 `mfoc_port_pwm(duty, false)`，不会重新使能
已关断的桥；Motor Start 才使用 `start=true`。ClearFault 只恢复软件状态。
清除物理故障锁存和确认外部故障原因消失由应用/目标适配负责。

前台调用启停、故障恢复和查询；Motor Step 在最高优先级、不可重入的控制 ISR
中执行。SetCurrent、SetSpeed、SetBus 在运行时由该 ISR 调用，停机时可由前台调用。
mfoc 不关闭中断，前台修改由应用串行安排；不支持更高优先级软件 ISR 抢占 Step
并修改同一 Motor。Start 最后发布 RUNNING；Stop 先发布非运行状态，阻止后续
ISR 进入 Core，再关闭相输出并复位指令及积分。硬件 Break 独立关断输出。

热路径只在 Motor Step 入口检查一次 RUNNING；IDLE/FAULT 时不读取电流帧。
Core Step、Clarke/Park、PI Step/ApplyLimit 和 SVPWM 要求有效指针及数值，
直接计算并返回 void，不逐层传播仅用于空指针检查的 result。
GetStatus 通过有效输出指针写入状态，返回 void。
采样失败、反馈缺帧/过期、PWM 写入失败均 Trip；关断错误优先返回。
Motor 只在成功提交 PWM 后缓存当拍 Core 输出，首拍、Stop 或故障后缓存无效。

## Motor 顶层编排

`class/mfoc_app` 按值拥有一个 `mfoc_motor_t`，配置为 `mfoc_app_cfg_t.motor`。
Motor 持有反馈、速度 PI、Core、对齐及故障状态，直接执行采样和 PWM 提交。
板级先完成 ADC 触发和 perfc 初始化，再调用
`mfoc_motor_Init(&motor, &cfg)`。Init 阻塞采集十帧并求 U/V/W 偏置，
初始母线 PU 由 `cfg.core.bus` 提供，不读取母线 ADC、不使能 PWM；
成功后在前台反复调用 `mfoc_motor_Run(&motor)`，至少成功采集一帧编码器数据后
再调用 `mfoc_motor_Start(&motor)`。Run 在停机和运行期间均应继续调用，
编码器内部按配置周期读取；控制 IRQ 只执行 Step，不访问 I2C。

固定磁场对齐使用 `mfoc_motor_AlignStart(&motor, id_pu, angle_pu)`，
只能从 IDLE 启动。`angle_pu` 为 [0,1) 电角度周数，`Iq` 固定为零。
对齐期间 Step 使用固定电角度，跳过反馈和速度 PI，不需要编码器读取成功。
前台 Run 仍持续更新反馈；反馈读取失败不会阻止对齐计时或定时停止。
Run 用 perfc 实例计时槽等待 `MFOC_MOTOR_ALIGN_TIME_MS`（默认 1500 ms，可宏覆盖），
到时调用 Stop；必须持续调度 Run，停机延迟包含一个 Run 调度周期。
显式 Stop 或 Motor 故障均结束对齐；该操作不捕获编码器零位。

Core 持有当前母线 PU 和调制系数。按需在控制 ISR 调用
`mfoc_motor_UpdateBus(&motor)` 读取 MDI 已发布值并更新 Core，不等待新转换。
`speed_divider = 0` 为 dq 电流模式，通过 `mfoc_motor_SetCurrent` 下发指令；
此时忽略速度 PI 参数和反馈速度。`speed_divider = N > 0` 为速度模式，
通过 `mfoc_motor_SetSpeed` 下发速度指令，PI 输出 `iq_ref`，`id_ref = 0`。
速度 PI 输出上下界必须包含零并落在 Motor 的 ±`i_max` 内。
速度参考和反馈均为 `omega_e/omega_base`，等价于
`omega_m/(omega_base/pole_pairs)`；不把 rad/s 或 RPM 直接传入。

控制 ISR 每个电流完成帧调用一次 `mfoc_motor_Step(&motor)`，顺序为：

```text
mfoc_port_current -> Clarke -> mfoc_feedback_Step
                  -> 速度 PI（分频）-> mfoc_core_Step -> mfoc_port_pwm
```

速度 PI 在启动后的第 1、N+1、2N+1…拍执行，其余拍保持电流参考；
`ki_ts` 必须按速度周期 `N * 电流环周期` 配置。
`motor/src/mfoc_feedback.c` 与 `motor/include/mfoc_feedback.h`
提供 Init、Run 和 Step。算法启用与控制输出来源分开：
`MFOC_FEEDBACK_ENABLE_ENCODER=1`（默认）编入并运行编码器；
`MFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_ENCODER`（默认）选择编码器输出。
BACKEND 只选择最终角度和速度，不控制各算法是否运行。
`MFOC_FEEDBACK_NONE` 不提供控制反馈，Step 返回 INVALID_STATE；此时仍可启用编码器。
目前只实现编码器，SMO 等算法按需要另加独立配置、状态和启用宏。

`position/include/mfoc_encoder.h` 与 `position/src/mfoc_encoder.c` 实现编码器算法。
Motor 的 `cfg.feedback.encoder` 配置读取周期、超时、滤波、电角度零偏、方向和
电速度基值 Hz；极对数直接使用 `cfg.params.pole_pairs`。
示例配置如下，基值与零偏必须替换成系统实际配置：

```c
.feedback = {.encoder = {
    .electrical_speed_base_hz = MFOC_SCALAR(100.0f),
    .speed_filter_alpha = MFOC_SCALAR(0.2f),
    .electrical_zero = {0U},
    .sample_period_us = 1000U,
    .timeout_ms = 10U,
    .inverted = false,
}},
```

Run 通过 `mfoc_port_encoder()` 和 MDI `encoder_angle` 读取 AS5600 的机械 BAM32，
按实际读取完成 tick 间隔估算机械速度，完成滤波与电速度 PU 换算后，
通过双缓冲发布完整快照。Step 检查快照年龄并按机械速度外推电角度，
直接复制缓存的电速度 PU；它忽略电流和
上一拍 Core 输入。快照只有一个前台写入者和一个更高优先级 ISR 读取者。
首帧速度为零；读取失败保留旧快照，过期后 ISR 返回错误并锁存 FEEDBACK 故障。
电角度零偏由外部校准后配置；这里不自动对齐。相邻采样须小于半机械圈，
读取完成时间戳不包含传感器内部延迟补偿。

Feedback 输入仍保留本拍 `i_ab_k` 与可选的前一控制区间 Core 输出。
Motor 按实例保存上一拍 Core 输出；首拍传入 NULL，Stop 或故障后清除有效标志。
未来模型算法可通过 duty 和同拍 bus 快照重建电压，不改变 Motor 的采样主流程。

Motor 的 Start、Stop、Trip、ClearFault、GetStatus 直接以 `&motor` 调用。
Stop 统一清零速度指令、PI 积分、分频和对齐状态。故障恢复时先 Stop，再
ClearFault，最后 Start；命令更新与其它写操作保持串行。

中断函数本体位于 `motor/src/mfoc_motor_irq.c`，入口为无参数的
`void mfoc_motor_IRQHandler(void)`。`typedef void mfoc_motor_irq_handler_t(void)`
定义函数签名；GCC/Clang 的 `alias` 把实际向量名映射到同一地址，不经过转发函数。
`motor/include/mfoc_motor_irq_binding.h` 通过 `MFOC_MOTOR_IRQ_*` 的
`VECTOR/INSTANCE/READY/BEGIN/END` 做编译期绑定。G431 向量为
`ADC1_2_IRQHandler`，实例直接绑定 `tMfocApp.motor`，与前台和 Shell 使用同一对象。
READY 判断采样完成，BEGIN 准备帧消费状态，END 清理未消费帧；保留先读电流、
后清 JEOS 的顺序。未运行或采样失败时也执行 END；已消费后新到达的帧保持待处理。
Motor 的 `irq_result` 仅记录最近一次 IRQ 的 Step 返回值用于诊断。

启用时为整个目标定义 `MFOC_MOTOR_IRQ_ENABLED=1`；G431 的 `it.c` 据此排除
原来的 ADC 控制 IRQ 定义。应用 Motor 初始化成功、配置反馈并将故障 IRQ
绑定到同一 Motor 后，再启用硬件控制 IRQ。

当前默认 G431 固件保留停止状态，不启用 Motor 控制 IRQ。
`class/mfoc_app.c` 通过 MODUS 创建应用并直接拥有 Motor；Init 调用 Motor Init，
成功后才注册对象。Run 直接服务 Motor 的前台反馈和对齐定时停止。
`haladc_Init()` 准备 ADC 触发；校零使用轮询，每次等待 10 ms 后读取一次完成帧，
未就绪或读取失败立即返回错误。DMA 和 SysTick 保持开启。
App 不访问 MDI、NVIC 或芯片头文件；TIM1 CH4 触发在板级启动，三相门控保持关闭。
实际控制/故障 IRQ 的硬件开启及波形接入仍待板级集成。

App 已注册 `motor current <id> <iq>`、`motor speed <pu>`、`motor align`、
`motor stop`、`motor clear`、`motor status` 和 `motor help`。
`motor align` 在启用 Motor IRQ 的集成中直接调用 Motor AlignStart，默认
Id=0.04 PU、电角度=0；`motor stop` 直接调用 Motor Stop。
默认停止配置明确报告控制 IRQ 未启用。current/speed/clear 仍只解析参数，
对应执行位置保留 TODO；status 仅显示 App 状态和最近一次前台结果。
当前配置磁链为未测量的 1 PU，编码器零位和方向未校准，仅供停止初始化。

## Motor 标幺参数

极对数 `pole_pairs` 为正整数。`rs/ld/lq/flux` 是正的标幺值，
`inertia` 是非负标幺惯量，零表示尚未提供。物理参数在调用 Init 前换算：

- `rs = Rs * Ibase / Vbase`，Rs 为每相电阻。
- `ld/lq = Ld/Lq * omega_base * Ibase / Vbase`。
- `flux = psi * omega_base / Vbase`。
- `omega_base` 为电角速度基值 rad/s；Hz 需要先乘 2π。
- 采用幅值不变 Clarke、相电流峰值基准，令
  `tbase=1/omega_base`、`omega_m_base=omega_base/pole_pairs`、
  `psi_base=Vbase/omega_base`、
  `torque_base=1.5*pole_pairs*psi_base*Ibase`，
  则 `Jbase=torque_base*tbase/omega_m_base`，`inertia=J/Jbase`。

Motor 与 Core 共用固定的电压、电流基值。Motor 持有运行用参数和 Core，
不另存完整 cfg，不缓存逐拍电流、角度或输出。ADC 原始零偏与标度属于
采样适配层。更改本体参数时，必须同步更新将来依赖它们的算法系数。

## 电流控制 Core

`core/include/mfoc_core.h` 提供 `Init / SetBus / Reset / Step`，由 `mfoc.mk`
一起编译；G431 应用的停止状态接入见上文。通用 PI 仍在 `base`。

- 每拍输入 αβ 电流、电角度和 dq 电流参考。
  三相采样由调用方先调用 `mfoc_clarke`，不重复保存在 Core。
- 电流使用 I/Ibase，PI 输出使用 V/Vbase，母线使用 Vdc/Vbase。
  `ki_ts` 已含采样周期；d/q 两轴独立配置同一种 PI。
- 计算顺序：一次 sin/cos → Park → 两轴 PI → 电压圆限幅 →
  乘以预计算的母线倒数 → 逆 Park → SVPWM。输出限幅后 dq 电压、αβ 调制度、
  dq 电流、占空比及圆限幅/PWM 触轨标志。
- `m_max` 是允许的电压矢量幅值与母线之比，推荐留出 PWM 裕量，
  例如 0.55，最高 1/√3；初始化预计算 `u_max = bus * m_max`
  和 `bus_inv = 1 / bus`，Step 不再计算母线半径和除法。
  当前 Core 要求两轴 PI 输出上下限位于 [-1,1] V/Vbase，
  选取 Vbase 时应覆盖所需的单轴电压范围。
- 圆限幅通过 `mfoc_pi_ApplyLimit` 回写 `ui += u_limited - u`，
  保留积分上下限；它与面向下一拍预装载的 `Track` 不同。
  积分限值过窄时不能保证完全补偿饱和误差。
- FIXED 的圆限幅保留 Q30 平方和，以整数平方根向上取整，
  缩放向零取整；避免先量化平方和造成小矢量限幅失真。
  母线更新拒绝半径或倒数量化为零的配置。逆 Park 和 PWM 仍有量化误差，
  `clipped` 独立报告最终占空比触轨。
- 电机操作层保证初始化、有限值、正母线、有效角度和算术范围；
  Core 不逐层做 `isfinite`，不承担故障停机。输出是下一控制区间的
  请求，不是本拍电流对应的实际施加电压。

固定母线：设置 `mfoc_core_params_t.bus = Vdc/Vbase` 后初始化即可。
动态母线：调用方采样并换算，在新值生效前调用
`mfoc_core_SetBus(&core, bus)`，检查返回值后再执行 Step。
SetBus 不读取硬件、不清空积分；失败保留原状态。调用方必须串行执行
同一实例的 SetBus、Step 和 Reset，防止中断读到只更新了一半的系数。
Reset 清空积分但保留母线系数。Vbase 始终固定。

FIXED 倒数使用 Q15 向零截断，随后乘法也向零截断，与原来的直接除法
不保证逐位一致。在轴电压不超过 1 pu 时，单轴归一化相对精确除法的
绝对误差小于两个 Q15 LSB。SetBus 拒绝无法表示正半径/正倒数的母线值；
母线极低时可表示的半径还会造成较大相对误差，软件范围检查不等于
该工况适合运行电机。
动态母线若每拍更新，仍有每拍一次倒数计算；降低更新频率需要由实际
母线变化速度决定。保护仍应使用实际测量值。

Core 直接调用基座，无运行时分派。加上 Core 入口，默认 LUT
路径的源码调用深度为四层（Step → sincos → LUT → 插值），
不将编译器可能内联视作已经缩短，也不宣称实测执行时间改善。

## 数值契约

- 编译时显式选择 `MFOC_NUMERIC_FLOAT=1` 或 `MFOC_NUMERIC_FIXED=1`，不可同时选择。
- FIXED 是 **int32、15 个小数位**，不是 int16 Q1.15。可表示范围为 [-65536,65536)，一个计数为 1/32768。标幺基值由使用者统一决定。
- `mfoc_mul_wide`：FIXED 使用 int64 中间积、向零缩放、int32 饱和；FLOAT 直接相乘。
- `mfoc_mul_pu` 沿用旧契约：FIXED 先把两个输入裁到 [-1,1]，乘积就近舍入、半值远离零；FLOAT 直接相乘。两者在超出 PU 范围时不等价，调用者不得把宽量传给 PU 运算。
- BAM32：一圈对应 2^32，无符号加法自然回绕；最短角差约定 [-0.5,0.5)，恰好半圈返回 -0.5。
- sin/cos 输出采用所选标量后端；同一控制角每拍计算一次，将结果复用于 Park 与逆 Park。
- `mfoc_angle_from_turns`、`mfoc_from_float` 是边界转换函数；FIXED 的 float 转换将 NaN 转零、正负无穷饱和，FLOAT 标量转换原样返回；角度转换将非有限输入转零。需要拒绝输入的调用者必须在调用前检查有效性，零值不是有效性标记。
- 坐标变换保留原有逐乘积缩放/舍入次序；Clarke 假定三相平衡，alpha=U、beta=(V-W)/sqrt(3)。不自动去除零序。
- 调制输入为相对当前母线的电压调制度，输出占空比 [0,1]；电压模型乘 `bus/baseVoltage`，返回相对电压基值的量，不能把两者混称为同一电压。
- 电流端口保证归一化结果有限且可表示，Motor 校验电流命令和参数；Motor/Core 信任约定内的采样及反馈，不逐层重复验证。调制、重建和 `atan2` 按头文件的输入前提计算，算法限幅、零向量处理、转换保护和 `mfoc_div_checked` 的检查保留。

## 通用 PI

`mfoc_pi_t` 持有配置 `cfg` 和积分输出 `ui`。参数为 `kp`、`ki_ts`、`out_min/out_max`、`i_min/i_max`，其中 `ki_ts = Ki * Ts`，在配置时预乘采样周期。参考值与反馈值单位相同，积分状态和两组限幅均使用输出单位；增益和误差不强制裁到 `[-1,1]`。

`Init` 复制配置并检查限幅顺序，`Reset` 把 `ui` 复位到限幅后的零，`Track` 为下一拍相同输入预置积分状态，`Step` 执行一次 PI。每个实例先成功初始化，同一实例的操作由调用方串行执行。

FLOAT 的核心计算对应以下并联公式，FIXED 使用等价的 Q15 缩放和载体饱和操作：

```c
err = ref - fbk;
up = c->kp * err;
di = c->ki_ts * err;
u = up + p->ui;
/* u 已饱和且 di 继续向外推动时，令 di = 0。 */
p->ui = mfoc_sat(p->ui + di, c->i_min, c->i_max);
out = mfoc_sat(up + p->ui, c->out_min, c->out_max);
```

条件积分依据更新前的 `up + ui` 与增量 `di` 的方向判断，所以允许跨过输出边界一拍，反向增量可退饱和，也支持负增益。跟踪使用 `ui = clamp(target - up - di)`；积分限幅或数值表示不足时不能保证精确跟踪。后续电压矢量或 PWM 限幅的联动抗饱和属于控制流程。

PI 不检查 NaN/Inf，也不检查浮点中间运算溢出。调用方在采样、命令和参数入口保证数值有限且运算范围有效；`Step/Track` 的成功结果不表示输入已经通过数值校验。FIXED 使用 int64 中间乘积、Q15 向零缩放和 int32 载体饱和；小于一个 Q15 计数的积分增量会量化为零。

## 基座内部调用

公开 API 直接完成主要公式，短小的共模计算、占空比限幅和角度编码不再经过单独转发层。通用饱和运算、LUT 插值及定点向量归一化保留复用。

按入口算第 1 层、包含私有函数及 inline 的源码调用关系：SVPWM/SPWM 为 FLOAT 2 层、FIXED 3 层；PI 各接口最多 2 层；角度转换、增量及 atan2 最多 2 层；LUT sincos 为 3 层。标准数学库、目标三角后端及编译器辅助函数内部不计入此范围。编译器可能已内联这些函数，源码层数减少不等于实机周期减少。

## 构建和测试

在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File mfoc/tests/run_tests.ps1
```

测试独立编译新库，覆盖 FLOAT/FIXED、LUT、FLOAT LIBM、直接绑定的外部三角后端、随机及边界数值。迁移对照另行链接旧数学实现，不影响新库依赖边界。ARM 编译及 FIXED 查表无软浮点检查运行 `mfoc/tests/check_arm.ps1`，可通过 `-Toolchain` 指定 LLVM 工具链目录。

全量主机测试包含通用 PI 和 Motor；Motor 测试覆盖采样/Clarke/电流环串联、
速度 PI 分频、停机重启和采样失败关断。IRQ 测试验证无参数入口、向量与 Motor
同地址、先读后清以及停机/故障时的中断清理。单独运行 PI 测试可使用
`mfoc/tests/run_pi_tests.ps1`。ARM 检查同时编译 Motor 和 PI，
并检查 FIXED PI 不引用软浮点辅助函数，同时编译 G431 的实际 IRQ 绑定并验证向量别名。

全量主机测试还包含 `mfoc/tests/run_port_tests.ps1`：使用 G431 的真实 MDI 绑定和
模拟寄存器，覆盖未完成帧、全部 14 位 ADC 码、两种极性和多组换算标度，
以及 PWM 比较值换算、非法定时器状态和门控保持。该测试直接初始化端口内部状态，
不覆盖依赖定时 ADC 帧及 perfc 的阻塞式 Init。ARM 检查编译 FLOAT/FIXED G431 端口，
并确认 FIXED 端口没有软浮点辅助依赖；这些检查不替代实机采样/PWM 时序验证。

`mfoc.mk` 是可选的源文件清单，当前没有加入任何 target。调用方指定 `MFOC_NUMERIC_*` 和 `base/include` 包含路径后可使用 `mfoc.h`。旧 FOC 与新库使用不同文件名、类型和符号，可在审核阶段并存。

默认三角后端是 LUT。`MFOC_TRIG_BACKEND_LIBM` 仅支持 FLOAT；`MFOC_TRIG_BACKEND_EXTERNAL` 由目标提供两个直接函数 `mfoc_trig_port_sincos` / `mfoc_trig_port_atan2`，不引入运行时 ops 表。首批不复制 G431 CORDIC 驱动，也不宣称已经完成硬件适配。

公式审核、已修正问题和暂缓优化见 [迁移审核](MIGRATION_REVIEW.md)。本轮仅有主机及目标编译证据，没有电机台架性能结论。
