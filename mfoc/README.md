# mfoc

mfoc 是 `dev-focv2` 的新 FOC 库，目录、文件和符号统一使用 `mfoc` 名称。

新库尚未接入当前固件。旧 `foc/` 保留作为参考和对照；
base/Core 不依赖硬件，Motor 通过直接绑定的端口访问采样和 PWM，
库内不包含旧库、MODUS、MDI 或厂商寄存器头文件。

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

电流环计算编排位于独立的 `core/`；SMO、编码器、硬拖、识别、接管、安全状态机和调试统计留到业务流程审核后迁移；它们不进入基础类型和配置头。

## Motor 基础电流控制

`motor/include/mfoc_motor.h` 定义电机本体参数、限值、指令、配置和运行对象。
`mfoc_motor_Init()` 校验并复制参数、初始化 Core，成功后进入 IDLE，
电流指令和故障位清零；失败保留原对象。零初始化对象为 UNINITIALIZED，
不能启用 PWM。初始化必须在硬件未运行时执行，不能用 Init 绕过故障恢复。

| API | 行为 |
| --- | --- |
| `Init` | 校验本体参数、限值及 PI 配置，初始化到 IDLE |
| `Start` | 检查端口就绪，复位积分，提交 50% 中性占空比，再启用 PWM |
| `Stop` | 关断 PWM、清零指令和积分；已有故障继续保留 |
| `SetCurrent` | 检查 dq 参考矢量幅值不超过 i_max，整体更新两个分量 |
| `SetBus` | 更新 Core 母线系数；拒绝时保留原值，不自动停机 |
| `Sample` | 读取完成帧、检查三相电流、Clarke，输出共享 αβ 电流 |
| `Step` | 检查电流矢量和角度有效性、执行 Core、提交 PWM |
| `Trip` | 立即关断并锁存故障；允许高优先级故障 ISR 调用 |
| `ClearFault` | 确认关断且就绪后复位控制器，恢复到 IDLE，不自动启动 |
| `GetStatus` | 一次短临界区取得状态及故障位 |

`Start` 保留已接受的电流指令，`Stop/Trip/ClearFault` 清零指令。
`Sample/Step` 检查实测电流；超限或无效输入锁存故障并请求关断。
`i_max` 同时约束各相瞬时绝对值及 αβ 矢量幅值。
写 PWM、使能或关断失败均锁存 PWM 故障；关断失败不宣称硬件已安全。
硬件过流 Break 仍是独立保护，供电、温度等系统监测由外层调用 Trip。
无效的动态母线测量也必须由外层按安全策略处理。

`motor/include/mfoc_motor_port.h` 声明七个直接绑定接口：
CheckReady、SampleCurrent、SetDuty、EnablePwm、DisablePwm、
EnterCritical、ExitCritical。目标适配文件通过 MDI 实现这些接口，
没有默认成功实现。当前只提供测试端口，尚无 STM32/AT32 实机绑定。
一个编译绑定的功率级由一个应用持有的 Motor 实例独占；不支持两个
活动 Motor 共用这些端口。配置和计算对象可独立实例化。

端口返回已校准、调整极性并归一化为 I/Ibase 的三相完成帧；
ADC 零偏、count-to-PU 系数和帧新鲜度由端口负责。使能前必须保证
中性占空比已真正生效，SetDuty 不得重新使能已关断的桥。
CheckReady 只检查校准、供电和硬件故障条件，不等待外设或自动清故障。
清除物理故障锁存和确认外部故障原因消失由应用/目标适配负责。

前台调用启停、故障恢复、指令更新和查询；Sample/Step 在同一个控制
ISR 中串行执行。SetBus 在控制 ISR 内调用，或停机时在前台调用；
运行期间不得从前台直接更新 Core 的两项系数。运行状态和故障锁存
允许 Trip 中断 Step，最终 PWM 提交前再次检查状态。Core 数学计算
不屏蔽中断；端口临界区必须支持嵌套、保存恢复原屏蔽状态及编译器屏障。
Step 输入必须是同一采样时刻的新电流与电角度，输入/输出不与对象重叠。
失败时丢弃输出；没有额外样本缓存或重用旧结果的后备路径。

典型 ISR 编排为 `Sample -> 外部角度获取 -> Step`，每步检查返回值。
`mfoc_motor_input_t` 只有 αβ 电流、电角度、角度有效标志；
Step 直接输出已有的 `mfoc_core_output_t`。观测器、速度环、对齐和
校准业务流程后续单独接入，不在 Motor 中预留算法状态。

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
一起编译；不接入现有固件。通用 PI 仍在 `base`。

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
- 电机操作层在采样、命令和参数入口统一校验有限性及范围。调制、重建和 `atan2` 按头文件的输入前提计算，不逐层重复验证；返回 OK 不代表数据已通过校验。算法限幅、零向量处理、转换保护和 `mfoc_div_checked` 的检查保留。

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

全量主机测试包含通用 PI；单独运行 PI 测试可使用 `mfoc/tests/run_pi_tests.ps1`。ARM 检查同时编译 PI，并检查 FIXED PI 不引用软浮点辅助函数。

`mfoc.mk` 是可选的源文件清单，当前没有加入任何 target。调用方指定 `MFOC_NUMERIC_*` 和 `base/include` 包含路径后可使用 `mfoc.h`。旧 FOC 与新库使用不同文件名、类型和符号，可在审核阶段并存。

默认三角后端是 LUT。`MFOC_TRIG_BACKEND_LIBM` 仅支持 FLOAT；`MFOC_TRIG_BACKEND_EXTERNAL` 由目标提供两个直接函数 `mfoc_trig_port_sincos` / `mfoc_trig_port_atan2`，不引入运行时 ops 表。首批不复制 G431 CORDIC 驱动，也不宣称已经完成硬件适配。

公式审核、已修正问题和暂缓优化见 [迁移审核](MIGRATION_REVIEW.md)。本轮仅有主机及目标编译证据，没有电机台架性能结论。
