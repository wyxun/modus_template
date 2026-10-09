# SMO 算法模板整理记录

日期：2026-10-09。依据 embedded-coding L1.9 和标幺参考文档第 8 节。本次保留等电感 SMO 的模型、积分、切换反馈和反电势滤波；速度接口明确改为共享基值下的 pu。

## 变化与理由

| 整理前 | 整理后 | 理由 |
|---|---|---|
| 初始化换算与递推在 `foc_smo.c` 中混排 | 初始化系数函数与每拍递推函数保留在同一 `foc_smo.c` | 一个算法模块只有一个实现文件，初始化/运行职责仍由函数边界区分 |
| SMO 初始化依赖完整 `motor_params_t` | 使用五个必要物理字段的模型；随后 Motor 整理统一为共享 `foc_electrical_model_t`，Position 在初始化适配 | 算法不再 include Motor；配置不在运行路径读取 |
| 长状态名与系数名遮住公式 | `i_hat`、`e_hat`、`d_prev`、`z_prev`、`ku`、`kr`、`b0`、`a1`、`h`、`i_limit` | 类型定义旁标注符号和单位，源码直接对应公式 |
| 浮点乘加也经通用数值函数 | 浮点直接表达公式；定点保留宽乘、饱和及截断 | 可读性与定点数值安全分别表达，不增加宏语言 |
| `qElectricalSpeedTurnsPerSecond` 输出 Hz，Position 多处乘归一化增益 | `qElectricalSpeedPu`；初始化合成 `speed_scale`，Position 直接消费 pu | 消除控制模块之间的有量纲速度传递和重复换算 |
| Position 保存 `qObserverSpeedGain` | 删除该系数，初始化向 SMO 提供同一个电气速度基值 | 避免两处独立定义速度尺度 |
| 部分 SMO 测试因早期模型变更而跳过 | 更新过期断言及源码位置，恢复执行 | 不以跳过测试作为验证成功 |

## 边界和公式

ADC 校准及归一化仍属于 Motor。SMO 输入为电流、电压 pu，电压为前一个控制区间的模型电压 `u_model`，尚未重构裁剪/死区/管压降后的实际施加电压；状态由实例持有，不保存父对象或硬件依赖。初始化可直接代数合并标幺推导，不额外保存一份无人使用的 R/L 标幺结构：

`ku = Ts*Vb/(Ib*Ls) = dt_pu/l_pu`；`kr = Ts*Rs/Ls = r_pu*dt_pu/l_pu`。

`b0 = wc*Ts/(2+wc*Ts)`；`a1 = (wc*Ts-2)/(2+wc*Ts)`；`h = sliding_voltage/Vb`。

`speed_scale = 1/(Ts*f_base)`，与角差的圈数相乘产生电气速度 pu。Position 从自身配置注入该基值，不使用第二套默认值。SMO 单独初始化的调用方必须明确提供它。

未触发限幅和冻结时，单轴递推为：

```text
d     = ku*u - kr*i_hat - ku*z_prev
i_hat = i_hat + (d + d_prev)/2
err   = i_hat - i
z     = h*sign(err)
e_hat = b0*(z + z_prev) - a1*e_hat
```

`d` 已含 Ts，是增量输入，不是有量纲导数。冻结解除所在拍仍不积分，下一拍恢复。电流修正继续使用上一拍原始切换量，不替换成滤波后的反电势。历史在旧值消费后写回。Clang 的浮点核心关闭乘加收缩，保留原分步运算舍入。

速度接口变化属于明确的边界调整：定点从“先生成 Hz，再乘量化的 1/f_base”变为初始化生成归一化系数，不保证与旧 Position 的两次量化逐位相同。新的输出应对照角差、Ts 和基值验证，而不能要求保留旧倒数系数的量化误差。

## 验证

- FLOAT/FIXED：SMO 基础、切换反馈、Observer 接口、Position 无感接管、Motor 速度 pu、Motor Clarke 链路、App/Shell 测试通过；补充了速度基值为零/NaN、无效输入复位检查。
- 定点初始化、核心契约和软件 atan2 检查通过。
- 临时旧版/新版对照程序：两个后端各执行 50,000 拍相同输入，包含周期复位、正负激励和冻结；电流估计、反电势、增量历史、切换历史、冻结状态、角度和信号强度逐拍一致。主机对照使用 `-O2 -ffp-contract=off`。
- 测试参数 2.55 Ω、860 µH、12 V/3.5 A、20 kHz、400 Hz 基值下，新速度输出对照“旧 Hz 输出 / 400”：浮点最大绝对差约 `1.90735e-6 pu`；定点该组输入差为 0。该结果不等于所有配置、所有目标下的全面数值证明。
- Clarke 测试夹具原依赖旧板量程，119 code/基值的截断误差超过其 0.001 pu 断言；现明确使用模拟量程 1000 code/基值。生产 G431 参数仍为 119U。
- G431 `debug-rel` 与 `release` 经 `.\make.bat` 构建，生成 ELF/bin/hex，无编译警告或错误。

未烧录、未做真机闭环或 ISR 时序测量，因此不宣称执行时间已经改善。PI、PLL、启动斜坡和 PWM 调制实现不在本次优化范围内。

后续按模块单一职责复核：初始化与每拍递推是同一个 SMO 的生命周期，因此系数函数合回 `foc_smo.c`，不单独保留初始化源文件。

## 后续边界审核

`qElectricalSpeedBaseHz` 的名字在 App/Motor/Position/SMO 已统一；物理 Hz 的 FLOAT/FIXED 编码、范围、舍入和转换位置见各配置字段与标幺参考第 8.5 节。去掉重复的浮点弧度换算宏，`qSignalStrengthPu` 明确是 `(|e_alpha|+|e_beta|)/Vbase`，不是欧氏幅值；该修改不改变接管数值判据。

本记录列出的是该阶段已运行的验证，未覆盖后来参数/日志/Core 边界变化。审核曾复现完整 23 项测试中的 3 项过期断言，现已修复；完整回归与剩余 App/Identify 边界见 [Motor 审核收尾](2026-10-09-motor-boundary-refactor.md#审核收尾2026-10-09)。
