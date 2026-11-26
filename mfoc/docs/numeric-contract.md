# MFOC 数值契约

## 标量、基值与单位

一次构建选择 `MFOC_NUMERIC_FLOAT` 或 `MFOC_NUMERIC_FIXED`，库与调用方使用同一选择。FLOAT 的 `mfoc_scalar_t` 是 `float`。FIXED 的 `mfoc_scalar_t` 是带 15 位小数的 `int32_t`，数值等于原始计数除以 32768。1 LSB 等于 2⁻¹⁵，1 PU 编码为 32768，2 PU 编码为 65536。可表示范围是 **[−65536, 65536 − 2⁻¹⁵]**。接口或控制器可以规定更窄的工作范围。与 CMSIS `q15_t` 等 16 位接口对接时，须按接口范围显式缩放和限幅，再转换位宽；直接窄化会丢失 MFOC 的 1 PU 编码。[标量类型](../math/include/mfoc_types.h) · [缩放常量](../math/include/mfoc_numeric.h)

Port 相电流按相电流峰值基准取 `I/Ibase`，母线使用 `Vdc/Vbase`，电速度使用 `ωe/ωbase`，其中 `ωbase = 2π × electrical_speed_base_hz`。Core 的 dq 电压按 `V/Vbase` 表示，调制电压按 `V/Vdc` 表示，三相 duty 的范围是 [0, 1]。同一标量类型可承载这些不同单位，换算时以接口字段和该 Motor 实例的基值为准。[Port 契约](port-contract.md) · [Core 输入输出](../core/include/mfoc_core.h)

<a id="feedback-model-range"></a>

## Feedback 模型更新的边界

停机调用 `mfoc_feedback_SetMotorParamModel` 时，调用方提供有限的 PU 参数。默认 Rs 范围为 [0.05, 8]，Ld 和 Lq 各为 [0.0001, 16]，磁链为 (0, 16]，惯量为 [0, 1024]，极对数为 1 至 65535。FIXED 按转换后的 Q15 端点比较，Rs、L 的下端点原始计数分别为 1638、3。启用的观测器还要能从这组模型与自己的配置生成系数，SMO 和 Flux 都要求 Ld 等于 Lq。[模型单位](../motor/include/mfoc_motor_params.h) · [默认模型范围](../motor/include/mfoc_motor_config_internal.h) · [模型更新](../motor/src/mfoc_feedback.c)

以 Flux 的磁链平方系数为例，输入 127/32768 PU 时，FIXED 的平方舍入为零，模型更新返回 `OUT_OF_RANGE`。128/32768 PU 的平方恰为半个 Q15 计数，舍入为 1，测试配置下更新成功。FLOAT 在这两点都能精确保存平方。主机回归对 Rs、Ld、Lq、惯量逐项检查紧邻端点的越界值，另测磁链零值、上界及其外侧、极对数边界，每次拒绝都核对整个 Feedback 实例逐字节保持原值。[两后端真实接口用例](../tests/position/flux_test.c) · [三种 Feedback 配置](../tests/scripts/run_flux_tests.ps1)

## 标量转换与算术

| 接口 | FLOAT | FIXED |
| --- | --- | --- |
| `mfoc_from_float(x)` | 原样返回，包括有符号零、NaN、±∞ | 舍入到最近 Q15 计数，恰好半个计数时远离零。越界及 ±∞ 饱和到相应端点，NaN 返回 0。 |
| `mfoc_to_float(x)` | 原样返回 | 将计数转为 float 再乘 2⁻¹⁵。binary32 在大幅值处会丢低位，例如 `INT32_MAX` 读回 65536.0f。 |
| `mfoc_add_sat`、`mfoc_sub_sat` | 普通浮点加减 | 结果饱和到 int32 两端。`mfoc_abs(INT32_MIN)` 返回 `INT32_MAX`。 |
| `mfoc_mul_wide(a,b)` | 直接相乘 | 适用于整个标量范围。以 int64 相乘，除以 32768 向零截断，再饱和到 int32。 |
| `mfoc_mul_pu(a,b)` | 直接相乘，输入超出 ±1 时仍参与计算 | 两个输入各自限于 [−1, 1]，积舍入到最近 Q15 计数，恰好半个计数时远离零。 |
| `mfoc_div_checked(a,b,&out)` | 非有限输入、零分母或非有限商返回错误 | 用宽整数计算 `a_raw × 32768 / b_raw`，向零截断。零分母或商超出载体范围返回错误。 |

对可能超出 ±1 的测量和增益，使用 `mfoc_mul_wide`。例如 2 PU × 0.5 在两后端都得到 1 PU；改用 `mfoc_mul_pu` 后，FLOAT 得 1，FIXED 得 0.5。即使输入均在 ±1 内，两种 FIXED 乘法也有半计数差异。−1 个原始计数乘 0.5，`mul_wide` 得 0，`mul_pu` 得 −1 个计数。需要实际控制限幅时，给 `mfoc_sat` 传入有序上下界。FLOAT 调用方保证数值有限；NaN 会穿过该限幅。受检除法的输出指针为空时返回 `MFOC_RESULT_NULL`，其余失败均保留原输出。[算术定义](../math/include/mfoc_numeric.h) · [边界转换与除法实现](../math/src/mfoc_numeric.c)

`MFOC_SCALAR(x)` 用于可表示、无副作用的常量表达式，例如 `MFOC_SCALAR(0.5f)`。FIXED 宏对缩放后的值加或减 0.5f，再强转为整数，调用方须保证转换结果在 int32 范围内。浮点中间舍入可能改变紧邻半计数或大幅值处的结果。例如 binary32 常量 `0x1.000002p8f` 对应 8388609 个 Q15 计数，宏得到 8388610，`mfoc_from_float` 得到 8388609。运行时参数和需要逐计数准确的浮点边界转换使用 `mfoc_from_float`。[常量宏](../math/include/mfoc_numeric.h) · [转换边界用例](../tests/math/math_test.c)

## 角度

两个后端的 `mfoc_angle_t.wBam32` 都是 `uint32_t`，2³² 个 BAM32 计数为一周。四分之一周是 `0x40000000`，半周是 `0x80000000`，相加按一周回绕。`mfoc_angle_from_scalar` 的输入单位为周，FIXED 的一个 Q15 计数对应 2¹⁷ 个 BAM32 计数。浮点边界接口 `mfoc_angle_from_turns` 将有限输入按周回绕，NaN、±∞ 返回零角；`mfoc_angle_to_turns` 返回 [0, 1) 内的显示值。[角度接口与实现](../math/src/mfoc_angle.c)

`mfoc_angle_diff(target, actual)` 返回从实际角到目标角的最短有向差，恰好半周时取 −0.5 周。FIXED 的角差以 Q15 周为单位向负无穷方向量化，例如 −1 个 BAM32 计数返回 −1 个 Q15 计数。FLOAT 的返回值在正半周边界附近可能舍入为 +0.5 周。[角差定义](../math/include/mfoc_angle.h) · [回绕与半周用例](../tests/math/math_test.c)

## PI 与控制热路径

PI 的离散积分增益为 `ki_ts = Ki × Ts`。FIXED 的输入、输出和积分读数 `ui` 都是 Q15，精确积分的 Q30 原始计数为 `ui × 32768 + remainder`。`ui` 向零截断，有符号余数位于 [−32767, 32767]。每拍只增加一个 Q30 计数时，32768 拍累积一个 Q15 计数。积分钳到上下限时余数清零；`ApplyLimit` 施加外部输出修正时保留界内余量，`Track` 按下一拍的比例项和积分增量预装积分，`Reset` 清除余量并将零夹到积分范围内。[PI 定义](../control/include/mfoc_pi.h) · [小信号与限幅回归](../tests/control/pi_precision_test.c)

<a id="fixed-sampling-check"></a>

## FIXED 采样路径与三角后端验收

Cortex-M0 以 `-O2`、软浮点 ABI、LUT 构建 FIXED，同时启用 SMO、HFI、Flux 与辨识。从 `mfoc_motor_Step` 保留可达的库函数，另外保留由 Port 调用的位置采样入口和待接入 Motor 的电感、磁链辨识采样入口。链接后仅余 4 个板级 Port 符号、3 个整数乘除和 2 个内存清零辅助符号。临时对照将使用浮点换算的前台启动 Init 加入入口，检查即报出 `__aeabi_fdiv` 等引用。此项检查覆盖库内依赖，整机中断周期与栈峰值在 P10 上板测量。[ARM 检查入口](../tests/scripts/check_arm.ps1)

G431 选择 LUT，直接消费 BAM32 并输出后端标量。主机对 100,000 个相位的 sin/cos 与双精度参考比较，最大绝对误差为 FLOAT 1.224 × 10⁻⁶ 以内、FIXED 3.031 × 10⁻⁵ 以内。外部接口的主机桩覆盖调用约定。若目标接入 CMSIS-DSP 等实际后端，再核对其位宽、缩放与舍入误差。[G431 构建选项](../../target/stm32g431/target.mk) · [三角后端选择](../math/include/mfoc_trig.h) · [误差用例](../tests/math/math_test.c) · [主机回归入口](../tests/scripts/run_tests.ps1)
