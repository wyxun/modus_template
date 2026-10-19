# mfoc

mfoc 是 `dev-focv2` 的新 FOC 库，目录、文件和符号统一使用 `mfoc` 名称。

首批仅迁移可独立审核的计算基建，不接入当前固件。旧 `foc/` 保留作为参考和对照；本目录不依赖旧库、MODUS、MDI、ADC、PWM 寄存器或 ISR。

## 首批边界

| 文件 | 内容 | 来源 |
| --- | --- | --- |
| `mfoc_numeric.*` | FLOAT / int32 Q15 标量、饱和、乘除、转换 | `foc/math/foc_numeric.*`，不带旧 PID 的分拆增益类型 |
| `mfoc_angle.*` | BAM32 角度、环绕、角差、atan2、sincos | `foc/math/foc_angle.*` |
| `mfoc_trig.*`、`mfoc_sin_table.inc` | 四分之一波表、插值、编译期后端 | `foc/math/foc_trig*` |
| `mfoc_math.*` | 标量平方根 | `foc/math/foc_math.*` |
| `mfoc_transform.*` | Clarke、Park、逆 Park | 从旧 `foc_core.c` 提取纯变换 |
| `mfoc_modulation.*` | SVPWM、SPWM、理想占空比电压重建 | `foc/modulation/foc_modulation.*` |
| `mfoc_types.h` | 标量、统一结果码、uvw、alpha-beta、dq 公共类型 | 不依赖数值运算头，三相量统一为 `mfoc_uvw_t` |

PID、电流环编排、SMO、编码器、硬拖、识别、接管、安全状态机和调试统计留到业务流程审核后迁移；首批不把它们带入基础类型和配置头。

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

## 构建和测试

在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File mfoc/tests/run_tests.ps1
```

测试独立编译新库，覆盖 FLOAT/FIXED、LUT、FLOAT LIBM、直接绑定的外部三角后端、随机及边界数值。迁移对照另行链接旧数学实现，不影响新库依赖边界。ARM 编译及 FIXED 查表无软浮点检查运行 `mfoc/tests/check_arm.ps1`，可通过 `-Toolchain` 指定 LLVM 工具链目录。

`mfoc.mk` 是可选的源文件清单，当前没有加入任何 target。调用方指定 `MFOC_NUMERIC_*` 和包含路径后可使用 `include/mfoc.h`。旧 FOC 与新库使用不同文件名、类型和符号，可在审核阶段并存。

默认三角后端是 LUT。`MFOC_TRIG_BACKEND_LIBM` 仅支持 FLOAT；`MFOC_TRIG_BACKEND_EXTERNAL` 由目标提供两个直接函数 `mfoc_trig_port_sincos` / `mfoc_trig_port_atan2`，不引入运行时 ops 表。首批不复制 G431 CORDIC 驱动，也不宣称已经完成硬件适配。

公式审核、已修正问题和暂缓优化见 [迁移审核](MIGRATION_REVIEW.md)。本轮仅有主机及目标编译证据，没有电机台架性能结论。
