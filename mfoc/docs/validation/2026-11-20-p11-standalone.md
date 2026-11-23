# P11 独立构建与系统回归

实现基线为 `648f4b3`。独立构建使用与 G431 工程相同的 `mfoc.mk` 源码清单。主机回放提供自己的 Port，控制和辨识模块沿用生产实现。

## 复跑入口

在仓库根目录执行。主机回归需要 GCC 和 GNU Make，ARM 检查需要 LLVM 裸机工具链。

```powershell
./mfoc/tests/scripts/run_tests.ps1
./mfoc/tests/scripts/check_standalone.ps1
./mfoc/tests/scripts/run_per_unit_tests.ps1
./mfoc/tests/scripts/check_arm.ps1
```

## 独立消费与主机回归

导出目录仅包含库源码、公共头、构建清单和主机示例。GCC 15.2.0 与 GNU Make 4.4.1 从这个目录构建并执行，使用示例 Port、标准 C 数学库和编译器运行库。矩阵包括以下组合。

| 构建 | 组合 |
| --- | --- |
| FLOAT LUT | NONE、POSITION、SMO、HFI、Flux、EKF |
| FIXED LUT | NONE、POSITION、SMO、HFI、Flux |
| 旁路观测与模型更新 | 两种数值后端以 POSITION 为主，编入该后端的全部观测器、辨识和模型更新 |
| 三角函数后端 | FLOAT + LIBM，FLOAT/FIXED + 外部绑定 |
| 配置拒绝 | FIXED + EKF，FIXED + LIBM |

LUT 基础矩阵的 40 组构建覆盖 HFI 和辨识开关。HFI 为主反馈时要求编入 HFI。加上旁路观测和三角函数后端，Windows 共验证 45 个合法配置和 2 个预期拒绝配置。示例执行初始化、前台和采样调用、PWM 提交与停机。打开辨识的配置从 PWM 回放完成 Rs 求值。POSITION 配置还交错运行两个 Motor，与单独回放的 duty 逐值对照。

在同一台 Windows 电脑的 WSL Ubuntu 中，将导出目录复制到 Linux 文件系统，再用 Linux 的 GCC 和 Make 构建运行 FLOAT POSITION 和 FIXED POSITION + Rs，核对独立示例的 Linux 构建流程。

主机总入口覆盖 Encoder、Hall、SMO、Flux、EKF、HFI、SMO/Flux 启动交接、波形、PU 等价性和独立消费。[总入口](../../tests/scripts/run_tests.ps1) · [主机回放](../../examples/host_replay/README.md)

## PU 基值与采样率

回放同一段 72 电 Hz 转子轨迹。20 kHz 下，将 3.5 A / 12 V / 400 Hz 和 4 A / 14 V / 320 Hz 的实例逐拍交错执行，比较相电流校验、Core 写出的 duty 以及观测结果的物理值。相同采样率下，三相 duty 的差均小于 0.002。稳定后两套基值的观测差如下，角度为电角度。

| 算法 | FLOAT 最大角差 / 电频差 | FIXED 最大角差 / 电频差 |
| --- | ---: | ---: |
| SMO | 0.00004° / 0.00012 Hz | 0.09888° / 0.27832 Hz |
| Flux | 0.00006° / 0.00009 Hz | 0.09668° / 0.15137 Hz |
| EKF | 0.00002° / 0.00002 Hz | 仅 FLOAT |

16 kHz 实例与两组 20 kHz 实例交错执行，用独立轨迹真值验收。三个实例的最大 Core 电压误差为 FLOAT 0.124 mV、FIXED 57.46 mV。观测相对真值的最大误差为 SMO 1.34° / 0.564 Hz、Flux 0.595° / 0.227 Hz、FLOAT EKF 1.473° / 0.286 Hz。

参数辨识复用 P09 的 SI 功率级模型，以两套基值分别构建。保持同一电机、12 V 母线、0.42 A 上限和 200 电气 rad/s 上限。四种公开辨识操作均运行，串联结果折回 SI 后，两套基值的最大差为 Rs 0.006%、Ld 0.082%、Lq 0.075%、磁链 0.043%、惯量 0.109%。这些差值用于软件等价性回归，板上精度由台架数据确定。另验证了启用 perfc 并关闭位置源时的两种数值后端。[PU 回放](../../tests/scripts/run_per_unit_tests.ps1) · [辨识模型](../../tests/motor/motor_identification_plant_test.c)

## ARM 裸机链接与资源

用 Embedded LLVM 19.1.5、`-O2`、C11 和 LUT 构建，链接器保留全部已编译段。测试 ELF 由库、Port 模板和测试端的板级钩子组成，显式链接工具链的 libc、libm 和 compiler-rt。表内栈值来自编译器的单函数栈报告，单位均为字节。

| 目标与构建 | text | data | bss | Motor 实例 | Init / Run / Step 局部栈 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Cortex-M0 FIXED，POSITION 主反馈、Flux 旁路 | 29728 | 0 | 528 | 416 | 528 / 40 / 168 |
| Cortex-M0 FIXED，全部 FIXED 观测器、辨识和模型更新 | 30972 | 0 | 968 | 856 | 968 / 40 / 176 |
| Cortex-M4F FLOAT，全部观测器、辨识和模型更新 | 26394 | 0 | 1160 | 1048 | 1208 / 24 / 152 |
| Cortex-M4F FIXED，全部 FIXED 观测器、辨识和模型更新 | 27708 | 0 | 968 | 856 | 952 / 24 / 152 |

四个 ELF 都完成链接，未定义符号、堆分配符号和库对象内的可写全局均为零。Cortex-M0 FIXED 采样链的符号检查包含 SMO、Flux、HFI、Encoder、Hall 和辨识，外部调用限于 Port 与整数、内存运行库。另检查了 G431 的 MDI Port 和 Motor/ADC IRQ 绑定。[ARM 检查](../../tests/scripts/check_arm.ps1)

用 STM32G431 工程入口重新构建默认 Flux `BUILD=release` 固件。FLOAT 的 `.bin` 为 25,836 B，FIXED 为 27,760 B，两份均与修改前逐字节相同。实际栈峰值与 PWM 截止余量按 P10 的板上方案验收。

## V1 初始化栈复测（2026-11-23）

以 `cf50772` 为对照，使用 Embedded LLVM 19.1.5、`-O2`、LUT 和
相同的 POS 主反馈回放。全开构建编入该数值格式支持的全部观测器、
辨识和模型更新，初始化同时配置速度 PI 和 OpenLoop。Motor 和
Feedback 在 Port 就绪后才写入新实例，算法系数只计算一次。主机
FLOAT、FIXED 用例覆盖后序观测器拒绝、OpenLoop 拒绝、Port Init
最后失败及 Port 时钟不可用，失败时保留原 Motor 的每个字节。

| 构建 | Init 局部栈，改前 → 改后 | Init 含子调用峰值，改前 → 改后 |
| --- | ---: | ---: |
| Cortex-M0 FIXED 全开 | 968 → 400 B | 1544 → 816 B |
| Cortex-M4F FLOAT 全开 | 1208 → 584 B | 1784 → 904 B |
| Cortex-M4F FIXED 全开 | 952 → 384 B | 1448 → 768 B |

局部栈取编译器 `.su`，含子调用的峰值取链接 ELF 在 Unicorn 中
从 `mfoc_motor_Init` 入口至返回的最大 SP 落差，使用可完成校准和
采样的 Port 模板钩子。G431 Flux 的 `-Os` 对照中，Init 局部栈
为 FLOAT 520 → 272 B，FIXED 528 → 280 B。三个全开构建和两个
G431 Flux 构建的 `mfoc_motor_Step`、`mfoc_feedback_Step` 指令
和重定位逐项一致。

辨识的新 Motor 入口在前台保存终态和已有结果。开桥失败时释放辨识
占用并恢复结果，Motor 仍保留 PWM 故障。用同一静止 Port 回放两次
调用启动和新入口，计数包括子调用。新入口数字依次为无旧结果、有
旧结果时的指令数。

| 构建 | 原启动指令 | 新启动指令 | 含子调用峰值，改前 → 改后 |
| --- | ---: | ---: | ---: |
| Cortex-M0 FIXED 全开 | 415 | 452 / 468 | 120 → 192 B |
| Cortex-M4F FLOAT 全开 | 216 | 246 / 250 | 104 → 152 B |
| Cortex-M4F FIXED 全开 | 207 | 237 / 241 | 104 → 152 B |

这笔开销发生在空闲 Motor 的前台启动。回放不含目标板开桥握手的
实际等待。主机用例另外核对了开桥失败后的恢复，以及算完但自动关桥
失败时仍能读取结果。

在仓库根目录执行。将 `$arm` 设为检查脚本最后打印的 ARM 产物目录。
用 `FIXED-all-observers.elf --core m0` 或
`M4F-FIXED.elf --core m4` 替换 ELF 及核心参数即可测其余全开组合。

```powershell
./mfoc/tests/scripts/check_arm.ps1 -MeasureIdentificationStart
uv run --with unicorn --with pyelftools python mfoc/tests/arm/motor_stack_replay.py `
    "$arm/FLOAT.elf" --core m4 --function mfoc_motor_Init `
    --function start_identification_legacy --function start_identification_motor
```

复查旧基线时，把同一份 `mfoc/tests/arm/link_smoke.c` 放进
`cf50772` 的源码副本，使用基线的 ARM 检查脚本构建 ELF，并运行
上面的回放，只传 `--function mfoc_motor_Init`。基线脚本未登记
HFI 所用的两个整数辅助符号，四种 ELF 在该符号检查之前已生成。

## 验收边界

- HFI 和 Motor 辨识仍使用编译期 profile。同一个程序中的启动、SMO、Flux、EKF 可按实例配置。旋转辨识的跨基值回放使用分别匹配各套基值的构建。
- FLOAT 支持 EKF，FIXED + EKF 由配置头拒绝。每个构建内，库、Port 和应用使用一致的功能开关。
- P10 用生产配置核对真实采样至最后一次写 PWM 的截止余量。P12 验证第二芯片和板上功能，P13 确定发布包许可证并从发布包复跑本页检查。
