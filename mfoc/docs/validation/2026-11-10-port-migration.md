# 2026-11-10 Port 迁移验证

## 范围与环境

工作目录：`E:\Project\modus_template`。本次验证覆盖 Port 边界迁移、真实 G431 MDI 寄存器适配的 host 测试、FLOAT/FIXED 编译及 G431 release 裁剪。没有执行 flash、reset、halt、GDB 或实机电机测试。

G431 构建通过仓库 `.\make.bat`，使用 `D:/software/llvm_for_arm/bin` 工具链。Host Port 测试使用 GCC，`-O2 -Wall -Wextra -Werror -ffp-contract=off`。各 G431 构建使用独立目录，没有删除原 `build` 中的产物。

## 改动前 G431 基线

| 检查 | 结果 | 证据 |
|---|---|---|
| 默认目录 G431 构建 | 失败，既有依赖缓存引用已不存在的 `mfoc/motor/src/mfoc_motor_irq.c` | 原日志 `mfoc-g431-baseline-89bea4bf-ac49-4ddb-a40e-60c7a7bb57ee.log` |
| 独立目录 FLOAT 构建 | 通过 | text=51320，data=480，bss=17424 |
| 独立目录 FIXED 构建 | 失败，target 同时硬编码 FLOAT | `Select exactly one mfoc numeric backend` |

执行命令：

```powershell
.\make.bat TARGET_CHIP=stm32g431 -j4
New-Item -ItemType Directory -Path build\mfoc-port-baseline -Force
.\make.bat TARGET_CHIP=stm32g431 BUILD_DIR=build/mfoc-port-baseline -j4
New-Item -ItemType Directory -Path build\mfoc-port-baseline-fixed -Force
.\make.bat TARGET_CHIP=stm32g431 MFOC_NUMERIC=fixed `
    BUILD_DIR=build/mfoc-port-baseline-fixed -j4
```

首个独立目录尝试还暴露 Windows `mkdir` 处理带 `/` 路径的既有问题；使用 PowerShell 先建立目录后完整构建通过。`make.bat` 在某些 Make 失败后仍返回 0，因此本次同时检查日志中的 `error:` / `***` 和目标 ELF/size，未仅依据外层退出码判断成功。

基线日志位于执行机器 `C:\Users\xun\AppData\Local\Temp\`：

- FLOAT：`mfoc-g431-baseline-clean-6a9031c2-ab4e-47a2-8bcc-167af5a888fc.log`。
- FIXED：`mfoc-g431-baseline-fixed-83f67af2-89fd-4be6-bf9e-b410928d839d.log`。

FIXED 的双宏冲突由 `target/stm32g431/target.mk` 去除硬编码 FLOAT 解决，最终后端统一由顶层 `MFOC_NUMERIC` 选择。

## Port 测试：先失败、再通过

先将测试迁移到显式 cfg、64 位 tick、分离 PWM API 和板级机械角入口，并补充位置边界用例，再修改板级实现。旧实现编译失败，明确暴露 `mfoc_port_tick`、`mfoc_port_init`、`mfoc_port_pwm` 签名冲突及缺少板级入口/基值字段。

RED 日志：`C:\Users\xun\AppData\Local\Temp\mfoc-port-boundary-red-fd2620f7-daf1-44c2-9e90-7df9ca26cb0b.log`。

最终命令：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
    -File mfoc/tests/scripts/run_port_tests.ps1
```

最终输出：

```text
G431 bases, current, position, clock and PWM passed
FLOAT G431 port passed
G431 bases, current, position, clock and PWM passed
FIXED G431 port passed
```

测试链接真实 G431 Port、真实 Encoder 和数学源文件；ADC、PWM、I2C、COMP 使用 host 寄存器存储和假时钟。测试无需 Motor fake，能独立验证板级拥有位置实例后的边界。

| 覆盖项 | 检查内容 |
|---|---|
| ADC 原有换算 | 缺失帧、相序、正反极性、超 1 PU 不截断、全部 14 位 ADC 范围、多组增益 |
| 初始化 | 相门开启拒绝校准；SF=3 写入/回读；10 帧零偏；采样失败保留原配置 |
| 显式基值 | 电流基值 3500→7000 mA、母线基值 12000→24000 mV 后相同物理输入的 PU 减半；拒绝零基值/零极对数/零速度基值；失败保留既有换算 |
| 时间 | `4294967299` tick 完整返回；位置样本跨低 32 位回绕仍连续；超过一个 32 位周期的陈旧样本不能伪装成新鲜样本 |
| 前台到 ISR | 首次无样本状态；1 ms 节拍；0.25 滤波；电角度/PU 速度由同一实例输出；ISR 在 I2C NACK 时仍可读已有样本并细分 |
| 失败与校准 | 前台 I2C 失败保留原样本；过期不改输出；校准后电角度等于保持角、速度清零；校准 I/O 失败保留原零位 |
| PWM | duty 换算、错误 ARR、写入不使能、初始 duty 先于使能、故障拒绝启动、停止保留 ADC 触发、停止后单纯更新不重开相门 |

GREEN 日志：`C:\Users\xun\AppData\Local\Temp\mfoc-port-green-ae1f637c-c4dc-4be0-8134-0cabed8f2e98.log`。

最终测试产物：`C:\Users\xun\AppData\Local\Temp\mfoc-port-f8985479-4f4e-44c5-bdac-7bc1c0ce7d5f`。

## 改动后 G431 构建

先在三个新目录完整构建，再在公共接口、HFI 基值约束和校准注释修正后增量验证。以下均成功，复查日志无 warning/error：

| 配置 | BUILD_DIR | text | data | bss |
|---|---|---:|---:|---:|
| FLOAT，默认 debug-rel | `build/mfoc-port-final-float` | 51896 | 468 | 17456 |
| FIXED，默认 debug-rel | `build/mfoc-port-final-fixed` | 51120 | 468 | 17440 |
| FLOAT，release | `build/mfoc-port-final-release` | 27216 | 296 | 5120 |

执行命令：

```powershell
New-Item -ItemType Directory -Path build\mfoc-port-final-float -Force
.\make.bat TARGET_CHIP=stm32g431 MFOC_NUMERIC=float `
    BUILD_DIR=build/mfoc-port-final-float -j4
New-Item -ItemType Directory -Path build\mfoc-port-final-fixed -Force
.\make.bat TARGET_CHIP=stm32g431 MFOC_NUMERIC=fixed `
    BUILD_DIR=build/mfoc-port-final-fixed -j4
New-Item -ItemType Directory -Path build\mfoc-port-final-release -Force
.\make.bat TARGET_CHIP=stm32g431 MFOC_NUMERIC=float BUILD=release `
    BUILD_DIR=build/mfoc-port-final-release -j4
```

各目录生成 `template.elf`、`.bin`、`.hex` 和 `.map`。release 的编译定义为 `MFOC_ISR_MEASURE=0`，ELF 符号检查未发现 `mfoc_debug_CaptureWaveform` 或 ISR 计时调试函数；控制入口 `mfoc_motor_IRQHandler` 保留。

完整构建日志位于前述 Temp 目录：

- FLOAT：`mfoc-g431-final-float-cecc4aab-2ad9-4b7e-9268-8b9eb71c02f5.log`。
- FIXED：`mfoc-g431-final-fixed-267e1269-03e4-430d-a6c5-187eebf870b4.log`。
- release：`mfoc-g431-final-release-479edec4-a018-410c-8f3b-5c13526ba289.log`。

公共调用点修正后的增量复查日志：

- FLOAT：`mfoc-g431-final-recheck-float-6b59ba01-0df6-4d5f-b84d-6b88c86cebc2.log`。
- FIXED：`mfoc-g431-final-recheck-fixed-98b785c8-5851-4d35-a004-f331efc17d03.log`。
- release：`mfoc-g431-final-recheck-float-release-5020bccc-1485-45c4-81e4-aa128234da1b.log`。

最终生产源码停止修改后的复查日志，size 与上表一致：

- FLOAT：`mfoc-g431-approved-float-4057071d-a4f4-49d2-b0ac-b7e1a1c0fa57.log`。
- FIXED：`mfoc-g431-approved-fixed-0d23fb11-a5d9-4359-ae88-644386c5d1a7.log`。
- release：`mfoc-g431-approved-float-release-aa290b99-30f0-4f6a-af8d-1f264d2275ba.log`。

额外在 `MFOC_FEEDBACK_ENABLE_POSITION=0` 下编译 G431 Port 的 FLOAT/FIXED 对象，均通过。`llvm-nm` 确认对象不再引用 `mfoc_encoder_*`，也不定义三个 `mfoc_port_position*` 符号，证实物理位置服务可以完整裁剪。

复现命令如下；环境变量覆盖仅作用于本次命令，结束后恢复原值：

```powershell
$previousDefs = $env:C_DEFS
try {
    $env:C_DEFS = '-DMFOC_FEEDBACK_ENABLE_POSITION=0'
    foreach ($numeric in @('float', 'fixed')) {
        $buildDir = "build/mfoc-port-no-position-$numeric"
        New-Item -ItemType Directory -Path $buildDir -Force
        .\make.bat TARGET_CHIP=stm32g431 "MFOC_NUMERIC=$numeric" `
            "BUILD_DIR=$buildDir" "$buildDir/mfoc_port.o"
        & D:/software/llvm_for_arm/bin/llvm-nm.exe "$buildDir/mfoc_port.o"
    }
} finally {
    $env:C_DEFS = $previousDefs
}
```

无位置对象编译日志：`mfoc-g431-no-position-float-8407c657-62d5-4b7c-aff9-95c640825ac7.log`、`mfoc-g431-no-position-fixed-cc1e3735-a555-4a05-b4fb-633f3be91731.log`。

日志和二进制为执行机器本地产物；上面的命令及结果摘要保存在仓库，便于重新验证。

## 库级 FLOAT/FIXED 回归

基线在任何源码/测试修改前收集，仓库 HEAD 为 `147a3ffc9eea96b25ac2f4def6c1c81dfa9f3720`。math/PI、Core 常规控制、Encoder/SMO/Flux/EKF、Port、App、debug 和 HFI 叶子已通过 FLOAT/FIXED 适用矩阵。

基线存在两组测试契约失配，聚合脚本不能跑完：

1. `motor_identification` 要求尚未实现的 Flux 阶段读取 Feedback 并推进算法；实际阶段为 TODO，返回 ERROR。另有 HFI 关闭时无条件访问 `acquiring` 的编译错误。迁移测试保留 Rs 闭环/超时/取消覆盖，补充暂停的 virtual 路由使用辨识自有参考、不读取 Feedback，以及未实现阶段报错并关断；按 HFI 开关裁剪断言。armed Rs 在 IDLE 的旧返回码断言也按既有辨识路由修正，并单独验证位置服务已执行。
2. HFI shadow + SMO 测试在所选 SMO 返回失败时要求公共注入输出更新，与 Feedback 失败不写输出的原契约冲突。改为验证内部 HFI 仍推进、成功时发布激励、失败时公共输出保持原值。

以上未修改生产算法以迎合测试。详细基线和独立运行日志位于 `C:\Users\xun\AppData\Local\Temp\mfoc-port-baseline-da20983b-0c78-45d8-9224-233ac04ff27f`，其中 `README.md` 记录基线，`MIGRATION.md` 记录迁移矩阵和命令。

最终聚合命令：

```powershell
.\mfoc\tests\scripts\run_tests.ps1
```

完整通过，退出码 0。日志：`C:\Users\xun\AppData\Local\Temp\mfoc-port-root-final-tests.log`。覆盖 math 五种数值/三角后端组合、PI、Core/Motor/IRQ/辨识/HFI Motor、Encoder、SMO、HFI、G431 Port 和 App；适用的 FLOAT/FIXED 均通过。

聚合之外的补充检查也通过：

| 命令（`mfoc/tests/scripts/` 下） | 验证范围 |
|---|---|
| `run_core_tests.ps1 -Hfi 0` | 关闭 HFI 的 FLOAT/FIXED 完整控制套件；与 HFI 开启合计 40 项 |
| `run_core_tests.ps1 -Tests motor -Hfi 1` 和 `-Hfi 0` | HFI 基值一致性约束加入后的四组合复查 |
| `run_flux_tests.ps1` | FLOAT/FIXED 叶子及三种接线配置 |
| `run_ekf_tests.ps1` | FLOAT 叶子及四种接线；FIXED 明确拒绝启用；禁用时无 EKF 符号 |
| `run_debug_tests.ps1` | 14 种诊断配置 |
| `run_app_tests.ps1 -Hfi 0`、`-Flux 1`、`-Ekf 1` | 额外 15 种应用配置 |
| `run_core_tests.ps1 -Tests motor_handover -Hfi 0 -Backend 2` | SMO 接管，FLOAT/FIXED |
| 同上，`-Backend 4` / `-Backend 5` | Flux 接管 FLOAT/FIXED；EKF 接管 FLOAT |
| `check_arm.ps1` | Cortex-M0 FIXED / Cortex-M4 FLOAT；G431 Port/IRQ 双后端；FIXED Port 无意外浮点辅助调用 |

本次新增边界覆盖：Motor_Init 基值传递；辨识期间前台位置仍刷新；PWM 更新不使能；对齐和辨识跨低 32 位边界及完整 `2^32` tick 停顿；位置失败不跳过到期停机；Feedback 不持有位置实例；HFI 基值冲突在硬件初始化前拒绝。位置服务和 HFI 基值用例均先在旧行为上确认 RED，再验证迁移实现通过。

Encoder API 独立矩阵为 FLOAT/FIXED × POSITION 选中/仅启用/NONE 全禁用，六项通过。旧 32 位时间实现的完整 `2^32` 停顿漏检已先复现；新实现拒绝过期样本且保持输出。保留短间隔计算公式的逐拍等价回放，在每个后端比较 20,000 组角度/速度，逐位一致，覆盖低 32 位跨界、正反方向、滤波、漏采样及周期零位标定。

等价回放证据：`C:\Users\xun\AppData\Local\Temp\mfoc-encoder-equivalence-k7hr37_i`；长停顿 RED：`C:\Users\xun\AppData\Local\Temp\mfoc-encoder-red-3be66cc4-3452-4bd2-bcde-e157b575e4e0\red.log`。

源码差异核对：Core、math 数学实现、control、OpenLoop、SensorlessStart、SMO、Flux、HFI、EKF 和辨识叶子递推文件没有修改；辨识调度只替换超时时基，保留 PT 流程。原有 `mfoc_motor_config.h` 用户改动保持不变，SHA256 为 `9F0D897AD2358BDA4FA8E62CA753CFF49B1702559AB0045BA5CB59C08103202F`。

## 证据边界

构建和寄存器模拟验证了接口链接、换算、时基、位置发布和输出门控的软件契约。它们不验证真实电流波形、AS5600 总线电气行为、PWM/ADC 硬件窗口、ISR 最坏耗时或电机参数辨识的实机效果。本次没有改变已经验证的算法计算来追求性能，也没有将未进行的实机测试标为通过。

## 2026-11-11 移植模板与 G431 诊断复查

| 检查 | 结果 |
| --- | --- |
| 移植模板，FLOAT/FIXED × POSITION 开/关 | 四种组合通过 GCC 编译、符号检查和模拟板链接运行。启用时导出 11 个 Port 函数，关闭时导出 8 个。覆盖母线换算、PWM 装载/使能顺序和失败返回，有感组合额外验证采样、外推、超时和校零。 |
| 公开头、Encoder、Feedback | 两种数值后端均验证关闭 POSITION 时头文件隐藏三个位置操作。Encoder 六种配置及 SMO/物理位置的反馈选择回归通过。 |
| G431 Port 与应用 | 真实板级代码的寄存器模拟、应用六种配置通过。Shell 原始读数 2048 对应 180.00°，4095 对应 359.91°。 |
| G431 整机 | FLOAT 和 FIXED 完成编译及链接。FLOAT 关闭 POSITION 时仍可链接原始角诊断，符号表包含板级原始读数入口，已裁掉三个位置操作。 |

复跑入口为 `tests/scripts/run_encoder_tests.ps1`、`run_smo_tests.ps1`、`run_port_tests.ps1` 和 `run_app_tests.ps1`。G431 使用上述构建命令，独立目录为 `build/mfoc-port-raw-1111-float` 与 `build/mfoc-port-raw-1111-fixed`。本次覆盖主机运行和目标链接。
