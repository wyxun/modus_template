# P12 · 运行期母线帧龄与控制拍碰撞回放

2026-11-22，主仓库基线 `bc6affe` 加本地 P12 实现。两块板均以 Clang 19.1.5、Cortex-M4F、release、20 kHz 和 2804 默认电机配置构建。每板各有 FLOAT、FIXED，每组分别编译 `MFOC_RUNTIME_BUS=0/1`。HFI 为主反馈，SMO 为旁路，Flux 关闭，G431 保留 POSITION，AT32 关闭 Hall。下表为 Unicorn 执行已链接 ARM Thumb 代码的动态指令数与软件栈深度。硬件计时和控制窗口余量待台架测量。

## 帧龄策略和最重拍的选择

每个运行中且电流帧已就绪的 IRQ 把控制帧号加一。前台先读取该帧号，再读绝对时钟，从采样起点算出剩余寿命，随系数发布一个**排他**截止帧号。延迟发布及前台被 IRQ 抢占都消耗本帧寿命。20 kHz 时 30 ms 对应 600 拍，发布相位量化最多提前一拍保护。该时限以运行中的固定采样节拍为前提，若整个控制中断长期停摆，须由系统的时基或独立保护检出。开桥前按绝对时间核验，开桥后的第一次有效 IRQ 再核验一次，停机时间和开桥至第一帧的延迟均计入帧龄。普通拍和运行中收到新母线帧时只用帧号判过期。

| 板 | 无新帧，旧版 → 本版 | 运行中新帧，旧版 → 本版 | 运行中新帧，每次新帧在 IRQ 读时钟的备选 | 开桥首拍，本版 / 备选 |
|---|---:|---:|---:|---:|
| G431，FLOAT / FIXED | +81 → **+36** | +87 → **+54** | +98 | +109 / +98 |
| AT32F413，FLOAT / FIXED | +82 → **+37** | +88 → **+55** | +99 | +110 / +99 |

以上均为相对同工况关闭母线更新的增量。两种方案都拦截发布前已老化的样本。备选方案在每次新帧进入 IRQ 后取绝对时间计算剩余帧数，随后用帧号判过期。本版把这笔计算移到前台，两板每个运行中新帧都少 **44 条**，无新帧的指令数及所有健康路径的峰值栈相同。代价是每次开桥后的首次交接多 11 条，两板各多 92 B Flash、20 B 静态 RAM。对最重运行帧，G431 FIXED 三者同拍从备选的 3241 条降到 3197 条，AT32 FIXED 从 2901 降到 2857 条。两种方案均实编并回放，交付固件采用前台计算。

## 普通 IRQ 与碰撞 IRQ

从 `mfoc_motor_IRQHandler` 首条指令计到返回。普通拍跳过速度 PI、SMO 测速窗更新及 HFI 第八拍解调。碰撞拍按行标题强制对应调度同时发生。每个成功样本均提交 PWM，三相 CCR 和重建 αβ 电压与同工况关闭母线时逐位相同。母线取 1 PU，G431 的 POSITION 旁路输入有效并执行正常外推。开桥列刻意把交接与该行所列负载同时回放，以便看到其独立增量。

| 板 / 数值 | 控制工况 | 功能关闭 | 运行中无新帧 | 运行中新帧 | 开桥首拍 |
|---|---|---:|---:|---:|---:|
| G431 FLOAT | 普通拍 | 1290 | 1326 | 1344 | 1399 |
| G431 FLOAT | SMO 测速窗 + 速度 PI | 1439 | 1475 | 1493 | 1548 |
| G431 FLOAT | HFI 第八拍 + 速度 PI | 1603 | 1639 | 1657 | 1712 |
| G431 FLOAT | 三者同拍 | 1700 | 1736 | 1754 | 1809 |
| G431 FIXED | 普通拍 | 2044 | 2080 | 2098 | 2153 |
| G431 FIXED | SMO 测速窗 + 速度 PI | 2441 | 2477 | 2495 | 2550 |
| G431 FIXED | HFI 第八拍 + 速度 PI | 2879 | 2915 | 2933 | 2988 |
| G431 FIXED | 三者同拍 | 3143 | 3179 | 3197 | 3252 |
| AT32F413 FLOAT | 普通拍 | 975 | 1012 | 1030 | 1085 |
| AT32F413 FLOAT | SMO 测速窗 + 速度 PI | 1124 | 1161 | 1179 | 1234 |
| AT32F413 FLOAT | HFI 第八拍 + 速度 PI | 1288 | 1325 | 1343 | 1398 |
| AT32F413 FLOAT | 三者同拍 | 1385 | 1422 | 1440 | 1495 |
| AT32F413 FIXED | 普通拍 | 1703 | 1740 | 1758 | 1813 |
| AT32F413 FIXED | SMO 测速窗 + 速度 PI | 2100 | 2137 | 2155 | 2210 |
| AT32F413 FIXED | HFI 第八拍 + 速度 PI | 2538 | 2575 | 2593 | 2648 |
| AT32F413 FIXED | 三者同拍 | 2802 | 2839 | 2857 | 2912 |

先在启动交接时提供 29 ms 的合法旧样本，逐控制帧推进至首次过期，再从该帧真实 IRQ 重放保护。G431、AT32 分别用 **103、85 条**。另一路先成功 Arm，等待至少 30 ms 并跨过时钟低 32 位回绕，再进入第一帧，分别用 **166、148 条**。两种数值配置一致，均返回 `SAFETY`、调用一次 `Trip`，未调用 `mfoc_port_pwm`。G431 正常 IRQ 的 POSITION 旁路自身仍读时钟一次。对照 OFF ELF，在开启母线后，普通及新帧的时钟读取次数均无增加，启动交接仅多一次。

## 栈、Flash、静态 RAM

软件栈以 IRQ 入口的 SP 为基准，取至返回的最大落差。芯片异常入栈、浮点上下文与更高优先级异常另计。正常路径包括开桥首拍、新帧和无新帧。运行中过期保护栈为 56 B，启动交接超时保护为 80 B，两板一致。

| 板 / 数值 | 普通或 SMO + 速度 PI，关 → 开 | HFI 第八拍 + 速度 PI 或三者同拍，关 → 开 |
|---|---:|---:|
| G431 FLOAT | 352 → 352 B | 352 → 352 B |
| G431 FIXED | 416 → 416 B | 416 → 416 B |
| AT32F413 FLOAT | 248 → 256 B | 272 → 280 B |
| AT32F413 FIXED | 288 → 296 B | 384 → 392 B |

上次 G431 FLOAT 回放的 POSITION 无有效输入，本次有效外推使关闭功能的碰撞基线多 21 条，软件栈由 336 B 变为 352 B。两版成本比较采用各自配对的开关差额。仅计 `mfoc_board_bus_Apply` 本体，普通拍执行 27 条，新帧 45 条，开桥首拍 62 条，运行过期 / 首拍过期为 26 / 51 条。另测直接 `mfoc_core_SetBus(core, 1 PU)` 含子函数，FLOAT 是 70 条 / 32 B，FIXED 是 101 条 / 136 B。本版采样中断只提交预计算的系数，稳态路径无需读时钟或运行倒数除法。

Flash 取 ELF 的 Flash 可装载区段末地址减 `0x08000000`，包含 `.data` 初值。静态 RAM 为 `.data + .bss`。此表对应碰撞配置，ELF 哈希包含调试信息。

| 板 / 数值 | Flash 关 → 开（B） | 增量 | 静态 RAM 关 → 开（B） | 增量 | ELF SHA-256 前 12 位，关 / 开 |
|---|---:|---:|---:|---:|---|
| G431 FLOAT | 28392 → 29500 | +1108 | 2432 → 2548 | +116 | `553cb48ea213` / `4a4f4116576b` |
| G431 FIXED | 29104 → 30160 | +1056 | 2456 → 2572 | +116 | `177e608f6313` / `9b7116b7f939` |
| AT32F413 FLOAT | 20336 → 21288 | +952 | 1564 → 1664 | +100 | `eed7b6d6ba74` / `1689fbd31b3c` |
| AT32F413 FIXED | 21184 → 22076 | +892 | 1588 → 1688 | +100 | `5ca1140f32ca` / `c688adac3743` |

与上次开启母线的版本相比，FLOAT/ FIXED 分别多 360 / 280 B Flash，两板各多 28 B 静态 RAM，保存截止帧号和启动交接状态。

## 复现

回放用实际 Motor Init、板级控制 IRQ、时钟读取、反馈、PI、PWM 和故障路径，以标定中点及板级增益构造零电流，预置 SMO 反电势和已锁定 HFI 的历史。脚本检查速度 PI 的实例、SMO 测速计数回绕、HFI 有效解调与锁定、PWM 提交及端口时钟读取次数。发布系数前后的判断由 `board_bus_test` 另外覆盖。无交互命令的 release 链接会裁掉所有前台启动命令，故另以同一批对象保留 `mfoc_board_bus_Arm`，验证真实 Arm 与同实例再次 Arm 后的交接，断言指令数、栈、数值和故障均与表中一致。尺寸统计采用常规 release ELF。

```powershell
$observer = 'MFOC_MOTOR_CONFIG_FLAGS=-DMFOC_FEEDBACK_ENABLE_HFI=1 -DMFOC_FEEDBACK_ENABLE_SMO=1 -DMFOC_FEEDBACK_ENABLE_FLUX=0 -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_HFI'
foreach ($board in 'stm32g431','at32f413') {
    foreach ($numeric in 'float','fixed') {
        $prefix = "build/p12-overlap-$board-$numeric"
        foreach ($bus in 0,1) {
            $dir = "$prefix-$bus"
            $targets = @("$dir/template.elf")
            if ($bus) { $targets += "$dir/bus-arm.elf" }
            .\make.bat -j4 -f makefile -f mfoc/tests/port/runtime_bus_replay.mk "TARGET_CHIP=$board" BUILD=release "MFOC_NUMERIC=$numeric" "MFOC_RUNTIME_BUS=$bus" "BUILD_DIR=$dir" $observer @targets
            if ($LASTEXITCODE) { throw "Build failed $dir" }
        }
        uv run --python 3.13 --with unicorn --with pyelftools python mfoc/tests/port/runtime_bus_irq_replay.py $board $numeric "$prefix-0/template.elf" "$prefix-1/template.elf" --armed-on "$prefix-1/bus-arm.elf"
        if ($LASTEXITCODE) { throw "Replay failed $board $numeric" }
    }
}
```
