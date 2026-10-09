# Motor 内部整理与模块边界存档

日期：2026-10-09。基线是本次操作开始时的工作区，包含此前的参数调整、SMO 整理及日志清理，不是 Git HEAD。未提交，未烧录或运行电机。

## 变化与理由

| 项目 | 整理前 | 整理后 | 理由 |
|---|---|---|---|
| Motor 顶层成员 | 33 个直接成员，启动/对中/速度/校准状态散落 | 16 个直接成员，按 `tStart/tAlign/tSpeed/tCalib` 分组 | 状态和所属流程一起阅读，不新增独立源文件或对象框架 |
| 电流基值 | 根成员与 `tParams` 各存一份 | Motor 只保留 `tParams.wCurrentBaseMilliamp`，Init 用 `FOC_CURRENT_BASE_MILLIAMP` 绑定；电感辨识保留同值只读快照 | 采样和辨识使用同一物理基值；应用不能另设采样基准 |
| 采样路径 | ALIGN/RUN 重复采样、Clarke 和故障分支 | `_motor_SampleCurrent` 统一 ADC 零偏、pu 归一化和 Clarke | 同一入口处理采样与数学失败，保留原 ADC ±1 pu 限幅 |
| 控制提交 | ALIGN/RUN 重复 Core/PWM 提交及错误处理 | `_motor_ApplyControl` 统一提交 | 相同失败映射，不增加输出数据拷贝 |
| 初始化 | 长函数混合公共绑定、启动换算及子对象成员初始化 | 校验/推导计数 → 绑定分组 → 配置启动 → 安全停机 → Core/速度 PI → ADC_CAL | 物理频率及宽角度运算只在初始化；Core 自行初始化其 PI |
| Iq 上限 | 速度环与限幅检查每拍分别取最小值 | Init 生成 `tSpeed.qIqLimit` | 稳定配置的选择不反复执行；速度环递推公式不变 |
| Core 输出 | 电流、电压和占空比混在 Core 运行对象中；Motor 读取 PI 内部 | 显式 `foc_core_output_t` 输出参数，Motor 持有 `tOutput` | Core 拥有 PI 历史，调用者持有结果数据，职责不反向渗透 |
| 电压限制判据 | Motor 查询 Core PI 上下限、计算幅值 | Core 报告 `bVoltageLimited` | 保留原判据，只转移到它的状态拥有者；不新增电压向量裁剪 |
| Observer 初始化 | 引入 `motor.h`，从完整 Motor 元数据适配 SMO | 接收共享纯数据 `foc_electrical_model_t` | 最小电气模型与算法无关，Position 初始化适配，算法不认识 Motor |
| Motor 头文件依赖 | 引入 Position，间接引入 SMO | 只通过公共采样/反馈/事件数据对接 | 更换观测后端不改变 Motor 的输入输出契约 |
| 公共数据 | 分散在 Motor/Position/SMO 头文件 | 共享物理模型、采样、反馈及事件在 `foc_types.h` | 共享数据不引入父对象、算法或硬件；实例状态仍归各模块 |

Core `Step` 新增输出参数，`Reset` 同时复位输出；SVPWM 新增可选边界报告参数。仓库中的调用者及 host fake 已同步适配；外部调用者也需适配这些签名。

## 单独注明的行为修正

- `bPwmSaturated` 原来没有写入来源，现在由调制在占空比裁剪前报告达到/超出边界的情况；Motor 的采样限幅信息纳入此标志，避免 Position 把受裁剪的区间作为正常接管证据。占空比公式和最终裁剪数值不变。
- ALIGN 的完成动作显式为 `MOTOR_ALIGN_START_RUN` 或 `MOTOR_ALIGN_CAPTURE_ZERO`，不再依靠残留的无感启动标志推断。固定 0 电角度励磁不读取编码器；仅零点捕获后处理需要 Position/编码器。
- 停机失败和 INITIALIZING 中安全停机失败会锁存 PWM 故障；初始化 PI 失败进入故障状态，不再静默丢弃停机返回值。
- 校准状态归组后，重试会保留配置的超时计数，清除采样累加和已用计数；测试覆盖该不变量。

电阻辨识的激励坐标、角度来源和计算公式没有改动。本次只同步电流基值读取位置。

## 首轮整理时的代码量（审核修复前）

| 度量 | 整理前 | 整理后 |
|---|---|---|
| `motor.c` 总行数 | 1052 | 1011 |
| `motor.c` 非空、非注释行数 | 909 | 846 |
| `motor.h` 总行数 | 269 | 274 |
| Motor 直接成员 | 33 | 16 |

Motor 源文件减少 41 行，实际代码行减少 63 行。头文件增加了有职责的分组声明；部分纯数据定义移到共享头文件，不能据此推断整个库或固件按相同比例缩小。未测量 ISR 时间，不宣称本次执行更快。

## 强制边界和尚未完成的部分

实现约束以 [embedded-coding L1.10](../../.agents/skills/embedded-coding/SKILL.md) 为准；字段、单位、拍关系与未来观测后端的质量语义见 [FOC 模块架构第 7 节](../../.agents/skills/foc-motor-control-methodology/references/foc-module-architecture.md)。

运行链路保持 `Prepare(i[k], u_model[k-1]) → Position 最终反馈 → Core/PWM → Observer 更新`，本轮新估计供下一拍使用。磁链观测器、EKF 等后端接在 Observer，接管策略在 Position，Motor 只消费最终反馈和事件；当前只实现 SMO，没有新增空后端。

Motor 的 Core/调制链尚未接收实时母线 pu，也尚未从裁剪后的占空比、死区和管压降重构实际施加电压。Identify 已单独通过 Motor 母线采样 API 使用实时 `vbus_pu` 重构辨识输入；这不改变控制器和 Observer 的旧电压模型。完整控制电压边界仍需独立设计、验证，不能放进具体观测算法补救。

## 首轮验证记录（未覆盖完整测试集）

- 浮点/定点：Core、Motor alpha-beta 采样、速度 pu、参考限幅、Break/停机失败、Position 边界与接管、App 编码器命令、Observer、SMO、电阻辨识及电感失败路径测试通过。
- Core 对照：临时 host 程序分别运行 FLOAT/FIXED 各 50,000 拍，覆盖电压/电流/速度模式、随机角度、正负输入及周期复位。整理前后电流、电压、占空比、两个 PI 状态逐字节一致；新增的边界标志另行测试。此对照使用同一未改变数值公式的调制实现，不证明实际电压模型或整台电机行为。
- 新增 `foc/tests/run_foc_module_boundary_test.ps1`，检查直接依赖、公共数据及 App 调用顺序；注入 SMO 引入 `motor.h` 的临时负例，确认守卫拒绝。首轮未检测 Motor 的传递性 include；审核收尾已增加 GCC 依赖清单及间接负例。守卫仍不代替数值或硬件验证。
- SMO 初始化/热路径源码守卫通过；同时修复已有初始化守卫缺少 `$sourcePath` 和函数定位仍假定 Init 位于文件末尾的问题。
- `make.bat TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software OS=Windows_NT`：`BUILD=debug-rel` 与 `BUILD=release` 均通过，构建日志无编译警告或错误。

| G431 构建 | text | data | bss |
|---|---:|---:|---:|
| debug-rel | 67280 | 592 | 18168 |
| release | 39636 | 420 | 5864 |

以上为当前整个工作区固件大小，包含其他既有改动；不用于宣称单独的性能收益。未进行 MCU 运行或电机硬件验证。

## 审核收尾（2026-10-09）

首轮遗漏完整测试集，审核复现 23 项中的 20 通过 / 3 失败。失败来自波形输出旧字段和 G431 旧 ADC 标度断言；当前已经分别改为 `tOutput` 与模拟网络推算，并同时校验默认配置和 G431 target 的 `119U/3500U`。下方收尾验证才代表当前工作区。

| 补充登记项 | 变化与原因 | 验证/边界 |
|---|---|---|
| 传递性头文件依赖 | Startup 从 Position include 收窄为 `foc_types.h`，App 显式引入自己拥有的 Position | GCC `-M` 检查 Motor 实际依赖，另注入间接 include 负例 |
| Core Init 校验 | 新增空指针、非有限/非正/大于 1 的电压限值拒绝，PI 初始化返回错误向上传递 | Core FLOAT/FIXED 负例与正常 Step 测试；FLOAT 另查 NaN |
| 初始化快照 | `qIqLimit` 和 `qVoltageLimitSquared` 在 Init 计算，热路径消费 | 稳定配置下等价；禁止跨模块直接修改快照，配置改变必须安全重初始化 |
| PWM 饱和与电感辨识 | 调制边界经 Core/Motor/App 填入 Identify，原来恒 false 的退出输入开始生效 | Core 覆盖限幅标志；实际调制产生饱和后的电感采样测试覆盖首拍/已有两拍样本、调用退出接口、锁存、重复调用、拒绝发布及拒绝重启；Motor 为测试替身，App 转发另经源码核对，不等于整链硬件试验 |
| 调试观测移除 | 三相 offset 打印与 CCR 延时/bottomLate 统计随日志清理删除 | 零偏改用 ELF 布局的只读 peek；CCR/PWM 需受控仪表测量，尚未实测 |
| PWM enable 失败 | 路径改为由统一故障入口执行一次安全停机，避免先停机再重复进入故障停机 | Break/PWM 停机失败测试；不是取消安全停机 |
| Init 状态发布竞态 | 在开启 ADC 触发前发布 ADC_CAL，避免触发期间锁存的 FAULT 被覆盖 | 同步故障注入在旧顺序下失败、修复后 FLOAT/FIXED 通过 |
| 名称/存储契约 | 物理速度宏不再用 `_PU`，统一 `qElectricalSpeedBaseHz`；明确 Hz×32768 编码与转换位置 | 数值及 PI 整定公式不变；定点小增益量化仍是此前登记限制 |
| 小项 | 初始化局部返回码、补回 NULL 防御、校准复位显式传入超时、修正捕获标志归属和 Stop 注释 | 复位、故障与 App 回归覆盖 |

电阻辨识的 dq 矢量 0.14 pu 和电感辨识的逐轴 ±0.10 pu 是现有不同激励的退出策略，本轮明确保留，没有统一保护算法。App 为电阻也提供 qCurrentQ 是此前矢量保护所需，属于参数调整基线，不是本轮公式整理偷偷新增。两种软件判据都依赖有效采样，不能替代硬件保护。

### 尚未完成的边界

- App ISR 仍读 Motor 的 `eState/tInput`，以及 Identify 操作状态。这是已登记的遗留边界偏差，本轮不新增带临界区的 GetStatus 调用；后续应设计最小 ISR 数据快照/阶段结果接口，并验证拍关系。`tOutput` 是公开纯数据结果，读取它本身不属于窥探 Core/Motor 内部状态。
- Identify 仍 include `motor.h` 并把 Motor 传入 ISR/流程控制，包括调用即时退出接口。该耦合尚未整理；下一步需限定控制命令与最终结果数据，不能宣称整个 FOC 库已完全解耦。
- Core/调制链的实时母线 pu 和实际电压重构、硬件电流校准、控制环/接管可靠性及 ISR 时序仍未完成。本轮不扩展 Flux/EKF 空接口，不恢复复杂诊断。

### 收尾验证

- 完整执行当前 `foc/tests/run_*.ps1`：23 / 23 通过；包括波形、ADC 契约和依赖该契约的对齐检查。修改后的 Core/App FLOAT/FIXED 测试也已单独复跑；电感补测加强了前台后的锁存与拒绝重启断言，再次 FLOAT/FIXED 通过。
- 传递性依赖负例：仅将临时副本的 `motor_startup.h` 改回引入 Position，真实 `-M` 清单守卫拒绝；未修改生产文件制造负例。
- G431 `debug-rel/release` 均经 `make.bat` 构建；日志没有编译警告/错误，ELF/BIN/HEX 齐全。不是只依据批处理返回码判定。
- `git diff --check` 通过；当前改动 C/H 无超过 82 列的行，本次存档相对路径链接检查通过。

| 当前 G431 构建 | text | data | bss |
|---|---:|---:|---:|
| debug-rel | 67280 | 592 | 18168 |
| release | 39636 | 420 | 5864 |

复现构建命令（先创建目录，`BUILD_DIR` 使用 Windows 反斜杠）：

```powershell
New-Item -ItemType Directory -Force build\motor-review-debugrel, build\motor-review-release | Out-Null
.\make.bat TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software OS=Windows_NT BUILD=debug-rel BUILD_DIR=build\motor-review-debugrel
.\make.bat TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software OS=Windows_NT BUILD=release BUILD_DIR=build\motor-review-release
```

审核修复后的 `motor.c` 为 1013 行 / 847 非空非注释行，`motor.h` 为 281 行；Motor 顶层仍 16 个直接成员。相对于本记录声明的起始工作区，源文件净减 39 行、实际代码净减 62 行。新增范围/存储/故障契约使头文件略增长，不能将头文件行数增长等同于运行开销。

以上更改均未提交、未烧录。原始参数/SMO 阶段及后来删除的诊断测试之间的关系见 [2804 参数存档](2026-10-09-g431-2804-parameter-change.md)、[SMO 模板记录](2026-10-09-smo-algorithm-template.md)；零偏和 CCR/PWM 的后续观测流程见 [采样审计](g431-current-adc-and-identify-audit.md)。
