# Observer 独立算法与组合边界整理

日期：2026-10-09。目标是搭好算法契约和组合入口，保留现有 SMO 公式及强拖路径；不实现 HFI、Flux、EKF，不烧录硬件。

| 内容 | 修改前 | 修改后 | 原因 |
|---|---|---|---|
| SMO 输入 | 四参数 Step，借用 Observer 定义的输入语义 | 独立 `foc_smo_input_t`，Step 接收自身输入 | 新算法不改变 SMO 契约 |
| 通用估计 | 包含 SMO 的反电势强度 | 仅角度 BAM32、速度 PU、有效性 | 质量指标不能在不同算法间混用 |
| SMO 结果 | 别名指向通用结构 | `foc_smo_output_t` 含 `tEstimate` 与 `qBemfStrengthPu` | 保留指标真实语义和原有数值 |
| Observer 输入/结果 | 平铺两个指针，单一缓存结果 | `tSmo` 独立输入与候选结果组成集合 | 后续按算法新增成员，保持既有语义 |
| 对接方式 | Position 读取、修改 `tObserver.tOutput` | Step 输出 `foc_observer_estimates_t`；Position 拥有快照 | 清楚划分状态与结果所有权 |
| 算法选择 | 单后端排他分支 | 保留构建参数名，按位集合；算法独立条件块直接调用 | 给并行算法组合留下明确接入位置 |
| 错误边界 | 某些空输入错误保留旧有效结果 | Step 在输出可用时先清空；失败结果无效 | 防止使用错误调用前的陈旧估计 |
| 初始化/复位 | 配置失效时可能保留旧实例 | 重新初始化失败使组合不可执行；Reset 只清历史 | 区分参数绑定与运行历史 |

## 已有组合

- NONE + HARD_DRAG：纯强拖，由强拖候选直接产生最终反馈。
- SMO + HARD_DRAG，关闭接管：SMO 旁路估计，最终反馈仍为强拖候选。
- SMO + HARD_DRAG，启用接管：保留资格判定、渐变接管、估计丢失事件、故障处理。
- SENSOR + 可选 SMO：保留传感器反馈路径和旁路估计。

输入仍为 `i[k]` 与上一控制区间 `u_model[k-1]`，Observer 在本拍控制提交之后推进，新估计供下一拍选择。此次只改类型、所有权及失败输出，不更改 Observer 电压模型；实际母线电压重构目前属于 Identify 的独立路径。

## 扩展规则

新增算法自行定义输入、配置、状态、结果，并作为成员加入 Observer 组合；Init/Step/Reset 独立编译块直接调用。并发算法不用 union，共用基础坐标及通用估计，独立保存证据。

HFI 是预留算法位，当前选择 HFI 或 HFI+SMO 均编译失败，`FOC_ENABLE_HFI=1` 也明确拒绝。落实 HFI 时需加入独立输入、注入请求、Core 电压预算及采样时序，并补充 Position 双向融合策略。Observer 不拥有强拖状态、不自行决定反馈采用谁、不直接访问 PWM。

## 验证

先用旧接口复现：有效 Step 后以 NULL 输入再次调用，旧输出仍有效；失败断言确认该问题。新接口覆盖 NULL 输入/输出、缺少采样指针、未初始化、失败重初始化、非有限输入、实例隔离与复位。

组合入口与独立 SMO 对同一 512 拍变化电流/电压输入逐字段比较，复位后再重复一次，FLOAT/FIXED 要完全一致。既有 SMO 理论值测试继续验证递推公式和限幅冻结。

额外验证 NONE 与 SMO 两种构建、拒绝不支持算法配置、强拖旁路 SMO、不自动接管、强拖切 SMO、环绕角渐变、估计丢失以及 Position 的纯强拖/传感器边界。目标构建使用仓库 make.bat。无硬件运行证据，不声称双无感或真机性能已验证。


实测结果：

| 检查 | 结果 |
|---|---|
| Observer 组合入口，NONE/SMO × FLOAT/FIXED | 全部通过；每次错误清输出、失败重初始化、实例隔离、复位后逐拍一致 |
| 不支持的 HFI、HFI+SMO、未知位及 SMO 开关冲突 | 全部按预期拒绝编译 |
| SMO 理论递推、原始滑模反馈、定点初始化和热路径守卫 | 通过 |
| 强拖旁路 SMO、强拖切 SMO、非自动接管、估计丢失和环绕渐变 | FLOAT/FIXED 通过 |
| 纯强拖/传感器 Position 边界与无感启动 | FLOAT/FIXED 通过 |
| Motor αβ、App 命令/波形回归 | FLOAT/FIXED 通过；波形超通道编译负例按预期拒绝 |
| FOC 模块边界与 Identify PU 接口守卫 | 通过 |
| G431 FLOAT 默认构建与 FIXED release 构建 | 均成功生成 ELF，日志未检出编译警告或错误 |

构建命令：`make.bat TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software -j4`；定点另加 `FOC_NUMERIC=fixed BUILD=release BUILD_DIR=build/identify-fixed-check`。没有提交、烧录或运行真机。
