# FOC 框架 V1 收敛实施计划

用户已批准落实全量审核中的优化项。本轮保留 Motor / Position / Observer / Core 的职责和调制度语义，不提交、不烧录。采用 writing-plans 与 subagent-driven-development 技能，独立文件组分别实施，再统一审查和验证。

## 设计和验收

- Motor 的目标速度与执行参考分开，SetSpeedReference 更新无感目标；ALIGN 限流统一在 Init 校验；清故障依据校准完成状态恢复仅采样触发；速度 PI 输出范围与有效 Iq 上限求交。
- Core 保留原有 PI 单轴限制和 SVPWM 裁剪；按用户要求撤回新增径向限幅及 PWM 抗饱和反馈。
- PID Init 拒绝非有限边界及无效增益。清除无调用者的旧角度/类型/算术兼容层，保留 SMO 所需开方。
- App 的嵌套 motor_cfg_t 始终采用 PU；物理用户 profile 只在唯一绑定点换算。执行频率唯一来自目标配置，缺失必需位置能力或非法来源初始化失败。
- Position 静态与回退路径使用同一传感器上下文。组合测试连接实际 App/Motor/Position/Observer/Core，端口夹具模拟硬件，覆盖来源/后端/数值与失败恢复。
- 未接入组件归档到 experimental；删除无消费者的运行时 controller vtable，保留独立算法。验收脚本统一工具查找及 PowerShell 行为。

## 执行清单

- [x] Motor：添加改速、ALIGN、校准恢复、Iq 抗饱和失败测试，实施修复并运行相关回归。
- [x] Core / Numeric：保留 PID 初始化校验和数学兼容层清理；撤回径向限幅、PWM 反馈及配套测试。
- [x] App / Position：添加配置拒绝、PU 绑定、静态绑定组合测试，实施配置与上下文收敛。
- [x] 外围：归档十个未接入编译单元，移除运行时控制器适配；修复两处测试脚本环境问题。
- [x] 复核：逐项检查需求和代码质量，运行全部主机测试及 G431 FLOAT/FIXED 目标构建。
- [x] 文档：同步主架构和 README，记录冻结契约、验证证据及真机/WCET 限制。

独立实施者先观察新增断言在旧实现失败，再修改代码。最终目标构建由根代理串行执行；实际硬件行为仍需单独验收。

最终验收：合入 master-dev d135d86 / MODUS 0.6.1.3 后 26 个 FOC 主机脚本全通过，MDI 回归全通过，G431 FLOAT/FIXED 全新目标构建通过。保留硬件时序、WCET、栈和既有调用深度限制，详见 V1 收敛记录。

最新范围调整：按用户要求回退 Core 新增径向限幅、快速路径及 PWM 抗饱和，删除配套 PID 状态/API 和测试；保留原控制路径、独立缺陷修复及每秒计时。后续修改默认使用 Ponytail，优先复用和删除。
