# FOC V1 收敛与开发基线

更新：2026-10-10。本记录承接全量框架审核及用户批准的优化实施，当前契约以 [主架构](foc-architecture.md) 为准。

## 已落实的优化

| 范围 | 行为和边界 |
|---|---|
| Motor 参考 | 无感接管后改速更新斜坡目标，避免下一拍被旧目标覆盖 |
| Motor 安全 | ALIGN 电流统一受相电流限制；校准未完成时清故障恢复 ADC 触发，功率 PWM 保持关闭 |
| 速度环 | PI 输出范围与有效 Iq 范围求交，避免积分在外部电流裁剪后继续累积；非法范围及非有限值拒绝 |
| Core | 按最新要求恢复原实现，撤回新增径向限幅、快速路径及 PWM 抗饱和反馈 |
| PID / 数学 | Init 校验增益与有限边界；移除无消费者的旧类型、角度和算术兼容层，保留统一 scalar 开方 |
| App 配置 | 内嵌 Motor 配置始终为 PU；物理速度 profile 仅在 Init 换算一次。频率必须等于目标宏，必需 Encoder 不可静默降级 |
| Position | 静态与回退绑定统一使用 tSensor 上下文，删除重复 pSensorState |
| 精简 | 九个未接入算法移至 experimental；删除 controller 运行时 vtable 及 LADRC 适配；主线编译集合保持明确 |
| 验收 | 修复 GCC 路径及 PS5 预期编译失败处理；增加真实模块组合矩阵及失效恢复回归 |

Core 的 tVoltage/tVoltageAlphaBeta 恢复原来的调制度指令语义，未进行径向缩放。Observer 的模型电压仍由 Motor 使用实际母线重构上一控制区间电压 PU。两者单位和用途不同。用户反馈控制耗时并明确要求优先回退后，已一并删除径向/PWM 限制反馈、PID 前拍积分快照、配套接口、新增性能及矢量测试；保留 PID 初始化校验及原有单轴条件积分。

## 验证与集成

合入远端前，G431 debug-rel FLOAT 和 release FIXED 均完成编译、链接及 bin/hex 生成；实验算法 FLOAT/FIXED 独立语法检查通过。新增矢量与 PWM 抗饱和回归曾通过，现已随相应功能撤回。真实 App/Motor/Position/Observer/Core 组合覆盖 SENSOR/HARD_DRAG × NONE/SMO × FLOAT/FIXED × static/fallback，共 16 个组合，含校准、缓存失效、提交失败、停机与重启代次。

远端 master-dev 已快进到 `d135d86333ba515470709f005c119458e0bf12e2`（adapt v0.6.1.3）。MODUS 更新到远端指定的 `b96b2acc4fec79defdd7adf3ffe6fa90c2e265c3`，global_define.h 的版本值为 0.6.1.3。版本标签尚停留在 v0.6.1.2，验收以实际提交及版本宏为准。

先保存本地变更，合入远端和子模块，再恢复 FOC 修改。唯一双方修改的 G431 foc_adapter.h 自动三方合并成功：保留远端 MDI PWM 生命周期/故障门控和 CH4 ADC 触发，同时保留本轮统一的 tSensor.pContext。没有遗留文本冲突，未提交或推送本轮优化。

合入后重新运行全部 26 个 FOC 主机脚本，失败数为 0；包含上述 16 组合矩阵。MODUS MDI 测试入口 `python modus/src/mdi/tests/run_tests.py` 全部通过，包括 G431 PWM 故障门控、ADC DMA、模板服务、实际 MODUS Init 次序、预期编译拒绝及 Cortex-M4 -O2/-Os 代码生成检查。

合入后使用独立全新构建目录完成 G431 构建，两者均无编译警告/错误，生成 ELF/BIN/HEX：

| 构建 | text | data | bss | 构建目录 |
|---|---:|---:|---:|---|
| debug-rel FLOAT | 74356 | 592 | 18288 | build/foc-v1-integrated-float |
| release FIXED | 41816 | 420 | 5976 | build/foc-v1-integrated-fixed |

构建入口为 make.bat，显式使用 TARGET_CHIP=stm32g431、SW_ROOT=D:/software、FOC_NUMERIC=float/fixed。以上尺寸为字节，不能从不同构建模式的差值推断算法收益。

## 固定开发契约

2026-10-11：七个无状态基础数学运算移到 numeric 头文件内联，删除原外部实现，模块接口和状态归属不变；FOC 禁止乘加收缩以保留舍入顺序。Core/PID/Motor 目标文件分别消除 31/16/17 处基础运算调用点。26 个回归脚本和 FLOAT/FIXED 各 100121 组旧实现对照通过。G431 debug-rel FLOAT 的 text 从 73984 降至 73264；release FIXED 因展开饱和逻辑从 41008 增至 43480，data/bss 均不变。构建补充标准头文件依赖跟踪。本轮没有 OpenOCD 连接，保留已运行的软件 atan2，CORDIC 切换及 30 μs 目标仍待目标板验证；未烧录。

2026-10-10 按 Ponytail 复核 G431 ADC→App→Motor→Position/Core/Observer→PWM 和波形、计时路径：撤回径向限幅；电流范围校验改为等价乘积比较，FLOAT 不再为校验执行三次 Q15 整数除法；删除编码器初始化已保证的非零频率重复判断；合并 ADC W1C 清标志写入和重复的强拖候选清空分支。保留动态采样、位置过期、数值范围、PWM 故障及公共接口边界检查。26 个主机脚本通过，另补测编码器零频率初始化拒绝；G431 debug-rel FLOAT / release FIXED 构建通过，text/data/bss 分别为 73984/592/18320 和 41008/420/5968 字节。没有新增运行时策略或快慢两条路径；耗时和 fault=0x4 是否消失仍待真机复测。

固定 Motor / Position / Observer / Core 职责、共同基值与单位、i[k] / u[k-1] 拍关系、编译期绑定、对象状态所有权及失败安全收敛。扩展算法应保留独立 cfg/input/output/state 和现有回归；experimental 不属于已验收能力。

这次没有烧录或台架验证。实际 PWM/ADC 生效点、死区/压降、母线样本时效、WCET、栈峰值、静止辨识的转子保持仍需硬件验收；既有实时调用链超过技能三层规定的差距仍保留记录。因此这是源码开发基线，不宣称所有硬实时规范已闭合。
# 2026-10-11 ISR 第二轮精简

- 波形准备改为 MODUS `mwaveform.Step` 接受采样后直接调用：复用停止、抽取与队列满判定，保持六通道 10 kHz 同拍采集。删除 FOC 波形 Step 纯转发函数；不增加运行时回调或计数器。MODUS 子模块 `src/mdebug/mwaveform.c` 有配套本地修改，移植时须一起保留。
- Identify 的现有操作查询改为内联，保持 NONE 早退及故障收尾；不新增活动标志。
- 电流转换保留原 64 位差值及范围检查。当编译期电流基值计数不超过 Q15 尺度时，检查通过已证明差值可用 int32 表达，避免三次软件 int64 转 float；大尺度配置保留原转换。
- 27 个主机测试脚本通过；新增真实 MODUS 波形停止、抽取、满队列、恢复和同拍捕获测试。电流测试扩展到两种极性、FLOAT/FIXED、119/1000/32768/32769 四种尺度，核对原宽数值公式及极端输入。
- G431 debug-rel FLOAT 构建 text 73264 → 73232，data/bss 不变；release FIXED text 43488。尚未烧录，实际微秒收益待相同 `make auto` 配置复测。计时方法、SMO 软件 atan2、控制算法及动态保护未变。

