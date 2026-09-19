# MESC 参数辨识验证与验收记录

## 1. 软件环境与基线信息

- **当前工作区 Commit:** `7fd3b1ff84c6b430dd6edebda201a7e94a237ea2` (with uncommitted working tree modifications)
- **MESC 参考 Commit:** `aa159cc6f0b55aee0d07dabd10055dfd89d20317`
- **基线测试状态:**
  - `run_identify_test.ps1`: FLOAT / FIXED Identify tests passed (旧最小实现基线)
  - `run_encoder_command_test.ps1`: FLOAT / FIXED FOC App tests passed (默认辨识关闭)
- **MDI 验证入口:** MDI 硬件抽象层的契约、负编译和零开销检查统一由
  `python modus/src/mdi/tests/run_tests.py` 执行；本目录只保留 FOC 自身的算法与
  编码器测试。

---

## 2. 验证矩阵（四栏跟踪）

| 类别 | 验证项 | 状态 | 证据 / 结果 |
|---|---|---|---|
| **主机验证 (Host)** | 基线单元测试与应用测试 | 已通过 | `run_identify_test.ps1` / `run_encoder_command_test.ps1` 退出码 0 |
| 主机验证 (Host) | 独立 RL 模型红测与数值收敛 (Task 1) | 已通过 | `foc_identify_plant.h` 建立，PU↔SI、边界校验、生命周期与安全壳通过 (FLOAT/FIXED) |
| 主机验证 (Host) | Rs 闭环与冻结平均测试 (Task 2) | 已通过 | 独立 RL 模型驱动闭环 PI，R={0.25,0.5,1.0}Ω 及偏置鲁棒性测试通过 (误差<0.4%，FLOAT/FIXED) |
| 主机验证 (Host) | 正负积分窗口与 Ld/Lq 估计测试 (Task 3) | 已通过 | 梯形校正、多周期平滑、分散度检验通过，Rs<0.1%, Ld/Lq<0.08% (FLOAT/FIXED) |
| 主机验证 (Host) | Motor 按需 metrics 视图与同一 ISR 契约 (Task 4) | 已通过 | `motor_CaptureStepMetrics` 零拷贝接入，`run_encoder_command_test.ps1` 100% 通过 |
| 主机验证 (Host) | 应用安全、状态流与停机确认 (Task 5) | 已通过 | 邮箱仲裁、运动命令互斥锁、前台停机监控与超时保护完成并通过基线测试 |
| 主机验证 (Host) | 完整集成 App/Motor/Core/Identify (Task 6) | 进行中 | `foc_app_identify_test.c` 已构建，9大场景正在调优 Mock position 依赖 |
| **目标构建 (Target Build)** | STM32G431 FLOAT/FIXED debug/release (IDENTIFY=1) | 未验证 | 待执行 |
| 目标构建 (Target Build) | STM32G431 / AT32F413 release (IDENTIFY=0) | 未验证 | 待执行 |
| 目标构建 (Target Build) | 编译宏与 map 符号审计 (关闭时无 identify 符号) | 未验证 | 待执行 |
| **目标时序 (Target Timing)** | PWM 提交至生效延迟 D 上界分析与示波器实测 | 未验证 | `bIdentifyTimingVerified=false` (默认拒绝真实启动) |
| 目标时序 (Target Timing) | JEOS 中断与 TIM1 更新相位裕量分析 | 未验证 | 待台架测量 |
| **真机台架 (Bench Test)** | 零输出启动、Stop、Cancel、Break 停机安全性 | 未验证 | 待台架测量 |
| 真机台架 (Bench Test) | 独立已知 RL 负载精度检验 (Rs≤10%, L≤15%) | 未验证 | 待台架测量 |
| 真机台架 (Bench Test) | 连续 10 次重复性检验 ((max-min)/mean≤5%) | 未验证 | 待台架测量 |
| 真机台架 (Bench Test) | ISR 最坏耗时分析 (≤6800 cycles) | 未验证 | 待台架测量 |
