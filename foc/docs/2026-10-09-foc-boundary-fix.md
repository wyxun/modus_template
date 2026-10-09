# FOC 边界修复记录

日期：2026-10-09。对应 [框架审核](2026-10-09-foc-framework-boundary-audit.md)，在当前工作区实施；未提交、未烧录。

## 修改前后与原因

| 项目 | 修改前 | 修改后 | 原因 |
|---|---|---|---|
| 电压模型 | Observer 收到 Core 调制度；Identify 在 App 中另用母线重构 | Motor Prepare 用上一提交占空比及本拍母线重构统一 αβ 电压 PU；双方消费同一快照 | 母线变化与占空比裁剪不能在不同算法中产生不同电压语义 |
| 模型失效 | 没有共用有效性契约 | 发布 `bVoltageModelValid`；失效时 Position 清观测历史/候选，辨识按活动采集阶段中止 | 零值不能冒充有效的零施加电压 |
| 电流测量范围 | ADC 差值隐式裁到 ±COUNTS_PER_BASE | 保留宽 PU 电流；用 int64 差值检查共同数值范围，异常进入采样故障 | 电流基值是尺度，不是 ADC 满量程或保护阈值 |
| 定点坐标变换 | Clarke/Park 使用会裁输入到 ±1 的 `foc_mul_pu` | 电流坐标变换改用 `foc_mul_wide` | 超过 1 pu 的有效测量必须在 FLOAT/FIXED 中保持相同含义 |
| App 状态访问 | 直接读取 Motor 状态和 Position 内部交接计数 | Prepare 发布 `bAlignActive`；Position 通过 `CheckStartupBudget()` 校验初始化预算 | 外部逻辑不依赖对象内部状态布局；不增加一串成员 getter |
| 控制失败后的观测 | 只凭位置成功仍会推进 Observer | 位置和控制均成功才推进，失败后清候选及历史 | PWM/Core 失败与估计有效性的生命周期一致 |
| Encoder / Identify 调度 | App 按 Rs/Ld 操作跳过 Encoder；Encoder 失败可能跳过辨识 | 启用 Encoder 每轮正常服务，随后独立推进 Identify；整轮完成后报告错误 | 两个模块无调度依赖，不由辨识枚举决定传感器是否执行 |
| Rs/Ld 内核依赖 | 采集、窗口和公式辅助函数收到整个 Identify | 采集/均值/公式只接收自身子状态；公开 ISR 入口留在对应源文件 | 隔离算法内部状态，并保持入口与实现可直接查找 |
| 日志与安全检查 | LOG=0/1 各维护一份安全条件 | 两种构建调用同一套判定，日志仅输出故障原因 | 防止关闭日志改变安全行为，减少重复条件 |
| 公共 Identify 头文件 | 包含 `motor.h`，传递硬件依赖 | 只前向声明 Motor，公共数值类型不引入硬件 | 前台会话仍能显式借用 Motor，算法数据头不依赖硬件实现 |
| Position 速度滤波 | 热路径固定 alpha=0.02 | Init 按执行频率推导同时间常数系数；20 kHz 行为保留 | 换执行频率时不隐式改变滤波动态 |

## 关键契约

- `i[k]` 与 `[k−1,k]` 理想模型电压配对。Motor 在本拍 Core 计算前重构模型，不把新命令配给旧电流。
- Core 的电压字段仍是调制度命令，当前 PI 增益和限幅未改成物理电压 PU 输出。统一模型位于 Motor 的测量/执行边界，不在 SMO 内补母线比例。
- 纯强拖/有感控制可以在模型无效时继续用自己的有效位置；无感估计会失效，已接管的 SMO 通过原有丢失路径停机，不回退到不合格候选。
- `qMaxPhaseCurrent` 当前是参考约束，不能称作实测过流检测；保留硬件 break 安全路径。ADC 表达范围错误与参考超限分别处理。
- 固定轴辨识继续绕过 Position/Observer，只在 Motor 的辨识会话内允许无反馈控制。参考请求下一控制步骤应用，中止当前 ISR 生效。
- Parent 的活动操作、错误锁存、前台 Motor 会话和结果标签仍在现有框架中。没有增加通用回调、统一可扩展样本或共享静态算法状态。

## 验证

先增加电压模型、电流范围及独立调度的失败断言，观察旧代码失败，再实施修复。新采样回归使用真实 Motor/Core/调制代码，包含：

- FLOAT/FIXED × 119/1000 counts/base × 两种电流极性，共 8 个构建运行组合。
- 占空比裁剪、0.8 pu/3 pu 母线、零输出、母线无效、超过 1 pu 电流、异常 ADC 数值与采样故障。
- 当前电流与上一提交占空比的拍关系、无效模型清候选、控制失败清观测历史。
- Encoder 失败时仍推进 Rs/Ld，固定辨识不调用 Position，调制度请求拒绝时立即中止。

以下测试入口均通过：

| 范围 | 入口 |
|---|---|
| 统一采样边界 | `run_motor_sample_boundary_test.ps1` |
| 接口/依赖守卫 | `run_foc_module_boundary_test.ps1`、`run_identify_pu_boundary_test.ps1` |
| Rs/Ld 数值与异常 | `run_identify_resistance_test.ps1`、`run_identify_inductance_failure_test.ps1`；各覆盖 FLOAT/FIXED 与 LOG=0/1 |
| App 独立调度/请求/故障 | `run_encoder_command_test.ps1` |
| Position 与强拖 | `run_motor_position_boundary_test.ps1`、`run_sensorless_position_handoff_test.ps1`、`run_sensorless_startup_test.ps1` |
| Motor | `run_motor_alpha_beta_test.ps1`、`run_motor_break_fault_test.ps1`、`run_motor_reference_limit_test.ps1`、`run_motor_speed_pu_test.ps1` |
| 数学及估计 | `run_core_step_test.ps1`、`run_observer_contract_test.ps1`、`run_smo_test.ps1` |

G431 构建通过：

| 配置 | 命令（均经 make.bat） | ELF text/data/bss |
|---|---|---|
| debug-rel FLOAT | `TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software -j4` | 70900 / 592 / 18232 字节 |
| release FIXED | `TARGET_CHIP=stm32g431 SW_ROOT=D:/0_software BUILD=release FOC_NUMERIC=fixed BUILD_DIR=build/foc-boundary-fixed -j4` | 40508 / 420 / 5952 字节 |

以上是当前整体工作区构建结果，不是本次修改前后的体积差；工作区已有其他未提交变更。检查构建日志没有编译错误/警告，确认生成了本轮 ELF。

## 保留限制

没有进行真机、WCET 或栈深度验证。理想电压模型使用最新母线样本，实际 PWM 生效点、母线样本年龄、死区和器件压降仍需硬件证据。Ld 横轴扰动门槛不能证明转子静止。

实时调用链仍超过 embedded-coding 的三层深度规定，本次没有擅自更改规范、排除 inline 或把数学公式合并回 Motor。App Init 的既有长度、Position 绑定布局等后续简化也未据本次测试宣称全部合规。HFI/Flux/EKF 和最终矢量限幅/抗饱和反馈没有在本次实现。
