# FOC Sguan SMO 适配设计与实施计划

> 本文只定义 SMO 算法如何接入已统一的无感核心框架。模块拓扑、控制时序、公共
> Observer 契约和反馈源状态机以[无感估算核心框架设计](../specs/2026-09-13-foc-sensorless-framework-design.md)
> 为准，本文不重复定义。
>
> 当前工作仅收敛设计文档；以下内容不授权本次修改源码。源码实施须在框架与算法方案
> 审核通过后另行确认。

**Goal:** 将 SguanFOC v3.1.0 的 SMO 与其配套 PLL 算法适配为首个 Observer 实现，输出
符合核心框架的电气角度、速度和有效状态，并可先 Shadow、再受控切换闭环。

**Architecture:** Observer 公共边界、Motor 调用顺序和反馈切换完全遵循核心框架文档。
SMO 实现拥有 α/β 模型状态及配套 PLL 状态；所有运行状态按实例保存。电流采样周期和
离散系数在初始化时确定，Step 不执行角度裸差分，也不逐拍传 `dt`。

**Tech Stack:** C；`foc_scalar_t` float/Q15；现有 BAM32 电角度、Clarke/Park、SVPWM；
主机端 float/fixed 算法测试；Mermaid 只用于解释 SMO 内部信号关系。

---

## 1. SMO 来源与本计划边界

参考实现位于本机 `E:\Project\SguanFOC_Library\SguanFOC库v3.1.0(无感foc，浮点运算，进阶)`：

- `Sguan_SMO.c/.h`：α/β 电流模型、交叉轴估算、积分限幅冻结、反电势低通；
- `Sguan_PLL.c/.h` 与 `SguanFOC.c` 的 `Transfer_SMO_Loop()`：SMO 相位误差、PLL 递推、
  极对数换算及前一拍电气速度回馈。

不能只移植 `Sguan_SMO.c` 后再对每拍 atan2 角度做差分。原工程的角度和速度由
`Transfer_SMO_Loop()` 与 PLL 联合产生；该耦合关系是本次算法参考的一部分。不开源库中的
控制模式、应用层、硬件、调试命令、全局对象或其它算法实现。

源码出处有一项实施前 Gate：v3.1.0 源码文件头标注 “All Rights Reserved”，库根目录
同时存在 MIT `LICENSE`。两者文字不一致。确认适用授权和需要保留的版权声明之前，不
逐段复制源代码；可以先完成方程映射与独立实现评审，但不得忽略来源声明。

本计划允许后续实施时修改的范围仅限 `foc/` 内框架、算法、构建清单、测试和 FOC 文档。
不改 `peripheral/`、`target/`、MODUS、perf_counter 或其它第三方源码；若算法验证发现
这些边界必须变化，先停下报告并请示。

## 2. 算法信号链与方程映射

Sguan SMO 在静止 α/β 坐标分别估算电流和反电势，电流模型包含电阻、电感、估算电气速度
以及交叉轴电流。对每个轴 `x`，原实现等价于：

```text
model_error_x = Vx/Ld - (Rs/Ld)*Ix
                - ((Ld-Lq)/Ld)*omega_e_prev*Iy_pred
                - Ex_hat/Ld

Ix_hat += (Ts/2) * (model_error_x + model_error_x_prev)
Ex_hat  = LPF( h * sign(Ix_hat - Ix) )
```

其中 `Iy_pred` 为另一轴的预测电流；积分器到达限幅时按原算法冻结并按其解除条件恢复。
反电势低通使用 `Ts` 与截止频率 `Wc` 离散化。Init 时预计算：

```text
I_num  = Ts / 2
F_num  = (Ts * Wc) / (2 + Ts * Wc)
F_den  = (-2 + Ts * Wc) / (2 + Ts * Wc)
Gain0  = 1 / Ld
Gain1  = Rs / Ld
Gain2  = (Ld - Lq) / Ld
```

实现需逐项对照参考代码的离散递推、轴间连接、符号、积分冻结和滤波历史值；不得仅凭
连续时间公式重写后假设行为相同。参数 `Ts/Rs/Ld/Lq/Wc/h/积分限幅` 的单位、输入
电压电流基准及可表示范围都要在设计表中列明。

Sguan 的完整耦合路径使用上一拍 PLL 电气速度驱动 SMO 模型，以 PLL 角度投影估算反电势
生成相位误差，再由离散 PLL 更新角度和速度。适配必须保留这一反馈闭环；速度由 PLL
跟踪结果给出，不由 `foc_angle_diff()` 裸差分给出。需要逐项核实原实现中 `OutRe`、
`OutWe` 与极对数相乘的机械/电气含义，最后映射到项目单位：电角度为 BAM32，电速度为
电气圈/秒。

原 `Transfer_SMO_Loop()` 把 `SMO_We` 声明成函数内 `static`。移植时将该状态归入每个
`foc_smo_t` 实例；SMO 积分、滤波历史、PLL 角度/速度及有效性所需状态也全部实例化，禁止
跨电机共享文件级/函数级运行状态。

## 3. 数值、采样和有效性要求

- `Ts` 来自固定高频控制周期，在 Init 计算离散系数；运行时 Step 只接收本拍 `Iαβ[n]`
  和代表前一采样区间的 `Vmodel[n-1]`；两者均为 `foc_ab_t`。具体时序/电压模型门槛遵守
  核心框架 Gate。
- 项目 fixed 后端是 Q15 归一化标量。不能把物理量或可能大于 1 的 SMO 系数直接强转成
  `foc_scalar_t`。Gate 0 必须确定电流/电压标幺基准或等价定标、每个系数的 Q 格式、乘加
  中间精度、移位/舍入和饱和策略，并证明积分与滤波各中间量范围可表示。不要只以“使用
  32 位”代替定点范围分析。
- 先以 float 建立 SMO 方程的参考行为，再用相同输入向量验证 Q15 误差和饱和边界。双后端
  兼容仍是完成条件；若 Q15 精度或动态范围不够，暂停并报告，不静默退回 float。
- 对 SMO/PLL 的 `bValid`，要求同时满足算法锁定质量和反电势/速度可用条件，并能在失锁
  后撤销。相位误差窗口、最小反电势、最小/最大速度和持续样本数通过仿真与台架确定，
  不能在计划中凭经验编造。它们是算法内部质量判据，不另建外层 Observer 状态机。
- ALIGN 和带编码器起转仍由 Motor 负责。SMO 在 Encoder 控制下逐拍运行用于 Shadow；
  首次验证不宣称零速无感启动。切换阈值、故障策略和状态转换遵循核心框架文档。

## 4. 实施文件与任务

### Gate 0：参考来源、物理单位与电压时序

**检查文件：** 本机 Sguan v3.1.0 的 SMO、PLL、Transfer 源码和根目录许可证；当前
`foc/motor/motor.h` 参数单位；`foc/middleware/foc_core.c` 电压输出；现有 FOC 主机测试。

- [ ] 完成来源/许可证声明核实；未核实时不复制原代码。
- [ ] 对照参考实现列出每个输入、状态、系数和输出的量纲、极对数作用位置及正方向。
- [ ] 核实 Core 电压命令到 `Vmodel[n-1]` 的缩放、极性、限幅及采样区间关系；确认母线
      电压缺失是否限制本次试验区域。
- [ ] 确认现有 motor 参数 `Rs/Ld/Lq` 的物理单位转换，以及 float/Q15 两后端的范围。
- [ ] 建立 fixed 后端的量纲/标幺基准、Gain 系数格式、宽中间乘加、移位舍入和饱和范围
      说明；特别证明 `1/Ld` 等系数不会被错误塞入 Q15 标量范围。

任何一项未知会导致算法模型或移植授权不确定时，停在 Gate 0，不进入硬件闭环测试。

### Task 1：落实核心框架的最小控制周期接缝

**文件：** `foc/foc_types.h`、`foc/middleware/foc_core.h/.c`、
`foc/motor/motor.h/.c`、`foc/observer/foc_observer.h/.c`、`foc/foc.mk`、
`foc/tests/`。

- [ ] 按核心框架文档实现唯一 Observer 边界；初始化固定算法和 `Ts`，每拍推进输入只有
      `Iαβ` 与上一采样区间电压模型值。Init 绑定一个 typed Step 函数指针，Motor 直接
      调用；不增加只负责转发的 `foc_observer_Step()` 调用帧。
- [ ] 电压历史使用 `foc_ab_t`，由 Core 的 `foc_core_Reset()` 清为零。Motor 初始化和每次
      PWM 重新使能前必须复位 Core；Observer 状态在 Motor Stop/故障/重启生命周期复位。
      不把清理职责放进 `foc_pwm_Stop()`，不添加未存在的 `motor_Reset()` API。
- [ ] `motor_RunStep()` 采样后调用一次 Clarke，再推进 Observer，然后向单一 Core Step
      提交同一份 `Iαβ` 与选定电气反馈。`motor_AlignStep()` 也须在调用 Core 前将 U/V/W
      转为 `Iαβ`，使用固定 ALIGN 角度且不推进 Observer。Core 不重复 Clarke、不增加两阶段
      状态 API。
- [ ] 保持 Encoder 行为及 ADC U/V/W 语义不变；分别回归运行与 ALIGN 路径，证明 Core 输入
      边界变化没有扰动原控制结果。
- [ ] 首期只注册 SMO 实现；不创建空的磁链/EKF 类型，不增加动态插件、算法回调链或外层
      选择器。该指针只用于 Observer 内部的单一算法入口，不形成 `void *` 多层回调链。

### Task 2：重写 Sguan SMO 与配套 PLL 适配

**文件：** `foc/observer/foc_smo.h/.c`、`foc/observer/foc_observer.h/.c`、
必要的 `foc/foc.mk` 条目。

- [ ] 删除旧实现的裸角度差分速度路径，以 Sguan 的模型、滤波和 PLL 相位跟踪递推为基线。
- [ ] 所有 SMO/PLL 状态放入单一 `foc_smo_t` 实例；Init 校验输入并预计算常系数，Reset
      清动态历史而保留参数。
- [ ] 显式实现归一化与机械/电气量转换；在 `foc_observer_t` 统一输出 BAM32 电角度、
      电气圈/秒速度与闭环质量 `valid`。
- [ ] 先做算法级 float 与 Q15 单测，再接入 Motor；不得为测试而更改板级 ADC/PWM 映射。

### Task 3：Host 算法和控制路径测试

**文件：** `foc/tests/foc_core_step_test.c`、
`foc/tests/foc_observer_smo_test.c`、现有 float/fixed 测试目标与脚本。

- [ ] Core 路径测试验证 Clarke 外置前后的 Encoder 输入/输出行为；测试覆盖平衡三相、零
      电流、不同角度及电压/占空比输出一致性。
- [ ] SMO 测试覆盖：初始化/复位、PLL 正反转与加减速跟踪、角度回绕、不同极对数换算、
      启动未锁定、有效判据建立/撤销、弱反电势、系数量化边界及模型符号方向。
- [ ] float 与 Q15 使用同一输入向量和明确的角度/速度误差容限；按 Gate 0 定标验证大增益、
      宽中间乘加、移位舍入和饱和边界，不以“编译通过”代替数值行为验证。

### Gate 1：Shadow、闭环和资源验收

- [ ] Encoder 闭环下观察 SMO 角度/速度，覆盖方向、稳态误差、加减速、负载变化和失锁；
      Observer Shadow 不改变控制输出。
- [ ] 仅在已验证工作区执行显式 Encoder→Observer 接管；检查拒绝不合格请求、失锁故障停
      机以及电流环/速度环稳定性。不把该测试写成无感冷启动通过。
- [ ] 测量当前 Encoder 15–17 μs ISR 基线、增加 Observer 后的总耗时和 SMO 增量；20 kHz
      周期预算为 50 μs，需记录最坏值并留出抖动余量；同时审计高频生产路径实际调用深度，
      不以转发层、宏或未验证 inline 回避项目调用层数限制。
- [ ] 目标构建、Host float/fixed 测试及实机 Shadow/切换结果分别记录。若模型输入、Q15
      范围或 ISR 预算不满足，暂停后续磁链/EKF/Fusion 接入并报告具体瓶颈。

## 5. 后续算法的关系

磁链观测器、EKF 和其它被动估算器不应复制本计划的 Motor/Core/App 接缝，只需按核心
框架实现 Observer 公共契约。每种算法另行明确模型参数、数值范围、有效性定义、Host
测试和 ISR 增量；若输入需求超出 `Iαβ + Vmodel + Ts`，先审核是否值得扩展核心契约。
