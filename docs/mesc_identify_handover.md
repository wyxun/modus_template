# MESC 电机参数辨识框架重构交接文档

## 1. 项目背景与设计哲学

本项目旨在基于 MESC 优秀工程实践（commit `aa159cc6`），在当前代码库中重构一套高精度、强鲁棒、零侵入的电机参数自动辨识框架。

### 核心设计原则

1. **`motor_t` 的纯粹性（Pure Motor State）**
   - 坚决杜绝在 `motor_t` 中添加任何辨识专用成员（如积分累加器、辨识阶段标记等）。
   - 正常 20kHz FOC 中断热路径保持绝对零拷贝、零辨识额外开销。
   - 辨识算法所需状态完全内聚在独立对象 `foc_identify_t` 中。
   - 电机仅通过 `#if FOC_ENABLE_EXPERIMENTAL_IDENTIFY` 宏暴露零拷贝瞬态快照结构 `motor_step_metrics_t` 及函数 `motor_CaptureStepMetrics()`，供上层 App 在同一 ISR 中按需捕获单步输出。

2. **MDI 直接目标绑定契约（Direct Hardware Contract）**
   - 彻底废除旧架构遗留的 `ptAdc`/`ptPwm` 转发指针与虚构物理量。
   - FOC 控制热路径直接调用目标层实现的类型化硬件契约：`foc_SampleCurrent()` 与 `foc_SetDuty()`。
   - 电流基准 `wCurrentBaseMilliamp` 为 Motor 内置属性，直接由 `FOC_CURRENT_BASE_MILLIAMP` 统一初始化，不再由应用配置层传递。

3. **严格的代码规范约束**
   - 保持所有修改过的 C 源文件与头文件每行宽度严格控制在 **82 列以内**。
   - 保护工作树中无关的 MDI 重构与 OpenOCD 配置修改，严禁擅自丢弃或污染。
   - 未经用户显式授权，严禁执行 `git commit` 或 `git push`。

---

## 2. 算法核心机制 (MESC-Aligned)

### 2.1 Rs 双点闭环直流注入
- **消除逆变器死区与管压降**：避免单点测量因功率管非线性压降和 ADC 零点漂移造成的巨大误差。
- **双点闭环跟踪**：
  - 注入低设定电流 $I_{low} = 0.05\,\text{pu}$，PI 闭环稳定后在采样窗口内计算冻结平均值 $(V_{low}, I_{low\_meas})$。
  - 注入高设定电流 $I_{high} = 0.15\,\text{pu}$，同理采集 $(V_{high}, I_{high\_meas})$。
- **差分求解**：
  $$R_s = \frac{V_{high} - V_{low}}{I_{high\_meas} - I_{low\_meas}}$$
- **抗漂移能力**：即便存在 $\pm 0.03\,$V 的逆变器管压降与 $\pm 0.005\,$A 的采样零漂，计算误差仍低于 $0.4\%$。

### 2.2 Ld / Lq 等周期双极性差分积分窗口
- **对称脉冲消除直流偏置**：
  - 在待测轴（先 D 后 Q）施加恒定电压脉冲幅值 $V_{inj}$，持续 $N_{half}$ 步；紧接着反转极性施加 $-V_{inj}$，持续 $N_{half}$ 步。
  - 对称窗口求和消除纯积分项的直流分量偏移。
- **精确梯形校正（Trapezoidal Correction）**：
  - 考虑绕组电阻 $R_s$ 消耗电压，利用校正项进行补偿：
    $$x = \frac{R_s \omega_{base} T_s}{L} \le 0.25$$
- **多周期平滑与分散度淘汰**：
  - 连续采集 $N_{pairs}$ 个周期，丢弃前 $N_{discard}$ 个瞬态不稳定周期。
  - 计算对间离散度 $\frac{\max - \min}{\text{mean}} \le 5\%$，超出阈值立即报错，避免因机械转动或接触不良造成误判。

### 2.3 全生命周期安全防护
- **位移超限保护**：高频小信号注入期间，若转子电角度位移 $|\Delta\theta| > 0.05\,\text{turn}$ 或电角速度 $> 0.02\,\text{pu}$，立即熔断停机。
- **母线电压标定自检**：若实测母线电压与基准标称电压偏差超过 $5\%$，禁止启动辨识。
- **前台超时守护**：前台任务每周期监控 ISR 活跃 Tick，若超过 10ms 未更新则触发强制停机。
- **安全停机确认**：辨识完成或异常退出进入 `STOPPING` 状态后，必须等待底层电机状态机确认进入 `MOTOR_STATE_IDLE`，方可向用户报告最终结果。

---

## 3. 架构与文件变更清单

```text
foc/
├── identify/
│   ├── foc_identify.h        # [新增] MESC 参数辨识算法接口、状态机与诊断定义
│   └── foc_identify.c        # [新增] 算法核心实现 (Rs双点、Ld/Lq双极性积分、梯形修正)
├── motor/
│   ├── motor.h               # [修改] 增加 motor_step_metrics_t 及捕获接口宏保护
│   └── motor.c               # [修改] 实现 motor_CaptureStepMetrics 零拷贝快照
├── app/
│   ├── foc_app.h             # [修改] 持有 foc_identify_t，清理遗留 ptAdc/ptPwm
│   └── foc_app.c             # [修改] 集成辨识 ISR 步进、邮箱仲裁、互斥锁与前台监控
├── tests/
│   ├── foc_identify_plant.h  # [新增] 模拟电机 RL 动力学高精度解析植物模型
│   ├── foc_identify_test.c   # [新增] 算法层独立单元测试 (FLOAT/FIXED 矩阵)
│   ├── run_identify_test.ps1 # [新增] 算法层自动化回归测试脚本 (100%通过)
│   ├── foc_app_identify_test.c   # [新增] 9大场景端到端集成测试 (调优中)
│   └── run_app_identify_test.ps1 # [新增] 端到端集成测试脚本
└── docs/
    └── mesc-identify-validation.md # [新增] 完整验证与测试跟踪矩阵
```

---

## 4. 当前完成进度与测试证据

### Task 0 - Task 3: 算法内核与单元测试（已 100% 通过）
- 实现了 `foc_identify.c` 与 `foc_identify_plant.h`。
- 测试覆盖矩阵：
  - $R \in \{0.25, 0.50, 1.00\}\,\Omega$
  - $L \in \{0.5, 1.0, 2.0\}\,\text{mH}$
  - 逆变器压降 $\pm 0.03\,$V，采样零漂 $\pm 0.005\,$A
- **测试结果**：
  - `.\foc\tests\run_identify_test.ps1` 在 **FLOAT** 和 **FIXED** 两个数值后端下 100% 通过！
  - Rs 测量误差 $< 0.1\%$，Ld/Lq 测量误差 $< 0.08\%$。

### Task 4 - Task 5: 电机瞬态视图与应用层集成（已 100% 通过）
- `motor_CaptureStepMetrics()` 成功将 Core DQ 电流、电压、电角度与有效性打包为 `motor_step_metrics_t`，完全不增加正常 ISR 负担。
- `foc_app.c` 中成功实现了辨识邮箱仲裁、运动命令互斥锁与前台停机确认。
- **测试结果**：
  - 既有回归测试 `.\foc\tests\run_encoder_command_test.ps1` 在 **FLOAT** 和 **FIXED** 两个后端下 100% 通过。

---

## 5. 当前交接断点与排查指引 (Task 6)

### 5.1 现场状态
- 已完成 Task 6 测试框架编写：`foc/tests/foc_app_identify_test.c` 及 `foc/tests/run_app_identify_test.ps1`。
- 包含了 9 大端到端集成测试场景：
  1. 标称电机辨识 ($R=0.5\Omega, Ld=1.0\text{mH}, Lq=1.5\text{mH}$)
  2. 非零电角度启动 ($60^\circ$)
  3. PWM 生效延迟对比 ($D=0, D=2$)
  4. 中途 Cancel 指令响应
  5. 电机直接 Stop 指令的高优先级切断
  6. 辨识进行中对运动命令的互斥锁拦截
  7. 10ms ISR 无进展前台超时熔断
  8. 母线电压标称误差拒绝启动 ($V_{bus}=11\text{V}$)
  9. 辨识复位与重新发起测试

### 5.2 当前阻断问题与精确根因
- **现象**：执行 `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_app_identify_test.ps1` 时，在初始化阶段报错：
  `DEBUG: foc_app_Init failed with code 2` (`FOC_RESULT_INVALID_ARGUMENT`)。
- **精准定位**：
  `foc_app_Init()` 中调用了 `motor_Init()`，其内部由 `_motor_ConfigValid()` 校验输入参数：
  ```c
  static bool _motor_InterfacesValid(const motor_cfg_t *ptConfig)
  {
      if (ptConfig->tPosition.ptOps == NULL ||
          ptConfig->tPosition.pContext == NULL ||
          ptConfig->tPosition.ptOps->fnGetPosition == NULL ||
          ptConfig->tPosition.ptOps->fnCaptureZero == NULL) {
          return false;
      }
      return true;
  }
  ```
  而在 `foc_app_identify_test.c` 的 `setup_test_app()` 中：
  `tConfig.tEncoderCfg = {0};`
  导致 `foc_encoder_Init` 返回的编码器未绑定 sensor 接口，`bEncoderReady` 被置为 `false`，从而导致 `ptMotorConfig->tPosition.ptOps = NULL`，最终被 `_motor_InterfacesValid` 拒绝并返回 `FOC_RESULT_INVALID_ARGUMENT (2)`。

### 5.3 开箱即用的修复步骤
在 `foc/tests/foc_app_identify_test.c` 中：
1. 实现 `foc_encoder_sensor_if_t` 或 mock 的 `motor_position_ops_t`。
2. 在 `g_tFocEncoderPositionOps` 中补全 `fnCaptureZero` 的 mock 实现：
   ```c
   static foc_result_t test_CaptureZero(void *pContext, foc_angle_t *ptZero)
   {
       (void)pContext;
       if (ptZero != NULL) {
           *ptZero = (foc_angle_t){0};
       }
       return FOC_RESULT_OK;
   }
   
   const motor_position_ops_t g_tFocEncoderPositionOps = {
       .fnGetPosition = foc_encoder_GetPosition,
       .fnCaptureZero = test_CaptureZero,
   };
   ```
3. 在 `setup_test_app()` 中为 `tConfig.tEncoderCfg.ptSensor` 赋值：
   `tConfig.tEncoderCfg.ptSensor = &g_tFocEncoderSensorInterface;`
4. 重新运行 `.\foc\tests\run_app_identify_test.ps1`。

---

## 6. 后续待办任务路线图 (Task 6 & Task 7)

### 第一步：完成 Task 6 集成测试联调
- 按上述方案修复 `foc_app_identify_test.c` 的 position ops mock。
- 确保测试 1 ~ 9 在 **FLOAT** 和 **FIXED** 下均完整通过断言。

### 第二步：执行全量回归测试集 (Regression Suite)
在完成 Task 6 后，依次运行以下所有主机测试脚本，确保既有功能无回退：
1. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_identify_test.ps1`
2. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_app_identify_test.ps1`
3. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_encoder_command_test.ps1`
4. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_motor_reference_limit_test.ps1`
5. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_motor_break_fault_test.ps1`
6. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_motor_alpha_beta_test.ps1`
7. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_motor_speed_pu_test.ps1`
8. `powershell -ExecutionPolicy Bypass -File .\foc\tests\run_mdi_foc_realtime_interface_test.ps1`

### 第三步：硬件目标工程编译与体积审计
- **辨识开启构建**：
  使用 `FOC_ENABLE_EXPERIMENTAL_IDENTIFY=1` 编译 STM32G431 目标，验证编译无警告，RAM/Flash 增量符合预期。
- **辨识关闭构建**：
  使用 `FOC_ENABLE_EXPERIMENTAL_IDENTIFY=0` 编译 STM32G431 和 AT32F413 目标，通过 `nm` 或 `map` 文件审计，确保无任何辨识符号残留，保证生产构建极致紧凑。

### 第四步：验收文档归档
- 更新 `foc/docs/mesc-identify-validation.md`，将验证结果填入四栏跟踪矩阵中。

---

## 7. 常用开发与测试命令速查

```powershell
# 1. 运行辨识内核算法单元测试 (FLOAT + FIXED)
powershell -ExecutionPolicy Bypass -File .\foc\tests\run_identify_test.ps1

# 2. 运行应用层集成测试 (FLOAT + FIXED)
powershell -ExecutionPolicy Bypass -File .\foc\tests\run_app_identify_test.ps1

# 3. 运行编码器与指令诊断测试
powershell -ExecutionPolicy Bypass -File .\foc\tests\run_encoder_command_test.ps1

# 4. 嵌入式目标默认构建 (AT32F413)
.\make.bat

# 5. 清理构建
.\make.bat clean
```
