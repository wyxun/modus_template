# FOC 参数受控台架辨识敏捷实施计划 (KISS 版)

**目标：** 在 STM32G431 上实现极简受控参数辨识，直接测量并输出 $R_s$、$L_d$、$L_q$ 的 PU 结果。  
**技术栈：** Embedded C、STM32G431 20 kHz ADC ISR、PowerShell 单测。  
**关联规范：** [2026-09-15-foc-parameter-identification-minimal-design.md](../specs/2026-09-15-foc-parameter-identification-minimal-design.md)

---

## 任务拆分与执行顺序

### Task 1: 编写极简 `foc_identify` Controller 与单测 (~120 行)
**目标文件：**
- 新增：`foc/identify/foc_identify.h`
- 新增：`foc/identify/foc_identify.c`
- 修改：`foc/tests/foc_identify_test.c`
- 脚本：`foc/tests/run_identify_test.ps1`

**执行细节：**
1. 声明极简状态（`RS_LOW`、`RS_HIGH`、`LD`、`LQ`、`COMPLETE`、`ERROR`）；
2. 实现 `foc_identify_Init`、`Start`、`Step`、`Abort`、`GetResult`、`ConsumeTerminal`；
3. 电阻阶段：等待 100 拍 $\to$ 累加 100 拍，两电平差分算 $R_s$；
4. 电感阶段：脉冲 30 拍累加 $\sum (V - R_s I)$ 除以 $\Delta I$，算 $L_d$ 与 $L_q$；
5. 首拍自动锁存机械角度，超限或故障立即报错；
6. 运行单测验证已知 $R/L$ 合成输入，确保数值正确。

---

### Task 2: 填充 `motor_config.h` 静态部署常量 (~25 行)
**目标文件：**
- 修改：`foc/app/motor_config.h`

**执行细节：**
1. 填入极对数（7）、基准电压（12000 mV）、基准电流（7000 mA）、基准频率（100 Hz）；
2. 定义编译期算好的 PU 常量（`IDENTIFY_V_LOW_PU`、`IDENTIFY_V_HIGH_PU`、`IDENTIFY_V_LD_PU`、`IDENTIFY_V_LQ_PU` 等）；
3. 消除 `foc_app.c` 底部的魔数。

---

### Task 3: `foc_app` 接入与 G431 硬件安全闭环 (~50 行)
**目标文件：**
- 修改：`foc/app/foc_app.h`
- 修改：`foc/app/foc_app.c`
- 修改：`peripheral/stm32g431/foc_port.c`

**执行细节：**
1. `foc_app.h` 瘦身：删掉多余的 8 个变量，只保留 `tIdentify` 和 `chIdentifyCommand`；
2. 20 kHz ISR 接入：
   ```c
   /* 1. 组装输入快照 */
   foc_identify_input_t in = {
       .tCurrentPu = tInput.tCurrentAlphaBeta,
       .tVmodelPu = s_tLastVmodel,
       .tMechanicalAngle = ptThis->tEncoder.tAngle,
       .bValid = bValid,
       .bFault = motor_GetStatus(&ptThis->tMotor) == MOTOR_STATE_FAULT
   };
   /* 2. 单步推进 Controller */
   foc_identify_output_t out;
   foc_identify_Step(&ptThis->tIdentify, &in, &out);
   /* 3. 下发动作 */
   if (out.bRefChanged) motor_SetVoltageReference(&ptThis->tMotor, out.tVoltageRefPu);
   if (out.bStopPwm) motor_Stop(&ptThis->tMotor);
   ```
3. 在 G431 `foc_port.c` 中补齐 TIM1 BIF 故障查询；
4. Shell `motor identify` 支持启动并在完成后双单位同屏打印（PU 与 $\Omega / \mu\text{H}$）。

---

## 验收条件
1. `foc_identify` 单测 100% 通过；
2. `foc_app.h` 无任何冗余胶水变量；
3. G431 台架启动辨识后，在 ~25 ms 内平稳完成测试并安全停止 PWM，打印物理量与标幺值。
