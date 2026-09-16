# FOC 离线参数识别鲁棒算法实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**目标：** 在不改变现有识别公开接口的前提下，使 Rs、Ld、Lq 识别能够识别低信噪比、机械微动和 q 轴转矩污染，并在数据不可信时明确失败。

**架构：** 保留 `foc_identify_Init/Start/Step/GetResult` 和输入输出结构的现有调用方式，只扩展 `foc_identify_t` 的内部状态。识别过程分为 Rs 多次激励、Ld 正向 RL 拟合、Lq 正向 RL 拟合加反向转矩抵消；高频周期内只做固定数量的标量累加和状态切换，统计、排序和最终一致性判定放在阶段边界执行。

**技术栈：** C11、FOC float/fixed 双数值后端、STM32G431、PowerShell 主机测试、AITrace 被动 RTT 验证。

---

## 现有证据和设计约束

- 最近三次 Rs 为 `1.1577、1.1482、1.5051 PU`，而 `deltaI` 只有
  `0.0266~0.0348 PU`，说明两电压点的电流差太小，不能把单次结果当作可靠 Rs。
- Ld 有时 `sumV<0`，Lq 多次 `sumV<0`。这不是“电感为负”，而是
  `V-R×(I-Istart)` 与 RL 上升响应不一致，应拒绝该次结果。
- 读取 `tMotor.tCore.tVoltage` 后，Ld 曾出现 `sumV=+0.4452`，说明实际电压链路已修正；后续算法必须继续使用该值，不能退回应用层参考缓存。
- q 轴直流电压会产生电磁转矩；没有刚性夹具时，不允许把 Lq 结果标记为 COMPLETE。
- 不修改 `foc_identify_input_t`、`foc_identify_output_t` 和公开函数签名；不把 PU 识别结果自动写回 SMO 参数。

## Task 1：建立带噪声和电压延迟的失败测试

**文件：**

- 修改：`foc/tests/foc_identify_test.c`
- 修改：`foc/tests/run_identify_test.ps1`

- [ ] **Step 1：保留现有理想 RL 流程测试，并增加电流零偏测试。**

  在现有 `fCurrentOffset=0.030f` 流程中继续验证 float/fixed 均能得到
  正的 `qSumV`、正的 `qDeltaI`，并确认 Ld/Lq 结果在各自后端容差内。

- [ ] **Step 2：增加电压记录为零的失败测试。**

  在 LD 阶段让电流按 RL 模型上升，但连续 64 个采样把
  `tLastVoltageCommandDqPu.qD` 置零；断言返回 `FOC_RESULT_SAFETY`、
  `eFailureStage == FOC_IDENTIFY_STATUS_LD`，且 `tLd.bValid == false`。

- [ ] **Step 3：增加 q 轴反向抵消状态测试。**

  测试正向 q 脉冲结束后输出 `-qV_Lq`，反向脉冲期间不增加拟合样本，
  反向脉冲结束后才输出 COMPLETE 和零电压。

- [ ] **Step 4：增加重复结果离群测试。**

  给三次模拟 Ld 结果 `0.08、0.081、0.30 PU`，断言离群值不进入最终结果；
  给三次结果 `0.08、0.12、0.16 PU`，断言一致性判定失败。

- [ ] **Step 5：运行测试确认新增测试先失败。**

  ```powershell
  .\foc\tests\run_identify_test.ps1
  ```

  预期：现有流程继续通过，新增反向脉冲和重复一致性测试在算法尚未实现
  时失败。

## Task 2：把 Rs 识别改为重复双点测量和鲁棒汇总

**文件：**

- 修改：`foc/identify/foc_identify.h`
- 修改：`foc/identify/foc_identify.c`
- 修改：`foc/app/motor_config.h`

- [ ] **Step 1：增加内部 Rs trial 状态，不改变公开配置结构。**

  在 `foc_identify_t` 增加三个 Rs trial 结果、trial 计数和有效计数；
  `foc_identify_cfg_t` 不增加字段，避免破坏已有初始化代码。

  ```c
  foc_scalar_t qRsTrial[3];
  uint8_t chRsTrial;
  uint8_t chRsValid;
  ```

  每次 RS_HIGH 完成后先验证 `deltaV>0`、`deltaI>qMinDeltaI` 和有限性，
  只有有效结果才写入 trial 数组。

- [ ] **Step 2：把 RS 稳态窗口扩大并保持 ISR 固定工作量。**

  将低、高电压的平均窗口从 100 个采样扩大到 200 个采样，保持每周期只
  做一次标量累加；settle 窗口不变。使用当前 `0.010/0.050 PU` 配置，
  不先盲目把电压提高到可能导致过流的值。

- [ ] **Step 3：连续执行三次双点测量。**

  第一次 RS_HIGH 得到有效 Rs 后，如果 `chRsValid<3`，清空 RS 窗口，
  重新进入 RS_LOW；第三次完成后才进入 ZERO/LD。每次 trial 之间保持
  现有零电压沉淀，防止残余电流污染下一次低电压采样。

- [ ] **Step 4：用三点中位数作为 Rs。**

  对三个有效结果使用无循环的三值比较选择中间值，避免在高频路径增加
  排序循环。有效结果少于两次时返回 `FOC_RESULT_SAFETY`；三次结果的
  最大最小差超过中位数的 20% 时返回安全失败。

- [ ] **Step 5：运行 float/fixed 识别测试。**

  ```powershell
  .\foc\tests\run_identify_test.ps1
  ```

  预期：理想模型的 Rs、Ld、Lq 均通过；加入 Rs 离群值时最终结果等于
  中位数，加入高离散数据时状态为 ERROR。

## Task 3：保留 RL 斜率拟合并修正 Ld/Lq 采样窗口

**文件：**

- 修改：`foc/identify/foc_identify.c`
- 修改：`foc/identify/foc_identify.h`
- 测试：`foc/tests/foc_identify_test.c`

- [ ] **Step 1：固定每 4 个原始采样形成一个块。**

  块内只累加电流和实际 D/Q 电压；块满后计算块平均值，并使用相邻块
  电流差形成 `dI` 样本。拟合公式保持：

  ```text
  dI[k] = slope * Iavg[k] + noise
  L = Rs * (4 * Ts) / abs(slope)
  ```

  `4` 必须使用整数计数辅助函数，不能写成 `FOC_SCALAR(4)`；float 和
  fixed 两个后端都要经过相同的单位换算。

- [ ] **Step 2：使用上一拍已提交的电压作为识别输入。**

   由于 Motor API 的命令提交和 ADC 采样不是同一时刻，在
   `foc/app/foc_app.c` 中使用应用层缓存：

   ```c
   tInput.tLastVoltageCommandDqPu = ptApp->tLastVoltageCommandDqPu;
   ```

   该值对应当前 ADC 样本实际经历的上一拍 Motor 电压命令，避免参考值
   更新与电流采样错位。

- [ ] **Step 3：保留零偏修正和 RL 方向检查。**

   `sumV` 累计实际施加的脉冲电压，仅用于正值和有限值检查；若拟合斜率
   非负、有效块少于 8 个或当前增量低于 `qMinDeltaI`，返回
   `FOC_RESULT_SAFETY`，诊断快照保留失败原因。

- [ ] **Step 4：检查定点后端的所有整数缩放。**

  对块平均、块时长乘法、拟合分子分母分别加入 float/fixed 断言；禁止把
  Q15 数值和普通整数直接混用，禁止在定点分支中产生隐式 Q15 缩放。

- [ ] **Step 5：运行识别测试并检查 ISR 侵入。**

  ```powershell
  .\foc\tests\run_identify_test.ps1
  rg -n "printf|MLOG|for|while" foc\identify\foc_identify.c
  ```

  拟合函数只在脉冲结束时执行，ISR 中不得出现日志、排序、浮点转换或
  变长循环。

## Task 4：为 Lq 增加反向转矩抵消阶段

**文件：**

- 修改：`foc/identify/foc_identify.h`
- 修改：`foc/identify/foc_identify.c`
- 修改：`foc/app/foc_app.c`
- 修改：`foc/docs/foc-encoder-pu-identify-smo-validation.md`

- [ ] **Step 1：增加内部 return-pulse 标志。**

  在 LQ 正向 64 个采样完成且拟合成功后，不立即 COMPLETE；设置内部
  `bReturnPulse=true`，清零 tick，输出 `qQ=-qV_Lq`。状态仍保持
  `FOC_IDENTIFY_STATUS_LQ`，所以不改变 Shell 和公开枚举接口。

- [ ] **Step 2：反向脉冲期间只计时，不参与拟合。**

  反向 64 个采样期间忽略拟合累加器，仅检查过流、编码器有效和机械位移；
  反向脉冲完成后输出零电压并设置 COMPLETE。若机械位移超过
  `qMaxDisplacement`，立即 ERROR 并停止 PWM。

- [ ] **Step 3：补充 Lq 安全说明。**

  文档明确写出：反向脉冲只能降低平均转矩，不能替代刚性机械夹具；没有
  夹具时 Lq 识别仍应视为无效。

- [ ] **Step 4：运行反向阶段测试。**

  ```powershell
  .\foc\tests\run_identify_test.ps1
  ```

  预期：正向拟合结果保持不变，反向脉冲后 PWM 停止；反向阶段出现过流或
  位移时状态为 ERROR。

## Task 5：重复识别结果的一致性门控

**文件：**

- 修改：`foc/identify/foc_identify.h`
- 修改：`foc/identify/foc_identify.c`
- 修改：`foc/app/foc_app.c`

- [ ] **Step 1：为 Ld/Lq 保存最近三次有效结果。**

  每个轴完成一次正向拟合后保存结果；只有三次都完成且结果范围不超过
  中位数的 20% 才将该轴结果写入 `tResult`。重复过程中保持 PWM 零电压
  和已有阶段状态，不改变外部命令接口。

- [ ] **Step 2：失败时输出可定位诊断。**

  将 trial 数、有效数、最小值、最大值、中位数和失败阶段加入已有诊断快照；
  Shell 只在前台打印这些字段，高频 ISR 只保存标量。

- [ ] **Step 3：避免“为了完成而放宽安全门限”。**

  `sumV<=0`、非正斜率、机械位移、过流和有效样本不足继续返回 ERROR；
  只对同一物理条件下的重复有效样本做鲁棒汇总。

## Task 6：完整验证和上板步骤

**文件：**

- 检查：`foc/identify/foc_identify.c`
- 检查：`foc/app/foc_app.c`
- 检查：`foc/app/motor_config.h`
- 检查：`foc/docs/foc-encoder-pu-identify-smo-validation.md`

- [ ] **Step 1：运行全部 FOC 主机测试。**

  ```powershell
  $ErrorActionPreference = 'Stop'
  $scripts = Get-ChildItem foc\tests\run_*.ps1
  foreach ($script in $scripts) { & $script.FullName }
  ```

  预期：所有脚本退出码为 0，并输出 `ALL_FOC_HOST_TESTS_PASSED`。

- [ ] **Step 2：构建带识别功能的 STM32G431 固件。**

  ```powershell
  .\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=float FOC_ENABLE_SMO=0 FOC_ENABLE_HFI=0 FOC_EXPERIMENTAL_IDENTIFY=1 SW_ROOT=D:/software
  ```

  构建输出必须包含 `-DFOC_ENABLE_EXPERIMENTAL_IDENTIFY=1`，且无新增编译
  警告。

- [ ] **Step 3：烧录并进行固定轴测试。**

  ```powershell
  .\make.bat flash TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=float FOC_ENABLE_SMO=0 FOC_ENABLE_HFI=0 FOC_EXPERIMENTAL_IDENTIFY=1 SW_ROOT=D:/software
  ```

  机械上刚性固定转子，等待 `motor align` 完成并确认 `motor status` 为
  `pwm=0` 后执行三次 `motor identify start`。每次记录 Rs/Ld/Lq、各阶段
  `deltaI`、`sumV`、位移和失败阶段。

- [ ] **Step 4：使用 AITrace 做被动验收。**

  ```powershell
  .\tools\aitrace.exe shell --raw "motor identify status"
  .\tools\aitrace.exe shell --raw "motor encoder"
  .\tools\aitrace.exe wave stat 3
  ```

  验收条件：三次有效结果离散不超过 20%，Ld/Lq 的 `sumV` 为正，编码器
  位移不超过配置门限，HF ISR 没有超时或故障。

- [ ] **Step 5：只有通过门控后才评估 SMO。**

  将稳定的物理单位 Rs/Ld/Lq 写入 Motor 配置后，再以
  `FOC_ENABLE_SMO=1` 单独验证 SMO；当前识别失败期间不修改 SMO 参数。

## 计划自检

- 公开识别接口未改变；新增内容只在对象内部状态和诊断快照中。
- 所有算法变化都有 float/fixed 主机测试和失败路径测试。
- q 轴机械锁定仍是硬件前提，软件反向脉冲不被当作替代夹具。
- 没有把负 `sumV`、离群结果或未完成的 Lq 结果强行标记为 COMPLETE。
- 没有要求提交、推送或覆盖用户现有工作树修改。
