# FOC 参数识别诊断快照设计

## 目标

为现场参数识别增加低侵入诊断数据，解释 Ld/Lq 识别结果漂移和
`FOC_RESULT_SAFETY` 失败原因。第一步不改变识别公式、脉冲长度或安全阈值。

## 设计

- `foc_identify_t` 持有 RS、Ld、Lq 三个阶段的诊断快照，以及失败时的阶段。
- 快照保存电流起点、终点、`DeltaI`、累计 `V-RI`、电阻估计值、结果值和
  采样周期数。
- 高频 ISR 只保存已经计算出的标量和状态，不执行日志、拟合、离群值判断或
  额外循环。
- 增加 `foc_identify_GetDiagnostics()`，Shell 通过接口读取快照，避免直接依赖
  识别对象内部成员。
- `motor identify status` 在 COMPLETE 或 ERROR 状态下打印 RS、Ld、Lq
  快照和失败阶段；打印完成后继续沿用现有终态消费行为。

## 验证

- 主机 FLOAT/FIXED 识别测试验证成功流程能得到三阶段诊断快照。
- 主机测试验证安全失败能保留失败阶段和末次诊断数据。
- 运行识别测试脚本，并构建 STM32G431 debug-rel 固件。
