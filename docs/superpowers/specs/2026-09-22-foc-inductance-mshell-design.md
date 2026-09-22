# FOC 电感辨识 MShell 接入设计

## 目标

在现有 `foc_app` 中开放 `identify inductance` 命令，使其使用固定的
台架参数调用 `identify_StartInductance()`，并在前台流程完成后输出
Phase 1 的 `Ld` 辨识结果。

## 范围

- 仅接入已实现的机械锁轴 `Ld` 辨识；不实现 `Lq`。
- 不增加 Driver、函数指针、通用算法层或命令参数解析器。
- 不改变 Identify、Motor 和 ISR 的现有职责边界。
- 不绕过现有的母线配置、Motor 状态、角度、PWM 饱和、转动和电流保护。

## 命令和配置

命令固定为：

```text
identify inductance
```

命令执行前沿用电阻辨识的准入检查：Motor 必须处于 IDLE、PWM 关闭且已完成
电角度对齐。机械锁轴由台架操作者负责，命令不增加软件确认参数。

测试参数放在 `foc/app/motor_config.h`，由 App 在命令处理时构造局部
`identify_inductance_cfg_t`，Start 内部继续将用户配置换算为 ISR 直接使用的
运行变量。初始参数为：

| 参数 | 初始值 |
|---|---:|
| 注入频率 | 1000 Hz |
| 采样延迟 | 1 ISR 周期 |
| 每半周期采样数 | 4 |
| 半周期数 | 4 |
| 注入调制度 | 0.10 PU |
| 辨识电流上限 | 0.10 PU |
| 最小电流变化 | 0.01 PU |
| 最大电角速度 | 0.01 PU |
| 转动故障周期数 | 1 |

默认 `FOC_DCBUS_SOURCE_NONE` 保持不变。在目标板未配置有效母线来源时，
`identify_StartInductance()` 返回 `FOC_RESULT_DISABLED`，命令打印现有的拒绝日志，
不会启动 PWM。

## 数据流

1. MShell 调用 `foc_app_CmdIdentify("inductance")`。
2. App 检查 Motor 空闲、PWM 关闭和电角度零点有效。
3. App 构造局部配置并调用 `identify_StartInductance()`。
4. 现有 `identify_Run()` 前台 PT 启动 Motor 并收敛结果或错误。
5. 现有高频 ISR 根据 `IDENTIFY_OPERATION_INDUCTANCE` 提交同步快照。
6. 完成后 App 调用 `identify_GetInductance()`，一次性打印结果。

结果输出包含 `Ld`、注入频率、有效电压、平均电流、采样数和半周期数。

## 错误处理

- 未对齐、Motor 非空闲或 PWM 已开启时，命令沿用当前拒绝路径。
- 母线来源未配置、配置非法或 Identify 忙时，直接报告返回码。
- ISR 中的过流、转动、母线无效、PWM 饱和和 Motor fault 继续由现有
  Identify/Motor 路径 safe-stop 并锁存。
- `stop` 和 `reset` 命令行为不变。

## 验证

- 扩展 App 命令主机测试，先证明缺少命令分支时测试失败，再验证命令启动
  `IDENTIFY_OPERATION_INDUCTANCE`，且准备后的参数与宏一致。
- 验证未对齐时命令不启动。
- 运行 FLOAT/FIXED 电感辨识测试、电阻辨识测试和 App 命令测试。
- 执行 `git diff --check` 和嵌入式 C 82 字符检查。
