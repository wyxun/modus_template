# R/L 无编码器辨识实现与验证

## 已确认的设计

R/L 各自保留独立的采样类型，改为固定 α 轴电压、αβ 电流 PU。Motor 拥有固定矢量对齐和注入生命周期；对齐完成后保持同一 PWM 会话，切换到固定零角度电压注入，App 不调用 Position/Observer。Core 在零角度下的 d/q 与 αβ 重合，仅在 Motor/Core 接口做一次明确映射。

Identify 的 R/L 前置流程共用有超时的前台 PT；启动时自动对齐，不需要编码器零位有效。此准备方式只用于 R/L，不强制后续磁链等旋转辨识使用静止注入。统一 PU 结果出口及前台物理单位转换接口保持原语义。

电压模型由上一份提交占空比与本拍母线 PU 重构到 αβ，禁止把固定 12 V 当作实测母线。Ld 原有编码器转速门槛改为连续横轴电流扰动门槛；它可以发现横轴扰动，不能证明转子静止。Ld 标为沿对齐轴的增量电感，转子需在采样期间保持静止；凸极电机保持对齐尤其重要。模型仍不包含死区和器件压降。

## 改动前后

| 边界 / 配置 | 改动前 | 改动后 | 原因 |
|---|---|---|---|
| Rs/Ld 启动 | 要求电角零位；依赖位置反馈 | 自动固定矢量对齐，再进入 Motor 固定轴会话 | 静止注入不需要编码器；明确运行模式而不是删掉反馈校验 |
| Rs/Ld 输入 | dq 电流、D 电压；Ld 含速度 | 各自独立的 αβ 电流、α 电压、有效/饱和标志 | 删除角度和假零转速，保持后续算法接口独立 |
| 电压模型 | 上一占空比 + 实测母线，再 Park | 同样的实测母线，直接重构 αβ | 保留真实幅值，取消角度和三角运算 |
| 结果 | 内部 PU、统一带标签输出 | 不变，显示时才转换物理值 | 无感模型消费者继续使用 PU |
| `IDENTIFY_ALIGNMENT_TIMEOUT_MS` | 无辨识对齐等待 | `5000U` | 对齐未完成、PWM 停止或 Motor 故障有明确收敛出口 |
| Ld 运动门槛 | `MAX_SPEED_PU=0.005`、`MOTION_CYCLES=1` | `MAX_TRANSVERSE_CURRENT_PU=0.025`、`TRANSVERSE_CYCLES=1` | 无编码器无法实测转速；改为横轴扰动诊断，不能证明静止 |
| Ld `CAPTURE_DELAY` | `1U` | `2U` | k 提交参考，k+1 更新 PWM，k+2 才取得相应区间样本 |
| Encoder 前台服务 | 读取失败可阻断辨识推进 | Rs/Ld 活动期间跳过 | 去除隐藏的前台编码器依赖 |

`0.025 PU` 在 3.5 A 基值下为 87.5 mA，在 119 code/base 标度下约 3 code；阈值仅作为初调配置，需用实机静止电流噪声验证。其余电流、电压基值、PI、Rs/Ld 激励幅值均未因本次去角度依赖而调整。

## 执行计划

- [x] 增加真实 Motor 无反馈对齐/注入、App 跳过位置源、αβ 电压模型及 R/L 回归测试；先观察失败。
- [x] 增加 Motor 固定注入会话和独立 ISR phase；Stop/Fault 清除会话，普通运行继续拒绝无效反馈。
- [x] R/L 独立输入改为 αβ PU，共用前台对齐准备；移除电角零位门槛，保留同 ISR 中止与请求提交。
- [x] 更新横轴扰动配置、接口说明和方法边界文档；新算法继续增加自身类型与调度分支。
- [x] 运行 FLOAT/FIXED、LOG=0/1 的 R/L 数值和异常回归、Motor/App 边界测试、G431 debug-rel/release 构建。

## 验证结果

- `run_motor_alpha_beta_test.ps1`：FLOAT/FIXED 通过。真实 Motor 实现覆盖无零位对齐、无反馈注入、普通电流/电压参考设置不能覆盖辨识会话、Stop/Fault 清理和普通运行无效反馈拒绝。
- `run_motor_break_fault_test.ps1`、`run_motor_reference_limit_test.ps1`、`run_motor_speed_pu_test.ps1`、`run_motor_position_boundary_test.ps1`、`run_sensorless_startup_test.ps1`、`run_sensorless_position_handoff_test.ps1`：FLOAT/FIXED 通过，包含固定轴 ALIGN 的 break 中止。
- Rs/Ld 独立回归：FLOAT/FIXED × LOG=0/1 通过。R 覆盖实测母线下降的数值；L 的独立物理 RL 瞬态在 10.8 V 和 15 V 下均得到 855–865 µH（真值 860 µH）。
- L 异常回归覆盖对齐前不采集、4999 ms 继续等待/5000 ms 停止、对齐故障、Stop/Reset 后重启、正负横轴扰动及计数复位、日志只在前台输出一次。超时 fake 使用与真实 perf_counter 一致的 tick 截止槽语义。
- `run_encoder_command_test.ps1`：FLOAT/FIXED 通过；无电角零位启动 Rs/Ld、编码器读失败仍推进前台、固定轴 ISR 无编码器读取、原 6 通道波形组合与超上限拒绝通过。
- `run_identify_pu_boundary_test.ps1`、`run_foc_module_boundary_test.ps1`：通过；独立 αβ PU 输入、固定轴控制跳过 Position/Observer、正常控制调用顺序守卫保留。
- `.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel`、`BUILD=release`：通过，无 warning/error。release 辨识目标文件不包含 `util_debug_Printf` 依赖和辨识诊断文本；最后恢复 debug-rel 构建产物。

横轴扰动计数尚未达到阈值时，采样时钟照常推进；不能暂停相位计数后仍用固定 `Δt_pu` 计算电感。

## 验收与限制

主机应覆盖无编码器启动、对齐前不采集、对齐超时停 PWM、正常运行仍要求真实有效反馈、当前采样配对守卫、实测母线变化、横轴扰动与过流/饱和中止。未进行硬件启动、注入或辨识验证；不得据编译/主机测试声称转子已静止或辨识已符合铭牌。
