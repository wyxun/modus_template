# Encoder 编码器位置反馈

Encoder 把带时间戳的单圈机械角换算为本控制拍的电角度和电速度。输入、输出角度均为 BAM32，`2^32` 个计数分别对应一机械周、一电气周。输出速度采用 PU，等于带方向的机械转速（转/秒）乘极对数，再除以 Motor 的电速度基值（Hz）。算法由库提供，G431 Port 持有 `mfoc_encoder_t` 实例，负责读取 AS5600 和计时。[Encoder 公共接口](../../position/include/mfoc_encoder.h)

## 选择编码器闭环

两个编译选项分别决定是否采集物理位置，以及 Feedback 采用哪一路角速。G431 的物理位置源是编码器。[反馈配置](../../mfoc_config.h)

| 使用方式 | `MFOC_FEEDBACK_ENABLE_POSITION` | `MFOC_FEEDBACK_BACKEND` | 常规闭环采用的角速 |
| --- | --- | --- | --- |
| 编码器闭环 | `1` | `MFOC_FEEDBACK_POSITION` | 编码器的电角度、电速度 |
| 用编码器对照 SMO（当前默认） | `1` | `MFOC_FEEDBACK_SMO` | SMO 的估计值，编码器仍随前台采样 |
| 仅用 SMO | `0` | `MFOC_FEEDBACK_SMO` | SMO 的估计值，G431 裁掉编码器实例和位置服务 |

## 从一次采样到控制拍

1. **初始化。** 应用调用 `mfoc_motor_Init()`。G431 Port 用 Motor 的极对数、电速度基值和板级时钟频率初始化 Encoder。板级配置给出机械方向、初始电零位、测速滤波系数及超时。实例覆盖前台采样与控制中断的整个使用期。[G431 Port](../../../peripheral/stm32g431/mfoc/mfoc_port.c)
2. **前台采样。** `mfoc_motor_Run()` 调用 `mfoc_port_position_run()`。G431 距上次轮询满 1 ms 后读取 AS5600，把机械角和读取完成时的 64 位 tick 交给 `mfoc_encoder_Update()`。Encoder 用相邻样本的最短角差和实际时间差测速，以系数 0.25 滤波，发布机械角、速度和采样时刻。相邻有效读数之间的转动须小于半个机械周，才能由单圈角判明方向。[Encoder 实现](../../position/src/mfoc_encoder.c)
3. **控制中断。** 需要反馈时，`mfoc_feedback_Step()` 调用 `mfoc_port_position(&angle, &speed)`。Port 把当前 tick 传给同一实例的 `mfoc_encoder_Step()`。Step 从已发布的样本按滤波速度外推到当前拍，乘极对数、减电零位，给出电角度，并返回已算好的 PU 速度。选中 POSITION 时，两者直接成为 `mfoc_feedback_output_t.angle` 和 `speed`。例如默认 20 kHz 电流环下，相隔 1 ms 的两次读数之间，每个需要反馈的控制拍都能得到对应时刻的角度。[Feedback 实现](../../motor/src/mfoc_feedback.c)

前台是唯一写者，先写完备用样本槽，再切换索引。单核上的高优先级控制 ISR 读取发布槽，读完后前台才恢复。采样和外推使用同一时钟，实例使用期间 tick 持续累计、频率固定。[Port 契约](../port-contract.md)

## 零位与失效

固定角校准保持到期时，`mfoc_motor_Run()` 在释放磁场前调用 `mfoc_port_position_calibrate_zero(held_angle)`。G431 重新读取机械角，调用 `mfoc_encoder_CalibrateZero()`，使当前转子位置对应已知保持电角度，并把速度历史清零。先确认传感器方向与相序一致。校准读取失败时保留原零位。成功的零位保存在实例 RAM 中，停机再启动沿用该值，重新初始化则恢复配置的初始值，当前 G431 取 0。[Motor 对齐流程](../../motor/src/mfoc_motor.c)

首个样本发布前，Step 返回 `MFOC_RESULT_INVALID_STATE`。采集读取失败时，前台保留上一份样本，Step 仍可在有效期内外推。样本年龄超过 G431 的 20 ms 上限，Step 返回 `MFOC_RESULT_SAFETY`。两类 Step 失败都保留调用方的角速输出。首次采到样本或在超时后恢复采样时，Encoder 从该次实测位置和零速度重新开始测速。[回归用例](../../tests/position/encoder_test.c)

编码器作为常规闭环反馈时，Step 失败会触发 Motor 的反馈故障和停机流程。选择 SMO 时，Feedback 按 SMO 自身状态返回结果。[Feedback 输出选择](../../motor/src/mfoc_feedback.c)
