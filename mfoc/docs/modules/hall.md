# Hall 有感位置组件

`mfoc_hall` 接收三路数字霍尔码及其捕获时刻，输出控制拍的电角度 BAM32 和电速度 PU。板级持有实例、读取 GPIO、捕获边沿，并提供这台电机的相序和电角度标定表。[公共头文件](../../position/include/mfoc_hall.h)

## 输入与输出

| 操作 | 板级需要提供的量 | 库的处理 |
| --- | --- | --- |
| `mfoc_hall_Init` | 六个扇区的霍尔码、各扇区正向入口电角度、电频率基值、边沿超时及 tick 频率 | 验证标定表并复制入实例 |
| `mfoc_hall_OnEdge` | 初次读取的电平及其时刻，此后每个变化后的三位码及实际捕获 tick | 确认进入的扇区和方向，用刚刚完整走过的扇区宽度除以两条边沿的时间差测速 |
| `mfoc_hall_Step` | 控制拍当前 tick | 从已发布的边界按电转速外推，交回角度和最近测得的 PU 速度 |

表按电角度的正向顺序填写，适用于有效码为 1 到 6 的三数字霍尔布置。相邻码只有一位变化，六个正向角宽合计一电周，每个角宽小于半周，支持跨零和不等宽扇区。表中已含相序、方向与电零位。速度基值取 `mfoc_port_cfg_t.electrical_speed_base_hz`。三路霍尔作为 FOC 位置源的物理背景可参见 [Microchip AN4064](https://ww1.microchip.com/downloads/en/Appnotes/Sensored-Hall-Effect-Sensor-Based-Field-Oriented-Control-of-Three-Phase-BLDC-Motor-Using-dsPIC33CK-DS00004064.pdf)。

例如，沿正方向先读到 A，再依次跨入 B、C。初读 A 时返回 A 的中点、速度为零。A→B 时角度锚定在 A/B 边界，速度仍为零。B→C 时，两次真实跳变包围了完整的 B 扇区，这时才得到转速。如果序列为 A→B→A，第二条边沿回到同一边界，速度归零，在下一条同向边沿重新测速。霍尔的边界和角宽均按电周表达。

## 接到板级 Port

1. 在 `mfoc_port_init` 中用实际板级标定表初始化一个 `mfoc_hall_t`。进入控制前交给它一次当前电平，建立初始扇区。
2. 按捕获顺序把随后每一条边沿交给同一实例。可由捕获中断保存事件，在 `mfoc_port_position_run` 中消费。若直接在捕获中断调用 `OnEdge`，保证它与控制读者串行，或让控制读者以更高优先级运行。双槽发布依赖单写者，读者复制快照期间写者须被排除。
3. 板级实现下列位置读取。选择 `MFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_POSITION` 后，Feedback 直接发布这里得到的角度和速度。

```c
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    return mfoc_hall_Step(&hall, mfoc_port_tick(), angle, speed);
}
```

零位由板级的六边界标定表给出。单次固定角对齐只观察到一个扇区，无法测出完整边界。使用预标定表的板子若未实现重新标定，`mfoc_port_position_calibrate_zero` 应向对齐流程返回 `MFOC_RESULT_INVALID_STATE`。G431 现有应用使用 AS5600，其标定路径见 [Encoder 组件](encoder.md)。

## 边界行为

外推到达当前扇区出口后，角度停在边界，报告的速度仍是上一完整扇区的测量值。实际新码到来后才能进入下一扇区。重复码不刷新时间。边沿年龄大于 `timeout_ms` 时，Step 返回 `SAFETY` 并保留调用方输出。因此，连续静止或极低速且长于该窗口的工况也会超时，板级要按工作范围选择窗口。长间隔后的首个不同有效码从扇区中点重新起步。

有效窗口内的非法码、非相邻跳变或同 tick 的不同码使本次估计失效，下一有效码重新建立扇区。迟到的旧捕获会被拒绝，最近的有效估计继续保留。时间比较使用 64 位 tick。

Encoder 和 Hall 共用库内的位移换算。Encoder 用机械转/秒算机械位移，再按极对数换为电角度。Hall 用电转/秒算电位移，再按扇区限位。公开 Port 始终交付同口径的电角度和电速度 PU。[共享算式](../../position/src/mfoc_position_travel.h) · [FLOAT/FIXED 边沿回放与 Feedback 接线测试](../../tests/position/hall_test.c)

运行 `mfoc/tests/scripts/run_hall_tests.ps1` 可在主机重放正反转、跨零、不等宽扇区、迟到事件与超时恢复，并验证实际 Feedback 的输出。该回放使用合成标定表。接入实际霍尔板时还需实测码序、扇区边界和捕获时间。G431 的双后端固件构建验证的是现有 AS5600 配置和未引用组件的链接裁剪。
