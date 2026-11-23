# 接入 MFOC

## 先运行主机示例

进入源码包的 `examples/host_replay`，使用 GNU Make 和 host GCC 运行示例。Windows 上用 `mingw32-make` 代替 `make`。

```sh
make NUMERIC=FLOAT BACKEND=POSITION
make NUMERIC=FIXED BACKEND=POSITION IDENTIFY=1
```

示例模拟功率级、电流采样和已提交 PWM。第二条命令会完成模拟 Rs 辨识。可选反馈和可用的编译组合见 [主机回放说明](../examples/host_replay/README.md)。

## 接入自己的功率板

1. 在项目的 Makefile 中包含 `mfoc/mfoc.mk`，编译 `MFOC_SOURCES`，使用 `MFOC_INCLUDES`。项目和库使用同一组数值格式、外设回调、反馈与辨识编译宏。若使用自定义电机头，为这些编译单元统一追加 `MFOC_MOTOR_CONFIG_FLAGS`。[配置说明](configuration.md)
2. 按 `mfoc_port.h` 实现电流、时钟、PWM 和启用的反馈输入。在写 Motor 代码前，先确认三相顺序、电流极性、ADC 就绪标志和 PWM 实际更新点。[Port 契约](port-contract.md) · [移植模板](../port/template/mfoc_port_template.c)
3. 保持相输出关闭并配置外设，提供 `mfoc_motor_cfg_t`，调用 `mfoc_motor_Init`。处理返回值。配置的相电流、母线电压和电频率基值也用于 Port 换算。[Motor 公共入口](../motor/include/mfoc_motor.h) · [标幺单位](numeric-contract.md)
4. 每个完整 ADC 帧在控制 ISR 内调用一次 `mfoc_motor_Step`，前台周期调用 `mfoc_motor_Run`。前台启停、命令更新和 ISR 之间按 Motor 契约串行化。启动前提交电流或速度指令，检查 `mfoc_motor_Start` 的返回值。
5. 需要停机时调用 `mfoc_motor_Stop`。故障排除后检查 `mfoc_motor_ClearFault` 的返回值，再明确发起启动。电流校零、母线采样、PWM 保护和故障清锁存的板级约束见 [Port 契约](port-contract.md)。

启用辨识后，前台通过 `mfoc_motor_StartIdentification(&motor, MFOC_IDENTIFICATION_RESISTANCE)` 启动 Rs 实验，后续用 `mfoc_motor_Run` 推进，再用 `mfoc_motor_GetIdentificationResult(&motor, &params)` 读取完成的结果。打开停机模型更新时，`mfoc_motor_ApplyModel` 应用同基值上的 PU 参数。更换基值或极对数需要重新初始化。对齐、停机或取消失败时按返回值和 Motor 状态处理。

新连接和运行范围分开验收。需要依据现有记录选择首个目标组合时，查 [发布支持矩阵](support-matrix.md)。
