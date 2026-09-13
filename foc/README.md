# FOC 框架

FOC 框架当前面向单台电机，支持 float 和 Q16.15 fixed 两种数值后端。产品组合、Motor 控制、FOC 数学、位置观测与目标硬件端口各自承担明确职责；
具体 ADC、PWM 和传感器映射由目标适配层提供。

## 文档入口

- [使用与电机调试指南](./docs/foc-test-guide.md)：介绍 API 和 Shell 命令、新电机配置、U/V/W 核对、新硬件适配与首次上电调试。
- [当前架构说明](docs/foc-architecture.md)：说明模块职责、Motor 状态机、调度边界和硬件无关的数据流。

## 当前功能范围

已实现 VOLTAGE、CURRENT 和编码器 SPEED 控制。位置闭环、无感观测器接入与接管、多电机管理尚未实现；`FOC_MODE_POSITION` 是明确的未实现边界。`motor_limits_t` 目前只声明，控制路径不调用。定子电阻和 D/Q 电感保留为电机元数据，当前不参与控制参数计算。

## 架构概览

`foc_app_t` 组合 `motor_t` 与 `foc_encoder_t`。Motor 编排 `foc_core`、速度 PI 和直接 ADC/PWM 端口；目标适配层负责提供具体硬件实现。模块职责与控制时序见[架构说明](docs/foc-architecture.md)。

## 构建与验证

构建目标、float/fixed 后端选择、主机回归测试和固件调试步骤见[使用与电机调试指南](../docs/foc-test-guide.md)。README 仅作为入口和当前能力摘要，不重复列出板级命令、接线与调试步骤。
