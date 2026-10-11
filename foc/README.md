# FOC 框架

框架面向单台电机，支持 FLOAT 和 Q16.15 FIXED 后端。`foc_app_t` 组合 Motor、Position、Encoder 和 Identify；目标端口提供 ADC、PWM 与传感器的编译期绑定。

[当前架构与模块契约](docs/foc-architecture.md) 是能力、单位、对象所有权和 ISR 顺序的唯一说明。[V1 收敛记录](docs/2026-10-09-foc-framework-v1.md) 记录本轮修改与验证。历史审核保留当时证据，不作为当前能力快照。

当前支持 SENSOR、固定频率强拖、I/f 启动及强拖到 SMO 接管、旁路 SMO、固定轴 Rs/Ld 辨识。是否启用启动和自动接管取决于 `app/motor_config.h`。HFI、Flux/EKF、任意来源切换和位置环尚未实现。

运行 profile 在 `app/motor_config.h`；板级采样、标度与执行频率在目标配置中。Core 电压命令沿用无量纲调制度，模型电压采用电压 PU，二者不可互换。Motor 配置始终采用 PU；物理速度 profile 只在 App 初始化时绑定一次。

开发时保持对象职责、单位、样本拍关系、错误收敛及验收入口；内部布局可随实现调整。[experimental](experimental/README.md) 中的组件不属于框架已验收能力。

构建使用仓库根目录 `make.bat`；主机测试位于 `foc/tests`。目标编译与主机回归不替代 PWM/ADC 生效时序、WCET、栈及台架验证。
