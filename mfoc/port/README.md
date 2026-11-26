# MFOC 移植与项目配置

`port/` 是 MFOC 与目标工程的接合处。每个项目目录保存板级适配、
电机参数、校准和运行配置。

| 目录 | 职责 |
| --- | --- |
| `include/` | [mfoc_port.h](include/mfoc_port.h) 定义电流、母线、位置、时钟和 PWM 的接口、单位及调用时机。 |
| `template/` | 提供可复制的 [板级源码](template/mfoc_port_template.c) 和 [电机参数头](template/motor_template.h)。 |
| `projects/<项目>/` | 保存 `project.mk`、`motor_<电机>.h`、项目整定和校准数据。`project.mk` 选择目标板、电机头、反馈和运行条件。 |
| `projects/<项目>/board/` | 接通该项目的采样、反馈、PWM、故障保护和 IRQ 绑定。G431 的 `mfoc_pwm_phase.*` 负责更新相位同步，F413 的 `mfoc_safety.*` 负责开桥检查、帧保护和断帧关桥。底层驱动和芯片构建分别由主工程的 `peripheral/`、`target/` 提供。 |
| `modus/` | 两个示例目标共用的 MODUS ADC 中断流程。各板的 `mfoc_motor_irq_binding.h` 在编译期绑定向量、应用实例和采样帧。 |

## 构建现有项目

在主工程根目录运行。下列命令生成固件。

```powershell
.\make.bat TARGET_CHIP=stm32g431 MFOC_PROJECT=g431_2804 MFOC_NUMERIC=float
.\make.bat TARGET_CHIP=at32f413 MFOC_PROJECT=f413_5562 MFOC_NUMERIC=fixed
```

`f413_5562` 选择 F413、5562 和 24 V Hall 配置，电机参数和实测霍尔表
都在该项目根目录。`MFOC_PROJECT` 对应 `projects/` 下的目录名。
省略它时，G431 和 F413 分别选择 `g431_2804` 和 `f413_5562`。

G431 默认选 Flux。用 `MFOC_G431_FEEDBACK=flux|smo|hfi|ekf` 选择
主反馈，其中 EKF 只支持 FLOAT，原有 `STM32G431_EKF=1` 仍可使用。例如：

```powershell
.\make.bat TARGET_CHIP=stm32g431 MFOC_NUMERIC=fixed MFOC_G431_FEEDBACK=smo
```

在 [g431_2804 项目整定头](projects/g431_2804/mfoc_project_config.h)
调整 d/q 电流环 PI、速度环 PI、四种无感算法和启动接管参数。电机物理量
和标幺基值在同目录的 [motor_2804.h](projects/g431_2804/motor_2804.h)。

## 换电机或新增项目

1. 从 [motor_template.h](template/motor_template.h) 复制一份到
   自己的项目目录，命名为 `motor_<电机>.h`，替换示例参数和基值。
   项目支持多台电机时，在该项目内各放一份参数头。多个项目用到同一
   台电机时，各项目可以保留自己的副本。
2. 使用霍尔时，把实测扇区表和对应电机头放在一起。霍尔 GPIO 和捕获通道
   由板级绑定。相序或霍尔线序变化后要重新核对扇区表和电角度零点。
3. 在项目的 `project.mk` 用 `MFOC_MOTOR_CONFIG` 直接选电机头，
   并设置反馈、母线基值和命令限流。板级适配放进项目的 `board/`。

## 移植新板或接入独立工程

1. 复制 [mfoc_port_template.c](template/mfoc_port_template.c)，按
   [Port 契约](../docs/port-contract.md) 实现电流采样、PWM、时钟和
   所选位置输入。为库和应用统一提供实际控制频率与调制上限。
2. 将库源文件和一份板级 Port 实现编入同个目标。应用初始化板上
   外设后调用 `mfoc_motor_Init()`。前台调用 `mfoc_motor_Run()`，
   每个完整电流采样帧在控制中断中调用一次 `mfoc_motor_Step()`。
3. 独立源码包提供本说明、Port 接口和模板。接入自己的工程时，
   由工程提供板级实现和 ADC 中断入口，并通过 `mfoc_motor_cfg_t`
   传入电机参数和基值。编译选项与参数选择见
   [配置说明](../docs/configuration.md)。
