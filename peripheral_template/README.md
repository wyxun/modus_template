# `peripheral_template`：32 位芯片 MDI 参考后端

本目录是一个**不绑定具体厂商**的 32 位 MCU 参考工程。它展示芯片层如何把同一颗
MCU 的多个硬件外设绑定到公共 MDI，再交给应用层使用。寄存器地址和位定义都是模板
值，移植到真实芯片时只修改 `mdi/backend.h` 和启动初始化，不修改应用调用方式。

```text
peripheral_template/
├── README.md
├── mdi/
│   ├── backend.h       抽象 32 位寄存器模型和芯片后端绑定宏
│   ├── instance.h      本芯片的多外设资源实例
│   └── state.c         DMA 缓冲区和发布状态的实例存储
└── examples/
    ├── multi_peripheral_app.c
    └── multi_peripheral_app.h
```

## 本例绑定的外设

`mdi/instance.h` 使用同一颗抽象 32 位芯片展示：

| 资源 token | 外设语义 | 应用接口 |
| --- | --- | --- |
| `status_led`、`user_button` | GPIO 输入输出 | `MDI_IO_Read/Write` |
| `dac_parallel` | 跨 GPIO 端口的并口数据 | `MDI_IO_Write`、`MDI_IO_WriteMasked` |
| `adc1_dma` + `adc1_mean` | ADC/DMA 多通道块和按需均值 | `template_SetAdcSampleFrequency`、`template_AdcService` |
| `phase_u`、`phase_v`、`bus_voltage` 等 | 同一采集组的通道视图 | `MDI_ADC_Read` |
| `phase_current` | 同一采集组的三相一致帧视图 | `MDI_Sample_ReadCompleted` |
| `bridge` | 三相中心对齐 PWM | `MDI_PWM_SetDuty/Commit` |
| `buzzer` | 单路变频 PWM | `MDI_PWM_SetFrequency/SetDuty` |
| `encoder_i2c_hw` | 硬件 I2C 主机 | `MDI_I2C_Reg8_Read` |
| `sensor_bus` + `encoder_i2c_sw` | 软件 I2C 主机 | `MDI_I2C_Reg8_Read` |
| `config_eeprom` | SPI 25xx EEPROM | `MDI_SPI_EEPROM_Read/Write` |

硬件 I2C 和软件 I2C 是两个可替换 provider。实际产品只能让一个 provider 获得同一组
SDA/SCL 资源的所有权；本例同时声明它们是为了展示替换关系。

ADC/DMA 的完成中断只调用内部的 `MDI_ADC_DMA_Publish(adc1_dma)`，不做累加、均值或
滤波。应用在自己的任务或控制循环中调用 `template_AdcService(current_tick)`：函数先
检查 DMA 完成标志，有新 block 才在当前上下文执行均值或其他滤波；然后比较当前 tick
与上次启动 tick，决定是否启动下一次 ADC。通道读取只读取已经发布的快照。没有及时
读取时，内部可以检测 DMA 丢块；
真实芯片需要用双缓冲、环形缓冲或暂停 DMA 保证正在处理的 block 不会被覆盖。

## ADC/DMA feature 用法

模板把一次 ADC 扫描定义为 5 个通道、每通道 `PT32_ADC_SAMPLE_COUNT` 个连续样本。芯片实例隐藏 DMA 缓冲区，
应用只看到按语义命名的通道：

```c
mdi_adc_value_t tBusVoltage = {0};
mdi_adc_value_t tBusCurrent = {0};

(void)template_SetAdcSampleFrequency(1000U, wSysTick);
(void)template_AdcService(wSysTick);
(void)MDI_ADC_Read(bus_voltage, &tBusVoltage);
(void)MDI_ADC_Read(bus_current, &tBusCurrent);
```

应用侧也可以只依赖 `examples/multi_peripheral_app.h` 的模板包装入口，隐藏
`adc1_mean` 资源 token：

```c
uint32_t wBusVoltage = 0U;

(void)template_SetAdcSampleFrequency(1000U, wSysTick);
(void)template_AdcService(wSysTick);
(void)template_ReadBusVoltage(&wBusVoltage);
```

`template_AdcService()` 只应在任务或控制上下文调用；DMA 中断入口
`template_AdcDmaCompleteIrq()` 仍然只发布完成计数。`MDI_ADC_Read()` 不启动转换，也不
做滤波；如果还没有完成一次服务处理，会返回 `MDI_BUSY`。模板用 SysTick 驱动示例，
`template_SetAdcSampleFrequency()` 的第二个参数就是设置时刻的 SysTick，用来建立
下一次触发的时间基准。需要高于 SysTick 的采样率时，应把同一
`MDI_ADC_Start(adc1_mean)` 绑定到硬件定时器触发，而不是提高 while 循环频率；那种
后端可以直接使用公共的 `MDI_ADC_SetSampleFrequency()` 硬件触发接口。

`bus_voltage` 和 `bus_current` 使用同一个 ADC/DMA 采集组，但每次读取仍是统一的
`MDI_ADC_Read()`。均值次数、通道顺序和 DMA 缓冲区由 `instance.h` 固定；DMA 中断不
调用均值函数。FOC 使用同一组中的 `phase_u`、`phase_v`、`phase_w` 一致帧视图，避免
把三相采样误设计成三个独立硬件采集器。模板中的 `template_FocCycle()` 先在控制
上下文调用一次内部更新，再把同一快照交给 FOC。

`PT32_ADC_SAMPLE_COUNT` 是实例级编译期配置，默认值为 8，也可以在构建配置中覆盖，
例如 `-DPT32_ADC_SAMPLE_COUNT=16`。它对该采集组的所有通道同时生效，并会改变 DMA
缓冲区大小和均值循环次数。不同通道需要不同重复次数时，应拆成多个采集组或增加新的
feature；不要把通道差异加入 core 契约。

## 与 MDI 框架规范的符合性

本例已经覆盖芯片后端的**结构和静态绑定规则**，可用于检查应用层调用是否保持芯片无关：

- `instance.h` 提供资源 token、引脚/通道映射和能力组合；
- `backend.h` 提供寄存器宽度、直接访问 provider、范围检查和编译期资源检查；
- IO、采样帧、PWM 组以及 I2C/SPI 使用公共 MDI 契约；
- 软件 I2C 和硬件 I2C 使用相同的事务接口，设备协议在 feature 层组合；
- 应用调用不保存设备对象、`void *` 上下文或函数指针表。

它还不是完整的生产芯片后端，以下内容必须由真实芯片目录补齐：

- 时钟、GPIO 模式、复用、上拉、ADC 触发/DMA、PWM 预装载和故障门控初始化；
- 真实 I2C/SPI 的状态位、重复 START、STOP、片选、错误清除和超时换算；
- DMA/中断发布与消费的所有权、缓存一致性、并发保护和板级故障恢复；
- 目标芯片上的时序、汇编、链接和故障动作验证。

因此，模板满足 MDI 的**分层、资源绑定和调用形态规范**，但不能宣称已经满足某一颗真实
芯片的全部硬件时序规范。`MDI_FOC_BIND`、25xx EEPROM 和 AS5600 读取属于应用侧组合
示例，不是 `core` 公共契约的一部分。

## 接入真实芯片时修改什么

1. 用真实芯片的 GPIO、ADC、定时器、I2C 和 SPI 寄存器替换 `backend.h` 中的示例结构。
2. 把时钟、GPIO 模式、开漏上拉、ADC 触发/DMA、PWM 预装载和故障门控初始化放入板级
   启动代码；MDI 访问函数不隐式初始化外设。
3. 根据芯片手册填写 I2C/SPI 状态位、超时和清错顺序。模板 transfer provider 只用于
   展示职责边界，不是可直接生产的时序实现。
4. 保持 `instance.h` 中的资源语义和应用接口不变。换芯片只替换绑定和后端。

## 示例编译

示例不需要厂商头文件，可以用主机编译器做语法检查：

```text
gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only                                 \
    -Imodus/src -Iperipheral_template                                            \
    peripheral_template/examples/multi_peripheral_app.c                          \
    peripheral_template/mdi/state.c
```

应用层完整调用见 [multi_peripheral_app.c](examples/multi_peripheral_app.c)，公开包装
声明见 [multi_peripheral_app.h](examples/multi_peripheral_app.h)，资源绑定见
[instance.h](mdi/instance.h)，后端职责见 [backend.h](mdi/backend.h)。
