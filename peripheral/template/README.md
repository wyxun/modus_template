# `peripheral/template`：32 位芯片 MDI 参考后端

本目录是一个**不绑定具体厂商**的 32 位 MCU 参考工程。它展示芯片层如何把同一颗
MCU 的多个硬件外设绑定到公共 MDI，再交给应用层使用。寄存器地址和位定义都是模板
值，移植到真实芯片时只修改 `mdi/backend.h` 和启动初始化，不修改应用调用方式。

```text
peripheral/template/
├── README.md
├── mdi/
│   ├── backend.h       抽象 32 位寄存器模型和芯片后端绑定宏
│   ├── foc_adapter.h   FOC 语义端口到静态 MDI 资源的适配
│   ├── instance.h      本芯片的多外设资源实例
│   ├── state.c         DMA、Tick 和 Stream 的实例存储
│   ├── service.c       DMA 中断发布、mdi_Init/mdi_Service/mdi_Clock 板级维护
│   └── service.h       DMA 中断入口声明
├── tests/
│   ├── contract.c       Timer/Stream 编译期契约
│   ├── foc_port_contract.c FOC 静态端口编译期契约
│   ├── runtime.c        Timer/Stream 主机运行检查
│   └── service_runtime.c Board service 主机运行检查
├── foc_port.h           FOC 工程的目标头文件入口
└── examples/
    └── multi_peripheral_app.c
```

## 本例绑定的外设

`mdi/instance.h` 使用同一颗抽象 32 位芯片展示：

| 资源 token | 外设语义 | 应用接口 |
| --- | --- | --- |
| `status_led`、`user_button` | GPIO 输入输出 | `MDI_IO_Read/Write` |
| `dac_parallel` | 跨 GPIO 端口的并口数据 | `MDI_IO_Write`、`MDI_IO_WriteMasked` |
| `adc1_dma` + `adc1_mean` | 规则 ADC/DMA 多通道块和均值 | `mdi_Service` |
| `bus_voltage`、`bus_current`、`temperature` | 已发布均值的通道视图 | `MDI_ADC_Read` |
| `phase_current` | 注入 ADC 的三相完成帧 | `MDI_Sample_ReadCompleted` |
| `bridge` | 三相中心对齐 PWM | `MDI_PWM_SetDuty/Commit` |
| `buzzer` | 单路变频 PWM | `MDI_PWM_SetFrequency/SetDuty` |
| `encoder_i2c_hw` | 硬件 I2C 主机 | `MDI_I2C_Reg8_Read` |
| `sensor_bus` + `encoder_i2c_sw` | 软件 I2C 主机 | `MDI_I2C_Reg8_Read` |
| `config_eeprom` | SPI 25xx EEPROM | `MDI_SPI_EEPROM_Read/Write` |
| `adc_service_timer` | 通用外设计时器示例 | `MDI_TIMER_SetFrequency/Start/Stop` |
| `pt32_raw_tick` | 板级原始 Tick | `MDI_TICK_Now` |
| `board_stream` | 静态环形字节流 | `MDI_STREAM_Write/Read` |

硬件 I2C 和软件 I2C 是两个可替换 provider。实际产品只能让一个 provider 获得同一组
SDA/SCL 资源的所有权；本例同时声明它们是为了展示替换关系。

DMA 完成中断调用 `pt32_AdcDmaCompleteIrq()`，释放当前槽位并发布完成计数。
`modus_Init()` 在日志和对象初始化之前调用 `mdi_Init()` 准备队列及服务状态；`modus_Run()` 在对象 Run 前调用 `mdi_Service()`：先对完成的块求均值，再用 perfc 显式截止时间
启动下一次规则 ADC 扫描。通道读取只读取已发布的快照。模板启动时将 DMA 目的地址
设为当前槽位；真实芯片后端还需按硬件规则配置、停止 DMA。

## ADC/DMA feature 用法

模板把一次规则 ADC 扫描定义为 3 个通道、每通道 `PT32_ADC_SAMPLE_COUNT` 个连续样本。芯片实例隐藏 DMA 缓冲区，
应用只看到按语义命名的通道：

```c
mdi_adc_value_t tBusVoltage = {0};
mdi_adc_value_t tBusCurrent = {0};

(void)modus_Run();  /* 内部自动调用 mdi_Service() */
(void)MDI_ADC_Read(bus_voltage, &tBusVoltage);
(void)MDI_ADC_Read(bus_current, &tBusCurrent);
```

`MDI_ADC_Read()` 不启动转换，也不做滤波；首次服务处理完成前返回 `MDI_BUSY`。
模板默认按 100 Hz 调度。`mdi_service_SetAdcFrequency(wHz)` 修改实际服务周期（1..1000000 Hz，周期向上取整为微秒），有效调用后重新计时，非法设置保留旧值；实际频率受前台调用延迟与 DMA 忙限制，需要严格硬件时序时改用硬件触发。

`bus_voltage`、`bus_current` 和 `temperature` 共用规则 ADC/DMA 采集组；注入 ADC
的 `phase_current` 独立提供一次完成的 U/V/W 帧。FOC 控制中断直接调用
`MDI_Sample_ReadCompleted(phase_current, ...)`，完成控制计算后调用
`MDI_PWM_SetDuty(bridge, ...)` 和 `MDI_PWM_Commit(bridge)`。这两个阶段之间需要运行
电流环，因此模板不再提供把采样和提交紧挨着执行的 `template_FocCycle()` 包装。

`foc_port.h` 选择 `mdi/foc_adapter.h`。适配器把 FOC 的三相原始值、归一化占空比、
母线原始 ADC 值和 PWM 安全操作直接映射到 MDI 的静态资源。FOC 应用负责把母线 ADC
计数换算成电压；编码器和位置服务由具体目标板接入。移植时将模板的 ADC 注入触发
寄存器、DMA 和 PWM 故障动作换成实际芯片实现。

`PT32_ADC_SAMPLE_COUNT` 是实例级编译期配置，默认值为 8，也可以在构建配置中覆盖，
例如 `-DPT32_ADC_SAMPLE_COUNT=16`。它对该采集组的所有通道同时生效，并会改变 DMA
缓冲区大小和均值循环次数。不同通道需要不同重复次数时，应拆成多个采集组或增加新的
feature；不要把通道差异加入 core 契约。

## Timer、Raw Tick 和 Stream

`adc_service_timer` 使用独立的 TIMER3 资源，作为 Timer 契约示例。Timer provider 只负责频率、启停和运行状态，
不注册 ISR 回调；真实芯片应在后端补齐时钟树、更新事件和中断向量配置。

`pt32_raw_tick` 是板级无单位计数器，模板 provider 读取 `g_qwPt32RawTick`；真实板应像参考
项目一样从单调硬件计数器读取。`mdi_Clock()` 当前为空，不负责制造 raw tick。

`board_stream` 显式使用 `mdi/feature/uart_stream.h` 与 mringbuf。Read/Write 返回状态，第四个参数返回实际字节数；Read 立即消费 RX 队列，分帧归上层协议。短传输与空队列返回 OK，溢出返回 OVERRUN 并保留已读数据。`Available` 返回立即可读数量，`IsBusy` 包含队列和最后一字节物理发送。队列初始化成功后才启用 UART IRQ，Clock 不维护分帧计时。

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
芯片的全部硬件时序规范。25xx EEPROM 和 AS5600 读取属于应用侧组合
示例，不是 `core` 公共契约的一部分。

## 接入真实芯片时修改什么

1. 用真实芯片的 GPIO、ADC、定时器、I2C 和 SPI 寄存器替换 `backend.h` 中的示例结构。
2. 把时钟、GPIO 模式、开漏上拉、ADC 触发/DMA、PWM 预装载和故障门控初始化放入板级
   启动代码；MDI 访问函数不隐式初始化外设。
3. 根据芯片手册填写 I2C/SPI 状态位、超时和清错顺序。模板 transfer provider 只用于
   展示职责边界，不是可直接生产的时序实现。
4. 保持 `instance.h` 中的资源语义和应用接口不变。换芯片只替换绑定和后端。

## 验证入口

从项目根目录执行：

```text
python modus/src/mdi/tests/run_tests.py
```

The runner covers template Timer/Stream and ADC scheduling, links mringbuf and supplies the perfc host port. Override tools with HOST_CC, ARM_CC and ARM_OBJDUMP.

应用层完整调用见 [multi_peripheral_app.c](examples/multi_peripheral_app.c)，资源绑定见
[instance.h](mdi/instance.h)，板级服务见 [service.c](mdi/service.c)，后端职责见
[backend.h](mdi/backend.h)。
