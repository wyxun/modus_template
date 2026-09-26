---
name: foc-motor-control-methodology
description: Use when designing, explaining, tuning, validating, or debugging PMSM/FOC control, including motor equations, per-unit or fixed-point scaling, current and speed loops, sensorless observers, parameter identification, PWM/ADC timing, startup, MTPA, field weakening, or overmodulation.
---

# FOC 电机控制开发方法论

## Overview

把 FOC 问题拆成可验证的模型、时序、模块和实验。先建立电机与控制器的基线，再讨论参数优化；不要把某一组电机的经验参数当成普适定律。

本 Skill 是控制域知识层：它覆盖公式、推导、标幺化、模块设计、整定和验证。它不替代项目的 C/MODUS 代码规范，也不替代 MCU 调试工具流程。

## When to use

- 需要推导或解释 PMSM、FOC、SVPWM、坐标变换、MTPA、弱磁或过调制；
- 需要设计标幺值、Q 格式、离散滤波器或控制环参数；
- 需要划分采样、变换、观测器、环路、调制、启动或保护模块；
- 需要辨识 Rs、Ld/Lq、磁链、惯量、极对数或检查辨识条件；
- 需要分析 PWM/ADC 时序、角度补偿、单电阻采样或高频注入；
- 需要从仿真、编码器基线和硬件波形证据定位 FOC 运行问题。

## Core rules

1. 先列出电机类型、接法、坐标约定、极对数、采样方式、PWM 更新点、执行周期、参数单位和限幅，再开始推理。
2. 明确区分四类内容：模型公式、带假设的近似、工程经验、待验证实验。
3. 先验证模型、符号、单位、采样和 PWM，再调整 PI、观测器或启动参数。
4. 先用仿真验证公式和离散时序，再用编码器建立闭环基线，最后让观测器参与 FOC；每轮实验只改变一个主要变量。
5. 对每个建议给出观察量、实验动作、通过条件和停止条件；没有证据时不要宣称已定位根因或已验证真机性能。

## Choose a reference

按问题读取最相关的参考文件，不需要默认加载全部内容：

| 问题 | 读取 |
|---|---|
| 模型、公式、坐标、SVPWM、MTPA、弱磁 | `references/foc-foundations-and-formulas.md` |
| 标幺值、Q 格式、定点、滤波、离散延时 | `references/per-unit-fixed-point-and-discrete-time.md` |
| FOC 模块划分、数据流、执行频率、接口 | `references/foc-module-architecture.md` |
| 参数辨识、电流环、速度环、限幅和整定 | `references/parameter-identification-and-loop-tuning.md` |
| 仿真到实机、波形证据、故障现象和排查 | `references/validation-debugging-and-symptoms.md` |

## Default workflow

1. 写下目标工况和安全边界：电机、母线、电流、速度、负载、方向和最大允许输出。
2. 核对模型和单位：相量/线量、Y 型等效、机械角/电角、峰值/RMS、真实值/ 标幺值、浮点/Q 格式、正负号和变换约定。
3. 核对实时链路：PWM 触发 ADC，ADC/DMA 完成，快速控制计算，比较值更新，下一拍生效；记录每个模块的执行频率和延时。
4. 逐层建立基线：PWM 波形、直流电流和坐标变换，电流环，编码器 FOC，观测器对比，无感接管，最后再做弱磁、MTPA、过调制和降噪。
5. 输出证据表：假设、测量值、波形、改动、结果、未解决风险和下一实验。

## Project integration

**REQUIRED SUB-SKILL:** 修改嵌入式 C、MODUS、MDI 或 FOC 实现时使用 `embedded-coding`，遵守其对象所有权、实时路径、测试和代码约束。

**REQUIRED SUB-SKILL:** 需要 MCU 运行态、RTT、波形、寄存器或故障证据时使用 `aitrace`；先采用被动采集，停机、复位、GDB 或烧录遵守该 Skill 的确认要求。

本 Skill 的数值和方法来自所给课件的提炼，并结合公式适用条件重新组织；不复制课程宣传、个人信息或版权声明。课件中的经验数值只能作为初始范围，必须结合电机参数、采样时序、功率级和波形重新验证。
