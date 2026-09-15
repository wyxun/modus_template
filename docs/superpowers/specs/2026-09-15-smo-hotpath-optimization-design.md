# SMO 高频路径优化设计

## 1. 目标与边界

本次只优化 `foc_smo_Step()` 的运算路径，不改变 SMO 的输入输出契约、启动策略、
Observer 所有权或 HFI 预留边界。输入仍是 PU 电流和上一控制周期的 PU 模型电压，
输出仍是 BAM32 电气角度、电气速度和有效标志。

优化目标是按照现有 SMO/PLL 离散公式减少重复乘法和不必要的函数调用，同时保持
FLOAT/FIXED 两种后端的饱和、舍入和状态迁移语义。第一阶段不做可能改变定点舍入
顺序的激进代数合并。

## 2. 当前热路径与问题

当前路径为：

```text
foc_observer_Step
  -> foc_smo_Step
      -> smo_AxisStep(alpha)
      -> smo_AxisStep(beta)
      -> PLL / angle / qualification
```

两轴模型中都有公共项：

```text
qElectricalModelSpeed = qPllMechanicalSpeed * qPolePairs
qCrossAxisSpeedGain = qCrossAxisGain * qElectricalModelSpeed
```

`qCrossAxisSpeedGain` 对 α、β 两轴相同，但当前在每次 `smo_AxisStep()` 中重复计算。
此外，`foc_mul_wide()`、`foc_add_sat()`、`foc_sub_sat()` 和 `foc_abs()` 是高频路径
中的普通外部函数，无法从源码保证被内联。固定点 `foc_mul_wide()`还包含 32 位乘积
扩展到 64 位、缩放和饱和处理。

## 3. 设计方案

### 3.1 公共系数只计算一次

在 `foc_smo_Step()` 内先计算机械速度对应的电气速度，再计算一次交叉耦合公共系数：

```c
qElectricalModelSpeed = foc_mul_wide(
    ptSmo->qPllMechanicalSpeed,
    ptSmo->qPolePairs);
qCrossAxisSpeedGain = foc_mul_wide(
    ptSmo->qCrossAxisGain,
    qElectricalModelSpeed);
```

`smo_AxisStep()` 不再接收原始 `qElectricalModelSpeed` 后重复计算，而是直接使用
`qCrossAxisSpeedGain` 与另一轴电流估计相乘。这样每个采样周期从两次公共系数乘法
减少为一次，α/β 的交叉电流乘法仍各保留一次。

### 3.2 保留单轴辅助函数，不展开两份代码

`smo_AxisStep()`继续承担单轴积分器、滑模符号、反电动势滤波和限幅职责。它仍被调用
两次，因为 α、β 轴拥有独立状态；`foc_smo_Step()`在第一次调用前保存另一轴历史估计，
确保交叉耦合使用上一采样状态。

数值基础操作提供可内联实现，优先覆盖：

- `foc_mul_wide()`；
- `foc_add_sat()`；
- `foc_sub_sat()`；
- `foc_abs()`。

内联实现必须与现有 `foc_numeric.c` 语义一致。FLOAT 保持普通浮点运算，FIXED 保持
当前 Q15 缩放、INT32 饱和和截断行为。对外函数名不变，其他模块不需要改 API。

### 3.3 暂不做的优化

以下变更不放入第一阶段：

- 将 `Kv * voltage - Kv * bemf` 合并成 `Kv * (voltage - bemf)`；
- 将 PLL 中的 `FOC_HALF` 提前折叠到系数；
- 将 α/β 两个单轴函数手工展开成一大段代码；
- 删除公共 API 的空指针检查；
- 增加运行时函数指针或新的 Observer 调度层。

这些做法可能减少更多指令，但会改变定点舍入/饱和顺序，或增加代码体积和维护成本，
必须在独立的数值等价性和目标 MCU 周期测试后再评估。

## 4. 测试与验收

先增加一个可执行的结构契约，确保公共交叉系数在 `foc_smo_Step()` 只计算一次，且
`smo_AxisStep()`不再计算原始速度交叉系数。该测试在优化前应失败，优化后通过。

行为回归继续覆盖：

- FLOAT 和 FIXED 初始化；
- SMO 复位后的首拍状态；
- α/β 交叉耦合使用上一采样值；
- PLL 角度回绕和速度输出；
- 资格判断与失锁清零；
- 空指针和配置错误。

最终验证需在目标编译器上分别比较优化前后的：

1. SMO 输出角度、速度和 `valid`；
2. FIXED 中间状态和边界饱和结果；
3. 高频入口峰值周期数和栈使用量。

主机测试脚本使用 `-O0` 时只能验证行为，不能作为产品周期数结论。没有目标反汇编
和周期测量前，不宣称已经满足 20 kHz 的实时预算。

## 5. 文件范围

- 修改 `foc/observer/foc_smo.c`：提取公共交叉系数并调整单轴函数参数。
- 修改 `foc/math/foc_numeric.h` 与 `foc/math/foc_numeric.c`：为热路径数值操作提供
  保持原语义的内联实现，避免重复导出定义。
- 修改或新增 `foc/tests/` 下 SMO 测试：验证结构契约和 FLOAT/FIXED 行为回归。

不修改 Motor/Observer 所有权设计，不实现 HFI，不引入新的无感算法。
