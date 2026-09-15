# FOC 参数受控台架辨识极简设计规范 (工程落地版)

**日期：** 2026-09-15  
**目标：** 在 STM32G431 单核控制路径下，用最纯粹、零冗余的极简 Controller 实现电机定子电阻 $R_s$、直交轴电感 $L_d$、$L_q$ 的受控阶跃辨识，满足上真实台架的高可靠性要求。

---

## 1. 架构哲学与前置条件

1. **强前置条件：必须完成电气零位对齐**：
   * $L_d$ 与 $L_q$ 的物理去耦以及 $I_d$ 的直流施加，均建立在转子电气坐标系（D/Q 轴）准确对齐的前提下；
   * 启动辨识前必须强制验证 `bElectricalZeroValid == true`（由 `motor align` 完成捕获）；未校准时拒绝启动。
2. **拒绝状态机膨胀**：
   * 状态压缩为 4 个宏观测试阶段：`RS_LOW` $\to$ `RS_HIGH` $\to$ `LD` $\to$ `LQ` $\to$ `COMPLETE`。
3. **确定性物理沉淀时间（400 拍等待 + 100 拍均值）**：
   * 考虑低压大扭矩及不同规格微型电机的电气时间常数 $\tau = L/R \approx 1\sim 2\text{ ms}$；
   * 必须保证至少 $10\tau$ 的充分衰减时间以达到严格的直流稳态；
   * 设定每个阻抗测试段：**等待 400 拍 (20 ms) $\to$ 累加 100 拍均值 (5 ms)**，全流程总耗时约 51.1 ms (20 kHz)。
4. **高精度时序与梯形伏秒积分**：
   * $L_d/L_q$ 脉冲期间（10 拍短脉冲防过流和磁饱和），严格对齐上一拍施加的线性区电压指令 $V[k-1]$ 与本拍电流响应 $I[k]$；
   * 采用梯形数值积分 $\sum [V - R \cdot (I_{k-1} + I_k)/2]$ 累积反电动势伏秒积，离散阶跃计算误差 $< 1.5 \times 10^{-6}$。
5. **宿主零胶水与全向保护**：
   * 矢量模长平方过流保护：$I_d^2 + I_q^2 > I_{limit}^2$；
   * 电压指令上限校验（禁止越出线性区 $0.577\text{ PU}$）；
   * `foc_app_t` 仅需接入 `foc_identify_t tIdentify`、命令缓冲及电压指令追踪。

---

## 2. 状态机与接口设计

### 2.1 状态枚举 (4 个测试阶段)
```c
typedef enum {
    FOC_IDENTIFY_STATUS_IDLE = 0,
    FOC_IDENTIFY_STATUS_RS_LOW,     /* 1. D轴低电压: 等待400拍 -> 累加100拍均值 */
    FOC_IDENTIFY_STATUS_RS_HIGH,    /* 2. D轴高电压: 等待400拍 -> 累加100拍均值 -> 算 Rs */
    FOC_IDENTIFY_STATUS_LD,         /* 3. D轴阶跃脉冲10拍 -> 梯形伏秒积分 -> 算 Ld */
    FOC_IDENTIFY_STATUS_LQ,         /* 4. Q轴阶跃脉冲10拍 -> 梯形伏秒积分 -> 算 Lq */
    FOC_IDENTIFY_STATUS_COMPLETE,   /* 成功完成 */
    FOC_IDENTIFY_STATUS_ERROR,      /* 异常报错 */
} foc_identify_status_e;
```

### 2.2 紧凑输入与输出 (D/Q 坐标系)
```c
typedef struct {
    foc_dq_t tCurrentDqPu;               /* 本拍实测 DQ 电流 (PU) */
    foc_dq_t tLastVoltageCommandDqPu;    /* 上一拍施加的线性区 DQ 电压指令 (PU) */
    foc_angle_t tMechanicalAngle;        /* 本拍机械角 (BAM32) */
    bool bValid;                         /* 输入有效 (编码器读取成功+对齐+运行) */
    bool bFault;                         /* 硬件/系统故障 (OCP/Break 或 MotorFault) */
} foc_identify_input_t;

typedef struct {
    foc_dq_t tVoltageRefPu;              /* 下一拍施加的 D/Q 电压参考 (PU) */
    bool bRefChanged;                    /* 参考电压是否改变 */
    bool bStopPwm;                       /* 是否请求停机 */
    foc_identify_status_e eStatus;       /* 当前阶段 */
} foc_identify_output_t;

typedef struct {
    foc_scalar_t qResistancePu;
    foc_scalar_t qInductanceDPu;
    foc_scalar_t qInductanceQPu;
} foc_identify_result_t;
```

### 2.3 Controller 结构体 (`foc_identify_t`)
```c
typedef struct {
    /* 1. 激励与限值配置 */
    foc_scalar_t qV_low;          /* Rs 低测试电压 (PU, 线性区内) */
    foc_scalar_t qV_high;         /* Rs 高测试电压 (PU, <= 0.577 PU) */
    foc_scalar_t qV_Ld;           /* Ld 阶跃脉冲电压 (PU) */
    foc_scalar_t qV_Lq;           /* Lq 阶跃脉冲电压 (PU) */
    foc_scalar_t qCurrentLimit;   /* 矢量模长限流保护 (PU) */
    foc_scalar_t qMinDeltaI;      /* 最小电流响应增量 (防除零) */
    foc_scalar_t qTurnsPerSample; /* f_base * Ts 基准常数 */
    foc_scalar_t qMaxDisplacement;/* 最大允许机械位移 (PU) */

    /* 2. 交互与结果交付 */
    foc_identify_result_t tResult;
    foc_identify_output_t tOutput;
    foc_result_t eFailure;
    foc_angle_t tZeroAngle;
    bool bZeroPrimed;

    /* 3. 通用工作累加器 (梯形积分与均值计算) */
    float fSumV;                  /* 电压/伏秒累加器 */
    float fSumI;                  /* 电流累加器 */
    float fI_start;               /* 阶跃初值电流 */
    float fI_last;                /* 上一拍电流 (用于梯形积分) */
    float fI_low;                 /* Rs 低段稳态均值 */
    float fV_low;                 /* Rs 低段实际电压均值 */
    uint16_t hwTicks;             /* 当前阶段拍数计数器 */
    bool bInitialized;
} foc_identify_t;
```

---

## 3. 算法与状态推进流程

### 3.1 核心流程（4 步直达终点）
1. **启动 (`foc_identify_Start`)**：
   * 状态置为 `FOC_IDENTIFY_STATUS_RS_LOW`，输出电压置为 `qV_low`；
   * 复位累加器 `fSumV=0, fSumI=0, hwTicks=0, bZeroPrimed=false`。
2. **第一阶段：`RS_LOW` (共 500 拍，25 ms)**：
   * 前 400 拍：仅等待，确保达到严格直流稳态（首拍自动将机械角存入 `tZeroAngle`）；
   * 后 100 拍：累加 $V_{d}$ 与 $I_d$；
   * 第 500 拍：保存 $V_{low} = fSumV/100, I_{low} = fSumI/100$，清空累加器；
   * 输出切换为 `qV_high`，切入 `RS_HIGH`。
3. **第二阶段：`RS_HIGH` (共 500 拍，25 ms)**：
   * 前 400 拍等待，后 100 拍累加；
   * 第 500 拍：
     $$\Delta V = \frac{fSumV}{100} - V_{low}, \quad \Delta I = \frac{fSumI}{100} - I_{low}$$
     $$R_{s\_pu} = \frac{\Delta V}{\Delta I}$$
   * 检查 $\Delta I > qMinDeltaI$，否则报错；
   * 输出切换为 $0\text{V}$，切入 `LD`。
4. **第三阶段：`LD` (共 11 拍，0.55 ms)**：
   * 第 0 拍：记录基准初值 $fI\_start = I_d, fI\_last = I_d$，输出施加脉冲 `qV_Ld`；
   * 第 1~10 拍：逐拍梯形积分累加伏秒积：
     $$fSumV \mathrel{+}= \left[V_d - R_{s\_pu} \cdot \frac{I_{k-1} + I_k}{2}\right]$$
   * 第 10 拍：
     $$\Delta I_d = I_d - fI\_start$$
     $$L_{d\_pu} = qTurnsPerSample \times \frac{fSumV}{\Delta I_d}$$
   * 校验 $\Delta I_d > qMinDeltaI$，否则报错；
   * 输出切换为 $0\text{V}$，切入 `LQ`。
5. **第四阶段：`LQ` (共 11 拍，0.55 ms)**：
   * 激励轴为 Q 轴，电压为保守的 `qV_Lq`，过程与 D 轴完全对称；
   * 第 10 拍算得 $L_{q\_pu}$；
   * 切入 `FOC_IDENTIFY_STATUS_COMPLETE`，输出请求停机 `bStopPwm = true`。

---

## 4. 安全防护线

每拍进入阶段逻辑前，执行 3 道安全防护：
1. **矢量电流超限检查**：$I_d^2 + I_q^2 > qCurrentLimit^2 \implies$ 触发过流停机；
2. **位移超限检查**：$|\text{Angle} - \text{ZeroAngle}| > qMaxDisplacement \implies$ 触发微动超限停机；
3. **系统/输入有效性**：`bFault == true` 或 `bValid == false` $\implies$ 触发故障停机。

---

## 5. 配置参数建议 (`foc/app/motor_config.h`)

* `IDENTIFY_V_LOW_PU`：$0.0125\text{ PU} \ (0.15\text{V} \implies \sim 0.3\text{A})$；
* `IDENTIFY_V_HIGH_PU`：$0.0333\text{ PU} \ (0.40\text{V} \implies \sim 0.8\text{A})$；
* `IDENTIFY_V_LD_PU` / `IDENTIFY_V_LQ_PU`：$0.0667\text{ PU} \ (0.80\text{V} \implies \Delta I \sim 0.4\text{A})$；
* `IDENTIFY_CURRENT_LIMIT_PU`：$0.20\text{ PU} \ (1.4\text{A})$；
* `IDENTIFY_MAX_MOVE_PU`：$0.002\text{ PU}$（约 5° 电角度以内）。
