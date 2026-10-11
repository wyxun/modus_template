/*******************************************************************************
 * @file    foc_numeric.h
 * @brief   Architecture-independent scalar backend for FOC algorithms
 *
 * ===== 数值后端 =====
 * 整个 FOC 框架通过 foc_scalar_t 抽象数值类型，通过一个编译宏选择后端：
 *
 *   FOC_NUMERIC_FLOAT —— float 后端，用于开发调试和 FPU 目标
 *   FOC_NUMERIC_FIXED —— int32_t Q15 定点后端，用于无 FPU 的 MCU
 *
 * 所有 FOC 算法（PID、滤波器、观测器、变换）都只使用 foc_scalar_t，
 * 切换后端只需修改编译宏，无需改动算法代码。
 *
 * ===== Q15 定点格式 =====
 * FOC_NUMERIC_FIXED 使用 int32_t，低 15 位为小数位：
 *
 *   stored = value * 32768，存储范围 [-65536, 65536)，步长 1/32768。
 *   这不是 16 位 Q1.15；常用 pu 上限由具体控制模块规定。
 *   初始化的有量纲 Hz 也可借此编码，但必须注明单位与换算边界。
 *   FOC_SCALAR 常量就近舍入（半值远离零），不检查/饱和溢出；
 *   动态输入使用 foc_from_float()，调用方先检查有限性和物理范围。
 *
 *   Q15 乘法缩放：a * b / 32768；中间乘积必须先加宽。
 *
 * ===== foc_gain_t 定点增益 =====
 * PID 增益用整数部分加小数部分表示，与历史配置格式兼容。
 * foc_gain_t 用 {整数部分, Q15 小数} 组合表示任意范围的增益，
 * 其取值范围同时受整数载体和小数部分校验约束。
 *
 *   gain = nInteger + qFraction / 32768
 *   例如 Kp=12.5 → {12, 16384}
 ******************************************************************************/

#ifndef FOC_NUMERIC_H
#define FOC_NUMERIC_H

#include <stdbool.h>
#include <stdint.h>

#if defined(FOC_NUMERIC_FLOAT) && defined(FOC_NUMERIC_FIXED)
#error "Select only one FOC numeric backend"
#elif defined(FOC_NUMERIC_FLOAT)
typedef float foc_scalar_t;
#elif defined(FOC_NUMERIC_FIXED)
typedef int32_t foc_scalar_t;
#else
#error "Select FOC_NUMERIC_FLOAT or FOC_NUMERIC_FIXED"
#endif

typedef enum {
    FOC_RESULT_OK = 0,
    FOC_RESULT_NULL,
    FOC_RESULT_INVALID_ARGUMENT,
    FOC_RESULT_OUT_OF_RANGE,
    FOC_RESULT_DIVIDE_BY_ZERO,
    FOC_RESULT_DISABLED,
    FOC_RESULT_SAFETY,
    FOC_RESULT_BUSY,
} foc_result_t;

typedef struct {
    int16_t nInteger;
    foc_scalar_t qFraction;
} foc_gain_t;

#define FOC_Q_FRACTION_BITS 15
#define FOC_Q_SCALE         32768

#if defined(FOC_NUMERIC_FLOAT)
#define FOC_SCALAR(value) ((foc_scalar_t)(value))
#define FOC_ZERO          ((foc_scalar_t)0.0f)
#define FOC_HALF          ((foc_scalar_t)0.5f)
#define FOC_ONE           ((foc_scalar_t)1.0f)
#define FOC_NEG_ONE       ((foc_scalar_t)-1.0f)
#else
#define FOC_SCALAR(value)                                                   \
    ((foc_scalar_t)(((value) >= 0.0f)                                      \
                        ? ((value) * (float)FOC_Q_SCALE + 0.5f)            \
                        : ((value) * (float)FOC_Q_SCALE - 0.5f)))
#define FOC_ZERO          ((foc_scalar_t)0)
#define FOC_HALF          ((foc_scalar_t)(FOC_Q_SCALE / 2))
#define FOC_ONE           ((foc_scalar_t)FOC_Q_SCALE)
#define FOC_NEG_ONE       ((foc_scalar_t)-FOC_Q_SCALE)
#endif

/**
 * @brief  检查标量是否为有限值（非 NaN 且非 Inf）
 * @param  qValue  标量值
 * @return true=有限合法值, false=NaN 或 Inf
 */
#if defined(FOC_NUMERIC_FLOAT)
#include <math.h>
static inline bool foc_scalar_is_finite(foc_scalar_t qValue)
{
    return isfinite(qValue) != 0;
}
#else
static inline bool foc_scalar_is_finite(foc_scalar_t qValue)
{
    (void)qValue;
    return true;
}
#endif

/**
 * @brief  将 float 转换为当前数值后端表示的定点标量
 * @param  fValue  浮点输入
 * @return         定点标量
 */
foc_scalar_t foc_from_float(float fValue);
/**
 * @brief  将定点标量转换为 float
 * @param  qValue  定点标量
 * @return         浮点值
 */
static inline float foc_to_float(foc_scalar_t qValue)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qValue;
#else
    return (float)qValue / (float)FOC_Q_SCALE;
#endif
}
/**
 * @brief  将值钳位到指定范围
 * @param  qValue   输入值
 * @param  qMinimum 下限
 * @param  qMaximum 上限
 * @return          钳位后的值
 */
static inline foc_scalar_t foc_sat(foc_scalar_t qValue,
                     foc_scalar_t qMinimum,
                     foc_scalar_t qMaximum)
{
    if (qValue < qMinimum) {
        return qMinimum;
    }
    if (qValue > qMaximum) {
        return qMaximum;
    }
    return qValue;
}
/**
 * @brief  加法：FIXED 在 int32 范围饱和，FLOAT 直接相加；不裁到 ±1 pu
 * @param  qA  加数 A
 * @param  qB  加数 B
 * @return     饱和和
 */
static inline foc_scalar_t foc_add_sat(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA + qB;
#else
    if (qB > 0 && qA > INT32_MAX - qB) {
        return INT32_MAX;
    }
    if (qB < 0 && qA < INT32_MIN - qB) {
        return INT32_MIN;
    }
    return qA + qB;
#endif
}
/**
 * @brief  减法：FIXED 在 int32 范围饱和，FLOAT 直接相减；不裁到 ±1 pu
 * @param  qA  被减数
 * @param  qB  减数
 * @return     饱和差
 */
static inline foc_scalar_t foc_sub_sat(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA - qB;
#else
    if (qB < 0 && qA > INT32_MAX + qB) {
        return INT32_MAX;
    }
    if (qB > 0 && qA < INT32_MIN + qB) {
        return INT32_MIN;
    }
    return qA - qB;
#endif
}
/**
 * @brief  PU 乘法；FIXED 先裁输入到 ±1，宽 PU 输入应使用 foc_mul_wide
 * @param  qA  乘数 A
 * @param  qB  乘数 B
 * @return     乘积
 */
static inline foc_scalar_t foc_mul_pu(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA * qB;
#else
    int32_t nProduct;
    uint32_t wMagnitude;
    bool bNegative;

    qA = foc_sat(qA, FOC_NEG_ONE, FOC_ONE);
    qB = foc_sat(qB, FOC_NEG_ONE, FOC_ONE);
    nProduct = qA * qB;
    bNegative = nProduct < 0;
    wMagnitude = bNegative ? (uint32_t)(-nProduct) : (uint32_t)nProduct;
    wMagnitude = (wMagnitude + (1u << (FOC_Q_FRACTION_BITS - 1))) >>
                 FOC_Q_FRACTION_BITS;

    return bNegative ? -(foc_scalar_t)wMagnitude : (foc_scalar_t)wMagnitude;
#endif
}
/**
 * @brief  定点乘法，使用全精度中间值（qA * qB / Q_SCALE）
 * @param  qA  乘数 A
 * @param  qB  乘数 B
 * @return     乘积
 */
static inline foc_scalar_t foc_mul_wide(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA * qB;
#else
    int64_t llProduct = (int64_t)qA * (int64_t)qB;
    llProduct /= FOC_Q_SCALE;
    if (llProduct > INT32_MAX) {
        return INT32_MAX;
    }
    if (llProduct < INT32_MIN) {
        return INT32_MIN;
    }
    return (foc_scalar_t)llProduct;
#endif
}
/**
 * @brief  带零除检查的定点除法
 * @param  qNumerator    被除数
 * @param  qDenominator  除数
 * @param  pqResult      输出商
 * @return FOC_RESULT_OK 或 FOC_RESULT_DIVIDE_BY_ZERO
 */
foc_result_t foc_div_checked(foc_scalar_t qNumerator,
                             foc_scalar_t qDenominator,
                             foc_scalar_t *pqResult);
/**
 * @brief  定点绝对值
 * @param  qValue  输入值
 * @return         绝对值
 */
static inline foc_scalar_t foc_abs(foc_scalar_t qValue)
{
#if defined(FOC_NUMERIC_FIXED)
    if (qValue == INT32_MIN) {
        return INT32_MAX;
    }
#endif
    return qValue < FOC_ZERO ? -qValue : qValue;
}
/**
 * @brief  从整数部分和小数部分初始化增益
 * @param  ptGain     输出增益
 * @param  nInteger   整数部分
 * @param  qFraction  小数部分（foc_scalar_t）
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_gain_Init(foc_gain_t *ptGain,
                           int16_t nInteger,
                           foc_scalar_t qFraction);
/**
 * @brief  从浮点数初始化增益
 * @param  fGain   浮点增益值
 * @param  ptGain  输出增益
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t foc_gain_from_float(float fGain, foc_gain_t *ptGain);
/**
 * @brief  从定点标量初始化增益
 * @param  qGain   定点标量
 * @param  ptGain  输出增益
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t foc_gain_from_scalar(foc_scalar_t qGain, foc_gain_t *ptGain);
/**
 * @brief  检查增益参数是否有效
 * @param  ptGain  增益参数指针
 * @return         true=有效, false=无效
 */
bool foc_gain_IsValid(const foc_gain_t *ptGain);
/**
 * @brief  将增益应用到值上（ptGain * qValue）
 * @param  ptGain  增益参数
 * @param  qValue  输入值
 * @return         输出值
 */
foc_scalar_t foc_gain_apply(const foc_gain_t *ptGain,
                            foc_scalar_t qValue);

#endif /* FOC_NUMERIC_H */
