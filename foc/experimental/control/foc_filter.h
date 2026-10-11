/*******************************************************************************
 * @file    foc_filter.h
 * @brief   Multi-instance first-order and biquad filters
 ******************************************************************************/

#ifndef FOC_FILTER_H
#define FOC_FILTER_H

#include "foc_numeric.h"

typedef struct {
    foc_scalar_t qAlpha;    /**< 滤波系数（0 ~ 1） */
    foc_scalar_t qState;    /**< 状态：滤波器输出 */
} foc_lpf1_t;

typedef struct {
    foc_gain_t tB0;         /**< 分子系数 B0 */
    foc_gain_t tB1;         /**< 分子系数 B1 */
    foc_gain_t tB2;         /**< 分子系数 B2 */
    foc_gain_t tA1;         /**< 分母系数 A1 */
    foc_gain_t tA2;         /**< 分母系数 A2 */
} foc_biquad_coeffs_t;

typedef struct {
    foc_biquad_coeffs_t tCoefficients;  /**< 双二阶系数 */
    foc_scalar_t qState1;               /**< 状态：延迟单元 1 */
    foc_scalar_t qState2;               /**< 状态：延迟单元 2 */
} foc_biquad_t;

typedef enum {
    FOC_FILTER_BUTTERWORTH = 0,
    FOC_FILTER_CHEBYSHEV_I_0P1_DB,
    FOC_FILTER_BESSEL,
} foc_filter_response_t;

/**
 * @brief  初始化一阶低通滤波器
 * @param  ptFilter      滤波器实例指针
 * @param  qAlpha        滤波系数（0 ~ 1）
 * @param  qInitialValue 初始状态值
 * @return               FOC_RESULT_OK 或错误码
 */
foc_result_t foc_lpf1_Init(foc_lpf1_t *ptFilter,
                           foc_scalar_t qAlpha,
                           foc_scalar_t qInitialValue);
/**
 * @brief  复位一阶低通滤波器状态
 * @param  ptFilter  滤波器实例指针
 * @param  qValue    复位后的状态值
 */
void foc_lpf1_Reset(foc_lpf1_t *ptFilter, foc_scalar_t qValue);
/**
 * @brief  执行一步一阶低通滤波
 * @param  ptFilter  滤波器实例指针
 * @param  qInput    输入值
 * @return           滤波后的输出
 */
foc_scalar_t foc_lpf1_Step(foc_lpf1_t *ptFilter,
                           foc_scalar_t qInput);

/**
 * @brief  从系数表初始化双二阶滤波器
 * @param  ptFilter       滤波器实例指针
 * @param  ptCoefficients 滤波器系数
 * @return                FOC_RESULT_OK 或错误码
 */
foc_result_t foc_biquad_Init(foc_biquad_t *ptFilter,
                             const foc_biquad_coeffs_t *ptCoefficients);
/**
 * @brief  按响应类型和截止频率初始化低通双二阶滤波器
 * @param  ptFilter           滤波器实例指针
 * @param  eResponse          响应类型（Butterworth / Chebyshev / Bessel）
 * @param  wSampleFrequencyHz 采样频率，单位 Hz
 * @param  wCutoffFrequencyHz 截止频率，单位 Hz
 * @return                    FOC_RESULT_OK 或错误码
 */
foc_result_t foc_biquad_LowPassInit(foc_biquad_t *ptFilter,
                                    foc_filter_response_t eResponse,
                                    uint32_t wSampleFrequencyHz,
                                    uint32_t wCutoffFrequencyHz);
/**
 * @brief  复位双二阶滤波器状态
 * @param  ptFilter  滤波器实例指针
 */
void foc_biquad_Reset(foc_biquad_t *ptFilter);
/**
 * @brief  执行一步双二阶滤波
 * @param  ptFilter  滤波器实例指针
 * @param  qInput    输入值
 * @return           滤波后的输出
 */
foc_scalar_t foc_biquad_Step(foc_biquad_t *ptFilter,
                             foc_scalar_t qInput);

#endif /* FOC_FILTER_H */
