/*******************************************************************************
 * @file    foc_sta.h
 * @brief   Multi-instance super-twisting controller
 ******************************************************************************/

#ifndef FOC_STA_H
#define FOC_STA_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tK1;                 /**< 超螺旋增益 K1 */
    foc_gain_t tK2Ts;               /**< 超螺旋增益 K2 * Ts */
    foc_gain_t tBoundaryInverse;    /**< 边界层倒数 */
    foc_scalar_t qIntegratorMinimum;    /**< 积分器下限 */
    foc_scalar_t qIntegratorMaximum;    /**< 积分器上限 */
    foc_scalar_t qOutputMinimum;        /**< 输出下限 */
    foc_scalar_t qOutputMaximum;        /**< 输出上限 */
} foc_sta_params_t;

typedef struct {
    foc_sta_params_t tParams;       /**< STA 参数 */
    foc_scalar_t qIntegrator;       /**< 状态：积分器 */
} foc_sta_t;

/**
 * @brief  初始化超螺旋控制器实例
 * @param  ptSta    STA 实例指针
 * @param  ptParams 参数（K1、K2Ts、边界倒数、积分器/输出限幅）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_sta_Init(foc_sta_t *ptSta,
                          const foc_sta_params_t *ptParams);
/**
 * @brief  复位超螺旋控制器状态
 * @param  ptSta  STA 实例指针
 */
void foc_sta_Reset(foc_sta_t *ptSta);
/**
 * @brief  执行一步超螺旋控制运算
 * @param  ptSta      STA 实例指针
 * @param  qReference 参考值
 * @param  qFeedback  反馈值
 * @return            控制输出
 */
foc_scalar_t foc_sta_Step(foc_sta_t *ptSta,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);

#endif /* FOC_STA_H */
