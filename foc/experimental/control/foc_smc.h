/*******************************************************************************
 * @file    foc_smc.h
 * @brief   Multi-instance sliding-mode controller
 ******************************************************************************/

#ifndef FOC_SMC_H
#define FOC_SMC_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tDerivative;         /**< 微分增益 */
    foc_gain_t tSurface;            /**< 滑模面增益 */
    foc_gain_t tDiscontinuous;      /**< 不连续控制增益 */
    foc_gain_t tReach;              /**< 趋近律增益 */
    foc_gain_t tPlant;              /**< 被控对象增益 */
    foc_gain_t tIntegrator;         /**< 积分增益 */
    foc_scalar_t qOutputMinimum;    /**< 输出下限 */
    foc_scalar_t qOutputMaximum;    /**< 输出上限 */
} foc_smc_params_t;

typedef struct {
    foc_smc_params_t tParams;       /**< SMC 参数 */
    foc_scalar_t qPreviousError;    /**< 状态：上次误差 */
    foc_scalar_t qIntegrator;       /**< 状态：积分器 */
} foc_smc_t;

/**
 * @brief  初始化滑模控制器实例
 * @param  ptSmc    SMC 实例指针
 * @param  ptParams 参数（微分、滑模面、不连续增益、趋近律等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_smc_Init(foc_smc_t *ptSmc,
                          const foc_smc_params_t *ptParams);
/**
 * @brief  复位滑模控制器状态
 * @param  ptSmc  SMC 实例指针
 */
void foc_smc_Reset(foc_smc_t *ptSmc);
/**
 * @brief  执行一步滑模控制运算
 * @param  ptSmc      SMC 实例指针
 * @param  qReference 参考值
 * @param  qFeedback  反馈值
 * @return            控制输出
 */
foc_scalar_t foc_smc_Step(foc_smc_t *ptSmc,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);

#endif /* FOC_SMC_H */
