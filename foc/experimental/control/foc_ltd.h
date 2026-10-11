/*******************************************************************************
 * @file    foc_ltd.h
 * @brief   Multi-instance linear tracking differentiator
 ******************************************************************************/

#ifndef FOC_LTD_H
#define FOC_LTD_H

#include "foc_numeric.h"

typedef struct {
    foc_scalar_t qMaximumVelocity;      /**< 最大速度 */
    foc_scalar_t qMaximumAcceleration;  /**< 最大加速度 */
} foc_ltd_params_t;

typedef struct {
    foc_ltd_params_t tParams;   /**< LTD 参数 */
    foc_scalar_t qPosition;     /**< 状态：跟踪位置 */
    foc_scalar_t qVelocity;     /**< 状态：跟踪速度 */
} foc_ltd_t;

/**
 * @brief  初始化线性跟踪微分器实例
 * @param  ptLtd             LTD 实例指针
 * @param  ptParams         参数（最大速度、最大加速度）
 * @param  qInitialPosition 初始位置
 * @return                  FOC_RESULT_OK 或错误码
 */
foc_result_t foc_ltd_Init(foc_ltd_t *ptLtd,
                          const foc_ltd_params_t *ptParams,
                          foc_scalar_t qInitialPosition);
/**
 * @brief  复位线性跟踪微分器状态
 * @param  ptLtd     LTD 实例指针
 * @param  qPosition 复位后的位置值
 */
void foc_ltd_Reset(foc_ltd_t *ptLtd, foc_scalar_t qPosition);
/**
 * @brief  执行一步 LTD 跟踪，输出跟踪后的位置
 * @param  ptLtd   LTD 实例指针
 * @param  qTarget 目标位置
 * @return         跟踪后的位置
 */
foc_scalar_t foc_ltd_Step(foc_ltd_t *ptLtd, foc_scalar_t qTarget);

#endif /* FOC_LTD_H */
