/*******************************************************************************
 * @file    foc_dob.h
 * @brief   Multi-instance normalized disturbance observer
 ******************************************************************************/

#ifndef FOC_DOB_H
#define FOC_DOB_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tTorqueGain;         /**< 力矩增益 */
    foc_gain_t tModelGain;          /**< 模型增益 */
    foc_gain_t tK1;                 /**< 观测器增益 K1 */
    foc_gain_t tK2Ts;               /**< 观测器增益 K2 * Ts */
    foc_gain_t tBoundaryInverse;    /**< 边界层倒数 */
    foc_scalar_t qDisturbanceMinimum;   /**< 干扰量下限 */
    foc_scalar_t qDisturbanceMaximum;   /**< 干扰量上限 */
    foc_scalar_t qSpeedMinimum;         /**< 速度下限 */
    foc_scalar_t qSpeedMaximum;         /**< 速度上限 */
} foc_dob_params_t;

typedef struct {
    foc_scalar_t qEstimatedSpeed;   /**< 估计速度 */
    foc_scalar_t qDisturbance;      /**< 估计干扰量 */
} foc_dob_output_t;

typedef struct {
    foc_dob_params_t tParams;       /**< DOB 参数 */
    foc_scalar_t qEstimatedSpeed;   /**< 状态：估计速度 */
    foc_scalar_t qDisturbance;      /**< 状态：估计干扰量 */
} foc_dob_t;

/**
 * @brief  初始化干扰观测器实例
 * @param  ptDob    观测器实例指针
 * @param  ptParams 参数（力矩增益、模型增益、K1、K2Ts、边界倒数等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_dob_Init(foc_dob_t *ptDob,
                          const foc_dob_params_t *ptParams);
/**
 * @brief  复位干扰观测器状态
 * @param  ptDob  观测器实例指针
 */
void foc_dob_Reset(foc_dob_t *ptDob);
/**
 * @brief  执行一步干扰观测器运算
 * @param  ptDob          观测器实例指针
 * @param  qCurrentQ      Q 轴电流
 * @param  qMeasuredSpeed 实测速度
 * @param  ptOutput       输出（估计速度 + 干扰量）
 * @return                FOC_RESULT_OK 或错误码
 */
foc_result_t foc_dob_Step(foc_dob_t *ptDob,
                          foc_scalar_t qCurrentQ,
                          foc_scalar_t qMeasuredSpeed,
                          foc_dob_output_t *ptOutput);

#endif /* FOC_DOB_H */
