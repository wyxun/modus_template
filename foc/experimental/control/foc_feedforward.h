/*******************************************************************************
 * @file    foc_feedforward.h
 * @brief   PMSM current-loop decoupling feedforward
 ******************************************************************************/

#ifndef FOC_FEEDFORWARD_H
#define FOC_FEEDFORWARD_H

#include "foc_core.h"

typedef struct {
    foc_gain_t tLq;             /**< Q 轴电感 */
    foc_gain_t tLd;             /**< D 轴电感 */
    foc_gain_t tFlux;           /**< 永磁磁链 */
    foc_scalar_t qOutputLimit;  /**< 输出限幅 */
} foc_feedforward_params_t;

typedef struct {
    foc_feedforward_params_t tParams;   /**< 前馈参数 */
} foc_feedforward_t;

/**
 * @brief  初始化前馈解耦实例
 * @param  ptFeedforward  前馈实例指针
 * @param  ptParams       参数（Lq、Ld、磁链、输出限幅）
 * @return                FOC_RESULT_OK 或错误码
 */
foc_result_t foc_feedforward_Init(
    foc_feedforward_t *ptFeedforward,
    const foc_feedforward_params_t *ptParams);
/**
 * @brief  PMSM 电流环前馈解耦，计算 dq 轴前馈电压
 * @param  ptFeedforward   前馈实例指针
 * @param  qElectricalSpeed 电角速度
 * @param  qId             D 轴电流
 * @param  qIq             Q 轴电流
 * @param  ptVoltage       输出 dq 前馈电压
 * @return                 FOC_RESULT_OK 或错误码
 */
foc_result_t foc_feedforward_Pmsm(
    const foc_feedforward_t *ptFeedforward,
    foc_scalar_t qElectricalSpeed,
    foc_scalar_t qId,
    foc_scalar_t qIq,
    foc_dq_t *ptVoltage);

#endif /* FOC_FEEDFORWARD_H */
