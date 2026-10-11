/*******************************************************************************
 * @file    foc_ladrc.h
 * @brief   Multi-instance discrete linear active disturbance rejection control
 ******************************************************************************/

#ifndef FOC_LADRC_H
#define FOC_LADRC_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tTrackingPosition;       /**< 跟踪器位置增益 */
    foc_gain_t tTrackingVelocity;       /**< 跟踪器速度增益 */
    foc_gain_t tTrackingIntegrator;     /**< 跟踪器积分增益 */
    foc_gain_t tObserverBeta1;          /**< 扩张状态观测器 β1 */
    foc_gain_t tObserverBeta2;          /**< 扩张状态观测器 β2 */
    foc_gain_t tObserverBeta3;          /**< 扩张状态观测器 β3 */
    foc_gain_t tPlantGain;              /**< 被控对象增益 b0 */
    foc_gain_t tKp;                     /**< 状态误差反馈 Kp */
    foc_gain_t tKd;                     /**< 状态误差反馈 Kd */
    foc_gain_t tPlantInverse;           /**< 1/b0 */
    foc_scalar_t qOutputMinimum;        /**< 输出下限 */
    foc_scalar_t qOutputMaximum;        /**< 输出上限 */
} foc_ladrc_params_t;

typedef struct {
    foc_ladrc_params_t tParams;             /**< LADRC 参数 */
    foc_scalar_t qTrackingPosition;         /**< 状态：跟踪位置 */
    foc_scalar_t qTrackingVelocity;         /**< 状态：跟踪速度 */
    foc_scalar_t qObserverPosition;         /**< 状态：ESO 位置估计 */
    foc_scalar_t qObserverVelocity;         /**< 状态：ESO 速度估计 */
    foc_scalar_t qObserverDisturbance;      /**< 状态：ESO 总扰动估计 */
    foc_scalar_t qOutput;                   /**< 状态：当前输出 */
} foc_ladrc_t;

/**
 * @brief  初始化 LADRC 实例
 * @param  ptLadrc   LADRC 实例指针
 * @param  ptParams  参数（跟踪系数、观测器增益、Kp/Kd、输出限幅等）
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_ladrc_Init(foc_ladrc_t *ptLadrc,
                            const foc_ladrc_params_t *ptParams);
/**
 * @brief  复位 LADRC 状态（跟踪器、观测器、输出）
 * @param  ptLadrc  LADRC 实例指针
 */
void foc_ladrc_Reset(foc_ladrc_t *ptLadrc);
/**
 * @brief  无条件跟踪：用给定输出、参考和反馈更新 LADRC 状态
 *         用于无扰切换场景
 * @param  ptLadrc    LADRC 实例指针
 * @param  qOutput    当前输出
 * @param  qReference 参考值
 * @param  qFeedback  反馈值
 */
void foc_ladrc_Track(foc_ladrc_t *ptLadrc,
                     foc_scalar_t qOutput,
                     foc_scalar_t qReference,
                     foc_scalar_t qFeedback);
/**
 * @brief  执行一步 LADRC 控制运算
 * @param  ptLadrc    LADRC 实例指针
 * @param  qReference 参考值
 * @param  qFeedback  反馈值
 * @return            控制输出
 */
foc_scalar_t foc_ladrc_Step(foc_ladrc_t *ptLadrc,
                            foc_scalar_t qReference,
                            foc_scalar_t qFeedback);
#endif /* FOC_LADRC_H */
