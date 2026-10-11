/*******************************************************************************
 * @file    foc_pll.h
 * @brief   Multi-instance normalized-angle phase-locked loop
 ******************************************************************************/

#ifndef FOC_PLL_H
#define FOC_PLL_H

#include "foc_angle.h"
#include "foc_pid.h"

typedef struct {
    foc_gain_t tKp;             /**< 比例增益 */
    foc_gain_t tKi;             /**< 积分增益 */
    foc_scalar_t qMaximumSpeed; /**< 最大跟踪速度 */
    foc_scalar_t qLockError;    /**< 锁定角度误差阈值 */
    foc_scalar_t qUnlockError;  /**< 解锁角度误差阈值 */
    uint16_t hwLockSamples;     /**< 锁定确认的连续样本数 */
} foc_pll_params_t;

typedef struct {
    foc_pll_params_t tParams;   /**< PLL 参数 */
    foc_pid_t tLoopFilter;      /**< 环路滤波器（PID） */
    foc_angle_t tAngle;         /**< 状态：锁定角度输出 */
    foc_scalar_t qSpeed;        /**< 状态：速度输出 */
    uint16_t hwLockCounter;     /**< 锁定计数器 */
    bool bIsLocked;             /**< 是否已锁定 */
} foc_pll_t;

/**
 * @brief  初始化 PLL 角度跟踪器实例
 * @param  ptPll    PLL 实例指针
 * @param  ptParams 参数（Kp、Ki、最大速度、锁定/解锁误差阈值等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_pll_Init(foc_pll_t *ptPll,
                          const foc_pll_params_t *ptParams);
/**
 * @brief  复位 PLL 角度和状态
 * @param  ptPll         PLL 实例指针
 * @param  tInitialAngle 复位后的初始角度
 */
void foc_pll_Reset(foc_pll_t *ptPll, foc_angle_t tInitialAngle);
/**
 * @brief  执行一步 PLL 角度跟踪
 * @param  ptPll           PLL 实例指针
 * @param  tMeasuredAngle  实测角度输入
 * @return                 FOC_RESULT_OK 或错误码
 */
foc_result_t foc_pll_Step(foc_pll_t *ptPll,
                          foc_angle_t tMeasuredAngle);

#endif /* FOC_PLL_H */
