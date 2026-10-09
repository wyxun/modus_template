/*******************************************************************************
 * @file    foc_modulation.h
 * @brief   Architecture-independent three-phase modulation
 *
 * ===== 调制方式 =====
 * 提供三种调制策略，在电机控制中可通过 motor_control_config_t 选择：
 *
 * SVPWM（Space Vector PWM）
 *   电压利用率最高（直流母线电压的 √3/2 ≈ 86.7%），谐波小，
 *   是 FOC 中最常用的调制方式。将 αβ 电压矢量映射到 6 个
 *   非零矢量和 2 个零矢量的组合，通过相邻矢量占空比合成目标矢量。
 *
 * SPWM（Sinusoidal PWM）
 *   最基础的正弦调制，直接用三相正弦波与载波比较产生 PWM。
 *   电压利用率低（约 50%），波形质量好。
 *
 * 三次谐波注入 SPWM
 *   在 SPWM 基础上注入 1/6 幅值的 3 次谐波，可将电压利用率
 *   提升到与 SVPWM 相同的水平（86.7%），实现上与 SVPWM 等价。
 *
 * ===== 占空比 foc_duty_abc_t =====
 *   范围 [0, 1]（pu），对应 0%~100% 占空比。
 *   0.5 为互补 PWM 的中点（输出 0V）。
 ******************************************************************************/

#ifndef FOC_MODULATION_H
#define FOC_MODULATION_H

#include "foc_types.h"

/**
 * @brief Ideal stationary voltage from an effective PWM interval and bus PU.
 * @param ptDuty Prior interval duty ratios, each in [0,1].
 * @param qBusPu Measured bus voltage / shared voltage base, may exceed 1 PU.
 * @param ptVoltage Output model voltage PU, excluding deadtime/device drops.
 * @return Checked model status; caller owns interval/sample alignment.
 */
foc_result_t foc_modulation_ModelVoltageAlphaBeta(const foc_duty_abc_t *ptDuty,
    foc_scalar_t qBusPu, foc_ab_t *ptVoltage);

/**
 * @brief  SVPWM 调制，将 αβ 电压转换为三相占空比
 * @param  ptVoltage  αβ 轴电压输入
 * @param  ptDuty     输出的三相占空比（U/V/W）
 * @param  pbSaturated 可选：本拍占空比达到或超出 [0, 1] 边界
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_svpwm(const foc_ab_t *ptVoltage,
                       foc_duty_abc_t *ptDuty, bool *pbSaturated);
/**
 * @brief  SPWM 调制，将 αβ 电压转换为三相占空比
 * @param  ptVoltage  αβ 轴电压输入
 * @param  ptDuty     输出的三相占空比（U/V/W）
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_spwm(const foc_ab_t *ptVoltage,
                      foc_duty_abc_t *ptDuty);
/**
 * @brief  三次谐波注入 SPWM 调制
 * @param  ptVoltage  αβ 轴电压输入
 * @param  ptDuty     输出的三相占空比（U/V/W）
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_third_harmonic_spwm(const foc_ab_t *ptVoltage,
                                     foc_duty_abc_t *ptDuty);

#endif /* FOC_MODULATION_H */
