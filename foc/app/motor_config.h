/****************************************************************************
 * @file    motor_config.h
 * @brief   STM32G431 motor physical deployment profile and PU constants.
 * @author  Antigravity
 * @date    2026-09-15
 ****************************************************************************/

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "foc_numeric.h"

/* ==================== 1. Physical Nameplate & Bench Constants ============= */
#define MOTOR_NOMINAL_VOLTAGE_V              12.0f
#define MOTOR_NOMINAL_CURRENT_A              1.0f
#define MOTOR_POLE_PAIRS                     7U
#define MOTOR_RESISTANCE_MILLIOHM            500U
#define MOTOR_INDUCTANCE_D_MICROHENRY        1000U
#define MOTOR_INDUCTANCE_Q_MICROHENRY        1000U

#define MOTOR_BASE_VOLTAGE_MV                12000U
#define MOTOR_BASE_CURRENT_MA                7000U
#define MOTOR_BASE_ELECTRICAL_HZ             100.0f
#define MOTOR_HF_PERIOD_NANOSECONDS          50000U
#define MOTOR_PWM_FREQ_HZ                    20000U

/* ==================== 2. Derived PU Identification Constants ============== */
#define IDENTIFY_V_LOW_PU                    FOC_SCALAR(0.0100f)
#define IDENTIFY_V_HIGH_PU                   FOC_SCALAR(0.0500f)
#define IDENTIFY_V_LD_PU                     FOC_SCALAR(0.0667f)
#define IDENTIFY_V_LQ_PU                     FOC_SCALAR(0.0667f)
#define IDENTIFY_CURRENT_LIMIT_PU            FOC_SCALAR(0.20f)
#define IDENTIFY_MIN_DELTA_I_PU              FOC_SCALAR(0.005f)
/* 角频率采样步长：Lpu = 2*pi*f_base*Ts * sum(Vpu-Rpu*Ipu)/dIpu。
   Lbase = Zbase/(2*pi*f_base) 使用 rad/s，故步长必须是 rad/sample，
   否则辨识出的 Ld/Lq 会小 2*pi 倍。 */
#define IDENTIFY_RADIANS_PER_SAMPLE          FOC_SCALAR( \
    (2.0f * 3.14159265358979f * MOTOR_BASE_ELECTRICAL_HZ) / \
    (float)MOTOR_PWM_FREQ_HZ)
#define IDENTIFY_MAX_DISPLACEMENT_PU         FOC_SCALAR(0.002f)

#endif /* MOTOR_CONFIG_H */
