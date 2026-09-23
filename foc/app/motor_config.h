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
#define MOTOR_NOMINAL_CURRENT_A              0.5f
#define MOTOR_POLE_PAIRS                     7U
#define MOTOR_RESISTANCE_MILLIOHM            2740U
#define MOTOR_INDUCTANCE_D_MICROHENRY        1000U
#define MOTOR_INDUCTANCE_Q_MICROHENRY        1000U

#define MOTOR_IDENTIFY_LD_FREQUENCY_HZ       1000U
#define MOTOR_IDENTIFY_LD_CAPTURE_DELAY      1U
#define MOTOR_IDENTIFY_LD_CAPTURE_SAMPLES    4U
#define MOTOR_IDENTIFY_LD_HALF_CYCLES        4U
#define MOTOR_IDENTIFY_LD_MODULATION_PU      0.10f
#define MOTOR_IDENTIFY_LD_MAX_CURRENT_PU     0.15f
#define MOTOR_IDENTIFY_LD_MIN_DELTA_PU       0.01f
#define MOTOR_IDENTIFY_LD_MAX_SPEED_PU       0.01f
#define MOTOR_IDENTIFY_LD_MOTION_CYCLES      1U

#define MOTOR_BASE_VOLTAGE_MV                12000U
#define MOTOR_BASE_CURRENT_MA                3500U
#define MOTOR_BASE_ELECTRICAL_HZ             100.0f
#define MOTOR_HF_PERIOD_NANOSECONDS          \
    (1000000000U / FOC_HF_ISR_HZ)

#endif /* MOTOR_CONFIG_H */
