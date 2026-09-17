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

#endif /* MOTOR_CONFIG_H */
