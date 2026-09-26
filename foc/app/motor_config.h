/****************************************************************************
 * @file    motor_config.h
 * @brief   User-editable FOC motor and application profile.
 * @author  Codex
 * @date    2026-09-23
 ****************************************************************************/

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "foc_numeric.h"

/* User-editable motor and application profile. Board ADC scaling, DC-bus
 * conversion, trig backend and FOC_HF_ISR_HZ belong to the selected target. */

/* Motor nameplate and identified electrical parameters. */
#define MOTOR_CONFIG_POLE_PAIRS                            7U
#define MOTOR_CONFIG_RESISTANCE_MILLIOHM                   2740U
#define MOTOR_CONFIG_INDUCTANCE_D_MICROHENRY               800U
#define MOTOR_CONFIG_INDUCTANCE_Q_MICROHENRY               800U

/* Drive bases. HF frequency itself is supplied by the selected target. */
#define MOTOR_CONFIG_BASE_VOLTAGE_MILLIVOLT                12000U
#define MOTOR_CONFIG_BASE_ELECTRICAL_HZ                    100.0f

/* Position source: SENSOR or HARD_DRAG. Hard drag is disabled at zero Hz. */
#define MOTOR_CONFIG_POSITION_SOURCE                       \
    MOTOR_POSITION_SOURCE_SENSOR
#define MOTOR_CONFIG_HARD_DRAG_ELECTRICAL_MILLIHZ           0

/* Operating limits and current/speed controller tuning. */
#define MOTOR_CONFIG_MAX_SPEED_REFERENCE_PU                100.0f
#define MOTOR_CONFIG_MAX_PHASE_CURRENT_PU                  1.0f
#define MOTOR_CONFIG_MAX_MODULATION_PU                     0.5773502692f
#define MOTOR_CONFIG_PI_GAIN_INTEGER                       0
#define MOTOR_CONFIG_CURRENT_PI_KP_PU                      0.20f
#define MOTOR_CONFIG_CURRENT_PI_KI_TS_PU                   0.005f
#define MOTOR_CONFIG_CURRENT_PI_KD_OVER_TS_PU              0.0f
#define MOTOR_CONFIG_CURRENT_PI_OUTPUT_MIN_PU              -0.55f
#define MOTOR_CONFIG_CURRENT_PI_OUTPUT_MAX_PU              0.55f
#define MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MIN_PU          -0.50f
#define MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MAX_PU          0.50f
#define MOTOR_CONFIG_SPEED_PI_KP_PU                        0.20f
#define MOTOR_CONFIG_SPEED_PI_KI_TS_PU                     0.005f
#define MOTOR_CONFIG_SPEED_PI_KD_OVER_TS_PU                0.0f
#define MOTOR_CONFIG_SPEED_PI_OUTPUT_MIN_PU                -0.10f
#define MOTOR_CONFIG_SPEED_PI_OUTPUT_MAX_PU                0.10f
#define MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MIN_PU            -0.10f
#define MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MAX_PU            0.10f
#define MOTOR_CONFIG_ADC_CALIBRATION_TIMEOUT_SECONDS       0.1f
#define MOTOR_CONFIG_ALIGN_TIME_SECONDS                    1.5f 
#define MOTOR_CONFIG_SPEED_LOOP_FREQUENCY_HZ               1000U
#define MOTOR_CONFIG_ALIGN_CURRENT_PU                      0.005f

/* Position sensor setup; used only when FOC_PORT_HAS_POSITION is enabled. */
#define MOTOR_CONFIG_ENCODER_SPEED_FILTER_ALPHA            0.25f
#define MOTOR_CONFIG_ENCODER_INVALID_TIMEOUT_SECONDS       0.005f
#define MOTOR_CONFIG_ENCODER_DIRECTION_INVERT              false

/* Optional observer tuning. */
#define MOTOR_CONFIG_SMO_BEMF_CUTOFF_RADIANS_PER_SECOND    4000U
#define MOTOR_CONFIG_SMO_SLIDING_GAIN_MILLIVOLT            5000U
#define MOTOR_CONFIG_SMO_CURRENT_ESTIMATE_LIMIT_PU         1.0f

/* Debug identification routine settings. */
#define MOTOR_CONFIG_IDENTIFY_LD_FREQUENCY_HZ              1000U
#define MOTOR_CONFIG_IDENTIFY_LD_CAPTURE_DELAY             1U
#define MOTOR_CONFIG_IDENTIFY_LD_CAPTURE_SAMPLES           4U
#define MOTOR_CONFIG_IDENTIFY_LD_HALF_CYCLES               4U
#define MOTOR_CONFIG_IDENTIFY_LD_MODULATION_PU             0.10f
#define MOTOR_CONFIG_IDENTIFY_LD_MAX_CURRENT_PU            0.02f
#define MOTOR_CONFIG_IDENTIFY_LD_MIN_DELTA_PU              0.001f
#define MOTOR_CONFIG_IDENTIFY_LD_MAX_SPEED_PU              0.01f
#define MOTOR_CONFIG_IDENTIFY_LD_MOTION_CYCLES             1U

#endif /* MOTOR_CONFIG_H */
