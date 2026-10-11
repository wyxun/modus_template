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
#define MOTOR_CONFIG_RESISTANCE_MILLIOHM                   4000U
#define MOTOR_CONFIG_INDUCTANCE_D_MICROHENRY               1200U
#define MOTOR_CONFIG_INDUCTANCE_Q_MICROHENRY               1200U

/* Drive bases. HF frequency itself is supplied by the selected target. */
#define MOTOR_CONFIG_BASE_VOLTAGE_MILLIVOLT                12000U
#define MOTOR_CONFIG_BASE_ELECTRICAL_HZ                    400.0f

/* Position source: SENSOR or HARD_DRAG. */
#define MOTOR_CONFIG_POSITION_SOURCE    MOTOR_POSITION_SOURCE_HARD_DRAG

/* Fixed-frequency current/voltage test; speed uses the startup ramp below. */
#define MOTOR_CONFIG_HARD_DRAG_ELECTRICAL_MILLIHZ           20000

/* 20 kHz: 2 s I/f ramp, at most 10 s open-loop run after alignment.
 * Automatic SMO handoff remains disabled during hard-drag testing. */
#define MOTOR_CONFIG_STARTUP_RAMP_STEPS                     40000U
#define MOTOR_CONFIG_STARTUP_MAX_RUN_STEPS                  200000U
#define MOTOR_CONFIG_STARTUP_IQ_PU                          0.06f
#define MOTOR_CONFIG_STARTUP_CLOSED_SPEED_STEP_PU           0.0005f
#define MOTOR_CONFIG_OBSERVER_TAKEOVER                      false
#define MOTOR_CONFIG_OBSERVER_AUTO_TAKEOVER                 false
#define MOTOR_CONFIG_OBSERVER_QUALIFICATION_STEPS           4000U
#define MOTOR_CONFIG_OBSERVER_BLEND_STEPS                   400U
#define MOTOR_CONFIG_OBSERVER_MIN_BEMF_PU                   0.13f
#define MOTOR_CONFIG_OBSERVER_MIN_SPEED_PU                  0.075f
#define MOTOR_CONFIG_OBSERVER_MAX_SPEED_ERROR_RATIO         0.10f
#define MOTOR_CONFIG_OBSERVER_MAX_ANGLE_ERROR_TURNS         0.125f

/* Operating limits and current/speed controller tuning. */
/* Physical electrical Hz; App binding converts this limit to speed pu. */
#define MOTOR_CONFIG_MAX_SPEED_ELECTRICAL_HZ                303.333333f
#define MOTOR_CONFIG_MAX_PHASE_CURRENT_PU                  0.14f
#define MOTOR_CONFIG_MAX_MODULATION_PU                     0.5773502692f
#define MOTOR_CONFIG_PI_GAIN_INTEGER                       0
/* 12 V, 3.5 A base: 300 Hz current-loop model, 50 us control period. */
#define MOTOR_CONFIG_CURRENT_PI_KP_PU                      0.472810f
#define MOTOR_CONFIG_CURRENT_PI_KI_TS_PU                   0.070097f
#define MOTOR_CONFIG_CURRENT_PI_KD_OVER_TS_PU              0.0f
#define MOTOR_CONFIG_CURRENT_PI_OUTPUT_MIN_PU              -0.55f
#define MOTOR_CONFIG_CURRENT_PI_OUTPUT_MAX_PU              0.55f
#define MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MIN_PU          -0.50f
#define MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MAX_PU          0.50f
/* Gains per electrical Hz; App binding multiplies by the speed base.
 * Bare 2804 rotor: J=3.7e-6 kg*m^2, 5 Hz natural frequency, damping=1. */
#define MOTOR_CONFIG_SPEED_PI_KP_PER_ELECTRICAL_HZ           0.001622326f
#define MOTOR_CONFIG_SPEED_PI_KI_TS_PER_ELECTRICAL_HZ        0.00002548343f
#define MOTOR_CONFIG_SPEED_PI_KD_OVER_TS_PER_ELECTRICAL_HZ   0.0f
#define MOTOR_CONFIG_SPEED_PI_OUTPUT_MIN_PU                -0.10f
#define MOTOR_CONFIG_SPEED_PI_OUTPUT_MAX_PU                0.10f
#define MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MIN_PU            -0.10f
#define MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MAX_PU            0.10f
#define MOTOR_CONFIG_ADC_CALIBRATION_TIMEOUT_SECONDS       0.1f
#define MOTOR_CONFIG_ALIGN_TIME_SECONDS                    1.5f 
#define MOTOR_CONFIG_SPEED_LOOP_FREQUENCY_HZ               1000U
#define MOTOR_CONFIG_ALIGN_CURRENT_PU                      0.04f

/* Position sensor setup; used only when FOC_PORT_HAS_POSITION is enabled. */
#define MOTOR_CONFIG_ENCODER_SPEED_FILTER_ALPHA            0.25f
#define MOTOR_CONFIG_ENCODER_INVALID_TIMEOUT_SECONDS       0.005f
#define MOTOR_CONFIG_ENCODER_DIRECTION_INVERT              false

/* Optional observer tuning. */
#define MOTOR_CONFIG_SMO_BEMF_CUTOFF_RADIANS_PER_SECOND    4000U
#define MOTOR_CONFIG_SMO_SLIDING_GAIN_MILLIVOLT            5000U
#define MOTOR_CONFIG_SMO_CURRENT_ESTIMATE_LIMIT_PU         1.0f

/* Debug identification routine settings. */
/* Rs is a compile-time profile: no per-instance configuration copy.
 * Voltage levels are modulation commands, not measured voltage PU.
 * Maximum current is measured alpha-beta magnitude, checked every ISR. */
#define IDENTIFY_RESISTANCE_SAMPLE_COUNT                    (100U)
#define IDENTIFY_RESISTANCE_SAMPLE_HZ                       (100U)
#define IDENTIFY_RESISTANCE_SETTLE_TIME_MS                   (100U)
#define IDENTIFY_RESISTANCE_SETTLE_TIMEOUT_MS                (500U)
#define IDENTIFY_RESISTANCE_CAPTURE_TIMEOUT_MS              (1500U)
#define IDENTIFY_ALIGNMENT_TIMEOUT_MS                       (5000U)
#define IDENTIFY_RESISTANCE_MIN_CURRENT_PU                  FOC_SCALAR(0.025f)
#define IDENTIFY_RESISTANCE_MAX_CURRENT_PU                  FOC_SCALAR(0.14f)
#define IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU               FOC_SCALAR(0.04f)
#define IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU               FOC_SCALAR(0.09f)

#define MOTOR_CONFIG_IDENTIFY_LD_FREQUENCY_HZ              1000U
#define MOTOR_CONFIG_IDENTIFY_LD_CAPTURE_DELAY             2U
#define MOTOR_CONFIG_IDENTIFY_LD_CAPTURE_SAMPLES           8U
#define MOTOR_CONFIG_IDENTIFY_LD_HALF_CYCLES               20U
#define MOTOR_CONFIG_IDENTIFY_LD_MODULATION_PU             0.06f
#define MOTOR_CONFIG_IDENTIFY_LD_MAX_CURRENT_PU            0.10f
#define MOTOR_CONFIG_IDENTIFY_LD_MIN_DELTA_PU              0.025f
#define MOTOR_CONFIG_IDENTIFY_LD_MAX_TRANSVERSE_CURRENT_PU 0.025f
#define MOTOR_CONFIG_IDENTIFY_LD_TRANSVERSE_CYCLES          1U

#endif /* MOTOR_CONFIG_H */
