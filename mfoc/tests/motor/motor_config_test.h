/**
 * @file motor_config_test.h
 * @brief Alternate motor profile for compile-time selection tests.
 * @author Codex
 * @date 2026-11-21
 * @note Synthetic values for build and simulation tests.
 */
#ifndef MFOC_TEST_MOTOR_CONFIG_H
#define MFOC_TEST_MOTOR_CONFIG_H

/* mfoc_motor_params_t physical values. */
#define MFOC_MOTOR_POLE_PAIRS              6U
#define MFOC_MOTOR_RESISTANCE_OHM          2.75f
#define MFOC_MOTOR_LD_HENRY                0.00095f
#define MFOC_MOTOR_LQ_HENRY                0.00095f
#define MFOC_MOTOR_FLUX_WEBER              0.0039f
#define MFOC_MOTOR_INERTIA_KGM2            0.0000047f

/* mfoc_motor_cfg_t normalization bases. */
#define MFOC_MOTOR_BASE_CURRENT_MILLIAMP   4000U
#define MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT  14000U
#define MFOC_MOTOR_BASE_ELECTRICAL_HZ       320U

/* Stopped Motor model update limits. */
#define MFOC_MOTOR_RS_MIN_PU               0.05f
#define MFOC_MOTOR_RS_MAX_PU               8.0f
#define MFOC_MOTOR_L_MIN_PU                0.0001f
#define MFOC_MOTOR_L_MAX_PU                16.0f
#define MFOC_MOTOR_FLUX_MAX_PU             16.0f
#define MFOC_MOTOR_INERTIA_MAX_PU          1024.0f

/* Sensorless start and closed-loop handover. */
#define MFOC_START_MIN_SPEED_PU            0.4f
#define MFOC_START_SPEED_WINDOW            0.1f
#define MFOC_START_HANDOVER_MS             2000U
#define MFOC_START_STABLE_MS               20U
#define MFOC_START_CAPTURE_TURNS           (10.0f / 360.0f)
#define MFOC_START_BLEND_TURNS_S           1U

/* Capture q-axis current adjustment. */
#define MFOC_START_IQ_UPDATE_MS            5U
#define MFOC_START_IQ_ANGLE_GAIN           0.5f
#define MFOC_START_IQ_SPEED_GAIN           0.05f
#define MFOC_START_IQ_SLEW_PU_S            0.2f
#define MFOC_START_IQ_MIN_PU               0.001f

#endif
