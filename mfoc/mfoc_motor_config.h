/**
 * @file mfoc_motor_config.h
 * @brief Selected motor profile and its per-unit bases.
 * @author Codex
 * @date 2026-10-25
 */
#ifndef MFOC_MOTOR_CONFIG_H
#define MFOC_MOTOR_CONFIG_H

/* Independent bases for the selected single-motor build profile.
 * Current base is peak phase current. Voltage base is the MFOC voltage PU
 * scale; G431 currently uses 12 V and normalizes measured Vdc against it.
 * Electrical speed base is specified in Hz and converted to rad/s below. */
#define MFOC_MOTOR_BASE_CURRENT_MILLIAMP 3500U
#define MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT 12000U
#define MFOC_MOTOR_BASE_ELECTRICAL_HZ 400U
#define MFOC_MOTOR_POLE_PAIRS 7U

#define MFOC_MOTOR_BASE_CURRENT_AMP \
    ((float)MFOC_MOTOR_BASE_CURRENT_MILLIAMP / 1000.0f)
#define MFOC_MOTOR_BASE_VOLTAGE_VOLT \
    ((float)MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT / 1000.0f)
#define MFOC_MOTOR_BASE_ELECTRICAL_RAD_S \
    (6.28318530718f * (float)MFOC_MOTOR_BASE_ELECTRICAL_HZ)

/* Derived bases; keep these as formulas to prevent duplicate settings. */
#define MFOC_MOTOR_BASE_RESISTANCE_OHM \
    (MFOC_MOTOR_BASE_VOLTAGE_VOLT / MFOC_MOTOR_BASE_CURRENT_AMP)
#define MFOC_MOTOR_BASE_INDUCTANCE_HENRY \
    (MFOC_MOTOR_BASE_RESISTANCE_OHM / \
     MFOC_MOTOR_BASE_ELECTRICAL_RAD_S)
#define MFOC_MOTOR_BASE_FLUX_WEBER \
    (MFOC_MOTOR_BASE_VOLTAGE_VOLT / \
     MFOC_MOTOR_BASE_ELECTRICAL_RAD_S)
#define MFOC_MOTOR_BASE_TIME_SECOND \
    (1.0f / MFOC_MOTOR_BASE_ELECTRICAL_RAD_S)
#define MFOC_MOTOR_BASE_MECHANICAL_RAD_S \
    (MFOC_MOTOR_BASE_ELECTRICAL_RAD_S / \
     (float)MFOC_MOTOR_POLE_PAIRS)
#define MFOC_MOTOR_BASE_TORQUE_NM \
    (1.5f * (float)MFOC_MOTOR_POLE_PAIRS * \
     MFOC_MOTOR_BASE_FLUX_WEBER * MFOC_MOTOR_BASE_CURRENT_AMP)
#define MFOC_MOTOR_BASE_INERTIA_KGM2 \
    (MFOC_MOTOR_BASE_TORQUE_NM * MFOC_MOTOR_BASE_TIME_SECOND / \
     MFOC_MOTOR_BASE_MECHANICAL_RAD_S)

/* Physical motor values inherited from the previous profile. */
#define MFOC_MOTOR_RESISTANCE_OHM 4.0f
#define MFOC_MOTOR_LD_HENRY 0.0012f
#define MFOC_MOTOR_LQ_HENRY 0.0012f

#define MFOC_MOTOR_RS_PU \
    (MFOC_MOTOR_RESISTANCE_OHM / MFOC_MOTOR_BASE_RESISTANCE_OHM)
#define MFOC_MOTOR_LD_PU \
    (MFOC_MOTOR_LD_HENRY / MFOC_MOTOR_BASE_INDUCTANCE_HENRY)
#define MFOC_MOTOR_LQ_PU \
    (MFOC_MOTOR_LQ_HENRY / MFOC_MOTOR_BASE_INDUCTANCE_HENRY)

/* Flux remains the previous unmeasured placeholder; inertia is unspecified. */
#define MFOC_MOTOR_FLUX_PU 1.0f
#define MFOC_MOTOR_INERTIA_PU 0.0f

/* Accepted stopped model-update ranges for this profile, including R > 1 PU. */
#define MFOC_MOTOR_RS_MIN_PU 0.05f
#define MFOC_MOTOR_RS_MAX_PU 8.0f
#define MFOC_MOTOR_L_MIN_PU 0.0001f
#define MFOC_MOTOR_L_MAX_PU 16.0f
#define MFOC_MOTOR_FLUX_MAX_PU 16.0f
#define MFOC_MOTOR_INERTIA_MAX_PU 1024.0f

#endif
