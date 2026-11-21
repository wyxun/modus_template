/**
 * @file mfoc_identification_rotation.h
 * @brief Encoderless no-load speed qualification and rotating captures.
 * @author Codex
 * @date 2026-11-19
 */
#ifndef MFOC_IDENTIFICATION_ROTATION_H
#define MFOC_IDENTIFICATION_ROTATION_H
#include "mfoc_identification_algorithms.h"

#define MFOC_IDENTIFICATION_ROTATION_CURRENT_MIN 0.012f

/* A power-of-two block spanning 1..2 ms makes FIXED block means exact shifts.
 * At the safety ceiling the vector advances at most 0.4 electrical rad. */
#define MFOC_ROTATION_2MS (MFOC_IDENTIFICATION_SAMPLE_HZ / 500U)
#define MFOC_ROTATION_RATE_SUPPORTED \
    (MFOC_ROTATION_2MS >= 1U && MFOC_ROTATION_2MS <= 1024U)
/* Each crossed power of two adds one bit to the constant shift. */
enum {
    MFOC_ROTATION_BLOCK = 1U << (
        (MFOC_ROTATION_2MS >= 2U)   + (MFOC_ROTATION_2MS >= 4U) +
        (MFOC_ROTATION_2MS >= 8U)   + (MFOC_ROTATION_2MS >= 16U) +
        (MFOC_ROTATION_2MS >= 32U)  + (MFOC_ROTATION_2MS >= 64U) +
        (MFOC_ROTATION_2MS >= 128U) + (MFOC_ROTATION_2MS >= 256U) +
        (MFOC_ROTATION_2MS >= 512U) + (MFOC_ROTATION_2MS >= 1024U))
};
typedef enum {
    MFOC_ROTATION_SPINUP = MFOC_IDENTIFICATION_CONTROL_VIRTUAL | 5U,
    MFOC_ROTATION_FLUX = MFOC_IDENTIFICATION_CONTROL_VIRTUAL | 6U,
    MFOC_ROTATION_INERTIA = MFOC_IDENTIFICATION_CONTROL_VIRTUAL | 7U
} mfoc_rotation_stage_t;

/** @brief Volt-second capture and midpoint rotor angle of the previous block. */
typedef struct {
    mfoc_ab_t sum_u, sum_i, start_i;
    mfoc_angle_t previous_angle;
    uint32_t count;
    bool baseline, angle_valid;
} mfoc_rotation_observer_t;

/** @brief Equal-speed-range accelerating and braking impulses. */
typedef struct {
    mfoc_scalar_t sum_torque[2], delta_speed[2], start_speed, previous_torque;
    uint32_t blocks[2];
    unsigned phase;
} mfoc_rotation_inertia_t;

/** @brief Per-instance virtual drive and decimated electrical observer. */
typedef struct {
    mfoc_dq_t i_ref;
    mfoc_angle_t drive_angle;
    uint32_t drive_step, ramp_increment, elapsed_blocks, stable_blocks;
    mfoc_scalar_t current_max, speed_max, target_speed, previous_speed;
    mfoc_dq_t previous_current;
    mfoc_rotation_observer_t observer;
    union {
        struct {
            mfoc_scalar_t sum_emf, sum_speed, sum_speed_id;
            uint32_t count;
        } flux;
        mfoc_rotation_inertia_t inertia;
    };
} mfoc_identification_rotation_t;

/** @brief Prepare bounded electrical speed and current, in PU, in foreground.
 * @param s Storage. @param current_max Motor current cap in PU.
 * @param speed_max Electrical speed cap in PU. Limits are validated before PWM. */
void mfoc_identification_rotation_Init(mfoc_identification_rotation_t *s,
    mfoc_scalar_t current_max, mfoc_scalar_t speed_max);

/** @brief Advance the virtual angle and process one preceding PWM interval.
 * @param s Rotation storage shared across stages. @param stage Selected stage.
 * @param model Known Rs, Ld/Lq; inertia also requires known magnet flux.
 * @param current Measured i[k]; caller enforces current limit before sampling.
 * @param previous_voltage u over [k-1,k).
 * @return Stage progress. drive_angle/i_ref hold the next virtual command
 * while ongoing and on SPINUP completion. Caller removes excitation on exit. */
mfoc_fsm_rt_t mfoc_identification_rotation_Step(mfoc_identification_rotation_t *s,
    mfoc_rotation_stage_t stage, const mfoc_motor_params_t *model,
    const mfoc_ab_t *current, const mfoc_ab_t *previous_voltage);

/** @brief Evaluate frozen qualified blocks; writes the result only on success.
 * @param s Rotation storage. @param stage FLUX or INERTIA.
 * @param model Known stator parameters. @param result Flux or inertia in PU.
 * @return Complete or failed. Foreground only. */
mfoc_fsm_rt_t mfoc_identification_rotation_Finish(
    const mfoc_identification_rotation_t *s, mfoc_rotation_stage_t stage,
    const mfoc_motor_params_t *model, mfoc_scalar_t *result);

/** @brief Select the next torque sign and integrate measured torque on a block.
 * @param s Rotation capture. @param speed Electrical PU at the previous block
 * boundary. @param torque Measured midpoint torque PU. @return Stage progress. */
mfoc_fsm_rt_t mfoc_identification_inertia_Block(mfoc_identification_rotation_t *s,
    mfoc_scalar_t speed, mfoc_scalar_t torque);

/** @brief Fit inertia and time-invariant drag across matched speed bands.
 * @param capture Frozen torque impulses. @param result Output J/Jbase.
 * @return Complete or failed. */
mfoc_fsm_rt_t mfoc_identification_inertia_Finish(
    const mfoc_rotation_inertia_t *capture, mfoc_scalar_t *result);
#endif
