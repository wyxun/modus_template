/**
 * @file mfoc_identification_algorithms.h
 * @brief Typed leaf inputs and control requests for motor identification.
 * @author Codex
 * @date 2026-10-24
 */
#ifndef MFOC_IDENTIFICATION_ALGORITHMS_H
#define MFOC_IDENTIFICATION_ALGORITHMS_H
#include "mfoc_core.h"
#include "mfoc_motor_config.h"
#include "mfoc_motor_params.h"
#include "perf_counter.h"

/* Must match control sampling and the shared electrical speed PU base.
 * Timing stays constant across a run; no period is copied in every sample. */
#ifndef MFOC_IDENTIFICATION_SAMPLE_HZ
#define MFOC_IDENTIFICATION_SAMPLE_HZ 20000U
#endif
#define MFOC_IDENTIFICATION_SPEED_BASE_HZ MFOC_MOTOR_BASE_ELECTRICAL_HZ

/** @brief Stage control flags; inactive uses normal Motor control. */
typedef enum {
    MFOC_IDENTIFICATION_CONTROL_INACTIVE = 0,
    MFOC_IDENTIFICATION_CONTROL_VIRTUAL = 1U << 4,
    MFOC_IDENTIFICATION_CONTROL_VOLTAGE = 1U << 5,
} mfoc_identification_control_t;

#define MFOC_IDENTIFICATION_CONTROL_MASK \
    (MFOC_IDENTIFICATION_CONTROL_VIRTUAL | \
     MFOC_IDENTIFICATION_CONTROL_VOLTAGE)

/** @brief Motor command selected by the stage control flags.
 * VOLTAGE sends modulation_ab (V/Vdc) directly to SVPWM, bypassing Core.
 * VIRTUAL sends i_ref and an algorithm-owned angle through Core; no Feedback
 * component supplies angle or speed during an identification stage. */
typedef struct {
    mfoc_dq_t i_ref; /* CONTROL_VIRTUAL. */
    mfoc_angle_t angle; /* CONTROL_VIRTUAL; algorithm-owned virtual frame. */
    mfoc_ab_t modulation_ab; /* CONTROL_VOLTAGE, V/Vdc. */
} mfoc_identification_output_t;

/** @brief Standalone rotor-dq flux experiment with caller-supplied angle/speed.
 * Not connected to the Feedback-independent Motor identification path. */
typedef struct {
    const mfoc_ab_t *i_ab_k;
    const mfoc_core_output_t *previous_core_output; /* NULL at startup. */
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /* Electrical omega/omega_base. */
} mfoc_identification_flux_input_t;

/** @brief Mechanical experiment; torque uses measured i[k] and the PU model.
 * Sample timing comes from SAMPLE_HZ; speed and angle match i[k].
 * Leaf derives iq only at its required cadence, not from stale Core i[k-1]. */
typedef struct {
    const mfoc_ab_t *i_ab_k;
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /* Electrical PU; same ratio as mechanical speed PU. */
} mfoc_identification_inertia_input_t;

typedef enum {
    MFOC_IDENTIFICATION_RS_LOW = 0,
    MFOC_IDENTIFICATION_RS_HIGH,
    MFOC_IDENTIFICATION_RS_DONE,
} mfoc_identification_rs_level_t;

/** @brief Two fixed-voltage current windows; zero starts the low level. */
typedef struct {
    mfoc_scalar_t sum_i[2];
    mfoc_scalar_t half_i[2]; /* First-half sums, not means. */
    uint32_t count;
    mfoc_identification_rs_level_t level;
} mfoc_identification_rs_t;

/** @brief Wait for settling and collect each voltage level's current window.
 * @param state Zeroed experiment state. @param current Synchronous sample.
 * @return Sampling ongoing or complete.
 * @note Valid pointers are required. Caller applies LOW before sampling,
 * then uses the updated level for the next interval; DONE removes excitation.
 * Means, stability and resistance are evaluated by Finish in foreground. */
fsm_rt_t mfoc_identification_rs_Step(
    mfoc_identification_rs_t *state,
    const mfoc_ab_t *current);

/** @brief Validate both frozen windows and compute Rs in the foreground.
 * @param state Completed samples. @param bus Startup Vdc/Vbase.
 * @param rs Result R/Rbase, success only.
 * @return Complete or error; failure leaves rs unchanged. */
fsm_rt_t mfoc_identification_rs_Finish(
    const mfoc_identification_rs_t *state, mfoc_scalar_t bus,
    mfoc_scalar_t *rs);

/** @brief One fixed-axis voltage pulse; zero initializes a fresh capture. */
typedef struct {
    mfoc_scalar_t i_start;
    mfoc_scalar_t i_last;
    mfoc_scalar_t sum_u;
    mfoc_scalar_t sum_i_twice; /* Sum of adjacent current endpoint pairs. */
    uint32_t count; /* One baseline sample, followed by L_SAMPLES intervals. */
} mfoc_identification_inductance_t;

/** @brief Capture a stationary d- or q-axis current transient.
 * @param state Zeroed capture. @param i_axis_k Current i[k], I/Ibase.
 * @param u_axis_previous Voltage over the preceding interval, V/Vbase.
 * @return Ongoing, or complete after baseline plus L_SAMPLES calls.
 * @note Valid pointers and finite samples within +/-1 PU are preconditions.
 * Rotor and selected dq frame must remain fixed. Caller supplies the same
 * axis for both values; voltage comes from the applied interval, not the
 * next PWM request. First call captures i[0] and ignores voltage: apply the
 * pulse afterwards. Subsequent calls capture the rise immediately, with no
 * settling delay. Caller removes excitation on completion, then calls Finish.
 * No PWM, voltage selection, projection or result calculation occurs here. */
fsm_rt_t mfoc_identification_inductance_Step(
    mfoc_identification_inductance_t *state,
    mfoc_scalar_t i_axis_k, mfoc_scalar_t u_axis_previous);

/** @brief Calculate L/Lbase from captured volt-seconds minus Rs*i loss.
 * @param state Completed capture. @param rs Known nonnegative R/Rbase.
 * @param inductance Result, unchanged on error. @return Complete or error.
 * @note Foreground; valid pointers and Rs <= MFOC_MOTOR_RS_MAX_PU required.
 * Time base is 1/omega_base. Uses trapezoidal current integration; ADC/PWM
 * timing and rotor standstill must be established by the caller. */
fsm_rt_t mfoc_identification_inductance_Finish(
    const mfoc_identification_inductance_t *state, mfoc_scalar_t rs,
    mfoc_scalar_t *inductance);

/** @brief Steady rotor-dq window; zero before capture, freeze before Finish.
 * Sums retain PU scaling; count is the number of accepted samples. */
typedef struct {
    mfoc_scalar_t sum_u_q;
    mfoc_scalar_t sum_i_q;
    mfoc_scalar_t sum_speed;
    mfoc_scalar_t sum_speed_i_d; /* Sum of omega_pu * id_pu. */
    uint32_t count;
} mfoc_identification_flux_t;

/** @brief Capture flux samples after rotation and current have settled.
 * @param state Instance-owned window. @param input Current and prior voltage.
 * @return Ongoing or complete after the sampling window.
 * @note Caller establishes valid rotor feedback and preceding PWM history.
 * Step reconstructs voltage and accumulates rotor-dq quantities;
 * it does not start Motor, select feedback, or change current references. */
fsm_rt_t mfoc_identification_flux_Step(
    mfoc_identification_flux_t *state,
    const mfoc_identification_flux_input_t *input);

/** @brief Calculate steady-state flux from a frozen window in foreground.
 * @param state Captured samples. @param model Known Rs/Ld PU parameters.
 * @param flux Result psi/psibase, written on success only.
 * @return Complete or error; flux remains unchanged on failure. */
fsm_rt_t mfoc_identification_flux_Finish(
    const mfoc_identification_flux_t *state,
    const mfoc_motor_params_t *model,
    mfoc_scalar_t *flux);

/** @brief Acceleration/deceleration leaf; estimator remains TODO.
 * @param input Sample. @param model Torque model. @param output Request.
 * @param inertia Result J/Jbase, written on cpl. @return Ongoing/cpl/error.
 * @note A load-torque assumption/estimator is part of the future algorithm. */
fsm_rt_t mfoc_identification_inertia_Step(
    const mfoc_identification_inertia_input_t *input,
    const mfoc_motor_params_t *model, mfoc_identification_output_t *output,
    mfoc_scalar_t *inertia);
#endif
