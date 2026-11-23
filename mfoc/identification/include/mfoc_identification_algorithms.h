/**
 * @file mfoc_identification_algorithms.h
 * @brief Typed leaf inputs and control requests for motor identification.
 * @author wyxun
 * @date 2026-10-24
 */
#ifndef MFOC_IDENTIFICATION_ALGORITHMS_H
#define MFOC_IDENTIFICATION_ALGORITHMS_H
#include "mfoc_angle.h"
#include "mfoc_config.h"
#include "mfoc_motor_config.h"
#include "mfoc_motor_params.h"
#include "mfoc_types.h"

/* Must match control sampling and the shared electrical speed PU base.
 * Identification consumes one sample per control interrupt, so the rate is
 * the current-loop rate and is not an independent setting. */
#define MFOC_IDENTIFICATION_SAMPLE_HZ MFOC_MOTOR_RUN_FREQ_HZ
#define MFOC_IDENTIFICATION_SPEED_BASE_HZ MFOC_MOTOR_BASE_ELECTRICAL_HZ

/* Standstill alpha modulation relative to the DC bus. The default levels
 * leave current margin on the 2.55 ohm motor at 12 V. ALIGN uses low. */
#ifndef MFOC_IDENTIFICATION_RS_MODULATION_LOW
#define MFOC_IDENTIFICATION_RS_MODULATION_LOW 0.04f
#endif
#ifndef MFOC_IDENTIFICATION_RS_MODULATION_HIGH
#define MFOC_IDENTIFICATION_RS_MODULATION_HIGH 0.08f
#endif
#ifndef MFOC_IDENTIFICATION_RS_CURRENT_MAX
#define MFOC_IDENTIFICATION_RS_CURRENT_MAX 0.12f
#endif
#ifndef MFOC_IDENTIFICATION_RS_CURRENT_MIN
#define MFOC_IDENTIFICATION_RS_CURRENT_MIN 0.01f
#endif
#ifndef MFOC_IDENTIFICATION_RS_STABILITY
#define MFOC_IDENTIFICATION_RS_STABILITY 0.003f
#endif
/* Preserve the qualified nominal-bus slope range at the smaller step. */
#ifndef MFOC_IDENTIFICATION_RS_DELTA_MIN
#define MFOC_IDENTIFICATION_RS_DELTA_MIN 0.016f
#endif
#ifndef MFOC_IDENTIFICATION_RS_ALIGN_MS
#define MFOC_IDENTIFICATION_RS_ALIGN_MS 300U
#endif
#ifndef MFOC_IDENTIFICATION_RS_SETTLE_MS
#define MFOC_IDENTIFICATION_RS_SETTLE_MS 200U
#endif
#ifndef MFOC_IDENTIFICATION_RS_AVERAGE_MS
#define MFOC_IDENTIFICATION_RS_AVERAGE_MS 100U
#endif

/* Locked-rotor pulse (V/Vdc), capture intervals and minimum current step. */
#ifndef MFOC_IDENTIFICATION_L_PULSE
#define MFOC_IDENTIFICATION_L_PULSE 0.04f
#endif
#ifndef MFOC_IDENTIFICATION_L_SAMPLES
#define MFOC_IDENTIFICATION_L_SAMPLES 8U
#endif
#ifndef MFOC_IDENTIFICATION_L_DELTA_MIN
#define MFOC_IDENTIFICATION_L_DELTA_MIN 0.001f
#endif

/** @brief Stage control flags; inactive uses normal Motor control. */
typedef enum {
    MFOC_IDENTIFICATION_CONTROL_INACTIVE = 0,
    MFOC_IDENTIFICATION_CONTROL_VIRTUAL = 1U << 4,
    MFOC_IDENTIFICATION_CONTROL_VOLTAGE = 1U << 5,
} mfoc_identification_control_t;

#define MFOC_IDENTIFICATION_CONTROL_MASK \
    (MFOC_IDENTIFICATION_CONTROL_VIRTUAL | \
     MFOC_IDENTIFICATION_CONTROL_VOLTAGE)

#if MFOC_PWM_PRELOAD_DELAY
/** @brief Tags carried with the voltage model by a delayed PWM port. */
enum {
    MFOC_IDENTIFICATION_EXCITATION_NONE = 0,
    MFOC_IDENTIFICATION_EXCITATION_RS_LOW,
    MFOC_IDENTIFICATION_EXCITATION_RS_HIGH,
    MFOC_IDENTIFICATION_EXCITATION_LD,
    MFOC_IDENTIFICATION_EXCITATION_LQ,
};
#endif

/** @brief Motor command selected by the stage control flags.
 * VOLTAGE sends modulation_ab (V/Vdc) directly to SVPWM, bypassing Core.
 * VIRTUAL sends i_ref and an algorithm-owned angle through Core; no Feedback
 * component supplies angle or speed during an identification stage. */
typedef struct {
    mfoc_dq_t i_ref; /* CONTROL_VIRTUAL. */
    mfoc_angle_t angle; /* CONTROL_VIRTUAL; algorithm-owned virtual frame. */
    mfoc_ab_t modulation_ab; /* CONTROL_VOLTAGE, V/Vdc. */
#if MFOC_PWM_PRELOAD_DELAY
    uint8_t excitation; /* Experiment tag for the submitted voltage. */
#endif
} mfoc_identification_output_t;

/** @brief Standalone rotor-dq flux experiment with caller-supplied angle/speed.
 * Motor uses a block EMF estimate from electrical samples for rotation. */
typedef struct {
    const mfoc_ab_t *i_ab_k;
    /** Model voltage over [k-1,k), alpha-beta V/Vbase; NULL without history. */
    const mfoc_ab_t *u_ab_previous;
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /* Electrical omega/omega_base. */
} mfoc_identification_flux_input_t;

typedef enum {
    MFOC_IDENTIFICATION_RS_LOW = 0,
    MFOC_IDENTIFICATION_RS_HIGH,
    MFOC_IDENTIFICATION_RS_DONE,
} mfoc_identification_rs_level_t;

/** @brief Paired voltage/current windows; zero starts the low level. */
typedef struct {
#if defined(MFOC_NUMERIC_FIXED)
    int64_t sum_u[2]; /* Q15 model voltage can exceed one PU. */
#else
    mfoc_scalar_t sum_u[2];
#endif
    mfoc_scalar_t sum_i[2];
    mfoc_scalar_t half_i[2]; /* First-half sums, not means. */
    uint32_t count;
    mfoc_identification_rs_level_t level;
} mfoc_identification_rs_t;

/** @brief Settle and collect paired voltage/current at each level.
 * @param state Zeroed experiment state. @param current Current at k, I/Ibase.
 * @param u_ab_previous Model voltage over [k-1,k), V/Vbase; NULL skips sample.
 * @return Sampling ongoing or complete.
 * @note The stage checks its measured current limit before calling this leaf.
 * State and current are required. Caller applies LOW before sampling,
 * then uses the updated level for the next interval; DONE removes excitation.
 * Means, stability and resistance are evaluated by Finish in foreground. */
mfoc_fsm_rt_t mfoc_identification_rs_Step(
    mfoc_identification_rs_t *state, const mfoc_ab_t *current,
    const mfoc_ab_t *u_ab_previous);

/** @brief Validate both frozen windows and compute Rs in the foreground.
 * @param state Completed pairs of current and model voltage.
 * @param rs Result R/Rbase, success only.
 * @return Complete or error; failure leaves rs unchanged. */
mfoc_fsm_rt_t mfoc_identification_rs_Finish(
    const mfoc_identification_rs_t *state, mfoc_scalar_t *rs);

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
mfoc_fsm_rt_t mfoc_identification_inductance_Step(
    mfoc_identification_inductance_t *state,
    mfoc_scalar_t i_axis_k, mfoc_scalar_t u_axis_previous);

/** @brief Calculate L/Lbase from captured volt-seconds minus Rs*i loss.
 * @param state Completed capture. @param rs Known nonnegative R/Rbase.
 * @param inductance Result, unchanged on error. @return Complete or error.
 * @note Foreground; valid pointers and Rs <= MFOC_MOTOR_RS_MAX_PU required.
 * Time base is 1/omega_base. Uses trapezoidal current integration; ADC/PWM
 * timing and rotor standstill must be established by the caller. */
mfoc_fsm_rt_t mfoc_identification_inductance_Finish(
    const mfoc_identification_inductance_t *state, mfoc_scalar_t rs,
    mfoc_scalar_t *inductance);

/** @brief Prepare one optional foreground time coefficient for both axes.
 * @param dt_pu Receives 2*pi*electrical_base_hz/sample_hz on success only.
 * @param sample_hz Control samples per second.
 * @param electrical_base_hz Positive electrical Hz in the selected backend.
 * @return OK, NULL, INVALID_ARGUMENT or OUT_OF_RANGE.
 * @note Call once during experiment initialization. Keep the coefficient
 * outside the capture state. The default Finish uses the compiled profile. */
mfoc_result_t mfoc_identification_inductance_InitTiming(
    mfoc_scalar_t *dt_pu, uint32_t sample_hz, mfoc_scalar_t electrical_base_hz);

/** @brief Evaluate a frozen axis capture with the experiment's time base.
 * @param state Frozen capture, with the same preconditions as Finish.
 * @param rs Known nonnegative R/Rbase, at most MFOC_MOTOR_RS_MAX_PU.
 * @param dt_pu Coefficient returned by a successful InitTiming.
 * @param inductance Destination L/Lbase, written on success only.
 * @return Complete or error, as for the default Finish. Foreground only. */
mfoc_fsm_rt_t mfoc_identification_inductance_FinishPrepared(
    const mfoc_identification_inductance_t *state, mfoc_scalar_t rs,
    mfoc_scalar_t dt_pu, mfoc_scalar_t *inductance);

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
 * @note Caller supplies rotor angle and speed at k. A missing model voltage
 * skips the sample; valid intervals contribute rotor-dq quantities. */
mfoc_fsm_rt_t mfoc_identification_flux_Step(
    mfoc_identification_flux_t *state,
    const mfoc_identification_flux_input_t *input);

/** @brief Calculate steady-state flux from a frozen window in foreground.
 * @param state Captured samples. @param model Known Rs/Ld PU parameters.
 * @param flux Result psi/psibase, written on success only.
 * @return Complete or error; flux remains unchanged on failure. */
mfoc_fsm_rt_t mfoc_identification_flux_Finish(
    const mfoc_identification_flux_t *state,
    const mfoc_motor_params_t *model,
    mfoc_scalar_t *flux);

#endif
