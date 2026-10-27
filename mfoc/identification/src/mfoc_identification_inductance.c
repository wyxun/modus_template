/**
 * @file mfoc_identification_inductance.c
 * @brief Standstill pulse capture and foreground inductance integration.
 * @author Codex
 * @date 2026-10-26
 */
#include "mfoc_identification_algorithms.h"

#define L_SAMPLES MFOC_IDENTIFICATION_L_SAMPLES
#define L_DT_PU MFOC_SCALAR(MFOC_MOTOR_BASE_ELECTRICAL_RAD_S /                 \
    (float)MFOC_IDENTIFICATION_SAMPLE_HZ)

/* With +/-1 PU samples, both Q15 sums fit int32 for this capture length. */
#if L_SAMPLES < 1U || L_SAMPLES > 1024U
#error "Inductance capture requires 1..1024 intervals"
#endif

/** @brief Accumulate interval voltage and trapezoidal current endpoints.
 * @param state Capture. @param i_axis_k Current at this interval's end.
 * @param u_axis_previous Voltage applied during this interval.
 * @return Ongoing or complete. First call records the initial current only. */
fsm_rt_t mfoc_identification_inductance_Step(
    mfoc_identification_inductance_t *state,
    mfoc_scalar_t i_axis_k, mfoc_scalar_t u_axis_previous)
{
    if (state->count > L_SAMPLES) { return fsm_rt_cpl; }
    if (state->count == 0U) {
        state->i_start = i_axis_k;
    } else {
        state->sum_u += u_axis_previous;
        state->sum_i_twice += state->i_last + i_axis_k;
    }
    state->i_last = i_axis_k;
    state->count++;
    return state->count > L_SAMPLES ? fsm_rt_cpl : fsm_rt_on_going;
}

/** @brief Evaluate L = Ts_pu * sum(u - Rs*i_mean) / (i_end - i_start).
 * @param state Frozen capture. @param rs Known PU resistance.
 * @param inductance Destination L/Lbase, written on success only.
 * @return Complete or error; no computation is added to the capture path. */
fsm_rt_t mfoc_identification_inductance_Finish(
    const mfoc_identification_inductance_t *state, mfoc_scalar_t rs,
    mfoc_scalar_t *inductance)
{
    mfoc_scalar_t delta_i = state->i_last - state->i_start;
    if (state->count != L_SAMPLES + 1U || mfoc_abs(delta_i) <
        MFOC_SCALAR(MFOC_IDENTIFICATION_L_DELTA_MIN)) { return fsm_rt_err; }
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t value = L_DT_PU *
        (state->sum_u - rs * state->sum_i_twice * 0.5f) / delta_i;
#else
    /* Keep the resistive subtraction in Q30; divide before time scaling. */
    int64_t residual = (int64_t)state->sum_u * MFOC_Q_SCALE -
        (int64_t)rs * state->sum_i_twice / 2;
    int64_t value = (residual / delta_i) * L_DT_PU / MFOC_Q_SCALE;
#endif
    if (value < MFOC_SCALAR(MFOC_MOTOR_L_MIN_PU) ||
        value > MFOC_SCALAR(MFOC_MOTOR_L_MAX_PU)) { return fsm_rt_err; }
    *inductance = (mfoc_scalar_t)value;
    return fsm_rt_cpl;
}
