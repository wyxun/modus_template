/**
 * @file mfoc_identification_inductance.c
 * @brief Standstill pulse capture and foreground inductance integration.
 * @author wyxun
 * @date 2026-10-26
 */
#include "mfoc_identification_inductance_internal.h"
#include "mfoc_identification_internal.h"

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
mfoc_fsm_rt_t mfoc_identification_inductance_Step(
    mfoc_identification_inductance_t *state,
    mfoc_scalar_t i_axis_k, mfoc_scalar_t u_axis_previous)
{
    if (state->count > L_SAMPLES) { return MFOC_FSM_COMPLETE; }
    if (state->count == 0U) {
        state->i_start = i_axis_k;
    } else {
        state->sum_u += u_axis_previous;
        state->sum_i_twice += state->i_last + i_axis_k;
    }
    state->i_last = i_axis_k;
    state->count++;
    return state->count > L_SAMPLES ?
        MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING;
}

/** @copydoc mfoc_identification_inductance_Finish */
mfoc_fsm_rt_t mfoc_identification_inductance_Finish(
    const mfoc_identification_inductance_t *state, mfoc_scalar_t rs,
    mfoc_scalar_t *inductance)
{
    return _mfoc_identification_EvaluateInductance(state, rs, L_DT_PU,
                                                   inductance);
}

/** @copydoc mfoc_identification_inductance_SeriesStep */
mfoc_fsm_rt_t mfoc_identification_inductance_SeriesStep(
    mfoc_identification_inductance_series_t *state,
    mfoc_scalar_t i_axis_k, mfoc_scalar_t u_axis_previous, bool negative)
{
    const uint32_t count = state->count;
    if (count == 0U) {
        state->i_start = i_axis_k;
    } else {
        const mfoc_scalar_t trapezoid = state->i_last + i_axis_k;
        state->sum_u += negative ? -u_axis_previous : u_axis_previous;
        state->sum_i_twice += negative ? -trapezoid : trapezoid;
        /* The ending current is also the next half-wave's baseline. */
        if ((count % L_SAMPLES) == 0U) {
            const mfoc_scalar_t delta_i = i_axis_k - state->i_start;
            state->delta_i_signed += negative ? -delta_i : delta_i;
            state->i_start = i_axis_k;
        }
    }
    state->i_last = i_axis_k;
    state->count = count + 1U;
    return count == MFOC_IDENTIFICATION_L_SERIES_INTERVALS ?
        MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING;
}

/** @copydoc mfoc_identification_inductance_SeriesFinish */
mfoc_fsm_rt_t mfoc_identification_inductance_SeriesFinish(
    const mfoc_identification_inductance_series_t *state,
    mfoc_scalar_t rs, mfoc_scalar_t *inductance)
{
    /* The 16 signed half-waves cancel constant voltage and current offsets. */
    const mfoc_scalar_t min_delta = MFOC_SCALAR(MFOC_IDENTIFICATION_L_DELTA_MIN)
        * (mfoc_scalar_t)MFOC_IDENTIFICATION_L_SERIES_HALVES;
    if (state->count != MFOC_IDENTIFICATION_L_SERIES_INTERVALS + 1U ||
        state->delta_i_signed < min_delta) {
        return MFOC_FSM_FAILED;
    }
    return _mfoc_identification_SolveInductance(state->sum_u,
        state->sum_i_twice, state->delta_i_signed, rs, L_DT_PU, inductance);
}
