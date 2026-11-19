/**
 * @file mfoc_identification_inductance.c
 * @brief Standstill pulse capture and foreground inductance integration.
 * @author wyxun
 * @date 2026-10-26
 */
#include "mfoc_identification_inductance_internal.h"

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
