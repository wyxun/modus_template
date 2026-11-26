/**
 * @file mfoc_identification_rs.c
 * @brief Paired voltage/current capture and foreground resistance slope.
 * @author wyxun
 * @date 2026-10-25
 */
#include "mfoc_identification_algorithms.h"
#include "mfoc_motor_config_internal.h"
#include <stddef.h>

#define RS_SETTLE (MFOC_IDENTIFICATION_SAMPLE_HZ *                              \
    MFOC_IDENTIFICATION_RS_SETTLE_MS / 1000U)
#define RS_SAMPLES (MFOC_IDENTIFICATION_SAMPLE_HZ *                             \
    MFOC_IDENTIFICATION_RS_AVERAGE_MS / 1000U)

#if RS_SAMPLES < 2U || (RS_SAMPLES % 2U) != 0U
#error "Rs averaging window must contain a positive even sample count"
#endif
#if defined(MFOC_NUMERIC_FIXED) && RS_SAMPLES > (INT32_MAX / MFOC_Q_SCALE)
#error "Bounded Q15 window sums must fit in the int32 accumulator"
#endif

/** @brief Validate frozen windows and calculate delta(mean u)/delta(mean i).
 * @param state Captured voltage/current pairs for the two levels.
 * @param rs Result, unchanged on failure. @return Complete or error.
 * @note Foreground only. Model voltage excludes deadtime and device drops. */
mfoc_fsm_rt_t mfoc_identification_rs_Finish(
    const mfoc_identification_rs_t *state, mfoc_scalar_t *rs)
{
    if (state == NULL || rs == NULL ||
        state->level != MFOC_IDENTIFICATION_RS_DONE) { return MFOC_FSM_FAILED; }
    mfoc_scalar_t delta_i = MFOC_ZERO;
    for (uint32_t level = 0U; level < 2U; ++level) {
        mfoc_scalar_t current = state->sum_i[level] /
            (mfoc_scalar_t)RS_SAMPLES;
        mfoc_scalar_t half_i = state->half_i[level] /
            (mfoc_scalar_t)(RS_SAMPLES / 2U);
        if (current < MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MIN) ||
            mfoc_abs(current - half_i) >
                MFOC_SCALAR(MFOC_IDENTIFICATION_RS_STABILITY * 0.5f)) {
            return MFOC_FSM_FAILED;
        }
        delta_i = level == MFOC_IDENTIFICATION_RS_LOW ?
            -current : delta_i + current;
    }
    if (delta_i < MFOC_SCALAR(MFOC_IDENTIFICATION_RS_DELTA_MIN)) {
        return MFOC_FSM_FAILED;
    }
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t delta_u = (state->sum_u[MFOC_IDENTIFICATION_RS_HIGH] -
        state->sum_u[MFOC_IDENTIFICATION_RS_LOW]) / (mfoc_scalar_t)RS_SAMPLES;
    mfoc_scalar_t value = delta_u / delta_i;
#else
    int64_t delta_u = (state->sum_u[MFOC_IDENTIFICATION_RS_HIGH] -
        state->sum_u[MFOC_IDENTIFICATION_RS_LOW]) / (int64_t)RS_SAMPLES;
    int64_t value = delta_u * MFOC_Q_SCALE / delta_i;
#endif
    if (value < MFOC_SCALAR(MFOC_MOTOR_RS_MIN_PU) ||
        value > MFOC_SCALAR(MFOC_MOTOR_RS_MAX_PU)) {
        return MFOC_FSM_FAILED;
    }
    *rs = (mfoc_scalar_t)value;
    return MFOC_FSM_COMPLETE;
}

/** @brief Discard settling samples, then accumulate paired alpha-axis values.
 * @param state Experiment. @param current Present alpha-beta current.
 * @param u_ab_previous Model voltage from the preceding PWM interval.
 * @return Ongoing or complete; caller owns voltage application.
 * @note Missing voltage holds the level and excludes the sample. */
mfoc_fsm_rt_t mfoc_identification_rs_Step(
    mfoc_identification_rs_t *state, const mfoc_ab_t *current,
    const mfoc_ab_t *u_ab_previous)
{
    if (state->level == MFOC_IDENTIFICATION_RS_DONE) {
        return MFOC_FSM_COMPLETE;
    }
    if (u_ab_previous == NULL) { return MFOC_FSM_ONGOING; }
    state->count++;
    if (state->count <= RS_SETTLE) { return MFOC_FSM_ONGOING; }
    state->sum_u[state->level] += u_ab_previous->qAlpha;
    state->sum_i[state->level] += current->qAlpha;
    if (state->count == RS_SETTLE + RS_SAMPLES / 2U) {
        state->half_i[state->level] = state->sum_i[state->level];
    }
    if (state->count < RS_SETTLE + RS_SAMPLES) {
        return MFOC_FSM_ONGOING;
    }
    state->count = 0U;
    state->level++;
    return state->level == MFOC_IDENTIFICATION_RS_DONE ?
        MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING;
}
