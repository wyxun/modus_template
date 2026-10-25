/**
 * @file mfoc_identification_rs.c
 * @brief Fixed-voltage current capture and foreground resistance slope.
 * @author Codex
 * @date 2026-10-25
 */
#include "mfoc_identification_algorithms.h"
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

enum { RS_LOW = 0, RS_HIGH, RS_DONE, RS_FAILED };

/** @brief Validate frozen currents and calculate delta(m)*bus/delta(i).
 * @param state Captured windows. @param bus Startup Vdc/Vbase.
 * @param rs Result, unchanged on failure. @return Complete or error.
 * @note Foreground only. Supply must remain stable during both levels;
 * ideal commanded voltage excludes deadtime and current-dependent drops. */
fsm_rt_t mfoc_identification_rs_Finish(
    const mfoc_identification_rs_t *state, mfoc_scalar_t bus,
    mfoc_scalar_t *rs)
{
    if (state == NULL || rs == NULL || state->level != RS_DONE ||
        bus <= MFOC_ZERO) { return fsm_rt_err; }
    mfoc_scalar_t delta_i = MFOC_ZERO;
    for (uint32_t level = 0U; level < 2U; ++level) {
        mfoc_scalar_t current = state->sum_i[level] /
            (mfoc_scalar_t)RS_SAMPLES;
        mfoc_scalar_t half_i = state->half_i[level] /
            (mfoc_scalar_t)(RS_SAMPLES / 2U);
        if (current < MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MIN) ||
            mfoc_abs(current - half_i) >
                MFOC_SCALAR(MFOC_IDENTIFICATION_RS_STABILITY * 0.5f)) {
            return fsm_rt_err;
        }
        delta_i = level == RS_LOW ? -current : delta_i + current;
    }
    if (delta_i < MFOC_SCALAR(MFOC_IDENTIFICATION_RS_DELTA_MIN)) {
        return fsm_rt_err;
    }
    mfoc_scalar_t delta_u = mfoc_mul_wide(bus,
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH) -
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW));
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t value = delta_u / delta_i;
#else
    int64_t value = (int64_t)delta_u * MFOC_Q_SCALE / delta_i;
#endif
    if (value < MFOC_SCALAR(MFOC_MOTOR_RS_MIN_PU) ||
        value > MFOC_SCALAR(MFOC_MOTOR_RS_MAX_PU)) { return fsm_rt_err; }
    *rs = (mfoc_scalar_t)value;
    return fsm_rt_cpl;
}

/** @brief Apply two fixed alpha modulation levels and collect current sums.
 * @param state Experiment. @param current Present alpha-beta current.
 * @param output Next modulation/stop request. @return Capture progress.
 * @note ISR contains no PI, voltage accumulation, mean or ratio calculation. */
fsm_rt_t mfoc_identification_rs_Step(mfoc_identification_rs_t *state,
    const mfoc_ab_t *current, mfoc_identification_output_t *output)
{
    if (output == NULL) { return fsm_rt_err; }
    *output = (mfoc_identification_output_t){.stop = true};
    if (state == NULL || current == NULL) { return fsm_rt_err; }
    if (state->level == RS_FAILED) { return fsm_rt_err; }
    if (state->level == RS_DONE) { output->stop = false; return fsm_rt_cpl; }
    if (mfoc_abs(current->qAlpha) >
            MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MAX) ||
        mfoc_abs(current->qBeta) >
            MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MAX) -
                mfoc_abs(current->qAlpha)) {
        state->level = RS_FAILED;
        return fsm_rt_err;
    }
    output->stop = false;
    output->modulation = state->level == RS_LOW ?
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW) :
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH);
    state->count++;
    if (state->count <= RS_SETTLE) { return fsm_rt_on_going; }
    if (mfoc_abs(current->qBeta) >
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_TRANSVERSE_MAX)) {
        state->level = RS_FAILED;
        output->stop = true;
        return fsm_rt_err;
    }
    state->sum_i[state->level] += current->qAlpha;
    if (state->count == RS_SETTLE + RS_SAMPLES / 2U) {
        state->half_i[state->level] = state->sum_i[state->level];
    }
    if (state->count != RS_SETTLE + RS_SAMPLES) { return fsm_rt_on_going; }
    state->count = 0U;
    state->level++;
    if (state->level == RS_DONE) {
        output->modulation = MFOC_ZERO;
        return fsm_rt_cpl;
    }
    output->modulation = MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH);
    return fsm_rt_on_going;
}
