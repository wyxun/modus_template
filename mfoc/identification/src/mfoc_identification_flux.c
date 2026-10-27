/**
 * @file mfoc_identification_flux.c
 * @brief Steady rotor-dq flux capture and foreground voltage-model estimate.
 * @author Codex
 * @date 2026-10-27
 */
#include "mfoc_identification_algorithms.h"
#include "mfoc_modulation.h"
#include "mfoc_transform.h"
#include <stddef.h>

/* 100 ms at the configured sampling rate; no runtime configuration copy. */
#ifndef MFOC_IDENTIFICATION_FLUX_SAMPLES
#define MFOC_IDENTIFICATION_FLUX_SAMPLES (MFOC_IDENTIFICATION_SAMPLE_HZ / 10U)
#endif
#define FLUX_SAMPLES MFOC_IDENTIFICATION_FLUX_SAMPLES
#define FLUX_SPEED_MIN MFOC_SCALAR(0.05f)
#define FLUX_HALF_STEP_BAM (INT64_C(2147483648) *                                \
    MFOC_IDENTIFICATION_SPEED_BASE_HZ / MFOC_IDENTIFICATION_SAMPLE_HZ)

/* With |i|, |speed| <= 1 and bus <= 8 PU, Q15 sums remain within int32. */
#if FLUX_SAMPLES < 1U || FLUX_SAMPLES > 4096U
#error "Flux capture requires 1..4096 samples"
#endif

/** @brief Reconstruct preceding voltage and accumulate steady rotor-dq data.
 * @param state Capture window. @param input Current/prior-voltage samples.
 * @return Ongoing or complete; missing PWM history does not advance capture.
 * @note Valid pointers and bounded finite samples are caller preconditions.
 * Caller establishes steady rotation/current before capture. i/angle are at
 * k; duty/bus cover [k-1,k]. Midpoint voltage projection assumes constant
 * speed and small electrical advance per sample. No settling or PWM here. */
fsm_rt_t mfoc_identification_flux_Step(
    mfoc_identification_flux_t *state,
    const mfoc_identification_flux_input_t *input)
{
    if (state->count >= FLUX_SAMPLES) { return fsm_rt_cpl; }
    const mfoc_core_output_t *previous = input->previous_core_output;
    if (previous == NULL) { return fsm_rt_on_going; }

    mfoc_ab_t voltage = {0};
    mfoc_dq_t i_dq = {0};
    mfoc_dq_t u_dq = {0};
    mfoc_scalar_t sine = MFOC_ZERO;
    mfoc_scalar_t cosine = MFOC_ZERO;
    mfoc_modulation_ModelVoltageAlphaBeta(&previous->duty,
        previous->bus, &voltage);
    mfoc_angle_sincos(input->angle, &sine, &cosine);
    mfoc_park_cached(input->i_ab_k, sine, cosine, &i_dq);

#if defined(MFOC_NUMERIC_FLOAT)
    int32_t half_step = (int32_t)(input->speed * (float)FLUX_HALF_STEP_BAM);
#else
    int32_t half_step = (int32_t)((int64_t)input->speed *
        FLUX_HALF_STEP_BAM / MFOC_Q_SCALE);
#endif
    mfoc_angle_t midpoint = {
        input->angle.wBam32 - (uint32_t)half_step};
    mfoc_angle_sincos(midpoint, &sine, &cosine);
    mfoc_park_cached(&voltage, sine, cosine, &u_dq);

    state->sum_u_q += u_dq.qQ;
    state->sum_i_q += i_dq.qQ;
    state->sum_speed += input->speed;
    state->sum_speed_i_d += mfoc_mul_wide(input->speed, i_dq.qD);
    state->count++;
    return state->count == FLUX_SAMPLES ? fsm_rt_cpl : fsm_rt_on_going;
}

/** @brief Calculate psi = sum(uq - Rs*iq - Ld*omega*id) / sum(omega).
 * @param state Frozen window. @param model Known Rs/Ld model.
 * @param flux Destination PU flux, written on success only.
 * @return Complete or error for incomplete/slow capture or invalid result.
 * @note Foreground; omits Lq*diq/dt, so iq must be settled. Valid model and
 * pointers are preconditions. Ideal voltage excludes deadtime/device drops;
 * this window cannot establish rotor lock or verify stability by itself. */
fsm_rt_t mfoc_identification_flux_Finish(
    const mfoc_identification_flux_t *state,
    const mfoc_motor_params_t *model,
    mfoc_scalar_t *flux)
{
    if (state->count != FLUX_SAMPLES ||
        mfoc_abs(state->sum_speed) <
            FLUX_SPEED_MIN * (mfoc_scalar_t)FLUX_SAMPLES) {
        return fsm_rt_err;
    }
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t value = (state->sum_u_q - model->rs * state->sum_i_q -
        model->ld * state->sum_speed_i_d) / state->sum_speed;
#else
    /* Keep subtraction in Q30; the quotient returns directly to Q15. */
    int64_t residual = (int64_t)state->sum_u_q * MFOC_Q_SCALE -
        (int64_t)model->rs * state->sum_i_q -
        (int64_t)model->ld * state->sum_speed_i_d;
    int64_t value = residual / state->sum_speed;
#endif
    if (value <= MFOC_ZERO ||
        value > MFOC_SCALAR(MFOC_MOTOR_FLUX_MAX_PU)) { return fsm_rt_err; }
    *flux = (mfoc_scalar_t)value;
    return fsm_rt_cpl;
}
