/**
 * @file flux_test.c
 * @brief Flux lifecycle and Feedback integration contracts.
 * @author wyxun
 * @date 2026-10-26
 */
#include "mfoc_feedback.h"
#include "mfoc_port.h"
#include "mfoc_motor_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned position_reads;

/** @brief Supply board-owned electrical position independently of observers.
 * @param angle Electrical BAM32. @param speed Electrical speed PU.
 * @return OK. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    ++position_reads;
    *angle = (mfoc_angle_t){UINT32_C(0x70000000)};
    *speed = MFOC_SCALAR(0.125f);
    return MFOC_RESULT_OK;
}

/** @brief Verify invalid samples, reset and independent feedback selection.
 * @return Zero after all assertions pass. */
int main(void)
{
    mfoc_motor_params_t model = {7U, MFOC_ONE, MFOC_ONE,
        MFOC_ONE, MFOC_HALF, MFOC_ZERO};
    mfoc_feedback_cfg_t cfg = {
        .smo = {MFOC_MOTOR_RUN_FREQ_HZ, 4000U, MFOC_SCALAR(400.0f),
            MFOC_HALF, MFOC_HALF, MFOC_ONE},
        .flux = {MFOC_MOTOR_RUN_FREQ_HZ, MFOC_SCALAR(400.0f), MFOC_ONE,
            MFOC_HALF, MFOC_SCALAR(100.0f)}};
    mfoc_flux_t flux = {0};
    assert(mfoc_flux_Init(&flux, &cfg.flux, &model) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(flux.c.flux_sq) - 0.25f) < 0.0001f);
    assert(fabsf(mfoc_to_float(flux.c.dt) - 0.1256637f) < 0.0001f);
    assert(fabsf(mfoc_to_float(flux.c.angle_step) - 0.02f) < 0.0001f);
    assert(fabsf(mfoc_to_float(flux.c.pll_ki_ts) - 0.01f) < 0.0001f);
    mfoc_flux_t saved = flux;
    model.lq = MFOC_HALF;
    assert(mfoc_flux_Init(&flux, &cfg.flux, &model) != MFOC_RESULT_OK);
    assert(memcmp(&flux, &saved, sizeof(flux)) == 0);
    model.lq = MFOC_ONE;
    assert(mfoc_flux_Init(NULL, &cfg.flux, &model) == MFOC_RESULT_NULL);
    mfoc_flux_cfg_t bad = cfg.flux;
    bad.sample_hz = 0U;
    assert(mfoc_flux_Init(&flux, &bad, &model) != MFOC_RESULT_OK);
    mfoc_ab_t i = {0}, u = {0};
    mfoc_flux_output_t estimate = {.valid = true, .speed = MFOC_ONE};
    mfoc_flux_Step(&flux, &i, &u, &estimate);
    assert(!estimate.valid && estimate.speed == MFOC_ZERO);
    u.qBeta = MFOC_HALF;
    mfoc_flux_Step(&flux, &i, &u, &estimate);
    assert(estimate.valid && estimate.speed > MFOC_ZERO);
    assert(flux.angle.wBam32 != 0U);
    flux.psi_s = i;
    flux.pll_i = MFOC_ONE;
    flux.angle.wBam32 = 123U;
    mfoc_flux_Reset(&flux);
    assert(memcmp(&flux, &saved, sizeof(flux)) == 0);

    mfoc_feedback_t feedback = {0};
    assert(mfoc_feedback_Init(&feedback, &cfg, &model) == MFOC_RESULT_OK);
    mfoc_core_output_t previous = {
        .duty = {MFOC_HALF, MFOC_SCALAR(0.6f), MFOC_SCALAR(0.4f)},
        .bus = MFOC_ONE};
    mfoc_feedback_input_t input = {&i, &previous};
    mfoc_feedback_output_t output = {.angle = {123U}, .speed = MFOC_ONE};
    mfoc_result_t rc = mfoc_feedback_Step(&feedback, &input, &output);
    assert(feedback.flux_output.valid);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
    assert(rc == MFOC_RESULT_OK && output.angle.wBam32 == 0x70000000U);
#else
    assert(rc == MFOC_RESULT_OK);
    assert(output.angle.wBam32 == feedback.flux_output.angle.wBam32);
#endif
    feedback.flux.psi_s = i;
    input.previous_core_output = NULL;
    rc = mfoc_feedback_Step(&feedback, &input, &output);
    assert(feedback.flux.psi_s.qAlpha == MFOC_ZERO);
    assert(!feedback.flux_output.valid);
    mfoc_feedback_t before = feedback;
    unsigned reads = position_reads;
    /* SMO accepts this model, but FIXED Flux squared amplitude underflows. */
    model.rs = MFOC_HALF;
    model.flux = MFOC_SCALAR(0.0001f);
#if defined(MFOC_NUMERIC_FIXED)
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) !=
        MFOC_RESULT_OK);
    assert(memcmp(&feedback, &before, sizeof(feedback)) == 0);
#else
    model.lq = MFOC_HALF;
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) !=
        MFOC_RESULT_OK);
    assert(memcmp(&feedback, &before, sizeof(feedback)) == 0);
    model.lq = MFOC_ONE;
#endif
    model.flux = MFOC_ONE;
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) ==
        MFOC_RESULT_OK);
    assert(feedback.flux.c.flux_sq == MFOC_ONE);
    assert(position_reads == reads); /* Model refresh cannot resample sensors. */
#if MFOC_FEEDBACK_ENABLE_SMO
    assert(feedback.smo.c.kr < before.smo.c.kr);
#endif
    model = (mfoc_motor_params_t){MFOC_MOTOR_POLE_PAIRS,
        MFOC_SCALAR(MFOC_MOTOR_RS_PU), MFOC_SCALAR(MFOC_MOTOR_LD_PU),
        MFOC_SCALAR(MFOC_MOTOR_LQ_PU), MFOC_SCALAR(MFOC_MOTOR_FLUX_PU),
        MFOC_SCALAR(MFOC_MOTOR_INERTIA_PU)};
    cfg.flux.gain = MFOC_SCALAR(MFOC_FLUX_GAIN);
    cfg.flux.pll_kp = MFOC_SCALAR(MFOC_FLUX_PLL_KP);
    cfg.flux.pll_ki = MFOC_SCALAR(MFOC_FLUX_PLL_KI);
    assert(fabsf(mfoc_to_float(model.rs) - 0.74375f) < 0.0001f);
    assert(fabsf(mfoc_to_float(model.flux) - 0.7330383f) < 0.0001f);
    assert(fabsf(mfoc_to_float(model.inertia) *
        MFOC_MOTOR_BASE_INERTIA_KGM2 - 3.7e-6f) < 1e-10f);
    assert(mfoc_feedback_Init(&feedback, &cfg, &model) == MFOC_RESULT_OK);
    puts("Flux reset, output selection and model update passed");
    return 0;
}
