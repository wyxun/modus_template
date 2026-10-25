/**
 * @file smo_feedback_test.c
 * @brief Feedback voltage preparation and independent algorithm selection.
 * @author Codex
 * @date 2026-10-23
 */
#include "mfoc_feedback.h"
#include "mfoc_modulation.h"
#include "mfoc_port.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
/** @brief Supply the test clock. @return Microseconds. */
uint32_t mfoc_port_tick(void) { return tick; }
/** @brief Supply the test clock scale. @return Ticks per second. */
uint32_t mfoc_port_tick_frequency(void) { return 1000000U; }
/** @brief Hold the rotor at a known mechanical angle.
 * @param angle Destination. @return OK. */
mfoc_result_t mfoc_port_encoder(mfoc_angle_t *angle)
{
    *angle = (mfoc_angle_t){0x10000000U};
    return MFOC_RESULT_OK;
}

/** @brief Check shadow execution, output selection and missing-frame reset.
 * @return Zero after all assertions pass. */
int main(void)
{
    mfoc_motor_params_t model = {.pole_pairs = 7U,
        .rs = MFOC_ONE, .ld = MFOC_ONE, .lq = MFOC_ONE, .flux = MFOC_HALF};
    mfoc_feedback_cfg_t cfg = {
        .encoder = {.electrical_speed_base_hz = MFOC_SCALAR(400.0f),
            .speed_filter_alpha = MFOC_ONE, .sample_period_us = 1000U,
            .timeout_ms = 10U},
        .smo = {.sample_hz = 20000U, .cutoff_rad_s = 4000U,
            .speed_base_hz = MFOC_SCALAR(400.0f),
            .h = MFOC_SCALAR(5.0f / 12.0f),
            .boundary = MFOC_SCALAR(0.1f), .i_limit = MFOC_ONE}};
    mfoc_feedback_t feedback = {0};
    mfoc_feedback_output_t output = {
        .angle = {0x12345678U}, .speed = MFOC_HALF};
    assert(mfoc_feedback_Init(&feedback, &cfg, &model) == MFOC_RESULT_OK);
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK);
    mfoc_result_t rc = mfoc_feedback_Step(&feedback, NULL, &output);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
    assert(rc == MFOC_RESULT_OK && output.angle.wBam32 == 0x70000000U);
#else
    assert(rc == MFOC_RESULT_INVALID_STATE);
    assert(output.angle.wBam32 == 0x12345678U);
#endif
    mfoc_smo_t reference = {0};
    assert(mfoc_smo_Init(&reference, &cfg.smo, &model) == MFOC_RESULT_OK);
    mfoc_core_output_t previous = {
        .duty = {MFOC_SCALAR(0.75f), MFOC_SCALAR(0.25f), MFOC_HALF},
        .bus = MFOC_SCALAR(2.0f), .m = {MFOC_ONE, MFOC_ONE}};
    mfoc_ab_t current = {MFOC_SCALAR(0.03f), MFOC_SCALAR(-0.04f)};
    mfoc_feedback_input_t input = {.i_ab_k = &current,
        .previous_core_output = &previous};
    for (unsigned j = 0U; j < 50U; ++j) {
        mfoc_ab_t voltage = {0};
        mfoc_smo_output_t expected = {0};
        assert(mfoc_modulation_ModelVoltageAlphaBeta(&previous.duty,
            previous.bus, &voltage) == MFOC_RESULT_OK);
        mfoc_smo_Step(&reference, &current, &voltage, &expected);
        rc = mfoc_feedback_Step(&feedback, &input, &output);
        assert(feedback.smo_status == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_ENABLE_HFI
        assert(!feedback.hfi_output.valid);
        assert(output.u_inject_ab.qAlpha ==
            feedback.hfi_output.u_inject_ab.qAlpha);
        assert(output.u_inject_ab.qBeta ==
            feedback.hfi_output.u_inject_ab.qBeta);
        assert(mfoc_abs(output.u_inject_ab.qAlpha) +
            mfoc_abs(output.u_inject_ab.qBeta) == MFOC_HFI_VOLTAGE);
#endif
        assert(feedback.smo_output.angle.wBam32 == expected.angle.wBam32);
        assert(feedback.smo_output.speed == expected.speed);
        assert(feedback.smo.axis[0].i_hat == reference.axis[0].i_hat);
        assert(feedback.smo.axis[1].e_hat == reference.axis[1].e_hat);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
        assert(rc == MFOC_RESULT_OK);
        assert(output.angle.wBam32 == 0x70000000U);
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
        assert(rc == MFOC_RESULT_OK);
        assert(output.angle.wBam32 == expected.angle.wBam32);
#else
        assert(rc == MFOC_RESULT_INVALID_STATE);
#endif
        previous.bus = j % 2U == 0U ? MFOC_ONE : MFOC_SCALAR(2.0f);
    }
    /* An expired encoder must not prevent the independent SMO update. */
    tick = 20000U;
    rc = mfoc_feedback_Step(&feedback, &input, &output);
    assert(feedback.smo_status == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
    assert(rc == MFOC_RESULT_OK);
#else
    assert(rc != MFOC_RESULT_OK);
#endif
    input.previous_core_output = NULL;
    (void)mfoc_feedback_Step(&feedback, &input, &output);
    assert(feedback.smo_status == MFOC_RESULT_INVALID_STATE);
    assert(!feedback.smo_output.valid && !feedback.smo.has_angle);
    assert(feedback.smo.axis[0].i_hat == MFOC_ZERO);
    assert(feedback.smo.axis[1].e_hat == MFOC_ZERO);
#if MFOC_FEEDBACK_ENABLE_ENCODER
    mfoc_encoder_t encoder = feedback.encoder;
#endif
    model.rs = MFOC_HALF;
    model.ld = MFOC_HALF;
    model.lq = MFOC_HALF;
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) ==
        MFOC_RESULT_OK);
    assert(mfoc_smo_Init(&reference, &cfg.smo, &model) == MFOC_RESULT_OK);
    assert(feedback.smo.c.ku == reference.c.ku);
    assert(feedback.smo.c.kr == reference.c.kr);
    assert(!feedback.smo_output.valid && !feedback.smo.has_angle);
#if MFOC_FEEDBACK_ENABLE_ENCODER
    assert(memcmp(&encoder, &feedback.encoder, sizeof(encoder)) == 0);
#endif
    mfoc_smo_t retained = feedback.smo;
    model.lq = MFOC_ONE;
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) ==
        MFOC_RESULT_INVALID_ARGUMENT);
    assert(memcmp(&retained, &feedback.smo, sizeof(retained)) == 0);
    puts("Feedback preparation, selection and model refresh passed");
    return 0;
}
