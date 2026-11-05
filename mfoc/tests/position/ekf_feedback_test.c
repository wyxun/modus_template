/**
 * @file ekf_feedback_test.c
 * @brief EKF routing, missing samples and transactional model updates.
 * @author wyxun
 * @date 2026-10-31
 */
#include "mfoc_feedback.h"
#include "mfoc_port.h"
#include "mfoc_modulation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t mfoc_port_tick(void) { return 0U; }
uint32_t mfoc_port_tick_frequency(void) { return 1000000U; }
mfoc_result_t mfoc_port_encoder(mfoc_angle_t *angle)
{
    *angle = (mfoc_angle_t){0x10000000U};
    return MFOC_RESULT_OK;
}

int main(void)
{
    const mfoc_motor_params_t model = {7U, 1.0f, 1.0f, 1.0f, 0.5f, 0.0f};
    const mfoc_feedback_cfg_t cfg = {
        .encoder = {.electrical_speed_base_hz = 400.0f,
            .speed_filter_alpha = 1.0f, .sample_period_us = 1000U,
            .timeout_ms = 10U},
        .smo = {MFOC_MOTOR_RUN_FREQ_HZ, 4000U, 400.0f, 0.5f, 0.5f, 1.0f},
        .flux = {MFOC_MOTOR_RUN_FREQ_HZ, 400.0f, 1.0f, 0.5f, 100.0f},
        .ekf = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .speed_base_hz = 400.0f,
            .q = {1e-6f, 1e-6f, 1e-7f, 1e-8f},
            .r = {1e-6f, 0.2e-6f, 2e-6f},
            .p0 = {0.01f, 0.01f, 0.25f, 0.08333333f},
            .min_speed = 0.01f, .max_angle_var = 0.1f,
            .max_speed_var = 0.5f}};
    mfoc_feedback_t feedback = {0};
    assert(mfoc_feedback_Init(&feedback, &cfg, &model) == MFOC_RESULT_OK);
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK);
    mfoc_ab_t i = {0.03f, -0.01f}, voltage = {0};
    mfoc_core_output_t previous = {
        .duty = {0.55f, 0.48f, 0.47f}, .bus = 0.9f};
    mfoc_feedback_input_t input = {&i, &previous};
    mfoc_feedback_output_t output = {.angle = {123U}, .speed = 1.0f};
    mfoc_feedback_output_t saved_output = output;
    mfoc_result_t rc = mfoc_feedback_Step(&feedback, &input, &output);
    assert(feedback.ekf.has_sample && !feedback.ekf_output.valid);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
    assert(rc == MFOC_RESULT_OK && output.angle.wBam32 == 0x70000000U);
#else
    assert(rc == MFOC_RESULT_INVALID_STATE);
    assert(memcmp(&output, &saved_output, sizeof(output)) == 0);
#endif
    (void)saved_output;
    /* Compare direct algorithm with reconstructed prior duty, not dq or m. */
    mfoc_ekf_t direct = feedback.ekf;
    mfoc_ekf_output_t expected = {0};
    assert(mfoc_modulation_ModelVoltageAlphaBeta(&previous.duty,
        previous.bus, &voltage) == MFOC_RESULT_OK);
    direct.speed = feedback.ekf.speed = 0.3f;
    mfoc_ekf_Step(&direct, &i, &voltage, &expected);
    rc = mfoc_feedback_Step(&feedback, &input, &output);
    assert(feedback.ekf.angle.wBam32 == direct.angle.wBam32);
    assert(memcmp(feedback.ekf.p, direct.p, sizeof(direct.p)) == 0);
    assert(fabsf(feedback.ekf.speed - direct.speed) < 1e-7f);
    assert(expected.valid);
    assert(rc == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_EKF
    assert(output.angle.wBam32 == expected.angle.wBam32);
    assert(output.speed == expected.speed);
#endif
    input.previous_core_output = NULL;
    rc = mfoc_feedback_Step(&feedback, &input, &output);
    assert(!feedback.ekf.has_sample && !feedback.ekf_output.valid);
    assert(feedback.ekf.p[9] == cfg.ekf.p0[3]);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
    assert(rc == MFOC_RESULT_OK);
#else
    assert(rc == MFOC_RESULT_INVALID_STATE);
#endif
    input.previous_core_output = &previous;
    input.i_ab_k = NULL;
    rc = mfoc_feedback_Step(&feedback, &input, &output);
    assert(!feedback.ekf.has_sample);
    input.i_ab_k = &i;
    rc = mfoc_feedback_Step(&feedback, NULL, &output);
    assert(!feedback.ekf.has_sample);
    (void)rc;

    mfoc_feedback_t before = feedback;
    mfoc_motor_params_t changed = model;
    changed.lq = 0.5f; /* Rejected by EKF after SMO accepts. */
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &changed) !=
        MFOC_RESULT_OK);
    assert(memcmp(&feedback, &before, sizeof(feedback)) == 0);
    changed = model;
    changed.rs *= 0.5f;
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &changed) ==
        MFOC_RESULT_OK);
    assert(feedback.ekf.c.ki > before.ekf.c.ki);
    assert(!feedback.ekf.has_sample && !feedback.ekf_output.valid);
#if MFOC_FEEDBACK_ENABLE_ENCODER
    assert(memcmp(&feedback.encoder, &before.encoder,
        sizeof(feedback.encoder)) == 0);
#endif
    puts("EKF feedback routing, reset and model transaction passed");
    return 0;
}
