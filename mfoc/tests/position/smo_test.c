/**
 * @file smo_test.c
 * @brief SMO coefficient, boundary-layer and reset regression.
 * @author wyxun
 * @date 2026-10-23
 */
#include "mfoc_smo.h"
#include "mfoc_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Compare a coefficient on both numeric backends.
 * @param value Encoded scalar. @param expected Physical value.
 * @return None. */
static void near(mfoc_scalar_t value, float expected)
{
    assert(fabsf(mfoc_to_float(value) - expected) < 0.0004f);
}

/** @brief Exercise the old FOC profile using shared PU motor parameters.
 * @return Zero after all assertions pass. */
int main(void)
{
    mfoc_motor_params_t model = {
        .pole_pairs = 7U, .rs = MFOC_SCALAR(4.0f * 3.5f / 12.0f),
        .ld = MFOC_SCALAR(0.0012f * 2513.274123f * 3.5f / 12.0f),
        .lq = MFOC_SCALAR(0.0012f * 2513.274123f * 3.5f / 12.0f)};
    mfoc_smo_cfg_t cfg = {
        .sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .speed_base_hz = MFOC_SCALAR(400.0f),
        .cutoff_rad_s = 4000U, .h = MFOC_SCALAR(5.0f / 12.0f),
        .boundary = MFOC_SCALAR(0.1f), .i_limit = MFOC_ONE};
    mfoc_smo_t smo = {0};
    assert(mfoc_smo_Init(NULL, &cfg, &model) == MFOC_RESULT_NULL);
    assert(mfoc_smo_Init(&smo, &cfg, &model) == MFOC_RESULT_OK);
    /* Expectations follow MFOC_MOTOR_RUN_FREQ_HZ, so the same case also runs
     * at other control rates; span keeps the observer's own 32-step cap. */
    const float fs = (float)MFOC_MOTOR_RUN_FREQ_HZ;
    const float speed_base = 400.0f;
    const float cutoff = 4000.0f;
    const float rs_pu = 4.0f * 3.5f / 12.0f;
    const float ld_pu = 0.0012f * 2513.274123f * 3.5f / 12.0f;
    const float ku = 6.28318530718f * speed_base / fs / ld_pu;
    unsigned span = (MFOC_MOTOR_RUN_FREQ_HZ + 500U) / 1000U;
    if (span == 0U) { span = 1U; }
    if (span > 127U) { span = 127U; }
    near(smo.c.ku, ku);
    near(smo.c.kr, rs_pu * ku);
    near(smo.c.b0, cutoff / (2.0f * fs + cutoff));
    near(smo.c.a1, (cutoff - 2.0f * fs) / (2.0f * fs + cutoff));
    near(smo.c.speed_scale, fs / ((float)span * speed_base));
    assert(smo.c.speed_span == span);
    mfoc_ab_t voltage = {0};
    mfoc_smo_output_t output = {0};
    for (int j = -12; j <= 12; ++j) {
        mfoc_smo_Reset(&smo);
        mfoc_ab_t current = {mfoc_from_float((float)j / 100.0f), 0};
        mfoc_smo_Step(&smo, &current, &voltage, &output);
        float error = -(float)j / 10.0f;
        if (error > 1.0f) { error = 1.0f; }
        if (error < -1.0f) { error = -1.0f; }
        near(smo.axis[0].z_prev, (5.0f / 12.0f) * error);
        assert(mfoc_abs(smo.axis[0].z_prev) <= smo.c.h);
    }
    mfoc_smo_Reset(&smo);
    mfoc_ab_t zero = {0};
    mfoc_smo_Step(&smo, &zero, &zero, &output);
    assert(!output.valid && output.speed == MFOC_ZERO);
    /* Hold the EMF filter to isolate angle wrapping and speed-window units.
     * One sample advances 0.05 PU at any control rate; the last step is the
     * one that completes the observer's speed window. */
    const float step_turns = 0.05f * speed_base / fs;
    for (int direction = -1; direction <= 1; direction += 2) {
        mfoc_smo_Reset(&smo);
        smo.c.b0 = MFOC_ZERO;
        smo.c.a1 = -MFOC_ONE;
        for (unsigned j = 0U; j <= span; ++j) {
            float phase = (0.99f + (float)direction * (float)j * step_turns) *
                          6.28318530718f;
            float lag = atanf((float)direction * 0.05f *
                              400.0f * 6.28318530718f / 4000.0f);
            smo.axis[0].e_hat = mfoc_from_float(
                -(float)direction * 0.2f * sinf(phase - lag));
            smo.axis[1].e_hat = mfoc_from_float(
                (float)direction * 0.2f * cosf(phase - lag));
            mfoc_smo_Step(&smo, &zero, &zero, &output);
            near(output.speed, j == span ? (float)direction * 0.05f : 0.0f);
            if (j == span) {
                mfoc_angle_t expected = mfoc_angle_from_turns(
                    phase / 6.28318530718f);
                assert(output.valid);
                assert(mfoc_abs(mfoc_angle_diff(output.angle, expected)) <
                    MFOC_SCALAR(0.0005f));
            }
        }
    }
    mfoc_smo_Reset(&smo);
    mfoc_smo_t saved = smo;
    cfg.sample_hz = 0U;
    assert(mfoc_smo_Init(&smo, &cfg, &model) != MFOC_RESULT_OK);
    assert(memcmp(&smo, &saved, sizeof(smo)) == 0);
    cfg.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ;
    model.lq += 1;
    assert(mfoc_smo_Init(&smo, &cfg, &model) != MFOC_RESULT_OK);
    puts("SMO coefficients, boundary layer and reset passed");
    return 0;
}
