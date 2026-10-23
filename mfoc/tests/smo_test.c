/**
 * @file smo_test.c
 * @brief SMO coefficient, boundary-layer and reset regression.
 * @author Codex
 * @date 2026-10-23
 */
#include "mfoc_smo.h"
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
        .sample_hz = 20000U, .speed_base_hz = MFOC_SCALAR(400.0f),
        .cutoff_rad_s = 4000U, .h = MFOC_SCALAR(5.0f / 12.0f),
        .boundary = MFOC_SCALAR(0.1f), .i_limit = MFOC_ONE};
    mfoc_smo_t smo = {0};
    assert(mfoc_smo_Init(NULL, &cfg, &model) == MFOC_RESULT_NULL);
    assert(mfoc_smo_Init(&smo, &cfg, &model) == MFOC_RESULT_OK);
    near(smo.c.ku, 0.14285714f);
    near(smo.c.kr, 0.16666667f);
    near(smo.c.b0, 0.09090909f);
    near(smo.c.a1, -0.81818182f);
    near(smo.c.speed_scale, 2.5f);
    assert(smo.c.speed_span == 20U);
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
    /* Hold the EMF filter to isolate angle wrapping and speed-window units. */
    for (int direction = -1; direction <= 1; direction += 2) {
        mfoc_smo_Reset(&smo);
        smo.c.b0 = MFOC_ZERO;
        smo.c.a1 = -MFOC_ONE;
        for (unsigned j = 0U; j <= 20U; ++j) {
            float phase = (0.99f + (float)(direction * (int)j) * 0.001f) *
                          6.28318530718f;
            smo.axis[0].e_hat = mfoc_from_float(-0.2f * sinf(phase));
            smo.axis[1].e_hat = mfoc_from_float(0.2f * cosf(phase));
            mfoc_smo_Step(&smo, &zero, &zero, &output);
            assert(output.valid);
            near(output.speed, j == 20U ? (float)direction * 0.05f : 0.0f);
        }
    }
    mfoc_smo_Reset(&smo);
    mfoc_smo_t saved = smo;
    cfg.sample_hz = 0U;
    assert(mfoc_smo_Init(&smo, &cfg, &model) != MFOC_RESULT_OK);
    assert(memcmp(&smo, &saved, sizeof(smo)) == 0);
    cfg.sample_hz = 20000U;
    model.lq += 1;
    assert(mfoc_smo_Init(&smo, &cfg, &model) != MFOC_RESULT_OK);
    puts("SMO coefficients, boundary layer and reset passed");
    return 0;
}
