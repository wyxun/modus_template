/**
 * @file core_test.c
 * @brief Bus updates, voltage limiting and normalization regression.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    mfoc_pi_params_t pi = {MFOC_ONE, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_core_params_t cfg = {pi, pi, MFOC_SCALAR(0.55f), MFOC_ONE};
    mfoc_core_t c = {0};
    mfoc_core_input_t in = {0};
    mfoc_core_output_t out = {0};
    assert(mfoc_core_Init(&c, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_core_SetBus(NULL, MFOC_ONE) == MFOC_RESULT_NULL);
    float worst = 0.0f;
    for (int b = 1; b <= 64; ++b) {
        mfoc_scalar_t bus = mfoc_from_float((float)b / 16.0f);
        assert(mfoc_core_SetBus(&c, bus) == MFOC_RESULT_OK);
        for (int j = -20; j <= 20; ++j) {
            assert(mfoc_core_Reset(&c) == MFOC_RESULT_OK);
            in.ref.qD = mfoc_from_float((float)j / 20.0f);
            in.ref.qQ = MFOC_SCALAR(0.3f);
            mfoc_core_Step(&c, &in, &out);
            float d = mfoc_to_float(out.u.qD);
            float q = mfoc_to_float(out.u.qQ);
            float radius = mfoc_to_float(c.u_max);
            assert(hypotf(d, q) <= radius + 0.000002f);
            float err = fabsf(mfoc_to_float(out.m.qAlpha) -
                              d / mfoc_to_float(bus));
            if (err > worst) { worst = err; }
            assert(err < 0.0001f);
            assert(fabsf(mfoc_to_float(out.m.qBeta) -
                         q / mfoc_to_float(bus)) < 0.0001f);
            if (out.limited) {
                assert(fabsf(mfoc_to_float(c.d.ui) -
                    (d - mfoc_to_float(in.ref.qD))) < 0.0001f);
            }
        }
    }
    mfoc_core_t saved = c;
    assert(mfoc_core_SetBus(&c, MFOC_ZERO) != MFOC_RESULT_OK);
    assert(memcmp(&c, &saved, sizeof(c)) == 0);
    assert(mfoc_core_SetBus(&c, -MFOC_ONE) != MFOC_RESULT_OK);
#if defined(MFOC_NUMERIC_FLOAT)
    assert(mfoc_core_SetBus(&c, NAN) != MFOC_RESULT_OK);
    assert(mfoc_core_SetBus(&c, INFINITY) != MFOC_RESULT_OK);
#else
    assert(mfoc_core_SetBus(&c, 1) == MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_core_SetBus(&c, INT32_MAX) == MFOC_RESULT_OUT_OF_RANGE);
#endif
    assert(memcmp(&c, &saved, sizeof(c)) == 0);
    cfg.bus = MFOC_ZERO;
    assert(mfoc_core_Init(&c, &cfg) != MFOC_RESULT_OK);
    assert(memcmp(&c, &saved, sizeof(c)) == 0);
    assert(mfoc_core_SetBus(&c, MFOC_ONE) == MFOC_RESULT_OK);
    assert(c.d.ui == saved.d.ui && c.q.ui == saved.q.ui);
    assert(mfoc_core_Reset(&c) == MFOC_RESULT_OK);
    assert(c.bus_inv == MFOC_ONE && c.u_max == cfg.m_max);
    assert(c.d.ui == MFOC_ZERO && c.q.ui == MFOC_ZERO);
    printf("Core passed; max normalization error %.9g\n", (double)worst);
    return 0;
}
