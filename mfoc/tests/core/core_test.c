/**
 * @file core_test.c
 * @brief Bus updates, voltage limiting and normalization regression.
 * @author wyxun
 * @date 2026-10-20
 */
#include "mfoc_core.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Limit PI and stationary injection together; track the loss in dq. */
static void injection_limit(const mfoc_core_params_t *cfg)
{
    mfoc_core_t core = {0};
    mfoc_core_output_t out = {0};
    mfoc_core_input_t in = {
        .ref = {MFOC_SCALAR(0.24f), MFOC_SCALAR(-0.18f)},
        .angle = {.wBam32 = UINT32_C(0x40000000)},
        .u_inject_ab = {MFOC_SCALAR(0.08f), MFOC_SCALAR(0.10f)}
    };
    assert(mfoc_core_Init(&core, cfg) == MFOC_RESULT_OK);
    /* At a quarter turn, injection in dq is (beta, -alpha). */
    mfoc_core_Step(&core, &in, &out);
    assert(!out.limited);
    assert(fabsf(mfoc_to_float(out.u.qD) - 0.34f) < 0.0002f);
    assert(fabsf(mfoc_to_float(out.u.qQ) + 0.26f) < 0.0002f);
    assert(core.d.ui == MFOC_ZERO && core.q.ui == MFOC_ZERO);

    in.u_inject_ab = (mfoc_ab_t){MFOC_SCALAR(0.24f), MFOC_SCALAR(0.38f)};
    mfoc_core_Step(&core, &in, &out);
    assert(out.limited);
    const float d = mfoc_to_float(out.u.qD);
    const float q = mfoc_to_float(out.u.qQ);
    const float scale = MFOC_CORE_M_MAX / hypotf(0.62f, -0.42f);
    assert(fabsf(d - 0.62f * scale) < 0.0002f);
    assert(fabsf(q + 0.42f * scale) < 0.0002f);
    assert(fabsf(mfoc_to_float(core.d.ui) - (d - 0.62f)) < 0.0002f);
    assert(fabsf(mfoc_to_float(core.q.ui) - (q + 0.42f)) < 0.0002f);
}

/** @brief Prepare and apply must publish the same bits as SetBus. */
static void prepared_bus(const mfoc_core_params_t *cfg)
{
    mfoc_core_t direct = {0};
    assert(mfoc_core_Init(&direct, cfg) == MFOC_RESULT_OK);
    direct.d.ui = MFOC_SCALAR(-0.13f);
    direct.q.ui = MFOC_SCALAR(0.17f);
    mfoc_core_t applied = direct;
    mfoc_core_bus_coeff_t coeff = {0};
    assert(mfoc_core_PrepareBus(MFOC_ONE, NULL) == MFOC_RESULT_NULL);
    for (int b = 1; b <= 64; ++b) {
        const mfoc_scalar_t bus = mfoc_from_float((float)b / 16.0f);
        assert(mfoc_core_PrepareBus(bus, &coeff) == MFOC_RESULT_OK);
        mfoc_core_ApplyBus(&applied, &coeff);
        assert(mfoc_core_SetBus(&direct, bus) == MFOC_RESULT_OK);
        assert(memcmp(&direct, &applied, sizeof(direct)) == 0);
    }
    const mfoc_scalar_t invalid[] = {MFOC_ZERO, -MFOC_ONE,
#if defined(MFOC_NUMERIC_FIXED)
        1, INT32_MAX
#else
        NAN, INFINITY, FLT_MAX, 1.0e-20f
#endif
    };
    const mfoc_core_bus_coeff_t saved = coeff;
    for (size_t i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        const mfoc_result_t rc = mfoc_core_SetBus(&direct, invalid[i]);
        assert(rc != MFOC_RESULT_OK);
        assert(mfoc_core_PrepareBus(invalid[i], &coeff) == rc);
        assert(memcmp(&saved, &coeff, sizeof(coeff)) == 0);
    }
}

#if defined(MFOC_NUMERIC_FIXED)
/** @brief Find the smallest integer whose square covers the Q30 norm. */
static int64_t reference_radius(uint64_t norm_sq)
{
    uint64_t lo = 0U;
    uint64_t hi = UINT32_MAX;
    while (lo < hi) {
        uint64_t mid = (lo + hi) / 2U;
        if (mid * mid < norm_sq) { lo = mid + 1U; }
        else { hi = mid; }
    }
    return (int64_t)lo;
}

/** @brief Exercise the current loop at radius, square and word boundaries. */
static void fixed_limit_boundaries(const mfoc_core_params_t *cfg)
{
    mfoc_core_t core = {0};
    assert(mfoc_core_Init(&core, cfg) == MFOC_RESULT_OK);
    int32_t r = core.bus_coeff.u_max;
    const mfoc_ab_t requests[] = {
        {r - 1, 0}, {r, 0}, {r, 1}, {r + 1, 0}, {-r, -1},
        /* 32769 squared minus one, exactly, and plus one. */
        {32768, 256}, {32769, 0}, {32769, 1}, {32768, 32768},
        {65535, 0}, {65536, 0}, {65536, 1}, {INT32_MIN, 0},
        {INT32_MAX, INT32_MAX}, {INT32_MIN, INT32_MIN}, {0, -r}
    };
    for (size_t i = 0U; i < sizeof(requests) / sizeof(requests[0]); ++i) {
        assert(mfoc_core_Reset(&core) == MFOC_RESULT_OK);
        mfoc_core_input_t in = {.u_inject_ab = requests[i]};
        mfoc_core_output_t out = {0};
        mfoc_core_Step(&core, &in, &out);
        int64_t d = requests[i].qAlpha;
        int64_t q = requests[i].qBeta;
        uint64_t n2 = (uint64_t)(d * d) + (uint64_t)(q * q);
        int64_t n = reference_radius(n2);
        assert(out.limited == (n > r));
        assert(out.u.qD == (n > r ? d * r / n : d));
        assert(out.u.qQ == (n > r ? q * r / n : q));
    }
}
#endif

int main(void)
{
    mfoc_pi_params_t pi = {MFOC_ONE, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_core_params_t cfg = {pi, pi, MFOC_ONE};
    injection_limit(&cfg);
    prepared_bus(&cfg);
#if defined(MFOC_NUMERIC_FIXED)
    fixed_limit_boundaries(&cfg);
    mfoc_core_params_t small_bus = cfg;
    small_bus.bus = 2; /* Smallest representable radius (one Q15 count). */
    fixed_limit_boundaries(&small_bus);
#endif
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
            float radius = mfoc_to_float(bus) * MFOC_CORE_M_MAX;
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
    assert(mfoc_core_SetBus(&c, FLT_MAX) == MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_core_SetBus(&c, 1.0e-20f) == MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_core_SetBus(&c, 1.0e-30f) == MFOC_RESULT_OUT_OF_RANGE);
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
    assert(c.bus_coeff.bus_inv == MFOC_ONE);
#if defined(MFOC_NUMERIC_FLOAT)
    assert(c.bus_coeff.u_max_sq == MFOC_CORE_M_MAX * MFOC_CORE_M_MAX);
    assert(mfoc_core_SetBus(&c, 1.0e10f) == MFOC_RESULT_OK);
    in.ref = (mfoc_dq_t){MFOC_ONE, MFOC_ONE};
    mfoc_core_Step(&c, &in, &out);
    assert(!out.limited && out.u.qD == MFOC_ONE && out.u.qQ == MFOC_ONE);
#else
    assert(c.bus_coeff.u_max == MFOC_SCALAR(MFOC_CORE_M_MAX));
#endif
    assert(c.d.ui == MFOC_ZERO && c.q.ui == MFOC_ZERO);
    printf("Core passed; max normalization error %.9g\n", (double)worst);
    return 0;
}
