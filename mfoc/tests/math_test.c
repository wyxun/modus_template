/**
 * @file math_test.c
 * @brief Numeric contracts and analytic reference tests for mfoc.
 * @author Codex
 * @date 2026-10-19
 */
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "mfoc.h"

static uint32_t s_random = 1U;
static unsigned int s_external_calls = 0U;

/** @brief Generate reproducible input bits. @return Next word. */
static uint32_t random_word(void)
{
    s_random = s_random * 1664525U + 1013904223U;
    return s_random;
}

/** @brief Compare a scalar against a physical reference.
 * @param actual Backend value. @param expected Physical value.
 * @param tolerance Absolute error bound. @return None. */
static void near_value(mfoc_scalar_t actual, double expected,
                       double tolerance)
{
    assert(fabs((double)mfoc_to_float(actual) - expected) <= tolerance);
}

#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_EXTERNAL
/** @brief Fake direct target binding, used only by this host test.
 * @param phase BAM32. @param sine Optional sine output.
 * @param cosine Optional cosine output. @return None. */
void mfoc_trig_port_sincos(uint32_t phase, mfoc_scalar_t *sine,
                          mfoc_scalar_t *cosine)
{
    double radians = (double)phase * (6.283185307179586 / 4294967296.0);
    s_external_calls++;
    if (sine != NULL) {
        *sine = mfoc_from_float((float)sin(radians));
    }
    if (cosine != NULL) {
        *cosine = mfoc_from_float((float)cos(radians));
    }
}

/** @brief Fake target atan2 binding. @param y Y. @param x X.
 * @return BAM32 phase. */
mfoc_angle_t mfoc_trig_port_atan2(mfoc_scalar_t y, mfoc_scalar_t x)
{
    s_external_calls++;
    return mfoc_angle_from_turns((float)(atan2(mfoc_to_float(y),
        mfoc_to_float(x)) / 6.283185307179586));
}
#endif

/** @brief Check scalar rounding, saturation and checked division.
 * @return None. */
static void test_numeric(void)
{
    mfoc_scalar_t result = MFOC_HALF;
    near_value(mfoc_mul_wide(MFOC_SCALAR(2.0f), MFOC_HALF), 1.0, 0.0);
    assert(mfoc_div_checked(MFOC_ONE, MFOC_ZERO, &result) ==
           MFOC_RESULT_DIVIDE_BY_ZERO);
    assert(result == MFOC_HALF);
    assert(mfoc_div_checked(MFOC_ONE, MFOC_ONE, NULL) == MFOC_RESULT_NULL);
    assert(mfoc_div_checked(MFOC_ONE, MFOC_HALF, &result) == MFOC_RESULT_OK);
    near_value(result, 2.0, 0.0);
#if defined(MFOC_NUMERIC_FIXED)
    assert(mfoc_add_sat(INT32_MAX, 1) == INT32_MAX);
    assert(mfoc_sub_sat(INT32_MIN, 1) == INT32_MIN);
    assert(mfoc_abs(INT32_MIN) == INT32_MAX);
    assert(mfoc_from_float(NAN) == 0);
    assert(mfoc_from_float(INFINITY) == INT32_MAX);
    assert(mfoc_from_float(-INFINITY) == INT32_MIN);
    assert(mfoc_mul_pu(-1, MFOC_HALF) == -1);
    assert(mfoc_mul_wide(-1, MFOC_HALF) == 0);
    assert(mfoc_mul_pu(MFOC_SCALAR(2.0f), MFOC_HALF) == MFOC_HALF);
    for (unsigned int i = 0U; i < 20000U; i++) {
        int32_t a = (int32_t)(random_word() >> 1);
        int32_t b = (int32_t)(random_word() >> 1);
        if ((i & 1U) != 0U) { a = -a; }
        if ((i & 2U) != 0U) { b = -b; }
        int64_t expected = (int64_t)a * b / MFOC_Q_SCALE;
        if (expected > INT32_MAX) { expected = INT32_MAX; }
        if (expected < INT32_MIN) { expected = INT32_MIN; }
        assert(mfoc_mul_wide(a, b) == (int32_t)expected);
    }
#else
    assert(!mfoc_scalar_is_finite(NAN));
    assert(mfoc_div_checked(NAN, MFOC_ONE, &result) ==
           MFOC_RESULT_INVALID_ARGUMENT);
    result = MFOC_HALF;
    assert(mfoc_div_checked(FLT_MAX, 0.01f, &result) ==
           MFOC_RESULT_OUT_OF_RANGE);
    assert(result == MFOC_HALF);
#endif
}

/** @brief Check phase wrapping, half-turn tie and conversion boundaries.
 * @return None. */
static void test_angle(void)
{
    const mfoc_angle_t zero = {0U};
    const mfoc_angle_t half = {0x80000000U};
    assert(mfoc_angle_from_turns(-0.25f).wBam32 == 0xC0000000U);
    assert(mfoc_angle_from_turns(1.0f).wBam32 == 0U);
    assert(mfoc_angle_from_turns(NAN).wBam32 == 0U);
    assert(mfoc_angle_from_turns(INFINITY).wBam32 == 0U);
    assert(mfoc_angle_from_turns(1.0e30f).wBam32 == 0U);
    assert(mfoc_angle_to_turns((mfoc_angle_t){UINT32_MAX}) < 1.0f);
    assert(mfoc_angle_add((mfoc_angle_t){UINT32_MAX},
                         (mfoc_angle_t){1U}).wBam32 == 0U);
    assert(mfoc_angle_add_scalar(half, MFOC_HALF).wBam32 == 0U);
    assert(mfoc_angle_add_scalar(zero, -MFOC_SCALAR(0.25f)).wBam32 ==
           0xC0000000U);
    near_value(mfoc_angle_diff(half, zero), -0.5, 0.0);
    near_value(mfoc_angle_diff(zero, half), -0.5, 0.0);
    for (unsigned int i = 0U; i < 20000U; i++) {
        uint32_t delta = random_word();
        mfoc_scalar_t turns = mfoc_from_float(
            (float)(delta & 65535U) / 32768.0f - 1.0f);
        mfoc_angle_t increment = mfoc_angle_from_turns(mfoc_to_float(turns));
        assert(mfoc_angle_from_scalar(turns).wBam32 == increment.wBam32);
        assert(mfoc_angle_add_scalar((mfoc_angle_t){delta}, turns).wBam32 ==
               delta + increment.wBam32);
        int64_t signed_delta = delta < 0x80000000U ? (int64_t)delta :
                               (int64_t)delta - 4294967296LL;
        mfoc_scalar_t actual = mfoc_angle_diff((mfoc_angle_t){delta}, zero);
#if defined(MFOC_NUMERIC_FIXED)
        int64_t expected = signed_delta >= 0 ? signed_delta / 131072 :
                          -((-signed_delta + 131071) / 131072);
        assert(actual == (mfoc_scalar_t)expected);
#else
        near_value(actual, (double)signed_delta / 4294967296.0, 2.0e-8);
#endif
    }
}

/** @brief Sweep all quadrants and random phases against double libm.
 * @return None. */
static void test_trig(void)
{
#if defined(MFOC_NUMERIC_FIXED)
    const double tolerance = 3.2e-5;
#else
    const double tolerance = 2.5e-6;
#endif
    double maximum_error = 0.0;
    for (unsigned int i = 0U; i < 100000U; i++) {
        uint32_t phase = i < 65536U ? i * 65536U : random_word();
        double radians = (double)phase * (6.283185307179586 / 4294967296.0);
        mfoc_scalar_t sine = MFOC_ZERO;
        mfoc_scalar_t cosine = MFOC_ZERO;
        mfoc_angle_sincos((mfoc_angle_t){phase}, &sine, &cosine);
        near_value(sine, sin(radians), tolerance);
        near_value(cosine, cos(radians), tolerance);
        double es = fabs(mfoc_to_float(sine) - sin(radians));
        double ec = fabs(mfoc_to_float(cosine) - cos(radians));
        if (es > maximum_error) { maximum_error = es; }
        if (ec > maximum_error) { maximum_error = ec; }
    }
    mfoc_scalar_t sine = MFOC_ZERO;
    mfoc_angle_sincos((mfoc_angle_t){0x40000000U}, &sine, NULL);
    near_value(sine, 1.0, 0.0);
    mfoc_angle_sincos((mfoc_angle_t){0U}, NULL, NULL);
    assert(mfoc_angle_atan2(MFOC_ZERO, MFOC_ZERO).wBam32 == 0U);
    near_value(mfoc_angle_diff(mfoc_angle_atan2(1, 1),
        mfoc_angle_from_turns(0.125f)), 0.0, 0.0003);
    for (int yi = -8; yi <= 8; yi++) {
        for (int xi = -8; xi <= 8; xi++) {
            if (yi == 0 && xi == 0) { continue; }
            double expected = atan2((double)yi, (double)xi) /
                              6.283185307179586;
            mfoc_angle_t actual = mfoc_angle_atan2(
                mfoc_from_float((float)yi), mfoc_from_float((float)xi));
            near_value(mfoc_angle_diff(actual,
                mfoc_angle_from_turns((float)expected)), 0.0, 0.0003);
        }
    }
#if defined(MFOC_NUMERIC_FIXED)
    near_value(mfoc_angle_diff(mfoc_angle_atan2(INT32_MIN, INT32_MIN),
        mfoc_angle_from_turns(-0.375f)), 0.0, 0.0003);
    for (unsigned int i = 0U; i < 20000U; i++) {
        int32_t x = (int32_t)(random_word() >> 1);
        int32_t y = (int32_t)(random_word() >> 1);
        if ((i & 1U) != 0U) { x = -x; }
        if ((i & 2U) != 0U) { y = -y; }
        if (i < 100U) { x /= 10000000; y /= 10000000; }
        if (x == 0 && y == 0) { continue; }
        mfoc_angle_t expected = mfoc_angle_from_turns(
            (float)(atan2((double)y, (double)x) / 6.283185307179586));
        near_value(mfoc_angle_diff(mfoc_angle_atan2(y, x), expected),
                   0.0, 0.0003);
    }
#endif
#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_EXTERNAL
    assert(s_external_calls > 0U);
#else
    (void)s_external_calls;
#endif
    printf("100000-phase sin/cos maximum error: %.9g\n", maximum_error);
}

/** @brief Verify coordinate axes, round trips and modulation reconstruction.
 * @return None. */
static void test_transforms(void)
{
    const mfoc_ab_t stationary = {MFOC_HALF, MFOC_SCALAR(-0.25f)};
    mfoc_ab_t roundtrip = {0};
    mfoc_dq_t rotating = {0};
    mfoc_uvw_t duty = {0};
    bool saturated = true;
    assert(mfoc_park_cached(NULL, MFOC_ZERO, MFOC_ONE, &rotating) ==
           MFOC_RESULT_NULL);
    for (unsigned int i = 0U; i < 4096U; i++) {
        mfoc_scalar_t sine = MFOC_ZERO;
        mfoc_scalar_t cosine = MFOC_ZERO;
        mfoc_angle_sincos((mfoc_angle_t){i * 1048576U}, &sine, &cosine);
        assert(mfoc_park_cached(&stationary, sine, cosine, &rotating) ==
               MFOC_RESULT_OK);
        assert(mfoc_ipark_cached(&rotating, sine, cosine, &roundtrip) ==
               MFOC_RESULT_OK);
        near_value(roundtrip.qAlpha, 0.5, 0.0002);
        near_value(roundtrip.qBeta, -0.25, 0.0002);
        assert(mfoc_svpwm(&roundtrip, &duty, &saturated) == MFOC_RESULT_OK);
        assert(!saturated);
        assert(mfoc_modulation_ModelVoltageAlphaBeta(&duty,
            MFOC_ONE, &roundtrip) == MFOC_RESULT_OK);
        near_value(roundtrip.qAlpha, 0.5, 0.0003);
        near_value(roundtrip.qBeta, -0.25, 0.0003);
    }
    assert(mfoc_clarke(MFOC_ONE, -MFOC_HALF, -MFOC_HALF,
                       &roundtrip) == MFOC_RESULT_OK);
    near_value(roundtrip.qAlpha, 1.0, 0.0);
    near_value(roundtrip.qBeta, 0.0, 0.0);
    roundtrip = (mfoc_ab_t){MFOC_ONE, MFOC_ZERO};
    assert(mfoc_svpwm(&roundtrip, &duty, &saturated) == MFOC_RESULT_OK);
    assert(saturated);
    assert(duty.qU == MFOC_ONE && duty.qV == MFOC_ZERO);
    duty = (mfoc_uvw_t){MFOC_HALF, MFOC_HALF, MFOC_HALF};
    assert(mfoc_modulation_ModelVoltageAlphaBeta(&duty,
        MFOC_SCALAR(2.0f), &roundtrip) == MFOC_RESULT_OK);
    assert(roundtrip.qAlpha == MFOC_ZERO && roundtrip.qBeta == MFOC_ZERO);
    roundtrip = (mfoc_ab_t){MFOC_HALF, MFOC_ZERO};
    assert(mfoc_spwm(&roundtrip, &duty) == MFOC_RESULT_OK);
    assert(duty.qU == MFOC_ONE);
    assert(duty.qV == MFOC_SCALAR(0.25f));
    assert(duty.qW == MFOC_SCALAR(0.25f));
    roundtrip.qAlpha = MFOC_ONE;
    assert(mfoc_spwm(&roundtrip, &duty) == MFOC_RESULT_OK);
    assert(duty.qU == MFOC_ONE);
    assert(duty.qV == MFOC_ZERO && duty.qW == MFOC_ZERO);
}

/** @brief Check the bounded reconstruction domain, including maximum bus.
 * @return None; valid caller-supplied inputs must give finite outputs. */
static void test_model_boundaries(void)
{
    mfoc_uvw_t duty = {0};
    mfoc_ab_t voltage = {0};
#if defined(MFOC_NUMERIC_FLOAT)
    const mfoc_scalar_t maximum_bus = FLT_MAX;
#else
    const mfoc_scalar_t maximum_bus = INT32_MAX;
#endif
    for (unsigned int i = 0U; i < 8U; i++) {
        duty = (mfoc_uvw_t){(i & 1U) != 0U ? MFOC_ONE : MFOC_ZERO,
                           (i & 2U) != 0U ? MFOC_ONE : MFOC_ZERO,
                           (i & 4U) != 0U ? MFOC_ONE : MFOC_ZERO};
        assert(mfoc_modulation_ModelVoltageAlphaBeta(&duty,
            maximum_bus, &voltage) == MFOC_RESULT_OK);
        assert(mfoc_scalar_is_finite(voltage.qAlpha));
        assert(mfoc_scalar_is_finite(voltage.qBeta));
        assert(mfoc_abs(voltage.qAlpha) <= maximum_bus);
        assert(mfoc_abs(voltage.qBeta) <= maximum_bus);
    }
}

/** @brief Check square root over tiny through full-range fixed inputs.
 * @return None. */
static void test_sqrt(void)
{
    assert(mfoc_sqrt(MFOC_ZERO) == MFOC_ZERO);
    assert(mfoc_sqrt(-MFOC_ONE) == MFOC_ZERO);
    for (unsigned int i = 0U; i < 10000U; i++) {
#if defined(MFOC_NUMERIC_FIXED)
        mfoc_scalar_t value = (mfoc_scalar_t)(random_word() >> 1);
#else
        mfoc_scalar_t value = (float)(random_word() >> 1) / 32768.0f;
#endif
        double expected = sqrt((double)mfoc_to_float(value));
        near_value(mfoc_sqrt(value), expected, 0.0001 + expected * 0.0002);
    }
}

/** @brief Run independent foundational contracts. @return Zero on success. */
int main(void)
{
    test_numeric();
    test_angle();
    test_trig();
    test_transforms();
    test_model_boundaries();
    test_sqrt();
    puts("mfoc foundational math contracts passed");
    return 0;
}
