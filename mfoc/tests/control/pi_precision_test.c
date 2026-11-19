/**
 * @file pi_precision_test.c
 * @brief PI regressions for persistent increments below one Q15 count.
 */
#include "mfoc_pi.h"
#include <assert.h>
#include <float.h>
#include <stdio.h>

static const mfoc_scalar_t lsb = MFOC_SCALAR(1.0f / 32768.0f);
static const mfoc_pi_params_t half_gain = {MFOC_ZERO, MFOC_HALF,
    MFOC_NEG_ONE, MFOC_ONE, MFOC_NEG_ONE, MFOC_ONE};

/** @brief Project an exact signed number of half-LSBs to the output. */
static mfoc_scalar_t half_counts(int count)
{
#if defined(MFOC_NUMERIC_FIXED)
    return (mfoc_scalar_t)(count / 2);
#else
    return (float)count * 0.5f * lsb;
#endif
}

static mfoc_scalar_t step(mfoc_pi_t *pi, mfoc_scalar_t error)
{
    mfoc_scalar_t output = MFOC_ZERO;
    mfoc_pi_Step(pi, error, MFOC_ZERO, &output);
    return output;
}

static void sustained_error(int sign)
{
    mfoc_pi_t pi = {0};
    assert(mfoc_pi_Init(&pi, &half_gain) == MFOC_RESULT_OK);
    for (int i = 1; i <= 1024; ++i) {
        assert(step(&pi, sign * lsb) == half_counts(sign * i));
    }
    for (int i = 1023; i >= -2; --i) {
        assert(step(&pi, -sign * lsb) == half_counts(sign * i));
    }
}

/** @brief Carry one raw Q30 count per step, then borrow back through zero. */
static void smallest_increment(int sign)
{
    mfoc_pi_params_t cfg = half_gain;
    cfg.ki_ts = lsb;
    mfoc_pi_t pi = {0};
    assert(mfoc_pi_Init(&pi, &cfg) == MFOC_RESULT_OK);

    /* Reverse one tick beyond the first Q15 boundary, with a live remainder. */
    const int outward_steps = MFOC_Q_SCALE + 1;
    int count = 0;
    for (int tick = 0; tick < 3 * outward_steps; ++tick) {
        const int direction = tick < outward_steps ? sign : -sign;
        count += direction;
        mfoc_scalar_t out = step(&pi, direction * lsb);
#if defined(MFOC_NUMERIC_FIXED)
        assert(out == count / MFOC_Q_SCALE);
#else
        assert(out == (float)count * lsb * lsb);
#endif
    }
}

/** @brief Reset discards fractions; Track preloads the next increment. */
static void reset_and_track(int sign)
{
    mfoc_pi_t pi = {0};
    mfoc_scalar_t error = sign * lsb;
    assert(mfoc_pi_Init(&pi, &half_gain) == MFOC_RESULT_OK);
    (void)step(&pi, error);
    assert(mfoc_pi_Reset(&pi) == MFOC_RESULT_OK);
    assert(step(&pi, error) == half_counts(sign));
    assert(mfoc_pi_Track(&pi, 4 * error, error, MFOC_ZERO) == MFOC_RESULT_OK);
    assert(step(&pi, error) == 4 * error);
    assert(step(&pi, error) == half_counts(9 * sign));
    assert(step(&pi, error) == 5 * error);
}

/** @brief An applied-output correction preserves fractions until clamped. */
static void external_limit(int sign)
{
    mfoc_pi_params_t cfg = half_gain;
    cfg.kp = MFOC_SCALAR(8.0f);
    cfg.i_min = -2 * lsb;
    cfg.i_max = 2 * lsb;
    mfoc_pi_t pi = {0};
    const mfoc_scalar_t error = sign * lsb;
    assert(mfoc_pi_Init(&pi, &cfg) == MFOC_RESULT_OK);
    mfoc_scalar_t request = step(&pi, error);
    mfoc_pi_ApplyLimit(&pi, request, request - 2 * error);
    assert(step(&pi, MFOC_ZERO) == half_counts(-3 * sign));

    assert(mfoc_pi_Reset(&pi) == MFOC_RESULT_OK);
    request = step(&pi, error);
    mfoc_pi_ApplyLimit(&pi, request, MFOC_ZERO);
    assert(step(&pi, MFOC_ZERO) == -2 * error);
    /* P = 8 and I = -1.5 counts (mirrored for negative error). */
    assert(step(&pi, error) == half_counts(13 * sign));
    assert(step(&pi, MFOC_ZERO) == half_counts(-3 * sign));
}

static void output_limit(int sign)
{
    mfoc_pi_params_t cfg = half_gain;
    cfg.out_min = -3 * lsb;
    cfg.out_max = 3 * lsb;
    mfoc_pi_t pi = {0};
    assert(mfoc_pi_Init(&pi, &cfg) == MFOC_RESULT_OK);
    for (int i = 0; i < 32; ++i) {
        (void)step(&pi, sign * lsb);
    }
    assert(step(&pi, MFOC_ZERO) == 3 * sign * lsb);
    assert(step(&pi, -sign * lsb) == half_counts(5 * sign));
}

/** @brief A negative fraction lets +P advance to the upper output limit. */
static void fraction_at_output_limit(void)
{
    mfoc_pi_params_t cfg = half_gain;
    cfg.kp = MFOC_ONE;
    cfg.out_min = -3 * lsb;
    cfg.out_max = lsb;
    mfoc_pi_t pi = {0};
    assert(mfoc_pi_Init(&pi, &cfg) == MFOC_RESULT_OK);
    (void)step(&pi, -lsb);
    assert(step(&pi, lsb) == lsb);
    (void)step(&pi, -lsb);
    assert(step(&pi, MFOC_ZERO) == half_counts(-1));
}

static void carrier_extremes(void)
{
    mfoc_pi_t pi = {0};
#if defined(MFOC_NUMERIC_FIXED)
    const mfoc_pi_params_t cfg = {0, INT32_MIN, INT32_MIN, INT32_MAX,
                                  INT32_MIN, INT32_MAX};
    assert(mfoc_pi_Init(&pi, &cfg) == MFOC_RESULT_OK);
    /* Each out-of-carrier increment saturates before entering the sum. */
    assert(step(&pi, INT32_MIN) == INT32_MAX);
    assert(step(&pi, INT32_MAX) == -1);
    assert(step(&pi, INT32_MAX) == INT32_MIN);
    mfoc_pi_ApplyLimit(&pi, INT32_MIN, 0);
    assert(step(&pi, 0) == 0);
    assert(step(&pi, INT32_MIN) == INT32_MAX);

    /* P + I exceeds the Q15 carrier by one count in each direction. */
    const mfoc_pi_params_t unity = {MFOC_ONE, 0, INT32_MIN, INT32_MAX,
                                    INT32_MIN, INT32_MAX};
    assert(mfoc_pi_Init(&pi, &unity) == MFOC_RESULT_OK);
    mfoc_pi_ApplyLimit(&pi, 0, 1);
    assert(step(&pi, INT32_MAX) == INT32_MAX);
    mfoc_pi_ApplyLimit(&pi, 1, -1);
    assert(step(&pi, INT32_MIN) == INT32_MIN);
#else
    const mfoc_pi_params_t cfg = {0.0f, 1.0f, -FLT_MAX, FLT_MAX,
                                  -FLT_MAX, FLT_MAX};
    assert(mfoc_pi_Init(&pi, &cfg) == MFOC_RESULT_OK);
    assert(step(&pi, FLT_MAX) == FLT_MAX);
    assert(step(&pi, -FLT_MAX) == 0.0f);
    assert(step(&pi, -FLT_MAX) == -FLT_MAX);
    assert(step(&pi, FLT_MAX) == 0.0f);
#endif
}

int main(void)
{
    for (int sign = -1; sign <= 1; sign += 2) {
        sustained_error(sign);
        smallest_increment(sign);
        reset_and_track(sign);
        external_limit(sign);
        output_limit(sign);
    }
    fraction_at_output_limit();
    carrier_extremes();
    puts("PI sub-LSB accumulation and recovery passed");
    return 0;
}
