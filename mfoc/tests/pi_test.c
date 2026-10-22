/**
 * @file pi_test.c
 * @brief Generic PI behavior, recovery and numerical boundary checks.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_pi.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

/** @brief Exact binary-fraction configuration. @return PI parameters. */
static mfoc_pi_params_t parameters(void)
{
    return (mfoc_pi_params_t){
        .kp = MFOC_HALF,
        .ki_ts = MFOC_SCALAR(0.125f),
        .out_min = MFOC_NEG_ONE,
        .out_max = MFOC_ONE,
        .i_min = MFOC_NEG_ONE,
        .i_max = MFOC_ONE,
    };
}

/** @brief Check ordinary PI and independent instance histories.
 * @return None. */
static void test_response(void)
{
    mfoc_pi_params_t params = parameters();
    mfoc_pi_t pi = {0};
    mfoc_pi_t other = {0};
    mfoc_scalar_t output = MFOC_ZERO;
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
    assert(mfoc_pi_Init(&other, &params) == MFOC_RESULT_OK);
    mfoc_pi_Step(&pi, MFOC_ONE, MFOC_ZERO, &output);
    assert(output == MFOC_SCALAR(0.625f));
    mfoc_pi_Step(&pi, MFOC_ONE, MFOC_ZERO, &output);
    assert(output == MFOC_SCALAR(0.75f));
    mfoc_pi_Step(&other, MFOC_ZERO, MFOC_ZERO, &output);
    assert(output == MFOC_ZERO);
    assert(mfoc_pi_Reset(&pi) == MFOC_RESULT_OK);
    mfoc_pi_Step(&pi, MFOC_ZERO, MFOC_ZERO, &output);
    assert(output == MFOC_ZERO);

    params.kp = MFOC_SCALAR(2.0f);
    params.ki_ts = MFOC_ZERO;
    params.out_min = MFOC_SCALAR(-8.0f);
    params.out_max = MFOC_SCALAR(8.0f);
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
    mfoc_pi_Step(&pi, MFOC_SCALAR(3.0f), MFOC_ONE, &output);
    assert(output == MFOC_SCALAR(4.0f));
}

/** @brief Check both saturation directions and reverse-action gains.
 * @return None. */
static void test_anti_windup(void)
{
    for (int sign = -1; sign <= 1; sign += 2) {
        mfoc_pi_params_t params = parameters();
        mfoc_pi_t pi = {0};
        mfoc_scalar_t output = MFOC_ZERO;
        mfoc_scalar_t error = mfoc_from_float((float)sign);
        assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
        for (int i = 0; i < 100; i++) {
            mfoc_pi_Step(&pi, error, MFOC_ZERO, &output);
        }
        assert(output == error);
        assert(pi.ui == mfoc_from_float((float)sign * 0.5f));
        mfoc_pi_Step(&pi, MFOC_ZERO, error, &output);
        assert(output == mfoc_from_float((float)sign * -0.125f));

        params.kp = -MFOC_HALF;
        params.ki_ts = -MFOC_SCALAR(0.125f);
        assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
        for (int i = 0; i < 100; i++) {
            mfoc_pi_Step(&pi, error, MFOC_ZERO, &output);
        }
        assert(output == -error);
        assert(pi.ui == mfoc_from_float((float)sign * -0.5f));
    }
}

/** @brief Check next-step tracking and constrained integral reset.
 * @return None. */
static void test_tracking(void)
{
    mfoc_pi_params_t params = parameters();
    mfoc_pi_t pi = {0};
    mfoc_scalar_t output = MFOC_ZERO;
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
    assert(mfoc_pi_Track(&pi, MFOC_SCALAR(0.75f), MFOC_ONE, MFOC_ZERO) ==
           MFOC_RESULT_OK);
    mfoc_pi_Step(&pi, MFOC_ONE, MFOC_ZERO, &output);
    assert(output == MFOC_SCALAR(0.75f));
    assert(mfoc_pi_Track(&pi, MFOC_SCALAR(8.0f), MFOC_ZERO, MFOC_ZERO) ==
           MFOC_RESULT_OK);
    mfoc_pi_Step(&pi, MFOC_ZERO, MFOC_ZERO, &output);
    assert(output == MFOC_ONE);

    params.i_min = MFOC_SCALAR(0.25f);
    params.i_max = MFOC_HALF;
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
    assert(pi.ui == MFOC_SCALAR(0.25f));
    assert(mfoc_pi_Track(&pi, MFOC_NEG_ONE, MFOC_ZERO, MFOC_ZERO) ==
           MFOC_RESULT_OK);
    assert(pi.ui == MFOC_SCALAR(0.25f));
}

/** @brief Check a large integration increment may cross a bound once.
 * @return None. */
static void test_crossing(void)
{
    mfoc_pi_params_t params = parameters();
    mfoc_pi_t pi = {0};
    mfoc_scalar_t output = MFOC_ZERO;
    params.kp = MFOC_ZERO;
    params.ki_ts = MFOC_SCALAR(2.0f);
    params.i_min = MFOC_SCALAR(-4.0f);
    params.i_max = MFOC_SCALAR(4.0f);
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
    for (int i = 0; i < 10; i++) {
        mfoc_pi_Step(&pi, MFOC_ONE, MFOC_ZERO, &output);
        assert(pi.ui == MFOC_SCALAR(2.0f));
        assert(output == MFOC_ONE);
    }
    for (int i = 0; i < 3; i++) {
        mfoc_pi_Step(&pi, MFOC_ZERO, MFOC_SCALAR(0.25f), &output);
    }
    assert(output == MFOC_HALF);
}

/** @brief Check rejected inputs preserve history and caller output.
 * @return None. */
static void test_boundaries(void)
{
    mfoc_pi_params_t params = parameters();
    mfoc_pi_t pi = {0};
    assert(mfoc_pi_Init(NULL, &params) == MFOC_RESULT_NULL);
    assert(mfoc_pi_Init(&pi, NULL) == MFOC_RESULT_NULL);
    assert(mfoc_pi_Reset(NULL) == MFOC_RESULT_NULL);
    assert(mfoc_pi_Track(NULL, MFOC_ZERO, MFOC_ZERO, MFOC_ZERO) ==
           MFOC_RESULT_NULL);

    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
    params.out_min = params.out_max;
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_INVALID_ARGUMENT);
    assert(pi.cfg.out_min == MFOC_NEG_ONE);
    params = parameters();
    params.i_min = MFOC_SCALAR(2.0f);
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_INVALID_ARGUMENT);
}

#if defined(MFOC_NUMERIC_FIXED)
/** @brief Check fixed carrier saturation without integer overflow.
 * @return None. */
static void test_fixed_range(void)
{
    mfoc_pi_params_t params = parameters();
    mfoc_pi_t pi = {0};
    mfoc_scalar_t output = MFOC_ZERO;
    params.kp = INT32_MAX;
    assert(mfoc_pi_Init(&pi, &params) == MFOC_RESULT_OK);
    mfoc_pi_Step(&pi, INT32_MAX, INT32_MIN, &output);
    assert(output == MFOC_ONE);
    mfoc_pi_Step(&pi, INT32_MIN, INT32_MAX, &output);
    assert(output == MFOC_NEG_ONE);
}
#endif

/** @brief Run standalone controller contracts. @return Zero on success. */
int main(void)
{
    test_response();
    test_anti_windup();
    test_tracking();
    test_crossing();
    test_boundaries();
#if defined(MFOC_NUMERIC_FIXED)
    test_fixed_range();
#endif
    puts("mfoc PI response, recovery, tracking and boundaries passed");
    return 0;
}
