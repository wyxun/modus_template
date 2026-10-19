/**
 * @file mfoc_numeric.c
 * @brief Boundary conversion and checked division from foc_numeric.
 * @author Codex
 * @date 2026-10-19
 */
#include "mfoc_numeric.h"
#include <stddef.h>

mfoc_scalar_t mfoc_from_float(float value)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return value;
#else
    if (isnan(value)) { return MFOC_ZERO; }
    if (value >= 65536.0f) { return INT32_MAX; }
    if (value <= -65536.0f) { return INT32_MIN; }
    return (mfoc_scalar_t)(value * 32768.0f +
        (value >= 0.0f ? 0.5f : -0.5f));
#endif
}

mfoc_result_t mfoc_div_checked(mfoc_scalar_t numerator,
    mfoc_scalar_t denominator, mfoc_scalar_t *result)
{
    if (result == NULL) { return MFOC_RESULT_NULL; }
    if (!mfoc_scalar_is_finite(numerator) ||
        !mfoc_scalar_is_finite(denominator)) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    if (denominator == MFOC_ZERO) { return MFOC_RESULT_DIVIDE_BY_ZERO; }
#if defined(MFOC_NUMERIC_FLOAT)
    float quotient = numerator / denominator;
    if (!isfinite(quotient)) { return MFOC_RESULT_OUT_OF_RANGE; }
#else
    int64_t quotient = (int64_t)numerator * MFOC_Q_SCALE / denominator;
    if (quotient > INT32_MAX || quotient < INT32_MIN) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#endif
    *result = (mfoc_scalar_t)quotient;
    return MFOC_RESULT_OK;
}
