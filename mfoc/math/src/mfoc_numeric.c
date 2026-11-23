/**
 * @file mfoc_numeric.c
 * @brief Boundary conversion and checked division from foc_numeric.
 * @author wyxun
 * @date 2026-10-19
 */
#include "mfoc_numeric.h"
#include <stddef.h>

#if defined(MFOC_ABI_ELF)
/* One absolute definition belongs to the linked MFOC library. */
__asm__(".globl " MFOC_ABI_STRING(MFOC_ABI_SIGNATURE) "\n"
        MFOC_ABI_STRING(MFOC_ABI_SIGNATURE) " = 0");
#endif

mfoc_scalar_t mfoc_from_float(float value)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return value;
#else
    if (isnan(value)) { return MFOC_ZERO; }
    if (value >= 65536.0f) { return INT32_MAX; }
    if (value <= -65536.0f) { return INT32_MIN; }
    /* Adding 0.5f first can round a value just below a tie onto it. */
    const float scaled = value * (float)MFOC_Q_SCALE;
    const mfoc_scalar_t whole = (mfoc_scalar_t)scaled;
    const float fraction = scaled - (float)whole;
    if (fraction >= 0.5f) { return whole + 1; }
    if (fraction <= -0.5f) { return whole - 1; }
    return whole;
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
