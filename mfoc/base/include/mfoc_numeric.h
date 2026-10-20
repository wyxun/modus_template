/**
 * @file mfoc_numeric.h
 * @brief FLOAT or int32 Q15 scalar arithmetic, migrated from foc_numeric.
 * @author Codex
 * @date 2026-10-19
 */
#ifndef MFOC_NUMERIC_H
#define MFOC_NUMERIC_H

#include "mfoc_types.h"
#include <math.h>

#define MFOC_Q_FRACTION_BITS 15
#define MFOC_Q_SCALE 32768

#if defined(MFOC_NUMERIC_FLOAT)
#define MFOC_SCALAR(value) ((mfoc_scalar_t)(value))
#define MFOC_ZERO 0.0f
#define MFOC_HALF 0.5f
#define MFOC_ONE 1.0f
#define MFOC_NEG_ONE (-1.0f)
#else
/* Constant expressions only, in range; ties round away from zero. */
#define MFOC_SCALAR(value)                                                      \
    ((mfoc_scalar_t)(((value) >= 0.0f)                                          \
        ? ((value) * (float)MFOC_Q_SCALE + 0.5f)                                \
        : ((value) * (float)MFOC_Q_SCALE - 0.5f)))
#define MFOC_ZERO ((mfoc_scalar_t)0)
#define MFOC_HALF ((mfoc_scalar_t)16384)
#define MFOC_ONE ((mfoc_scalar_t)32768)
#define MFOC_NEG_ONE ((mfoc_scalar_t)-32768)
#endif

/** @brief Check finite representation. @param value Scalar.
 * @return True for all fixed values; false for float NaN/Inf. */
static inline bool mfoc_scalar_is_finite(mfoc_scalar_t value)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return isfinite(value) != 0;
#else
    (void)value;
    return true;
#endif
}

/** @brief Convert a boundary float. @param value Physical scalar.
 * @return FLOAT unchanged; FIXED rounded and saturated, NaN becomes zero. */
mfoc_scalar_t mfoc_from_float(float value);

/** @brief Decode a scalar for boundary use. @param value Backend scalar.
 * @return Physical float. */
static inline float mfoc_to_float(mfoc_scalar_t value)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return value;
#else
    return (float)value * (1.0f / 32768.0f);
#endif
}

/** @brief Clamp to ordered bounds. @param value Input.
 * @param minimum Lower bound. @param maximum Upper bound.
 * @return Clamped value; caller validates finite, ordered bounds. */
static inline mfoc_scalar_t mfoc_sat(mfoc_scalar_t value,
    mfoc_scalar_t minimum, mfoc_scalar_t maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

/** @brief Add with FIXED carrier saturation. @param a Left. @param b Right.
 * @return Sum; FLOAT uses native IEEE arithmetic. */
static inline mfoc_scalar_t mfoc_add_sat(mfoc_scalar_t a, mfoc_scalar_t b)
{
#if defined(MFOC_NUMERIC_FIXED)
    if (b > 0 && a > INT32_MAX - b) { return INT32_MAX; }
    if (b < 0 && a < INT32_MIN - b) { return INT32_MIN; }
#endif
    return a + b;
}

/** @brief Subtract with FIXED carrier saturation. @param a Left.
 * @param b Right. @return Difference; FLOAT uses native IEEE arithmetic. */
static inline mfoc_scalar_t mfoc_sub_sat(mfoc_scalar_t a, mfoc_scalar_t b)
{
#if defined(MFOC_NUMERIC_FIXED)
    if (b < 0 && a > INT32_MAX + b) { return INT32_MAX; }
    if (b > 0 && a < INT32_MIN + b) { return INT32_MIN; }
#endif
    return a - b;
}

/** @brief PU multiply with legacy backend rounding. @param a First PU.
 * @param b Second PU. @return Product; FIXED clips inputs to [-1,1] and
 * rounds ties away from zero. FLOAT directly multiplies; keep inputs in PU. */
static inline mfoc_scalar_t mfoc_mul_pu(mfoc_scalar_t a, mfoc_scalar_t b)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return a * b;
#else
    a = mfoc_sat(a, MFOC_NEG_ONE, MFOC_ONE);
    b = mfoc_sat(b, MFOC_NEG_ONE, MFOC_ONE);
    int32_t product = a * b;
    bool negative = product < 0;
    uint32_t magnitude = negative ? (uint32_t)(-product) : (uint32_t)product;
    magnitude = (magnitude + 16384U) >> MFOC_Q_FRACTION_BITS;
    return negative ? -(mfoc_scalar_t)magnitude : (mfoc_scalar_t)magnitude;
#endif
}

/** @brief General multiply. @param a First value. @param b Second value.
 * @return FIXED wide product scaled toward zero and carrier-saturated;
 * FLOAT directly multiplies. */
static inline mfoc_scalar_t mfoc_mul_wide(mfoc_scalar_t a, mfoc_scalar_t b)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return a * b;
#else
    int64_t product = (int64_t)a * b / MFOC_Q_SCALE;
    if (product > INT32_MAX) { return INT32_MAX; }
    if (product < INT32_MIN) { return INT32_MIN; }
    return (mfoc_scalar_t)product;
#endif
}

/** @brief Checked division. @param numerator Dividend.
 * @param denominator Divisor. @param result Output, unchanged on failure.
 * @return Null, non-finite, zero-divisor or range error; otherwise OK. */
mfoc_result_t mfoc_div_checked(mfoc_scalar_t numerator,
    mfoc_scalar_t denominator, mfoc_scalar_t *result);

/** @brief Absolute value. @param value Scalar.
 * @return Magnitude; FIXED INT32_MIN saturates to INT32_MAX. */
static inline mfoc_scalar_t mfoc_abs(mfoc_scalar_t value)
{
#if defined(MFOC_NUMERIC_FIXED)
    if (value == INT32_MIN) { return INT32_MAX; }
#endif
    return value < MFOC_ZERO ? -value : value;
}
#endif
