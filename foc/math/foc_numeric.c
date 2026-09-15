/*******************************************************************************
 * @file    foc_numeric.c
 * @brief   Architecture-independent scalar backend implementation
 ******************************************************************************/

#include "foc_numeric.h"

#include <limits.h>
#include <stddef.h>

foc_scalar_t foc_from_float(float fValue)
{
#if defined(FOC_NUMERIC_FLOAT)
    return fValue;
#else
    const float fMaximum = (float)INT32_MAX / (float)FOC_Q_SCALE;
    const float fMinimum = (float)INT32_MIN / (float)FOC_Q_SCALE;

    if (fValue >= fMaximum) {
        return INT32_MAX;
    }
    if (fValue <= fMinimum) {
        return INT32_MIN;
    }
    return (foc_scalar_t)(fValue * (float)FOC_Q_SCALE +
                          (fValue >= 0.0f ? 0.5f : -0.5f));
#endif
}

float foc_to_float(foc_scalar_t qValue)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qValue;
#else
    return (float)qValue / (float)FOC_Q_SCALE;
#endif
}

foc_scalar_t foc_add_sat(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA + qB;
#else
    if (qB > 0 && qA > INT32_MAX - qB) {
        return INT32_MAX;
    }
    if (qB < 0 && qA < INT32_MIN - qB) {
        return INT32_MIN;
    }
    return qA + qB;
#endif
}

foc_scalar_t foc_sub_sat(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA - qB;
#else
    if (qB < 0 && qA > INT32_MAX + qB) {
        return INT32_MAX;
    }
    if (qB > 0 && qA < INT32_MIN + qB) {
        return INT32_MIN;
    }
    return qA - qB;
#endif
}

foc_scalar_t foc_mul_pu(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA * qB;
#else
    int32_t nProduct;
    uint32_t wMagnitude;
    bool bNegative;

    qA = foc_sat(qA, FOC_NEG_ONE, FOC_ONE);
    qB = foc_sat(qB, FOC_NEG_ONE, FOC_ONE);
    nProduct = qA * qB;
    bNegative = nProduct < 0;
    wMagnitude = bNegative ? (uint32_t)(-nProduct) : (uint32_t)nProduct;
    wMagnitude = (wMagnitude + (1u << (FOC_Q_FRACTION_BITS - 1))) >>
                 FOC_Q_FRACTION_BITS;

    return bNegative ? -(foc_scalar_t)wMagnitude : (foc_scalar_t)wMagnitude;
#endif
}

foc_scalar_t foc_mul_wide(foc_scalar_t qA, foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA * qB;
#else
    int64_t llProduct = (int64_t)qA * (int64_t)qB;
    llProduct /= FOC_Q_SCALE;
    if (llProduct > INT32_MAX) {
        return INT32_MAX;
    }
    if (llProduct < INT32_MIN) {
        return INT32_MIN;
    }
    return (foc_scalar_t)llProduct;
#endif
}

foc_result_t foc_div_checked(foc_scalar_t qNumerator,
                             foc_scalar_t qDenominator,
                             foc_scalar_t *pqResult)
{
    if (pqResult == NULL) {
        return FOC_RESULT_NULL;
    }
#if defined(FOC_NUMERIC_FLOAT)
    if (!isfinite(qNumerator) || !isfinite(qDenominator)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (qDenominator == FOC_ZERO) {
        return FOC_RESULT_DIVIDE_BY_ZERO;
    }
    *pqResult = qNumerator / qDenominator;
    if (!isfinite(*pqResult)) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#else
    if (qDenominator == FOC_ZERO) {
        return FOC_RESULT_DIVIDE_BY_ZERO;
    }
    int64_t llResult = ((int64_t)qNumerator * FOC_Q_SCALE) / qDenominator;
    if (llResult > INT32_MAX || llResult < INT32_MIN) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    *pqResult = (foc_scalar_t)llResult;
#endif
    return FOC_RESULT_OK;
}

foc_scalar_t foc_sat(foc_scalar_t qValue,
                     foc_scalar_t qMinimum,
                     foc_scalar_t qMaximum)
{
    if (qValue < qMinimum) {
        return qMinimum;
    }
    if (qValue > qMaximum) {
        return qMaximum;
    }
    return qValue;
}

foc_scalar_t foc_abs(foc_scalar_t qValue)
{
#if defined(FOC_NUMERIC_FIXED)
    if (qValue == INT32_MIN) {
        return INT32_MAX;
    }
#endif
    return qValue < FOC_ZERO ? -qValue : qValue;
}

foc_result_t foc_gain_Init(foc_gain_t *ptGain,
                           int16_t nInteger,
                           foc_scalar_t qFraction)
{
    if (ptGain == NULL) {
        return FOC_RESULT_NULL;
    }
    if (qFraction < FOC_NEG_ONE || qFraction > FOC_ONE) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptGain->nInteger = nInteger;
    ptGain->qFraction = qFraction;
    return FOC_RESULT_OK;
}

foc_result_t foc_gain_from_float(float fGain, foc_gain_t *ptGain)
{
    int32_t nInteger;

    if (ptGain == NULL) {
        return FOC_RESULT_NULL;
    }
    if (fGain > 32767.999f || fGain < -32767.999f) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    nInteger = (int32_t)fGain;
    return foc_gain_Init(ptGain, (int16_t)nInteger,
                         foc_from_float(fGain - (float)nInteger));
}

foc_result_t foc_gain_from_scalar(foc_scalar_t qGain, foc_gain_t *ptGain)
{
    int32_t nInteger;
    foc_scalar_t qFraction;

    if (ptGain == NULL) {
        return FOC_RESULT_NULL;
    }
#if defined(FOC_NUMERIC_FLOAT)
    if (qGain > 32767.999f || qGain < -32767.999f) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    nInteger = (int32_t)qGain;
    qFraction = qGain - (foc_scalar_t)nInteger;
#else
    nInteger = qGain / FOC_Q_SCALE;
    if (nInteger > INT16_MAX || nInteger < INT16_MIN) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    qFraction = qGain - nInteger * FOC_Q_SCALE;
#endif
    return foc_gain_Init(ptGain, (int16_t)nInteger, qFraction);
}

bool foc_gain_IsValid(const foc_gain_t *ptGain)
{
    return ptGain != NULL && ptGain->qFraction >= FOC_NEG_ONE &&
           ptGain->qFraction <= FOC_ONE;
}

foc_scalar_t foc_gain_apply(const foc_gain_t *ptGain,
                            foc_scalar_t qValue)
{
    foc_scalar_t qIntegerTerm;
    foc_scalar_t qFractionTerm;

    if (!foc_gain_IsValid(ptGain)) {
        return FOC_ZERO;
    }
    qValue = foc_sat(qValue, FOC_SCALAR(-1.999f), FOC_SCALAR(1.999f));
#if defined(FOC_NUMERIC_FLOAT)
    qIntegerTerm = qValue * (foc_scalar_t)ptGain->nInteger;
    qFractionTerm = qValue * ptGain->qFraction;
#else
    int32_t nProduct;
    uint32_t wMagnitude;
    bool bNegative;

    qIntegerTerm = qValue * ptGain->nInteger;
    nProduct = qValue * ptGain->qFraction;
    bNegative = nProduct < 0;
    wMagnitude = bNegative ? (uint32_t)(-nProduct) : (uint32_t)nProduct;
    wMagnitude = (wMagnitude + (1u << (FOC_Q_FRACTION_BITS - 1))) >>
                 FOC_Q_FRACTION_BITS;
    qFractionTerm = bNegative ? -(foc_scalar_t)wMagnitude :
                                (foc_scalar_t)wMagnitude;
#endif
    return foc_add_sat(qIntegerTerm, qFractionTerm);
}
