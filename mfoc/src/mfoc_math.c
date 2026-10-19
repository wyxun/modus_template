/**
 * @file mfoc_math.c
 * @brief Scalar square root migrated from foc_math.
 * @author Codex
 * @date 2026-10-19
 */
#include "mfoc_math.h"

#if defined(MFOC_NUMERIC_FLOAT)
#include <math.h>
#endif

mfoc_scalar_t mfoc_sqrt(mfoc_scalar_t qValue)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return qValue <= MFOC_ZERO ? MFOC_ZERO : sqrtf(qValue);
#else
    uint32_t wOperand = 0U;
    uint32_t wResult = 0;
    uint32_t wBit = 1uL << 30;
    uint32_t wScaled = 0U;
    uint32_t wResultScale = 1U;

    if (qValue <= MFOC_ZERO) {
        return MFOC_ZERO;
    }
    /* Keep the Q15 radicand inside 32 bits.  Dividing the input by four
     * divides its square root by two, so the final scale is exact and avoids
     * 64-bit arithmetic on Cortex-M0/RV32 targets. */
    wScaled = (uint32_t)qValue;
    while (wScaled > (UINT32_MAX >> MFOC_Q_FRACTION_BITS)) {
        wScaled = (wScaled + 2U) >> 2;
        wResultScale <<= 1;
    }
    wOperand = wScaled << MFOC_Q_FRACTION_BITS;
    while (wBit > wOperand) {
        wBit >>= 2;
    }
    while (wBit != 0u) {
        if (wOperand >= wResult + wBit) {
            wOperand -= wResult + wBit;
            wResult = (wResult >> 1) + wBit;
        } else {
            wResult >>= 1;
        }
        wBit >>= 2;
    }
    return (mfoc_scalar_t)(wResult * wResultScale);
#endif
}
