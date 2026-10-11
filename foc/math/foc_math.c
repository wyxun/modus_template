/*******************************************************************************
 * @file    foc_math.c
 * @brief   Scalar square root for the unified FOC numeric backend
 ******************************************************************************/

#include "foc_math.h"

#if defined(FOC_NUMERIC_FLOAT)
#include <math.h>
#endif

foc_scalar_t foc_sqrt(foc_scalar_t qValue)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qValue <= FOC_ZERO ? FOC_ZERO : sqrtf(qValue);
#else
    uint32_t wOperand;
    uint32_t wResult = 0;
    uint32_t wBit = 1uL << 30;
    uint32_t wScaled;
    uint32_t wResultScale = 1U;

    if (qValue <= FOC_ZERO) {
        return FOC_ZERO;
    }
    /* Keep the Q15 radicand inside 32 bits.  Dividing the input by four
     * divides its square root by two, so the final scale is exact and avoids
     * 64-bit arithmetic on Cortex-M0/RV32 targets. */
    wScaled = (uint32_t)qValue;
    while (wScaled > (UINT32_MAX >> FOC_Q_FRACTION_BITS)) {
        wScaled = (wScaled + 2U) >> 2;
        wResultScale <<= 1;
    }
    wOperand = wScaled << FOC_Q_FRACTION_BITS;
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
    return (foc_scalar_t)(wResult * wResultScale);
#endif
}
