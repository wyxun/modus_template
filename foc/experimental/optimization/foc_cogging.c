/*******************************************************************************
 * @file    foc_cogging.c
 * @brief   Periodic linear interpolation without global calibration storage
 ******************************************************************************/

#include "foc_cogging.h"

#include <stddef.h>

foc_result_t foc_cogging_Init(foc_cogging_t *ptCogging,
                              const foc_scalar_t *pqTable,
                              uint16_t hwCount)
{
    if (ptCogging == NULL || pqTable == NULL) {
        return FOC_RESULT_NULL;
    }
    if (hwCount < 2U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptCogging->pqTable = pqTable;
    ptCogging->hwCount = hwCount;
    return FOC_RESULT_OK;
}

foc_result_t foc_cogging_Get(const foc_cogging_t *ptCogging,
                             foc_angle_t tMechanicalAngle,
                             foc_scalar_t *pqCompensation)
{
    uint16_t hwIndex;
    uint16_t hwNext;
    foc_scalar_t qFraction;
    foc_scalar_t qDelta;

    if (ptCogging == NULL || pqCompensation == NULL ||
        ptCogging->pqTable == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptCogging->hwCount < 2U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    {
        uint64_t wPosition = (uint64_t)tMechanicalAngle.wBam32 *
                             (uint64_t)ptCogging->hwCount;
        hwIndex = (uint16_t)(wPosition >> 32);
        uint32_t wFraction32 = (uint32_t)wPosition;
#if defined(FOC_NUMERIC_FLOAT)
        qFraction = (float)wFraction32 * (1.0f / 4294967296.0f);
#else
        qFraction = (foc_scalar_t)(wFraction32 >> 17);
#endif
    }
    if (hwIndex >= ptCogging->hwCount) {
        hwIndex = 0U;
        qFraction = FOC_ZERO;
    }
    hwNext = (uint16_t)(hwIndex + 1U);
    if (hwNext >= ptCogging->hwCount) {
        hwNext = 0U;
    }
    qDelta = foc_sub_sat(ptCogging->pqTable[hwNext],
                         ptCogging->pqTable[hwIndex]);
    *pqCompensation = foc_add_sat(
        ptCogging->pqTable[hwIndex], foc_mul_pu(qDelta, qFraction));
    return FOC_RESULT_OK;
}
