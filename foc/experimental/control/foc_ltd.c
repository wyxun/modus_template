/*******************************************************************************
 * @file    foc_ltd.c
 * @brief   Multi-instance linear tracking differentiator
 ******************************************************************************/

#include "foc_ltd.h"

#include <stddef.h>

foc_result_t foc_ltd_Init(foc_ltd_t *ptLtd,
                          const foc_ltd_params_t *ptParams,
                          foc_scalar_t qInitialPosition)
{
    if (ptLtd == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qMaximumVelocity <= FOC_ZERO ||
        ptParams->qMaximumAcceleration <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptLtd->tParams = *ptParams;
    foc_ltd_Reset(ptLtd, qInitialPosition);
    return FOC_RESULT_OK;
}

void foc_ltd_Reset(foc_ltd_t *ptLtd, foc_scalar_t qPosition)
{
    if (ptLtd == NULL) {
        return;
    }
    ptLtd->qPosition = qPosition;
    ptLtd->qVelocity = FOC_ZERO;
}

foc_scalar_t foc_ltd_Step(foc_ltd_t *ptLtd, foc_scalar_t qTarget)
{
    foc_scalar_t qError;
    foc_scalar_t qDesiredVelocity;
    foc_scalar_t qAcceleration;

    if (ptLtd == NULL) {
        return FOC_ZERO;
    }
    qError = foc_sub_sat(qTarget, ptLtd->qPosition);
    qDesiredVelocity = foc_sat(qError,
                               -ptLtd->tParams.qMaximumVelocity,
                               ptLtd->tParams.qMaximumVelocity);
    qAcceleration = foc_sat(
        foc_sub_sat(qDesiredVelocity, ptLtd->qVelocity),
        -ptLtd->tParams.qMaximumAcceleration,
        ptLtd->tParams.qMaximumAcceleration);
    ptLtd->qVelocity = foc_add_sat(ptLtd->qVelocity, qAcceleration);
    ptLtd->qPosition = foc_add_sat(ptLtd->qPosition, ptLtd->qVelocity);
    return ptLtd->qPosition;
}
