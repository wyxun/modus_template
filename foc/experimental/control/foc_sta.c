/*******************************************************************************
 * @file    foc_sta.c
 * @brief   Bounded discrete super-twisting control
 ******************************************************************************/

#include "foc_sta.h"

#include <stddef.h>

#include "foc_math.h"

foc_result_t foc_sta_Init(foc_sta_t *ptSta,
                          const foc_sta_params_t *ptParams)
{
    if (ptSta == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qIntegratorMinimum > ptParams->qIntegratorMaximum ||
        ptParams->qOutputMinimum >= ptParams->qOutputMaximum) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!foc_gain_IsValid(&ptParams->tK1) ||
        !foc_gain_IsValid(&ptParams->tK2Ts) ||
        !foc_gain_IsValid(&ptParams->tBoundaryInverse)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptSta->tParams = *ptParams;
    foc_sta_Reset(ptSta);
    return FOC_RESULT_OK;
}

void foc_sta_Reset(foc_sta_t *ptSta)
{
    if (ptSta != NULL) {
        ptSta->qIntegrator = FOC_ZERO;
    }
}

foc_scalar_t foc_sta_Step(foc_sta_t *ptSta,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback)
{
    foc_scalar_t qError;
    foc_scalar_t qSign;
    foc_scalar_t qNonlinear;
    foc_scalar_t qOutput;

    if (ptSta == NULL) {
        return FOC_ZERO;
    }
    qError = foc_sat(foc_sub_sat(qReference, qFeedback),
                     FOC_NEG_ONE, FOC_ONE);
    /* SguanFOC v3.1.0 compares an array address here.  Use the current
     * scalar error so the boundary layer works on all architectures. */
    qSign = foc_sat(
        foc_gain_apply(&ptSta->tParams.tBoundaryInverse, qError),
        FOC_NEG_ONE, FOC_ONE);
    ptSta->qIntegrator = foc_sat(
        foc_add_sat(ptSta->qIntegrator,
                    foc_gain_apply(&ptSta->tParams.tK2Ts, qSign)),
        ptSta->tParams.qIntegratorMinimum,
        ptSta->tParams.qIntegratorMaximum);
    qNonlinear = foc_gain_apply(
        &ptSta->tParams.tK1,
        foc_mul_pu(foc_sqrt(foc_abs(qError)), qSign));
    qOutput = foc_add_sat(qNonlinear, ptSta->qIntegrator);
    return foc_sat(qOutput, ptSta->tParams.qOutputMinimum,
                   ptSta->tParams.qOutputMaximum);
}
