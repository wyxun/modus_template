/*******************************************************************************
 * @file    foc_smc.c
 * @brief   Bounded sliding-mode control with conditional anti-windup
 ******************************************************************************/

#include "foc_smc.h"

#include <stddef.h>

static foc_scalar_t smc_sign(foc_scalar_t qValue)
{
    return qValue > FOC_ZERO ? FOC_ONE :
           (qValue < FOC_ZERO ? FOC_NEG_ONE : FOC_ZERO);
}

foc_result_t foc_smc_Init(foc_smc_t *ptSmc,
                          const foc_smc_params_t *ptParams)
{
    if (ptSmc == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qOutputMinimum >= ptParams->qOutputMaximum) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!foc_gain_IsValid(&ptParams->tDerivative) ||
        !foc_gain_IsValid(&ptParams->tSurface) ||
        !foc_gain_IsValid(&ptParams->tDiscontinuous) ||
        !foc_gain_IsValid(&ptParams->tReach) ||
        !foc_gain_IsValid(&ptParams->tPlant) ||
        !foc_gain_IsValid(&ptParams->tIntegrator)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptSmc->tParams = *ptParams;
    foc_smc_Reset(ptSmc);
    return FOC_RESULT_OK;
}

void foc_smc_Reset(foc_smc_t *ptSmc)
{
    if (ptSmc != NULL) {
        ptSmc->qPreviousError = FOC_ZERO;
        ptSmc->qIntegrator = FOC_ZERO;
    }
}

foc_scalar_t foc_smc_Step(foc_smc_t *ptSmc,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback)
{
    foc_scalar_t qError;
    foc_scalar_t qDerivative;
    foc_scalar_t qSurface;
    foc_scalar_t qDrive;
    foc_scalar_t qCandidate;
    bool bWindupHigh;
    bool bWindupLow;

    if (ptSmc == NULL) {
        return FOC_ZERO;
    }
    qError = foc_sat(foc_sub_sat(qReference, qFeedback),
                     FOC_NEG_ONE, FOC_ONE);
    qDerivative = foc_gain_apply(
        &ptSmc->tParams.tDerivative,
        foc_sub_sat(qError, ptSmc->qPreviousError));
    qSurface = foc_add_sat(
        foc_gain_apply(&ptSmc->tParams.tSurface, qError), qDerivative);
    qDrive = foc_add_sat(
        foc_gain_apply(&ptSmc->tParams.tDiscontinuous, smc_sign(qSurface)),
        foc_gain_apply(&ptSmc->tParams.tReach, qSurface));
    qDrive = foc_gain_apply(&ptSmc->tParams.tPlant,
                            foc_sat(qDrive, FOC_NEG_ONE, FOC_ONE));
    qCandidate = foc_add_sat(
        ptSmc->qIntegrator,
        foc_gain_apply(&ptSmc->tParams.tIntegrator, qDrive));
    bWindupHigh = qCandidate > ptSmc->tParams.qOutputMaximum &&
                  qDrive > FOC_ZERO;
    bWindupLow = qCandidate < ptSmc->tParams.qOutputMinimum &&
                 qDrive < FOC_ZERO;
    if (!bWindupHigh && !bWindupLow) {
        ptSmc->qIntegrator = qCandidate;
    }
    ptSmc->qPreviousError = qError;
    return foc_sat(ptSmc->qIntegrator,
                   ptSmc->tParams.qOutputMinimum,
                   ptSmc->tParams.qOutputMaximum);
}
