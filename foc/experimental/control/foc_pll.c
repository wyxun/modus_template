/*******************************************************************************
 * @file    foc_pll.c
 * @brief   Multi-instance normalized-angle phase-locked loop
 ******************************************************************************/

#include "foc_pll.h"

#include <stddef.h>

foc_result_t foc_pll_Init(foc_pll_t *ptPll,
                          const foc_pll_params_t *ptParams)
{
    foc_pid_params_t tPidParams;

    if (ptPll == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qMaximumSpeed <= FOC_ZERO ||
        ptParams->qLockError <= FOC_ZERO ||
        ptParams->qUnlockError <= ptParams->qLockError ||
        ptParams->hwLockSamples == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptPll->tParams = *ptParams;
    tPidParams.tKp = ptParams->tKp;
    tPidParams.tKiTs = ptParams->tKi;
    (void)foc_gain_Init(&tPidParams.tKdOverTs, 0, FOC_ZERO);
    tPidParams.qOutputMinimum = -ptParams->qMaximumSpeed;
    tPidParams.qOutputMaximum = ptParams->qMaximumSpeed;
    tPidParams.qIntegratorMinimum = -ptParams->qMaximumSpeed;
    tPidParams.qIntegratorMaximum = ptParams->qMaximumSpeed;
    if (foc_pid_Init(&ptPll->tLoopFilter, &tPidParams) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    foc_pll_Reset(ptPll, foc_angle_from_scalar(FOC_ZERO));
    return FOC_RESULT_OK;
}

void foc_pll_Reset(foc_pll_t *ptPll, foc_angle_t tInitialAngle)
{
    if (ptPll == NULL) {
        return;
    }
    foc_pid_Reset(&ptPll->tLoopFilter);
    ptPll->tAngle = tInitialAngle;
    ptPll->qSpeed = FOC_ZERO;
    ptPll->hwLockCounter = 0U;
    ptPll->bIsLocked = false;
}

foc_result_t foc_pll_Step(foc_pll_t *ptPll,
                          foc_angle_t tMeasuredAngle)
{
    foc_scalar_t qPhaseError;

    if (ptPll == NULL) {
        return FOC_RESULT_NULL;
    }
    qPhaseError = foc_angle_diff(tMeasuredAngle, ptPll->tAngle);
    ptPll->qSpeed = foc_pid_Step(&ptPll->tLoopFilter,
                                 qPhaseError, FOC_ZERO);
    ptPll->tAngle = foc_angle_add_scalar(ptPll->tAngle, ptPll->qSpeed);

    if (foc_abs(qPhaseError) <= ptPll->tParams.qLockError) {
        if (ptPll->hwLockCounter < ptPll->tParams.hwLockSamples) {
            ptPll->hwLockCounter++;
        }
        if (ptPll->hwLockCounter >= ptPll->tParams.hwLockSamples) {
            ptPll->bIsLocked = true;
        }
    } else {
        ptPll->hwLockCounter = 0U;
        if (foc_abs(qPhaseError) >= ptPll->tParams.qUnlockError) {
            ptPll->bIsLocked = false;
        }
    }
    return FOC_RESULT_OK;
}
