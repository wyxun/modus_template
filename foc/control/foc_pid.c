/*******************************************************************************
 * @file    foc_pid.c
 * @brief   Multi-instance PID controller with conditional anti-windup
 ******************************************************************************/

#include "foc_pid.h"

#include <stddef.h>

foc_result_t foc_pid_Init(foc_pid_t *ptPid,
                          const foc_pid_params_t *ptParams)
{
    if (ptPid == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_gain_IsValid(&ptParams->tKp) ||
        !foc_gain_IsValid(&ptParams->tKiTs) ||
        !foc_gain_IsValid(&ptParams->tKdOverTs) ||
        !foc_scalar_is_finite(ptParams->qOutputMinimum) ||
        !foc_scalar_is_finite(ptParams->qOutputMaximum) ||
        !foc_scalar_is_finite(ptParams->qIntegratorMinimum) ||
        !foc_scalar_is_finite(ptParams->qIntegratorMaximum) ||
        ptParams->qOutputMinimum >= ptParams->qOutputMaximum ||
        ptParams->qIntegratorMinimum > ptParams->qIntegratorMaximum) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptPid->tParams = *ptParams;
    foc_pid_Reset(ptPid);
    return FOC_RESULT_OK;
}

void foc_pid_Reset(foc_pid_t *ptPid)
{
    if (ptPid == NULL) {
        return;
    }
    ptPid->qIntegrator = FOC_ZERO;
    ptPid->qPreviousError = FOC_ZERO;
}

void foc_pid_Track(foc_pid_t *ptPid,
                   foc_scalar_t qOutput,
                   foc_scalar_t qReference,
                   foc_scalar_t qFeedback)
{
    foc_scalar_t qError;
    foc_scalar_t qProportional;
    foc_scalar_t qIntegralIncrement;

    if (ptPid == NULL) {
        return;
    }
    qError = foc_sat(foc_sub_sat(qReference, qFeedback),
                     FOC_NEG_ONE, FOC_ONE);
    qProportional = foc_gain_apply(&ptPid->tParams.tKp, qError);
    qIntegralIncrement = foc_gain_apply(
        &ptPid->tParams.tKiTs, qError);
    ptPid->qIntegrator = foc_sat(
        foc_sub_sat(
            foc_sub_sat(qOutput, qProportional),
            qIntegralIncrement),
        ptPid->tParams.qIntegratorMinimum,
        ptPid->tParams.qIntegratorMaximum);
    ptPid->qPreviousError = qError;
}

foc_scalar_t foc_pid_Step(foc_pid_t *ptPid,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback)
{
    foc_scalar_t qError;
    foc_scalar_t qProportional;
    foc_scalar_t qDerivative;
    foc_scalar_t qCandidateIntegrator;
    foc_scalar_t qCandidateOutput;
    bool bWindupHigh;
    bool bWindupLow;

    if (ptPid == NULL) {
        return FOC_ZERO;
    }
    qError = foc_sat(foc_sub_sat(qReference, qFeedback),
                     FOC_NEG_ONE, FOC_ONE);
    qProportional = foc_gain_apply(&ptPid->tParams.tKp, qError);
    qDerivative = foc_gain_apply(
        &ptPid->tParams.tKdOverTs,
        foc_sat(foc_sub_sat(qError, ptPid->qPreviousError),
                FOC_NEG_ONE, FOC_ONE));
    qCandidateIntegrator = foc_add_sat(
        ptPid->qIntegrator,
        foc_gain_apply(&ptPid->tParams.tKiTs, qError));
    qCandidateIntegrator = foc_sat(
        qCandidateIntegrator,
        ptPid->tParams.qIntegratorMinimum,
        ptPid->tParams.qIntegratorMaximum);
    qCandidateOutput = foc_add_sat(
        foc_add_sat(qProportional, qCandidateIntegrator), qDerivative);
    bWindupHigh = qCandidateOutput > ptPid->tParams.qOutputMaximum &&
                  qError > FOC_ZERO;
    bWindupLow = qCandidateOutput < ptPid->tParams.qOutputMinimum &&
                 qError < FOC_ZERO;
    if (!bWindupHigh && !bWindupLow) {
        ptPid->qIntegrator = qCandidateIntegrator;
    }
    ptPid->qPreviousError = qError;
    qCandidateOutput = foc_add_sat(
        foc_add_sat(qProportional, ptPid->qIntegrator), qDerivative);
    return foc_sat(qCandidateOutput,
                   ptPid->tParams.qOutputMinimum,
                   ptPid->tParams.qOutputMaximum);
}
