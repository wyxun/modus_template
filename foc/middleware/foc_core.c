/****************************************************************************
 * @file    foc_core.c
 * @brief   Architecture-independent Clarke and Park transforms
 * @author  Codex
 * @date    2026-10-09
 ************************************************************************** */

#include "foc_core.h"
#include "foc_modulation.h"

#include <stddef.h>

foc_result_t foc_core_Init(foc_core_state_t *ptState,
    const foc_pid_params_t *ptPi, foc_scalar_t qVoltageLimit)
{
    if (ptState == NULL || ptPi == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_scalar_is_finite(qVoltageLimit) ||
        qVoltageLimit <= FOC_ZERO || qVoltageLimit > FOC_ONE) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    foc_result_t eResult = foc_pid_Init(&ptState->tIdPi, ptPi);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptState->tIqPi, ptPi);
    }
    ptState->qVoltageLimitSquared = foc_mul_pu(qVoltageLimit, qVoltageLimit);
    return eResult;
}

void foc_core_Reset(foc_core_state_t *ptState, foc_core_output_t *ptOutput)
{
    if (ptState == NULL || ptOutput == NULL) {
        return;
    }
    foc_pid_Reset(&ptState->tIdPi);
    foc_pid_Reset(&ptState->tIqPi);
    ptOutput->tCurrentAlphaBeta = (foc_ab_t){FOC_ZERO, FOC_ZERO};
    ptOutput->tCurrent = (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptOutput->tVoltage = (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptOutput->tVoltageAlphaBeta = (foc_ab_t){FOC_ZERO, FOC_ZERO};
    ptOutput->tDuty = (foc_duty_abc_t){FOC_HALF, FOC_HALF, FOC_HALF};
    ptOutput->bVoltageLimited = false;
    ptOutput->bPwmSaturated = false;
}

foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput,
                           foc_core_output_t *ptOutput)
{
    foc_scalar_t qSin = FOC_ZERO;
    foc_scalar_t qCos = FOC_ZERO;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptState == NULL || ptCommand == NULL || ptInput == NULL ||
        ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptCommand->eMode == FOC_MODE_POSITION) {
        return FOC_RESULT_DISABLED;
    }
    if (!ptInput->bAngleValid) {
        return FOC_RESULT_SAFETY;
    }
    if (ptCommand->eMode >= FOC_MODE_MAX) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptOutput->tCurrentAlphaBeta = ptInput->tCurrentAlphaBeta;
    foc_angle_sincos(ptInput->tElectricalAngle, &qSin, &qCos);
    eResult = foc_park_cached(&ptOutput->tCurrentAlphaBeta, qSin, qCos,
                              &ptOutput->tCurrent);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (ptCommand->eMode == FOC_MODE_VOLTAGE) {
        ptOutput->tVoltage = ptCommand->tVoltageReference;
    } else {
        ptOutput->tVoltage.qD = foc_pid_Step(&ptState->tIdPi,
            ptCommand->tCurrentReference.qD, ptOutput->tCurrent.qD);
        ptOutput->tVoltage.qQ = foc_pid_Step(&ptState->tIqPi,
            ptCommand->tCurrentReference.qQ, ptOutput->tCurrent.qQ);
    }
    eResult = foc_ipark_cached(&ptOutput->tVoltage, qSin, qCos,
                               &ptOutput->tVoltageAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    /* Keep the previous Motor criterion; Core owns its PI boundaries. */
    ptOutput->bVoltageLimited =
        ptOutput->tVoltage.qD <= ptState->tIdPi.tParams.qOutputMinimum ||
        ptOutput->tVoltage.qD >= ptState->tIdPi.tParams.qOutputMaximum ||
        ptOutput->tVoltage.qQ <= ptState->tIqPi.tParams.qOutputMinimum ||
        ptOutput->tVoltage.qQ >= ptState->tIqPi.tParams.qOutputMaximum ||
        foc_add_sat(
            foc_mul_pu(ptOutput->tVoltage.qD, ptOutput->tVoltage.qD),
            foc_mul_pu(ptOutput->tVoltage.qQ, ptOutput->tVoltage.qQ)) >=
                ptState->qVoltageLimitSquared;
    return foc_svpwm(&ptOutput->tVoltageAlphaBeta, &ptOutput->tDuty,
                     &ptOutput->bPwmSaturated);
}

foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB)
{
    const foc_scalar_t qInvSqrtThree = FOC_SCALAR(0.5773502692f);

    if (ptAB == NULL) {
        return FOC_RESULT_NULL;
    }
    ptAB->qAlpha = qIu;
    ptAB->qBeta = foc_sub_sat(foc_mul_wide(qIv, qInvSqrtThree),
                              foc_mul_wide(qIw, qInvSqrtThree));
    return FOC_RESULT_OK;
}

foc_result_t foc_park_cached(const foc_ab_t *ptAB,
                             foc_scalar_t qSin,
                             foc_scalar_t qCos,
                             foc_dq_t *ptDQ)
{
    if (ptAB == NULL || ptDQ == NULL) {
        return FOC_RESULT_NULL;
    }
    ptDQ->qD = foc_add_sat(foc_mul_wide(ptAB->qAlpha, qCos),
                            foc_mul_wide(ptAB->qBeta, qSin));
    ptDQ->qQ = foc_sub_sat(foc_mul_wide(ptAB->qBeta, qCos),
                            foc_mul_wide(ptAB->qAlpha, qSin));
    return FOC_RESULT_OK;
}

foc_result_t foc_park(const foc_ab_t *ptAB,
                      foc_angle_t tTheta,
                      foc_dq_t *ptDQ)
{
    return foc_park_cached(ptAB, foc_angle_sin(tTheta),
                           foc_angle_cos(tTheta), ptDQ);
}

foc_result_t foc_ipark_cached(const foc_dq_t *ptDQ,
                              foc_scalar_t qSin,
                              foc_scalar_t qCos,
                              foc_ab_t *ptAB)
{
    if (ptDQ == NULL || ptAB == NULL) {
        return FOC_RESULT_NULL;
    }
    ptAB->qAlpha = foc_sub_sat(foc_mul_pu(ptDQ->qD, qCos),
                               foc_mul_pu(ptDQ->qQ, qSin));
    ptAB->qBeta = foc_add_sat(foc_mul_pu(ptDQ->qD, qSin),
                              foc_mul_pu(ptDQ->qQ, qCos));
    return FOC_RESULT_OK;
}

foc_result_t foc_ipark(const foc_dq_t *ptDQ,
                       foc_angle_t tTheta,
                       foc_ab_t *ptAB)
{
    return foc_ipark_cached(ptDQ, foc_angle_sin(tTheta),
                            foc_angle_cos(tTheta), ptAB);
}
