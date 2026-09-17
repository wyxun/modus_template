/****************************************************************************
 * @file    foc_identify.c
 * @brief   MESC-aligned motor parameter identification controller.
 * @author  Antigravity
 * @date    2026-09-17
 ****************************************************************************/

#include "foc_identify.h"
#include <math.h>

#define IDENTIFY_MAX_SAT_COUNT 64U

static bool _foc_identify_IsGainPositive(const foc_gain_t *ptGain)
{
    if ((ptGain == NULL) || (!foc_gain_IsValid(ptGain))) {
        return false;
    }
    if (ptGain->nInteger > 0) {
        return true;
    }
    if ((ptGain->nInteger == 0) && (ptGain->qFraction > FOC_ZERO)) {
        return true;
    }
    return false;
}

static bool _foc_identify_IsGainZero(const foc_gain_t *ptGain)
{
    if ((ptGain == NULL) || (!foc_gain_IsValid(ptGain))) {
        return false;
    }
    return (ptGain->nInteger == 0) && (ptGain->qFraction == FOC_ZERO);
}

static foc_result_t _foc_identify_ValidateConfig(
    const foc_identify_cfg_t *ptConfig)
{
    const foc_identify_excitation_cfg_t *ptEx = NULL;
    const foc_identify_timing_cfg_t *ptTm = NULL;
    const foc_identify_acceptance_cfg_t *ptAc = NULL;
    const foc_pid_params_t *ptPi = NULL;
    foc_scalar_t qDiff = FOC_ZERO;

    if (ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    ptEx = &ptConfig->tExcitation;
    ptTm = &ptConfig->tTiming;
    ptAc = &ptConfig->tAcceptance;
    ptPi = &ptConfig->tCurrentPi;

    /* Check finite */
    if (!foc_scalar_is_finite(ptEx->qCurrentLow) ||
        !foc_scalar_is_finite(ptEx->qCurrentHigh) ||
        !foc_scalar_is_finite(ptEx->qInjectionVoltage) ||
        !foc_scalar_is_finite(ptEx->qVoltageLimit) ||
        !foc_scalar_is_finite(ptEx->qCurrentLimit) ||
        !foc_scalar_is_finite(ptTm->qRadiansPerSample) ||
        !foc_scalar_is_finite(ptAc->qCurrentTolerance) ||
        !foc_scalar_is_finite(ptAc->qSlopeTolerance) ||
        !foc_scalar_is_finite(ptAc->qZeroCurrent) ||
        !foc_scalar_is_finite(ptAc->qMinDeltaCurrent) ||
        !foc_scalar_is_finite(ptAc->qMaxElectricalDisplacement) ||
        !foc_scalar_is_finite(ptAc->qMaxElectricalSpeedPu) ||
        !foc_scalar_is_finite(ptAc->qResistanceMinPu) ||
        !foc_scalar_is_finite(ptAc->qResistanceMaxPu) ||
        !foc_scalar_is_finite(ptAc->qInductanceMinPu) ||
        !foc_scalar_is_finite(ptAc->qInductanceMaxPu) ||
        !foc_scalar_is_finite(ptAc->qMaxPairSpread)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* 0 < low < high < currentLimit */
    if ((ptEx->qCurrentLow <= FOC_ZERO) ||
        (ptEx->qCurrentHigh <= ptEx->qCurrentLow) ||
        (ptEx->qCurrentLimit <= ptEx->qCurrentHigh)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* high - low >= 4 * currentTolerance */
    qDiff = foc_sub_sat(ptEx->qCurrentHigh, ptEx->qCurrentLow);
    if (foc_to_float(qDiff) < (foc_to_float(ptAc->qCurrentTolerance) * 4.0f)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* Tolerances & thresholds > 0 and < currentLimit */
    if ((ptAc->qCurrentTolerance <= FOC_ZERO) ||
        (ptAc->qCurrentTolerance >= ptEx->qCurrentLimit) ||
        (ptAc->qZeroCurrent <= FOC_ZERO) ||
        (ptAc->qZeroCurrent >= ptEx->qCurrentLimit) ||
        (ptAc->qMinDeltaCurrent <= FOC_ZERO) ||
        (ptAc->qMinDeltaCurrent >= ptEx->qCurrentLimit)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* Displacement in (0, 0.5) electrical turns */
    if ((ptAc->qMaxElectricalDisplacement <= FOC_ZERO) ||
        (ptAc->qMaxElectricalDisplacement >= foc_from_float(0.5f))) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* Speed, slope, pair spread > 0 */
    if ((ptAc->qMaxElectricalSpeedPu <= FOC_ZERO) ||
        (ptAc->qSlopeTolerance <= FOC_ZERO) ||
        (ptAc->qMaxPairSpread <= FOC_ZERO)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* 0 < qRadiansPerSample <= 1.0 */
    if ((ptTm->qRadiansPerSample <= FOC_ZERO) ||
        (ptTm->qRadiansPerSample > foc_from_float(1.0f))) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* Voltage limits: qVoltageLimit <= 0.25, 0 < Vinj < qVoltageLimit */
    if ((ptEx->qVoltageLimit <= FOC_ZERO) ||
        (ptEx->qVoltageLimit > foc_from_float(0.25f)) ||
        (ptEx->qInjectionVoltage <= FOC_ZERO) ||
        (ptEx->qInjectionVoltage >= ptEx->qVoltageLimit)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* Timing counts */
    if ((ptTm->wStableTicks < 1U) ||
        (ptTm->wSettleMinTicks < ptTm->wStableTicks) ||
        (ptTm->wStageTimeoutTicks < ptTm->wSettleMinTicks) ||
        (ptTm->wTotalTimeoutTicks < ptTm->wStageTimeoutTicks)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if ((ptTm->wAverageTicks < 1U) || (ptTm->wAverageTicks > 4096U) ||
        (ptTm->wHalfPeriodTicks < 8U) || (ptTm->wHalfPeriodTicks > 64U) ||
        (ptTm->wDiscardPairs < 1U) || (ptTm->wDiscardPairs > 128U) ||
        (ptTm->wMeasurePairs < 8U) || (ptTm->wMeasurePairs > 1024U)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* Acceptance bounds */
    if ((ptAc->qResistanceMinPu <= FOC_ZERO) ||
        (ptAc->qResistanceMaxPu <= ptAc->qResistanceMinPu) ||
        (foc_to_float(ptAc->qResistanceMaxPu) > 5.0f) ||
        (ptAc->qInductanceMinPu <= FOC_ZERO) ||
        (ptAc->qInductanceMaxPu <= ptAc->qInductanceMinPu) ||
        (foc_to_float(ptAc->qInductanceMaxPu) > 5.0f)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    /* PI parameters */
    if (!_foc_identify_IsGainPositive(&ptPi->tKp) ||
        !_foc_identify_IsGainPositive(&ptPi->tKiTs) ||
        !_foc_identify_IsGainZero(&ptPi->tKdOverTs)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if ((ptPi->qOutputMaximum != ptEx->qVoltageLimit) ||
        (ptPi->qOutputMinimum != foc_sub_sat(FOC_ZERO, ptEx->qVoltageLimit))) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    return FOC_RESULT_OK;
}

static void _foc_identify_EnterError(
    foc_identify_t *ptIdentify,
    foc_identify_fail_reason_e eReason,
    foc_result_t eResult)
{
    ptIdentify->tDiagnostics.eFailureStage = ptIdentify->eStage;
    ptIdentify->tDiagnostics.eFailureReason = eReason;
    ptIdentify->tDiagnostics.eFailureResult = eResult;

    ptIdentify->eState = FOC_IDENTIFY_STATE_ERROR;
    ptIdentify->eStage = FOC_IDENTIFY_STATUS_ERROR;
    ptIdentify->eFailureReason = eReason;
    ptIdentify->eLastError = eResult;

    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ERROR;
    ptIdentify->tOutput.bStopPwm = true;
    ptIdentify->tOutput.bRefChanged = true;
    ptIdentify->tOutput.tVoltageRefPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};
}

foc_result_t foc_identify_Init(
    foc_identify_t *ptIdentify,
    const foc_identify_cfg_t *ptConfig)
{
    foc_result_t eRes = FOC_RESULT_OK;

    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    eRes = _foc_identify_ValidateConfig(ptConfig);
    if (eRes != FOC_RESULT_OK) {
        return eRes;
    }

    *ptIdentify = (foc_identify_t){0};
    ptIdentify->tConfig = *ptConfig;

    (void)foc_pid_Init(&ptIdentify->tIdPi, &ptIdentify->tConfig.tCurrentPi);
    (void)foc_pid_Init(&ptIdentify->tIqPi, &ptIdentify->tConfig.tCurrentPi);

    ptIdentify->eState = FOC_IDENTIFY_STATE_IDLE;
    ptIdentify->eStage = FOC_IDENTIFY_STATUS_IDLE;
    ptIdentify->bStoppedConfirmed = true;

    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_IDLE;
    ptIdentify->tOutput.bStopPwm = true;
    ptIdentify->tOutput.bRefChanged = false;
    ptIdentify->tOutput.tVoltageRefPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};

    return FOC_RESULT_OK;
}

foc_result_t foc_identify_Start(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState != FOC_IDENTIFY_STATE_IDLE) {
        return FOC_RESULT_BUSY;
    }

    (void)foc_pid_Init(&ptIdentify->tIdPi, &ptIdentify->tConfig.tCurrentPi);
    (void)foc_pid_Init(&ptIdentify->tIqPi, &ptIdentify->tConfig.tCurrentPi);

    ptIdentify->tResult = (foc_identify_result_t){0};
    ptIdentify->tDiagnostics = (foc_identify_diagnostics_t){0};
    ptIdentify->eLastError = FOC_RESULT_OK;
    ptIdentify->eFailureReason = FOC_IDENTIFY_FAIL_NONE;
    ptIdentify->bStoppedConfirmed = false;

    ptIdentify->wStageTicks = 0U;
    ptIdentify->wTotalTicks = 0U;
    ptIdentify->wStableCount = 0U;
    ptIdentify->wAverageCount = 0U;
    ptIdentify->wHalfPeriodCount = 0U;
    ptIdentify->wPairCount = 0U;
    ptIdentify->wSatCount = 0U;
    ptIdentify->bPiFrozen = false;
    ptIdentify->bPositiveHalf = true;
    ptIdentify->bPriorCurrentValid = false;

    ptIdentify->dSumI_low = 0.0;
    ptIdentify->dSumV_low = 0.0;
    ptIdentify->dSumI_high = 0.0;
    ptIdentify->dSumV_high = 0.0;
    ptIdentify->tBiasVoltageDqPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};

    ptIdentify->eState = FOC_IDENTIFY_STATE_RUNNING;
    ptIdentify->eStage = FOC_IDENTIFY_STATUS_PRIME;

    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_PRIME;
    ptIdentify->tOutput.bStopPwm = false;
    ptIdentify->tOutput.bRefChanged = true;
    ptIdentify->tOutput.tVoltageRefPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};

    return FOC_RESULT_OK;
}

void foc_identify_Abort(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return;
    }
    _foc_identify_EnterError(
        ptIdentify, FOC_IDENTIFY_FAIL_FAULT, FOC_RESULT_SAFETY);
}

foc_result_t foc_identify_Stop(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_RUNNING) {
        ptIdentify->eState = FOC_IDENTIFY_STATE_STOPPING;
        ptIdentify->eStage = FOC_IDENTIFY_STATUS_STOPPING;
        ptIdentify->eFailureReason = FOC_IDENTIFY_FAIL_CANCEL;

        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_STOPPING;
        ptIdentify->tOutput.bStopPwm = true;
        ptIdentify->tOutput.bRefChanged = true;
        ptIdentify->tOutput.tVoltageRefPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};

        (void)foc_pid_Init(&ptIdentify->tIdPi, &ptIdentify->tConfig.tCurrentPi);
        (void)foc_pid_Init(&ptIdentify->tIqPi, &ptIdentify->tConfig.tCurrentPi);
        return FOC_RESULT_OK;
    }
    /* Idempotent for already stopped or terminal states */
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_ConfirmStopped(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_RUNNING) {
        return FOC_RESULT_BUSY;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_STOPPING) {
        if (ptIdentify->eFailureReason == FOC_IDENTIFY_FAIL_NONE) {
            ptIdentify->eState = FOC_IDENTIFY_STATE_COMPLETE;
            ptIdentify->eStage = FOC_IDENTIFY_STATUS_COMPLETE;
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_COMPLETE;
        } else {
            ptIdentify->eState = FOC_IDENTIFY_STATE_ERROR;
            ptIdentify->eStage = FOC_IDENTIFY_STATUS_ERROR;
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ERROR;
        }
        ptIdentify->bStoppedConfirmed = true;
        return FOC_RESULT_OK;
    }
    if ((ptIdentify->eState == FOC_IDENTIFY_STATE_ERROR) ||
        (ptIdentify->eState == FOC_IDENTIFY_STATE_COMPLETE) ||
        (ptIdentify->eState == FOC_IDENTIFY_STATE_IDLE)) {
        ptIdentify->bStoppedConfirmed = true;
        return FOC_RESULT_OK;
    }
    return FOC_RESULT_INVALID_ARGUMENT;
}

foc_result_t foc_identify_Reset(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if ((ptIdentify->eState == FOC_IDENTIFY_STATE_RUNNING) ||
        (!ptIdentify->bStoppedConfirmed)) {
        return FOC_RESULT_BUSY;
    }
    *ptIdentify = (foc_identify_t){0};
    ptIdentify->eState = FOC_IDENTIFY_STATE_UNINITIALIZED;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_GetStatus(
    const foc_identify_t *ptIdentify,
    foc_identify_status_t *ptStatus)
{
    if ((ptIdentify == NULL) || (ptStatus == NULL)) {
        return FOC_RESULT_NULL;
    }
    ptStatus->eState = ptIdentify->eState;
    ptStatus->eStage = ptIdentify->eStage;
    ptStatus->eFailureReason = ptIdentify->eFailureReason;
    ptStatus->eLastError = ptIdentify->eLastError;
    ptStatus->bStoppedConfirmed = ptIdentify->bStoppedConfirmed;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_GetResult(
    const foc_identify_t *ptIdentify,
    foc_identify_result_t *ptResult)
{
    if ((ptIdentify == NULL) || (ptResult == NULL)) {
        return FOC_RESULT_NULL;
    }
    if ((ptIdentify->eState == FOC_IDENTIFY_STATE_RUNNING) ||
        (ptIdentify->eState == FOC_IDENTIFY_STATE_STOPPING)) {
        return FOC_RESULT_BUSY;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_COMPLETE) {
        *ptResult = ptIdentify->tResult;
        return FOC_RESULT_OK;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_ERROR) {
        return (ptIdentify->eLastError != FOC_RESULT_OK) ?
               ptIdentify->eLastError : FOC_RESULT_SAFETY;
    }
    return FOC_RESULT_INVALID_ARGUMENT;
}

foc_result_t foc_identify_GetDiagnostics(
    const foc_identify_t *ptIdentify,
    foc_identify_diagnostics_t *ptDiagnostics)
{
    if ((ptIdentify == NULL) || (ptDiagnostics == NULL)) {
        return FOC_RESULT_NULL;
    }
    *ptDiagnostics = ptIdentify->tDiagnostics;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_ConsumeTerminal(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if ((ptIdentify->eState == FOC_IDENTIFY_STATE_RUNNING) ||
        (!ptIdentify->bStoppedConfirmed)) {
        return FOC_RESULT_BUSY;
    }
    if ((ptIdentify->eState == FOC_IDENTIFY_STATE_COMPLETE) ||
        (ptIdentify->eState == FOC_IDENTIFY_STATE_ERROR)) {
        ptIdentify->eState = FOC_IDENTIFY_STATE_IDLE;
        ptIdentify->eStage = FOC_IDENTIFY_STATUS_IDLE;
        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_IDLE;
        ptIdentify->tOutput.bStopPwm = true;
        ptIdentify->tOutput.bRefChanged = false;
        ptIdentify->tOutput.tVoltageRefPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};
        return FOC_RESULT_OK;
    }
    return FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t _foc_identify_StepInductanceAxis(
    foc_identify_t *ptIdentify,
    const foc_identify_input_t *ptInput,
    bool bIsQAxis,
    foc_identify_output_t *ptOutput)
{
    foc_scalar_t qVbias_d = ptIdentify->tBiasVoltageDqPu.qD;
    foc_scalar_t qVbias_q = ptIdentify->tBiasVoltageDqPu.qQ;
    foc_scalar_t qVinj = ptIdentify->tConfig.tExcitation.qInjectionVoltage;
    foc_scalar_t qVlim = ptIdentify->tConfig.tExcitation.qVoltageLimit;
    foc_scalar_t qVd = qVbias_d;
    foc_scalar_t qVq = qVbias_q;
    foc_scalar_t qI_active = bIsQAxis ?
        ptInput->tCurrentDqPu.qQ : ptInput->tCurrentDqPu.qD;
    foc_scalar_t qV_int = bIsQAxis ?
        ptInput->tIntervalVoltageDqPu.qQ : ptInput->tIntervalVoltageDqPu.qD;

    if (!bIsQAxis) {
        qVd = ptIdentify->bPositiveHalf ?
            foc_add_sat(qVbias_d, qVinj) :
            foc_sub_sat(qVbias_d, qVinj);
    } else {
        qVq = ptIdentify->bPositiveHalf ?
            foc_add_sat(qVbias_q, qVinj) :
            foc_sub_sat(qVbias_q, qVinj);
    }

    ptIdentify->tOutput.tVoltageRefPu.qD = qVd;
    ptIdentify->tOutput.tVoltageRefPu.qQ = qVq;
    ptIdentify->tOutput.bRefChanged = true;

    if ((qVd >= qVlim) || (qVd <= foc_sub_sat(FOC_ZERO, qVlim)) ||
        (qVq >= qVlim) || (qVq <= foc_sub_sat(FOC_ZERO, qVlim))) {
        ptIdentify->wSatCount++;
        if (ptIdentify->wSatCount >= IDENTIFY_MAX_SAT_COUNT) {
            _foc_identify_EnterError(
                ptIdentify, FOC_IDENTIFY_FAIL_SATURATION, FOC_RESULT_SAFETY);
            *ptOutput = ptIdentify->tOutput;
            return FOC_RESULT_SAFETY;
        }
    } else {
        ptIdentify->wSatCount = 0U;
    }

    if (ptInput->bIntervalValid && ptIdentify->bPriorCurrentValid) {
        if (ptIdentify->bPositiveHalf) {
            if (ptIdentify->wWindowIntervalsPlus == 0U) {
                ptIdentify->qCurrentStart_plus = ptIdentify->qPriorCurrent;
            }
            ptIdentify->qCurrentEnd_plus = qI_active;
#if defined(FOC_NUMERIC_FLOAT)
            ptIdentify->dSumA_plus += 0.5 *
                ((double)foc_to_float(ptIdentify->qPriorCurrent) +
                 (double)foc_to_float(qI_active));
            ptIdentify->dSumB_plus += (double)foc_to_float(qV_int);
#else
            {
                int64_t mid = (int64_t)ptIdentify->qPriorCurrent +
                              (int64_t)qI_active;
                mid = (mid >= 0) ? ((mid + 1) >> 1) : -((-mid + 1) >> 1);
                ptIdentify->llSumA_plus += mid;
                ptIdentify->llSumB_plus += (int64_t)qV_int;
            }
#endif
            ptIdentify->wWindowIntervalsPlus++;
        } else {
            if (ptIdentify->wWindowIntervalsMinus == 0U) {
                ptIdentify->qCurrentStart_minus = ptIdentify->qPriorCurrent;
            }
            ptIdentify->qCurrentEnd_minus = qI_active;
#if defined(FOC_NUMERIC_FLOAT)
            ptIdentify->dSumA_minus += 0.5 *
                ((double)foc_to_float(ptIdentify->qPriorCurrent) +
                 (double)foc_to_float(qI_active));
            ptIdentify->dSumB_minus += (double)foc_to_float(qV_int);
#else
            {
                int64_t mid = (int64_t)ptIdentify->qPriorCurrent +
                              (int64_t)qI_active;
                mid = (mid >= 0) ? ((mid + 1) >> 1) : -((-mid + 1) >> 1);
                ptIdentify->llSumA_minus += mid;
                ptIdentify->llSumB_minus += (int64_t)qV_int;
            }
#endif
            ptIdentify->wWindowIntervalsMinus++;
        }
    }

    ptIdentify->wHalfPeriodCount++;
    if (ptIdentify->wHalfPeriodCount >=
        ptIdentify->tConfig.tTiming.wHalfPeriodTicks) {
        if (ptIdentify->bPositiveHalf) {
            ptIdentify->bPositiveHalf = false;
            ptIdentify->wHalfPeriodCount = 0U;
            ptIdentify->bPriorCurrentValid = false;
        } else {
            ptIdentify->wPairCount++;
            if (ptIdentify->wPairCount >
                ptIdentify->tConfig.tTiming.wDiscardPairs) {
                if ((ptIdentify->wWindowIntervalsPlus ==
                     ptIdentify->wWindowIntervalsMinus) &&
                    (ptIdentify->wWindowIntervalsPlus > 0U)) {
#if defined(FOC_NUMERIC_FLOAT)
                    double dDeltaA = ptIdentify->dSumA_plus -
                                     ptIdentify->dSumA_minus;
                    double dDeltaB = ptIdentify->dSumB_plus -
                                     ptIdentify->dSumB_minus;
                    double dD_plus =
                        (double)foc_to_float(ptIdentify->qCurrentEnd_plus) -
                        (double)foc_to_float(ptIdentify->qCurrentStart_plus);
                    double dD_minus =
                        (double)foc_to_float(ptIdentify->qCurrentEnd_minus) -
                        (double)foc_to_float(ptIdentify->qCurrentStart_minus);
                    double dD = dD_plus - dD_minus;
                    double dRs = (double)foc_to_float(
                        ptIdentify->tResult.qResistancePu);
                    double dN = dDeltaB - dRs * dDeltaA;
                    double dOmegaTs = 0.0;
                    double dL = 0.0;
                    double dX = 0.0;
                    double dLmin = 0.0;
                    double dLmax = 0.0;
                    foc_scalar_t qL = FOC_ZERO;

                    if ((dD < (double)foc_to_float(
                            ptIdentify->tConfig.tAcceptance.qMinDeltaCurrent)) ||
                        (dN <= 0.0)) {
                        _foc_identify_EnterError(
                            ptIdentify,
                            FOC_IDENTIFY_FAIL_LOW_RESPONSE,
                            FOC_RESULT_SAFETY);
                        *ptOutput = ptIdentify->tOutput;
                        return FOC_RESULT_SAFETY;
                    }

                    dOmegaTs = (double)foc_to_float(
                        ptIdentify->tConfig.tTiming.qRadiansPerSample);
                    dL = dOmegaTs * dN / dD;
                    dX = (dRs * dOmegaTs) / dL;
                    if (dX > 0.25) {
                        _foc_identify_EnterError(
                            ptIdentify,
                            FOC_IDENTIFY_FAIL_NUMERIC_RANGE,
                            FOC_RESULT_SAFETY);
                        *ptOutput = ptIdentify->tOutput;
                        return FOC_RESULT_SAFETY;
                    }

                    dLmin = (double)foc_to_float(
                        ptIdentify->tConfig.tAcceptance.qInductanceMinPu);
                    dLmax = (double)foc_to_float(
                        ptIdentify->tConfig.tAcceptance.qInductanceMaxPu);
                    if ((dL < dLmin) || (dL > dLmax)) {
                        _foc_identify_EnterError(
                            ptIdentify,
                            FOC_IDENTIFY_FAIL_NUMERIC_RANGE,
                            FOC_RESULT_SAFETY);
                        *ptOutput = ptIdentify->tOutput;
                        return FOC_RESULT_SAFETY;
                    }

                    qL = foc_from_float((float)dL);
                    ptIdentify->dL_sum += dL;
                    if (ptIdentify->wValidPairs == 0U) {
                        ptIdentify->qL_min = qL;
                        ptIdentify->qL_max = qL;
                    } else {
                        if (qL < ptIdentify->qL_min) {
                            ptIdentify->qL_min = qL;
                        }
                        if (qL > ptIdentify->qL_max) {
                            ptIdentify->qL_max = qL;
                        }
                    }
                    ptIdentify->wValidPairs++;
#else
                    int64_t deltaA = ptIdentify->llSumA_plus -
                                     ptIdentify->llSumA_minus;
                    int64_t deltaB = ptIdentify->llSumB_plus -
                                     ptIdentify->llSumB_minus;
                    int64_t d_plus =
                        (int64_t)ptIdentify->qCurrentEnd_plus -
                        (int64_t)ptIdentify->qCurrentStart_plus;
                    int64_t d_minus =
                        (int64_t)ptIdentify->qCurrentEnd_minus -
                        (int64_t)ptIdentify->qCurrentStart_minus;
                    int64_t D = d_plus - d_minus;
                    int64_t R_q15 = (int64_t)ptIdentify->tResult.qResistancePu;
                    int64_t r_times_a = R_q15 * deltaA;
                    int64_t r_a_q15 = (r_times_a >= 0) ?
                        ((r_times_a + 16384) >> 15) :
                        -((-r_times_a + 16384) >> 15);
                    int64_t N_q15 = deltaB - r_a_q15;
                    int64_t omegaTs = 0;
                    int64_t num = 0;
                    int64_t L_q15 = 0;
                    int64_t rs_omega = 0;
                    int64_t rs_omega_q15 = 0;
                    int64_t Lmin = 0;
                    int64_t Lmax = 0;
                    foc_scalar_t qL = FOC_ZERO;

                    if ((D < (int64_t)
                         ptIdentify->tConfig.tAcceptance.qMinDeltaCurrent) ||
                        (N_q15 <= 0)) {
                        _foc_identify_EnterError(
                            ptIdentify,
                            FOC_IDENTIFY_FAIL_LOW_RESPONSE,
                            FOC_RESULT_SAFETY);
                        *ptOutput = ptIdentify->tOutput;
                        return FOC_RESULT_SAFETY;
                    }

                    omegaTs =
                        (int64_t)ptIdentify->tConfig.tTiming.qRadiansPerSample;
                    num = omegaTs * N_q15;
                    L_q15 = (num >= 0) ?
                        ((num + (D >> 1)) / D) :
                        -((-num + (D >> 1)) / D);

                    rs_omega = R_q15 * omegaTs;
                    rs_omega_q15 = (rs_omega + 16384) >> 15;
                    if ((rs_omega_q15 * 4) > L_q15) {
                        _foc_identify_EnterError(
                            ptIdentify,
                            FOC_IDENTIFY_FAIL_NUMERIC_RANGE,
                            FOC_RESULT_SAFETY);
                        *ptOutput = ptIdentify->tOutput;
                        return FOC_RESULT_SAFETY;
                    }

                    Lmin = (int64_t)
                        ptIdentify->tConfig.tAcceptance.qInductanceMinPu;
                    Lmax = (int64_t)
                        ptIdentify->tConfig.tAcceptance.qInductanceMaxPu;
                    if ((L_q15 < Lmin) || (L_q15 > Lmax)) {
                        _foc_identify_EnterError(
                            ptIdentify,
                            FOC_IDENTIFY_FAIL_NUMERIC_RANGE,
                            FOC_RESULT_SAFETY);
                        *ptOutput = ptIdentify->tOutput;
                        return FOC_RESULT_SAFETY;
                    }

                    qL = (foc_scalar_t)L_q15;
                    ptIdentify->dL_sum += (double)L_q15;
                    if (ptIdentify->wValidPairs == 0U) {
                        ptIdentify->qL_min = qL;
                        ptIdentify->qL_max = qL;
                    } else {
                        if (qL < ptIdentify->qL_min) {
                            ptIdentify->qL_min = qL;
                        }
                        if (qL > ptIdentify->qL_max) {
                            ptIdentify->qL_max = qL;
                        }
                    }
                    ptIdentify->wValidPairs++;
#endif
                    if (ptIdentify->wValidPairs >=
                        ptIdentify->tConfig.tTiming.wMeasurePairs) {
                        double dMean = ptIdentify->dL_sum /
                                       (double)ptIdentify->wValidPairs;
#if defined(FOC_NUMERIC_FLOAT)
                        double dSpread =
                            ((double)foc_to_float(ptIdentify->qL_max) -
                             (double)foc_to_float(ptIdentify->qL_min)) / dMean;
                        foc_scalar_t qL_final = foc_from_float((float)dMean);
#else
                        double dSpread =
                            ((double)ptIdentify->qL_max -
                             (double)ptIdentify->qL_min) / dMean;
                        foc_scalar_t qL_final =
                            (foc_scalar_t)((int64_t)(dMean + 0.5));
#endif
                        if (dSpread > (double)foc_to_float(
                                ptIdentify->tConfig.tAcceptance.qMaxPairSpread)) {
                            _foc_identify_EnterError(
                                ptIdentify,
                                FOC_IDENTIFY_FAIL_PAIR_SPREAD,
                                FOC_RESULT_SAFETY);
                            *ptOutput = ptIdentify->tOutput;
                            return FOC_RESULT_SAFETY;
                        }

                        if (!bIsQAxis) {
                            ptIdentify->tResult.qInductanceDPu = qL_final;
                            ptIdentify->tDiagnostics.qLdEstimatePu = qL_final;
                            ptIdentify->tDiagnostics.wValidPairsD =
                                ptIdentify->wValidPairs;
                            ptIdentify->bLdCompleted = true;

                            ptIdentify->eStage = FOC_IDENTIFY_STATUS_ZERO;
                            ptIdentify->tOutput.eStatus =
                                FOC_IDENTIFY_STATUS_ZERO;
                            ptIdentify->tOutput.tVoltageRefPu =
                                (foc_dq_t){FOC_ZERO, FOC_ZERO};
                            ptIdentify->tOutput.bRefChanged = true;
                            ptIdentify->wStageTicks = 0U;
                            ptIdentify->wStableCount = 0U;
                            ptIdentify->bPiFrozen = false;
                            ptIdentify->eNextStageAfterZero =
                                FOC_IDENTIFY_STATUS_BIAS;
                        } else {
                            ptIdentify->tResult.qInductanceQPu = qL_final;
                            ptIdentify->tDiagnostics.qLqEstimatePu = qL_final;
                            ptIdentify->tDiagnostics.wValidPairsQ =
                                ptIdentify->wValidPairs;
                            ptIdentify->tResult.bValid = true;
                            ptIdentify->tDiagnostics.bValid = true;

                            ptIdentify->eStage = FOC_IDENTIFY_STATUS_STOPPING;
                            ptIdentify->eState = FOC_IDENTIFY_STATE_STOPPING;
                            ptIdentify->tOutput.eStatus =
                                FOC_IDENTIFY_STATUS_STOPPING;
                            ptIdentify->tOutput.bStopPwm = true;
                            ptIdentify->tOutput.bRefChanged = true;
                            ptIdentify->tOutput.tVoltageRefPu =
                                (foc_dq_t){FOC_ZERO, FOC_ZERO};
                        }
                    }
                }
            }

            ptIdentify->bPositiveHalf = true;
            ptIdentify->wHalfPeriodCount = 0U;
            ptIdentify->wWindowIntervalsPlus = 0U;
            ptIdentify->wWindowIntervalsMinus = 0U;
            ptIdentify->dSumA_plus = 0.0;
            ptIdentify->dSumB_plus = 0.0;
            ptIdentify->dSumA_minus = 0.0;
            ptIdentify->dSumB_minus = 0.0;
            ptIdentify->llSumA_plus = 0;
            ptIdentify->llSumB_plus = 0;
            ptIdentify->llSumA_minus = 0;
            ptIdentify->llSumB_minus = 0;
            ptIdentify->bPriorCurrentValid = false;
        }
    }

    ptIdentify->qPriorCurrent = qI_active;
    ptIdentify->bPriorCurrentValid = true;

    *ptOutput = ptIdentify->tOutput;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_IsrStep(
    foc_identify_t *ptIdentify,
    const foc_identify_input_t *ptInput,
    foc_identify_output_t *ptOutput)
{
    float fId = 0.0f;
    float fIq = 0.0f;
    float fImag2 = 0.0f;
    float fLimit = 0.0f;
    int32_t nAngleDiff = 0;
    float fDisp = 0.0f;

    if ((ptIdentify == NULL) || (ptOutput == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState != FOC_IDENTIFY_STATE_RUNNING) {
        *ptOutput = ptIdentify->tOutput;
        return FOC_RESULT_OK;
    }

    /* Basic input validity & safety checks */
    if ((ptInput == NULL) || (!ptInput->bCurrentValid) || ptInput->bFault) {
        _foc_identify_EnterError(
            ptIdentify, FOC_IDENTIFY_FAIL_FAULT, FOC_RESULT_SAFETY);
        *ptOutput = ptIdentify->tOutput;
        return FOC_RESULT_SAFETY;
    }

    /* Current vector magnitude check */
    fId = foc_to_float(ptInput->tCurrentDqPu.qD);
    fIq = foc_to_float(ptInput->tCurrentDqPu.qQ);
    fImag2 = fId * fId + fIq * fIq;
    fLimit = foc_to_float(ptIdentify->tConfig.tExcitation.qCurrentLimit);
    if (fImag2 > (fLimit * fLimit)) {
        _foc_identify_EnterError(
            ptIdentify, FOC_IDENTIFY_FAIL_OVERCURRENT, FOC_RESULT_SAFETY);
        *ptOutput = ptIdentify->tOutput;
        return FOC_RESULT_SAFETY;
    }

    /* Electrical displacement check */
    if (ptIdentify->eStage == FOC_IDENTIFY_STATUS_PRIME &&
        ptIdentify->wStageTicks == 0U) {
        ptIdentify->tStartAngle = ptInput->tElectricalAngle;
    } else {
        nAngleDiff = (int32_t)(ptInput->tElectricalAngle.wBam32 -
                               ptIdentify->tStartAngle.wBam32);
        fDisp = fabsf((float)nAngleDiff) / 4294967296.0f;
        if (fDisp > foc_to_float(
                ptIdentify->tConfig.tAcceptance.qMaxElectricalDisplacement)) {
            _foc_identify_EnterError(
                ptIdentify, FOC_IDENTIFY_FAIL_MOTION, FOC_RESULT_SAFETY);
            *ptOutput = ptIdentify->tOutput;
            return FOC_RESULT_SAFETY;
        }
    }

    /* Speed check */
    if (fabsf(foc_to_float(ptInput->qElectricalSpeedPu)) >
        foc_to_float(ptIdentify->tConfig.tAcceptance.qMaxElectricalSpeedPu)) {
        _foc_identify_EnterError(
            ptIdentify, FOC_IDENTIFY_FAIL_MOTION, FOC_RESULT_SAFETY);
        *ptOutput = ptIdentify->tOutput;
        return FOC_RESULT_SAFETY;
    }

    /* Timeouts */
    ptIdentify->wTotalTicks++;
    if (ptIdentify->wTotalTicks >
        ptIdentify->tConfig.tTiming.wTotalTimeoutTicks) {
        _foc_identify_EnterError(
            ptIdentify, FOC_IDENTIFY_FAIL_TOTAL_TIMEOUT, FOC_RESULT_SAFETY);
        *ptOutput = ptIdentify->tOutput;
        return FOC_RESULT_SAFETY;
    }

    ptIdentify->wStageTicks++;
    if (ptIdentify->wStageTicks >
        ptIdentify->tConfig.tTiming.wStageTimeoutTicks) {
        _foc_identify_EnterError(
            ptIdentify, FOC_IDENTIFY_FAIL_STAGE_TIMEOUT, FOC_RESULT_SAFETY);
        *ptOutput = ptIdentify->tOutput;
        return FOC_RESULT_SAFETY;
    }

    /* State machine dispatch */
    switch (ptIdentify->eStage) {
    case FOC_IDENTIFY_STATUS_PRIME:
        ptIdentify->tStartAngle = ptInput->tElectricalAngle;
        ptIdentify->eStage = FOC_IDENTIFY_STATUS_RS_LOW;
        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_RS_LOW;
        ptIdentify->wStageTicks = 0U;
        ptIdentify->wStableCount = 0U;
        ptIdentify->wAverageCount = 0U;
        ptIdentify->wSatCount = 0U;
        ptIdentify->bPiFrozen = false;
        ptIdentify->dSumI_low = 0.0;
        ptIdentify->dSumV_low = 0.0;
        foc_pid_Reset(&ptIdentify->tIdPi);
        foc_pid_Reset(&ptIdentify->tIqPi);
        break;

    case FOC_IDENTIFY_STATUS_RS_LOW:
    case FOC_IDENTIFY_STATUS_RS_HIGH: {
        bool bIsLow = (ptIdentify->eStage == FOC_IDENTIFY_STATUS_RS_LOW);
        foc_scalar_t qIdTarget = bIsLow ?
            ptIdentify->tConfig.tExcitation.qCurrentLow :
            ptIdentify->tConfig.tExcitation.qCurrentHigh;
        foc_scalar_t qIqTarget = FOC_ZERO;

        if (!ptIdentify->bPiFrozen) {
            foc_scalar_t qVd = foc_pid_Step(
                &ptIdentify->tIdPi, qIdTarget, ptInput->tCurrentDqPu.qD);
            foc_scalar_t qVq = foc_pid_Step(
                &ptIdentify->tIqPi, qIqTarget, ptInput->tCurrentDqPu.qQ);

            ptIdentify->tOutput.tVoltageRefPu.qD = qVd;
            ptIdentify->tOutput.tVoltageRefPu.qQ = qVq;
            ptIdentify->tOutput.bRefChanged = true;

            /* Saturation check: >= qOutputMaximum or <= qOutputMinimum */
            if ((qVd >= ptIdentify->tConfig.tCurrentPi.qOutputMaximum) ||
                (qVd <= ptIdentify->tConfig.tCurrentPi.qOutputMinimum) ||
                (qVq >= ptIdentify->tConfig.tCurrentPi.qOutputMaximum) ||
                (qVq <= ptIdentify->tConfig.tCurrentPi.qOutputMinimum)) {
                ptIdentify->wSatCount++;
                if (ptIdentify->wSatCount >= IDENTIFY_MAX_SAT_COUNT) {
                    _foc_identify_EnterError(
                        ptIdentify, FOC_IDENTIFY_FAIL_SATURATION, FOC_RESULT_SAFETY);
                    *ptOutput = ptIdentify->tOutput;
                    return FOC_RESULT_SAFETY;
                }
            } else {
                ptIdentify->wSatCount = 0U;
            }

            /* Settle and stability check */
            if (ptIdentify->wStageTicks >= ptIdentify->tConfig.tTiming.wSettleMinTicks) {
                float fErrD = fabsf(foc_to_float(foc_sub_sat(ptInput->tCurrentDqPu.qD, qIdTarget)));
                float fErrQ = fabsf(foc_to_float(foc_sub_sat(ptInput->tCurrentDqPu.qQ, qIqTarget)));
                float fTol = foc_to_float(ptIdentify->tConfig.tAcceptance.qCurrentTolerance);
                float fSlopeTol = foc_to_float(ptIdentify->tConfig.tAcceptance.qSlopeTolerance);
                bool bSlopeOk = true;

                if (ptIdentify->bPriorCurrentValid) {
                    float fSlope = fabsf(foc_to_float(foc_sub_sat(
                        ptInput->tCurrentDqPu.qD, ptIdentify->qPriorCurrent)));
                    if (fSlope > fSlopeTol) {
                        bSlopeOk = false;
                    }
                }

                if ((fErrD <= fTol) && (fErrQ <= fTol) && bSlopeOk) {
                    ptIdentify->wStableCount++;
                    if (ptIdentify->wStableCount >= ptIdentify->tConfig.tTiming.wStableTicks) {
                        ptIdentify->bPiFrozen = true;
                        ptIdentify->wAverageCount = 0U;
                        if (bIsLow) {
                            ptIdentify->dSumI_low = 0.0;
                            ptIdentify->dSumV_low = 0.0;
                        } else {
                            ptIdentify->dSumI_high = 0.0;
                            ptIdentify->dSumV_high = 0.0;
                        }
                    }
                } else {
                    ptIdentify->wStableCount = 0U;
                }
            }
        } else {
            /* Frozen PI regulation */
            float fErrD = fabsf(foc_to_float(foc_sub_sat(ptInput->tCurrentDqPu.qD, qIdTarget)));
            float fErrQ = fabsf(foc_to_float(foc_sub_sat(ptInput->tCurrentDqPu.qQ, qIqTarget)));
            float fTol = foc_to_float(ptIdentify->tConfig.tAcceptance.qCurrentTolerance);

            /* Check disturbance: if error exceeds 2*tolerance during freeze, unfreeze and resume regulation */
            if ((fErrD > (2.0f * fTol)) || (fErrQ > (2.0f * fTol))) {
                ptIdentify->bPiFrozen = false;
                ptIdentify->wStableCount = 0U;
                ptIdentify->wAverageCount = 0U;
                if (bIsLow) {
                    ptIdentify->dSumI_low = 0.0;
                    ptIdentify->dSumV_low = 0.0;
                } else {
                    ptIdentify->dSumI_high = 0.0;
                    ptIdentify->dSumV_high = 0.0;
                }
            } else if (ptInput->bIntervalValid && ptIdentify->bPriorCurrentValid) {
                double dImid = 0.5 * (foc_to_float(ptIdentify->qPriorCurrent) +
                                      foc_to_float(ptInput->tCurrentDqPu.qD));
                double dVint = foc_to_float(ptInput->tIntervalVoltageDqPu.qD);

                if (bIsLow) {
                    ptIdentify->dSumI_low += dImid;
                    ptIdentify->dSumV_low += dVint;
                } else {
                    ptIdentify->dSumI_high += dImid;
                    ptIdentify->dSumV_high += dVint;
                }
                ptIdentify->wAverageCount++;

                if (ptIdentify->wAverageCount >= ptIdentify->tConfig.tTiming.wAverageTicks) {
                    if (bIsLow) {
                        /* Transition RS_LOW -> RS_HIGH */
                        ptIdentify->eStage = FOC_IDENTIFY_STATUS_RS_HIGH;
                        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_RS_HIGH;
                        ptIdentify->wStageTicks = 0U;
                        ptIdentify->wStableCount = 0U;
                        ptIdentify->wAverageCount = 0U;
                        ptIdentify->wSatCount = 0U;
                        ptIdentify->bPiFrozen = false;
                        ptIdentify->dSumI_high = 0.0;
                        ptIdentify->dSumV_high = 0.0;
                        ptIdentify->tBiasVoltageDqPu.qD = foc_from_float(
                            (float)(ptIdentify->dSumV_low / (double)ptIdentify->tConfig.tTiming.wAverageTicks));
                        ptIdentify->tBiasVoltageDqPu.qQ = ptIdentify->tOutput.tVoltageRefPu.qQ;
                    } else {
                        /* RS_HIGH complete: compute Rs */
                        double dN = (double)ptIdentify->tConfig.tTiming.wAverageTicks;
                        double dIlow = ptIdentify->dSumI_low / dN;
                        double dIhigh = ptIdentify->dSumI_high / dN;
                        double dVlow = ptIdentify->dSumV_low / dN;
                        double dVhigh = ptIdentify->dSumV_high / dN;
                        double dDeltaI = dIhigh - dIlow;
                        double dDeltaV = dVhigh - dVlow;

                        if ((dDeltaI < foc_to_float(ptIdentify->tConfig.tAcceptance.qMinDeltaCurrent)) ||
                            (dDeltaV <= 0.0)) {
                            _foc_identify_EnterError(
                                ptIdentify, FOC_IDENTIFY_FAIL_LOW_RESPONSE, FOC_RESULT_SAFETY);
                            *ptOutput = ptIdentify->tOutput;
                            return FOC_RESULT_SAFETY;
                        }

                        double dRs = dDeltaV / dDeltaI;
                        if ((dRs < foc_to_float(ptIdentify->tConfig.tAcceptance.qResistanceMinPu)) ||
                            (dRs > foc_to_float(ptIdentify->tConfig.tAcceptance.qResistanceMaxPu))) {
                            _foc_identify_EnterError(
                                ptIdentify, FOC_IDENTIFY_FAIL_NUMERIC_RANGE, FOC_RESULT_SAFETY);
                            *ptOutput = ptIdentify->tOutput;
                            return FOC_RESULT_SAFETY;
                        }

                        ptIdentify->tResult.qResistancePu = foc_from_float((float)dRs);
                        ptIdentify->tDiagnostics.qRsEstimatePu = ptIdentify->tResult.qResistancePu;
                        ptIdentify->tDiagnostics.qDeltaI = foc_from_float((float)dDeltaI);
                        ptIdentify->tDiagnostics.qDeltaV = foc_from_float((float)dDeltaV);
                        ptIdentify->tDiagnostics.tBiasVoltageDqPu = ptIdentify->tBiasVoltageDqPu;

                        /* Transition to ZERO */
                        ptIdentify->eStage = FOC_IDENTIFY_STATUS_ZERO;
                        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ZERO;
                        ptIdentify->tOutput.tVoltageRefPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};
                        ptIdentify->tOutput.bRefChanged = true;
                        ptIdentify->wStageTicks = 0U;
                        ptIdentify->wStableCount = 0U;
                        ptIdentify->bPiFrozen = false;
                        ptIdentify->eNextStageAfterZero = FOC_IDENTIFY_STATUS_BIAS;
                    }
                }
            }
        }
        break;
    }

    case FOC_IDENTIFY_STATUS_ZERO: {
        float fIdMag = fabsf(foc_to_float(ptInput->tCurrentDqPu.qD));
        float fIqMag = fabsf(foc_to_float(ptInput->tCurrentDqPu.qQ));
        float fZeroThresh = foc_to_float(ptIdentify->tConfig.tAcceptance.qZeroCurrent);

        ptIdentify->tOutput.tVoltageRefPu = (foc_dq_t){FOC_ZERO, FOC_ZERO};
        ptIdentify->tOutput.bRefChanged = true;

        if ((fIdMag <= fZeroThresh) && (fIqMag <= fZeroThresh)) {
            ptIdentify->wStableCount++;
            if (ptIdentify->wStableCount >= ptIdentify->tConfig.tTiming.wStableTicks) {
                ptIdentify->eStage = ptIdentify->eNextStageAfterZero;
                ptIdentify->tOutput.eStatus = ptIdentify->eNextStageAfterZero;
                ptIdentify->wStageTicks = 0U;
                ptIdentify->wStableCount = 0U;
                ptIdentify->wAverageCount = 0U;
                ptIdentify->wSatCount = 0U;
                ptIdentify->bPiFrozen = false;
                if (ptIdentify->eStage == FOC_IDENTIFY_STATUS_BIAS) {
                    foc_pid_Track(&ptIdentify->tIdPi,
                                  ptIdentify->tBiasVoltageDqPu.qD,
                                  ptIdentify->tConfig.tExcitation.qCurrentLow,
                                  ptInput->tCurrentDqPu.qD);
                    foc_pid_Track(&ptIdentify->tIqPi,
                                  ptIdentify->tBiasVoltageDqPu.qQ,
                                  FOC_ZERO,
                                  ptInput->tCurrentDqPu.qQ);
                }
            }
        } else {
            ptIdentify->wStableCount = 0U;
        }
        break;
    }

    case FOC_IDENTIFY_STATUS_BIAS: {
        foc_scalar_t qIdTarget = ptIdentify->tConfig.tExcitation.qCurrentLow;
        foc_scalar_t qIqTarget = FOC_ZERO;

        if (!ptIdentify->bPiFrozen) {
            foc_scalar_t qVd = foc_pid_Step(
                &ptIdentify->tIdPi, qIdTarget, ptInput->tCurrentDqPu.qD);
            foc_scalar_t qVq = foc_pid_Step(
                &ptIdentify->tIqPi, qIqTarget, ptInput->tCurrentDqPu.qQ);

            ptIdentify->tOutput.tVoltageRefPu.qD = qVd;
            ptIdentify->tOutput.tVoltageRefPu.qQ = qVq;
            ptIdentify->tOutput.bRefChanged = true;

            if (ptIdentify->wStageTicks >= ptIdentify->tConfig.tTiming.wSettleMinTicks) {
                float fErrD = fabsf(foc_to_float(foc_sub_sat(ptInput->tCurrentDqPu.qD, qIdTarget)));
                float fErrQ = fabsf(foc_to_float(foc_sub_sat(ptInput->tCurrentDqPu.qQ, qIqTarget)));
                float fTol = foc_to_float(ptIdentify->tConfig.tAcceptance.qCurrentTolerance);

                if ((fErrD <= fTol) && (fErrQ <= fTol)) {
                    ptIdentify->wStableCount++;
                    if (ptIdentify->wStableCount >= ptIdentify->tConfig.tTiming.wStableTicks) {
                        ptIdentify->bPiFrozen = true;
                        ptIdentify->tBiasVoltageDqPu =
                            ptIdentify->tOutput.tVoltageRefPu;
                        ptIdentify->wStageTicks = 0U;
                        ptIdentify->wStableCount = 0U;
                        ptIdentify->wHalfPeriodCount = 0U;
                        ptIdentify->wPairCount = 0U;
                        ptIdentify->wValidPairs = 0U;
                        ptIdentify->bPositiveHalf = true;
                        ptIdentify->wWindowIntervalsPlus = 0U;
                        ptIdentify->wWindowIntervalsMinus = 0U;
                        ptIdentify->dSumA_plus = 0.0;
                        ptIdentify->dSumB_plus = 0.0;
                        ptIdentify->dSumA_minus = 0.0;
                        ptIdentify->dSumB_minus = 0.0;
                        ptIdentify->llSumA_plus = 0;
                        ptIdentify->llSumB_plus = 0;
                        ptIdentify->llSumA_minus = 0;
                        ptIdentify->llSumB_minus = 0;
                        ptIdentify->dL_sum = 0.0;
                        ptIdentify->qL_min = FOC_ZERO;
                        ptIdentify->qL_max = FOC_ZERO;
                        ptIdentify->bPriorCurrentValid = false;

                        if (!ptIdentify->bLdCompleted) {
                            ptIdentify->eStage = FOC_IDENTIFY_STATUS_LD;
                            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_LD;
                        } else {
                            ptIdentify->eStage = FOC_IDENTIFY_STATUS_LQ;
                            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_LQ;
                        }
                    }
                } else {
                    ptIdentify->wStableCount = 0U;
                }
            }
        }
        break;
    }

    case FOC_IDENTIFY_STATUS_LD:
        return _foc_identify_StepInductanceAxis(ptIdentify, ptInput, false, ptOutput);

    case FOC_IDENTIFY_STATUS_LQ:
        return _foc_identify_StepInductanceAxis(ptIdentify, ptInput, true, ptOutput);

    default:
        break;
    }

    ptIdentify->qPriorCurrent = ptInput->tCurrentDqPu.qD;
    ptIdentify->bPriorCurrentValid = true;

    *ptOutput = ptIdentify->tOutput;
    return FOC_RESULT_OK;
}
