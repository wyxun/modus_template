/****************************************************************************
 * @file    foc_identify.c
 * @brief   Minimal closed-loop motor parameter identification controller.
 * @author  Antigravity
 * @date    2026-09-15
 ****************************************************************************/

#include "foc_identify.h"
#include "foc_angle.h"

#include <stddef.h>

#define IDENTIFY_SETTLE_TICKS   400U
#define IDENTIFY_AVERAGE_TICKS  100U
#define IDENTIFY_PULSE_TICKS    64U
#define IDENTIFY_FIT_BLOCK_TICKS 4U
#define IDENTIFY_TRIAL_COUNT    3U
#define IDENTIFY_TRIAL_SPREAD   FOC_SCALAR(0.20f)
#define IDENTIFY_FAULT_ALGORITHM (1UL << 0)
/* 阶段间 0V 放电沉淀：让上一阶段的残余电流衰减，避免交叉耦合污染
   下一阶段的电感测量。160 拍约 8ms（50us 周期），按当前电机
   L/R≈2ms 的 4 倍时间常数设计；换电机需按 τ=L/R 重估。 */
#define IDENTIFY_ZERO_DWELL_TICKS 160U

static bool _foc_identify_is_valid_scalar(foc_scalar_t qVal)
{
    if (!foc_scalar_is_finite(qVal)) {
        return false;
    }
#if defined(FOC_NUMERIC_FIXED)
    if ((qVal < -FOC_SCALAR(4.0f)) || (qVal > FOC_SCALAR(4.0f))) {
        return false;
    }
#endif
    return true;
}

/**
 * @brief Add two identification fit accumulators.
 * @param qA First accumulator value.
 * @param qB Second accumulator value.
 * @return Sum of the two accumulator values.
 */
static foc_identify_accum_t _foc_identify_accum_add(
    foc_identify_accum_t qA,
    foc_identify_accum_t qB)
{
    return qA + qB;
}

/**
 * @brief Multiply two PU samples into the fit accumulator scale.
 * @param qA First PU sample.
 * @param qB Second PU sample.
 * @return Product in the accumulator scale.
 */
static foc_identify_accum_t _foc_identify_accum_mul(
    foc_scalar_t qA,
    foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA * qB;
#else
    return (foc_identify_accum_t)qA * (foc_identify_accum_t)qB;
#endif
}

/**
 * @brief Multiply an accumulator by a sample count.
 * @param qValue Accumulator value.
 * @param hwCount Number of samples.
 * @return Scaled accumulator value.
 */
static foc_identify_accum_t _foc_identify_accum_mul_count(
    foc_identify_accum_t qValue,
    uint16_t hwCount)
{
    return qValue * (foc_identify_accum_t)hwCount;
}

/**
 * @brief Multiply a PU scalar by an integer sample count.
 * @param qValue Scalar value in PU.
 * @param hwCount Integer multiplier.
 * @return Scaled scalar value in PU.
 */
static foc_scalar_t _foc_identify_scalar_mul_count(
    foc_scalar_t qValue,
    uint16_t hwCount)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qValue * (foc_scalar_t)hwCount;
#else
    return (foc_scalar_t)((int64_t)qValue * (int64_t)hwCount);
#endif
}

/**
 * @brief Calculate the mean of an accumulator sequence.
 * @param qValue Sum of the sequence.
 * @param hwCount Number of samples.
 * @return Mean value in PU scale.
 */
static foc_scalar_t _foc_identify_accum_mean(
    foc_identify_accum_t qValue,
    uint16_t hwCount)
{
    return (foc_scalar_t)(qValue / (foc_identify_accum_t)hwCount);
}

/**
 * @brief Return the middle value of three finite scalar samples.
 * @param qA First sample.
 * @param qB Second sample.
 * @param qC Third sample.
 * @return Middle sample.
 */
static foc_scalar_t _foc_identify_median3(foc_scalar_t qA,
                                     foc_scalar_t qB,
                                     foc_scalar_t qC)
{
    foc_scalar_t qMin = qA;
    foc_scalar_t qMax = qA;

    if (qB < qMin) {
        qMin = qB;
    }
    if (qC < qMin) {
        qMin = qC;
    }
    if (qB > qMax) {
        qMax = qB;
    }
    if (qC > qMax) {
        qMax = qC;
    }
    return qA + qB + qC - qMin - qMax;
}

/**
 * @brief Check whether three identification trials are mutually consistent.
 * @param pqTrials Three trial results.
 * @param pqMedian Optional median output.
 * @return true when the spread is within the configured relative limit.
 */
static bool _foc_identify_trials_consistent(const foc_scalar_t *pqTrials,
                                       foc_scalar_t *pqMedian)
{
    foc_scalar_t qMedian = FOC_ZERO;
    foc_scalar_t qMin = FOC_ZERO;
    foc_scalar_t qMax = FOC_ZERO;
    foc_scalar_t qSpread = FOC_ZERO;

    if (pqTrials == NULL || pqMedian == NULL ||
        !_foc_identify_is_valid_scalar(pqTrials[0]) ||
        !_foc_identify_is_valid_scalar(pqTrials[1]) ||
        !_foc_identify_is_valid_scalar(pqTrials[2])) {
        return false;
    }
    qMedian = _foc_identify_median3(pqTrials[0], pqTrials[1], pqTrials[2]);
    qMin = pqTrials[0];
    qMax = pqTrials[0];
    if (pqTrials[1] < qMin) {
        qMin = pqTrials[1];
    }
    if (pqTrials[2] < qMin) {
        qMin = pqTrials[2];
    }
    if (pqTrials[1] > qMax) {
        qMax = pqTrials[1];
    }
    if (pqTrials[2] > qMax) {
        qMax = pqTrials[2];
    }
    qSpread = qMax - qMin;
    if (qMedian <= FOC_ZERO ||
        qSpread > foc_mul_wide(qMedian, IDENTIFY_TRIAL_SPREAD)) {
        return false;
    }
    *pqMedian = qMedian;
    return true;
}

/**
 * @brief Divide two fit accumulators and return a PU scalar.
 * @param qNumerator Numerator accumulator.
 * @param qDenominator Denominator accumulator.
 * @param pqResult Destination for the ratio.
 * @return FOC_RESULT_OK or a null/divide-by-zero error.
 */
static foc_result_t _foc_identify_accum_ratio(
    foc_identify_accum_t qNumerator,
    foc_identify_accum_t qDenominator,
    foc_scalar_t *pqResult)
{
    if (pqResult == NULL) {
        return FOC_RESULT_NULL;
    }
    if (qDenominator == (foc_identify_accum_t)0) {
        return FOC_RESULT_DIVIDE_BY_ZERO;
    }
#if defined(FOC_NUMERIC_FLOAT)
    *pqResult = qNumerator / qDenominator;
#else
    *pqResult = (foc_scalar_t)((qNumerator *
                                (foc_identify_accum_t)FOC_Q_SCALE) /
                               qDenominator);
#endif
    return FOC_RESULT_OK;
}

/**
 * @brief Estimate inductance from the discrete RL current response.
 * @param ptIdentify Pointer to the identification controller.
 * @param qVoltage Applied pulse voltage in PU.
 * @param pqInductance Destination for the estimated inductance in PU.
 * @return FOC_RESULT_OK or a validation/calculation error.
 */
static foc_result_t _foc_identify_fit_inductance(
    const foc_identify_t *ptIdentify,
    foc_scalar_t qVoltage,
    foc_scalar_t *pqInductance)
{
    foc_scalar_t qSlope = FOC_ZERO;
    foc_scalar_t qMeanX = FOC_ZERO;
    foc_scalar_t qMeanY = FOC_ZERO;
    foc_scalar_t qL = FOC_ZERO;
    foc_identify_accum_t qDenominator = (foc_identify_accum_t)0;
    foc_identify_accum_t qNumerator = (foc_identify_accum_t)0;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptIdentify->hwFitSamples < 8U) {
        return FOC_RESULT_SAFETY;
    }
    if (!_foc_identify_is_valid_scalar(qVoltage) ||
        (qVoltage <= FOC_ZERO)) {
        return FOC_RESULT_SAFETY;
    }
    qMeanX = _foc_identify_accum_mean(ptIdentify->qFitSumX,
                                 ptIdentify->hwFitSamples);
    qMeanY = _foc_identify_accum_mean(ptIdentify->qFitSumY,
                                 ptIdentify->hwFitSamples);
    qDenominator = _foc_identify_accum_add(
        ptIdentify->qFitSumXX,
        -_foc_identify_accum_mul_count(
            _foc_identify_accum_mul(qMeanX, qMeanX),
            ptIdentify->hwFitSamples));
    qNumerator = _foc_identify_accum_add(
        ptIdentify->qFitSumXY,
        -_foc_identify_accum_mul_count(
            _foc_identify_accum_mul(qMeanX, qMeanY),
            ptIdentify->hwFitSamples));
    if (qDenominator <= (foc_identify_accum_t)0) {
        return FOC_RESULT_SAFETY;
    }
    eResult = _foc_identify_accum_ratio(qNumerator, qDenominator, &qSlope);
    if (eResult != FOC_RESULT_OK || qSlope >= FOC_ZERO) {
        return FOC_RESULT_SAFETY;
    }
    eResult = foc_div_checked(
        foc_mul_wide(
                _foc_identify_scalar_mul_count(
                    ptIdentify->qRadiansPerSample,
                    IDENTIFY_FIT_BLOCK_TICKS),
            ptIdentify->tResult.qResistancePu),
            foc_abs(qSlope), &qL);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (!_foc_identify_is_valid_scalar(qL) ||
        (qL <= FOC_ZERO) || (qL > FOC_SCALAR(5.0f)) ||
        !_foc_identify_is_valid_scalar(qVoltage)) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    *pqInductance = qL;
    return FOC_RESULT_OK;
}

static foc_result_t _foc_identify_fail(foc_identify_t *ptIdentify,
                                  foc_result_t eFailure)
{
    ptIdentify->tDiagnostics.eFailureStage =
        ptIdentify->tOutput.eStatus;
    ptIdentify->tDiagnostics.eFailure = eFailure;
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ERROR;
    ptIdentify->eState = FOC_IDENTIFY_STATE_ERROR;
    ptIdentify->eLastError = eFailure;
    ptIdentify->wFaults |= IDENTIFY_FAULT_ALGORITHM;
    ptIdentify->eFailure = eFailure;
    ptIdentify->tResult = (foc_identify_result_t){
        FOC_ZERO, FOC_ZERO, FOC_ZERO
    };
    ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
    ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
    ptIdentify->tOutput.bRefChanged = true;
    ptIdentify->tOutput.bStopPwm = true;
    return eFailure;
}

foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_cfg_t *ptConfig)
{
    if ((ptIdentify == NULL) || (ptConfig == NULL)) {
        return FOC_RESULT_NULL;
    }
    *ptIdentify = (foc_identify_t){0};
    ptIdentify->eState = FOC_IDENTIFY_STATE_UNINITIALIZED;
    ptIdentify->eLastError = FOC_RESULT_OK;
    if (!_foc_identify_is_valid_scalar(ptConfig->qV_low) ||
        !_foc_identify_is_valid_scalar(ptConfig->qV_high) ||
        !_foc_identify_is_valid_scalar(ptConfig->qV_Ld) ||
        !_foc_identify_is_valid_scalar(ptConfig->qV_Lq) ||
        !_foc_identify_is_valid_scalar(ptConfig->qCurrentLimit) ||
        !_foc_identify_is_valid_scalar(ptConfig->qMinDeltaI) ||
        !_foc_identify_is_valid_scalar(ptConfig->qRadiansPerSample) ||
        !_foc_identify_is_valid_scalar(ptConfig->qMaxDisplacement) ||
        (ptConfig->qV_low <= FOC_ZERO) ||
        (ptConfig->qV_high <= ptConfig->qV_low) ||
        (ptConfig->qV_high > FOC_SCALAR(0.577f)) ||
        (ptConfig->qV_Ld <= FOC_ZERO) ||
        (ptConfig->qV_Ld > FOC_SCALAR(0.577f)) ||
        (ptConfig->qV_Lq <= FOC_ZERO) ||
        (ptConfig->qV_Lq > FOC_SCALAR(0.577f)) ||
        (ptConfig->qCurrentLimit <= FOC_ZERO) ||
        (ptConfig->qMinDeltaI <= FOC_ZERO) ||
        (ptConfig->qRadiansPerSample <= FOC_ZERO) ||
        (ptConfig->qMaxDisplacement <= FOC_ZERO)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    ptIdentify->qVoltageLow = ptConfig->qV_low;
    ptIdentify->qVoltageHigh = ptConfig->qV_high;
    ptIdentify->qVoltageLd = ptConfig->qV_Ld;
    ptIdentify->qVoltageLq = ptConfig->qV_Lq;
    ptIdentify->qCurrentLimit = ptConfig->qCurrentLimit;
    ptIdentify->qMinDeltaI = ptConfig->qMinDeltaI;
    ptIdentify->qRadiansPerSample = ptConfig->qRadiansPerSample;
    ptIdentify->qMaxDisplacement = ptConfig->qMaxDisplacement;
    ptIdentify->eState = FOC_IDENTIFY_STATE_IDLE;
    ptIdentify->eLastError = FOC_RESULT_OK;
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_IDLE;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_Start(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_IDLE) {
        return FOC_RESULT_BUSY;
    }

    ptIdentify->tResult = (foc_identify_result_t){
        FOC_ZERO, FOC_ZERO, FOC_ZERO
    };
    ptIdentify->eFailure = FOC_RESULT_OK;
    ptIdentify->eLastError = FOC_RESULT_OK;
    ptIdentify->wFaults = 0U;
    ptIdentify->tDiagnostics = (foc_identify_diagnostics_t){0};
    ptIdentify->tDiagnostics.eFailureStage = FOC_IDENTIFY_STATUS_IDLE;
    ptIdentify->tDiagnostics.eFailure = FOC_RESULT_OK;
    ptIdentify->bZeroPrimed = false;
    ptIdentify->qSumV = FOC_ZERO;
    ptIdentify->qSumI = FOC_ZERO;
    ptIdentify->qI_start = FOC_ZERO;
    ptIdentify->qI_last = FOC_ZERO;
    ptIdentify->qI_low = FOC_ZERO;
    ptIdentify->qV_low = FOC_ZERO;
    ptIdentify->qFitSumX = (foc_identify_accum_t)0;
    ptIdentify->qFitSumY = (foc_identify_accum_t)0;
    ptIdentify->qFitSumXX = (foc_identify_accum_t)0;
    ptIdentify->qFitSumXY = (foc_identify_accum_t)0;
    ptIdentify->qFitBlockSumI = FOC_ZERO;
    ptIdentify->qFitBlockSumV = FOC_ZERO;
    ptIdentify->qFitLastI = FOC_ZERO;
    ptIdentify->bFitBlockPrimed = false;
    ptIdentify->chFitBlockTicks = 0U;
    ptIdentify->hwFitSamples = 0U;
    ptIdentify->hwTicks = 0U;
    ptIdentify->chRsTrial = 0U;
    ptIdentify->chLdTrial = 0U;
    ptIdentify->chLqTrial = 0U;
    ptIdentify->bReturnPulse = false;

    ptIdentify->eState = FOC_IDENTIFY_STATE_RUNNING;
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_RS_LOW;
    ptIdentify->tOutput.tVoltageRefPu.qD = ptIdentify->qVoltageLow;
    ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
    ptIdentify->tOutput.bRefChanged = true;
    ptIdentify->tOutput.bStopPwm = false;
    return FOC_RESULT_OK;
}

static foc_result_t _foc_identify_step_rs(foc_identify_t *ptIdentify,
                                     const foc_identify_input_t *ptInput)
{
    foc_scalar_t qV = ptInput->tLastVoltageCommandDqPu.qD;
    foc_scalar_t qI = ptInput->tCurrentDqPu.qD;
    uint32_t wTotal = IDENTIFY_SETTLE_TICKS + IDENTIFY_AVERAGE_TICKS;

    ptIdentify->hwTicks++;
    if (ptIdentify->hwTicks > IDENTIFY_SETTLE_TICKS) {
        ptIdentify->qSumV = foc_add_sat(ptIdentify->qSumV, qV);
        ptIdentify->qSumI = foc_add_sat(ptIdentify->qSumI, qI);
    }

    if (ptIdentify->hwTicks >= wTotal) {
        foc_scalar_t qAvgV = ptIdentify->qSumV /
                             (foc_scalar_t)IDENTIFY_AVERAGE_TICKS;
        foc_scalar_t qAvgI = ptIdentify->qSumI /
                             (foc_scalar_t)IDENTIFY_AVERAGE_TICKS;

        if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_LOW) {
            ptIdentify->qV_low = qAvgV;
            ptIdentify->qI_low = qAvgI;
            ptIdentify->qSumV = FOC_ZERO;
            ptIdentify->qSumI = FOC_ZERO;
            ptIdentify->hwTicks = 0U;

            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_RS_HIGH;
            ptIdentify->tOutput.tVoltageRefPu.qD = ptIdentify->qVoltageHigh;
            ptIdentify->tOutput.bRefChanged = true;
        } else {
            foc_scalar_t qDeltaV = foc_sub_sat(qAvgV, ptIdentify->qV_low);
            foc_scalar_t qDeltaI = foc_sub_sat(qAvgI, ptIdentify->qI_low);
            foc_scalar_t qR = FOC_ZERO;
            foc_identify_diag_t *ptDiag =
                &ptIdentify->tDiagnostics.tRs;

            ptDiag->qIStart = ptIdentify->qI_low;
            ptDiag->qILast = qAvgI;
            ptDiag->qDeltaI = qDeltaI;
            ptDiag->qDeltaV = qDeltaV;
            ptDiag->qSumV = FOC_ZERO;
            ptDiag->hwTicks = ptIdentify->hwTicks;
            ptDiag->bValid = false;

            if (!_foc_identify_is_valid_scalar(qDeltaI) ||
                !_foc_identify_is_valid_scalar(qDeltaV) ||
                (qDeltaI <= ptIdentify->qMinDeltaI) ||
                (qDeltaV <= FOC_ZERO)) {
                return _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            }
            if (foc_div_checked(qDeltaV, qDeltaI, &qR) != FOC_RESULT_OK) {
                return _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            }
            if (!_foc_identify_is_valid_scalar(qR) ||
                (qR <= FOC_ZERO) || (qR > FOC_SCALAR(5.0f))) {
                return _foc_identify_fail(ptIdentify,
                                          FOC_RESULT_OUT_OF_RANGE);
            }
            if (ptIdentify->chRsTrial >= IDENTIFY_TRIAL_COUNT) {
                return _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            }
            ptIdentify->aqRsTrial[ptIdentify->chRsTrial] = qR;
            ptIdentify->chRsTrial++;
            ptIdentify->qSumV = FOC_ZERO;
            ptIdentify->qSumI = FOC_ZERO;
            ptIdentify->hwTicks = 0U;

            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ZERO;
            if (ptIdentify->chRsTrial < IDENTIFY_TRIAL_COUNT) {
                ptIdentify->eNextStage = FOC_IDENTIFY_STATUS_RS_LOW;
            } else {
                if (!_foc_identify_trials_consistent(ptIdentify->aqRsTrial,
                                                &qR)) {
                return _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
                }
                ptIdentify->tResult.qResistancePu = qR;
                ptDiag->qResistancePu = qR;
                ptDiag->qResult = qR;
                ptDiag->bValid = true;
                ptIdentify->eNextStage = FOC_IDENTIFY_STATUS_LD;
            }
            ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
            ptIdentify->tOutput.bRefChanged = true;
        }
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Hold zero voltage for the inter-stage discharge dwell.
 * @param ptIdentify Identify controller instance.
 * @return FOC_RESULT_OK.
 */
static foc_result_t _foc_identify_step_zero(foc_identify_t *ptIdentify)
{
    ptIdentify->hwTicks++;
    if (ptIdentify->hwTicks >= IDENTIFY_ZERO_DWELL_TICKS) {
        ptIdentify->hwTicks = 0U;
        if (ptIdentify->eNextStage == FOC_IDENTIFY_STATUS_RS_LOW) {
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_RS_LOW;
            ptIdentify->tOutput.tVoltageRefPu.qD = ptIdentify->qVoltageLow;
            ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
        } else if (ptIdentify->eNextStage == FOC_IDENTIFY_STATUS_LD) {
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_LD;
            ptIdentify->tOutput.tVoltageRefPu.qD = ptIdentify->qVoltageLd;
            ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
        } else {
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_LQ;
            ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
            ptIdentify->tOutput.tVoltageRefPu.qQ = ptIdentify->qVoltageLq;
        }
        ptIdentify->tOutput.bRefChanged = true;
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Finish the reverse Q-axis pulse and stop the identification PWM.
 * @param ptIdentify Identify controller instance.
 * @return FOC_RESULT_OK.
 */
static foc_result_t _foc_identify_step_return_pulse(
    foc_identify_t *ptIdentify)
{
    ptIdentify->hwTicks++;
    if (ptIdentify->hwTicks >= IDENTIFY_PULSE_TICKS) {
        ptIdentify->bReturnPulse = false;
        ptIdentify->hwTicks = 0U;
        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_COMPLETE;
        ptIdentify->eState = FOC_IDENTIFY_STATE_COMPLETE;
        ptIdentify->tOutput.tVoltageRefPu =
            (foc_dq_t){FOC_ZERO, FOC_ZERO};
        ptIdentify->tOutput.bRefChanged = true;
        ptIdentify->tOutput.bStopPwm = true;
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Reset the block-fit accumulators for an inductance pulse.
 * @param ptIdentify Identify controller instance.
 * @param ptDiag Diagnostic snapshot for the active axis.
 * @param qCurrent First current sample of the pulse.
 * @param qResistance Resistance used for the fit result.
 * @return None.
 */
static void _foc_identify_reset_inductance_fit(
    foc_identify_t *ptIdentify,
    foc_identify_diag_t *ptDiag,
    foc_scalar_t qCurrent,
    foc_scalar_t qResistance)
{
    ptIdentify->qI_start = qCurrent;
    ptIdentify->qI_last = qCurrent;
    ptDiag->qIStart = qCurrent;
    ptDiag->qILast = qCurrent;
    ptDiag->qDeltaI = FOC_ZERO;
    ptDiag->qSumV = FOC_ZERO;
    ptDiag->qResistancePu = qResistance;
    ptDiag->hwTicks = 1U;
    ptDiag->bValid = false;
    ptIdentify->qSumV = FOC_ZERO;
    ptIdentify->qFitSumX = (foc_identify_accum_t)0;
    ptIdentify->qFitSumY = (foc_identify_accum_t)0;
    ptIdentify->qFitSumXX = (foc_identify_accum_t)0;
    ptIdentify->qFitSumXY = (foc_identify_accum_t)0;
    ptIdentify->qFitBlockSumI = FOC_ZERO;
    ptIdentify->qFitBlockSumV = FOC_ZERO;
    ptIdentify->qFitLastI = FOC_ZERO;
    ptIdentify->bFitBlockPrimed = false;
    ptIdentify->chFitBlockTicks = 0U;
    ptIdentify->hwFitSamples = 0U;
    ptIdentify->hwTicks = 1U;
}

/**
 * @brief Accumulate one inductance sample and complete full fit blocks.
 * @param ptIdentify Identify controller instance.
 * @param qCurrent Current sample for the active axis.
 * @param qVoltage Voltage command paired with the sample.
 * @return None.
 */
static void _foc_identify_accumulate_inductance_sample(
    foc_identify_t *ptIdentify,
    foc_scalar_t qCurrent,
    foc_scalar_t qVoltage)
{
    ptIdentify->qFitBlockSumI = foc_add_sat(
        ptIdentify->qFitBlockSumI, qCurrent);
    ptIdentify->qFitBlockSumV = foc_add_sat(
        ptIdentify->qFitBlockSumV, qVoltage);
    ptIdentify->chFitBlockTicks++;
    if (ptIdentify->chFitBlockTicks >= IDENTIFY_FIT_BLOCK_TICKS) {
        foc_scalar_t qBlockI = ptIdentify->qFitBlockSumI /
                               (foc_scalar_t)IDENTIFY_FIT_BLOCK_TICKS;
        foc_scalar_t qBlockV = ptIdentify->qFitBlockSumV /
                               (foc_scalar_t)IDENTIFY_FIT_BLOCK_TICKS;

        ptIdentify->qSumV = foc_add_sat(
            ptIdentify->qSumV,
            _foc_identify_scalar_mul_count(
                qBlockV, IDENTIFY_FIT_BLOCK_TICKS));
        if (ptIdentify->bFitBlockPrimed) {
            foc_scalar_t qIAvg = foc_mul_wide(
                foc_add_sat(ptIdentify->qFitLastI, qBlockI), FOC_HALF);
            foc_scalar_t qBlockDeltaI = foc_sub_sat(
                qBlockI, ptIdentify->qFitLastI);

            ptIdentify->qFitSumX = _foc_identify_accum_add(
                ptIdentify->qFitSumX, (foc_identify_accum_t)qIAvg);
            ptIdentify->qFitSumY = _foc_identify_accum_add(
                ptIdentify->qFitSumY,
                (foc_identify_accum_t)qBlockDeltaI);
            ptIdentify->qFitSumXX = _foc_identify_accum_add(
                ptIdentify->qFitSumXX,
                _foc_identify_accum_mul(qIAvg, qIAvg));
            ptIdentify->qFitSumXY = _foc_identify_accum_add(
                ptIdentify->qFitSumXY,
                _foc_identify_accum_mul(qIAvg, qBlockDeltaI));
            ptIdentify->hwFitSamples++;
        } else {
            ptIdentify->bFitBlockPrimed = true;
        }
        ptIdentify->qFitLastI = qBlockI;
        ptIdentify->chFitBlockTicks = 0U;
        ptIdentify->qFitBlockSumI = FOC_ZERO;
        ptIdentify->qFitBlockSumV = FOC_ZERO;
    }
    ptIdentify->qI_last = qCurrent;
    ptIdentify->hwTicks++;
}

/**
 * @brief Validate and store one completed inductance trial.
 * @param ptIdentify Identify controller instance.
 * @param bQAxis True for Lq, false for Ld.
 * @param ptDiag Diagnostic snapshot for the active axis.
 * @return FOC_RESULT_OK or a validation/calculation error.
 */
static foc_result_t _foc_identify_finish_inductance_trial(
    foc_identify_t *ptIdentify,
    bool bQAxis,
    foc_identify_diag_t *ptDiag)
{
    foc_scalar_t qDeltaI = foc_sub_sat(
        ptIdentify->qFitLastI, ptIdentify->qI_start);
    foc_scalar_t qMeanV = FOC_ZERO;
    foc_scalar_t qL = FOC_ZERO;
    foc_result_t eFit = FOC_RESULT_OK;
    foc_scalar_t *pqTrials = bQAxis ?
        ptIdentify->aqLqTrial : ptIdentify->aqLdTrial;
    uint8_t *pchTrial = bQAxis ?
        &ptIdentify->chLqTrial : &ptIdentify->chLdTrial;

    ptDiag->qILast = ptIdentify->qI_last;
    ptDiag->qDeltaI = qDeltaI;
    ptDiag->qSumV = ptIdentify->qSumV;
    ptDiag->hwTicks = ptIdentify->hwTicks;
    ptDiag->bValid = false;
    if (!_foc_identify_is_valid_scalar(qDeltaI) ||
        !foc_scalar_is_finite(ptIdentify->qSumV) ||
        (qDeltaI <= ptIdentify->qMinDeltaI) ||
        !ptIdentify->bFitBlockPrimed ||
        (ptIdentify->chFitBlockTicks != 0U) ||
        (ptIdentify->qSumV <= FOC_ZERO)) {
        return _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
    }

    qMeanV = ptIdentify->qSumV /
             (foc_scalar_t)ptIdentify->hwTicks;
    eFit = _foc_identify_fit_inductance(ptIdentify, qMeanV, &qL);
    if (eFit != FOC_RESULT_OK) {
        return _foc_identify_fail(ptIdentify, eFit);
    }
    if (*pchTrial >= IDENTIFY_TRIAL_COUNT) {
        return _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
    }
    pqTrials[*pchTrial] = qL;
    *pchTrial = (uint8_t)(*pchTrial + 1U);
    ptIdentify->qSumV = FOC_ZERO;
    ptIdentify->hwTicks = 0U;

    if (*pchTrial < IDENTIFY_TRIAL_COUNT) {
        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ZERO;
        ptIdentify->eNextStage = bQAxis ?
            FOC_IDENTIFY_STATUS_LQ : FOC_IDENTIFY_STATUS_LD;
        ptIdentify->tOutput.tVoltageRefPu =
            (foc_dq_t){FOC_ZERO, FOC_ZERO};
        ptIdentify->tOutput.bRefChanged = true;
    } else if (!_foc_identify_trials_consistent(pqTrials, &qL)) {
        return _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
    } else if (!bQAxis) {
        ptIdentify->tResult.qInductanceDPu = qL;
        ptDiag->qResult = qL;
        ptDiag->bValid = true;
        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ZERO;
        ptIdentify->eNextStage = FOC_IDENTIFY_STATUS_LQ;
        ptIdentify->tOutput.tVoltageRefPu =
            (foc_dq_t){FOC_ZERO, FOC_ZERO};
        ptIdentify->tOutput.bRefChanged = true;
    } else {
        ptIdentify->tResult.qInductanceQPu = qL;
        ptDiag->qResult = qL;
        ptDiag->bValid = true;
        ptIdentify->bReturnPulse = true;
        ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
        ptIdentify->tOutput.tVoltageRefPu.qQ =
            -ptIdentify->qVoltageLq;
        ptIdentify->tOutput.bRefChanged = true;
    }
    return FOC_RESULT_OK;
}

static foc_result_t _foc_identify_step_inductance(
    foc_identify_t *ptIdentify,
    const foc_identify_input_t *ptInput)
{
    bool bQAxis = (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_LQ);

    if (ptIdentify->bReturnPulse) {
        return _foc_identify_step_return_pulse(ptIdentify);
    }
    if (ptIdentify->hwTicks == 0U) {
        foc_scalar_t qCurrent = bQAxis ? ptInput->tCurrentDqPu.qQ :
                                         ptInput->tCurrentDqPu.qD;
        foc_identify_diag_t *ptDiag = bQAxis ?
            &ptIdentify->tDiagnostics.tLq : &ptIdentify->tDiagnostics.tLd;

        _foc_identify_reset_inductance_fit(
            ptIdentify, ptDiag, qCurrent,
            ptIdentify->tResult.qResistancePu);
        return FOC_RESULT_OK;
    }

    {
        foc_scalar_t qVoltage = bQAxis ?
            ptInput->tLastVoltageCommandDqPu.qQ :
            ptInput->tLastVoltageCommandDqPu.qD;
        foc_scalar_t qCurrent = bQAxis ? ptInput->tCurrentDqPu.qQ :
                                         ptInput->tCurrentDqPu.qD;

        _foc_identify_accumulate_inductance_sample(
            ptIdentify, qCurrent, qVoltage);
    }

    if (ptIdentify->hwTicks > IDENTIFY_PULSE_TICKS) {
        foc_identify_diag_t *ptDiag = bQAxis ?
            &ptIdentify->tDiagnostics.tLq : &ptIdentify->tDiagnostics.tLd;

        return _foc_identify_finish_inductance_trial(
            ptIdentify, bQAxis, ptDiag);
    }
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_IsrStep(foc_identify_t *ptIdentify,
                                  const foc_identify_input_t *ptInput,
                                  foc_identify_output_t *ptOutput)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if ((ptIdentify == NULL) || (ptOutput == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    ptIdentify->tOutput.bRefChanged = false;

    if ((ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_IDLE) ||
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE) ||
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR)) {
        *ptOutput = ptIdentify->tOutput;
        return (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR)
                   ? ptIdentify->eFailure : FOC_RESULT_OK;
    }

    if ((ptInput == NULL) || (!ptInput->bValid) || ptInput->bFault ||
        !_foc_identify_is_valid_scalar(ptInput->tCurrentDqPu.qD) ||
        !_foc_identify_is_valid_scalar(ptInput->tCurrentDqPu.qQ) ||
        !_foc_identify_is_valid_scalar(
            ptInput->tLastVoltageCommandDqPu.qD) ||
        !_foc_identify_is_valid_scalar(
            ptInput->tLastVoltageCommandDqPu.qQ)) {
        eResult = _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
        *ptOutput = ptIdentify->tOutput;
        return eResult;
    }

    /* Vector magnitude over-current safety check */
    {
        foc_scalar_t qId = ptInput->tCurrentDqPu.qD;
        foc_scalar_t qIq = ptInput->tCurrentDqPu.qQ;
        foc_scalar_t qLimit = ptIdentify->qCurrentLimit;
        foc_scalar_t qIdSq = foc_mul_wide(qId, qId);
        foc_scalar_t qIqSq = foc_mul_wide(qIq, qIq);
        foc_scalar_t qMagSq = foc_add_sat(qIdSq, qIqSq);
        foc_scalar_t qLimSq = foc_mul_wide(qLimit, qLimit);

        if (qMagSq > qLimSq) {
            eResult = _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            *ptOutput = ptIdentify->tOutput;
            return eResult;
        }
    }

    /* Mechanical displacement safety check */
    if (!ptIdentify->bZeroPrimed) {
        ptIdentify->tZeroAngle = ptInput->tMechanicalAngle;
        ptIdentify->bZeroPrimed = true;
    } else {
        foc_scalar_t qDisp = foc_abs(
            foc_angle_diff(ptInput->tMechanicalAngle, ptIdentify->tZeroAngle));

        if (qDisp > ptIdentify->qMaxDisplacement) {
            eResult = _foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            *ptOutput = ptIdentify->tOutput;
            return eResult;
        }
    }

    /* Execute active stage */
    if ((ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_LOW) ||
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_HIGH)) {
        eResult = _foc_identify_step_rs(ptIdentify, ptInput);
    } else if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ZERO) {
        eResult = _foc_identify_step_zero(ptIdentify);
    } else if ((ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_LD) ||
               (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_LQ)) {
        eResult = _foc_identify_step_inductance(ptIdentify, ptInput);
    } else {
        eResult = _foc_identify_fail(ptIdentify,
                                     FOC_RESULT_INVALID_ARGUMENT);
    }

    *ptOutput = ptIdentify->tOutput;
    return eResult;
}

void foc_identify_Abort(foc_identify_t *ptIdentify)
{
    if ((ptIdentify != NULL) &&
        (ptIdentify->eState != FOC_IDENTIFY_STATE_UNINITIALIZED)) {
        foc_identify_status_e eStatus = ptIdentify->tOutput.eStatus;
        if ((eStatus != FOC_IDENTIFY_STATUS_IDLE) &&
            (eStatus != FOC_IDENTIFY_STATUS_COMPLETE) &&
            (eStatus != FOC_IDENTIFY_STATUS_ERROR)) {
            (void)_foc_identify_fail(ptIdentify, FOC_RESULT_SAFETY);
        }
    }
}

/**
 * @brief Reset one Identify Driver to its uninitialized state.
 * @param ptIdentify Identify object.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_identify_Reset(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptIdentify = (foc_identify_t){0};
    ptIdentify->eState = FOC_IDENTIFY_STATE_UNINITIALIZED;
    ptIdentify->eLastError = FOC_RESULT_OK;
    return FOC_RESULT_OK;
}

/**
 * @brief Copy the Identify Driver status.
 * @param ptIdentify Identify object.
 * @param ptStatus Output status snapshot.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_identify_GetStatus(const foc_identify_t *ptIdentify,
                                    foc_identify_status_t *ptStatus)
{
    if ((ptIdentify == NULL) || (ptStatus == NULL)) {
        return FOC_RESULT_NULL;
    }
    ptStatus->eState = ptIdentify->eState;
    ptStatus->eStage = ptIdentify->tOutput.eStatus;
    ptStatus->eLastError = ptIdentify->eLastError;
    ptStatus->wFaults = ptIdentify->wFaults;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_GetResult(const foc_identify_t *ptIdentify,
                                    foc_identify_result_t *ptResult)
{
    if ((ptIdentify == NULL) || (ptResult == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE) {
        *ptResult = ptIdentify->tResult;
        return FOC_RESULT_OK;
    }
    if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR) {
        return ptIdentify->eFailure;
    }
    return FOC_RESULT_BUSY;
}

foc_result_t foc_identify_GetDiagnostics(
    const foc_identify_t *ptIdentify,
    foc_identify_diagnostics_t *ptDiagnostics)
{
    if ((ptIdentify == NULL) || (ptDiagnostics == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptDiagnostics = ptIdentify->tDiagnostics;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_ConsumeTerminal(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if ((ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_COMPLETE) &&
        (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_ERROR)) {
        return FOC_RESULT_BUSY;
    }
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_IDLE;
    ptIdentify->eState = FOC_IDENTIFY_STATE_IDLE;
    ptIdentify->eLastError = FOC_RESULT_OK;
    ptIdentify->eFailure = FOC_RESULT_OK;
    ptIdentify->wFaults = 0U;
    ptIdentify->tOutput.bStopPwm = true;
    ptIdentify->tOutput.bRefChanged = false;
    ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
    ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
    return FOC_RESULT_OK;
}
