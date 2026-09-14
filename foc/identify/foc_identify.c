/****************************************************************************
 * @file    foc_identify.c
 * @brief   Per-instance PU resistance and RL-step identification.
 * @author  Codex
 * @date    2026-09-14
 ****************************************************************************/

#include <limits.h>
#include <math.h>
#include <string.h>

#include "foc_identify.h"

/* 2*pi/ln(20), for the 95% RL step crossing. */
#define IDENTIFY_RL_SCALE_FLOAT 2.097378782f
#define IDENTIFY_RL_SCALE_Q15  68727

/**
 * @brief Check whether a scalar is finite and inside the PU range.
 * @param qValue Candidate scalar.
 * @return True when qValue is in [-1, 1].
 */
static bool identify_is_pu(foc_scalar_t qValue)
{
#if defined(FOC_NUMERIC_FLOAT)
    if (isfinite(qValue) == 0) {
        return false;
    }
#endif
    return (bool)((qValue >= FOC_NEG_ONE) && (qValue <= FOC_ONE));
}

/**
 * @brief Set the D/Q command and record whether it changed.
 * @param ptIdentify Instance owning the command.
 * @param qD D-axis voltage command.
 * @param qQ Q-axis voltage command.
 * @return None.
 */
static void identify_set_reference(foc_identify_t *ptIdentify,
                                   foc_scalar_t qD,
                                   foc_scalar_t qQ)
{
    ptIdentify->tOutput.bReferenceChanged = (bool)(
        (ptIdentify->tOutput.tVoltageReference.qD != qD) ||
        (ptIdentify->tOutput.tVoltageReference.qQ != qQ));
    ptIdentify->tOutput.tVoltageReference.qD = qD;
    ptIdentify->tOutput.tVoltageReference.qQ = qQ;
}

/**
 * @brief Begin a phase and reset its local counters and window.
 * @param ptIdentify Instance to update.
 * @param eStatus New phase.
 * @param qD D-axis voltage command.
 * @param qQ Q-axis voltage command.
 * @return None.
 */
static void identify_begin_phase(foc_identify_t *ptIdentify,
                                 foc_identify_status_e eStatus,
                                 foc_scalar_t qD,
                                 foc_scalar_t qQ)
{
    bool bChanged = (bool)(
        (ptIdentify->tOutput.tVoltageReference.qD != qD) ||
        (ptIdentify->tOutput.tVoltageReference.qQ != qQ));

    ptIdentify->tOutput.eStatus = eStatus;
    ptIdentify->wPhaseSamples = 0U;
    ptIdentify->qWindowVoltageMean = FOC_ZERO;
    ptIdentify->hwWindowSamples = 0U;
    ptIdentify->bRisePrimed = false;
    ptIdentify->tOutput.bReferenceChanged = bChanged;
    ptIdentify->tOutput.tVoltageReference.qD = qD;
    ptIdentify->tOutput.tVoltageReference.qQ = qQ;
}

/**
 * @brief Enter ERROR, discard partial results, and command zero voltage.
 * @param ptIdentify Instance to fail.
 * @param eFailure Failure returned to the caller.
 * @return The supplied failure code.
 */
static foc_result_t identify_fail(foc_identify_t *ptIdentify,
                                  foc_result_t eFailure)
{
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ERROR;
    ptIdentify->eFailure = eFailure;
    ptIdentify->tResult = (foc_identify_result_t){0};
    identify_set_reference(ptIdentify, FOC_ZERO, FOC_ZERO);
    return eFailure;
}

/**
 * @brief Update a running mean without a narrow fixed-point sum.
 * @param qMean Existing mean.
 * @param qValue New sample.
 * @param hwCount Sample count after insertion; must be nonzero.
 * @return Updated mean.
 */
static foc_scalar_t identify_update_mean(foc_scalar_t qMean,
                                         foc_scalar_t qValue,
                                         uint16_t hwCount)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qMean + ((qValue - qMean) / (float)hwCount);
#else
    int64_t llDelta = (int64_t)qValue - (int64_t)qMean;
    return (foc_scalar_t)((int64_t)qMean +
                          (llDelta / (int64_t)hwCount));
#endif
}

/**
 * @brief Validate excitation, limits, sampling window, and timeout settings.
 * @param ptConfig Candidate configuration.
 * @return FOC_RESULT_OK or the configuration error.
 */
static foc_result_t identify_validate_config(
    const foc_identify_cfg_t *ptConfig)
{
    uint32_t wRequiredSamples = 0U;

    if ((identify_is_pu(ptConfig->qResistanceLowVoltagePu) == false) ||
        (identify_is_pu(ptConfig->qResistanceHighVoltagePu) == false) ||
        (identify_is_pu(ptConfig->qInductanceDVoltagePu) == false) ||
        (identify_is_pu(ptConfig->qInductanceQVoltagePu) == false) ||
        (identify_is_pu(ptConfig->qCurrentLimitPu) == false) ||
        (identify_is_pu(ptConfig->qResetCurrentLimitPu) == false) ||
        (identify_is_pu(ptConfig->qMinimumResistanceDeltaPu) == false) ||
        (identify_is_pu(ptConfig->qCurrentStabilityTolerancePu) == false)) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#if defined(FOC_NUMERIC_FLOAT)
    if (isfinite(ptConfig->qElectricalBaseTurnsPerSample) == 0) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#endif

    if ((ptConfig->qResistanceLowVoltagePu ==
         ptConfig->qResistanceHighVoltagePu) ||
        (ptConfig->qInductanceDVoltagePu == FOC_ZERO) ||
        (ptConfig->qInductanceQVoltagePu == FOC_ZERO) ||
        (ptConfig->qCurrentLimitPu <= FOC_ZERO) ||
        (ptConfig->qResetCurrentLimitPu <= FOC_ZERO) ||
        (ptConfig->qResetCurrentLimitPu > ptConfig->qCurrentLimitPu) ||
        (ptConfig->qMinimumResistanceDeltaPu <= FOC_ZERO) ||
        (ptConfig->qElectricalBaseTurnsPerSample <= FOC_ZERO) ||
        (ptConfig->qElectricalBaseTurnsPerSample > FOC_ONE) ||
        (ptConfig->hwMinimumSettlingSamples == 0U) ||
        (ptConfig->hwAverageSamples == 0U) ||
        (ptConfig->hwPhaseTimeoutSamples == 0U)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    wRequiredSamples =
        (uint32_t)ptConfig->hwMinimumSettlingSamples +
        (uint32_t)ptConfig->hwAverageSamples;
    if ((uint32_t)ptConfig->hwPhaseTimeoutSamples < wRequiredSamples) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Validate configuration and initialize one identifier instance.
 * @param ptIdentify Instance that owns run state.
 * @param ptConfig PU excitation, limits, sample counts, and timeout.
 * @return FOC_RESULT_OK or an argument/range error.
 */
foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_cfg_t *ptConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    (void)memset(ptIdentify, 0, sizeof(*ptIdentify));
    if (ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }

    eResult = identify_validate_config(ptConfig);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    ptIdentify->tCfg = *ptConfig;
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_IDLE;
    ptIdentify->bInitialized = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Clear one run while preserving its validated configuration.
 * @param ptIdentify Instance whose next run is prepared.
 * @return None.
 */
static void identify_clear_run(foc_identify_t *ptIdentify)
{
    foc_identify_cfg_t tConfig = ptIdentify->tCfg;

    (void)memset(ptIdentify, 0, sizeof(*ptIdentify));
    ptIdentify->tCfg = tConfig;
    ptIdentify->bInitialized = true;
}

/**
 * @brief Start from IDLE or a consumed terminal state.
 * @param ptIdentify Initialized instance.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or an argument error.
 */
foc_result_t foc_identify_Start(foc_identify_t *ptIdentify)
{
    foc_identify_status_e eStatus = FOC_IDENTIFY_STATUS_IDLE;
    bool bTerminal = false;

    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->bInitialized == false) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    eStatus = ptIdentify->tOutput.eStatus;
    bTerminal = (bool)((eStatus == FOC_IDENTIFY_STATUS_COMPLETE) ||
                       (eStatus == FOC_IDENTIFY_STATUS_ERROR) ||
                       (eStatus == FOC_IDENTIFY_STATUS_ABORTED));
    if ((eStatus != FOC_IDENTIFY_STATUS_IDLE) &&
        ((bTerminal == false) ||
         (ptIdentify->bTerminalConsumed == false))) {
        return FOC_RESULT_BUSY;
    }

    identify_clear_run(ptIdentify);
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_STARTING;
    return FOC_RESULT_OK;
}

/**
 * @brief Validate paired input, voltage range, and vector current limit.
 * @param ptIdentify Identifier whose PU current limit applies.
 * @param ptSample Current and prior-interval model voltage.
 * @return FOC_RESULT_OK or a sample/safety error.
 */
static foc_result_t identify_validate_sample(
    const foc_identify_t *ptIdentify,
    const foc_identify_sample_t *ptSample)
{
    if (ptSample->bValid == false) {
        return FOC_RESULT_SAFETY;
    }
    if ((identify_is_pu(ptSample->tCurrentAlphaBeta.qAlpha) == false) ||
        (identify_is_pu(ptSample->tCurrentAlphaBeta.qBeta) == false) ||
        (identify_is_pu(ptSample->tVmodelAlphaBeta.qAlpha) == false) ||
        (identify_is_pu(ptSample->tVmodelAlphaBeta.qBeta) == false)) {
        return FOC_RESULT_OUT_OF_RANGE;
    }

#if defined(FOC_NUMERIC_FLOAT)
    {
        float fCurrent =
            (ptSample->tCurrentAlphaBeta.qAlpha *
             ptSample->tCurrentAlphaBeta.qAlpha) +
            (ptSample->tCurrentAlphaBeta.qBeta *
             ptSample->tCurrentAlphaBeta.qBeta);
        float fLimit = ptIdentify->tCfg.qCurrentLimitPu;
        float fVoltage =
            (ptSample->tVmodelAlphaBeta.qAlpha *
             ptSample->tVmodelAlphaBeta.qAlpha) +
            (ptSample->tVmodelAlphaBeta.qBeta *
             ptSample->tVmodelAlphaBeta.qBeta);

        if (fCurrent > (fLimit * fLimit)) {
            return FOC_RESULT_SAFETY;
        }
        if (fVoltage > 1.0f) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
    }
#else
    {
        int64_t llCurrent =
            ((int64_t)ptSample->tCurrentAlphaBeta.qAlpha *
             (int64_t)ptSample->tCurrentAlphaBeta.qAlpha) +
            ((int64_t)ptSample->tCurrentAlphaBeta.qBeta *
             (int64_t)ptSample->tCurrentAlphaBeta.qBeta);
        int64_t llLimit =
            (int64_t)ptIdentify->tCfg.qCurrentLimitPu;
        int64_t llVoltage =
            ((int64_t)ptSample->tVmodelAlphaBeta.qAlpha *
             (int64_t)ptSample->tVmodelAlphaBeta.qAlpha) +
            ((int64_t)ptSample->tVmodelAlphaBeta.qBeta *
             (int64_t)ptSample->tVmodelAlphaBeta.qBeta);
        int64_t llOne = (int64_t)FOC_ONE;

        if (llCurrent > (llLimit * llLimit)) {
            return FOC_RESULT_SAFETY;
        }
        if (llVoltage > (llOne * llOne)) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
    }
#endif
    return FOC_RESULT_OK;
}

/**
 * @brief Advance one resistance settling phase.
 * @param ptIdentify Active instance.
 * @return None.
 */
static void identify_process_resistance_settle(
    foc_identify_t *ptIdentify)
{
    ptIdentify->wPhaseSamples++;
    if (ptIdentify->wPhaseSamples >=
        (uint32_t)ptIdentify->tCfg.hwMinimumSettlingSamples) {
        ptIdentify->tOutput.eStatus =
            FOC_IDENTIFY_STATUS_RESISTANCE_AVERAGE;
    }
}

/**
 * @brief Track D-axis extrema and test the completed Rs window.
 * @param ptIdentify Active instance.
 * @param qCurrent Validated D-axis current sample in PU.
 * @param hwCount Window count after inserting this sample.
 * @return True when the complete window passes the stability limit.
 */
static bool identify_collect_resistance_window(
    foc_identify_t *ptIdentify,
    foc_scalar_t qCurrent,
    uint16_t hwCount)
{
    foc_scalar_t qSpread = FOC_ZERO;
    bool bStable = false;

    if (ptIdentify->hwWindowSamples == 0U) {
        ptIdentify->qWindowCurrentMin = qCurrent;
        ptIdentify->qWindowCurrentMax = qCurrent;
    } else {
        if (qCurrent < ptIdentify->qWindowCurrentMin) {
            ptIdentify->qWindowCurrentMin = qCurrent;
        } else if (qCurrent > ptIdentify->qWindowCurrentMax) {
            ptIdentify->qWindowCurrentMax = qCurrent;
        } else {
            /* The current remains inside the observed extrema. */
        }
    }
    ptIdentify->hwWindowSamples = hwCount;

    if (hwCount >= ptIdentify->tCfg.hwAverageSamples) {
        qSpread = (foc_scalar_t)(ptIdentify->qWindowCurrentMax -
                                 ptIdentify->qWindowCurrentMin);
        if (qSpread < FOC_ZERO) {
            qSpread = (foc_scalar_t)(-qSpread);
        }
        /* Reject a window that still has unresolved current drift. */
        bStable = (bool)(
            qSpread <= ptIdentify->tCfg.qCurrentStabilityTolerancePu);
        if (bStable == false) {
            ptIdentify->hwWindowSamples = 0U;
        }
    }
    return bStable;
}

/**
 * @brief Collect and finish one low/high resistance average phase.
 * @param ptIdentify Active instance.
 * @param ptSample Validated paired sample.
 * @return FOC_RESULT_OK or a stability, range, or timeout error.
 */
static foc_result_t identify_process_resistance_average(
    foc_identify_t *ptIdentify,
    const foc_identify_sample_t *ptSample)
{
    foc_identify_status_e eBefore = ptIdentify->tOutput.eStatus;
    bool bLow = (bool)(ptIdentify->bHighResistance == false);
    bool bStable = false;
    foc_result_t eResult = FOC_RESULT_OK;
    uint16_t hwCount = (uint16_t)(ptIdentify->hwWindowSamples + 1U);
    foc_scalar_t qCurrent = ptSample->tCurrentAlphaBeta.qAlpha;
    foc_scalar_t qVoltage = ptSample->tVmodelAlphaBeta.qAlpha;

    ptIdentify->wPhaseSamples++;
    if (ptIdentify->hwWindowSamples == 0U) {
        ptIdentify->qWindowCurrentMean = qCurrent;
        ptIdentify->qWindowVoltageMean = qVoltage;
    } else {
        ptIdentify->qWindowCurrentMean =
            identify_update_mean(ptIdentify->qWindowCurrentMean,
                                 qCurrent, hwCount);
        ptIdentify->qWindowVoltageMean =
            identify_update_mean(ptIdentify->qWindowVoltageMean,
                                 qVoltage, hwCount);
    }
    bStable = identify_collect_resistance_window(
        ptIdentify, qCurrent, hwCount);
    if (bStable != false) {
        if (bLow != false) {
            ptIdentify->qLowCurrentMean =
                ptIdentify->qWindowCurrentMean;
            ptIdentify->qLowVoltageMean =
                ptIdentify->qWindowVoltageMean;
            ptIdentify->bHighResistance = true;
            identify_begin_phase(
                ptIdentify,
                FOC_IDENTIFY_STATUS_RESISTANCE_SETTLE,
                ptIdentify->tCfg.qResistanceHighVoltagePu, FOC_ZERO);
        } else {
            foc_scalar_t qDeltaVoltage = (foc_scalar_t)(
                ptIdentify->qWindowVoltageMean -
                ptIdentify->qLowVoltageMean);
            foc_scalar_t qDeltaCurrent = (foc_scalar_t)(
                ptIdentify->qWindowCurrentMean -
                ptIdentify->qLowCurrentMean);

            if ((foc_abs(qDeltaCurrent) <
                 ptIdentify->tCfg.qMinimumResistanceDeltaPu) ||
                (qDeltaVoltage == FOC_ZERO) ||
                (((qDeltaVoltage > FOC_ZERO) &&
                  (qDeltaCurrent <= FOC_ZERO)) ||
                 ((qDeltaVoltage < FOC_ZERO) &&
                  (qDeltaCurrent >= FOC_ZERO)))) {
                eResult = FOC_RESULT_SAFETY;
            } else {
                eResult = foc_div_checked(
                    qDeltaVoltage, qDeltaCurrent,
                    &ptIdentify->tResult.qResistancePu);
#if defined(FOC_NUMERIC_FLOAT)
                if ((eResult == FOC_RESULT_OK) &&
                    (isfinite(ptIdentify->tResult.qResistancePu) == 0)) {
                    eResult = FOC_RESULT_OUT_OF_RANGE;
                }
#endif
                if ((eResult == FOC_RESULT_OK) &&
                    (ptIdentify->tResult.qResistancePu <= FOC_ZERO)) {
                    eResult = FOC_RESULT_OUT_OF_RANGE;
                }
                if (eResult == FOC_RESULT_OK) {
                    ptIdentify->bQAxis = false;
                    identify_begin_phase(
                        ptIdentify, FOC_IDENTIFY_STATUS_AXIS_RESET,
                        FOC_ZERO, FOC_ZERO);
                }
            }
        }
    }

    if ((eResult == FOC_RESULT_OK) &&
        (ptIdentify->tOutput.eStatus == eBefore) &&
        (ptIdentify->wPhaseSamples >=
         (uint32_t)ptIdentify->tCfg.hwPhaseTimeoutSamples)) {
        eResult = FOC_RESULT_SAFETY;
    }
    return eResult;
}

/**
 * @brief Wait for the active axis current to reset, then apply its step.
 * @param ptIdentify Active instance.
 * @param ptSample Validated current sample.
 * @return FOC_RESULT_OK or a reset timeout.
 */
static foc_result_t identify_process_reset(
    foc_identify_t *ptIdentify,
    const foc_identify_sample_t *ptSample)
{
    foc_scalar_t qAxisCurrent = ptIdentify->bQAxis != false
        ? ptSample->tCurrentAlphaBeta.qBeta
        : ptSample->tCurrentAlphaBeta.qAlpha;

    ptIdentify->wPhaseSamples++;
    if (foc_abs(qAxisCurrent) <=
        ptIdentify->tCfg.qResetCurrentLimitPu) {
        ptIdentify->qInitialAxisCurrent = qAxisCurrent;
        if (ptIdentify->bQAxis != false) {
            identify_begin_phase(ptIdentify,
                                 FOC_IDENTIFY_STATUS_AXIS_STEP,
                                 FOC_ZERO,
                                 ptIdentify->tCfg.qInductanceQVoltagePu);
        } else {
            identify_begin_phase(ptIdentify,
                                 FOC_IDENTIFY_STATUS_AXIS_STEP,
                                 ptIdentify->tCfg.qInductanceDVoltagePu,
                                 FOC_ZERO);
        }
        return FOC_RESULT_OK;
    }
    if (ptIdentify->wPhaseSamples >=
        (uint32_t)ptIdentify->tCfg.hwPhaseTimeoutSamples) {
        return FOC_RESULT_SAFETY;
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Calculate L from the first complete interval at the 95% crossing.
 * @param ptIdentify Active instance with measured Rs.
 * @param wIntervals Number of complete intervals; the first is one.
 * @param pqInductance Destination PU inductance.
 * @return FOC_RESULT_OK or a numeric range error.
 */
static foc_result_t identify_calculate_inductance(
    const foc_identify_t *ptIdentify,
    uint32_t wIntervals,
    foc_scalar_t *pqInductance)
{
#if defined(FOC_NUMERIC_FLOAT)
    {
        float fInductance = ptIdentify->tResult.qResistancePu *
            IDENTIFY_RL_SCALE_FLOAT *
            ptIdentify->tCfg.qElectricalBaseTurnsPerSample *
            (float)wIntervals;

        if ((isfinite(fInductance) == 0) || (fInductance <= 0.0f)) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        *pqInductance = fInductance;
    }
#else
    {
        int64_t llScale = (int64_t)FOC_Q_SCALE;
        int64_t llDenominator = llScale * llScale;
        int64_t llProduct =
            (int64_t)ptIdentify->tResult.qResistancePu *
            (int64_t)ptIdentify->tCfg.qElectricalBaseTurnsPerSample;
        int64_t llMultiplier =
            (int64_t)IDENTIFY_RL_SCALE_Q15 * (int64_t)wIntervals;
        int64_t llRaw = 0;

        if ((llProduct <= INT64_C(0)) ||
            (llMultiplier <= INT64_C(0)) ||
            (llProduct > ((INT64_MAX - (llDenominator / INT64_C(2))) /
                          llMultiplier))) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        llProduct *= llMultiplier;
        llRaw = (llProduct + (llDenominator / INT64_C(2))) /
                llDenominator;
        if ((llRaw <= INT64_C(0)) || (llRaw > (int64_t)INT32_MAX)) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        *pqInductance = (foc_scalar_t)llRaw;
    }
#endif
    return FOC_RESULT_OK;
}

/**
 * @brief Process a D/Q voltage step until its 95% current crossing.
 * @param ptIdentify Active instance.
 * @param ptSample Validated paired sample.
 * @return FOC_RESULT_OK or a crossing/range/timeout error.
 */
static foc_result_t identify_process_rise(
    foc_identify_t *ptIdentify,
    const foc_identify_sample_t *ptSample)
{
    foc_scalar_t qCurrent = ptIdentify->bQAxis != false
        ? ptSample->tCurrentAlphaBeta.qBeta
        : ptSample->tCurrentAlphaBeta.qAlpha;
    foc_scalar_t qVoltage = ptIdentify->bQAxis != false
        ? ptSample->tVmodelAlphaBeta.qBeta
        : ptSample->tVmodelAlphaBeta.qAlpha;
    foc_scalar_t qFinalCurrent = FOC_ZERO;
    foc_scalar_t qDeltaCurrent = FOC_ZERO;
    foc_scalar_t qThreshold = FOC_ZERO;
    foc_scalar_t qInductance = FOC_ZERO;
    foc_result_t eResult = FOC_RESULT_OK;
    bool bCrossed = false;

    if (ptIdentify->bRisePrimed == false) {
        /* The new PWM command cannot describe the prior Vmodel interval. */
        ptIdentify->bRisePrimed = true;
        return FOC_RESULT_OK;
    }
    ptIdentify->wPhaseSamples++;
    ptIdentify->qWindowVoltageMean = identify_update_mean(
        ptIdentify->qWindowVoltageMean, qVoltage,
        (uint16_t)ptIdentify->wPhaseSamples);
    eResult = foc_div_checked(ptIdentify->qWindowVoltageMean,
                              ptIdentify->tResult.qResistancePu,
                              &qFinalCurrent);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
#if defined(FOC_NUMERIC_FLOAT)
    if (isfinite(qFinalCurrent) == 0) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#endif
    if (foc_abs(qFinalCurrent) > ptIdentify->tCfg.qCurrentLimitPu) {
        return FOC_RESULT_SAFETY;
    }

    qDeltaCurrent = (foc_scalar_t)(
        qFinalCurrent - ptIdentify->qInitialAxisCurrent);
    qThreshold = (foc_scalar_t)(ptIdentify->qInitialAxisCurrent +
        foc_mul_wide(qDeltaCurrent, FOC_SCALAR(0.95f)));
    if (qThreshold == ptIdentify->qInitialAxisCurrent) {
        return FOC_RESULT_OUT_OF_RANGE;
    }

    if (qDeltaCurrent > FOC_ZERO) {
        if (ptIdentify->qInitialAxisCurrent >= qThreshold) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        bCrossed = (bool)(qCurrent >= qThreshold);
    } else if (qDeltaCurrent < FOC_ZERO) {
        if (ptIdentify->qInitialAxisCurrent <= qThreshold) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        bCrossed = (bool)(qCurrent <= qThreshold);
    } else {
        return FOC_RESULT_OUT_OF_RANGE;
    }

    if (bCrossed != false) {
        eResult = identify_calculate_inductance(
            ptIdentify, ptIdentify->wPhaseSamples, &qInductance);
        if (eResult != FOC_RESULT_OK) {
            return eResult;
        }
        if (ptIdentify->bQAxis != false) {
            ptIdentify->tResult.qInductanceQPu = qInductance;
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_COMPLETE;
            identify_set_reference(ptIdentify, FOC_ZERO, FOC_ZERO);
        } else {
            ptIdentify->tResult.qInductanceDPu = qInductance;
            ptIdentify->bQAxis = true;
            identify_begin_phase(ptIdentify, FOC_IDENTIFY_STATUS_AXIS_RESET,
                                 FOC_ZERO, FOC_ZERO);
        }
    } else if (ptIdentify->wPhaseSamples >=
               (uint32_t)ptIdentify->tCfg.hwPhaseTimeoutSamples) {
        return FOC_RESULT_SAFETY;
    } else {
        /* Continue until the crossing or bounded phase timeout. */
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Handle IDLE and terminal states without processing a sample.
 * @param ptIdentify Instance whose state is inspected.
 * @param ptOutput Caller-owned output record.
 * @param eStatus Current state snapshot.
 * @return True when no sample should be processed.
 */
static bool identify_handle_inactive(foc_identify_t *ptIdentify,
                                     foc_identify_output_t *ptOutput,
                                     foc_identify_status_e eStatus)
{
    bool bTerminal = (bool)(
        (eStatus == FOC_IDENTIFY_STATUS_COMPLETE) ||
        (eStatus == FOC_IDENTIFY_STATUS_ERROR) ||
        (eStatus == FOC_IDENTIFY_STATUS_ABORTED));

    if (eStatus == FOC_IDENTIFY_STATUS_IDLE) {
        *ptOutput = (foc_identify_output_t){0};
        return true;
    }
    if (bTerminal != false) {
        *ptOutput = ptIdentify->tOutput;
        ptOutput->bReferenceChanged = false;
        ptIdentify->bTerminalConsumed = true;
    }
    return bTerminal;
}

/**
 * @brief Process one paired sample and return the next voltage command.
 * @param ptIdentify Active identifier.
 * @param ptSample Current PU and prior-interval Vmodel.
 * @param ptOutput Caller-owned command and status.
 * @return FOC_RESULT_OK or a failure reason.
 */
foc_result_t foc_identify_Step(foc_identify_t *ptIdentify,
                               const foc_identify_sample_t *ptSample,
                               foc_identify_output_t *ptOutput)
{
    foc_result_t eResult = FOC_RESULT_OK;
    foc_identify_status_e eStatus = FOC_IDENTIFY_STATUS_IDLE;

    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptOutput == NULL) {
        if ((ptIdentify->bInitialized != false) &&
            (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_IDLE) &&
            (ptIdentify->tOutput.eStatus !=
             FOC_IDENTIFY_STATUS_COMPLETE) &&
            (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_ERROR) &&
            (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_ABORTED)) {
            (void)identify_fail(ptIdentify, FOC_RESULT_NULL);
        }
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->bInitialized == false) {
        *ptOutput = (foc_identify_output_t){0};
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    eStatus = ptIdentify->tOutput.eStatus;
    if (identify_handle_inactive(ptIdentify, ptOutput, eStatus) != false) {
        if (eStatus == FOC_IDENTIFY_STATUS_ERROR) {
            return ptIdentify->eFailure;
        }
        return eStatus == FOC_IDENTIFY_STATUS_IDLE
            ? FOC_RESULT_INVALID_ARGUMENT : FOC_RESULT_OK;
    }

    ptIdentify->tOutput.bReferenceChanged = false;
    if (ptSample == NULL) {
        eResult = FOC_RESULT_NULL;
    } else {
        eResult = identify_validate_sample(ptIdentify, ptSample);
    }
    if (eResult == FOC_RESULT_OK) {
        switch (eStatus) {
        case FOC_IDENTIFY_STATUS_STARTING:
            ptIdentify->bHighResistance = false;
            identify_begin_phase(
                ptIdentify,
                FOC_IDENTIFY_STATUS_RESISTANCE_SETTLE,
                ptIdentify->tCfg.qResistanceLowVoltagePu,
                FOC_ZERO);
            break;
        case FOC_IDENTIFY_STATUS_RESISTANCE_SETTLE:
            identify_process_resistance_settle(ptIdentify);
            break;
        case FOC_IDENTIFY_STATUS_RESISTANCE_AVERAGE:
            eResult = identify_process_resistance_average(
                ptIdentify, ptSample);
            break;
        case FOC_IDENTIFY_STATUS_AXIS_RESET:
            eResult = identify_process_reset(ptIdentify, ptSample);
            break;
        case FOC_IDENTIFY_STATUS_AXIS_STEP:
            eResult = identify_process_rise(ptIdentify, ptSample);
            break;
        default:
            eResult = FOC_RESULT_INVALID_ARGUMENT;
            break;
        }
    }

    if (eResult != FOC_RESULT_OK) {
        (void)identify_fail(ptIdentify, eResult);
    }

    *ptOutput = ptIdentify->tOutput;
    ptIdentify->bTerminalConsumed = (bool)(
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE) ||
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR));
    return eResult;
}

/**
 * @brief Abort an active run and discard every partial result.
 * @param ptIdentify Instance to abort; null is ignored.
 * @return None.
 */
void foc_identify_Abort(foc_identify_t *ptIdentify)
{
    if ((ptIdentify != NULL) && (ptIdentify->bInitialized != false)) {
        ptIdentify->tResult = (foc_identify_result_t){0};
        ptIdentify->eFailure = FOC_RESULT_SAFETY;
        ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ABORTED;
        ptIdentify->bTerminalConsumed = false;
        identify_set_reference(ptIdentify, FOC_ZERO, FOC_ZERO);
    }
}

/**
 * @brief Copy results only after all three parameters are complete.
 * @param ptIdentify Source instance.
 * @param ptResult Destination for PU Rs, Ld, and Lq.
 * @return FOC_RESULT_OK or an availability/argument error.
 */
foc_result_t foc_identify_GetResult(const foc_identify_t *ptIdentify,
                                    foc_identify_result_t *ptResult)
{
    if ((ptIdentify == NULL) || (ptResult == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (ptIdentify->bInitialized == false) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_COMPLETE) {
        if ((ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR) ||
            (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ABORTED)) {
            return FOC_RESULT_SAFETY;
        }
        return FOC_RESULT_BUSY;
    }
    *ptResult = ptIdentify->tResult;
    return FOC_RESULT_OK;
}
