/****************************************************************************
 * @file    motor_position.c
 * @brief   Sensor conversion and optional shadow position estimation.
 ****************************************************************************/
#include "motor_position.h"

#include <stddef.h>
#include <math.h>

#include "foc_port.h"

static foc_angle_t _motor_position_ToElectrical(
    const motor_position_t *ptPosition, foc_angle_t tMechanical)
{
    foc_angle_t tElectrical = {0U};
    tElectrical.wBam32 = (uint32_t)(
        (uint64_t)tMechanical.wBam32 * ptPosition->chPolePairs);
    return tElectrical;
}

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
static void _motor_position_ResetHandoff(motor_position_t *ptPosition)
{
    ptPosition->tHandoff.eFeedbackState = MOTOR_POSITION_FEEDBACK_PRIMARY;
    ptPosition->tHandoff.wQualifiedCount = 0U;
    ptPosition->tHandoff.wBlendCount = 0U;
    ptPosition->tHandoff.nBlendCorrectionBam32 = 0;
    ptPosition->tHandoff.qBlendWeight = FOC_ZERO;
}

static bool _motor_position_SmoQualified(
    const motor_position_t *ptPosition,
    const motor_electrical_feedback_t *ptForced,
    const motor_electrical_feedback_t *ptObserved)
{
    const foc_smo_output_t *ptRaw =
        &ptPosition->tEstimate;
    foc_scalar_t qAngleError = FOC_ZERO;
    foc_scalar_t qSpeedError = FOC_ZERO;
    foc_scalar_t qRawSpeedPu = FOC_ZERO;
    foc_scalar_t qAllowedSpeedError = FOC_ZERO;

    if (!ptRaw->bValid || !ptObserved->bValid ||
        !foc_scalar_is_finite(ptObserved->qElectricalSpeedPu) ||
        !foc_scalar_is_finite(ptPosition->tEstimate.qBemfStrengthPu) ||
        ptPosition->tEstimate.qBemfStrengthPu <
            ptPosition->tTracking.qMinimumBemfPu ||
        foc_abs(ptForced->qElectricalSpeedPu) <
            ptPosition->tTracking.qMinimumSpeedPu) {
        return false;
    }
    qAngleError = foc_abs(foc_angle_diff(
        ptObserved->tElectricalAngle, ptForced->tElectricalAngle));
    qSpeedError = foc_abs(foc_sub_sat(
        ptObserved->qElectricalSpeedPu,
        ptForced->qElectricalSpeedPu));
    qRawSpeedPu = ptRaw->qElectricalSpeedPu;
    qAllowedSpeedError = foc_mul_pu(
        foc_abs(ptForced->qElectricalSpeedPu),
        ptPosition->tTracking.qMaximumSpeedErrorRatio);
    return qAngleError <= ptPosition->tTracking.qMaximumAngleErrorTurns &&
           qSpeedError <= qAllowedSpeedError &&
           foc_abs(foc_sub_sat(qRawSpeedPu,
               ptForced->qElectricalSpeedPu)) <=
               foc_add_sat(qAllowedSpeedError,
                   foc_mul_pu(qAllowedSpeedError, FOC_HALF)) &&
           ((ptObserved->qElectricalSpeedPu > FOC_ZERO) ==
            (ptForced->qElectricalSpeedPu > FOC_ZERO));
}

static void _motor_position_FilterSmoSpeed(
    motor_position_t *ptPosition)
{
    const foc_smo_output_t *ptRaw =
        &ptPosition->tEstimate;
    foc_scalar_t qRawSpeedPu = FOC_ZERO;

    if (!ptRaw->bValid || !foc_scalar_is_finite(
            ptRaw->qElectricalSpeedPu)) {
        ptPosition->tTracking.bObserverSpeedReady = false;
        return;
    }
    qRawSpeedPu = ptRaw->qElectricalSpeedPu;
    if (foc_abs(qRawSpeedPu) > FOC_ONE) {
        ptPosition->tTracking.bObserverSpeedReady = false;
        return;
    }
    if (!ptPosition->tTracking.bObserverSpeedReady) {
        ptPosition->tTracking.qObserverSpeedFilteredPu = qRawSpeedPu;
        ptPosition->tTracking.bObserverSpeedReady = true;
    } else {
        ptPosition->tTracking.qObserverSpeedFilteredPu = foc_add_sat(
            ptPosition->tTracking.qObserverSpeedFilteredPu,
            foc_mul_pu(ptPosition->tTracking.qSpeedFilterAlpha,
                foc_sub_sat(qRawSpeedPu,
                    ptPosition->tTracking.qObserverSpeedFilteredPu)));
    }
}

static void _motor_position_Blend(
    motor_position_t *ptPosition,
    const motor_electrical_feedback_t *ptForced,
    const motor_electrical_feedback_t *ptObserved,
    motor_electrical_feedback_t *ptFeedback)
{
    int32_t nDifference = (int32_t)(
        ptObserved->tElectricalAngle.wBam32 -
        ptForced->tElectricalAngle.wBam32);
    int64_t lRemaining = 0;
    int64_t lStep = 0;

    ptPosition->tHandoff.wBlendCount++;
    if (ptPosition->tHandoff.wBlendCount >= ptPosition->tHandoff.wBlendSteps) {
        ptPosition->tHandoff.qBlendWeight = FOC_ONE;
    } else {
        ptPosition->tHandoff.qBlendWeight = foc_add_sat(
            ptPosition->tHandoff.qBlendWeight,
            ptPosition->tHandoff.qBlendWeightStep);
    }
    lRemaining = (int64_t)nDifference -
                 ptPosition->tHandoff.nBlendCorrectionBam32;
    lStep = lRemaining;
    if (lStep > MOTOR_POSITION_MAX_BLEND_CORRECTION_BAM32) {
        lStep = MOTOR_POSITION_MAX_BLEND_CORRECTION_BAM32;
    } else if (lStep < -(int64_t)MOTOR_POSITION_MAX_BLEND_CORRECTION_BAM32) {
        lStep = -(int64_t)MOTOR_POSITION_MAX_BLEND_CORRECTION_BAM32;
    }
    ptPosition->tHandoff.nBlendCorrectionBam32 += (int32_t)lStep;
    *ptFeedback = *ptForced;
    ptFeedback->tElectricalAngle.wBam32 +=
        (uint32_t)ptPosition->tHandoff.nBlendCorrectionBam32;
    ptFeedback->qElectricalSpeedPu = foc_add_sat(
        ptForced->qElectricalSpeedPu,
        foc_mul_pu(foc_sub_sat(
            ptObserved->qElectricalSpeedPu,
            ptForced->qElectricalSpeedPu),
            ptPosition->tHandoff.qBlendWeight));
    if (ptPosition->tHandoff.wBlendCount >= ptPosition->tHandoff.wBlendSteps &&
        lRemaining == lStep) {
        *ptFeedback = *ptObserved;
        ptPosition->tHandoff.eFeedbackState =
            MOTOR_POSITION_FEEDBACK_OBSERVER;
        ptFeedback->eEvent =
            MOTOR_POSITION_EVENT_OBSERVER_ACTIVE;
    }
}

static foc_result_t _motor_position_SmoActiveStep(
    motor_position_t *ptPosition,
    const motor_electrical_feedback_t *ptObserved,
    motor_electrical_feedback_t *ptFeedback)
{
    foc_scalar_t qStepAngle = FOC_ZERO;
    foc_scalar_t qRawSpeedPu = FOC_ZERO;
    foc_scalar_t qRawDeviationLimit = FOC_ZERO;
    const foc_smo_output_t *ptRaw =
        &ptPosition->tEstimate;

    qStepAngle = foc_abs(foc_angle_diff(
        ptObserved->tElectricalAngle,
        ptPosition->tHandoff.tPreviousObserverAngle));
    qRawSpeedPu = ptRaw->qElectricalSpeedPu;
    qRawDeviationLimit = foc_mul_pu(
        foc_abs(ptObserved->qElectricalSpeedPu),
        ptPosition->tTracking.qMaximumSpeedErrorRatio);
    qRawDeviationLimit = foc_add_sat(
        qRawDeviationLimit, qRawDeviationLimit);
    if (!ptObserved->bValid ||
        !foc_scalar_is_finite(ptObserved->qElectricalSpeedPu) ||
        !foc_scalar_is_finite(qRawSpeedPu) ||
        !foc_scalar_is_finite(ptPosition->tEstimate.qBemfStrengthPu) ||
        ptPosition->tEstimate.qBemfStrengthPu <
            ptPosition->tTracking.qMinimumBemfPu ||
        foc_abs(ptObserved->qElectricalSpeedPu) <
            ptPosition->tTracking.qMinimumHoldSpeedPu ||
        foc_abs(ptObserved->qElectricalSpeedPu) > FOC_ONE ||
        foc_abs(foc_sub_sat(qRawSpeedPu,
            ptObserved->qElectricalSpeedPu)) > qRawDeviationLimit ||
        qStepAngle > ptPosition->tTracking.qMaximumAngleErrorTurns) {
        ptPosition->tHandoff.eFeedbackState =
            MOTOR_POSITION_FEEDBACK_FAILED;
        ptFeedback->eEvent =
            MOTOR_POSITION_EVENT_OBSERVER_LOST;
        return FOC_RESULT_SAFETY;
    }
    ptPosition->tHandoff.tPreviousObserverAngle =
        ptObserved->tElectricalAngle;
    *ptFeedback = *ptObserved;
    return FOC_RESULT_OK;
}

static foc_result_t _motor_position_SelectSmo(
    motor_position_t *ptPosition,
    const motor_position_sample_t *ptSample,
    motor_electrical_feedback_t *ptFeedback)
{
    motor_electrical_feedback_t tObserved = {0};
    const motor_electrical_feedback_t *ptForced =
        &ptSample->tHardDragCandidate;
    const foc_smo_output_t *ptRaw =
        &ptPosition->tEstimate;
    bool bQualified = false;

    if (ptPosition->tHandoff.eFeedbackState ==
        MOTOR_POSITION_FEEDBACK_FAILED) {
        return FOC_RESULT_SAFETY;
    }
    tObserved.tElectricalAngle = ptRaw->tElectricalAngle;
    tObserved.qElectricalSpeedPu =
        ptPosition->tTracking.qObserverSpeedFilteredPu;
    tObserved.bValid = ptRaw->bValid &&
                       ptPosition->tTracking.bObserverSpeedReady;
    if (tObserved.bValid) {
        int64_t lAngleLead = 0;

#if defined(FOC_NUMERIC_FIXED)
        lAngleLead = ((int64_t)ptPosition->tTracking.wAngleLeadAtOnePuBam32 *
                      tObserved.qElectricalSpeedPu) / FOC_Q_SCALE;
#else
        lAngleLead = (int64_t)(
            (float)ptPosition->tTracking.wAngleLeadAtOnePuBam32 *
            tObserved.qElectricalSpeedPu);
#endif
        tObserved.tElectricalAngle.wBam32 += (uint32_t)lAngleLead;
    }
    if (ptPosition->tHandoff.eFeedbackState ==
        MOTOR_POSITION_FEEDBACK_OBSERVER) {
        return _motor_position_SmoActiveStep(
            ptPosition, &tObserved, ptFeedback);
    }
    if (!ptForced->bValid) {
        return FOC_RESULT_SAFETY;
    }
    bQualified = !ptSample->bControlLimited &&
        _motor_position_SmoQualified(ptPosition, ptForced, &tObserved);
    if (ptPosition->tHandoff.eFeedbackState ==
            MOTOR_POSITION_FEEDBACK_BLEND && !bQualified) {
        ptPosition->tHandoff.eFeedbackState =
            MOTOR_POSITION_FEEDBACK_FAILED;
        ptFeedback->eEvent =
            MOTOR_POSITION_EVENT_OBSERVER_LOST;
        return FOC_RESULT_SAFETY;
    }
    if (!bQualified) {
        ptPosition->tHandoff.wQualifiedCount = 0U;
        ptPosition->tHandoff.wBlendCount = 0U;
        ptPosition->tHandoff.nBlendCorrectionBam32 = 0;
        ptPosition->tHandoff.qBlendWeight = FOC_ZERO;
        ptPosition->tHandoff.eFeedbackState =
            MOTOR_POSITION_FEEDBACK_PRIMARY;
    } else if (ptPosition->tHandoff.wQualifiedCount <
               ptPosition->tHandoff.wQualificationSteps) {
        ptPosition->tHandoff.wQualifiedCount++;
    }
    if (ptPosition->tHandoff.bAutoTakeover &&
        ptPosition->tHandoff.wQualifiedCount >=
            ptPosition->tHandoff.wQualificationSteps) {
        ptPosition->tHandoff.eFeedbackState =
            MOTOR_POSITION_FEEDBACK_BLEND;
    }
    if (ptPosition->tHandoff.eFeedbackState ==
        MOTOR_POSITION_FEEDBACK_BLEND) {
        _motor_position_Blend(ptPosition, ptForced,
                               &tObserved, ptFeedback);
        ptPosition->tHandoff.tPreviousObserverAngle =
            tObserved.tElectricalAngle;
        if (ptPosition->tHandoff.eFeedbackState ==
                MOTOR_POSITION_FEEDBACK_BLEND &&
            ptPosition->tHandoff.wBlendCount >=
                ptPosition->tHandoff.wMaxBlendSteps) {
            ptPosition->tHandoff.eFeedbackState =
                MOTOR_POSITION_FEEDBACK_FAILED;
            ptFeedback->eEvent =
                MOTOR_POSITION_EVENT_OBSERVER_LOST;
            return FOC_RESULT_SAFETY;
        }
        return FOC_RESULT_OK;
    }
    *ptFeedback = *ptForced;
    return FOC_RESULT_OK;
}

static foc_result_t _motor_position_ConfigureTakeover(
    motor_position_t *ptPosition,
    const motor_position_cfg_t *ptConfig)
{
    if (!ptConfig->bObserverTakeover) {
        return ptConfig->bAutoTakeover ?
            FOC_RESULT_INVALID_ARGUMENT : FOC_RESULT_OK;
    }
    if (ptConfig->eSource != MOTOR_POSITION_SOURCE_HARD_DRAG ||
        ptConfig->wControlFrequencyHz == 0U ||
        foc_to_float(ptConfig->qElectricalSpeedBaseHz) >=
            (float)ptConfig->wControlFrequencyHz / 2.0f ||
        ptConfig->wQualificationSteps == 0U ||
        ptConfig->wBlendSteps == 0U ||
        !foc_scalar_is_finite(ptConfig->qMinimumBemfPu) ||
        !foc_scalar_is_finite(ptConfig->qMinimumSpeedPu) ||
        !foc_scalar_is_finite(
            ptConfig->qMaximumSpeedErrorRatio) ||
        !foc_scalar_is_finite(ptConfig->qMaximumAngleErrorTurns) ||
        ptConfig->qMinimumBemfPu <= FOC_ZERO ||
        ptConfig->qMinimumSpeedPu <= FOC_ZERO ||
        ptConfig->qMinimumSpeedPu > FOC_ONE ||
        ptConfig->qMaximumSpeedErrorRatio <= FOC_ZERO ||
        ptConfig->qMaximumSpeedErrorRatio >= FOC_ONE ||
        ptConfig->qMaximumAngleErrorTurns <= FOC_ZERO ||
        ptConfig->qMaximumAngleErrorTurns >= FOC_HALF) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptPosition->tHandoff.bObserverTakeover = true;
    /* Preserve alpha=0.02 at 20 kHz; derive once for a changed ISR rate. */
    ptPosition->tTracking.qSpeedFilterAlpha = foc_from_float(1.0f -
        powf(0.98f, 20000.0f / (float)ptConfig->wControlFrequencyHz));
    if (ptPosition->tTracking.qSpeedFilterAlpha <= FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptPosition->tTracking.wAngleLeadAtOnePuBam32 = (uint32_t)llround(
        (double)foc_to_float(
            ptConfig->qElectricalSpeedBaseHz) *
        4294967296.0 /
        (double)ptConfig->wControlFrequencyHz);
    ptPosition->tHandoff.bAutoTakeover = ptConfig->bAutoTakeover;
    ptPosition->tHandoff.wQualificationSteps =
        ptConfig->wQualificationSteps;
    ptPosition->tHandoff.wBlendSteps = ptConfig->wBlendSteps;
    /* Maximum initial phase error / 0.5 electrical degree per step. */
    {
        uint64_t ullAngle = foc_angle_from_scalar(
            ptConfig->qMaximumAngleErrorTurns).wBam32;
        uint64_t ullCorrectionSteps =
            (ullAngle + MOTOR_POSITION_MAX_BLEND_CORRECTION_BAM32 - 1U) /
            MOTOR_POSITION_MAX_BLEND_CORRECTION_BAM32;

        if (ullCorrectionSteps > UINT32_MAX - ptConfig->wBlendSteps) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        ptPosition->tHandoff.wMaxBlendSteps = ptConfig->wBlendSteps +
            (uint32_t)ullCorrectionSteps;
    }
    ptPosition->tTracking.qMinimumBemfPu = ptConfig->qMinimumBemfPu;
    ptPosition->tTracking.qMinimumSpeedPu = ptConfig->qMinimumSpeedPu;
    /* A speed accepted within the entry error band must remain valid
     * after handoff. Derive the holding floor once, in the same units. */
    ptPosition->tTracking.qMinimumHoldSpeedPu = foc_sub_sat(
        ptConfig->qMinimumSpeedPu,
        foc_mul_pu(ptConfig->qMinimumSpeedPu,
            ptConfig->qMaximumSpeedErrorRatio));
    if (ptPosition->tTracking.qMinimumHoldSpeedPu <= FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptPosition->tTracking.qMaximumSpeedErrorRatio =
        ptConfig->qMaximumSpeedErrorRatio;
    ptPosition->tTracking.qMaximumAngleErrorTurns =
        ptConfig->qMaximumAngleErrorTurns;
    ptPosition->tHandoff.qBlendWeightStep = foc_from_float(
        1.0f / (float)ptConfig->wBlendSteps);
    if (ptPosition->tHandoff.qBlendWeightStep == FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    return FOC_RESULT_OK;
}
#endif

foc_result_t motor_position_Init(motor_position_t *ptPosition,
                                  const motor_position_cfg_t *ptConfig)
{
    foc_scalar_t qPolePairs = FOC_ZERO;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptConfig->chPolePairs == 0U ||
        ptConfig->qElectricalSpeedBaseHz <= FOC_ZERO ||
        (ptConfig->eSource != MOTOR_POSITION_SOURCE_SENSOR &&
         ptConfig->eSource != MOTOR_POSITION_SOURCE_HARD_DRAG) ||
        (ptConfig->eSource == MOTOR_POSITION_SOURCE_SENSOR &&
         ptConfig->tSensor.pContext == NULL)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if !defined(FOC_POSITION_STATIC_BINDING)
    if (ptConfig->eSource == MOTOR_POSITION_SOURCE_SENSOR &&
        ptConfig->tSensor.fnGetPosition == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#endif
    *ptPosition = (motor_position_t){0};
    ptPosition->tSensor = ptConfig->tSensor;
    ptPosition->chPolePairs = ptConfig->chPolePairs;
    ptPosition->eSource = ptConfig->eSource;
    qPolePairs = foc_from_float((float)ptConfig->chPolePairs);
    eResult = foc_div_checked(
        qPolePairs, ptConfig->qElectricalSpeedBaseHz,
        &ptPosition->qMechanicalToElectricalSpeedPuGain);
    if (eResult != FOC_RESULT_OK ||
        ptPosition->qMechanicalToElectricalSpeedPuGain <= FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    if (ptConfig->ptMotorParams == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = _motor_position_ConfigureTakeover(ptPosition,
                                                  ptConfig);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    {
        const motor_params_t *params = ptConfig->ptMotorParams;
        const foc_electrical_model_t model = {
            .wResistanceMilliohm = params->wResistanceMilliohm,
            .wInductanceDMicroHenry = params->wInductanceDMicroHenry,
            .wInductanceQMicroHenry = params->wInductanceQMicroHenry,
            .wVoltageBaseMillivolt = params->wVoltageBaseMillivolt,
            .wCurrentBaseMilliamp = params->wCurrentBaseMilliamp,
        };
        foc_smo_cfg_t observer_cfg = ptConfig->tSmoCfg;

        /* Position supplies the one control-chain base; no second default. */
        observer_cfg.qElectricalSpeedBaseHz =
            ptConfig->qElectricalSpeedBaseHz;
        eResult = foc_smo_Init(&ptPosition->tSmo,
                                    &model, &observer_cfg);
    }
    /* An unsupported monitor must not block sensor or forced feedback. */
    if (eResult == FOC_RESULT_DISABLED && !ptConfig->bObserverTakeover) {
        return FOC_RESULT_OK;
    }
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
#endif
    return FOC_RESULT_OK;
}

foc_result_t motor_position_CheckStartupBudget(
    const motor_position_t *ptPosition, uint32_t wSteps)
{
    if (ptPosition == NULL) {
        return FOC_RESULT_NULL;
    }
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    const motor_position_handoff_t *plan = &ptPosition->tHandoff;
    if (plan->bObserverTakeover &&
        (wSteps <= plan->wQualificationSteps ||
         wSteps - plan->wQualificationSteps <= plan->wMaxBlendSteps)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#else
    (void)wSteps;
#endif
    return FOC_RESULT_OK;
}

foc_result_t motor_position_Step(
    motor_position_t *ptPosition, uint32_t wNowTick,
    const motor_position_sample_t *ptSample,
    motor_electrical_feedback_t *ptFeedback)
{
    foc_position_t tMechanical = {0};
    foc_angle_t tElectrical = {0U};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL || ptSample == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptFeedback = (motor_electrical_feedback_t){0};
    if (ptPosition->wLastRunGeneration != ptSample->wRunGeneration) {
        motor_position_ResetObserver(ptPosition);
        ptPosition->wLastRunGeneration = ptSample->wRunGeneration;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
        _motor_position_ResetHandoff(ptPosition);
#endif
    }
    if (ptPosition->eSource == MOTOR_POSITION_SOURCE_HARD_DRAG) {
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
        if (ptPosition->tHandoff.bObserverTakeover) {
            return _motor_position_SelectSmo(
                ptPosition, ptSample, ptFeedback);
        }
#endif
        if (!ptSample->tHardDragCandidate.bValid) {
            return FOC_RESULT_SAFETY;
        }
        *ptFeedback = ptSample->tHardDragCandidate;
        return FOC_RESULT_OK;
    }
    if (!ptPosition->bElectricalZeroValid) {
        return FOC_RESULT_SAFETY;
    }
    eResult = FOC_SENSOR_POSITION_GET(ptPosition, wNowTick, &tMechanical);
    if (eResult != FOC_RESULT_OK || !tMechanical.bValid) {
        return eResult == FOC_RESULT_OK ? FOC_RESULT_SAFETY : eResult;
    }
    tElectrical = _motor_position_ToElectrical(
        ptPosition, tMechanical.tMechanicalAngle);
    ptFeedback->tElectricalAngle = foc_angle_add(
        tElectrical,
        (foc_angle_t){0U - ptPosition->tElectricalZero.wBam32});
    ptFeedback->qElectricalSpeedPu = foc_mul_wide(
        tMechanical.qMechanicalSpeed,
        ptPosition->qMechanicalToElectricalSpeedPuGain);
    ptFeedback->bValid = true;
    return FOC_RESULT_OK;
}

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
void motor_position_ObserverStep(
    motor_position_t *ptPosition,
    const motor_position_sample_t *ptSample)
{
    foc_smo_input_t tObserverInput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL || ptSample == NULL) {
        return;
    }
    if (ptPosition->wLastRunGeneration != ptSample->wRunGeneration) {
        motor_position_ResetObserver(ptPosition);
        ptPosition->wLastRunGeneration = ptSample->wRunGeneration;
        _motor_position_ResetHandoff(ptPosition);
    }
    if (!ptSample->bVoltageModelValid) {
        motor_position_ResetObserver(ptPosition);
        return;
    }
    tObserverInput.ptCurrentAlphaBeta = &ptSample->tCurrentAlphaBeta;
    tObserverInput.ptVoltageModelAlphaBeta =
        &ptSample->tVoltageModelAlphaBeta;
    eResult = foc_smo_Step(&ptPosition->tSmo,
                                 &tObserverInput, &ptPosition->tEstimate);
    if (eResult != FOC_RESULT_OK) {
        ptPosition->tEstimate.bValid = false;
    }
    if (ptPosition->tHandoff.bObserverTakeover) {
        _motor_position_FilterSmoSpeed(ptPosition);
    }
}
#endif

foc_result_t motor_position_CaptureZero(motor_position_t *ptPosition,
                                        uint32_t wNowTick)
{
    foc_position_t tMechanical = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptPosition->eSource != MOTOR_POSITION_SOURCE_SENSOR) {
        return FOC_RESULT_DISABLED;
    }
    motor_position_InvalidateZero(ptPosition);
    eResult = FOC_SENSOR_POSITION_GET(
        ptPosition, wNowTick, &tMechanical);
    if (eResult != FOC_RESULT_OK || !tMechanical.bValid) {
        return eResult == FOC_RESULT_OK ? FOC_RESULT_SAFETY : eResult;
    }
    ptPosition->tElectricalZero = _motor_position_ToElectrical(
        ptPosition, tMechanical.tMechanicalAngle);
    ptPosition->bElectricalZeroValid = true;
    return FOC_RESULT_OK;
}

void motor_position_InvalidateZero(motor_position_t *ptPosition)
{
    if (ptPosition != NULL) {
        ptPosition->bElectricalZeroValid = false;
        ptPosition->tElectricalZero = (foc_angle_t){0U};
    }
}

bool motor_position_ZeroValid(const motor_position_t *ptPosition)
{
    return ptPosition != NULL && ptPosition->bElectricalZeroValid;
}

bool motor_position_UsesHardDrag(const motor_position_t *ptPosition)
{
    return ptPosition != NULL &&
           ptPosition->eSource == MOTOR_POSITION_SOURCE_HARD_DRAG;
}

void motor_position_ResetObserver(motor_position_t *ptPosition)
{
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    if (ptPosition != NULL) {
        foc_smo_Reset(&ptPosition->tSmo);
        ptPosition->tEstimate = (foc_smo_output_t){0};
        ptPosition->tTracking.bObserverSpeedReady = false;
        ptPosition->tTracking.qObserverSpeedFilteredPu = FOC_ZERO;
    }
#else
    (void)ptPosition;
#endif
}
