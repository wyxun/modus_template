#include <assert.h>

#include "motor_position.h"

static foc_observer_output_t s_tEstimate = {0};

foc_result_t foc_observer_Init(foc_observer_t *ptObserver,
    const struct motor_params_t *ptParams,
    const foc_observer_cfg_t *ptCfg)
{
    (void)ptParams;
    (void)ptCfg;
    *ptObserver = (foc_observer_t){0};
    return FOC_RESULT_OK;
}

foc_result_t foc_observer_Step(foc_observer_t *ptObserver,
    const foc_observer_input_t *ptInput)
{
    (void)ptInput;
    ptObserver->tOutput = s_tEstimate;
    return FOC_RESULT_OK;
}

void foc_observer_Reset(foc_observer_t *ptObserver)
{
    *ptObserver = (foc_observer_t){0};
}

int main(void)
{
    motor_position_t tPosition = {0};
    motor_position_cfg_t tCfg = {0};
    motor_position_sample_t tSample = {0};
    motor_electrical_feedback_t tControl = {0};
    struct motor_params_t *ptDummy = (struct motor_params_t *)1;
    uint32_t wIndex = 0U;

    tCfg.eSource = MOTOR_POSITION_SOURCE_HARD_DRAG;
    tCfg.chPolePairs = 7U;
    tCfg.wControlFrequencyHz = 20000U;
    tCfg.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(200.0f);
    tCfg.ptMotorParams = ptDummy;
    tCfg.bObserverTakeover = true;
    tCfg.bAutoTakeover = true;
    tCfg.wQualificationSteps = 3U;
    tCfg.wBlendSteps = 4U;
    tCfg.wMaxForcedSteps = 100U;
    tCfg.qMinimumBemfPu = FOC_SCALAR(0.13f);
    tCfg.qMinimumSpeedPu = FOC_SCALAR(0.1f);
    tCfg.qMaximumSpeedErrorRatio = FOC_SCALAR(0.1f);
    tCfg.qMaximumAngleErrorTurns = FOC_SCALAR(0.125f);
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.wRunGeneration = 1U;
    tSample.tHardDragCandidate = (motor_electrical_feedback_t){
        .tElectricalAngle = {0U},
        .qElectricalSpeedPu = FOC_SCALAR(0.2f),
        .bValid = true,
    };
    s_tEstimate = (foc_observer_output_t){
        .tElectricalAngle = {0x08000000U},
        .qElectricalSpeedTurnsPerSecond = FOC_SCALAR(40.0f),
        .qSignalStrengthPu = FOC_SCALAR(0.2f),
        .bValid = true,
    };
    for (wIndex = 0U; wIndex < 2U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
        assert(tControl.tElectricalAngle.wBam32 == 0U);
        assert(motor_position_TakeEvent(&tPosition) ==
               MOTOR_POSITION_EVENT_NONE);
    }
    for (wIndex = 0U; wIndex < 100U; wIndex++) {
        uint32_t wPrevious = tControl.tElectricalAngle.wBam32;

        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
        assert((int32_t)(tControl.tElectricalAngle.wBam32 -
                         wPrevious) <=
               (int32_t)tPosition.tHandoff.wMaxBlendCorrectionBam32);
        if (tPosition.tHandoff.eFeedbackState ==
            MOTOR_POSITION_FEEDBACK_OBSERVER) {
            break;
        }
    }
    assert(wIndex < 100U);
    assert(tControl.tElectricalAngle.wBam32 > 0x08000000U);
    assert(motor_position_TakeEvent(&tPosition) ==
           MOTOR_POSITION_EVENT_OBSERVER_ACTIVE);
    assert(motor_position_TakeEvent(&tPosition) ==
           MOTOR_POSITION_EVENT_NONE);
    tSample.tHardDragCandidate.bValid = false;
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_OK);
    assert(tControl.bValid);
    s_tEstimate.qElectricalSpeedTurnsPerSecond =
        FOC_SCALAR(400.0f);
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_SAFETY);
    assert(motor_position_TakeEvent(&tPosition) ==
           MOTOR_POSITION_EVENT_OBSERVER_LOST);

    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.tHardDragCandidate.bValid = true;
    tSample.wRunGeneration = 2U;
    s_tEstimate.bValid = true;
    s_tEstimate.qElectricalSpeedTurnsPerSecond =
        FOC_SCALAR(40.0f);
    s_tEstimate.qSignalStrengthPu = FOC_SCALAR(0.01f);
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert(tPosition.tHandoff.wQualifiedCount == 0U);
    s_tEstimate.qSignalStrengthPu = FOC_SCALAR(0.2f);
    s_tEstimate.qElectricalSpeedTurnsPerSecond =
        FOC_SCALAR(48.0f);
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert(tPosition.tHandoff.wQualifiedCount == 0U);
    s_tEstimate.qElectricalSpeedTurnsPerSecond =
        FOC_SCALAR(40.0f);
    tSample.wRunGeneration = 5U;
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert(tPosition.tHandoff.eFeedbackState ==
           MOTOR_POSITION_FEEDBACK_BLEND);
    s_tEstimate.bValid = false;
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_SAFETY);
    assert(motor_position_TakeEvent(&tPosition) ==
           MOTOR_POSITION_EVENT_OBSERVER_LOST);

    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.wRunGeneration = 4U;
    tSample.tHardDragCandidate.tElectricalAngle.wBam32 =
        0xF8000000U;
    s_tEstimate.bValid = true;
    s_tEstimate.tElectricalAngle.wBam32 = 0x08000000U;
    s_tEstimate.qSignalStrengthPu = FOC_SCALAR(0.2f);
    s_tEstimate.qElectricalSpeedTurnsPerSecond =
        FOC_SCALAR(40.0f);
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert((int32_t)(tControl.tElectricalAngle.wBam32 -
                     0xF8000000U) > 0);
    assert((int32_t)(tControl.tElectricalAngle.wBam32 -
                     0xF8000000U) < 0x10000000);

    tCfg.bAutoTakeover = false;
    tCfg.wMaxForcedSteps = 8U;
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.wRunGeneration = 3U;
    s_tEstimate.bValid = true;
    for (wIndex = 0U; wIndex < 7U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_SAFETY);
    assert(motor_position_TakeEvent(&tPosition) ==
           MOTOR_POSITION_EVENT_OBSERVER_LOST);
    return 0;
}
