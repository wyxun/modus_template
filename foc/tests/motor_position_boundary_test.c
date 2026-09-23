#include <assert.h>

#include "motor_position.h"

static foc_result_t test_GetSensor(const void *pContext, uint32_t wNowTick,
                                   foc_position_t *ptPosition)
{
    (void)pContext;
    (void)wNowTick;
    ptPosition->tMechanicalAngle = (foc_angle_t){0x20000000U};
    ptPosition->qMechanicalSpeed = FOC_SCALAR(10.0f);
    ptPosition->bValid = true;
    return FOC_RESULT_OK;
}

int main(void)
{
    uint8_t chContext = 0U;
    motor_position_t tPosition = {0};
    motor_position_cfg_t tCfg = {0};
    motor_position_sample_t tSample = {0};
    motor_electrical_feedback_t tFeedback = {0};

    tCfg.tSensor.fnGetPosition = test_GetSensor;
    tCfg.tSensor.pContext = &chContext;
    tCfg.chPolePairs = 7U;
    tCfg.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    assert(FOC_POSITION_GET(&tPosition, 1U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    assert(tFeedback.bValid);
    assert(tFeedback.tElectricalAngle.wBam32 == 0xE0000000U);
    tSample.tHardDragCandidate = (motor_electrical_feedback_t){
        .tElectricalAngle = {0x40000000U},
        .qElectricalSpeedPu = FOC_SCALAR(0.05f),
        .bValid = true,
    };
    assert(FOC_POSITION_GET(&tPosition, 1U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    assert(tFeedback.tElectricalAngle.wBam32 == 0xE0000000U);
    assert(foc_to_float(tFeedback.qElectricalSpeedPu) > 0.699f);
    assert(foc_to_float(tFeedback.qElectricalSpeedPu) < 0.701f);

    assert(motor_position_CaptureZero(&tPosition, 2U) == FOC_RESULT_OK);
    assert(motor_position_ZeroValid(&tPosition));
    assert(FOC_POSITION_GET(&tPosition, 3U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    assert(tFeedback.tElectricalAngle.wBam32 == 0U);
    motor_position_InvalidateZero(&tPosition);
    assert(!motor_position_ZeroValid(&tPosition));
    assert(FOC_POSITION_GET(&tPosition, 4U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    assert(tFeedback.tElectricalAngle.wBam32 == 0xE0000000U);

    tCfg.eSource = MOTOR_POSITION_SOURCE_HARD_DRAG;
    tCfg.tSensor = (motor_position_provider_t){0};
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    assert(FOC_POSITION_GET(&tPosition, 5U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    assert(tFeedback.tElectricalAngle.wBam32 == 0x40000000U);
    assert(motor_position_CaptureZero(&tPosition, 5U) ==
           FOC_RESULT_DISABLED);
    tSample.tHardDragCandidate.bValid = false;
    assert(FOC_POSITION_GET(&tPosition, 6U, &tSample, &tFeedback) ==
           FOC_RESULT_SAFETY);
    assert(!tFeedback.bValid);
    return 0;
}
