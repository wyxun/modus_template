/****************************************************************************
 * @file    motor_speed_pu_test.c
 * @brief   Host test for mechanical-to-electrical speed PU conversion.
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "motor.h"

static foc_core_input_t s_tLastInput = {0};
static foc_core_command_t s_tLastCommand = {0};
static foc_scalar_t s_qMechanicalSpeed = FOC_ZERO;

/**
 * @brief Check a scalar value against a floating-point expectation.
 * @param qActual Actual backend value.
 * @param fExpected Expected value.
 * @return None.
 */
static void test_AssertNear(foc_scalar_t qActual, float fExpected)
{
    assert(fabsf(foc_to_float(qActual) - fExpected) < 0.001f);
}

/**
 * @brief Supply one valid mechanical position.
 * @param pContext Unused callback context.
 * @param wNowTick Unused timestamp.
 * @param ptPosition Position output.
 * @return FOC_RESULT_OK.
 */
static foc_result_t test_GetPosition(const void *pContext,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    (void)pContext;
    (void)wNowTick;
    ptPosition->tMechanicalAngle = (foc_angle_t){0U};
    ptPosition->qMechanicalSpeed = s_qMechanicalSpeed;
    ptPosition->bValid = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Capture the Motor input passed to Core.
 * @param ptState Unused Core state.
 * @param ptCommand Motor command.
 * @param ptInput Motor input.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput)
{
    (void)ptState;
    s_tLastInput = *ptInput;
    s_tLastCommand = *ptCommand;
    return FOC_RESULT_OK;
}

/**
 * @brief Reset the test Core state.
 * @param ptState Core state.
 * @return None.
 */
void foc_core_Reset(foc_core_state_t *ptState)
{
    (void)ptState;
}

/**
 * @brief Accept the test current transform.
 * @param qIu U-phase current.
 * @param qIv V-phase current.
 * @param qIw W-phase current.
 * @param ptAB Output alpha-beta current.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB)
{
    (void)qIu;
    (void)qIv;
    (void)qIw;
    *ptAB = (foc_ab_t){FOC_ZERO, FOC_ZERO};
    return FOC_RESULT_OK;
}

static uint8_t s_chPositionContext = 0U;

/**
 * @brief Verify normalized speed feedback and speed-loop response.
 * @return Zero on success.
 */
int main(void)
{
    motor_t tMotor = {0};
    motor_position_t tPosition = {0};
    motor_cfg_t tConfig = {0};
    motor_position_cfg_t tPositionCfg = {0};
    motor_position_sample_t tSample = {0};
    motor_electrical_feedback_t tFeedback = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tConfig.tParams.chPolePairs = 7U;
    tConfig.tParams.wResistanceMilliohm = 500U;
    tConfig.tParams.wInductanceDMicroHenry = 1000U;
    tConfig.tParams.wInductanceQMicroHenry = 1000U;
    tConfig.tParams.wVoltageBaseMillivolt = 12000U;
    tConfig.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
    tConfig.nHardDragElectricalMilliHz = 5000;
    tConfig.wControlFrequencyHz = 20000U;
    tConfig.tLimits.qMaxSpeedReference = FOC_ONE;
    tConfig.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tConfig.tLimits.qMaxModulation = FOC_SCALAR(0.5773502692f);
    tPositionCfg.tSensor.fnGetPosition = test_GetPosition;
    tPositionCfg.tSensor.pContext = &s_chPositionContext;
    tPositionCfg.chPolePairs = 7U;
    tPositionCfg.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
    tPositionCfg.tObserverCfg.tSmo.wSampleFrequencyHz = 20000U;
    tPositionCfg.tObserverCfg.tSmo.wBemfCutoffRadiansPerSecond = 10000U;
    tPositionCfg.tObserverCfg.tSmo.wSlidingGainMillivolt = 3500U;
    tPositionCfg.tObserverCfg.tSmo.qCurrentEstimateLimit = FOC_ONE;
    tConfig.wSpeedLoopFrequencyHz = 20000U;
    tConfig.fAdcCalibrationTimeoutSeconds = 0.0001f;
    tConfig.fAlignTimeSeconds = 0.00005f;
    tConfig.qAlignCurrent = FOC_SCALAR(0.1f);
    eResult = foc_gain_from_float(1.0f,
        &tConfig.tSpeedPiParams.tKp);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_gain_from_float(0.0f,
        &tConfig.tSpeedPiParams.tKiTs);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_gain_from_float(0.0f,
        &tConfig.tSpeedPiParams.tKdOverTs);
    assert(eResult == FOC_RESULT_OK);
    tConfig.tSpeedPiParams.qOutputMinimum = FOC_NEG_ONE;
    tConfig.tSpeedPiParams.qOutputMaximum = FOC_ONE;
    tConfig.tSpeedPiParams.qIntegratorMinimum = FOC_NEG_ONE;
    tConfig.tSpeedPiParams.qIntegratorMaximum = FOC_ONE;
    tConfig.tCurrentPiParams = tConfig.tSpeedPiParams;

    eResult = motor_Init(&tMotor, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    tPositionCfg.ptMotorParams = &tMotor.tParams;
    assert(motor_position_Init(&tPosition, &tPositionCfg) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_NO_CONTROL);
    eResult = motor_Start(&tMotor, FOC_MODE_SPEED);
    assert(eResult == FOC_RESULT_OK);

    s_qMechanicalSpeed = FOC_SCALAR(10.0f);
    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(0.8f));
    assert(eResult == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(tSample.tHardDragCandidate.bValid);
    assert(tSample.tHardDragCandidate.tElectricalAngle.wBam32 == 0U);
    test_AssertNear(tSample.tHardDragCandidate.qElectricalSpeedPu, 0.05f);
    assert(FOC_POSITION_GET(&tPosition, 1U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    motor_IsrControlStep(&tMotor, &tFeedback);
    test_AssertNear(s_tLastInput.qElectricalSpeedPu, 0.7f);
    test_AssertNear(s_tLastCommand.tCurrentReference.qQ, 0.1f);

    s_qMechanicalSpeed = FOC_SCALAR(-10.0f);
    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(-0.8f));
    assert(eResult == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(tSample.tHardDragCandidate.tElectricalAngle.wBam32 ==
           tMotor.wHardDragAngleStepBam32);
    assert(FOC_POSITION_GET(&tPosition, 2U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    motor_IsrControlStep(&tMotor, &tFeedback);
    test_AssertNear(s_tLastInput.qElectricalSpeedPu, -0.7f);
    test_AssertNear(s_tLastCommand.tCurrentReference.qQ, -0.1f);

    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(1.1f));
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
#if defined(FOC_NUMERIC_FLOAT)
    eResult = motor_SetSpeedReference(&tMotor, (foc_scalar_t)NAN);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
#endif
    motor_Stop(&tMotor);
    assert(motor_Start(&tMotor, FOC_MODE_CURRENT) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(tSample.tHardDragCandidate.tElectricalAngle.wBam32 == 0U);
    motor_IsrControlStep(&tMotor, &tFeedback);
    motor_Stop(&tMotor);

    tConfig.nHardDragElectricalMilliHz = -5000;
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_NO_CONTROL);
    assert(motor_Start(&tMotor, FOC_MODE_CURRENT) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    test_AssertNear(tSample.tHardDragCandidate.qElectricalSpeedPu, -0.05f);
    motor_IsrControlStep(&tMotor, &tFeedback);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(tSample.tHardDragCandidate.tElectricalAngle.wBam32 ==
           tMotor.wHardDragAngleStepBam32);
    motor_IsrControlStep(&tMotor, &tFeedback);
    motor_Stop(&tMotor);
    return 0;
}
