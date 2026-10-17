/****************************************************************************
 * @file    motor_speed_pu_test.c
 * @brief   Host test for mechanical-to-electrical speed PU conversion.
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "motor.h"
#include "motor_position.h"

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
foc_result_t foc_core_Init(foc_core_state_t *ptState,
    const foc_pid_params_t *ptPi, foc_scalar_t qVoltageLimit)
{
    foc_result_t eResult = foc_pid_Init(&ptState->tIdPi, ptPi);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptState->tIqPi, ptPi);
    }
    ptState->qVoltageLimitSquared = foc_mul_pu(qVoltageLimit, qVoltageLimit);
    return eResult;
}

foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput,
                           foc_core_output_t *ptOutput)
{
    (void)ptState;
    (void)ptOutput;
    s_tLastInput = *ptInput;
    s_tLastCommand = *ptCommand;
    return FOC_RESULT_OK;
}

/**
 * @brief Reset the test Core state.
 * @param ptState Core state.
 * @return None.
 */
void foc_core_Reset(foc_core_state_t *ptState, foc_core_output_t *ptOutput)
{
    (void)ptState;
    (void)ptOutput;
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
    tConfig.qElectricalSpeedBaseHz = FOC_SCALAR(100.0f);
    tConfig.nHardDragElectricalMilliHz = 5000;
    tConfig.wControlFrequencyHz = 20000U;
    tConfig.tLimits.qMaxSpeedReference = FOC_ONE;
    tConfig.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tConfig.tLimits.qMaxModulation = FOC_SCALAR(0.5773502692f);
    tPositionCfg.tSensor.fnGetPosition = test_GetPosition;
    tPositionCfg.tSensor.pContext = &s_chPositionContext;
    tPositionCfg.chPolePairs = 7U;
    tPositionCfg.qElectricalSpeedBaseHz = FOC_SCALAR(100.0f);
    tPositionCfg.tSmoCfg.wSampleFrequencyHz = 20000U;
    tPositionCfg.tSmoCfg.wBemfCutoffRadiansPerSecond = 10000U;
    tPositionCfg.tSmoCfg.wSlidingGainMillivolt = 3500U;
    tPositionCfg.tSmoCfg.qCurrentEstimateLimit = FOC_ONE;
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
    tPosition.bElectricalZeroValid = true; /* Fixture zero is BAM32=0. */
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
           tMotor.tStart.wHardDragStepBam32);
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
           tMotor.tStart.wHardDragStepBam32);
    tFeedback.eEvent = MOTOR_POSITION_EVENT_OBSERVER_ACTIVE;
    tFeedback.qElectricalSpeedPu = FOC_SCALAR(-0.05f);
    assert(motor_IsrControlStep(&tMotor, &tFeedback) == FOC_RESULT_OK);
    tFeedback.eEvent = MOTOR_POSITION_EVENT_NONE;
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(!tSample.tHardDragCandidate.bValid);
    motor_IsrControlStep(&tMotor, &tFeedback);
    motor_Stop(&tMotor);

    tConfig.nHardDragElectricalMilliHz = 0;
    tConfig.wStartupRampSteps = 20U;
    tConfig.wStartupMaxRunSteps = 40U;
    tConfig.qStartupIqPu = FOC_SCALAR(0.02f);
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_NO_CONTROL);
    assert(motor_StartSensorlessSpeed(&tMotor,
        FOC_SCALAR(0.2f)) == FOC_RESULT_OK);
    assert(tMotor.eState == MOTOR_STATE_ALIGN);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_NO_CONTROL);
    assert(tMotor.eState == MOTOR_STATE_RUNNING);
    assert(tMotor.bPwmEnabled);
    /* The forced trajectory now denotes the rotating field/d axis. */
    test_AssertNear(tMotor.tCommand.tCurrentReference.qD, 0.02f);
    test_AssertNear(tMotor.tCommand.tCurrentReference.qQ, 0.0f);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(tSample.tHardDragCandidate.bValid);
    assert(tSample.tHardDragCandidate.tElectricalAngle.wBam32 == 0U);
    tFeedback = (motor_electrical_feedback_t){
        .tElectricalAngle = {0U},
        .qElectricalSpeedPu = FOC_SCALAR(0.2f),
        .bValid = true,
        .eEvent = MOTOR_POSITION_EVENT_OBSERVER_ACTIVE,
    };
    assert(motor_IsrControlStep(&tMotor, &tFeedback) == FOC_RESULT_OK);
    assert(tMotor.tCommand.eMode == FOC_MODE_SPEED);
    assert(!tMotor.tStart.tTrajectory.bActive);
    tFeedback.eEvent = MOTOR_POSITION_EVENT_NONE;
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(!tSample.tHardDragCandidate.bValid);
    motor_IsrControlStep(&tMotor, &tFeedback);
    test_AssertNear(s_tLastCommand.tCurrentReference.qQ, 0.0f);
    assert(s_tLastCommand.tCurrentReference.qD > FOC_ZERO);
    assert(s_tLastCommand.tCurrentReference.qD < FOC_SCALAR(0.02f));
    /* D-field exits monotonically; q starts from its previous command. */
    for (uint32_t i = 0U; i < 60U; i++) {
        foc_scalar_t qPrevious = tMotor.tCommand.tCurrentReference.qD;
        assert(motor_IsrPrepare(&tMotor, &tSample) ==
               MOTOR_ISR_CONTROL_READY);
        assert(motor_IsrControlStep(&tMotor, &tFeedback) == FOC_RESULT_OK);
        assert(tMotor.tCommand.tCurrentReference.qD <= qPrevious);
        assert(tMotor.tCommand.tCurrentReference.qD >= FOC_ZERO);
        assert(foc_to_float(qPrevious -
            tMotor.tCommand.tCurrentReference.qD) <= 0.00051f);
        test_AssertNear(tMotor.tCommand.tCurrentReference.qQ, 0.0f);
    }
    assert(tMotor.tCommand.tCurrentReference.qD == FOC_ZERO);
    /* While d remains, a conservative q bound prevents vector overflow.
     * Use a tighter valid phase budget to exercise clipping/retracking. */
    tMotor.tLimits.qMaxPhaseCurrent = FOC_SCALAR(0.03f);
    tMotor.tLimits.qMaxIq = FOC_SCALAR(0.03f);
    tMotor.tCommand.tCurrentReference.qD = FOC_SCALAR(0.02f);
    tMotor.tSpeed.tPi.qIntegrator = FOC_SCALAR(0.03f);
    tMotor.tSpeed.tPi.tParams.qIntegratorMaximum = FOC_SCALAR(0.05f);
    tMotor.tSpeed.tPi.tParams.qOutputMaximum = FOC_SCALAR(0.05f);
    for (uint32_t i = 0U; i < 12U; i++) {
        assert(motor_IsrPrepare(&tMotor, &tSample) ==
               MOTOR_ISR_CONTROL_READY);
        assert(motor_IsrControlStep(&tMotor, &tFeedback) == FOC_RESULT_OK);
        float fD = foc_to_float(tMotor.tCommand.tCurrentReference.qD);
        float fQ = foc_to_float(tMotor.tCommand.tCurrentReference.qQ);
        assert(fabsf(fD) + fabsf(fQ) <= 0.03001f);
    }
    tMotor.tCommand.tCurrentReference.qD = FOC_ZERO;
    tMotor.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tMotor.tLimits.qMaxIq = FOC_ONE;
    foc_pid_Reset(&tMotor.tSpeed.tPi);
    /* A changed target follows the existing closed-loop ramp. */
    assert(motor_SetSpeedReference(&tMotor, FOC_SCALAR(0.3f)) ==
           FOC_RESULT_OK);
    test_AssertNear(tMotor.tCommand.qSpeedReferencePu, 0.2f);
    for (uint32_t i = 0U; i < 120U; i++) {
        assert(motor_IsrPrepare(&tMotor, &tSample) ==
               MOTOR_ISR_CONTROL_READY);
        assert(motor_IsrControlStep(&tMotor, &tFeedback) == FOC_RESULT_OK);
        if (i == 0U) {
            test_AssertNear(tMotor.tCommand.qSpeedReferencePu, 0.201f);
        }
    }
    test_AssertNear(tMotor.tCommand.qSpeedReferencePu, 0.3f);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    tFeedback.eEvent = MOTOR_POSITION_EVENT_OBSERVER_LOST;
    assert(motor_IsrControlStep(&tMotor, &tFeedback) == FOC_RESULT_SAFETY);
    tFeedback.eEvent = MOTOR_POSITION_EVENT_NONE;
    assert(tMotor.eState == MOTOR_STATE_FAULT);
    assert(!tMotor.bPwmEnabled);
    assert(motor_ClearFault(&tMotor) == FOC_RESULT_OK);
    assert(motor_StartSensorlessSpeed(&tMotor,
        FOC_SCALAR(0.2f)) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_NO_CONTROL);
    for (uint32_t wIndex = 0U; wIndex < 40U; wIndex++) {
        assert(motor_IsrPrepare(&tMotor, &tSample) ==
               MOTOR_ISR_CONTROL_READY);
        motor_IsrControlStep(&tMotor,
            &tSample.tHardDragCandidate);
    }
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_NO_CONTROL);
    assert(tMotor.eState == MOTOR_STATE_FAULT);
    assert(!tMotor.bPwmEnabled);
    /* ALIGN completion is explicit even after a failed sensorless run. */
    assert(motor_ClearFault(&tMotor) == FOC_RESULT_OK);
    assert(motor_RequestAlignment(&tMotor, true) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, NULL) == MOTOR_ISR_CAPTURE_ZERO);
    assert(s_tLastInput.bAngleValid);
    assert(s_tLastInput.tElectricalAngle.wBam32 == 0U);
    assert(s_tLastInput.qElectricalSpeedPu == FOC_ZERO);
    motor_CompleteAlignIsr(&tMotor, FOC_RESULT_OK);
    assert(tMotor.eState == MOTOR_STATE_IDLE);
    /* Retrying ADC calibration must retain the configured timeout. */
    const uint32_t wTimeout = tMotor.tCalib.wTimeoutSteps;
    motor_IdentificationAbortIsr(&tMotor, MOTOR_FAULT_ADC_CAL);
    assert(motor_ClearFault(&tMotor) == FOC_RESULT_OK);
    assert(tMotor.eState == MOTOR_STATE_ADC_CAL);
    assert(tMotor.tCalib.wTimeoutSteps == wTimeout);
    assert(tMotor.tCalib.wElapsedSteps == 0U);
    assert(motor_IsrPrepare(&tMotor, NULL) == MOTOR_ISR_NO_CONTROL);
    assert(tMotor.eState == MOTOR_STATE_IDLE);
    /* PI must see the actual actuator range, including tracking at takeover. */
    tConfig.wStartupRampSteps = 0U;
    tConfig.tLimits.qMaxIq = FOC_SCALAR(0.2f);
    assert(foc_gain_from_float(0.0f,
        &tConfig.tSpeedPiParams.tKp) == FOC_RESULT_OK);
    assert(foc_gain_from_float(0.1f,
        &tConfig.tSpeedPiParams.tKiTs) == FOC_RESULT_OK);
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) == MOTOR_ISR_NO_CONTROL);
    assert(motor_Start(&tMotor, FOC_MODE_SPEED) == FOC_RESULT_OK);
    assert(motor_SetSpeedReference(&tMotor, FOC_SCALAR(0.8f)) ==
           FOC_RESULT_OK);
    tFeedback.qElectricalSpeedPu = FOC_ZERO;
    for (uint32_t i = 0U; i < 20U; i++) {
        assert(motor_IsrPrepare(&tMotor, &tSample) ==
               MOTOR_ISR_CONTROL_READY);
        assert(motor_IsrControlStep(&tMotor, &tFeedback) == FOC_RESULT_OK);
    }
    assert(tMotor.tSpeed.tPi.qIntegrator <= FOC_SCALAR(0.2f));
    assert(tMotor.tSpeed.tPi.tParams.qOutputMaximum == FOC_SCALAR(0.2f));
    assert(tMotor.tSpeed.tPi.tParams.qOutputMinimum == FOC_SCALAR(-0.2f));
    /* Preserve a narrower user interval; reject an empty intersection. */
    tConfig.tSpeedPiParams.qOutputMinimum = FOC_SCALAR(-0.1f);
    tConfig.tSpeedPiParams.qOutputMaximum = FOC_SCALAR(0.1f);
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    assert(tMotor.tSpeed.tPi.tParams.qOutputMaximum == FOC_SCALAR(0.1f));
    tConfig.tSpeedPiParams.qOutputMinimum = FOC_SCALAR(0.3f);
    tConfig.tSpeedPiParams.qOutputMaximum = FOC_SCALAR(0.4f);
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_INVALID_ARGUMENT);
    /* Startup tracking must be representable in the user's PI interval. */
    tConfig.wStartupRampSteps = 20U;
    tConfig.tSpeedPiParams.qOutputMinimum = FOC_SCALAR(0.1f);
    tConfig.tSpeedPiParams.qOutputMaximum = FOC_SCALAR(0.2f);
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_INVALID_ARGUMENT);
    tConfig.tSpeedPiParams.qOutputMinimum = FOC_SCALAR(-0.1f);
    tConfig.tSpeedPiParams.qOutputMaximum = FOC_SCALAR(0.1f);
    tConfig.tSpeedPiParams.qIntegratorMinimum = FOC_SCALAR(0.001f);
    tConfig.tSpeedPiParams.qIntegratorMaximum = FOC_SCALAR(0.1f);
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_INVALID_ARGUMENT);
    tConfig.tSpeedPiParams.qIntegratorMinimum = FOC_SCALAR(-0.1f);
#if defined(FOC_NUMERIC_FLOAT)
    tConfig.qStartupIqPu = NAN;
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_INVALID_ARGUMENT);
#endif
    tConfig.qStartupIqPu = FOC_ZERO;
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_INVALID_ARGUMENT);
    tConfig.qStartupIqPu = FOC_SCALAR(-0.02f);
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_INVALID_ARGUMENT);
    return 0;
}
