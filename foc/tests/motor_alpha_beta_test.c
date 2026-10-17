/****************************************************************************
 * @file    motor_alpha_beta_test.c
 * @brief   Host test for the single Clarke conversion in Motor.
 * @author  Codex
 * @date    2026-09-13
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "motor.h"
#include "motor_position.h"
#include "foc_smo.h"

static foc_core_input_t s_tLastCoreInput = {0};
static foc_current_abc_t s_tSample = {0};
static foc_current_sample_t s_tLastRawSample = {0};
static uint32_t s_wClarkeCalls = 0U;
static uint32_t s_wCoreCalls = 0U;
static bool s_bPwmEnabled = false;
static bool s_bPositionValid = true;
static uint32_t s_wDcBusRaw = 8000U;

/**
 * @brief Compare a backend scalar with a floating-point expectation.
 * @param qActual Actual scalar value.
 * @param fExpected Expected value.
 * @return None.
 */
static void test_AssertNear(foc_scalar_t qActual, float fExpected)
{
    assert(fabsf(foc_to_float(qActual) - fExpected) < 0.001f);
}

/**
 * @brief Supply one deterministic valid mechanical position.
 * @param pContext Unused encoder context.
 * @param wNowTick Unused current tick.
 * @param ptPosition Output position.
 * @return FOC_RESULT_OK.
 */
static foc_result_t test_GetPosition(const void *pContext,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    (void)pContext;
    (void)wNowTick;
    ptPosition->tMechanicalAngle = (foc_angle_t){0x10000000U};
    ptPosition->qMechanicalSpeed = FOC_SCALAR(0.25f);
    ptPosition->bValid = s_bPositionValid;
    return FOC_RESULT_OK;
}

/**
 * @brief Capture the current passed from Motor into Core.
 * @param ptState Core state.
 * @param ptCommand Core command.
 * @param ptInput Core input.
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
    (void)ptCommand;
    s_tLastCoreInput = *ptInput;
    ptOutput->tVoltageAlphaBeta = (foc_ab_t){
        FOC_SCALAR(0.12f), FOC_SCALAR(-0.08f)};
    ptOutput->tDuty = (foc_duty_abc_t){
        FOC_SCALAR(0.59f), FOC_SCALAR(0.38f), FOC_SCALAR(0.53f)};
    s_wCoreCalls++;
    return FOC_RESULT_OK;
}

/**
 * @brief Clear test Core history.
 * @param ptState Core state.
 * @return None.
 */
void foc_core_Reset(foc_core_state_t *ptState, foc_core_output_t *ptOutput)
{
    (void)ptState;
    ptOutput->tDuty = (foc_duty_abc_t){FOC_HALF, FOC_HALF, FOC_HALF};
}

/**
 * @brief Capture and convert one three-phase sample.
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
    const float fInvSqrtThree = 0.5773502692f;

    s_wClarkeCalls++;
    ptAB->qAlpha = qIu;
    ptAB->qBeta = foc_from_float(
        (foc_to_float(qIv) - foc_to_float(qIw)) * fInvSqrtThree);
    return FOC_RESULT_OK;
}

/**
 * @brief Begin test calibration in its completed safe state.
 * @param ptCalibration Calibration object.
 * @return None.
 */
static bool s_bCalibrationSample = true;

foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    int32_t nRawU = 2048;
    int32_t nRawV = 2048;
    int32_t nRawW = 2048;

    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }
    if (s_bCalibrationSample) {
        *ptSample = (foc_current_sample_t){2048U, 2048U, 2048U};
        s_tLastRawSample = *ptSample;
        s_bCalibrationSample = false;
        return FOC_RESULT_OK;
    }
    nRawU -= (int32_t)(foc_to_float(s_tSample.qU) *
                       (float)FOC_CURRENT_COUNTS_PER_BASE);
    nRawV -= (int32_t)(foc_to_float(s_tSample.qV) *
                       (float)FOC_CURRENT_COUNTS_PER_BASE);
    nRawW -= (int32_t)(foc_to_float(s_tSample.qW) *
                       (float)FOC_CURRENT_COUNTS_PER_BASE);
    ptSample->wU = (uint32_t)nRawU;
    ptSample->wV = (uint32_t)nRawV;
    ptSample->wW = (uint32_t)nRawW;
    s_tLastRawSample = *ptSample;
    return FOC_RESULT_OK;
}

foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    return (ptDuty == NULL) ? FOC_RESULT_NULL : FOC_RESULT_OK;
}

uint32_t foc_SampleDcBusRaw(foc_port_adc_channel_e eChannel)
{
    assert(eChannel == FOC_PORT_ADC_CHANNEL_DCBUS);
    return s_wDcBusRaw;
}

/**
 * @brief Accept the test ADC trigger start request.
 * @return None.
 */
void foc_port_StartAdcTrigger(void)
{
}

foc_result_t foc_PwmEnable(void)
{
    s_bPwmEnabled = true;
    return FOC_RESULT_OK;
}

foc_result_t foc_PwmSafeStop(void)
{
    s_bPwmEnabled = false;
    return FOC_RESULT_OK;
}

bool foc_PwmGetFault(void)
{
    return false;
}

foc_result_t foc_PwmClearFault(void)
{
    return FOC_RESULT_OK;
}

static uint8_t s_chPositionContext = 0U;

/**
 * @brief Verify Run and ALIGN pass one identical Clarke result to Core.
 * @param None.
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
    const foc_smo_cfg_t tObserverConfig = {
        .wSampleFrequencyHz = 20000U,
        .wBemfCutoffRadiansPerSecond = 10000U,
        .wSlidingGainMillivolt = 3500U,
        .qCurrentEstimateLimit = FOC_ONE,
    };
    foc_result_t eResult = FOC_RESULT_OK;
    foc_scalar_t qBusPu = FOC_ZERO;

    tConfig.wControlFrequencyHz = 20000U;
    tConfig.tParams.chPolePairs = 7U;
    tConfig.tParams.wResistanceMilliohm = 500U;
    tConfig.tParams.wInductanceDMicroHenry = 1000U;
    tConfig.tParams.wInductanceQMicroHenry = 1000U;
    tConfig.tParams.wVoltageBaseMillivolt = 12000U;
    tConfig.tParams.wCurrentBaseMilliamp = 7000U;
    tConfig.qElectricalSpeedBaseHz = FOC_SCALAR(100.0f);
    tConfig.tLimits.qMaxSpeedReference = FOC_ONE;
    tConfig.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tConfig.tLimits.qMaxModulation = FOC_SCALAR(0.5773502692f);
    tPositionCfg.tSensor.fnGetPosition = test_GetPosition;
    tPositionCfg.tSensor.pContext = &s_chPositionContext;
    tPositionCfg.chPolePairs = 7U;
    tPositionCfg.qElectricalSpeedBaseHz = FOC_SCALAR(100.0f);
    tPositionCfg.tSmoCfg = tObserverConfig;
    tConfig.tCurrentPiParams.qOutputMinimum = FOC_NEG_ONE;
    tConfig.tCurrentPiParams.qOutputMaximum = FOC_ONE;
    tConfig.tCurrentPiParams.qIntegratorMinimum = FOC_NEG_ONE;
    tConfig.tCurrentPiParams.qIntegratorMaximum = FOC_ONE;
    tConfig.tSpeedPiParams = tConfig.tCurrentPiParams;
    tConfig.fAdcCalibrationTimeoutSeconds = 0.0005f;
    tConfig.fAlignTimeSeconds = 0.00005f;
    tConfig.wSpeedLoopFrequencyHz = 20000U;
    tConfig.qAlignCurrent = FOC_SCALAR(0.1f);
    s_tSample = (foc_current_abc_t){
        FOC_SCALAR(0.2f), FOC_SCALAR(0.1f), FOC_SCALAR(-0.3f)};

    eResult = motor_Init(&tMotor, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    assert(motor_SampleDcBusPuIsr(&tMotor, &qBusPu) == FOC_RESULT_OK);
    test_AssertNear(qBusPu, 1.0f);
    s_wDcBusRaw = FOC_PORT_ADC_SAMPLE_INVALID;
    assert(motor_SampleDcBusPuIsr(&tMotor, &qBusPu) ==
           FOC_RESULT_SAFETY);
    test_AssertNear(qBusPu, 0.0f);
    s_wDcBusRaw = 8000U;
    tMotor.tParams.wVoltageBaseMillivolt = 0U;
    assert(motor_SampleDcBusPuIsr(&tMotor, &qBusPu) ==
           FOC_RESULT_INVALID_ARGUMENT);
    tMotor.tParams.wVoltageBaseMillivolt = 12000U;
    s_wDcBusRaw = 50000U;
    assert(motor_SampleDcBusPuIsr(&tMotor, &qBusPu) ==
           FOC_RESULT_OUT_OF_RANGE);
    test_AssertNear(qBusPu, 0.0f);
    s_wDcBusRaw = 8000U;
    tPositionCfg.ptMotorParams = &tMotor.tParams;
    assert(motor_position_Init(&tPosition, &tPositionCfg) == FOC_RESULT_OK);
    tPosition.bElectricalZeroValid = true; /* Fixture zero is BAM32=0. */
    assert(tMotor.tParams.wCurrentBaseMilliamp == FOC_CURRENT_BASE_MILLIAMP);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_NO_CONTROL);
    eResult = motor_Start(&tMotor, FOC_MODE_VOLTAGE);
    assert(eResult == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(FOC_POSITION_GET(&tPosition, 1U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    motor_IsrControlStep(&tMotor, &tFeedback);
    motor_position_ObserverStep(&tPosition, &tSample);

    assert(s_wClarkeCalls == 1U);
    assert(s_wCoreCalls == 1U);
    assert(tPosition.tSmo.tAxis[0].z_prev <
           FOC_ZERO);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qAlpha, 0.2f);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qBeta,
                    0.4f * 0.5773502692f);
    tMotor.tOutput.bPwmSaturated = true;
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    assert(tSample.bControlLimited);
    tMotor.tOutput.bPwmSaturated = false;
    assert(FOC_POSITION_GET(&tPosition, 2U, &tSample, &tFeedback) ==
           FOC_RESULT_OK);
    motor_IsrControlStep(&tMotor, &tFeedback);
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(s_wClarkeCalls == 2U);
    assert(s_wCoreCalls == 2U);
    assert(foc_to_float(tPosition.tSmo.tAxis[0].i_hat) >
           0.006f);

    motor_Stop(&tMotor);
    motor_position_ResetObserver(&tPosition);
    assert(tPosition.tSmo.tAxis[0].z_prev ==
           FOC_ZERO);
    eResult = motor_RequestAlignment(&tMotor, true);
    assert(eResult == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CAPTURE_ZERO);
    assert(motor_position_CaptureZero(&tPosition, 3U) == FOC_RESULT_OK);
    motor_CompleteAlignIsr(&tMotor, FOC_RESULT_OK);

    assert(s_wClarkeCalls == 3U);
    assert(s_wCoreCalls == 3U);
    assert(s_tLastCoreInput.tElectricalAngle.wBam32 == 0U);
    assert(tPosition.tSmo.tAxis[0].z_prev ==
           FOC_ZERO);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qAlpha, 0.2f);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qBeta,
                    0.4f * 0.5773502692f);
    assert(!s_bPwmEnabled);
    assert(tMotor.bElectricalZeroValid);

    /* A stationary identification session never needs position feedback. */
    tMotor.bElectricalZeroValid = false;
    assert(motor_StartIdentification(&tMotor) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&tMotor, &tSample) == MOTOR_ISR_NO_CONTROL);
    assert(tMotor.eState == MOTOR_STATE_RUNNING && s_bPwmEnabled);
    assert(!tMotor.bElectricalZeroValid);
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_IDENTIFICATION_READY);
    assert(motor_IsrControlStep(&tMotor, NULL) == FOC_RESULT_OK);
    assert(s_tLastCoreInput.tElectricalAngle.wBam32 == 0U);
    assert(s_tLastCoreInput.qElectricalSpeedPu == FOC_ZERO);
    foc_ab_t injection = {FOC_SCALAR(0.1f), FOC_ZERO};
    assert(motor_SetIdentificationModulation(&tMotor, &injection) ==
           FOC_RESULT_OK);
    test_AssertNear(tMotor.tCommand.tVoltageReference.qD, 0.1f);
    assert(motor_SetCurrentReference(&tMotor, FOC_ZERO, FOC_ZERO) ==
           FOC_RESULT_BUSY);
    assert(motor_SetVoltageReference(&tMotor, FOC_ZERO, FOC_ZERO) ==
           FOC_RESULT_BUSY);
    assert(motor_StartIdentification(&tMotor) == FOC_RESULT_SAFETY);
    assert(tMotor.bIdentificationActive && s_bPwmEnabled);
    injection.qBeta = FOC_SCALAR(0.01f);
    assert(motor_IdentificationApplyIsr(&tMotor, &injection) ==
           FOC_RESULT_OUT_OF_RANGE);
    motor_Stop(&tMotor);
    motor_status_t status = {0};
    assert(motor_GetStatus(&tMotor, &status) == FOC_RESULT_OK);
    assert(!status.bIdentificationActive && !status.bPwmEnabled);
    assert(motor_IdentificationApplyIsr(&tMotor, &injection) != FOC_RESULT_OK);
    assert(motor_StartIdentification(&tMotor) == FOC_RESULT_OK);
    motor_IdentificationAbortIsr(&tMotor, MOTOR_FAULT_IDENTIFICATION);
    assert(motor_GetStatus(&tMotor, &status) == FOC_RESULT_OK);
    assert(status.eState == MOTOR_STATE_FAULT);
    assert(!status.bIdentificationActive && !status.bPwmEnabled);
    assert(motor_ClearFault(&tMotor) == FOC_RESULT_OK);

    eResult = motor_Start(&tMotor, FOC_MODE_CURRENT);
    assert(eResult == FOC_RESULT_OK);
    s_bPositionValid = false;
    assert(motor_IsrPrepare(&tMotor, &tSample) ==
           MOTOR_ISR_CONTROL_READY);
    eResult = FOC_POSITION_GET(&tPosition, 4U, &tSample, &tFeedback);
    assert(eResult != FOC_RESULT_OK);
    motor_IsrControlStep(&tMotor, NULL);
    assert(tMotor.eState == MOTOR_STATE_FAULT);
    assert((tMotor.wFaults & MOTOR_FAULT_POSITION) != 0U);
    assert(!s_bPwmEnabled);
    return 0;
}
