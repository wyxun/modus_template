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
static foc_result_t test_GetPosition(void *pContext,
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

/**
 * @brief Start test ADC calibration as complete.
 * @param ptCalibration Calibration state.
 * @return None.
 */
void foc_adc_CalibBegin(foc_adc_calib_t *ptCalibration)
{
    ptCalibration->bIsCalibrated = true;
}

/**
 * @brief Report test ADC calibration complete.
 * @param ptCalibration Calibration state.
 * @return FOC_CALIBRATION_COMPLETE.
 */
foc_calibration_state_e foc_adc_CalibStep(
    foc_adc_calib_t *ptCalibration)
{
    (void)ptCalibration;
    return FOC_CALIBRATION_COMPLETE;
}

/**
 * @brief Return a zero-current sample.
 * @param ptCalibration Calibration state.
 * @param ptCurrent Sample output.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_adc_Sample(const foc_adc_calib_t *ptCalibration,
                            foc_current_abc_t *ptCurrent)
{
    (void)ptCalibration;
    *ptCurrent = (foc_current_abc_t){FOC_ZERO, FOC_ZERO, FOC_ZERO};
    return FOC_RESULT_OK;
}

/**
 * @brief Accept one safe test PWM duty.
 * @param ptDuty PWM duty.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_pwm_SetDuty(const foc_duty_abc_t *ptDuty)
{
    (void)ptDuty;
    return FOC_RESULT_OK;
}

/**
 * @brief Enable the test PWM.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_pwm_Enable(void)
{
    return FOC_RESULT_OK;
}

/**
 * @brief Disable the test PWM.
 * @return None.
 */
void foc_pwm_Stop(void)
{
}

/**
 * @brief Verify normalized speed feedback and speed-loop response.
 * @return Zero on success.
 */
int main(void)
{
    motor_t tMotor = {0};
    motor_cfg_t tConfig = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tConfig.tParams.chPolePairs = 7U;
    tConfig.tParams.wResistanceMilliohm = 500U;
    tConfig.tParams.wInductanceDMicroHenry = 1000U;
    tConfig.tParams.wInductanceQMicroHenry = 1000U;
    tConfig.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
    tConfig.tLimits.qMaxSpeedReference = FOC_ONE;
    tConfig.fnGetPosition = test_GetPosition;
    tConfig.tControl.chSpeedLoopDiv = 1U;
    tConfig.tControl.wAdcCalibrationTimeoutSteps = 2U;
    tConfig.tControl.wAlignSteps = 1U;
    tConfig.tControl.qAlignCurrent = FOC_SCALAR(0.1f);
    eResult = foc_gain_from_float(1.0f,
        &tConfig.tControl.tSpeedPiParams.tKp);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_gain_from_float(0.0f,
        &tConfig.tControl.tSpeedPiParams.tKiTs);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_gain_from_float(0.0f,
        &tConfig.tControl.tSpeedPiParams.tKdOverTs);
    assert(eResult == FOC_RESULT_OK);
    tConfig.tControl.tSpeedPiParams.qOutputMinimum = FOC_NEG_ONE;
    tConfig.tControl.tSpeedPiParams.qOutputMaximum = FOC_ONE;
    tConfig.tControl.tSpeedPiParams.qIntegratorMinimum = FOC_NEG_ONE;
    tConfig.tControl.tSpeedPiParams.qIntegratorMaximum = FOC_ONE;
    tConfig.tControl.tCurrentPiParams = tConfig.tControl.tSpeedPiParams;

    eResult = motor_Init(&tMotor, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    motor_HighFrequencyStep(&tMotor, 0U);
    eResult = motor_Start(&tMotor, FOC_MODE_SPEED);
    assert(eResult == FOC_RESULT_OK);

    s_qMechanicalSpeed = FOC_SCALAR(10.0f);
    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(0.8f));
    assert(eResult == FOC_RESULT_OK);
    motor_HighFrequencyStep(&tMotor, 1U);
    test_AssertNear(s_tLastInput.qElectricalSpeedPu, 0.7f);
    test_AssertNear(s_tLastCommand.tCurrentReference.qQ, 0.1f);

    s_qMechanicalSpeed = FOC_SCALAR(-10.0f);
    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(-0.8f));
    assert(eResult == FOC_RESULT_OK);
    motor_HighFrequencyStep(&tMotor, 2U);
    test_AssertNear(s_tLastInput.qElectricalSpeedPu, -0.7f);
    test_AssertNear(s_tLastCommand.tCurrentReference.qQ, -0.1f);

    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(1.1f));
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
    motor_Stop(&tMotor);
    return 0;
}
