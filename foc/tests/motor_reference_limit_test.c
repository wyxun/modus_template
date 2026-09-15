/****************************************************************************
 * @file    motor_reference_limit_test.c
 * @brief   Host test for current/voltage DQ vector-magnitude limits.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor.h"

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
    ptPosition->qMechanicalSpeed = FOC_ZERO;
    ptPosition->bValid = true;
    return FOC_RESULT_OK;
}

foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput)
{
    (void)ptState;
    (void)ptCommand;
    (void)ptInput;
    return FOC_RESULT_OK;
}

void foc_core_Reset(foc_core_state_t *ptState)
{
    (void)ptState;
}

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

void foc_adc_CalibBegin(foc_adc_calib_t *ptCalibration)
{
    ptCalibration->bIsCalibrated = true;
}

foc_calibration_state_e foc_adc_CalibStep(foc_adc_calib_t *ptCalibration)
{
    (void)ptCalibration;
    return FOC_CALIBRATION_COMPLETE;
}

foc_result_t foc_adc_Sample(const foc_adc_calib_t *ptCalibration,
                            foc_current_abc_t *ptCurrent)
{
    (void)ptCalibration;
    *ptCurrent = (foc_current_abc_t){FOC_ZERO, FOC_ZERO, FOC_ZERO};
    return FOC_RESULT_OK;
}

foc_result_t foc_pwm_SetDuty(const foc_duty_abc_t *ptDuty)
{
    (void)ptDuty;
    return FOC_RESULT_OK;
}

foc_result_t foc_pwm_Enable(void)
{
    return FOC_RESULT_OK;
}

void foc_pwm_Stop(void)
{
}

bool foc_pwm_GetFaultStatus(void)
{
    return false;
}

foc_result_t foc_pwm_ClearFaultStatus(void)
{
    return FOC_RESULT_OK;
}

/**
 * @brief Verify current/voltage reference vector-magnitude rejection.
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
    tConfig.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tConfig.tLimits.qMaxModulation = FOC_SCALAR(0.5773502692f);
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

    /* Current mode: vector magnitude limited to qMaxPhaseCurrent (1.0). */
    eResult = motor_Start(&tMotor, FOC_MODE_CURRENT);
    assert(eResult == FOC_RESULT_OK);
    eResult = motor_SetCurrentReference(&tMotor,
                                        FOC_SCALAR(0.5f), FOC_SCALAR(0.5f));
    assert(eResult == FOC_RESULT_OK);
    eResult = motor_SetCurrentReference(&tMotor,
                                        FOC_SCALAR(1.0f), FOC_SCALAR(1.0f));
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
#if defined(FOC_NUMERIC_FLOAT)
    eResult = motor_SetCurrentReference(&tMotor, (foc_scalar_t)NAN, FOC_ZERO);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
    eResult = motor_SetCurrentReference(&tMotor, (foc_scalar_t)INFINITY,
                                       FOC_ZERO);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
#endif
    motor_Stop(&tMotor);

    /* Voltage mode: vector magnitude limited to qMaxModulation (0.577). */
    eResult = motor_Start(&tMotor, FOC_MODE_VOLTAGE);
    assert(eResult == FOC_RESULT_OK);
    eResult = motor_SetVoltageReference(&tMotor,
                                        FOC_SCALAR(0.4f), FOC_SCALAR(0.4f));
    assert(eResult == FOC_RESULT_OK);
    eResult = motor_SetVoltageReference(&tMotor,
                                        FOC_SCALAR(0.5f), FOC_SCALAR(0.5f));
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
#if defined(FOC_NUMERIC_FLOAT)
    eResult = motor_SetVoltageReference(&tMotor, (foc_scalar_t)NAN, FOC_ZERO);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
    eResult = motor_SetVoltageReference(&tMotor, (foc_scalar_t)INFINITY,
                                       FOC_ZERO);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
#endif
    motor_Stop(&tMotor);

    printf("Motor reference limit tests passed!\n");
    return 0;
}
