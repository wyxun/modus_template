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
static foc_result_t test_GetPosition(const void *pContext,
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

static foc_result_t test_AdcCalibrationBegin(
    void *pContext,
    foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    ptCalibration->bIsCalibrated = true;
    return FOC_RESULT_OK;
}

static foc_calibration_state_e test_AdcCalibrationStep(
    void *pContext,
    foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    (void)ptCalibration;
    return FOC_CALIBRATION_COMPLETE;
}

static foc_result_t test_AdcSample(
    void *pContext,
    const foc_adc_calib_t *ptCalibration,
    foc_current_abc_t *ptCurrent)
{
    (void)pContext;
    (void)ptCalibration;
    *ptCurrent = (foc_current_abc_t){FOC_ZERO, FOC_ZERO, FOC_ZERO};
    return FOC_RESULT_OK;
}

static foc_result_t test_PwmSetDuty(void *pContext,
                                    const foc_duty_abc_t *ptDuty)
{
    (void)ptDuty;
    (void)pContext;
    return FOC_RESULT_OK;
}

static foc_result_t test_PwmEnable(void *pContext)
{
    (void)pContext;
    return FOC_RESULT_OK;
}

static foc_result_t test_PwmStop(void *pContext)
{
    (void)pContext;
    return FOC_RESULT_OK;
}

static bool test_PwmGetFault(void *pContext)
{
    (void)pContext;
    return false;
}

static foc_result_t test_PwmClearFault(void *pContext)
{
    (void)pContext;
    return FOC_RESULT_OK;
}

static foc_result_t test_AdcSetCurrentBase(void *pContext,
                                           uint32_t wCurrentBaseMilliamp)
{
    (void)pContext;
    (void)wCurrentBaseMilliamp;
    return FOC_RESULT_OK;
}

static const foc_adc_ops_t s_tAdcOps = {
    .fnSetCurrentBase = test_AdcSetCurrentBase,
    .fnCalibrationBegin = test_AdcCalibrationBegin,
    .fnCalibrationStep = test_AdcCalibrationStep,
    .fnSample = test_AdcSample,
};

static const foc_pwm_ops_t s_tPwmOps = {
    .fnSetDuty = test_PwmSetDuty,
    .fnEnable = test_PwmEnable,
    .fnStop = test_PwmStop,
    .fnGetFaultStatus = test_PwmGetFault,
    .fnClearFaultStatus = test_PwmClearFault,
};

static const motor_position_ops_t s_tPositionOps = {
    .fnGetPosition = test_GetPosition,
    .fnCaptureZero = test_GetPosition,
};

static uint8_t s_chPositionContext = 0U;

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
    tConfig.tAdc.ptOps = &s_tAdcOps;
    tConfig.tAdc.pContext = &s_chPositionContext;
    tConfig.tPwm.ptOps = &s_tPwmOps;
    tConfig.tPwm.pContext = &s_chPositionContext;
    tConfig.tPosition.ptOps = &s_tPositionOps;
    tConfig.tPosition.pContext = &s_chPositionContext;
    tConfig.chSpeedLoopDiv = 1U;
    tConfig.wAdcCalibrationTimeoutSteps = 2U;
    tConfig.wAlignSteps = 1U;
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
    motor_IsrStep(&tMotor, 0U);

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
