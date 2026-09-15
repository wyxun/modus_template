/****************************************************************************
 * @file    motor_break_fault_test.c
 * @brief   Host test for power-stage break fault propagation into Motor.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor.h"

static bool s_bBreakFault = false;
static bool s_bClearOk = true;

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
    return s_bBreakFault;
}

foc_result_t foc_pwm_ClearFaultStatus(void)
{
    if (!s_bClearOk) {
        return FOC_RESULT_SAFETY;
    }
    s_bBreakFault = false;
    return FOC_RESULT_OK;
}

/**
 * @brief Verify break fault latches, blocks start, and clears.
 * @return Zero on success.
 */
int main(void)
{
    motor_t tMotor = {0};
    motor_cfg_t tConfig = {0};
    motor_status_t tStatus = {0};
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

    /* 1. Latched break blocks motor_Start. */
    s_bBreakFault = true;
    eResult = motor_Start(&tMotor, FOC_MODE_CURRENT);
    assert(eResult == FOC_RESULT_SAFETY);

    /* 2. Released break allows start. */
    s_bBreakFault = false;
    eResult = motor_Start(&tMotor, FOC_MODE_CURRENT);
    assert(eResult == FOC_RESULT_OK);

    /* 3. Break poll latches MOTOR_FAULT_PWM and enters FAULT. */
    s_bBreakFault = true;
    motor_PollBreakFault(&tMotor);
    eResult = motor_GetStatus(&tMotor, &tStatus);
    assert(eResult == FOC_RESULT_OK);
    assert(tStatus.eState == MOTOR_STATE_FAULT);
    assert((tStatus.wFaults & (uint32_t)MOTOR_FAULT_PWM) != 0U);

    /* 4. ClearFault clears the software fault and the break latch. */
    s_bClearOk = true;
    eResult = motor_ClearFault(&tMotor);
    assert(eResult == FOC_RESULT_OK);
    assert(s_bBreakFault == false);
    eResult = motor_GetStatus(&tMotor, &tStatus);
    assert(eResult == FOC_RESULT_OK);
    assert(tStatus.eState == MOTOR_STATE_IDLE);

    /* 5. ClearFault refuses while the break source is still active. */
    s_bBreakFault = false;
    eResult = motor_Start(&tMotor, FOC_MODE_CURRENT);
    assert(eResult == FOC_RESULT_OK);
    s_bBreakFault = true;
    motor_PollBreakFault(&tMotor);
    s_bClearOk = false;
    eResult = motor_ClearFault(&tMotor);
    assert(eResult == FOC_RESULT_SAFETY);
    eResult = motor_GetStatus(&tMotor, &tStatus);
    assert(eResult == FOC_RESULT_OK);
    assert(tStatus.eState == MOTOR_STATE_FAULT);

    printf("Motor break fault tests passed!\n");
    return 0;
}
