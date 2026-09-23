/****************************************************************************
 * @file    motor_reference_limit_test.c
 * @brief   Host test for current/voltage DQ vector-magnitude limits.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor.h"

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


/**
 * @brief Verify current/voltage reference vector-magnitude rejection.
 * @return Zero on success.
 */
int main(void)
{
    motor_t tMotor = {0};
    motor_cfg_t tConfig = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tConfig.wControlFrequencyHz = 20000U;
    tConfig.tParams.chPolePairs = 7U;
    tConfig.tParams.wResistanceMilliohm = 500U;
    tConfig.tParams.wInductanceDMicroHenry = 1000U;
    tConfig.tParams.wInductanceQMicroHenry = 1000U;
    tConfig.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
    tConfig.tLimits.qMaxSpeedReference = FOC_ONE;
    tConfig.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tConfig.tLimits.qMaxModulation = FOC_SCALAR(0.5773502692f);
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
    assert(motor_IsrPrepare(&tMotor, NULL) == MOTOR_ISR_NO_CONTROL);

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
