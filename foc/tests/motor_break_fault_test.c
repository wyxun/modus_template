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
static bool s_bStopOk = true;
static motor_t *s_ptTriggerFaultMotor = NULL;

void foc_port_StartAdcTrigger(void)
{
    if (s_ptTriggerFaultMotor != NULL) {
        /* Inject a synchronous ISR fault at the trigger-enable boundary. */
        motor_IdentificationAbortIsr(s_ptTriggerFaultMotor, MOTOR_FAULT_PWM);
    }
}

foc_result_t foc_PwmSafeStop(void)
{
    return s_bStopOk ? FOC_RESULT_OK : FOC_RESULT_SAFETY;
}

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
    (void)ptCommand;
    (void)ptInput;
    return FOC_RESULT_OK;
}

void foc_core_Reset(foc_core_state_t *ptState, foc_core_output_t *ptOutput)
{
    (void)ptState;
    (void)ptOutput;
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

bool foc_PwmGetFault(void)
{
    return s_bBreakFault;
}

foc_result_t foc_PwmClearFault(void)
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

    tConfig.wControlFrequencyHz = 20000U;
    tConfig.tParams.chPolePairs = 7U;
    tConfig.tParams.wResistanceMilliohm = 500U;
    tConfig.tParams.wInductanceDMicroHenry = 1000U;
    tConfig.tParams.wInductanceQMicroHenry = 1000U;
    tConfig.tParams.wVoltageBaseMillivolt = 12000U;
    tConfig.qElectricalSpeedBaseHz = FOC_SCALAR(100.0f);
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

    /* Fixed-axis ALIGN uses the same break protection and clears ownership. */
    assert(motor_StartIdentification(&tMotor) == FOC_RESULT_OK);
    assert(tMotor.eState == MOTOR_STATE_ALIGN);
    assert(tMotor.bIdentificationActive);
    s_bBreakFault = true;
    motor_PollBreakFault(&tMotor);
    assert(motor_GetStatus(&tMotor, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == MOTOR_STATE_FAULT);
    assert(!tStatus.bIdentificationActive && !tStatus.bPwmEnabled);
    assert(tMotor.qIdentificationDcBusPu == FOC_ZERO);
    assert(motor_ClearFault(&tMotor) == FOC_RESULT_OK);

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

    /* A rejected stop must latch a fault instead of reporting IDLE. */
    s_bClearOk = true;
    s_bBreakFault = false;
    assert(motor_ClearFault(&tMotor) == FOC_RESULT_OK);
    assert(motor_Start(&tMotor, FOC_MODE_CURRENT) == FOC_RESULT_OK);
    s_bStopOk = false;
    motor_Stop(&tMotor);
    assert(motor_GetStatus(&tMotor, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == MOTOR_STATE_FAULT);
    assert((tStatus.wFaults & MOTOR_FAULT_PWM) != 0U);
    s_bStopOk = true;
    s_ptTriggerFaultMotor = &tMotor;
    assert(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    assert(tMotor.eState == MOTOR_STATE_FAULT);
    assert((tMotor.wFaults & MOTOR_FAULT_PWM) != 0U);
    s_ptTriggerFaultMotor = NULL;
    printf("Motor break fault tests passed!\n");
    return 0;
}
