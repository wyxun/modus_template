/****************************************************************************
 * @file    motor.c
 * @brief   Single-motor lifecycle and hard-real-time FOC control path.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "motor.h"

#include <limits.h>
#include <stddef.h>

#include "perf_counter.h"

/**
 * @brief Reset the optional Motor-owned Observer.
 * @param ptMotor Motor object.
 * @return None.
 */
static void motor_ResetObserver(motor_t *ptMotor)
{
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    if (ptMotor == NULL) {
        return;
    }
    foc_observer_Reset(&ptMotor->tObserver);
#else
    (void)ptMotor;
#endif
}

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
/**
 * @brief Build the common Observer input for the current Motor sample.
 * @param ptMotor Motor object.
 * @param ptInput Observer input to fill.
 * @return None.
 */
static void motor_BuildObserverInput(
    const motor_t *ptMotor,
    foc_observer_input_t *ptInput)
{
    if (ptMotor == NULL || ptInput == NULL) {
        return;
    }
    ptInput->ptCurrentAlphaBeta = &ptMotor->tInput.tCurrentAlphaBeta;
    ptInput->ptVoltageModelAlphaBeta =
        &ptMotor->tCore.tVoltageAlphaBeta;
    ptInput->ptVoltageAppliedAlphaBeta = NULL;
    ptInput->qDcBusVoltagePu = FOC_ZERO;
    ptInput->bVoltageAppliedValid = false;
    ptInput->bDcBusVoltageValid = false;
}
#endif

/**
 * @brief Enter the latched fault state with PWM already stopped.
 * @param ptMotor Motor object.
 * @param eFault Fault bit to latch.
 * @return None.
 */
static void motor_EnterFault(motor_t *ptMotor, motor_fault_e eFault)
{
    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->eState != MOTOR_STATE_FAULT) {
        foc_pwm_Stop();
    }
    motor_ResetObserver(ptMotor);
    ptMotor->bPwmEnabled = false;
    ptMotor->wFaults |= (uint32_t)eFault;
    ptMotor->eState = MOTOR_STATE_FAULT;
}

/**
 * @brief Validate the configuration that is required by Motor itself.
 * @param ptConfig Motor configuration.
 * @return true when the fixed real-time contract is valid.
 */
static bool motor_ConfigValid(const motor_cfg_t *ptConfig)
{
    if (ptConfig == NULL ||
        ptConfig->tParams.chPolePairs == 0U ||
        ptConfig->tParams.wResistanceMilliohm == 0U ||
        ptConfig->tParams.wInductanceDMicroHenry == 0U ||
        ptConfig->tParams.wInductanceQMicroHenry == 0U ||
        ptConfig->qElectricalSpeedBaseTurnsPerSecond <= FOC_ZERO ||
        ptConfig->tLimits.qMaxSpeedReference <= FOC_ZERO ||
        ptConfig->tLimits.qMaxSpeedReference > FOC_ONE ||
        ptConfig->tLimits.qMaxPhaseCurrent <= FOC_ZERO ||
        ptConfig->tLimits.qMaxPhaseCurrent > FOC_ONE ||
        ptConfig->tLimits.qMaxModulation <= FOC_ZERO ||
        ptConfig->tLimits.qMaxModulation > FOC_ONE ||
        ptConfig->tControl.chSpeedLoopDiv == 0U ||
        ptConfig->tControl.wAdcCalibrationTimeoutSteps == 0U ||
        ptConfig->tControl.wAlignSteps == 0U ||
        ptConfig->tControl.qAlignCurrent <= FOC_ZERO ||
        ptConfig->tControl.qAlignCurrent > FOC_ONE) {
        return false;
    }
    return true;
}

/**
 * @brief Convert mechanical speed to electrical speed PU.
 * @param qMechanicalSpeed Mechanical speed in the active scalar backend.
 * @param ptMotor Motor holding the init-time conversion gain.
 * @return Electrical speed in PU.
 */
static foc_scalar_t motor_ScaleSpeed(const motor_t *ptMotor,
                                     foc_scalar_t qMechanicalSpeed)
{
    return foc_mul_wide(qMechanicalSpeed,
                        ptMotor->qMechanicalToElectricalSpeedPuGain);
}

/**
 * @brief Convert a mechanical BAM32 angle to the Motor electrical angle.
 * @param ptMotor Motor object owning the pole-pair count.
 * @param tMechanicalAngle Mechanical BAM32 angle.
 * @return Electrical BAM32 angle before zero correction.
 */
static foc_angle_t motor_MechanicalToElectrical(
    const motor_t *ptMotor, foc_angle_t tMechanicalAngle)
{
    uint64_t llElectrical = (uint64_t)tMechanicalAngle.wBam32 *
                            (uint64_t)ptMotor->tCfg.tParams.chPolePairs;
    foc_angle_t tElectrical = {0U};

    tElectrical.wBam32 = (uint32_t)llElectrical;
    return tElectrical;
}

/**
 * @brief Convert one mechanical position to the electrical input of Core.
 * @param ptMotor Motor object owning pole pairs and electrical zero.
 * @param ptPosition Mechanical position snapshot.
 * @param ptInput Core input to fill.
 * @return None.
 */
static void motor_BuildPositionInput(const motor_t *ptMotor,
                                     const foc_position_t *ptPosition,
                                     foc_core_input_t *ptInput)
{
    foc_angle_t tElectrical = motor_MechanicalToElectrical(
        ptMotor, ptPosition->tMechanicalAngle);

    ptInput->tElectricalAngle = foc_angle_add(
        tElectrical,
        (foc_angle_t){0U - ptMotor->tElectricalZero.wBam32});
    ptInput->qElectricalSpeedPu = motor_ScaleSpeed(
        ptMotor, ptPosition->qMechanicalSpeed);
    ptInput->bAngleValid = ptPosition->bValid;
}

/**
 * @brief Set the initial safe duty and enable the power stage.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK or a hardware error.
 */
static foc_result_t motor_EnablePwm(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    foc_core_Reset(&ptMotor->tCore);
    motor_ResetObserver(ptMotor);
    eResult = foc_pwm_SetDuty(&ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = foc_pwm_Enable();
    if (eResult != FOC_RESULT_OK) {
        foc_pwm_Stop();
        return eResult;
    }
    ptMotor->bPwmEnabled = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Complete one ADC calibration step in the ISR-owned state machine.
 * @param ptMotor Motor object.
 * @return None.
 */
static void motor_AdcCalibrationStep(motor_t *ptMotor)
{
    foc_calibration_state_e eResult = FOC_CALIBRATION_BUSY;

    if (ptMotor->wCalibrationSteps < UINT32_MAX) {
        ptMotor->wCalibrationSteps++;
    }
    eResult = foc_adc_CalibStep(&ptMotor->tCalib);
    if (eResult == FOC_CALIBRATION_COMPLETE) {
        ptMotor->eState = MOTOR_STATE_IDLE;
    } else if (eResult == FOC_CALIBRATION_FAILED ||
               ptMotor->wCalibrationSteps >=
                   ptMotor->tCfg.tControl.wAdcCalibrationTimeoutSteps) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_CAL);
    } else {
        /* Calibration continues on the next ADC interrupt. */
    }
}

/**
 * @brief Construct one current-loop input from the ADC and position source.
 * @param ptMotor Motor object.
 * @param wNowTick Current low 32-bit system tick.
 * @return FOC_RESULT_OK or the first failed input dependency.
 */
static foc_result_t motor_ReadInput(motor_t *ptMotor,
                                    uint32_t wNowTick)
{
    foc_current_abc_t tCurrent = {0};
    foc_position_t tPosition = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_adc_Sample(&ptMotor->tCalib, &tCurrent);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = foc_clarke(tCurrent.qU, tCurrent.qV, tCurrent.qW,
                         &ptMotor->tInput.tCurrentAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (ptMotor->tCfg.fnGetPosition == NULL) {
        return FOC_RESULT_DISABLED;
    }
    eResult = ptMotor->tCfg.fnGetPosition(
        ptMotor->tCfg.pPositionContext, wNowTick, &tPosition);
    if (eResult != FOC_RESULT_OK || !tPosition.bValid) {
        return eResult == FOC_RESULT_OK ? FOC_RESULT_SAFETY : eResult;
    }
    motor_BuildPositionInput(ptMotor, &tPosition, &ptMotor->tInput);
    return FOC_RESULT_OK;
}

/**
 * @brief Run the speed loop at its configured sub-rate.
 * @param ptMotor Motor object.
 * @return None.
 */
static void motor_SpeedLoopStep(motor_t *ptMotor)
{
    if (ptMotor->tCommand.eMode != FOC_MODE_SPEED) {
        return;
    }
    ptMotor->chSpeedLoopCount++;
    if (ptMotor->chSpeedLoopCount < ptMotor->tCfg.tControl.chSpeedLoopDiv) {
        return;
    }
    ptMotor->chSpeedLoopCount = 0U;
    ptMotor->tCommand.tCurrentReference.qQ = foc_pid_Step(
        &ptMotor->tSpeedPi,
        ptMotor->tCommand.qSpeedReferencePu,
        ptMotor->tInput.qElectricalSpeedPu);
}

/**
 * @brief Run one current-loop step and submit its duty.
 * @param ptMotor Motor object.
 * @return None.
 */
static void motor_RunStep(motor_t *ptMotor, uint32_t wNowTick)
{
    foc_result_t eResult = FOC_RESULT_OK;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    foc_observer_input_t tObserverInput = {0};
    foc_result_t eObserver = FOC_RESULT_OK;
#endif

    eResult = motor_ReadInput(ptMotor, wNowTick);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
        return;
    }
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    /* HFI is not implemented, so only the model voltage is valid. */
    motor_BuildObserverInput(ptMotor, &tObserverInput);
    eObserver = foc_observer_Step(&ptMotor->tObserver,
                                  &tObserverInput);
    if (eObserver != FOC_RESULT_OK) {
        ptMotor->tObserver.tOutput.bValid = false;
    }
#endif
    motor_SpeedLoopStep(ptMotor);
    eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                            &ptMotor->tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = foc_pwm_SetDuty(&ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
    }
}

/**
 * @brief Run the fixed-angle ALIGN current loop and capture electrical zero.
 * @param ptMotor Motor object.
 * @param wNowTick Current low 32-bit system tick.
 * @return None.
 */
static void motor_AlignStep(motor_t *ptMotor, uint32_t wNowTick)
{
    foc_current_abc_t tCurrent = {0};
    foc_position_t tPosition = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_adc_Sample(&ptMotor->tCalib, &tCurrent);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_SAMPLE);
        return;
    }
    eResult = foc_clarke(tCurrent.qU, tCurrent.qV, tCurrent.qW,
                         &ptMotor->tInput.tCurrentAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    ptMotor->tInput.tElectricalAngle = (foc_angle_t){0U};
    ptMotor->tInput.qElectricalSpeedPu = FOC_ZERO;
    ptMotor->tInput.bAngleValid = true;
    eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                            &ptMotor->tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = foc_pwm_SetDuty(&ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
        return;
    }
    if (ptMotor->wAlignSteps < UINT32_MAX) {
        ptMotor->wAlignSteps++;
    }
    if (ptMotor->wAlignSteps < ptMotor->tCfg.tControl.wAlignSteps) {
        return;
    }
    if (ptMotor->tCfg.fnGetPosition == NULL) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_ALIGN);
        return;
    }
    eResult = ptMotor->tCfg.fnGetPosition(
        ptMotor->tCfg.pPositionContext, wNowTick, &tPosition);
    if (eResult != FOC_RESULT_OK || !tPosition.bValid) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_ALIGN);
        return;
    }
    ptMotor->tElectricalZero = motor_MechanicalToElectrical(
        ptMotor, tPosition.tMechanicalAngle);
    ptMotor->bElectricalZeroValid = true;
    foc_pwm_Stop();
    ptMotor->bPwmEnabled = false;
    motor_ResetObserver(ptMotor);
    ptMotor->eState = MOTOR_STATE_IDLE;
}

foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;
    foc_scalar_t qPolePairs = FOC_ZERO;

    foc_pwm_Stop();
    if (ptMotor == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_ConfigValid(ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptMotor = (motor_t){0};
    ptMotor->tCfg = *ptConfig;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    eResult = foc_observer_Init(&ptMotor->tObserver,
                                &ptMotor->tCfg.tParams,
                                &ptMotor->tCfg.tObserverCfg);
    if (eResult != FOC_RESULT_OK) {
        foc_pwm_Stop();
        return eResult;
    }
#endif
    qPolePairs = foc_from_float((float)ptConfig->tParams.chPolePairs);
    eResult = foc_div_checked(qPolePairs,
        ptConfig->qElectricalSpeedBaseTurnsPerSecond,
        &ptMotor->qMechanicalToElectricalSpeedPuGain);
    if (eResult != FOC_RESULT_OK ||
        ptMotor->qMechanicalToElectricalSpeedPuGain <= FOC_ZERO) {
        foc_pwm_Stop();
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptMotor->eState = MOTOR_STATE_INITIALIZING;
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    eResult = foc_pid_Init(&ptMotor->tCore.tIdPi,
                           &ptConfig->tControl.tCurrentPiParams);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tCore.tIqPi,
                               &ptConfig->tControl.tCurrentPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tSpeedPi,
                               &ptConfig->tControl.tSpeedPiParams);
    }
    if (eResult != FOC_RESULT_OK) {
        foc_pwm_Stop();
        return eResult;
    }
    foc_core_Reset(&ptMotor->tCore);
    motor_ResetObserver(ptMotor);
    foc_adc_CalibBegin(&ptMotor->tCalib);
    ptMotor->eState = MOTOR_STATE_ADC_CAL;
    return FOC_RESULT_OK;
}

foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (eMode >= FOC_MODE_POSITION) {
        return FOC_RESULT_DISABLED;
    }
    if (ptMotor->tCfg.fnGetPosition == NULL) {
        return FOC_RESULT_DISABLED;
    }
    /* 硬件 break 锁存未清除时禁止重新使能功率级 */
    if (foc_pwm_GetFaultStatus()) {
        return FOC_RESULT_SAFETY;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->eState != MOTOR_STATE_IDLE ||
        ptMotor->wFaults != MOTOR_FAULT_NONE ||
        !ptMotor->tCalib.bIsCalibrated) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommand = (foc_core_command_t){0};
    ptMotor->tCommand.eMode = eMode;
    foc_pid_Reset(&ptMotor->tSpeedPi);
    eResult = motor_EnablePwm(ptMotor);
    if (eResult == FOC_RESULT_OK) {
        ptMotor->eState = MOTOR_STATE_RUNNING;
    } else {
        motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

void motor_Stop(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    foc_pwm_Stop();
    ptMotor->bPwmEnabled = false;
    foc_pid_Reset(&ptMotor->tSpeedPi);
    motor_ResetObserver(ptMotor);
    if (ptMotor->eState != MOTOR_STATE_FAULT &&
        ptMotor->eState != MOTOR_STATE_ADC_CAL &&
        ptMotor->eState != MOTOR_STATE_INITIALIZING) {
        ptMotor->eState = MOTOR_STATE_IDLE;
    }
    perfc_port_resume_global_interrupt(tIrqState);
}

foc_result_t motor_ClearFault(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    bool bAdcCalibrationFault = false;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->eState != MOTOR_STATE_FAULT || ptMotor->bPwmEnabled) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    /* 清除 PWM 故障前先确认硬件 break 源已释放，否则拒绝 */
    if ((ptMotor->wFaults & (uint32_t)MOTOR_FAULT_PWM) != 0U &&
        foc_pwm_ClearFaultStatus() != FOC_RESULT_OK) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_SAFETY;
    }
    bAdcCalibrationFault = (ptMotor->wFaults &
                            (uint32_t)MOTOR_FAULT_ADC_CAL) != 0U;
    ptMotor->wFaults = MOTOR_FAULT_NONE;
    foc_core_Reset(&ptMotor->tCore);
    motor_ResetObserver(ptMotor);
    foc_pid_Reset(&ptMotor->tSpeedPi);
    if (bAdcCalibrationFault) {
        foc_adc_CalibBegin(&ptMotor->tCalib);
        ptMotor->wCalibrationSteps = 0U;
        ptMotor->eState = MOTOR_STATE_ADC_CAL;
    } else {
        ptMotor->eState = MOTOR_STATE_IDLE;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

void motor_PollBreakFault(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL || !foc_pwm_GetFaultStatus()) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
    perfc_port_resume_global_interrupt(tIrqState);
}

/**
 * @brief Update one selected reference under the ISR handoff guard.
 * @param ptMotor Motor object.
 * @param eMode Required command mode.
 * @param ptReference Reference pair to write.
 * @param qD D-axis value.
 * @param qQ Q-axis value.
 * @return FOC_RESULT_OK, FOC_RESULT_OUT_OF_RANGE when the vector
 *         magnitude exceeds the mode limit, or FOC_RESULT_INVALID_ARGUMENT
 *         for a non-finite value or state/mode error.
 */
static foc_result_t motor_SetDqReference(motor_t *ptMotor,
                                         foc_control_mode_e eMode,
                                         foc_dq_t *ptReference,
                                         foc_scalar_t qD,
                                         foc_scalar_t qQ)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_scalar_t qLimit = FOC_ZERO;
    foc_scalar_t qMagSq = FOC_ZERO;
    foc_scalar_t qLimSq = FOC_ZERO;

    if (ptMotor == NULL || ptReference == NULL) {
        return FOC_RESULT_NULL;
    }
    /* NaN/Inf 会使幅值比较恒为 false 而被放行，必须先拒 */
    if (!foc_scalar_is_finite(qD) || !foc_scalar_is_finite(qQ)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    /* 电压用调制度上限、电流用相电流上限；拒绝越界参考，
       避免浮点/定点行为分叉及逆变器饱和。 */
    if (eMode == FOC_MODE_VOLTAGE) {
        qLimit = ptMotor->tCfg.tLimits.qMaxModulation;
    } else {
        qLimit = ptMotor->tCfg.tLimits.qMaxPhaseCurrent;
    }
    qMagSq = foc_add_sat(foc_mul_wide(qD, qD), foc_mul_wide(qQ, qQ));
    qLimSq = foc_mul_wide(qLimit, qLimit);
    if (qMagSq > qLimSq) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tCommand.eMode != eMode ||
        ptMotor->eState == MOTOR_STATE_FAULT) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptReference->qD = qD;
    ptReference->qQ = qQ;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    return motor_SetDqReference(ptMotor, FOC_MODE_VOLTAGE,
                                &ptMotor->tCommand.tVoltageReference,
                                qD, qQ);
}

foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    return motor_SetDqReference(ptMotor, FOC_MODE_CURRENT,
                                &ptMotor->tCommand.tCurrentReference,
                                qD, qQ);
}

foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qSpeedReferencePu)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_scalar_is_finite(qSpeedReferencePu)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tCommand.eMode != FOC_MODE_SPEED ||
        ptMotor->eState == MOTOR_STATE_FAULT) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (qSpeedReferencePu > ptMotor->tCfg.tLimits.qMaxSpeedReference ||
        qSpeedReferencePu < -ptMotor->tCfg.tLimits.qMaxSpeedReference) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptMotor->tCommand.qSpeedReferencePu = qSpeedReferencePu;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tCfg.fnGetPosition == NULL) {
        return FOC_RESULT_DISABLED;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->eState != MOTOR_STATE_IDLE ||
        !ptMotor->tCalib.bIsCalibrated) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommand = (foc_core_command_t){0};
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    ptMotor->tCommand.tCurrentReference.qD =
        ptMotor->tCfg.tControl.qAlignCurrent;
    ptMotor->wAlignSteps = 0U;
    eResult = motor_EnablePwm(ptMotor);
    if (eResult == FOC_RESULT_OK) {
        ptMotor->eState = MOTOR_STATE_ALIGN;
    } else {
        motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

void motor_HighFrequencyStep(motor_t *ptMotor, uint32_t wNowTick)
{
    if (ptMotor == NULL) {
        return;
    }
    switch (ptMotor->eState) {
    case MOTOR_STATE_INITIALIZING:
        foc_pwm_Stop();
        ptMotor->eState = MOTOR_STATE_ADC_CAL;
        break;
    case MOTOR_STATE_ADC_CAL:
        motor_AdcCalibrationStep(ptMotor);
        break;
    case MOTOR_STATE_IDLE:
        break;
    case MOTOR_STATE_ALIGN:
        motor_AlignStep(ptMotor, wNowTick);
        break;
    case MOTOR_STATE_RUNNING:
        motor_RunStep(ptMotor, wNowTick);
        break;
    case MOTOR_STATE_FAULT:
        break;
    default:
        motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        break;
    }
}

foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL || ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptStatus->eState = ptMotor->eState;
    ptStatus->wFaults = ptMotor->wFaults;
    ptStatus->eMode = ptMotor->tCommand.eMode;
    ptStatus->bPwmEnabled = ptMotor->bPwmEnabled;
    ptStatus->bElectricalZeroValid = ptMotor->bElectricalZeroValid;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
