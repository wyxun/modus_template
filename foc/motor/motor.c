/****************************************************************************
 * @file    motor.c
 * @brief   Single-motor lifecycle and hard-real-time FOC control path.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "motor.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "internal/foc_units.h"
#include "perf_counter.h"

#define MOTOR_ADC_OFFSET_SAMPLE_COUNT FOC_OFFSET_CALIB_TIMES

_Static_assert(MOTOR_ADC_OFFSET_SAMPLE_COUNT > 0U &&
               MOTOR_ADC_OFFSET_SAMPLE_COUNT <= UINT16_MAX,
               "ADC offset calibration count must fit the sample counter");

static void _motor_ResetAdcCalibration(motor_t *ptMotor)
{
    ptMotor->tCalib = (motor_adc_calib_t){0};
    ptMotor->wCalibrationSteps = 0U;
}

static bool _motor_AdcOffsetsValid(const motor_adc_calib_t *ptCalibration)
{
    return ptCalibration->wOffsetU != 0U &&
           ptCalibration->wOffsetV != 0U &&
           ptCalibration->wOffsetW != 0U;
}

/**
 * @brief Convert one calibrated ADC current sample to FOC per-unit form.
 * @param wRaw Raw ADC sample.
 * @param wOffset Calibrated ADC zero-current offset.
 * @return Signed phase current in the active FOC scalar backend.
 */
static foc_scalar_t _motor_NormalizeCurrent(uint32_t wRaw,
                                            uint32_t wOffset)
{
    int32_t nDelta = (int32_t)wOffset - (int32_t)wRaw;
    const int32_t nMaximumCounts =
        (int32_t)FOC_CURRENT_COUNTS_PER_BASE;

#if !FOC_CURRENT_SAMPLE_INVERTED
    nDelta = -nDelta;
#endif
    if (nDelta > nMaximumCounts) {
        nDelta = nMaximumCounts;
    } else if (nDelta < -nMaximumCounts) {
        nDelta = -nMaximumCounts;
    }
#if defined(FOC_NUMERIC_FIXED)
    return (foc_scalar_t)(((int64_t)nDelta * FOC_Q_SCALE) /
                          FOC_CURRENT_COUNTS_PER_BASE);
#else
    return (foc_scalar_t)nDelta /
           (foc_scalar_t)FOC_CURRENT_COUNTS_PER_BASE;
#endif
}

static foc_result_t _motor_ReadCurrent(const motor_t *ptMotor,
                                       foc_current_abc_t *ptCurrent)
{
    foc_current_sample_t tSample = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL || ptCurrent == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptMotor->tCalib.bIsCalibrated) {
        return FOC_RESULT_SAFETY;
    }
    eResult = FOC_PORT_SAMPLE_CURRENT(&tSample);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    ptCurrent->qU = _motor_NormalizeCurrent(tSample.wU,                         \
        ptMotor->tCalib.wOffsetU);
    ptCurrent->qV = _motor_NormalizeCurrent(tSample.wV,                         \
        ptMotor->tCalib.wOffsetV);
    ptCurrent->qW = _motor_NormalizeCurrent(tSample.wW,                         \
        ptMotor->tCalib.wOffsetW);
    return FOC_RESULT_OK;
}

/**
 * @brief Enter the latched fault state with PWM already stopped.
 * @param ptMotor Motor object.
 * @param eFault Fault bit to latch.
 * @return None.
 */
static void _motor_EnterFault(motor_t *ptMotor, motor_fault_e eFault)
{
    foc_result_t eStop = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->eState != MOTOR_STATE_FAULT) {
        eStop = FOC_PORT_PWM_SAFE_STOP();
        if (eStop != FOC_RESULT_OK) {
            ptMotor->wFaults |= (uint32_t)MOTOR_FAULT_PWM;
        }
    }
    ptMotor->tCommand.tVoltageReference =
        (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptMotor->bControlPrepared = false;
    ptMotor->bAlignCapturePending = false;
    ptMotor->bPwmEnabled = false;
    ptMotor->wFaults |= (uint32_t)eFault;
    ptMotor->eState = MOTOR_STATE_FAULT;
}

/**
 * @brief Validate the configuration that is required by Motor itself.
 * @param ptConfig Motor configuration.
 * @return true when the fixed real-time contract is valid.
 */
static bool _motor_ParametersValid(const motor_cfg_t *ptConfig)
{
    return ptConfig->tParams.chPolePairs != 0U &&
           ptConfig->tParams.wResistanceMilliohm != 0U &&
           ptConfig->tParams.wInductanceDMicroHenry != 0U &&
           ptConfig->tParams.wInductanceQMicroHenry != 0U &&
           ptConfig->qElectricalSpeedBaseTurnsPerSecond > FOC_ZERO;
}

static bool _motor_LimitsValid(const motor_cfg_t *ptConfig)
{
    return ptConfig->tLimits.qMaxSpeedReference > FOC_ZERO &&
           ptConfig->tLimits.qMaxSpeedReference <= FOC_ONE &&
           ptConfig->tLimits.qMaxPhaseCurrent > FOC_ZERO &&
           ptConfig->tLimits.qMaxPhaseCurrent <= FOC_ONE &&
           ptConfig->tLimits.qMaxModulation > FOC_ZERO &&
           ptConfig->tLimits.qMaxModulation <= FOC_ONE;
}

static bool _motor_ControlValid(const motor_cfg_t *ptConfig)
{
    return isfinite(ptConfig->fAdcCalibrationTimeoutSeconds) &&
           ptConfig->fAdcCalibrationTimeoutSeconds > 0.0f &&
           isfinite(ptConfig->fAlignTimeSeconds) &&
           ptConfig->fAlignTimeSeconds > 0.0f &&
           ptConfig->wControlFrequencyHz != 0U &&
           ptConfig->wControlFrequencyHz <=
               FOC_NANOSECONDS_PER_SECOND &&
           ptConfig->wSpeedLoopFrequencyHz != 0U &&
           ptConfig->qAlignCurrent > FOC_ZERO &&
           ptConfig->qAlignCurrent <= FOC_ONE;
}

static bool _motor_SecondsToSteps(float fSeconds, uint32_t wFrequencyHz,
                                 uint32_t *pwSteps)
{
    double dSteps = 0.0;

    if (pwSteps == NULL || !isfinite(fSeconds) || fSeconds <= 0.0f ||
        wFrequencyHz == 0U) {
        return false;
    }
    dSteps = (double)fSeconds * (double)wFrequencyHz;
    if (dSteps > (double)UINT32_MAX) {
        return false;
    }
    dSteps = dSteps < 1.0 ? 1.0 : ceil(dSteps - 1.0e-6);
    *pwSteps = (uint32_t)dSteps;
    return true;
}

static bool _motor_ConfigValid(const motor_cfg_t *ptConfig)
{
    if (ptConfig == NULL) {
        return false;
    }
    return _motor_ParametersValid(ptConfig) &&
           _motor_LimitsValid(ptConfig) &&
           _motor_ControlValid(ptConfig);
}

/**
 * @brief Set the initial safe duty and enable the power stage.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK or a hardware error.
 */
static foc_result_t _motor_EnablePwm(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    foc_core_Reset(&ptMotor->tCore);
    ptMotor->bControlPrepared = false;
    ptMotor->bAlignCapturePending = false;
    eResult = FOC_PORT_SET_DUTY(&ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = FOC_PORT_PWM_ENABLE();
    if (eResult != FOC_RESULT_OK) {
        (void)FOC_PORT_PWM_SAFE_STOP();
        return eResult;
    }
    ptMotor->bPwmEnabled = true;
    ptMotor->wRunGeneration++;
    return FOC_RESULT_OK;
}

/**
 * @brief Complete one ADC calibration step in the ISR-owned state machine.
 * @param ptMotor Motor object.
 * @return None.
 */
static void _motor_AdcCalibrationStep(motor_t *ptMotor)
{
    foc_current_sample_t tSample = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor->wCalibrationSteps < UINT32_MAX) {
        ptMotor->wCalibrationSteps++;
    }
    eResult = FOC_PORT_SAMPLE_CURRENT(&tSample);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_CAL);
        return;
    }
    ptMotor->tCalib.ullSumU += (uint64_t)tSample.wU;
    ptMotor->tCalib.ullSumV += (uint64_t)tSample.wV;
    ptMotor->tCalib.ullSumW += (uint64_t)tSample.wW;
    if (ptMotor->tCalib.hwSampleCount < UINT16_MAX) {
        ptMotor->tCalib.hwSampleCount++;
    }
    if (ptMotor->tCalib.hwSampleCount >=
        MOTOR_ADC_OFFSET_SAMPLE_COUNT) {
        ptMotor->tCalib.wOffsetU = (uint32_t)(
            ptMotor->tCalib.ullSumU / MOTOR_ADC_OFFSET_SAMPLE_COUNT);
        ptMotor->tCalib.wOffsetV = (uint32_t)(
            ptMotor->tCalib.ullSumV / MOTOR_ADC_OFFSET_SAMPLE_COUNT);
        ptMotor->tCalib.wOffsetW = (uint32_t)(
            ptMotor->tCalib.ullSumW / MOTOR_ADC_OFFSET_SAMPLE_COUNT);
        ptMotor->tCalib.bIsCalibrated = _motor_AdcOffsetsValid(
            &ptMotor->tCalib);
    }
    if (ptMotor->tCalib.bIsCalibrated) {
        ptMotor->eState = MOTOR_STATE_IDLE;
    } else if (ptMotor->wCalibrationSteps >=
                   ptMotor->wAdcCalibrationTimeoutSteps) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_CAL);
    } else {
        /* Calibration continues on the next ADC interrupt. */
    }
}

/**
 * @brief Run the speed loop at its configured sub-rate.
 * @param ptMotor Motor object.
 * @return None.
 */
static void _motor_SpeedLoopStep(motor_t *ptMotor)
{
    if (ptMotor->tCommand.eMode != FOC_MODE_SPEED) {
        return;
    }
    ptMotor->chSpeedLoopCount++;
    if (ptMotor->chSpeedLoopCount < ptMotor->chSpeedLoopDiv) {
        return;
    }
    ptMotor->chSpeedLoopCount = 0U;
    ptMotor->tCommand.tCurrentReference.qQ = foc_pid_Step(
        &ptMotor->tSpeedPi,
        ptMotor->tCommand.qSpeedReferencePu,
        ptMotor->tInput.qElectricalSpeedPu);
}

/**
 * @brief Sample one RUN current and expose the prior voltage model.
 * @param ptMotor Motor object.
 * @param ptSample Synchronized sample output.
 * @return Whether the control half may run.
 */
static motor_isr_phase_t _motor_PrepareRun(
    motor_t *ptMotor, motor_position_sample_t *ptSample)
{
    foc_current_abc_t tCurrent = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor->bControlPrepared) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return MOTOR_ISR_NO_CONTROL;
    }
    eResult = _motor_ReadCurrent(ptMotor, &tCurrent);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_SAMPLE);
        return MOTOR_ISR_NO_CONTROL;
    }
    eResult = foc_clarke(tCurrent.qU, tCurrent.qV, tCurrent.qW,
                         &ptMotor->tInput.tCurrentAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return MOTOR_ISR_NO_CONTROL;
    }
    ptSample->tCurrentAlphaBeta = ptMotor->tInput.tCurrentAlphaBeta;
    ptSample->tVoltageModelAlphaBeta = ptMotor->tCore.tVoltageAlphaBeta;
    ptSample->tHardDragCandidate = (motor_electrical_feedback_t){
        .tElectricalAngle = ptMotor->tHardDragAngle,
        .qElectricalSpeedPu = ptMotor->qHardDragSpeedPu,
        .bValid = ptMotor->wHardDragAngleStepBam32 != 0U,
    };
    ptMotor->tHardDragAngle.wBam32 += ptMotor->wHardDragAngleStepBam32;
    ptSample->wRunGeneration = ptMotor->wRunGeneration;
    ptMotor->bControlPrepared = true;
    return MOTOR_ISR_CONTROL_READY;
}

/**
 * @brief Run the fixed-angle ALIGN current loop.
 * @param ptMotor Motor object.
 * @return Capture request after the configured hold period.
 */
static motor_isr_phase_t _motor_AlignStep(motor_t *ptMotor)
{
    foc_current_abc_t tCurrent = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor->bAlignCapturePending) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ALIGN);
        return MOTOR_ISR_NO_CONTROL;
    }
    eResult = _motor_ReadCurrent(ptMotor, &tCurrent);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_SAMPLE);
        return MOTOR_ISR_NO_CONTROL;
    }
    eResult = foc_clarke(tCurrent.qU, tCurrent.qV, tCurrent.qW,
                         &ptMotor->tInput.tCurrentAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return MOTOR_ISR_NO_CONTROL;
    }
    ptMotor->tInput.tElectricalAngle = (foc_angle_t){0U};
    ptMotor->tInput.qElectricalSpeedPu = FOC_ZERO;
    ptMotor->tInput.bAngleValid = true;
    eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                            &ptMotor->tInput);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return MOTOR_ISR_NO_CONTROL;
    }
    eResult = FOC_PORT_SET_DUTY(&ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
        return MOTOR_ISR_NO_CONTROL;
    }
    if (ptMotor->wAlignStepCount < UINT32_MAX) {
        ptMotor->wAlignStepCount++;
    }
    if (ptMotor->wAlignStepCount < ptMotor->wAlignTargetSteps) {
        return MOTOR_ISR_NO_CONTROL;
    }
    ptMotor->bAlignCapturePending = true;
    return MOTOR_ISR_CAPTURE_ZERO;
}

foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;
    uint32_t wAdcCalibrationTimeoutSteps = 0U;
    uint32_t wAlignTargetSteps = 0U;
    uint32_t wSpeedLoopDiv = 0U;

    if (ptMotor == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!_motor_ConfigValid(ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!_motor_SecondsToSteps(ptConfig->fAdcCalibrationTimeoutSeconds,
                               ptConfig->wControlFrequencyHz,
                               &wAdcCalibrationTimeoutSteps) ||
        !_motor_SecondsToSteps(ptConfig->fAlignTimeSeconds,
                               ptConfig->wControlFrequencyHz,
                               &wAlignTargetSteps) ||
        ptConfig->wControlFrequencyHz %
            ptConfig->wSpeedLoopFrequencyHz != 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wSpeedLoopDiv = ptConfig->wControlFrequencyHz /
                    ptConfig->wSpeedLoopFrequencyHz;
    if (wSpeedLoopDiv == 0U || wSpeedLoopDiv > UINT8_MAX) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    *ptMotor = (motor_t){0};
    ptMotor->tParams = ptConfig->tParams;
    ptMotor->wCurrentBaseMilliamp = FOC_CURRENT_BASE_MILLIAMP;
    ptMotor->tParams.wCurrentBaseMilliamp =
        FOC_CURRENT_BASE_MILLIAMP;
    ptMotor->tLimits = ptConfig->tLimits;
    ptMotor->qAlignCurrent = ptConfig->qAlignCurrent;
    ptMotor->wAdcCalibrationTimeoutSteps =
        wAdcCalibrationTimeoutSteps;
    ptMotor->wAlignTargetSteps = wAlignTargetSteps;
    ptMotor->chSpeedLoopDiv = (uint8_t)wSpeedLoopDiv;
    if (ptConfig->nHardDragElectricalMilliHz != 0) {
        int64_t llFrequency = ptConfig->nHardDragElectricalMilliHz;
        uint64_t ullStepsPerSecondMilli = 0U;
        uint64_t ullStep = 0U;
        float fSpeedPu = 0.0f;

        fSpeedPu = ((float)llFrequency / 1000.0f) /
                   foc_to_float(ptConfig->qElectricalSpeedBaseTurnsPerSecond);
        if (!isfinite(fSpeedPu) ||
            fSpeedPu > foc_to_float(ptConfig->tLimits.qMaxSpeedReference) ||
            fSpeedPu < -foc_to_float(ptConfig->tLimits.qMaxSpeedReference)) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        if (llFrequency < 0) {
            llFrequency = -llFrequency;
        }
        ullStepsPerSecondMilli =
            (uint64_t)ptConfig->wControlFrequencyHz * 1000ULL;
        ullStep = (((uint64_t)llFrequency << 32U) +
                   ullStepsPerSecondMilli / 2U) /
                  ullStepsPerSecondMilli;
        ptMotor->qHardDragSpeedPu = foc_from_float(fSpeedPu);
        if (ullStep == 0U || ullStep >= 0x80000000ULL ||
            ptMotor->qHardDragSpeedPu == FOC_ZERO) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        ptMotor->wHardDragAngleStepBam32 =
            ptConfig->nHardDragElectricalMilliHz < 0
                ? 0U - (uint32_t)ullStep : (uint32_t)ullStep;
    }
    eResult = FOC_PORT_PWM_SAFE_STOP();
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    ptMotor->eState = MOTOR_STATE_INITIALIZING;
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    eResult = foc_pid_Init(&ptMotor->tCore.tIdPi,
                           &ptConfig->tCurrentPiParams);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tCore.tIqPi,
                               &ptConfig->tCurrentPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tSpeedPi,
                               &ptConfig->tSpeedPiParams);
    }
    if (eResult != FOC_RESULT_OK) {
        (void)FOC_PORT_PWM_SAFE_STOP();
        return eResult;
    }
    foc_core_Reset(&ptMotor->tCore);
    _motor_ResetAdcCalibration(ptMotor);
    FOC_PORT_START_ADC_TRIGGER();
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
    /* 硬件 break 锁存未清除时禁止重新使能功率级 */
    if (FOC_PORT_PWM_GET_FAULT()) {
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
    ptMotor->tHardDragAngle = (foc_angle_t){0U};
    foc_pid_Reset(&ptMotor->tSpeedPi);
    eResult = _motor_EnablePwm(ptMotor);
    if (eResult == FOC_RESULT_OK) {
        ptMotor->eState = MOTOR_STATE_RUNNING;
    } else {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
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
    (void)FOC_PORT_PWM_SAFE_STOP();
    ptMotor->tCommand.tVoltageReference =
        (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptMotor->bPwmEnabled = false;
    foc_pid_Reset(&ptMotor->tSpeedPi);
    ptMotor->bControlPrepared = false;
    ptMotor->bAlignCapturePending = false;
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
        FOC_PORT_PWM_CLEAR_FAULT() != FOC_RESULT_OK) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_SAFETY;
    }
    bAdcCalibrationFault = (ptMotor->wFaults &
                            (uint32_t)MOTOR_FAULT_ADC_CAL) != 0U;
    ptMotor->wFaults = MOTOR_FAULT_NONE;
    foc_core_Reset(&ptMotor->tCore);
    ptMotor->bControlPrepared = false;
    ptMotor->bAlignCapturePending = false;
    foc_pid_Reset(&ptMotor->tSpeedPi);
    if (bAdcCalibrationFault) {
        _motor_ResetAdcCalibration(ptMotor);
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

    if (ptMotor == NULL || !FOC_PORT_PWM_GET_FAULT()) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
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
static foc_result_t _motor_SetDqReference(motor_t *ptMotor,
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
        qLimit = ptMotor->tLimits.qMaxModulation;
    } else {
        qLimit = ptMotor->tLimits.qMaxPhaseCurrent;
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
    return _motor_SetDqReference(ptMotor, FOC_MODE_VOLTAGE,
                                &ptMotor->tCommand.tVoltageReference,
                                qD, qQ);
}

/**
 * @brief Submit a prevalidated identification voltage from the FOC ISR.
 * @param ptMotor Running voltage-mode Motor.
 * @param ptVoltageCommand D/Q modulation command.
 * @return FOC_RESULT_OK or a state/safety error.
 * @note Identify calls this after motor_IsrStep(), so the command takes
 *       effect on the following PWM update.
 */
foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor,
    const foc_dq_t *ptVoltageCommand)
{
    if (ptMotor == NULL || ptVoltageCommand == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->eState != MOTOR_STATE_RUNNING || !ptMotor->bPwmEnabled ||
        ptMotor->tCommand.eMode != FOC_MODE_VOLTAGE) {
        return FOC_RESULT_SAFETY;
    }
    ptMotor->tCommand.tVoltageReference = *ptVoltageCommand;
    return FOC_RESULT_OK;
}

/**
 * @brief Atomically stop a failed identification run in the FOC ISR.
 * @param ptMotor Motor to stop.
 * @param eFault Fault bit to latch.
 * @return None.
 */
void motor_IdentificationAbortIsr(
    motor_t *ptMotor,
    motor_fault_e eFault)
{
    _motor_EnterFault(ptMotor, eFault);
}

foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    return _motor_SetDqReference(ptMotor, FOC_MODE_CURRENT,
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
    if (qSpeedReferencePu > ptMotor->tLimits.qMaxSpeedReference ||
        qSpeedReferencePu < -ptMotor->tLimits.qMaxSpeedReference) {
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
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->eState != MOTOR_STATE_IDLE ||
        !ptMotor->tCalib.bIsCalibrated) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommand = (foc_core_command_t){0};
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    ptMotor->tCommand.tCurrentReference.qD = ptMotor->qAlignCurrent;
    ptMotor->wAlignStepCount = 0U;
    ptMotor->bElectricalZeroValid = false;
    eResult = _motor_EnablePwm(ptMotor);
    if (eResult == FOC_RESULT_OK) {
        ptMotor->eState = MOTOR_STATE_ALIGN;
    } else {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

motor_isr_phase_t motor_IsrPrepare(motor_t *ptMotor,
                                   motor_position_sample_t *ptSample)
{
    if (ptMotor == NULL) {
        return MOTOR_ISR_NO_CONTROL;
    }
    switch (ptMotor->eState) {
    case MOTOR_STATE_INITIALIZING:
        (void)FOC_PORT_PWM_SAFE_STOP();
        ptMotor->eState = MOTOR_STATE_ADC_CAL;
        break;
    case MOTOR_STATE_ADC_CAL:
        _motor_AdcCalibrationStep(ptMotor);
        break;
    case MOTOR_STATE_IDLE:
        break;
    case MOTOR_STATE_ALIGN:
        return _motor_AlignStep(ptMotor);
    case MOTOR_STATE_RUNNING:
        if (ptSample == NULL) {
            _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
            break;
        }
        return _motor_PrepareRun(ptMotor, ptSample);
    case MOTOR_STATE_FAULT:
        break;
    default:
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        break;
    }
    return MOTOR_ISR_NO_CONTROL;
}

void motor_IsrControlStep(
    motor_t *ptMotor, const motor_electrical_feedback_t *ptFeedback)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->eState != MOTOR_STATE_RUNNING ||
        !ptMotor->bControlPrepared) {
        if (ptMotor->eState == MOTOR_STATE_RUNNING) {
            _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        }
        return;
    }
    ptMotor->bControlPrepared = false;
    if (ptFeedback == NULL || !ptFeedback->bValid ||
        !foc_scalar_is_finite(ptFeedback->qElectricalSpeedPu)) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
        return;
    }
    ptMotor->tInput.tElectricalAngle = ptFeedback->tElectricalAngle;
    ptMotor->tInput.qElectricalSpeedPu = ptFeedback->qElectricalSpeedPu;
    ptMotor->tInput.bAngleValid = true;
    _motor_SpeedLoopStep(ptMotor);
    eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                            &ptMotor->tInput);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = FOC_PORT_SET_DUTY(&ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
    }
}

void motor_CompleteAlignIsr(motor_t *ptMotor, foc_result_t eCapture)
{
    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->eState != MOTOR_STATE_ALIGN ||
        !ptMotor->bAlignCapturePending) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ALIGN);
        return;
    }
    ptMotor->bAlignCapturePending = false;
    if (eCapture != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ALIGN);
        return;
    }
    if (FOC_PORT_PWM_SAFE_STOP() != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
        return;
    }
    ptMotor->bElectricalZeroValid = true;
    ptMotor->bPwmEnabled = false;
    ptMotor->eState = MOTOR_STATE_IDLE;
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
