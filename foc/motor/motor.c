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
#include "foc_modulation.h"
#include "perf_counter.h"

#define MOTOR_ADC_OFFSET_SAMPLE_COUNT FOC_OFFSET_CALIB_TIMES

_Static_assert(MOTOR_ADC_OFFSET_SAMPLE_COUNT > 0U &&
               MOTOR_ADC_OFFSET_SAMPLE_COUNT <= UINT16_MAX,
               "ADC offset calibration count must fit the sample counter");

/** @brief Reject zero offsets that cannot represent bipolar current.
 * @param ptCalibration Completed ADC calibration. @return Offset validity. */
static bool _motor_AdcOffsetsValid(const motor_adc_calib_t *ptCalibration)
{
    return ptCalibration->wOffsetU != 0U &&
           ptCalibration->wOffsetV != 0U &&
           ptCalibration->wOffsetW != 0U;
}

/** @brief Sample and range-check the bus voltage in millivolts.
 * @param pwMillivolt Non-null caller-owned physical bus sample.
 * @return Sampling, configuration, or range status. */
static foc_result_t _motor_SampleDcBusMillivolt(uint32_t *pwMillivolt)
{
    int64_t llMillivolt = 0;

#if FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_NOMINAL
    llMillivolt = (int64_t)FOC_DCBUS_NOMINAL_MILLIVOLT;
#elif FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_ADC
#if FOC_DCBUS_MV_PER_COUNT_NUM == 0U || \
    FOC_DCBUS_MV_PER_COUNT_DEN == 0U
    return FOC_RESULT_SAFETY;
#else
    uint32_t wRaw = FOC_PORT_SAMPLE_DCBUS_RAW(
        FOC_PORT_ADC_CHANNEL_DCBUS);
    uint64_t ullScaled = 0U;

    if (wRaw == FOC_PORT_ADC_SAMPLE_INVALID) {
        return FOC_RESULT_SAFETY;
    }
    ullScaled = (uint64_t)wRaw * FOC_DCBUS_MV_PER_COUNT_NUM;
    if (ullScaled > (uint64_t)INT64_MAX) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    llMillivolt = (int64_t)(ullScaled / FOC_DCBUS_MV_PER_COUNT_DEN);
    if (FOC_DCBUS_OFFSET_MILLIVOLT > 0 &&
        llMillivolt > INT64_MAX -
                      (int64_t)FOC_DCBUS_OFFSET_MILLIVOLT) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    llMillivolt += (int64_t)FOC_DCBUS_OFFSET_MILLIVOLT;
#endif
#else
    return FOC_RESULT_DISABLED;
#endif
    if (llMillivolt <= 0 ||
        llMillivolt > (int64_t)FOC_DCBUS_MAX_MILLIVOLT ||
        llMillivolt > (int64_t)UINT32_MAX) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    *pwMillivolt = (uint32_t)llMillivolt;
    return FOC_RESULT_OK;
}

/**
 * @brief Convert one calibrated ADC current sample to FOC per-unit form.
 * @param wRaw Raw ADC sample.
 * @param wOffset Calibrated ADC zero-current offset.
 * @param pqCurrent Signed phase current on the shared PU base.
 * @return Whether the ADC delta is representable without truncation.
 */
static bool _motor_NormalizeCurrent(uint32_t wRaw, uint32_t wOffset,
                                    foc_scalar_t *pqCurrent)
{
    int64_t delta = (int64_t)wOffset - (int64_t)wRaw;

#if !FOC_CURRENT_SAMPLE_INVERTED
    delta = -delta;
#endif
    /* Ib is a scale, not an ADC range. Keep both backends' wide Q15 domain. */
    int64_t scaled = delta * FOC_Q_SCALE;
    if (scaled >= ((int64_t)INT32_MAX + 1) * FOC_CURRENT_COUNTS_PER_BASE ||
        scaled <= (int64_t)INT32_MIN * FOC_CURRENT_COUNTS_PER_BASE ||
        wRaw == UINT32_MAX) {
        return false;
    }
#if defined(FOC_NUMERIC_FIXED)
    *pqCurrent = (foc_scalar_t)(scaled / FOC_CURRENT_COUNTS_PER_BASE);
#elif FOC_CURRENT_COUNTS_PER_BASE <= FOC_Q_SCALE
    /* The checked Q15 range above implies INT32_MIN < delta < INT32_MAX+1.
     * Narrow only after validation; avoid the software int64-to-float call. */
    *pqCurrent = (foc_scalar_t)(int32_t)delta / FOC_CURRENT_COUNTS_PER_BASE;
#else
    *pqCurrent = (foc_scalar_t)delta / FOC_CURRENT_COUNTS_PER_BASE;
#endif
    return true;
}

/**
 * @brief Request safe PWM stop and enter the latched fault state.
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
    ptMotor->tAlign.bCapturePending = false;
    ptMotor->bIdentificationActive = false;
    ptMotor->qIdentificationDcBusPu = FOC_ZERO;
    ptMotor->bPwmEnabled = false;
    ptMotor->tStart.bObserverActive = false;
    motor_startup_Stop(&ptMotor->tStart.tTrajectory);
    ptMotor->wFaults |= (uint32_t)eFault;
    ptMotor->eState = MOTOR_STATE_FAULT;
}

/**
 * @brief Sample ADC, subtract zero offsets and form the pu alpha-beta input.
 * @param ptMotor Calibrated Motor instance.
 * @return true on success; otherwise a sampling/math fault is latched.
 */
static bool _motor_SampleCurrent(motor_t *ptMotor)
{
    foc_current_sample_t tSample = {0};
    foc_current_abc_t tCurrent = {0};

    if (!ptMotor->tCalib.bIsCalibrated ||
        FOC_PORT_SAMPLE_CURRENT(&tSample) != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_SAMPLE);
        return false;
    }
    ptMotor->tCalib.tLatestSample = tSample;
    if (!_motor_NormalizeCurrent(tSample.wU, ptMotor->tCalib.wOffsetU,
                                  &tCurrent.qU) ||
        !_motor_NormalizeCurrent(tSample.wV, ptMotor->tCalib.wOffsetV,
                                  &tCurrent.qV) ||
        !_motor_NormalizeCurrent(tSample.wW, ptMotor->tCalib.wOffsetW,
                                  &tCurrent.qW)) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ADC_SAMPLE);
        return false;
    }
    if (foc_clarke(tCurrent.qU, tCurrent.qV, tCurrent.qW,
                  &ptMotor->tInput.tCurrentAlphaBeta) != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return false;
    }
    return true;
}

/**
 * @brief Validate Motor's physical metadata, pu limits and control rates.
 * @param ptConfig Non-null configuration.
 * @return true when Motor's initialization contract is satisfied.
 */
static bool _motor_ConfigValid(const motor_cfg_t *ptConfig)
{
    if (ptConfig == NULL) {
        return false;
    }
    return ptConfig->tParams.chPolePairs != 0U &&
           ptConfig->tParams.wResistanceMilliohm != 0U &&
           ptConfig->tParams.wInductanceDMicroHenry != 0U &&
           ptConfig->tParams.wInductanceQMicroHenry != 0U &&
           ptConfig->qElectricalSpeedBaseHz > FOC_ZERO &&
           ptConfig->tLimits.qMaxSpeedReference > FOC_ZERO &&
           ptConfig->tLimits.qMaxSpeedReference <= FOC_ONE &&
           ptConfig->tLimits.qMaxPhaseCurrent > FOC_ZERO &&
           ptConfig->tLimits.qMaxPhaseCurrent <= FOC_ONE &&
           foc_scalar_is_finite(ptConfig->tLimits.qMaxIq) &&
           ptConfig->tLimits.qMaxIq >= FOC_ZERO &&
           ptConfig->tLimits.qMaxModulation > FOC_ZERO &&
           ptConfig->tLimits.qMaxModulation <= FOC_ONE &&
           isfinite(ptConfig->fAdcCalibrationTimeoutSeconds) &&
           ptConfig->fAdcCalibrationTimeoutSeconds > 0.0f &&
           isfinite(ptConfig->fAlignTimeSeconds) &&
           ptConfig->fAlignTimeSeconds > 0.0f &&
           ptConfig->wControlFrequencyHz != 0U &&
           ptConfig->wControlFrequencyHz <=
               FOC_NANOSECONDS_PER_SECOND &&
           ptConfig->wSpeedLoopFrequencyHz != 0U &&
           ptConfig->qAlignCurrent > FOC_ZERO &&
           ptConfig->qAlignCurrent <= ptConfig->tLimits.qMaxPhaseCurrent;
}

/** @brief Derive an ISR deadline once, rounding up to a complete tick.
 * @param fSeconds Positive duration. @param wFrequencyHz ISR frequency.
 * @param pwSteps Derived deadline. @return Whether conversion succeeded. */
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

/**
 * @brief Start a control phase with safe duty, or latch a PWM fault.
 * @param ptMotor Motor object.
 * @param eNextState RUNNING or ALIGN after PWM enable succeeds.
 * @return FOC_RESULT_OK or a hardware error.
 */
static foc_result_t _motor_EnablePwm(motor_t *ptMotor,
                                      motor_state_e eNextState)
{
    foc_result_t eResult = FOC_RESULT_OK;

    foc_core_Reset(&ptMotor->tCore, &ptMotor->tOutput);
    ptMotor->bControlPrepared = false;
    ptMotor->tAlign.bCapturePending = false;
    eResult = FOC_PORT_SET_DUTY(&ptMotor->tOutput.tDuty);
    if (eResult == FOC_RESULT_OK) {
        eResult = FOC_PORT_PWM_ENABLE();
    }
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
        return eResult;
    }
    ptMotor->bPwmEnabled = true;
    ptMotor->wRunGeneration++;
    ptMotor->eState = eNextState;
    return FOC_RESULT_OK;
}

/**
 * @brief Execute Core and commit duty, latching either failure locally.
 * @param ptMotor Motor with a complete current/angle input.
 * @return true when this control interval was submitted successfully.
 */
static bool _motor_ApplyControl(motor_t *ptMotor)
{
    if (foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                      &ptMotor->tInput, &ptMotor->tOutput) != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return false;
    }
    if (FOC_PORT_SET_DUTY(&ptMotor->tOutput.tDuty) != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
        return false;
    }
    return true;
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

    if (ptMotor->tCalib.wElapsedSteps < UINT32_MAX) {
        ptMotor->tCalib.wElapsedSteps++;
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
    } else if (ptMotor->tCalib.wElapsedSteps >=
                   ptMotor->tCalib.wTimeoutSteps) {
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
    foc_scalar_t qDifference = FOC_ZERO;

    if (ptMotor->tCommand.eMode != FOC_MODE_SPEED) {
        return;
    }
    ptMotor->tSpeed.chCount++;
    if (ptMotor->tSpeed.chCount < ptMotor->tSpeed.chDiv) {
        return;
    }
    ptMotor->tSpeed.chCount = 0U;
    if (ptMotor->tStart.bSensorless) {
        qDifference = foc_sub_sat(
            ptMotor->tStart.qTargetSpeedPu,
            ptMotor->tCommand.qSpeedReferencePu);
        qDifference = foc_sat(qDifference,
            FOC_ZERO - ptMotor->tStart.qClosedLoopSpeedStepPu,
            ptMotor->tStart.qClosedLoopSpeedStepPu);
        ptMotor->tCommand.qSpeedReferencePu = foc_add_sat(
            ptMotor->tCommand.qSpeedReferencePu, qDifference);
    }
    ptMotor->tCommand.tCurrentReference.qQ = foc_pid_Step(
        &ptMotor->tSpeed.tPi,
        ptMotor->tCommand.qSpeedReferencePu,
        ptMotor->tInput.qElectricalSpeedPu);
}

/** @brief Prepare current and prior-interval voltage for position feedback.
 * @param ptMotor Running Motor. @param ptSample Position-source input.
 * @return Whether the paired control step may execute. */
static motor_isr_phase_t _motor_PrepareRun(
    motor_t *ptMotor, motor_position_sample_t *ptSample)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor->bControlPrepared) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return MOTOR_ISR_NO_CONTROL;
    }
    if (ptMotor->tStart.bSensorless && !ptMotor->tStart.bObserverActive) {
        if (ptMotor->tStart.wRunSteps >=
            ptMotor->tStart.wMaxRunSteps) {
            _motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
            return MOTOR_ISR_NO_CONTROL;
        }
        ptMotor->tStart.wRunSteps++;
    }
    if (!_motor_SampleCurrent(ptMotor)) {
        return MOTOR_ISR_NO_CONTROL;
    }
    ptSample->tCurrentAlphaBeta = ptMotor->tInput.tCurrentAlphaBeta;
    foc_scalar_t bus = FOC_ZERO;
    bool busValid = false;
    if (ptMotor->bIdentificationActive) {
        /* Fixed-supply R/L uses its startup snapshot, not ADC readiness. */
        bus = ptMotor->qIdentificationDcBusPu;
        busValid = foc_scalar_is_finite(bus) && bus > FOC_ZERO;
    } else {
        busValid = motor_SampleDcBusPuIsr(ptMotor, &bus) == FOC_RESULT_OK;
    }
    ptSample->bVoltageModelValid = busValid &&
        foc_modulation_ModelVoltageAlphaBeta(&ptMotor->tOutput.tDuty, bus,
            &ptSample->tVoltageModelAlphaBeta) == FOC_RESULT_OK;
    ptSample->bPwmSaturated = ptMotor->tOutput.bPwmSaturated;
    ptSample->bControlLimited = ptMotor->tOutput.bVoltageLimited ||
        ptMotor->tOutput.bPwmSaturated ||
        foc_abs(ptMotor->tCommand.tCurrentReference.qD) >=
            ptMotor->tLimits.qMaxPhaseCurrent ||
        foc_abs(ptMotor->tCommand.tCurrentReference.qQ) >=
            ptMotor->tSpeed.qIqLimit;
    if (ptMotor->bIdentificationActive || ptMotor->tStart.bObserverActive) {
        ptSample->tHardDragCandidate = (motor_electrical_feedback_t){0};
    } else if (ptMotor->tStart.bSensorless) {
        eResult = motor_startup_IsrStep(
            &ptMotor->tStart.tTrajectory, &ptSample->tHardDragCandidate);
        if (eResult != FOC_RESULT_OK) {
            _motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
            return MOTOR_ISR_NO_CONTROL;
        }
    } else {
        ptSample->tHardDragCandidate = (motor_electrical_feedback_t){
            .tElectricalAngle = ptMotor->tStart.tHardDragAngle,
            .qElectricalSpeedPu = ptMotor->tStart.qHardDragSpeedPu,
            .bValid = ptMotor->tStart.wHardDragStepBam32 != 0U,
        };
        ptMotor->tStart.tHardDragAngle.wBam32 +=
            ptMotor->tStart.wHardDragStepBam32;
    }
    ptSample->wRunGeneration = ptMotor->wRunGeneration;
    ptMotor->bControlPrepared = true;
    return ptMotor->bIdentificationActive ? MOTOR_ISR_IDENTIFICATION_READY :
                                            MOTOR_ISR_CONTROL_READY;
}

/**
 * @brief Run the fixed-angle ALIGN current loop.
 * @param ptMotor Motor object.
 * @return Capture request after the configured hold period.
 */
static motor_isr_phase_t _motor_AlignStep(motor_t *ptMotor)
{

    if (ptMotor->tAlign.bCapturePending) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ALIGN);
        return MOTOR_ISR_NO_CONTROL;
    }
    if (!_motor_SampleCurrent(ptMotor)) {
        return MOTOR_ISR_NO_CONTROL;
    }
    /* Fixed stator vector: no encoder or observer feedback is required. */
    ptMotor->tInput.tElectricalAngle = (foc_angle_t){0U};
    ptMotor->tInput.qElectricalSpeedPu = FOC_ZERO;
    ptMotor->tInput.bAngleValid = true;
    if (!_motor_ApplyControl(ptMotor)) {
        return MOTOR_ISR_NO_CONTROL;
    }
    if (ptMotor->tAlign.wStepCount < UINT32_MAX) {
        ptMotor->tAlign.wStepCount++;
    }
    if (ptMotor->tAlign.wStepCount < ptMotor->tAlign.wTargetSteps) {
        return MOTOR_ISR_NO_CONTROL;
    }
    if (ptMotor->tAlign.eCompletion == MOTOR_ALIGN_START_IDENTIFICATION) {
        /* Keep PWM enabled; alpha/d and beta/q coincide at angle zero. */
        ptMotor->tCommand = (foc_core_command_t){.eMode = FOC_MODE_VOLTAGE};
        ptMotor->eState = MOTOR_STATE_RUNNING;
        return MOTOR_ISR_NO_CONTROL;
    }
    if (ptMotor->tAlign.eCompletion == MOTOR_ALIGN_START_RUN) {
        ptMotor->tCommand.tCurrentReference = (foc_dq_t){
            FOC_ZERO, ptMotor->tStart.qIqPu};
        ptMotor->eState = MOTOR_STATE_RUNNING;
        return MOTOR_ISR_NO_CONTROL;
    }
    if (ptMotor->tAlign.eCompletion == MOTOR_ALIGN_STOP) {
        motor_Stop(ptMotor);
        return MOTOR_ISR_NO_CONTROL;
    }
    ptMotor->tAlign.bCapturePending = true;
    return MOTOR_ISR_CAPTURE_ZERO;
}

/**
 * @brief Bind startup trajectory and derive the optional forced-angle step.
 * @param ptMotor Cleared Motor instance.
 * @param ptConfig Validated configuration; no pointer is retained.
 * @return Configuration result.
 * @note Physical frequency and wide angle arithmetic stay outside the ISR.
 */
static foc_result_t _motor_ConfigureStartup(motor_t *ptMotor,
                                           const motor_cfg_t *ptConfig)
{
    if (ptConfig->wStartupRampSteps != 0U) {
        motor_startup_cfg_t tStartupCfg = {
            .wControlFrequencyHz = ptConfig->wControlFrequencyHz,
            .fElectricalBaseHz = foc_to_float(
                ptConfig->qElectricalSpeedBaseHz),
            .wRampSteps = ptConfig->wStartupRampSteps,
        };

        if (ptConfig->wStartupMaxRunSteps <=
                ptConfig->wStartupRampSteps ||
            !foc_scalar_is_finite(ptConfig->qStartupIqPu) ||
            ptConfig->qStartupIqPu <= FOC_ZERO ||
            ptConfig->qStartupIqPu >
                ptConfig->tLimits.qMaxPhaseCurrent ||
            (ptConfig->tLimits.qMaxIq > FOC_ZERO &&
             ptConfig->qStartupIqPu >
                 ptConfig->tLimits.qMaxIq) ||
            ptConfig->qStartupIqPu >
                ptConfig->tSpeedPiParams.qOutputMaximum ||
            ptConfig->qStartupIqPu <
                ptConfig->tSpeedPiParams.qOutputMinimum ||
            !foc_scalar_is_finite(
                ptConfig->qStartupClosedLoopSpeedStepPu) ||
            ptConfig->qStartupClosedLoopSpeedStepPu >
                FOC_SCALAR(0.01f) ||
            motor_startup_Init(&ptMotor->tStart.tTrajectory,
                               &tStartupCfg) != FOC_RESULT_OK) {
            return FOC_RESULT_INVALID_ARGUMENT;
        }
    }
    if (ptConfig->nHardDragElectricalMilliHz != 0) {
        int64_t llFrequency = ptConfig->nHardDragElectricalMilliHz;
        uint64_t ullStepsPerSecondMilli = 0U;
        uint64_t ullStep = 0U;
        float fSpeedPu = 0.0f;

        fSpeedPu = ((float)llFrequency / 1000.0f) /
                   foc_to_float(ptConfig->qElectricalSpeedBaseHz);
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
        ptMotor->tStart.qHardDragSpeedPu = foc_from_float(fSpeedPu);
        if (ullStep == 0U || ullStep >= 0x80000000ULL ||
            ptMotor->tStart.qHardDragSpeedPu == FOC_ZERO) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        ptMotor->tStart.wHardDragStepBam32 =
            ptConfig->nHardDragElectricalMilliHz < 0
                ? 0U - (uint32_t)ullStep : (uint32_t)ullStep;
    }
    return FOC_RESULT_OK;
}

/** @brief Bind speed PI to the current range actually available to Motor.
 * @param ptMotor Motor with effective Iq limit.
 * @param ptParams User's PI configuration.
 * @return Initialization status; an empty output intersection is invalid. */
static foc_result_t _motor_ConfigureSpeedPi(motor_t *ptMotor,
    const foc_pid_params_t *ptParams)
{
    foc_pid_params_t tParams = *ptParams;
    const foc_scalar_t qLimit = ptMotor->tSpeed.qIqLimit;

    if (!foc_scalar_is_finite(tParams.qOutputMinimum) ||
        !foc_scalar_is_finite(tParams.qOutputMaximum)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (tParams.qOutputMinimum < FOC_ZERO - qLimit) {
        tParams.qOutputMinimum = FOC_ZERO - qLimit;
    }
    if (tParams.qOutputMaximum > qLimit) {
        tParams.qOutputMaximum = qLimit;
    }
    return foc_pid_Init(&ptMotor->tSpeed.tPi, &tParams);
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
    /* Bind immutable metadata, pu limits and per-phase runtime groups. */
    *ptMotor = (motor_t){0};
    ptMotor->tParams = ptConfig->tParams;
    ptMotor->tParams.wCurrentBaseMilliamp =
        FOC_CURRENT_BASE_MILLIAMP;
    ptMotor->tLimits = ptConfig->tLimits;
    ptMotor->tSpeed.qIqLimit = ptConfig->tLimits.qMaxPhaseCurrent;
    if (ptConfig->tLimits.qMaxIq > FOC_ZERO &&
        ptConfig->tLimits.qMaxIq < ptMotor->tSpeed.qIqLimit) {
        ptMotor->tSpeed.qIqLimit = ptConfig->tLimits.qMaxIq;
    }
    ptMotor->tAlign.qCurrentPu = ptConfig->qAlignCurrent;
    ptMotor->tStart.qIqPu = ptConfig->qStartupIqPu;
    ptMotor->tStart.wMaxRunSteps = ptConfig->wStartupMaxRunSteps;
    ptMotor->tStart.qClosedLoopSpeedStepPu =
        ptConfig->qStartupClosedLoopSpeedStepPu > FOC_ZERO ?
        ptConfig->qStartupClosedLoopSpeedStepPu :
        FOC_SCALAR(0.001f);
    ptMotor->tAlign.wTargetSteps = wAlignTargetSteps;
    ptMotor->tSpeed.chDiv = (uint8_t)wSpeedLoopDiv;
    eResult = _motor_ConfigureStartup(ptMotor, ptConfig);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    /* Stop the power stage before initializing control state. */
    eResult = FOC_PORT_PWM_SAFE_STOP();
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    ptMotor->eState = MOTOR_STATE_INITIALIZING;
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    eResult = foc_core_Init(&ptMotor->tCore, &ptConfig->tCurrentPiParams,
                            ptMotor->tLimits.qMaxModulation);
    if (eResult == FOC_RESULT_OK) {
        eResult = _motor_ConfigureSpeedPi(ptMotor,
                                         &ptConfig->tSpeedPiParams);
    }
    if (eResult != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return eResult;
    }
    foc_core_Reset(&ptMotor->tCore, &ptMotor->tOutput);
    ptMotor->tCalib = (motor_adc_calib_t){
        .wTimeoutSteps = wAdcCalibrationTimeoutSteps};
    /* Publish ADC_CAL only after all control state is ready. */
    ptMotor->eState = MOTOR_STATE_ADC_CAL;
    FOC_PORT_START_ADC_TRIGGER();
    return FOC_RESULT_OK;
}

/** @brief Sample and normalize the DC bus on Motor's voltage base.
 * @param ptMotor Initialized Motor with a nonzero voltage base.
 * @param pqBusPu Output DC-bus voltage in pu.
 * @return Sampling, normalization, or input status.
 * @note ISR-safe and nonblocking; also used at protected session startup.
 *       Motor owns scaling checks and physical-to-pu normalization. */
foc_result_t motor_SampleDcBusPuIsr(const motor_t *ptMotor,
                                   foc_scalar_t *pqBusPu)
{
    uint32_t wBusMillivolt = 0U;
    uint32_t wVoltageBaseMillivolt = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL || pqBusPu == NULL) {
        return FOC_RESULT_NULL;
    }
    *pqBusPu = FOC_ZERO;
    wVoltageBaseMillivolt = ptMotor->tParams.wVoltageBaseMillivolt;
    if (wVoltageBaseMillivolt == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = _motor_SampleDcBusMillivolt(&wBusMillivolt);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
#if defined(FOC_NUMERIC_FLOAT)
    *pqBusPu = (foc_scalar_t)wBusMillivolt /
               (foc_scalar_t)wVoltageBaseMillivolt;
#else
    {
        int64_t llBusPu = (int64_t)wBusMillivolt * FOC_Q_SCALE /
                          (int64_t)wVoltageBaseMillivolt;
        if (llBusPu > INT32_MAX) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        *pqBusPu = (foc_scalar_t)llBusPu;
    }
#endif
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
    ptMotor->tStart.tHardDragAngle = (foc_angle_t){0U};
    ptMotor->tStart.bSensorless = false;
    ptMotor->tStart.bObserverActive = false;
    ptMotor->bIdentificationActive = false;
    foc_pid_Reset(&ptMotor->tSpeed.tPi);
    eResult = _motor_EnablePwm(ptMotor, MOTOR_STATE_RUNNING);
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

foc_result_t motor_StartSensorlessSpeed(motor_t *ptMotor,
                                        foc_scalar_t qTargetSpeedPu)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_scalar_is_finite(qTargetSpeedPu) ||
        qTargetSpeedPu <= FOC_ZERO ||
        qTargetSpeedPu > FOC_SCALAR(0.8f) ||
        qTargetSpeedPu > ptMotor->tLimits.qMaxSpeedReference ||
        !motor_startup_IsConfigured(&ptMotor->tStart.tTrajectory)) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
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
    eResult = motor_startup_Start(&ptMotor->tStart.tTrajectory,
                                  qTargetSpeedPu);
    if (eResult != FOC_RESULT_OK) {
        perfc_port_resume_global_interrupt(tIrqState);
        return eResult;
    }
    ptMotor->tCommand = (foc_core_command_t){0};
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    ptMotor->tCommand.tCurrentReference.qD = ptMotor->tAlign.qCurrentPu;
    ptMotor->tStart.qTargetSpeedPu = qTargetSpeedPu;
    ptMotor->tStart.wRunSteps = 0U;
    ptMotor->tAlign.wStepCount = 0U;
    ptMotor->tAlign.eCompletion = MOTOR_ALIGN_START_RUN;
    ptMotor->bElectricalZeroValid = false;
    ptMotor->tStart.bSensorless = true;
    ptMotor->tStart.bObserverActive = false;
    ptMotor->bIdentificationActive = false;
    foc_pid_Reset(&ptMotor->tSpeed.tPi);
    eResult = _motor_EnablePwm(ptMotor, MOTOR_STATE_ALIGN);
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

/** @brief Start one fixed-axis alignment/injection PWM session.
 * @param ptMotor Idle Motor. @return Validated start or hardware failure. */
foc_result_t motor_StartIdentification(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t irq = 0U;
    foc_result_t result = FOC_RESULT_OK;
    foc_scalar_t bus = FOC_ZERO;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (FOC_PORT_PWM_GET_FAULT()) {
        return FOC_RESULT_SAFETY;
    }
    irq = perfc_port_disable_global_interrupt();
    if (ptMotor->eState != MOTOR_STATE_IDLE ||
        ptMotor->wFaults != MOTOR_FAULT_NONE ||
        !ptMotor->tCalib.bIsCalibrated ||
        ptMotor->tAlign.qCurrentPu > ptMotor->tLimits.qMaxPhaseCurrent) {
        perfc_port_resume_global_interrupt(irq);
        return FOC_RESULT_SAFETY;
    }
    result = motor_SampleDcBusPuIsr(ptMotor, &bus);
    if (result != FOC_RESULT_OK || bus <= FOC_ZERO) {
        perfc_port_resume_global_interrupt(irq);
        return result == FOC_RESULT_OK ? FOC_RESULT_OUT_OF_RANGE : result;
    }
    ptMotor->qIdentificationDcBusPu = bus;
    ptMotor->tCommand = (foc_core_command_t){.eMode = FOC_MODE_CURRENT};
    ptMotor->tCommand.tCurrentReference.qD = ptMotor->tAlign.qCurrentPu;
    ptMotor->tAlign.wStepCount = 0U;
    ptMotor->tAlign.eCompletion = MOTOR_ALIGN_START_IDENTIFICATION;
    ptMotor->bElectricalZeroValid = false;
    ptMotor->bIdentificationActive = true;
    ptMotor->tStart.bSensorless = false;
    ptMotor->tStart.bObserverActive = false;
    motor_startup_Stop(&ptMotor->tStart.tTrajectory);
    result = _motor_EnablePwm(ptMotor, MOTOR_STATE_ALIGN);
    perfc_port_resume_global_interrupt(irq);
    return result;
}

void motor_Stop(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (FOC_PORT_PWM_SAFE_STOP() != FOC_RESULT_OK) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
    }
    ptMotor->tCommand.tVoltageReference =
        (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptMotor->bPwmEnabled = false;
    motor_startup_Stop(&ptMotor->tStart.tTrajectory);
    ptMotor->tStart.bSensorless = false;
    ptMotor->tStart.bObserverActive = false;
    ptMotor->bIdentificationActive = false;
    ptMotor->qIdentificationDcBusPu = FOC_ZERO;
    foc_pid_Reset(&ptMotor->tSpeed.tPi);
    ptMotor->bControlPrepared = false;
    ptMotor->tAlign.bCapturePending = false;
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
    foc_core_Reset(&ptMotor->tCore, &ptMotor->tOutput);
    ptMotor->bControlPrepared = false;
    ptMotor->tAlign.bCapturePending = false;
    foc_pid_Reset(&ptMotor->tSpeed.tPi);
    if (bAdcCalibrationFault || !ptMotor->tCalib.bIsCalibrated) {
        ptMotor->tCalib = (motor_adc_calib_t){
            .wTimeoutSteps = ptMotor->tCalib.wTimeoutSteps};
        ptMotor->eState = MOTOR_STATE_ADC_CAL;
        /* Restart ADC sampling after break/stop; PWM output remains off. */
        FOC_PORT_START_ADC_TRIGGER();
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
    if (ptMotor->bIdentificationActive) {
        return FOC_RESULT_BUSY;
    }
    return _motor_SetDqReference(ptMotor, FOC_MODE_VOLTAGE,
                                &ptMotor->tCommand.tVoltageReference,
                                qD, qQ);
}

/**
 * @brief Submit a prevalidated identification voltage from the FOC ISR.
 * @param ptMotor Running voltage-mode Motor.
 * @param ptVoltageCommand Fixed alpha-axis modulation ratios.
 * @return FOC_RESULT_OK or a state/safety error.
 * @note App calls this after motor_IsrControlStep(), so the command takes
 *       effect on the following PWM update.
 */
foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor,
    const foc_ab_t *ptVoltageCommand)
{
    if (ptMotor == NULL || ptVoltageCommand == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->eState != MOTOR_STATE_RUNNING || !ptMotor->bPwmEnabled ||
        !ptMotor->bIdentificationActive ||
        ptMotor->tCommand.eMode != FOC_MODE_VOLTAGE) {
        return FOC_RESULT_SAFETY;
    }
    if (!foc_scalar_is_finite(ptVoltageCommand->qAlpha) ||
        !foc_scalar_is_finite(ptVoltageCommand->qBeta) ||
        ptVoltageCommand->qBeta != FOC_ZERO ||
        ptVoltageCommand->qAlpha > ptMotor->tLimits.qMaxModulation ||
        ptVoltageCommand->qAlpha < -ptMotor->tLimits.qMaxModulation) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptMotor->tCommand.tVoltageReference = (foc_dq_t){
        ptVoltageCommand->qAlpha, ptVoltageCommand->qBeta};
    return FOC_RESULT_OK;
}

/** @brief Atomically update the fixed-axis injection reference.
 * @param ptMotor Active Motor. @param ptCommand Alpha-axis modulation.
 * @return Checked submission status. */
foc_result_t motor_SetIdentificationModulation(motor_t *ptMotor,
    const foc_ab_t *ptCommand)
{
    perfc_global_interrupt_status_t irq =
        perfc_port_disable_global_interrupt();
    foc_result_t result = motor_IdentificationApplyIsr(ptMotor, ptCommand);

    perfc_port_resume_global_interrupt(irq);
    return result;
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
    if (ptMotor->bIdentificationActive) {
        return FOC_RESULT_BUSY;
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
    if (ptMotor->tStart.bSensorless) {
        ptMotor->tStart.qTargetSpeedPu = qSpeedReferencePu;
    } else {
        ptMotor->tCommand.qSpeedReferencePu = qSpeedReferencePu;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_RequestAlignment(motor_t *ptMotor, bool bCaptureZero)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (FOC_PORT_PWM_GET_FAULT()) {
        return FOC_RESULT_SAFETY;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->eState != MOTOR_STATE_IDLE ||
        !ptMotor->tCalib.bIsCalibrated) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommand = (foc_core_command_t){0};
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    ptMotor->tCommand.tCurrentReference.qD = ptMotor->tAlign.qCurrentPu;
    ptMotor->tAlign.wStepCount = 0U;
    ptMotor->tAlign.eCompletion = bCaptureZero ? MOTOR_ALIGN_CAPTURE_ZERO :
                                              MOTOR_ALIGN_STOP;
    ptMotor->bElectricalZeroValid = false;
    eResult = _motor_EnablePwm(ptMotor, MOTOR_STATE_ALIGN);
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

motor_isr_phase_t motor_IsrPrepare(motor_t *ptMotor,
                                   motor_position_sample_t *ptSample)
{
    if (ptSample != NULL) {
        *ptSample = (motor_position_sample_t){0};
        ptSample->bAlignActive = ptMotor != NULL &&
                                ptMotor->eState == MOTOR_STATE_ALIGN;
    }
    if (ptMotor == NULL) {
        return MOTOR_ISR_NO_CONTROL;
    }
    switch (ptMotor->eState) {
    case MOTOR_STATE_INITIALIZING:
        if (FOC_PORT_PWM_SAFE_STOP() != FOC_RESULT_OK) {
            _motor_EnterFault(ptMotor, MOTOR_FAULT_PWM);
        } else {
            ptMotor->eState = MOTOR_STATE_ADC_CAL;
        }
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

foc_result_t motor_IsrControlStep(
    motor_t *ptMotor, const motor_electrical_feedback_t *ptFeedback)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->eState != MOTOR_STATE_RUNNING ||
        !ptMotor->bControlPrepared) {
        if (ptMotor->eState == MOTOR_STATE_RUNNING) {
            _motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        }
        return FOC_RESULT_SAFETY;
    }
    ptMotor->bControlPrepared = false;
    if (ptMotor->bIdentificationActive) {
        ptMotor->tInput.tElectricalAngle = (foc_angle_t){0U};
        ptMotor->tInput.qElectricalSpeedPu = FOC_ZERO;
    } else if (ptFeedback == NULL || !ptFeedback->bValid ||
        !foc_scalar_is_finite(ptFeedback->qElectricalSpeedPu)) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
        return FOC_RESULT_SAFETY;
    } else {
        ptMotor->tInput.tElectricalAngle = ptFeedback->tElectricalAngle;
        ptMotor->tInput.qElectricalSpeedPu = ptFeedback->qElectricalSpeedPu;
    }
    ptMotor->tInput.bAngleValid = true;
    _motor_SpeedLoopStep(ptMotor);
    if (!_motor_ApplyControl(ptMotor)) {
        return FOC_RESULT_SAFETY;
    }
    return FOC_RESULT_OK;
}

void motor_CompleteAlignIsr(motor_t *ptMotor, foc_result_t eCapture)
{
    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->eState != MOTOR_STATE_ALIGN ||
        !ptMotor->tAlign.bCapturePending) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_ALIGN);
        return;
    }
    ptMotor->tAlign.bCapturePending = false;
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

void motor_ApplyPositionEventIsr(motor_t *ptMotor,
    motor_position_event_t eEvent, foc_scalar_t qObservedSpeedPu)
{
    if (ptMotor == NULL || ptMotor->eState != MOTOR_STATE_RUNNING) {
        return;
    }
    if (eEvent == MOTOR_POSITION_EVENT_OBSERVER_LOST) {
        _motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
    } else if (eEvent == MOTOR_POSITION_EVENT_OBSERVER_ACTIVE) {
        if (ptMotor->tStart.bSensorless) {
            if (!foc_scalar_is_finite(qObservedSpeedPu) ||
                qObservedSpeedPu <= FOC_ZERO ||
                qObservedSpeedPu > FOC_ONE) {
                _motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
                return;
            }
            ptMotor->tCommand.qSpeedReferencePu = qObservedSpeedPu;
            foc_pid_Track(&ptMotor->tSpeed.tPi,
                ptMotor->tCommand.tCurrentReference.qQ,
                qObservedSpeedPu, qObservedSpeedPu);
            ptMotor->tSpeed.chCount = 0U;
            ptMotor->tCommand.eMode = FOC_MODE_SPEED;
            motor_startup_Stop(&ptMotor->tStart.tTrajectory);
        }
        ptMotor->tStart.bObserverActive = true;
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
    ptStatus->bIdentificationActive = ptMotor->bIdentificationActive;
    ptStatus->bAngleValid = ptMotor->tInput.bAngleValid;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
