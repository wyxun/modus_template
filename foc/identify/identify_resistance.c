/****************************************************************************
 * @file    identify_resistance.c
 * @brief   Fixed alpha-axis motor resistance identification.
 * @author  Modus project
 * @date    2026-09-22
 ****************************************************************************/

#include "identify_resistance.h"

#include <stddef.h>

#if IDENTIFY_LOG_ACTIVE
#include "mdebug/util_debug.h"
#endif
#include "perf_counter.h"
#include "motor.h"

static void _identify_resistance_SetState(identify_t *ptThis,
                                          identify_resistance_state_t eState)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    tIrqState = perfc_port_disable_global_interrupt();
    /* An ISR failure must remain latched when foreground resumes. */
    if (ptThis->eState != IDENTIFY_STATE_ERROR) {
        ptThis->tResistance.lStateTimestamp = 0;
        ptThis->tResistance.eState = eState;
    }
    perfc_port_resume_global_interrupt(tIrqState);
}

static foc_scalar_t _identify_resistance_VoltageForLevel(uint8_t chLevel)
{
    if (chLevel == 0U) {
        return IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU;
    }
    return IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU;
}

#if IDENTIFY_LOG_ACTIVE
static const char *_identify_resistance_StateName(
    identify_resistance_state_t eState)
{
    switch (eState) {
    case IDENTIFY_RESISTANCE_STATE_APPLY_VOLTAGE: return "apply-voltage";
    case IDENTIFY_RESISTANCE_STATE_WAIT_STABLE: return "settle";
    case IDENTIFY_RESISTANCE_STATE_CAPTURE: return "capture";
    case IDENTIFY_RESISTANCE_STATE_PROCESS_LEVEL: return "process";
    case IDENTIFY_RESISTANCE_STATE_CALCULATE: return "calculate";
    case IDENTIFY_RESISTANCE_STATE_IDLE: return "idle";
    case IDENTIFY_RESISTANCE_STATE_ERROR:
    default: return "unknown";
    }
}
#endif

static void _identify_resistance_ClearCapture(identify_resistance_t *ptResistance)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    tIrqState = perfc_port_disable_global_interrupt();
    ptResistance->hwIsrDivider = 0U;
    ptResistance->hwCaptureSampleCount = 0U;
    ptResistance->qCurrentSum = FOC_ZERO;
    ptResistance->qVoltageSum = FOC_ZERO;
    ptResistance->bBatchReady = false;
    perfc_port_resume_global_interrupt(tIrqState);
}

/** @brief Latch failure and snapshot the last valid ISR pair before stopping.
 * @param ptThis Identification owner. @param ptMotor Session to stop.
 * @param eResult Failure status. @return None.
 * @note Snapshot under interrupt guard; diagnostics print in foreground. */
static void _identify_resistance_Fail(identify_t *ptThis,
                                      motor_t *ptMotor,
                                      foc_result_t eResult)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
#if IDENTIFY_LOG_ACTIVE
    identify_resistance_state_t eFailedState =
        IDENTIFY_RESISTANCE_STATE_IDLE;
    uint8_t chFailedLevel = 0U;
    bool bReportFailure = false;
    foc_scalar_t qFailedCurrent = FOC_ZERO;
    foc_scalar_t qFailedVoltage = FOC_ZERO;
    bool bSampleAvailable = false;
#endif
    tIrqState = perfc_port_disable_global_interrupt();
#if IDENTIFY_LOG_ACTIVE
    bReportFailure = ptThis->eState != IDENTIFY_STATE_ERROR;
    eFailedState = ptThis->tResistance.eState;
    chFailedLevel = ptThis->tResistance.chVoltageLevel;
    qFailedCurrent = ptThis->tResistance.qLatestCurrentAlphaPu;
    qFailedVoltage = ptThis->tResistance.qLatestVoltageModelAlphaPu;
    bSampleAvailable = ptThis->tResistance.bLatestSampleAvailable;
#endif
    ptThis->tResistance.bMotorStarted = false;
    ptThis->tResistance.bBatchReady = false;
    ptThis->tResistance.bResultPending = false;
    ptThis->tResistance.eState = IDENTIFY_RESISTANCE_STATE_ERROR;
    ptThis->eOperation = IDENTIFY_OPERATION_NONE;
    if (ptThis->eState != IDENTIFY_STATE_ERROR) {
        ptThis->eLastResult = eResult;
        ptThis->eState = IDENTIFY_STATE_ERROR;
    }
    perfc_port_resume_global_interrupt(tIrqState);
#if IDENTIFY_LOG_ACTIVE
    if (bReportFailure) {
        MLOGF(W, "identify R %s level=%u failed rc=%d "
              "i_alpha_pu=%.5f min_i_pu=%.5f mod_alpha=%.5f "
              "u_model_alpha_pu=%.5f sample_seen=%u\r\n",
              _identify_resistance_StateName(eFailedState),
              (unsigned)chFailedLevel, (int)eResult,
              (double)foc_to_float(qFailedCurrent),
              (double)foc_to_float(IDENTIFY_RESISTANCE_MIN_CURRENT_PU),
              (double)foc_to_float(
                  _identify_resistance_VoltageForLevel(chFailedLevel)),
              (double)foc_to_float(qFailedVoltage),
              (unsigned)bSampleAvailable);
    }
#endif
    if (ptMotor != NULL) {
        motor_Stop(ptMotor);
    }
}

static identify_failure_cause_t _identify_resistance_FailureCause(
    const identify_resistance_sample_t *ptSample)
{
    bool bFinite = foc_scalar_is_finite(ptSample->qVoltageModelAlphaPu) &&
        foc_scalar_is_finite(ptSample->tCurrentPu.qAlpha) &&
        foc_scalar_is_finite(ptSample->tCurrentPu.qBeta);

    if (!ptSample->bValid || !bFinite) {
        return IDENTIFY_FAILURE_CAUSE_INVALID_SAMPLE;
    }
    if (ptSample->bPwmSaturated) {
        return IDENTIFY_FAILURE_CAUSE_PWM_SATURATED;
    }
    if (foc_add_sat(foc_mul_wide(ptSample->tCurrentPu.qAlpha,
                               ptSample->tCurrentPu.qAlpha),
                    foc_mul_wide(ptSample->tCurrentPu.qBeta,
                               ptSample->tCurrentPu.qBeta)) >
        foc_mul_pu(IDENTIFY_RESISTANCE_MAX_CURRENT_PU,
                   IDENTIFY_RESISTANCE_MAX_CURRENT_PU)) {
        return IDENTIFY_FAILURE_CAUSE_OVERCURRENT;
    }
    return IDENTIFY_FAILURE_CAUSE_NONE;
}

/**
 * @brief Consume a published batch and retain two PU means.
 * @param ptResistance This child's capture state.
 * @return Capture validity status.
 * @note The copy/clear occurs under the foreground interrupt guard.
 */
static foc_result_t _identify_resistance_ProcessLevel(
    identify_resistance_t *ptResistance)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    identify_accumulator_t qCurrentSum = 0;
    identify_accumulator_t qVoltageSum = 0;
    uint16_t hwSampleCount = 0U;
    tIrqState = perfc_port_disable_global_interrupt();
    qCurrentSum = ptResistance->qCurrentSum;
    qVoltageSum = ptResistance->qVoltageSum;
    hwSampleCount = ptResistance->hwCaptureSampleCount;
    ptResistance->qCurrentSum = FOC_ZERO;
    ptResistance->qVoltageSum = FOC_ZERO;
    ptResistance->hwCaptureSampleCount = 0U;
    ptResistance->hwIsrDivider = 0U;
    ptResistance->bBatchReady = false;
    perfc_port_resume_global_interrupt(tIrqState);

    if (hwSampleCount != IDENTIFY_RESISTANCE_SAMPLE_COUNT) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptResistance->aqAverageCurrent[ptResistance->chVoltageLevel] =
        (foc_scalar_t)(qCurrentSum / hwSampleCount);
    ptResistance->aqAverageVoltageAlpha[ptResistance->chVoltageLevel] =
        (foc_scalar_t)(qVoltageSum / hwSampleCount);
    return FOC_RESULT_OK;
}

/** @brief r_pu = delta(u_model_pu) / delta(i_pu).
 * @param ptResistance Two-level means. @param pqResistance Result PU.
 * @return Validated ratio; physical display conversion is a separate API. */
static foc_result_t _identify_resistance_Calculate(
    identify_resistance_t *ptResistance, foc_scalar_t *pqResistance)
{
    foc_scalar_t du = foc_sub_sat(
        ptResistance->aqAverageVoltageAlpha[1U],
        ptResistance->aqAverageVoltageAlpha[0U]);
    foc_scalar_t di = foc_sub_sat(
        ptResistance->aqAverageCurrent[1U],
        ptResistance->aqAverageCurrent[0U]);

    if (foc_abs(di) <= IDENTIFY_RESISTANCE_MIN_CURRENT_PU ||
        du == FOC_ZERO || (du > FOC_ZERO) != (di > FOC_ZERO)) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    return foc_div_checked(du, di, pqResistance);
}

static foc_result_t _identify_resistance_ApplyVoltage(identify_t *ptThis,
                                                       motor_t *ptMotor)
{
    identify_resistance_t *ptResistance = &ptThis->tResistance;
    motor_status_t tStatus = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = motor_GetStatus(ptMotor, &tStatus);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (!tStatus.bIdentificationActive || !tStatus.bPwmEnabled ||
        tStatus.eState != MOTOR_STATE_RUNNING ||
        tStatus.eMode != FOC_MODE_VOLTAGE) {
        return FOC_RESULT_SAFETY;
    }
    foc_ab_t command = {
        _identify_resistance_VoltageForLevel(ptResistance->chVoltageLevel),
        FOC_ZERO};
    eResult = motor_SetIdentificationModulation(ptMotor, &command);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    ptResistance->bMotorStarted = true;
    _identify_resistance_SetState(ptThis,
                                   IDENTIFY_RESISTANCE_STATE_WAIT_STABLE);
    return FOC_RESULT_OK;
}

/**
 * @brief Start the two-level resistance identification flow.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t _identify_resistance_Start(identify_t *ptThis)
{
    identify_resistance_t *ptResistance = NULL;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    ptResistance = &ptThis->tResistance;
    *ptResistance = (identify_resistance_t){0};
    ptResistance->eState = IDENTIFY_RESISTANCE_STATE_APPLY_VOLTAGE;
    {
        perfc_global_interrupt_status_t tIrqState =
            perfc_port_disable_global_interrupt();

        ptThis->eLastResult = FOC_RESULT_OK;
        ptThis->eOperation = IDENTIFY_OPERATION_RESISTANCE;
        ptThis->eState = IDENTIFY_STATE_RUNNING;
        perfc_port_resume_global_interrupt(tIrqState);
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Record one decimated resistance-identification sample.
 * @param ptResistance This child's capture state.
 * @param ptSample Fixed Rs contract with synchronized PU model voltage.
 * @param ptRequest Same-ISR voltage/safety output.
 * @return Failure category; the public ISR adapter latches parent errors.
 */
static identify_failure_cause_t _identify_resistance_CaptureStep(
    identify_resistance_t *ptResistance,
    const identify_resistance_sample_t *ptSample,
    identify_isr_request_t *ptRequest)
{
    if (ptResistance == NULL || ptSample == NULL || ptRequest == NULL) {
        return IDENTIFY_FAILURE_CAUSE_INVALID_SAMPLE;
    }
    if (!ptResistance->bMotorStarted) {
        return IDENTIFY_FAILURE_CAUSE_NONE;
    }
    identify_failure_cause_t cause =
        _identify_resistance_FailureCause(ptSample);
    if (cause != IDENTIFY_FAILURE_CAUSE_NONE) {
        return cause;
    }
    ptResistance->qLatestCurrentAlphaPu = ptSample->tCurrentPu.qAlpha;
#if IDENTIFY_LOG_ACTIVE
    ptResistance->qLatestVoltageModelAlphaPu = ptSample->qVoltageModelAlphaPu;
    ptResistance->bLatestSampleAvailable = true;
#endif
    if (ptResistance->eState != IDENTIFY_RESISTANCE_STATE_CAPTURE ||
        ptResistance->bBatchReady) {
        return IDENTIFY_FAILURE_CAUSE_NONE;
    }
    ptResistance->hwIsrDivider++;
    if (ptResistance->hwIsrDivider < IDENTIFY_RESISTANCE_ISR_PER_SAMPLE) {
        return IDENTIFY_FAILURE_CAUSE_NONE;
    }
    ptResistance->hwIsrDivider = 0U;
    ptResistance->qCurrentSum += ptSample->tCurrentPu.qAlpha;
    ptResistance->qVoltageSum += ptSample->qVoltageModelAlphaPu;
    ptResistance->hwCaptureSampleCount++;
    if (ptResistance->hwCaptureSampleCount >=
        IDENTIFY_RESISTANCE_SAMPLE_COUNT) {
        ptResistance->bBatchReady = true;
    }
    return IDENTIFY_FAILURE_CAUSE_NONE;
}

/** @brief Dispatch Rs capture and latch only the parent's lifecycle failure.
 * @param ptThis Scheduler. @param ptSample Rs sample. @param ptRequest Request.
 * @return None. */
void identify_ResistanceIsrStep(identify_t *ptThis,
    const identify_resistance_sample_t *ptSample,
    identify_isr_request_t *ptRequest)
{
    if (ptRequest == NULL) {
        return;
    }
    *ptRequest = (identify_isr_request_t){0};
    if (ptThis != NULL &&
        ptThis->eOperation != IDENTIFY_OPERATION_RESISTANCE) {
        return;
    }
    identify_failure_cause_t cause = _identify_resistance_CaptureStep(
        ptThis == NULL ? NULL : &ptThis->tResistance, ptSample, ptRequest);
    if (cause != IDENTIFY_FAILURE_CAUSE_NONE) {
        identify_AbortIsr(ptThis, cause, FOC_RESULT_SAFETY);
        ptRequest->bAbort = true;
    }
}

static bool _identify_resistance_IsMotorReady(const motor_t *ptMotor,
                                              foc_result_t *peResult)
{
    motor_status_t tStatus = {0};

    *peResult = motor_GetStatus(ptMotor, &tStatus);
    if (*peResult != FOC_RESULT_OK) {
        return false;
    }
    if (tStatus.eState != MOTOR_STATE_RUNNING || !tStatus.bPwmEnabled ||
        !tStatus.bIdentificationActive || tStatus.eMode != FOC_MODE_VOLTAGE ||
        tStatus.wFaults != MOTOR_FAULT_NONE) {
        *peResult = FOC_RESULT_SAFETY;
        return false;
    }
    return true;
}

/** @brief Calculate, publish and display one completed resistance PU.
 * @param ptThis Identification object. @param ptMotor Motor to stop.
 * @return Measurement/publication status; ISR failures remain latched. */
static foc_result_t _identify_resistance_Complete(identify_t *ptThis,
                                                  motor_t *ptMotor)
{
    identify_resistance_t *ptResistance = &ptThis->tResistance;
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = _identify_resistance_Calculate(
        ptResistance, &ptResistance->qResistancePu);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    {
        perfc_global_interrupt_status_t tIrqState =
            perfc_port_disable_global_interrupt();

        if (ptThis->eState == IDENTIFY_STATE_ERROR) {
            perfc_port_resume_global_interrupt(tIrqState);
            return ptThis->eLastResult;
        }
        ptThis->qResistancePu = ptResistance->qResistancePu;
        ptResistance->bMotorStarted = false;
        ptResistance->bResultPending = true;
        ptResistance->eState = IDENTIFY_RESISTANCE_STATE_IDLE;
        ptThis->eLastResult = FOC_RESULT_OK;
        ptThis->eOperation = IDENTIFY_OPERATION_NONE;
        ptThis->eState = IDENTIFY_STATE_IDLE;
        perfc_port_resume_global_interrupt(tIrqState);
    }
    motor_Stop(ptMotor);
    return FOC_RESULT_OK;
}

/**
 * @brief Advance the resistance-identification child PT.
 * @param ptThis Identification object owning the child PT.
 * @param ptMotor Motor controlled by identification.
 * @return PT completion, progress, or error status.
 */
fsm_rt_t _identify_resistance_RunPt(identify_t *ptThis,
                                     motor_t *ptMotor)
{
    identify_resistance_t *ptResistance = &ptThis->tResistance;
    foc_result_t eResult = FOC_RESULT_OK;

    PERFC_PT_BEGIN(ptResistance->chRunPt)
    while (ptResistance->chVoltageLevel <
           IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT) {
        eResult = _identify_resistance_ApplyVoltage(ptThis, ptMotor);
        if (eResult != FOC_RESULT_OK) {
            _identify_resistance_Fail(ptThis, ptMotor, eResult);
            return fsm_rt_err;
        }
        PERFC_PT_WAIT_UNTIL(
            eResult != FOC_RESULT_OK ||
            !_identify_resistance_IsMotorReady(ptMotor, &eResult) ||
            perfc_is_time_out_ms(IDENTIFY_RESISTANCE_SETTLE_TIME_MS,
                                  &ptResistance->lStateTimestamp, false))
        if (eResult != FOC_RESULT_OK) {
            _identify_resistance_Fail(ptThis, ptMotor, eResult);
            return fsm_rt_err;
        }
        /* Start the current-stability timeout after the settle delay. */
        ptResistance->lStateTimestamp = 0;
        PERFC_PT_WAIT_UNTIL(
            eResult != FOC_RESULT_OK ||
            !_identify_resistance_IsMotorReady(ptMotor, &eResult) ||
            foc_abs(ptResistance->qLatestCurrentAlphaPu) >=
                IDENTIFY_RESISTANCE_MIN_CURRENT_PU ||
            perfc_is_time_out_ms(IDENTIFY_RESISTANCE_SETTLE_TIMEOUT_MS,
                                  &ptResistance->lStateTimestamp, false))
        if (eResult != FOC_RESULT_OK) {
            _identify_resistance_Fail(ptThis, ptMotor, eResult);
            return fsm_rt_err;
        }
        if (foc_abs(ptResistance->qLatestCurrentAlphaPu) <
            IDENTIFY_RESISTANCE_MIN_CURRENT_PU) {
            _identify_resistance_Fail(ptThis, ptMotor,
                                      FOC_RESULT_OUT_OF_RANGE);
            return fsm_rt_err;
        }
        _identify_resistance_ClearCapture(&ptThis->tResistance);
        _identify_resistance_SetState(
            ptThis, IDENTIFY_RESISTANCE_STATE_CAPTURE);
        PERFC_PT_WAIT_UNTIL(
            eResult != FOC_RESULT_OK ||
            !_identify_resistance_IsMotorReady(ptMotor, &eResult) ||
            ptResistance->bBatchReady ||
            perfc_is_time_out_ms(IDENTIFY_RESISTANCE_CAPTURE_TIMEOUT_MS,
                                  &ptResistance->lStateTimestamp, false))
        if (eResult != FOC_RESULT_OK) {
            _identify_resistance_Fail(ptThis, ptMotor, eResult);
            return fsm_rt_err;
        }
        if (!ptResistance->bBatchReady) {
            _identify_resistance_Fail(ptThis, ptMotor, FOC_RESULT_SAFETY);
            return fsm_rt_err;
        }
        _identify_resistance_SetState(
            ptThis, IDENTIFY_RESISTANCE_STATE_PROCESS_LEVEL);
        eResult = _identify_resistance_ProcessLevel(&ptThis->tResistance);
        if (eResult != FOC_RESULT_OK) {
            _identify_resistance_Fail(ptThis, ptMotor, eResult);
            return fsm_rt_err;
        }
        ptResistance->chVoltageLevel++;
    }
    _identify_resistance_SetState(
        ptThis, IDENTIFY_RESISTANCE_STATE_CALCULATE);
    eResult = _identify_resistance_Complete(ptThis, ptMotor);
    if (eResult != FOC_RESULT_OK) {
        _identify_resistance_Fail(ptThis, ptMotor, eResult);
        return fsm_rt_err;
    }
    PERFC_PT_RETURN(fsm_rt_cpl)
    PERFC_PT_END()
    return fsm_rt_on_going;
}

/**
 * @brief Cancel the resistance child after parent dispatch is disabled.
 * @param ptThis Identification object that owns the resistance child.
 * @return None.
 */
void _identify_resistance_Stop(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return;
    }
    ptThis->tResistance.chRunPt = 0U;
    ptThis->tResistance.bBatchReady = false;
    ptThis->tResistance.bResultPending = false;
    ptThis->tResistance.bMotorStarted = false;
    ptThis->tResistance.lStateTimestamp = 0;
    ptThis->tResistance.eState = IDENTIFY_RESISTANCE_STATE_IDLE;
}

/**
 * @brief Clear resistance child state after Motor has safely stopped.
 * @param ptThis Identification object that owns the resistance child.
 * @return None.
 */
void _identify_resistance_Reset(identify_t *ptThis)
{
    identify_resistance_t *ptResistance = NULL;

    if (ptThis == NULL) {
        return;
    }
    ptResistance = &ptThis->tResistance;
    *ptResistance = (identify_resistance_t){0};
    ptResistance->eState = IDENTIFY_RESISTANCE_STATE_IDLE;
}

/**
 * @brief Copy and consume a completed resistance result.
 * @param ptThis Identification object.
 * @param ptResult Output resistance measurement.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or an argument error.
 */
foc_result_t _identify_resistance_TakeResult(
    identify_t *ptThis, identify_resistance_result_t *ptResult)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis == NULL || ptResult == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (!ptThis->tResistance.bResultPending) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptResult->qResistancePu = ptThis->tResistance.qResistancePu;
    ptThis->tResistance.bResultPending = false;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
