/****************************************************************************
 * @file    identify_inductance.c
 * @brief   Incremental Ld along the aligned stationary alpha axis.
 ****************************************************************************/

#include "identify_inductance.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#if IDENTIFY_LOG_ACTIVE
#include "mdebug/util_debug.h"
#endif
#include "perf_counter.h"
#include "motor.h"

#define IDENTIFY_INDUCTANCE_TIMEOUT_MARGIN_MS (500U)

/* Request at k -> PWM at k+1 -> completed-interval sample at k+2. */
#define FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES  (2U)

typedef struct {
    uint16_t hwHalfCount;
    identify_accumulator_t u_sum;
    identify_accumulator_t di_sum;
    identify_accumulator_t i_sum;
} identify_inductance_polarity_t;

static uint32_t _identify_inductance_HalfPeriod(uint32_t wFrequencyHz)
{
    return FOC_HF_ISR_HZ / (2U * wFrequencyHz);
}

static uint32_t _identify_inductance_TimeoutMs(uint32_t wHalfPeriodCycles,
                                               uint16_t hwHalfCycleCount)
{
    uint64_t ullCycles = (uint64_t)wHalfPeriodCycles * hwHalfCycleCount;
    uint64_t ullMilliseconds =
        (ullCycles * 1000U + FOC_HF_ISR_HZ - 1U) / FOC_HF_ISR_HZ;

    ullMilliseconds += IDENTIFY_INDUCTANCE_TIMEOUT_MARGIN_MS;
    return ullMilliseconds > UINT32_MAX ? UINT32_MAX :
           (uint32_t)ullMilliseconds;
}

static bool _identify_inductance_ConfigValid(
    const identify_inductance_cfg_t *ptConfig,
    uint32_t *pwHalfPeriod)
{
    uint32_t wHalfPeriod = 0U;

    if (ptConfig == NULL || pwHalfPeriod == NULL ||
        ptConfig->wInjectionFrequencyHz == 0U ||
        ptConfig->hwCaptureSampleCount < 2U ||
        ptConfig->hwHalfCycleCount < 4U ||
        (ptConfig->hwHalfCycleCount & 1U) != 0U ||
        ptConfig->hwTransverseFaultCycles == 0U ||
        ptConfig->wInjectionFrequencyHz > (FOC_HF_ISR_HZ / 2U) ||
        ptConfig->hwCaptureDelayCycles <
            (FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES +
             FOC_DCBUS_SAMPLE_DELAY_CYCLES)) {
        return false;
    }
    if ((FOC_HF_ISR_HZ % (2U * ptConfig->wInjectionFrequencyHz)) != 0U) {
        return false;
    }
    wHalfPeriod = _identify_inductance_HalfPeriod(
        ptConfig->wInjectionFrequencyHz);
    if ((uint32_t)ptConfig->hwCaptureDelayCycles +
            (uint32_t)ptConfig->hwCaptureSampleCount > wHalfPeriod) {
        return false;
    }
    if (!foc_scalar_is_finite(ptConfig->qModulationAmplitude) ||
        !foc_scalar_is_finite(ptConfig->qMaxIdentificationCurrent) ||
        !foc_scalar_is_finite(ptConfig->qMinCurrentDelta) ||
        !foc_scalar_is_finite(ptConfig->qMaxTransverseCurrentPu)) {
        return false;
    }
    if (ptConfig->qModulationAmplitude <= FOC_ZERO ||
        ptConfig->qModulationAmplitude > FOC_ONE ||
        ptConfig->qMaxIdentificationCurrent <= FOC_ZERO ||
        ptConfig->qMaxIdentificationCurrent > FOC_ONE ||
        ptConfig->qMinCurrentDelta <= FOC_ZERO ||
        ptConfig->qMinCurrentDelta >= ptConfig->qMaxIdentificationCurrent ||
        ptConfig->qMaxTransverseCurrentPu <= FOC_ZERO ||
        ptConfig->qMaxTransverseCurrentPu >
            ptConfig->qMaxIdentificationCurrent) {
        return false;
    }
    *pwHalfPeriod = wHalfPeriod;
    return true;
}

static void _identify_inductance_Prepare(
    const identify_inductance_cfg_t *ptConfig,
    uint32_t wHalfPeriod,
    identify_inductance_t *ptInductance)
{
    ptInductance->tPlan.wHalfPeriodCycles = wHalfPeriod;
    ptInductance->tPlan.wCaptureStartCycle = ptConfig->hwCaptureDelayCycles;
    ptInductance->tPlan.wTimeoutMs = _identify_inductance_TimeoutMs(
        wHalfPeriod, ptConfig->hwHalfCycleCount);
    ptInductance->tPlan.atCommand[0].qAlpha = ptConfig->qModulationAmplitude;
    ptInductance->tPlan.atCommand[1].qAlpha = -ptConfig->qModulationAmplitude;
    ptInductance->tPlan.qMaxCurrent = ptConfig->qMaxIdentificationCurrent;
    ptInductance->tPlan.qMaxTransverseCurrent = ptConfig->qMaxTransverseCurrentPu;
    ptInductance->tPlan.qMinDelta = ptConfig->qMinCurrentDelta;
    ptInductance->tPlan.hwCaptureTarget = ptConfig->hwCaptureSampleCount;
    ptInductance->tPlan.hwHalfCycleTarget = ptConfig->hwHalfCycleCount;
    ptInductance->tPlan.hwTransverseFaultLimit =
        ptConfig->hwTransverseFaultCycles;

}

static void _identify_inductance_ResetCapture(identify_inductance_t *ptInductance)
{
    identify_inductance_capture_t *capture = &ptInductance->tCapture;

    capture->hwCaptureSampleCount = 0U;
    capture->u_window_sum = 0;
    capture->qFirstCurrent = FOC_ZERO;
    capture->qLastCurrent = FOC_ZERO;
    capture->i_window_sum = 0;
}

static foc_result_t _identify_inductance_FailForeground(
    identify_t *ptThis,
    motor_t *ptMotor,
    foc_result_t eResult,
    const char *pchStage)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;
    perfc_global_interrupt_status_t tIrqState = 0U;
#if IDENTIFY_LOG_ACTIVE
    bool bReportFailure = false;
#endif

    tIrqState = perfc_port_disable_global_interrupt();
#if IDENTIFY_LOG_ACTIVE
    bReportFailure = !ptInductance->bError;
#endif
    if (!ptInductance->bError) {
        ptInductance->eIsrResult = eResult;
    }
    ptInductance->bError = true;
    ptInductance->bActive = false;
    ptInductance->bBatchReady = false;
    eResult = ptInductance->eIsrResult;
    ptThis->eLastResult = eResult;
    ptThis->eState = IDENTIFY_STATE_ERROR;
    ptThis->eOperation = IDENTIFY_OPERATION_NONE;
    perfc_port_resume_global_interrupt(tIrqState);
#if IDENTIFY_LOG_ACTIVE
    if (bReportFailure) {
        MLOGF(W, "identify Ld %s failed rc=%d\r\n", pchStage,
              (int)eResult);
    }
#else
    (void)pchStage;
#endif
    if (ptMotor != NULL) {
        motor_Stop(ptMotor);
    }
    return eResult;
}

static identify_failure_cause_t _identify_inductance_FailureCause(
    const identify_inductance_sample_t *ptSample,
    const identify_inductance_plan_t *ptPlan)
{
    bool bFinite = foc_scalar_is_finite(ptSample->tCurrentPu.qAlpha) &&
        foc_scalar_is_finite(ptSample->tCurrentPu.qBeta) &&
        foc_scalar_is_finite(ptSample->qVoltageModelAlphaPu);

    if (!ptSample->bValid || !bFinite) {
        return IDENTIFY_FAILURE_CAUSE_INVALID_SAMPLE;
    }
    if (ptSample->bPwmSaturated) {
        return IDENTIFY_FAILURE_CAUSE_PWM_SATURATED;
    }
    if (ptSample->tCurrentPu.qAlpha > ptPlan->qMaxCurrent ||
        ptSample->tCurrentPu.qAlpha < -ptPlan->qMaxCurrent ||
        ptSample->tCurrentPu.qBeta > ptPlan->qMaxCurrent ||
        ptSample->tCurrentPu.qBeta < -ptPlan->qMaxCurrent) {
        return IDENTIFY_FAILURE_CAUSE_OVERCURRENT;
    }
    return IDENTIFY_FAILURE_CAUSE_NONE;
}

/** @brief Commit a completed half-window to its polarity accumulator.
 * @param ptInductance Ld state. @return None.
 * @note Fixed sums are int64 Q15; at most 32767*65535 samples/polarity. */
static void _identify_inductance_SaveHalfCycle(
    identify_inductance_t *ptInductance)
{
    identify_inductance_capture_t *capture = &ptInductance->tCapture;
    identify_accumulator_t di =
        (identify_accumulator_t)capture->qLastCurrent -
        (identify_accumulator_t)capture->qFirstCurrent;

    if ((capture->hwHalfCycle & 1U) == 0U) {
        capture->hwPositiveHalfCount++;
        capture->di_positive_sum += di;
        capture->i_positive_sum += capture->i_window_sum;
        capture->u_positive_sum += capture->u_window_sum;
    } else {
        capture->hwNegativeHalfCount++;
        capture->di_negative_sum += di;
        capture->i_negative_sum += capture->i_window_sum;
        capture->u_negative_sum += capture->u_window_sum;
    }
}

static bool _identify_inductance_MotorReady(const motor_t *ptMotor)
{
    motor_status_t tStatus = {0};

    if (motor_GetStatus(ptMotor, &tStatus) != FOC_RESULT_OK) {
        return false;
    }
    return tStatus.eState == MOTOR_STATE_RUNNING && tStatus.bPwmEnabled &&
           tStatus.eMode == FOC_MODE_VOLTAGE && tStatus.bIdentificationActive &&
           tStatus.wFaults == MOTOR_FAULT_NONE;
}

static bool _identify_inductance_CopyBatch(
    identify_inductance_t *ptInductance,
    identify_inductance_polarity_t *ptPositive,
    identify_inductance_polarity_t *ptNegative)
{
    identify_inductance_capture_t *capture = &ptInductance->tCapture;

    if (!ptInductance->bBatchReady) {
        return false;
    }
    __perfc_sync_barrier__();
    ptPositive->hwHalfCount = capture->hwPositiveHalfCount;
    ptNegative->hwHalfCount = capture->hwNegativeHalfCount;
    ptPositive->u_sum = capture->u_positive_sum;
    ptNegative->u_sum = capture->u_negative_sum;
    ptPositive->di_sum = capture->di_positive_sum;
    ptNegative->di_sum = capture->di_negative_sum;
    ptPositive->i_sum = capture->i_positive_sum;
    ptNegative->i_sum = capture->i_negative_sum;
    ptInductance->bBatchReady = false;
    return true;
}

/** @brief l_pu = (u_mean - r_pu*i_mean) * dt_pu / delta(i).
 * @param ptPolarity Captured signed voltage/current in one polarity.
 * @param plan PU resistance and capture interval.
 * @param pqResult Positive inductance PU.
 * @return Validity/range status. */
static foc_result_t _identify_inductance_CalculatePolarity(
    const identify_inductance_polarity_t *ptPolarity,
    const identify_inductance_plan_t *plan,
    foc_scalar_t *pqResult)
{
    uint32_t n = (uint32_t)ptPolarity->hwHalfCount *
                 plan->hwCaptureTarget;
    foc_scalar_t di = FOC_ZERO;
    foc_scalar_t i = FOC_ZERO;
    foc_scalar_t u = FOC_ZERO;

    if (ptPolarity->hwHalfCount < 2U || n == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    di = (foc_scalar_t)(ptPolarity->di_sum / ptPolarity->hwHalfCount);
    i = (foc_scalar_t)(ptPolarity->i_sum / n);
    u = (foc_scalar_t)(ptPolarity->u_sum / n);
    if (foc_abs(di) < plan->qMinDelta || di == FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#if defined(FOC_NUMERIC_FLOAT)
    {
        float net = u - plan->qResistancePu * i;
        float l = net * plan->qCaptureTimePu / di;

        if (!isfinite(l) || l <= 0.0f) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        *pqResult = l;
    }
#else
    {
        /* Q15 products stay wide: r may exceed 1 pu (no clamped mul). */
        int64_t net = (int64_t)u -
            (int64_t)plan->qResistancePu * i / FOC_Q_SCALE;
        int64_t l = 0;

        if (net == 0 || (net > 0) != (di > 0) ||
            net > INT32_MAX || net < INT32_MIN) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        l = net * plan->qCaptureTimePu / di;
        if (l <= 0 || l > INT32_MAX) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        *pqResult = (foc_scalar_t)l;
    }
#endif
    return FOC_RESULT_OK;
}

/** @brief Average independently estimated positive/negative inductance PU.
 * @param ptInductance Ld state. @param ptOutput Result PU.
 * @return Measurement status. */
static foc_result_t _identify_inductance_Calculate(
    identify_inductance_t *ptInductance, identify_inductance_result_t *ptOutput)
{
    identify_inductance_polarity_t positive = {0};
    identify_inductance_polarity_t negative = {0};
    foc_scalar_t lp = FOC_ZERO;
    foc_scalar_t ln = FOC_ZERO;

    if (!_identify_inductance_CopyBatch(ptInductance, &positive, &negative)) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    if (_identify_inductance_CalculatePolarity(
            &positive, &ptInductance->tPlan, &lp) != FOC_RESULT_OK ||
        _identify_inductance_CalculatePolarity(
            &negative, &ptInductance->tPlan, &ln) != FOC_RESULT_OK) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#if defined(FOC_NUMERIC_FLOAT)
    ptOutput->qInductanceDPu = (lp + ln) * 0.5f;
#else
    ptOutput->qInductanceDPu = (foc_scalar_t)(((int64_t)lp + ln) / 2);
#endif
    return FOC_RESULT_OK;
}

foc_result_t _identify_inductance_Start(
    identify_t *ptThis,
    const identify_inductance_cfg_t *ptConfig)
{
    identify_inductance_t *ptInductance = NULL;
    uint32_t wHalfPeriod = 0U;
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!_identify_inductance_ConfigValid(ptConfig, &wHalfPeriod)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptThis->eState == IDENTIFY_STATE_ERROR ||
        ptThis->eLastResult != FOC_RESULT_OK) {
        return FOC_RESULT_SAFETY;
    }
    if (ptThis->eState != IDENTIFY_STATE_IDLE ||
        ptThis->tResistance.bResultPending ||
        ptThis->tInductance.bResultPending) {
        return FOC_RESULT_BUSY;
    }
    ptInductance = &ptThis->tInductance;
    *ptInductance = (identify_inductance_t){0};
    _identify_inductance_Prepare(ptConfig, wHalfPeriod, ptInductance);
    ptInductance->eIsrResult = FOC_RESULT_OK;
    tIrqState = perfc_port_disable_global_interrupt();
    ptThis->eLastResult = FOC_RESULT_OK;
    ptThis->eOperation = IDENTIFY_OPERATION_INDUCTANCE;
    ptThis->eState = IDENTIFY_STATE_RUNNING;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

/** @brief Pure Ld capture; failures are latched by the public ISR adapter.
 * @param ptInductance Own state. @param ptSample PU input.
 * @param ptRequest Caller-owned request. @return Failure category. */
static identify_failure_cause_t _identify_inductance_CaptureStep(
    identify_inductance_t *ptInductance,
    const identify_inductance_sample_t *ptSample,
    identify_isr_request_t *ptRequest)
{
    if (ptInductance == NULL || ptSample == NULL || ptRequest == NULL) {
        return IDENTIFY_FAILURE_CAUSE_INVALID_SAMPLE;
    }
    if (!ptInductance->bActive || ptInductance->bBatchReady) {
        return IDENTIFY_FAILURE_CAUSE_NONE;
    }
    identify_inductance_capture_t *capture = &ptInductance->tCapture;
    const identify_inductance_plan_t *plan = &ptInductance->tPlan;
    identify_failure_cause_t cause =
        _identify_inductance_FailureCause(ptSample, plan);
    if (cause != IDENTIFY_FAILURE_CAUSE_NONE) {
        return cause;
    }
    if (ptSample->tCurrentPu.qBeta > plan->qMaxTransverseCurrent ||
        ptSample->tCurrentPu.qBeta < -plan->qMaxTransverseCurrent) {
        capture->hwTransverseFaultCount++;
        if (capture->hwTransverseFaultCount >= plan->hwTransverseFaultLimit) {
            return IDENTIFY_FAILURE_CAUSE_TRANSVERSE_CURRENT;
        }
    } else {
        capture->hwTransverseFaultCount = 0U;
    }
    if (capture->wPhaseCycle == 0U) {
        ptRequest->tModulationCommand =
            plan->atCommand[capture->hwHalfCycle & 1U];
        ptRequest->bSetModulation = true;
    }
    if ((capture->wPhaseCycle - plan->wCaptureStartCycle) <
        plan->hwCaptureTarget) {
        if (capture->hwCaptureSampleCount == 0U) {
            capture->qFirstCurrent = ptSample->tCurrentPu.qAlpha;
        }
        capture->qLastCurrent = ptSample->tCurrentPu.qAlpha;
        capture->i_window_sum += ptSample->tCurrentPu.qAlpha;
        capture->u_window_sum += ptSample->qVoltageModelAlphaPu;
        capture->hwCaptureSampleCount++;
    }
    if (capture->wPhaseCycle + 1U >= plan->wHalfPeriodCycles) {
        if (capture->hwCaptureSampleCount != plan->hwCaptureTarget) {
            return IDENTIFY_FAILURE_CAUSE_INCOMPLETE_CAPTURE;
        }
        _identify_inductance_SaveHalfCycle(ptInductance);
        capture->hwHalfCycle++;
        capture->wPhaseCycle = 0U;
        _identify_inductance_ResetCapture(ptInductance);
        if (capture->hwHalfCycle >= plan->hwHalfCycleTarget) {
            ptRequest->tModulationCommand = (foc_ab_t){0};
            ptRequest->bSetModulation = true;
            ptInductance->bActive = false;
            __perfc_sync_barrier__();
            ptInductance->bBatchReady = true;
        }
        return IDENTIFY_FAILURE_CAUSE_NONE;
    }
    capture->wPhaseCycle++;
    return IDENTIFY_FAILURE_CAUSE_NONE;
}

/** @brief Dispatch Ld capture and latch only the parent's lifecycle failure.
 * @param ptThis Scheduler. @param ptSample Ld sample. @param ptRequest Request.
 * @return None. */
void identify_InductanceIsrStep(identify_t *ptThis,
    const identify_inductance_sample_t *ptSample,
    identify_isr_request_t *ptRequest)
{
    if (ptRequest == NULL) {
        return;
    }
    *ptRequest = (identify_isr_request_t){0};
    if (ptThis != NULL &&
        ptThis->eOperation != IDENTIFY_OPERATION_INDUCTANCE) {
        return;
    }
    identify_failure_cause_t cause = _identify_inductance_CaptureStep(
        ptThis == NULL ? NULL : &ptThis->tInductance, ptSample, ptRequest);
    if (cause != IDENTIFY_FAILURE_CAUSE_NONE) {
        identify_AbortIsr(ptThis, cause, FOC_RESULT_SAFETY);
        ptRequest->bAbort = true;
    }
}

fsm_rt_t _identify_inductance_RunPt(identify_t *ptThis,
                                     motor_t *ptMotor)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;
    motor_status_t tStatus = {0};
    foc_result_t eResult = FOC_RESULT_OK;
    const char *pchFailureStage = "calculate";

    PERFC_PT_BEGIN(ptInductance->chRunPt)
    eResult = motor_GetStatus(ptMotor, &tStatus);
    if (eResult != FOC_RESULT_OK || tStatus.eState != MOTOR_STATE_RUNNING ||
        !tStatus.bPwmEnabled || tStatus.wFaults != MOTOR_FAULT_NONE ||
        !tStatus.bIdentificationActive ||
        tStatus.eMode != FOC_MODE_VOLTAGE ||
        ptInductance->tPlan.atCommand[0].qAlpha >
            ptMotor->tLimits.qMaxModulation ||
        ptInductance->tPlan.qMaxCurrent > ptMotor->tLimits.qMaxPhaseCurrent) {
        eResult = eResult == FOC_RESULT_OK ? FOC_RESULT_SAFETY : eResult;
        _identify_inductance_FailForeground(ptThis, ptMotor, eResult,
                                            "motor-ready");
    } else {
        double dt = (double)foc_to_float(ptThis->qSamplePeriodPu) *
                    (double)(ptInductance->tPlan.hwCaptureTarget - 1U);
        if (dt >= 65535.0) {
            _identify_inductance_FailForeground(ptThis, ptMotor,
                                               FOC_RESULT_OUT_OF_RANGE,
                                               "capture-window");
            return fsm_rt_err;
        }
        ptInductance->tPlan.qCaptureTimePu = foc_from_float((float)dt);
        ptInductance->tPlan.qResistancePu = ptThis->qResistancePu;
        __perfc_sync_barrier__();
        ptInductance->bActive = true;
    }
    ptInductance->lStateTimestamp = 0;
    (void)perfc_is_time_out_ms(ptInductance->tPlan.wTimeoutMs,
        &ptInductance->lStateTimestamp, false);
    PERFC_PT_WAIT_UNTIL(ptInductance->bError ||
                        ptInductance->bBatchReady ||
                        !_identify_inductance_MotorReady(ptMotor) ||
                         perfc_is_time_out_ms(ptInductance->tPlan.wTimeoutMs,
                                             &ptInductance->lStateTimestamp,
                                             false))
    if (ptInductance->bError) {
        eResult = ptInductance->eIsrResult;
        pchFailureStage = "isr";
    } else if (!_identify_inductance_MotorReady(ptMotor)) {
        eResult = FOC_RESULT_SAFETY;
        pchFailureStage = "motor-fault";
    } else if (!ptInductance->bBatchReady) {
        eResult = FOC_RESULT_BUSY;
        pchFailureStage = "capture-timeout";
    } else {
        eResult = _identify_inductance_Calculate(
            ptInductance, &ptInductance->tResult);
    }
    if (eResult != FOC_RESULT_OK) {
        _identify_inductance_FailForeground(ptThis, ptMotor, eResult,
                                            pchFailureStage);
        return fsm_rt_err;
    }
    ptInductance->bResultPending = true;
    ptThis->eLastResult = FOC_RESULT_OK;
    ptThis->eOperation = IDENTIFY_OPERATION_NONE;
    ptThis->eState = IDENTIFY_STATE_IDLE;
    motor_Stop(ptMotor);
    PERFC_PT_RETURN(fsm_rt_cpl)
    PERFC_PT_END()
    return fsm_rt_on_going;
}

void _identify_inductance_Stop(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return;
    }
    ptThis->tInductance.bActive = false;
    ptThis->tInductance.bBatchReady = false;
    ptThis->tInductance.bResultPending = false;
    ptThis->tInductance.chRunPt = 0U;
}

void _identify_inductance_Reset(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return;
    }
    ptThis->tInductance = (identify_inductance_t){0};
}

foc_result_t _identify_inductance_TakeResult(
    identify_t *ptThis,
    identify_inductance_result_t *ptResult)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis == NULL || ptResult == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (!ptThis->tInductance.bResultPending) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    *ptResult = ptThis->tInductance.tResult;
    ptThis->tInductance.bResultPending = false;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
