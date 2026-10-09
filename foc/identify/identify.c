/****************************************************************************
 * @file    identify.c
 * @brief   Parameter-identification parent flow and dispatch.
 * @author  Modus project
 * @date    2026-09-22
 ****************************************************************************/

#include "identify.h"
#include "motor.h"

#include <stddef.h>
#include <math.h>
#include <limits.h>

#include "identify_inductance.h"
#include "identify_resistance.h"
#if IDENTIFY_LOG_ACTIVE
#include "mdebug/util_debug.h"
#endif
#include "perf_counter.h"
#include "perfc_task_pt.h"

/**
 * @brief Initialize one identification object.
 * @param ptThis Caller-owned identification object.
 * @param ptConfig Physical model/base values converted once.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Init(identify_t *ptThis,
                            const identify_cfg_t *ptConfig)
{
    double rb = 0.0;
    double wb = 0.0;
    double r = 0.0;
    double ts = 0.0;

    if (ptThis == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptThis = (identify_t){0};
    if (ptConfig->wVoltageBaseMillivolt == 0U ||
        ptConfig->wCurrentBaseMilliamp == 0U ||
        ptConfig->wResistanceMilliohm == 0U ||
        !foc_scalar_is_finite(ptConfig->qElectricalSpeedBaseHz) ||
        ptConfig->qElectricalSpeedBaseHz <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    rb = (double)ptConfig->wVoltageBaseMillivolt /
         (double)ptConfig->wCurrentBaseMilliamp;
    wb = 6.283185307179586 *
         (double)foc_to_float(ptConfig->qElectricalSpeedBaseHz);
    r = (double)ptConfig->wResistanceMilliohm / (1000.0 * rb);
    ts = wb / (double)FOC_HF_ISR_HZ;
    if (r >= 65535.0 || ts >= 65535.0 ||
        rb * 1000.0 > (double)UINT32_MAX ||
        rb * 1000000.0 / wb > (double)UINT32_MAX) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptThis->qResistancePu = foc_from_float((float)r);
    ptThis->qSamplePeriodPu = foc_from_float((float)ts);
    if (ptThis->qResistancePu <= FOC_ZERO ||
        ptThis->qSamplePeriodPu <= FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptThis->fResistanceBaseMilliohm = (float)(rb * 1000.0);
    ptThis->fInductanceBaseMicroHenry = (float)(rb * 1000000.0 / wb);
    ptThis->eState = IDENTIFY_STATE_IDLE;
    ptThis->eLastResult = FOC_RESULT_OK;
    return FOC_RESULT_OK;
}

/** @brief Foreground physical display conversion.
 * @param ptThis Initialized object. @param eQuantity Result kind.
 * @param qValuePu Result PU. @param pwPhysical Output mOhm or microhenry.
 * @return Checked conversion status. */
foc_result_t identify_ResultToPhysical(const identify_t *ptThis,
    identify_operation_t eQuantity, foc_scalar_t qValuePu,
    uint32_t *pwPhysical)
{
    double scale = 0.0;
    double value = 0.0;

    if (ptThis == NULL || pwPhysical == NULL) {
        return FOC_RESULT_NULL;
    }
    if (eQuantity == IDENTIFY_OPERATION_RESISTANCE) {
        scale = (double)ptThis->fResistanceBaseMilliohm;
    } else if (eQuantity == IDENTIFY_OPERATION_INDUCTANCE) {
        scale = (double)ptThis->fInductanceBaseMicroHenry;
    } else {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    value = (double)foc_to_float(qValuePu) * scale;
    if (!isfinite(value) || scale <= 0.0 || value <= 0.0 ||
        value > (double)UINT32_MAX - 0.5) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    *pwPhysical = (uint32_t)(value + 0.5);
    return FOC_RESULT_OK;
}

/**
 * @brief Start the resistance child through the parent lifecycle boundary.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t identify_StartResistance(identify_t *ptThis)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
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
    eResult = _identify_resistance_Start(ptThis);
    if (eResult == FOC_RESULT_OK) {
        ptThis->tStationary = (identify_stationary_t){0};
    }
    return eResult;
}

foc_result_t identify_StartInductance(
    identify_t *ptThis,
    const identify_inductance_cfg_t *ptConfig)
{
    foc_result_t eResult = _identify_inductance_Start(ptThis, ptConfig);

    if (eResult == FOC_RESULT_OK) {
        ptThis->tStationary = (identify_stationary_t){0};
    }
    return eResult;
}

/** @brief End the R/L preparation wait on completion or Motor failure.
 * @param ptMotor Aligning Motor. @return Whether foreground must recheck. */
static bool _identify_AlignmentFinished(const motor_t *ptMotor)
{
    motor_status_t status = {0};
    foc_result_t result = motor_GetStatus(ptMotor, &status);

    return result != FOC_RESULT_OK || status.eState != MOTOR_STATE_ALIGN ||
           !status.bPwmEnabled || !status.bIdentificationActive;
}

/** @brief Stop PWM and latch a failed fixed-vector preparation.
 * @param ptThis Identification object. @param ptMotor Motor session.
 * @param eResult Failure code. @param stop Owned session must stop.
 * @return None. */
static void _identify_PreparationFailed(identify_t *ptThis,
    motor_t *ptMotor, foc_result_t eResult, bool stop)
{
    perfc_global_interrupt_status_t irq =
        perfc_port_disable_global_interrupt();

    if (ptThis->eState != IDENTIFY_STATE_ERROR) {
        ptThis->eLastResult = eResult;
        ptThis->eState = IDENTIFY_STATE_ERROR;
        ptThis->eOperation = IDENTIFY_OPERATION_NONE;
    }
    perfc_port_resume_global_interrupt(irq);
    if (stop) {
        motor_Stop(ptMotor);
    }
#if IDENTIFY_LOG_ACTIVE
    MLOGF(W, "identify fixed-axis align failed rc=%d\r\n", (int)eResult);
#endif
}

/** @brief Prepare only Rs/Ld with a bounded nonblocking alignment PT.
 * @param ptThis Identification instance. @param ptMotor Idle Motor.
 * @return Ready, progress or safely stopped failure. */
static fsm_rt_t _identify_PrepareStationary(identify_t *ptThis,
    motor_t *ptMotor)
{
    identify_stationary_t *prep = &ptThis->tStationary;
    motor_status_t status = {0};
    foc_result_t result = FOC_RESULT_OK;

    PERFC_PT_BEGIN(prep->chRunPt)
    result = motor_StartIdentification(ptMotor);
    if (result != FOC_RESULT_OK) {
        _identify_PreparationFailed(ptThis, ptMotor, result, false);
        return fsm_rt_err;
    }
    prep->lDeadline = 0;
    (void)perfc_is_time_out_ms(IDENTIFY_ALIGNMENT_TIMEOUT_MS,
                                   &prep->lDeadline, false);
    PERFC_PT_WAIT_UNTIL(_identify_AlignmentFinished(ptMotor) ||
        perfc_is_time_out_ms(IDENTIFY_ALIGNMENT_TIMEOUT_MS,
                             &prep->lDeadline, false))
    result = motor_GetStatus(ptMotor, &status);
    if (result != FOC_RESULT_OK || status.eState != MOTOR_STATE_RUNNING ||
        !status.bIdentificationActive || !status.bPwmEnabled ||
        status.eMode != FOC_MODE_VOLTAGE ||
        status.wFaults != MOTOR_FAULT_NONE) {
        result = result == FOC_RESULT_OK ? FOC_RESULT_SAFETY : result;
        _identify_PreparationFailed(ptThis, ptMotor, result, true);
        return fsm_rt_err;
    }
    prep->bReady = true;
    PERFC_PT_RETURN(fsm_rt_cpl)
    PERFC_PT_END()
    return fsm_rt_on_going;
}

/**
 * @brief Snapshot the active algorithm for typed ISR sample assembly.
 * @param ptThis Identification object.
 * @return Active operation or NONE.
 */
identify_operation_t identify_ActiveOperationIsr(const identify_t *ptThis)
{
    return ptThis == NULL ? IDENTIFY_OPERATION_NONE : ptThis->eOperation;
}

/** @brief Latch an ISR failure for one foreground diagnostic line.
 * @param ptThis Identification object. @param eCause Failure category.
 * @param eResult Failure result. @return None. */
void identify_AbortIsr(identify_t *ptThis,
    identify_failure_cause_t eCause, foc_result_t eResult)
{
    if (ptThis == NULL) {
        return;
    }
#if IDENTIFY_LOG_ACTIVE
    if (!ptThis->bFailureLogPending) {
        ptThis->eFailureOperation = ptThis->eOperation;
        ptThis->eFailureCause = eCause;
        ptThis->eFailureResult = eResult;
        __perfc_sync_barrier__();
        ptThis->bFailureLogPending = true;
    }
#else
    (void)eCause;
#endif
    ptThis->tResistance.bMotorStarted = false;
    ptThis->tResistance.bBatchReady = false;
    ptThis->tResistance.bResultPending = false;
    ptThis->tResistance.eState = IDENTIFY_RESISTANCE_STATE_ERROR;
    ptThis->tInductance.bActive = false;
    ptThis->tInductance.bError = true;
    ptThis->tInductance.eIsrResult = eResult;
    ptThis->tInductance.bResultPending = false;
    ptThis->eLastResult = eResult;
    ptThis->eState = IDENTIFY_STATE_ERROR;
    ptThis->eOperation = IDENTIFY_OPERATION_NONE;
}

#if IDENTIFY_LOG_ACTIVE
static const char *_identify_FailureCauseName(
    identify_failure_cause_t eCause)
{
    switch (eCause) {
    case IDENTIFY_FAILURE_CAUSE_INVALID_SAMPLE: return "invalid-sample";
    case IDENTIFY_FAILURE_CAUSE_PWM_SATURATED: return "pwm-saturated";
    case IDENTIFY_FAILURE_CAUSE_OVERCURRENT: return "overcurrent";
    case IDENTIFY_FAILURE_CAUSE_TRANSVERSE_CURRENT: return "transverse-current";
    case IDENTIFY_FAILURE_CAUSE_INCOMPLETE_CAPTURE: return "capture-short";
    case IDENTIFY_FAILURE_CAUSE_CONTROL_REJECTED: return "control-rejected";
    case IDENTIFY_FAILURE_CAUSE_NONE:
    default: return "unknown";
    }
}

static void _identify_ReportPendingIsrFailure(identify_t *ptThis)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    identify_operation_t eOperation = IDENTIFY_OPERATION_NONE;
    identify_failure_cause_t eCause = IDENTIFY_FAILURE_CAUSE_NONE;
    foc_result_t eResult = FOC_RESULT_OK;

    tIrqState = perfc_port_disable_global_interrupt();
    if (ptThis->bFailureLogPending) {
        eOperation = ptThis->eFailureOperation;
        eCause = ptThis->eFailureCause;
        eResult = ptThis->eFailureResult;
        ptThis->bFailureLogPending = false;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    if (eCause == IDENTIFY_FAILURE_CAUSE_NONE) {
        return;
    }

    if (eOperation == IDENTIFY_OPERATION_RESISTANCE) {
        MLOGF(W, "identify R ISR %s rc=%d\r\n",
              _identify_FailureCauseName(eCause), (int)eResult);
    } else if (eOperation == IDENTIFY_OPERATION_INDUCTANCE) {
        MLOGF(W, "identify Ld ISR %s rc=%d\r\n",
              _identify_FailureCauseName(eCause), (int)eResult);
    } else {
        MLOGF(W, "identify ISR op=%u %s rc=%d\r\n",
              (unsigned)eOperation, _identify_FailureCauseName(eCause),
              (int)eResult);
    }
}
#endif

/** @brief Collect one completed typed child result at the parent boundary.
 * @param ptThis Identification object. @param eOperation Completed child.
 * @param ptResult Unified output, initialized by identify_Run().
 * @return Take status; result remains empty unless collection succeeds. */
static foc_result_t _identify_TakeResult(identify_t *ptThis,
    identify_operation_t eOperation, identify_result_t *ptResult)
{
    foc_result_t eResult = FOC_RESULT_DISABLED;

    if (eOperation == IDENTIFY_OPERATION_RESISTANCE) {
        identify_resistance_result_t tResistance = {0};

        eResult = _identify_resistance_TakeResult(ptThis, &tResistance);
        if (eResult == FOC_RESULT_OK) {
            ptResult->eOperation = eOperation;
            ptResult->tValue.tResistance = tResistance;
        }
    } else if (eOperation == IDENTIFY_OPERATION_INDUCTANCE) {
        identify_inductance_result_t tInductance = {0};

        eResult = _identify_inductance_TakeResult(ptThis, &tInductance);
        if (eResult == FOC_RESULT_OK) {
            ptResult->eOperation = eOperation;
            ptResult->tValue.tInductance = tInductance;
        }
    }
    return eResult;
}

/**
 * @brief Advance the foreground flow and return its completed tagged result.
 * @param ptThis Identification object.
 * @param ptMotor Motor object controlled by this identification run.
 * @param ptResult Completed PU result, or NONE while busy/error.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or a measurement error.
 */
foc_result_t identify_Run(identify_t *ptThis, motor_t *ptMotor,
                          identify_result_t *ptResult)
{
    fsm_rt_t ePtResult = fsm_rt_on_going;
    identify_operation_t eOperation = IDENTIFY_OPERATION_NONE;

    if (ptResult != NULL) {
        *ptResult = (identify_result_t){0};
    }
    if (ptThis == NULL || ptMotor == NULL || ptResult == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptThis->eState == IDENTIFY_STATE_ERROR) {
#if IDENTIFY_LOG_ACTIVE
        _identify_ReportPendingIsrFailure(ptThis);
#endif
        return ptThis->eLastResult;
    }
    if (ptThis->eState == IDENTIFY_STATE_IDLE) {
        if (ptThis->tResistance.bResultPending) {
            return _identify_TakeResult(ptThis,
                IDENTIFY_OPERATION_RESISTANCE, ptResult);
        }
        if (ptThis->tInductance.bResultPending) {
            return _identify_TakeResult(ptThis,
                IDENTIFY_OPERATION_INDUCTANCE, ptResult);
        }
        return FOC_RESULT_BUSY;
    }
    eOperation = ptThis->eOperation;
    if ((eOperation == IDENTIFY_OPERATION_RESISTANCE ||
         eOperation == IDENTIFY_OPERATION_INDUCTANCE) &&
        !ptThis->tStationary.bReady) {
        ePtResult = _identify_PrepareStationary(ptThis, ptMotor);
        if (ePtResult == fsm_rt_err) {
            return ptThis->eLastResult;
        }
        if (ePtResult != fsm_rt_cpl) {
            return FOC_RESULT_BUSY;
        }
    }
    switch (eOperation) {
    case IDENTIFY_OPERATION_RESISTANCE:
        ePtResult = _identify_resistance_RunPt(ptThis, ptMotor);
        break;
    case IDENTIFY_OPERATION_INDUCTANCE:
        ePtResult = _identify_inductance_RunPt(ptThis, ptMotor);
        break;
    case IDENTIFY_OPERATION_NONE:
    default:
        return FOC_RESULT_DISABLED;
    }
    if (ePtResult == fsm_rt_err) {
#if IDENTIFY_LOG_ACTIVE
        _identify_ReportPendingIsrFailure(ptThis);
#endif
        return ptThis->eLastResult;
    }
    if (ePtResult == fsm_rt_cpl) {
        return _identify_TakeResult(ptThis, eOperation, ptResult);
    }
    return FOC_RESULT_BUSY;
}

/**
 * @brief Stop identification and the controlled Motor.
 * @param ptThis Identification object.
 * @param ptMotor Motor object, or NULL when no Motor is available.
 * @return None.
 */
void identify_Stop(identify_t *ptThis, motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis == NULL) {
        if (ptMotor != NULL) {
            motor_Stop(ptMotor);
        }
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptThis->eState != IDENTIFY_STATE_UNINITIALIZED) {
        ptThis->eOperation = IDENTIFY_OPERATION_NONE;
        ptThis->eState = IDENTIFY_STATE_IDLE;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    if (ptMotor != NULL) {
        motor_Stop(ptMotor);
    }
    _identify_resistance_Stop(ptThis);
    _identify_inductance_Stop(ptThis);
    ptThis->tStationary = (identify_stationary_t){0};
}

/**
 * @brief Reset identification state after stop or error.
 * @param ptThis Identification object.
 * @param ptMotor Motor whose PWM state is checked before reset.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Reset(identify_t *ptThis,
                            const motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    motor_status_t tStatus = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL || ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = motor_GetStatus(ptMotor, &tStatus);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (tStatus.bPwmEnabled) {
        return FOC_RESULT_BUSY;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptThis->eOperation != IDENTIFY_OPERATION_NONE) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptThis->eState = IDENTIFY_STATE_IDLE;
    ptThis->eLastResult = FOC_RESULT_OK;
#if IDENTIFY_LOG_ACTIVE
    ptThis->eFailureOperation = IDENTIFY_OPERATION_NONE;
    ptThis->eFailureCause = IDENTIFY_FAILURE_CAUSE_NONE;
    ptThis->eFailureResult = FOC_RESULT_OK;
    ptThis->bFailureLogPending = false;
#endif
    perfc_port_resume_global_interrupt(tIrqState);
    _identify_resistance_Reset(ptThis);
    _identify_inductance_Reset(ptThis);
    ptThis->tStationary = (identify_stationary_t){0};
    return FOC_RESULT_OK;
}

/**
 * @brief Copy a status snapshot.
 * @param ptThis Identification object.
 * @param ptStatus Destination status snapshot.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_GetStatus(const identify_t *ptThis,
                                identify_status_t *ptStatus)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis == NULL || ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptStatus->eState = ptThis->eState;
    ptStatus->eOperation = ptThis->eOperation;
    ptStatus->eLastResult = ptThis->eLastResult;
    ptStatus->bResultPending =
        ptThis->tResistance.bResultPending ||
        ptThis->tInductance.bResultPending;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
