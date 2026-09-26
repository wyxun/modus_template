/****************************************************************************
 * @file    identify_resistance.c
 * @brief   D/Q-axis motor resistance identification implementation.
 * @author  Modus project
 * @date    2026-09-22
 ****************************************************************************/

#include "identify_resistance.h"

#include <math.h>
#include <stddef.h>

#include "perf_counter.h"

static void _identify_resistance_SetState(identify_t *ptThis,
                                          identify_resistance_state_t eState)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    tIrqState = perfc_port_disable_global_interrupt();
    /* perfc_is_time_out_ms() initializes a zero timestamp to now + period. */
    ptThis->tResistance.lStateTimestamp = 0;
    ptThis->tResistance.eState = eState;
    perfc_port_resume_global_interrupt(tIrqState);
}

static foc_scalar_t _identify_resistance_VoltageForLevel(uint8_t chLevel)
{
    if (chLevel == 0U) {
        return IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU;
    }
    return IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU;
}

static void _identify_resistance_ClearCapture(identify_t *ptThis)
{
    identify_resistance_t *ptResistance = &ptThis->tResistance;
    perfc_global_interrupt_status_t tIrqState = 0U;

    tIrqState = perfc_port_disable_global_interrupt();
    ptResistance->hwIsrDivider = 0U;
    ptResistance->hwCaptureSampleCount = 0U;
    ptResistance->qCurrentSum = FOC_ZERO;
    ptResistance->qVoltageSum = FOC_ZERO;
    ptResistance->bBatchReady = false;
    perfc_port_resume_global_interrupt(tIrqState);
}

static void _identify_resistance_Fail(identify_t *ptThis,
                                      motor_t *ptMotor,
                                      foc_result_t eResult)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    tIrqState = perfc_port_disable_global_interrupt();
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
    if (ptMotor != NULL) {
        motor_Stop(ptMotor);
    }
}

static foc_result_t _identify_resistance_ProcessLevel(identify_t *ptThis)
{
    identify_resistance_t *ptResistance = &ptThis->tResistance;
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_scalar_t qCurrentSum = FOC_ZERO;
    foc_scalar_t qVoltageSum = FOC_ZERO;
    uint16_t hwSampleCount = 0U;
    foc_scalar_t qSampleCount = FOC_ZERO;
    foc_result_t eResult = FOC_RESULT_OK;

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
    qSampleCount = foc_from_float(
        (float)IDENTIFY_RESISTANCE_SAMPLE_COUNT);
    eResult = foc_div_checked(qCurrentSum, qSampleCount,
                              &ptResistance->aqAverageCurrent[
                                  ptResistance->chVoltageLevel]);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    return foc_div_checked(qVoltageSum, qSampleCount,
                           &ptResistance->aqAverageVoltageD[
                               ptResistance->chVoltageLevel]);
}

static foc_result_t _identify_resistance_Calculate(
    identify_t *ptThis,
    const motor_t *ptMotor,
    uint32_t *pwResistance)
{
    foc_scalar_t qDeltaVoltage = FOC_ZERO;
    foc_scalar_t qDeltaCurrent = FOC_ZERO;

    if (ptThis == NULL || ptMotor == NULL || pwResistance == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tParams.wVoltageBaseMillivolt == 0U ||
        ptMotor->wCurrentBaseMilliamp == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    qDeltaVoltage = foc_sub_sat(
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU,
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU);
    qDeltaCurrent = foc_sub_sat(
        ptThis->tResistance.aqAverageCurrent[1U],
        ptThis->tResistance.aqAverageCurrent[0U]);
    if (foc_abs(qDeltaCurrent) <= IDENTIFY_RESISTANCE_MIN_CURRENT_PU) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#if FOC_APP_LOG_RESISTANCE_ID
    ptThis->tResistance.qDeltaCurrent = qDeltaCurrent;
    ptThis->tResistance.wVoltageBaseMillivolt =
        ptMotor->tParams.wVoltageBaseMillivolt;
    ptThis->tResistance.wCurrentBaseMilliamp =
        ptMotor->wCurrentBaseMilliamp;
#endif

#if defined(FOC_NUMERIC_FLOAT)
    {
        float fResistance = fabsf(foc_to_float(qDeltaVoltage) /
                                   foc_to_float(qDeltaCurrent));
        fResistance *= (float)ptMotor->tParams.wVoltageBaseMillivolt;
        fResistance /= (float)ptMotor->wCurrentBaseMilliamp;
        fResistance *= 1000.0f;
        if (!isfinite(fResistance) || fResistance < 0.0f ||
            fResistance > (float)UINT32_MAX) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        *pwResistance = (uint32_t)(fResistance + 0.5f);
    }
#else
    {
        int64_t llVoltage = (int64_t)qDeltaVoltage;
        int64_t llCurrent = (int64_t)qDeltaCurrent;
        uint64_t ullNumerator = 0U;
        uint64_t ullDenominator = 0U;
        uint64_t ullResistance = 0U;

        if (llVoltage < 0) {
            llVoltage = -llVoltage;
        }
        if (llCurrent < 0) {
            llCurrent = -llCurrent;
        }
        ullNumerator = (uint64_t)llVoltage *
                       (uint64_t)ptMotor->tParams.wVoltageBaseMillivolt *
                       1000U;
        ullDenominator = (uint64_t)llCurrent *
                         (uint64_t)ptMotor->wCurrentBaseMilliamp;
        if (ullDenominator == 0U) {
            return FOC_RESULT_DIVIDE_BY_ZERO;
        }
        ullResistance = ullNumerator / ullDenominator;
        if (ullResistance > (uint64_t)UINT32_MAX) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        *pwResistance = (uint32_t)ullResistance;
    }
#endif
    return FOC_RESULT_OK;
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
    if (!ptResistance->bMotorStarted) {
        if (tStatus.bPwmEnabled || tStatus.eState != MOTOR_STATE_IDLE) {
            return FOC_RESULT_BUSY;
        }
        eResult = motor_Start(ptMotor, FOC_MODE_VOLTAGE);
        if (eResult != FOC_RESULT_OK) {
            return eResult;
        }
        ptResistance->bMotorStarted = true;
    } else if (tStatus.eMode != FOC_MODE_VOLTAGE) {
        return FOC_RESULT_BUSY;
    }
    eResult = motor_SetVoltageReference(
        ptMotor, _identify_resistance_VoltageForLevel(
            ptResistance->chVoltageLevel), FOC_ZERO);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
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
 * @brief Record one high-frequency D-axis current sample.
 * @param ptThis Identification object.
 * @param qCurrentD Motor's transformed D-axis current.
 * @return None.
 */
void _identify_resistance_IsrStep(identify_t *ptThis,
                                  foc_scalar_t qCurrentD,
                                  foc_scalar_t qVoltageD)
{
    identify_resistance_t *ptResistance = NULL;

    if (ptThis == NULL) {
        return;
    }
    ptResistance = &ptThis->tResistance;
    if (ptResistance->eState != IDENTIFY_RESISTANCE_STATE_CAPTURE ||
        ptResistance->bBatchReady) {
        return;
    }
    ptResistance->hwIsrDivider++;
    if (ptResistance->hwIsrDivider < IDENTIFY_RESISTANCE_ISR_PER_SAMPLE) {
        return;
    }
    ptResistance->hwIsrDivider = 0U;
    /* 100 samples of a [-1, 1] pu current fit in the scalar accumulator. */
    ptResistance->qCurrentSum += qCurrentD;
    ptResistance->qVoltageSum += qVoltageD;
    ptResistance->hwCaptureSampleCount++;
    if (ptResistance->hwCaptureSampleCount >=
        IDENTIFY_RESISTANCE_SAMPLE_COUNT) {
        ptResistance->bBatchReady = true;
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
    if (tStatus.eState == MOTOR_STATE_FAULT || !tStatus.bPwmEnabled) {
        *peResult = FOC_RESULT_SAFETY;
        return false;
    }
    return true;
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
            foc_abs(ptMotor->tCore.tCurrent.qD) >=
                IDENTIFY_RESISTANCE_MIN_CURRENT_PU ||
            perfc_is_time_out_ms(IDENTIFY_RESISTANCE_SETTLE_TIMEOUT_MS,
                                  &ptResistance->lStateTimestamp, false))
        if (eResult != FOC_RESULT_OK) {
            _identify_resistance_Fail(ptThis, ptMotor, eResult);
            return fsm_rt_err;
        }
        if (foc_abs(ptMotor->tCore.tCurrent.qD) <
            IDENTIFY_RESISTANCE_MIN_CURRENT_PU) {
            _identify_resistance_Fail(ptThis, ptMotor,
                                      FOC_RESULT_OUT_OF_RANGE);
            return fsm_rt_err;
        }
        _identify_resistance_ClearCapture(ptThis);
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
        eResult = _identify_resistance_ProcessLevel(ptThis);
        if (eResult != FOC_RESULT_OK) {
            _identify_resistance_Fail(ptThis, ptMotor, eResult);
            return fsm_rt_err;
        }
        ptResistance->chVoltageLevel++;
    }
    _identify_resistance_SetState(
        ptThis, IDENTIFY_RESISTANCE_STATE_CALCULATE);
    eResult = _identify_resistance_Calculate(
        ptThis, ptMotor, &ptResistance->wResistanceMilliohm);
    if (eResult != FOC_RESULT_OK) {
        _identify_resistance_Fail(ptThis, ptMotor, eResult);
        return fsm_rt_err;
    }
    {
        perfc_global_interrupt_status_t tIrqState =
            perfc_port_disable_global_interrupt();

        ptResistance->bMotorStarted = false;
        ptResistance->bResultPending = true;
        ptResistance->eState = IDENTIFY_RESISTANCE_STATE_IDLE;
        ptThis->eLastResult = FOC_RESULT_OK;
        ptThis->eOperation = IDENTIFY_OPERATION_NONE;
        ptThis->eState = IDENTIFY_STATE_IDLE;
        perfc_port_resume_global_interrupt(tIrqState);
    }
    motor_Stop(ptMotor);
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
 * @param ptResult Output measurement and scaling diagnostics.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or an argument error.
 */
foc_result_t identify_GetResistance(identify_t *ptThis,
                                    identify_resistance_result_t *ptResult)
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
#if FOC_APP_LOG_RESISTANCE_ID
    ptResult->aqVoltageLevelPu[0U] =
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU;
    ptResult->aqVoltageLevelPu[1U] =
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU;
    ptResult->aqAverageVoltageDPu[0U] =
        ptThis->tResistance.aqAverageVoltageD[0U];
    ptResult->aqAverageVoltageDPu[1U] =
        ptThis->tResistance.aqAverageVoltageD[1U];
    ptResult->aqAverageCurrentPu[0U] =
        ptThis->tResistance.aqAverageCurrent[0U];
    ptResult->aqAverageCurrentPu[1U] =
        ptThis->tResistance.aqAverageCurrent[1U];
    ptResult->qDeltaCurrentPu = ptThis->tResistance.qDeltaCurrent;
    ptResult->wVoltageBaseMillivolt =
        ptThis->tResistance.wVoltageBaseMillivolt;
    ptResult->wCurrentBaseMilliamp =
        ptThis->tResistance.wCurrentBaseMilliamp;
#endif
    ptResult->wResistanceMilliohm =
        ptThis->tResistance.wResistanceMilliohm;
    ptThis->tResistance.bResultPending = false;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
