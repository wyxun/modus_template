/****************************************************************************
 * @file    identify_inductance.c
 * @brief   Phase 1 d-axis incremental-inductance identification.
 ****************************************************************************/

#include "identify_inductance.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "perf_counter.h"

#define IDENTIFY_INDUCTANCE_TIMEOUT_MARGIN_MS (500U)

typedef struct {
    uint16_t hwHalfCount;
    uint64_t ullVoltageSumMillivolt;
#if defined(FOC_NUMERIC_FLOAT)
    float fDeltaSum;
    float fCurrentSum;
#else
    int64_t lDeltaSum;
    int64_t lCurrentSum;
#endif
} identify_inductance_polarity_t;

typedef struct {
    int64_t lInductanceMicroHenry;
    int64_t lVoltageMillivolt;
    int64_t lCurrentMilliamp;
} identify_inductance_calculated_t;

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
        ptConfig->hwMotionFaultCycles == 0U ||
        ptConfig->wInjectionFrequencyHz > (FOC_HF_ISR_HZ / 2U) ||
        ptConfig->hwCaptureDelayCycles <
            (FOC_IDENTIFY_COMMAND_PIPELINE_CYCLES +
             FOC_DCBUS_SAMPLE_DELAY_CYCLES)) {
        return false;
    }
    if ((FOC_HF_ISR_HZ % (2U * ptConfig->wInjectionFrequencyHz)) != 0U ||
        (uint64_t)ptConfig->hwCaptureSampleCount *
            FOC_DCBUS_MAX_MILLIVOLT > UINT32_MAX) {
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
        !foc_scalar_is_finite(ptConfig->qMaxElectricalSpeedPu)) {
        return false;
    }
    if (ptConfig->qModulationAmplitude <= FOC_ZERO ||
        ptConfig->qModulationAmplitude > FOC_ONE ||
        ptConfig->qMaxIdentificationCurrent <= FOC_ZERO ||
        ptConfig->qMaxIdentificationCurrent > FOC_ONE ||
        ptConfig->qMinCurrentDelta <= FOC_ZERO ||
        ptConfig->qMinCurrentDelta >= ptConfig->qMaxIdentificationCurrent ||
        ptConfig->qMaxElectricalSpeedPu <= FOC_ZERO ||
        ptConfig->qMaxElectricalSpeedPu > FOC_ONE) {
        return false;
    }
    *pwHalfPeriod = wHalfPeriod;
    return true;
}

static bool _identify_inductance_BusConfigured(void)
{
#if FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_NOMINAL
    return FOC_DCBUS_NOMINAL_MILLIVOLT != 0U;
#elif FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_ADC
    return true;
#else
    return false;
#endif
}

static void _identify_inductance_Prepare(
    const identify_inductance_cfg_t *ptConfig,
    uint32_t wHalfPeriod,
    identify_inductance_t *ptInductance)
{
    ptInductance->wHalfPeriodCycles = wHalfPeriod;
    ptInductance->wCaptureStartCycle = ptConfig->hwCaptureDelayCycles;
    ptInductance->wTimeoutMs = _identify_inductance_TimeoutMs(
        wHalfPeriod, ptConfig->hwHalfCycleCount);
    ptInductance->atCommand[0].qD = ptConfig->qModulationAmplitude;
    ptInductance->atCommand[1].qD = -ptConfig->qModulationAmplitude;
    ptInductance->qMaxCurrent = ptConfig->qMaxIdentificationCurrent;
    ptInductance->qMaxSpeed = ptConfig->qMaxElectricalSpeedPu;
    ptInductance->qMinDelta = ptConfig->qMinCurrentDelta;
    ptInductance->hwCaptureTarget = ptConfig->hwCaptureSampleCount;
    ptInductance->hwHalfCycleTarget = ptConfig->hwHalfCycleCount;
    ptInductance->hwMotionFaultLimit = ptConfig->hwMotionFaultCycles;
#if defined(FOC_NUMERIC_FLOAT)
    ptInductance->fVoltageScale = foc_to_float(
        ptConfig->qModulationAmplitude) *
        (float)FOC_DCBUS_MODULATION_NUM /
        (float)FOC_DCBUS_MODULATION_DEN;
#endif
}

static void _identify_inductance_ResetCapture(identify_t *ptThis)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;

    ptInductance->hwCaptureSampleCount = 0U;
    ptInductance->wWindowVoltageSumMillivolt = 0U;
    ptInductance->qFirstCurrent = FOC_ZERO;
    ptInductance->qLastCurrent = FOC_ZERO;
#if defined(FOC_NUMERIC_FLOAT)
    ptInductance->fWindowCurrentSum = 0.0f;
#else
    ptInductance->lWindowCurrentSum = 0;
#endif
}

static void _identify_inductance_FailIsr(identify_t *ptThis,
                                         motor_t *ptMotor,
                                         foc_result_t eResult)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;

    if (!ptInductance->bError) {
        ptInductance->eIsrResult = eResult;
    }
    ptInductance->bError = true;
    ptInductance->bActive = false;
    __perfc_sync_barrier__();
    ptInductance->bBatchReady = true;
    motor_IdentificationAbortIsr(ptMotor, MOTOR_FAULT_IDENTIFICATION);
}

static foc_result_t _identify_inductance_FailForeground(
    identify_t *ptThis,
    motor_t *ptMotor,
    foc_result_t eResult)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;

    if (!ptInductance->bError) {
        ptInductance->eIsrResult = eResult;
    }
    ptInductance->bError = true;
    ptInductance->bActive = false;
    ptInductance->bBatchReady = false;
    if (ptMotor != NULL) {
        motor_Stop(ptMotor);
    }
    eResult = ptInductance->eIsrResult;
    ptThis->eLastResult = eResult;
    ptThis->eState = IDENTIFY_STATE_ERROR;
    ptThis->eOperation = IDENTIFY_OPERATION_NONE;
    return eResult;
}

static void _identify_inductance_AddWindowCurrent(
    identify_inductance_t *ptInductance,
    foc_scalar_t qCurrent)
{
#if defined(FOC_NUMERIC_FLOAT)
    ptInductance->fWindowCurrentSum += qCurrent;
#else
    ptInductance->lWindowCurrentSum += (int64_t)qCurrent;
#endif
}

static __attribute__((always_inline)) inline void
_identify_inductance_SaveHalfCycle(identify_t *ptThis)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;
    bool bPositive = (ptInductance->hwHalfCycle & 1U) == 0U;

#if defined(FOC_NUMERIC_FLOAT)
    float fDelta = ptInductance->qLastCurrent -
                   ptInductance->qFirstCurrent;
    if (bPositive) {
        ptInductance->fPositiveDeltaSum += fDelta;
        ptInductance->fPositiveCurrentSum +=
            ptInductance->fWindowCurrentSum;
    } else {
        ptInductance->fNegativeDeltaSum += fDelta;
        ptInductance->fNegativeCurrentSum +=
            ptInductance->fWindowCurrentSum;
    }
#else
    int64_t lDelta = (int64_t)ptInductance->qLastCurrent -
                     (int64_t)ptInductance->qFirstCurrent;
    if (bPositive) {
        ptInductance->lPositiveDeltaSum += lDelta;
        ptInductance->lPositiveCurrentSum +=
            ptInductance->lWindowCurrentSum;
    } else {
        ptInductance->lNegativeDeltaSum += lDelta;
        ptInductance->lNegativeCurrentSum +=
            ptInductance->lWindowCurrentSum;
    }
#endif
    if (bPositive) {
        ptInductance->hwPositiveHalfCount++;
        ptInductance->ullPositiveVoltageSumMillivolt +=
            ptInductance->wWindowVoltageSumMillivolt;
    } else {
        ptInductance->hwNegativeHalfCount++;
        ptInductance->ullNegativeVoltageSumMillivolt +=
            ptInductance->wWindowVoltageSumMillivolt;
    }
}

static bool _identify_inductance_MotorReady(const motor_t *ptMotor)
{
    motor_status_t tStatus = {0};

    if (motor_GetStatus(ptMotor, &tStatus) != FOC_RESULT_OK) {
        return false;
    }
    return tStatus.eState == MOTOR_STATE_RUNNING && tStatus.bPwmEnabled &&
           tStatus.eMode == FOC_MODE_VOLTAGE;
}

static bool _identify_inductance_CopyBatch(
    identify_t *ptThis,
    identify_inductance_polarity_t *ptPositive,
    identify_inductance_polarity_t *ptNegative)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;

    if (!ptInductance->bBatchReady) {
        return false;
    }
    __perfc_sync_barrier__();
    ptPositive->hwHalfCount = ptInductance->hwPositiveHalfCount;
    ptNegative->hwHalfCount = ptInductance->hwNegativeHalfCount;
    ptPositive->ullVoltageSumMillivolt =
        ptInductance->ullPositiveVoltageSumMillivolt;
    ptNegative->ullVoltageSumMillivolt =
        ptInductance->ullNegativeVoltageSumMillivolt;
#if defined(FOC_NUMERIC_FLOAT)
    ptPositive->fDeltaSum = ptInductance->fPositiveDeltaSum;
    ptNegative->fDeltaSum = ptInductance->fNegativeDeltaSum;
    ptPositive->fCurrentSum = ptInductance->fPositiveCurrentSum;
    ptNegative->fCurrentSum = ptInductance->fNegativeCurrentSum;
#else
    ptPositive->lDeltaSum = ptInductance->lPositiveDeltaSum;
    ptNegative->lDeltaSum = ptInductance->lNegativeDeltaSum;
    ptPositive->lCurrentSum = ptInductance->lPositiveCurrentSum;
    ptNegative->lCurrentSum = ptInductance->lNegativeCurrentSum;
#endif
    ptInductance->bBatchReady = false;
    return true;
}

static foc_result_t _identify_inductance_CalculatePolarity(
    const identify_inductance_polarity_t *ptPolarity,
    const identify_inductance_t *ptInductance,
    bool bPositive,
    identify_inductance_calculated_t *ptResult)
{
    uint32_t wSamplesPerPolarity = 0U;

    if (ptPolarity == NULL || ptInductance == NULL || ptResult == NULL ||
        ptPolarity->hwHalfCount < 2U ||
        ptInductance->hwCaptureTarget == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wSamplesPerPolarity = (uint32_t)ptPolarity->hwHalfCount *
                          ptInductance->hwCaptureTarget;
#if defined(FOC_NUMERIC_FLOAT)
    {
        float fDelta = ptPolarity->fDeltaSum /
                       (float)ptPolarity->hwHalfCount;
        float fCurrent = ptPolarity->fCurrentSum /
                          (float)wSamplesPerPolarity;
        float fVoltage = (float)ptPolarity->ullVoltageSumMillivolt /
                          (float)wSamplesPerPolarity *
                          ptInductance->fVoltageScale;
        float fMeanCurrent = fCurrent *
                              (float)ptInductance->wCurrentBaseMilliamp;
        float fNet = (bPositive ? fVoltage : -fVoltage) -
                      (float)ptInductance->wResistanceMilliohm *
                      fMeanCurrent / 1000.0f;
        float fSlope = fDelta * ptInductance->fSlopeScale;
        if (fabsf(fDelta) < foc_to_float(ptInductance->qMinDelta) ||
            fNet * fSlope <= 0.0f || fSlope == 0.0f) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        fNet = fNet * 1000000.0f / fSlope;
        if (!isfinite(fNet) || fNet <= 0.0f ||
            fNet > (float)INT64_MAX) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        ptResult->lInductanceMicroHenry = (int64_t)(fNet + 0.5f);
        ptResult->lVoltageMillivolt = (int64_t)(fVoltage + 0.5f);
        ptResult->lCurrentMilliamp = (int64_t)(fMeanCurrent +
                                               (fMeanCurrent >= 0.0f ?
                                                0.5f : -0.5f));
    }
#else
    {
        int64_t lSamples = (int64_t)wSamplesPerPolarity;
        int64_t lDelta = ptPolarity->lDeltaSum /
                         (int64_t)ptPolarity->hwHalfCount;
        int64_t lCurrent = ptPolarity->lCurrentSum / lSamples;
        int64_t lBus = (int64_t)(ptPolarity->ullVoltageSumMillivolt /
                                 (uint64_t)lSamples);
        int64_t lVoltage = lBus * ptInductance->atCommand[0].qD /
                            FOC_Q_SCALE;
        lVoltage = lVoltage * FOC_DCBUS_MODULATION_NUM /
                   FOC_DCBUS_MODULATION_DEN;
        int64_t lMeanCurrent = lCurrent *
                                ptInductance->wCurrentBaseMilliamp /
                                FOC_Q_SCALE;
        int64_t lNet = (bPositive ? lVoltage : -lVoltage) -
                        ((int64_t)ptInductance->wResistanceMilliohm *
                         lMeanCurrent / 1000);
        int64_t lSlope = lDelta *
                          ptInductance->wCurrentBaseMilliamp *
                          FOC_HF_ISR_HZ /
                          (FOC_Q_SCALE *
                           (ptInductance->hwCaptureTarget - 1U));
        if ((lDelta < 0 ? -lDelta : lDelta) < ptInductance->qMinDelta ||
            lSlope == 0 ||
            (lNet > 0) != (lSlope > 0)) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        lNet = lNet * 1000000 / lSlope;
        if (lNet <= 0) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        ptResult->lInductanceMicroHenry = lNet;
        ptResult->lVoltageMillivolt = lVoltage;
        ptResult->lCurrentMilliamp = lMeanCurrent;
    }
#endif
    return FOC_RESULT_OK;
}

static foc_result_t _identify_inductance_Calculate(
    identify_t *ptThis,
    identify_inductance_result_t *ptOutput)
{
    identify_inductance_polarity_t tPositive = {0};
    identify_inductance_polarity_t tNegative = {0};
    identify_inductance_calculated_t tPositiveResult = {0};
    identify_inductance_calculated_t tNegativeResult = {0};
    const identify_inductance_t *ptInductance = &ptThis->tInductance;

    if (ptOutput == NULL || !_identify_inductance_CopyBatch(
            ptThis, &tPositive, &tNegative) ||
        _identify_inductance_CalculatePolarity(
            &tPositive, ptInductance, true, &tPositiveResult) != FOC_RESULT_OK ||
        _identify_inductance_CalculatePolarity(
            &tNegative, ptInductance, false, &tNegativeResult) != FOC_RESULT_OK) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    if (tPositiveResult.lInductanceMicroHenry <= 0 ||
        tNegativeResult.lInductanceMicroHenry <= 0 ||
        ((uint64_t)tPositiveResult.lInductanceMicroHenry +
         (uint64_t)tNegativeResult.lInductanceMicroHenry) / 2U >
            UINT32_MAX) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptOutput->wInductanceDMicroHenry = (uint32_t)(
        ((uint64_t)tPositiveResult.lInductanceMicroHenry +
         (uint64_t)tNegativeResult.lInductanceMicroHenry) / 2U);
    ptOutput->wEffectiveVoltageMillivolt = (uint32_t)(
        ((uint64_t)tPositiveResult.lVoltageMillivolt +
         (uint64_t)tNegativeResult.lVoltageMillivolt) / 2U);
    ptOutput->lMeanCurrentMilliamp = (int32_t)(
        (tPositiveResult.lCurrentMilliamp +
         tNegativeResult.lCurrentMilliamp) / 2);
    ptOutput->wInjectionFrequencyHz = FOC_HF_ISR_HZ /
        (2U * ptInductance->wHalfPeriodCycles);
    ptOutput->hwCaptureSampleCount = ptInductance->hwCaptureTarget;
    ptOutput->hwHalfCycleCount = ptInductance->hwHalfCycleTarget;
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
    if (!_identify_inductance_BusConfigured()) {
        return FOC_RESULT_DISABLED;
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

void _identify_inductance_IsrStep(
    identify_t *ptThis,
    motor_t *ptMotor,
    const identify_isr_sample_t *ptSample)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;

    if (!ptInductance->bActive || ptInductance->bBatchReady) {
        return;
    }
    if (ptSample->bMotorFault || !ptSample->bAngleValid ||
        !ptSample->bDcBusValid ||
        ptSample->wDcBusMillivolt > FOC_DCBUS_MAX_MILLIVOLT ||
        ptSample->bPwmSaturated ||
        ptSample->qCurrentD > ptInductance->qMaxCurrent ||
        ptSample->qCurrentD < -ptInductance->qMaxCurrent ||
        ptSample->qCurrentQ > ptInductance->qMaxCurrent ||
        ptSample->qCurrentQ < -ptInductance->qMaxCurrent) {
        _identify_inductance_FailIsr(ptThis, ptMotor, FOC_RESULT_SAFETY);
        return;
    }
    if (ptSample->qElectricalSpeedPu > ptInductance->qMaxSpeed ||
        ptSample->qElectricalSpeedPu < -ptInductance->qMaxSpeed) {
        ptInductance->hwMotionFaultCount++;
        if (ptInductance->hwMotionFaultCount >=
            ptInductance->hwMotionFaultLimit) {
            _identify_inductance_FailIsr(ptThis, ptMotor,
                                         FOC_RESULT_SAFETY);
        }
        return;
    }
    ptInductance->hwMotionFaultCount = 0U;
    if (ptInductance->wPhaseCycle == 0U &&
        motor_IdentificationApplyIsr(
            ptMotor, &ptInductance->atCommand[
                ptInductance->hwHalfCycle & 1U]) != FOC_RESULT_OK) {
        _identify_inductance_FailIsr(ptThis, ptMotor, FOC_RESULT_SAFETY);
        return;
    }
    if ((ptInductance->wPhaseCycle - ptInductance->wCaptureStartCycle) <
        ptInductance->hwCaptureTarget) {
        if (ptInductance->hwCaptureSampleCount == 0U) {
            ptInductance->qFirstCurrent = ptSample->qCurrentD;
        }
        ptInductance->qLastCurrent = ptSample->qCurrentD;
        _identify_inductance_AddWindowCurrent(ptInductance,
                                              ptSample->qCurrentD);
        ptInductance->wWindowVoltageSumMillivolt +=
            ptSample->wDcBusMillivolt;
        ptInductance->hwCaptureSampleCount++;
    }
    if (ptInductance->wPhaseCycle + 1U >=
        ptInductance->wHalfPeriodCycles) {
        if (ptInductance->hwCaptureSampleCount !=
            ptInductance->hwCaptureTarget) {
            _identify_inductance_FailIsr(ptThis, ptMotor,
                                         FOC_RESULT_SAFETY);
            return;
        }
        _identify_inductance_SaveHalfCycle(ptThis);
        ptInductance->hwHalfCycle++;
        ptInductance->wPhaseCycle = 0U;
        _identify_inductance_ResetCapture(ptThis);
        if (ptInductance->hwHalfCycle >= ptInductance->hwHalfCycleTarget) {
            if (motor_IdentificationApplyIsr(
                    ptMotor, &(foc_dq_t){FOC_ZERO, FOC_ZERO}) != FOC_RESULT_OK) {
                _identify_inductance_FailIsr(
                    ptThis, ptMotor, FOC_RESULT_SAFETY);
                return;
            }
            ptInductance->bActive = false;
            __perfc_sync_barrier__();
            ptInductance->bBatchReady = true;
        }
        return;
    }
    ptInductance->wPhaseCycle++;
}

fsm_rt_t _identify_inductance_RunPt(identify_t *ptThis,
                                     motor_t *ptMotor)
{
    identify_inductance_t *ptInductance = &ptThis->tInductance;
    motor_status_t tStatus = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    PERFC_PT_BEGIN(ptInductance->chRunPt)
    eResult = motor_GetStatus(ptMotor, &tStatus);
    if (eResult != FOC_RESULT_OK || tStatus.eState != MOTOR_STATE_IDLE ||
        tStatus.bPwmEnabled || tStatus.wFaults != MOTOR_FAULT_NONE ||
        !tStatus.bElectricalZeroValid ||
        ptInductance->atCommand[0].qD >
            ptMotor->tLimits.qMaxModulation ||
        ptInductance->qMaxCurrent > ptMotor->tLimits.qMaxPhaseCurrent) {
        eResult = eResult == FOC_RESULT_OK ? FOC_RESULT_SAFETY : eResult;
        _identify_inductance_FailForeground(ptThis, ptMotor, eResult);
    } else if (ptMotor->wCurrentBaseMilliamp == 0U ||
               ptMotor->tParams.wResistanceMilliohm == 0U) {
        _identify_inductance_FailForeground(ptThis, ptMotor,
                                            FOC_RESULT_INVALID_ARGUMENT);
    } else {
        ptInductance->wCurrentBaseMilliamp = ptMotor->wCurrentBaseMilliamp;
        ptInductance->wResistanceMilliohm =
            ptMotor->tParams.wResistanceMilliohm;
#if defined(FOC_NUMERIC_FLOAT)
        ptInductance->fSlopeScale =
            (float)ptInductance->wCurrentBaseMilliamp * FOC_HF_ISR_HZ /
            (float)(ptInductance->hwCaptureTarget - 1U);
#endif
        eResult = motor_Start(ptMotor, FOC_MODE_VOLTAGE);
        if (eResult != FOC_RESULT_OK) {
            _identify_inductance_FailForeground(ptThis, ptMotor, eResult);
        } else {
            ptInductance->bActive = true;
        }
    }
    PERFC_PT_WAIT_UNTIL(ptInductance->bError ||
                        ptInductance->bBatchReady ||
                        !_identify_inductance_MotorReady(ptMotor) ||
                         perfc_is_time_out_ms(ptInductance->wTimeoutMs,
                                             &ptInductance->lStateTimestamp,
                                             false))
    if (ptInductance->bError) {
        eResult = ptInductance->eIsrResult;
    } else if (!_identify_inductance_MotorReady(ptMotor)) {
        eResult = FOC_RESULT_SAFETY;
    } else if (!ptInductance->bBatchReady) {
        eResult = FOC_RESULT_BUSY;
    } else {
        eResult = _identify_inductance_Calculate(
            ptThis, &ptInductance->tResult);
    }
    if (eResult != FOC_RESULT_OK) {
        _identify_inductance_FailForeground(ptThis, ptMotor, eResult);
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

foc_result_t identify_GetInductance(
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
