/****************************************************************************
 * @file    foc_app.c
 * @brief   MODUS composition and scheduling class for one FOC Motor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_app.h"
#include "foc_debug.h"

#include "foc_port.h"

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
#define FOC_APP_INIT_OBSERVER_CONFIG                         \
    .tObserverCfg = {                                                       \
        .tSmo = {                                                           \
            .wBemfCutoffRadiansPerSecond =                                  \
                MOTOR_CONFIG_SMO_BEMF_CUTOFF_RADIANS_PER_SECOND,            \
            .wSlidingGainMillivolt =                                        \
                MOTOR_CONFIG_SMO_SLIDING_GAIN_MILLIVOLT,                    \
            .qCurrentEstimateLimit = FOC_SCALAR(                            \
                MOTOR_CONFIG_SMO_CURRENT_ESTIMATE_LIMIT_PU),                \
        },                                                                  \
    },
#else
#define FOC_APP_INIT_OBSERVER_CONFIG
#endif

#if FOC_PORT_HAS_POSITION
#if !defined(FOC_ENCODER_STATIC_BINDING)
extern const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface;
#define FOC_APP_INIT_ENCODER_CONFIG                    \
    .tEncoderCfg = {                                                  \
        .qSpeedFilterAlpha = FOC_SCALAR(                              \
            MOTOR_CONFIG_ENCODER_SPEED_FILTER_ALPHA),                 \
        .fInvalidTimeoutSeconds = \
            MOTOR_CONFIG_ENCODER_INVALID_TIMEOUT_SECONDS,             \
        .bDirectionInvert = MOTOR_CONFIG_ENCODER_DIRECTION_INVERT,    \
        .ptSensor = &g_tFocEncoderSensorInterface,                    \
    },
#else
#define FOC_APP_INIT_ENCODER_CONFIG                    \
    .tEncoderCfg = {                                                  \
        .qSpeedFilterAlpha = FOC_SCALAR(                              \
            MOTOR_CONFIG_ENCODER_SPEED_FILTER_ALPHA),                 \
        .fInvalidTimeoutSeconds = \
            MOTOR_CONFIG_ENCODER_INVALID_TIMEOUT_SECONDS,             \
        .bDirectionInvert = MOTOR_CONFIG_ENCODER_DIRECTION_INVERT,    \
    },
#endif
#else
#define FOC_APP_INIT_ENCODER_CONFIG \
    .tEncoderCfg = {0},
#endif

#define FOC_APP_INIT_CONFIG                                 \
    .tMotorCfg = FOC_APP_INIT_MOTOR_CONFIG,                 \
    .ePositionSource = MOTOR_CONFIG_POSITION_SOURCE,       \
    FOC_APP_INIT_OBSERVER_CONFIG                            \
    FOC_APP_INIT_ENCODER_CONFIG                             \
    .wVoltageBaseMillivolt = MOTOR_CONFIG_BASE_VOLTAGE_MILLIVOLT,          \
    .wHighFrequencyIsrHz = FOC_HF_ISR_HZ,                   \
    .qElectricalSpeedBaseTurnsPerSecond =                                  \
        FOC_SCALAR(MOTOR_CONFIG_BASE_ELECTRICAL_HZ),



#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "foc_config.h"
#include "internal/foc_units.h"
#include "mdebug/util_debug.h"
#include "perf_counter.h"
#include "perfc_task_pt.h"

static modus_base_t s_tFocAppBase;
extern foc_app_t tFocApp;
static int foc_app_Clock(uintptr_t wObjectAddr);
static int foc_app_Run(uintptr_t wObjectAddr);
#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
static bool foc_app_GetHfAverage(foc_app_t *ptThis,
                                 uint32_t *pwAverageTicks);
static void foc_app_ReportHfAverage(foc_app_t *ptThis);
#endif
#if (FOC_APP_LOG_TIMING_DIAGNOSTICS || \
     (FOC_APP_LOG_SMO_DIAGNOSTICS && \
      FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO)) && \
    !defined(__NO_USE_LOG__)
static void foc_app_ReportDiagnostics(foc_app_t *ptThis);
#endif

#if FOC_APP_LOG_SMO_DIAGNOSTICS && \
    FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO && \
    !defined(__NO_USE_LOG__)
/**
 * @brief Accumulate same-cycle SMO signal and tracking quality.
 * @param ptThis Application object owning the ISR statistics.
 * @param ptSample Current sample used by the observer.
 * @return None.
 * @note Called only from the HF ISR; no logging or square root here.
 */
static void foc_app_AccumulateSmoDiagnostic(
    foc_app_t *ptThis, const motor_position_sample_t *ptSample)
{
    const foc_smo_t *ptSmo = &ptThis->tPosition.tObserver.tSmo;
    float fBemfAlpha = 0.0f;
    float fBemfBeta = 0.0f;
    float fCurrentErrorAlpha = 0.0f;
    float fCurrentErrorBeta = 0.0f;
    float fBemfSquare = 0.0f;
    float fCurrentErrorSquare = 0.0f;
    foc_scalar_t qAngleError = FOC_ZERO;
    uint32_t wBin = 0U;
    uint32_t wSector = 0U;

    if (ptThis->tMotor.eState != MOTOR_STATE_RUNNING ||
        !ptThis->tMotor.tInput.bAngleValid ||
        !ptThis->tPosition.tObserver.tOutput.bValid) {
        return;
    }
    fBemfAlpha = foc_to_float(ptSmo->tAxis[0].qBemf);
    fBemfBeta = foc_to_float(ptSmo->tAxis[1].qBemf);
    fCurrentErrorAlpha =
        foc_to_float(ptSmo->tAxis[0].qCurrentEstimate) -
        foc_to_float(ptSample->tCurrentAlphaBeta.qAlpha);
    fCurrentErrorBeta =
        foc_to_float(ptSmo->tAxis[1].qCurrentEstimate) -
        foc_to_float(ptSample->tCurrentAlphaBeta.qBeta);
    qAngleError = foc_angle_diff(
        ptThis->tPosition.tObserver.tOutput.tElectricalAngle,
        ptThis->tMotor.tInput.tElectricalAngle);
    fBemfSquare = fBemfAlpha * fBemfAlpha +
                  fBemfBeta * fBemfBeta;
    fCurrentErrorSquare =
        fCurrentErrorAlpha * fCurrentErrorAlpha +
        fCurrentErrorBeta * fCurrentErrorBeta;
    ptThis->tHfStats.fBemfSquareTotal += fBemfSquare;
    ptThis->tHfStats.fCurrentErrorSquareTotal += fCurrentErrorSquare;
    ptThis->tHfStats.wSmoDiagnosticSampleCount++;
    wBin = fBemfSquare < 0.0025f ? 0U :
           fBemfSquare < 0.0081f ? 1U :
           fBemfSquare < 0.0169f ? 2U : 3U;
    wSector = (uint32_t)(
        ptThis->tMotor.tInput.tElectricalAngle.wBam32 >> 29);
    ptThis->tHfStats.awSmoBinSampleCount[wBin]++;
    if (wBin == 0U) {
        ptThis->tHfStats.awSmoSectorLowCount[wSector]++;
    }
    if (foc_abs(qAngleError) > FOC_SCALAR(0.25f)) {
        ptThis->tHfStats.wSmoLargeAngleErrorCount++;
        ptThis->tHfStats.fSmoBadBemfSquareTotal += fBemfSquare;
        ptThis->tHfStats.fSmoBadCurrentSquareTotal +=
            fCurrentErrorSquare;
        ptThis->tHfStats.awSmoBinBadCount[wBin]++;
        ptThis->tHfStats.awSmoSectorBadCount[wSector]++;
    }
}
#endif

static uint32_t foc_app_GetControlPeriodNanoseconds(uint32_t wFrequencyHz)
{
    if (wFrequencyHz == 0U ||
        wFrequencyHz > FOC_NANOSECONDS_PER_SECOND) {
        return 0U;
    }
    return (FOC_NANOSECONDS_PER_SECOND + (wFrequencyHz / 2U)) /
           wFrequencyHz;
}

static foc_result_t foc_app_SampleDcBusMillivolt(uint32_t *pwMillivolt)
{
    if (pwMillivolt == NULL) {
        return FOC_RESULT_NULL;
    }
#if FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_NOMINAL
    *pwMillivolt = FOC_DCBUS_NOMINAL_MILLIVOLT;
#elif FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_ADC
    uint32_t wRaw = FOC_PORT_SAMPLE_DCBUS_RAW(
        FOC_PORT_ADC_CHANNEL_DCBUS);
    int64_t llMillivolt = 0;

    if (wRaw == FOC_PORT_ADC_SAMPLE_INVALID ||
        FOC_DCBUS_MV_PER_COUNT_DEN == 0U) {
        return FOC_RESULT_SAFETY;
    }
    llMillivolt = ((int64_t)wRaw * FOC_DCBUS_MV_PER_COUNT_NUM) /
                  FOC_DCBUS_MV_PER_COUNT_DEN +
                  FOC_DCBUS_OFFSET_MILLIVOLT;
    if (llMillivolt <= 0 ||
        llMillivolt > FOC_DCBUS_MAX_MILLIVOLT) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    *pwMillivolt = (uint32_t)llMillivolt;
#else
    return FOC_RESULT_DISABLED;
#endif
    return (*pwMillivolt > 0U &&
            *pwMillivolt <= FOC_DCBUS_MAX_MILLIVOLT)
        ? FOC_RESULT_OK : FOC_RESULT_OUT_OF_RANGE;
}

static modus_base_cfg_t s_tFocAppBaseCfg = {
    .wId = FOC_APP,
    .wParent = 0U,
    .pchRingBuffer = NULL,
    .hwRingSize = 0U,
    .FcnInterface = {
        .Clock = foc_app_Clock,
        .Run = foc_app_Run,
    },
};

/**
 * @brief Scale one speed PID coefficient from turns/s to PU input.
 * @param ptGain PID gain to convert in place.
 * @param qSpeedBase Electrical speed base in turns/s.
 * @return FOC_RESULT_OK or a numeric range error.
 */
static foc_result_t foc_app_ScaleSpeedGain(foc_gain_t *ptGain,
                                           foc_scalar_t qSpeedBase)
{
    if (ptGain == NULL || !foc_gain_IsValid(ptGain) ||
        qSpeedBase <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if defined(FOC_NUMERIC_FIXED)
    {
        int64_t llGainQ = ((int64_t)ptGain->nInteger * FOC_Q_SCALE) +
                          (int64_t)ptGain->qFraction;
        int64_t llScaledGainQ = (llGainQ * (int64_t)qSpeedBase) /
                                FOC_Q_SCALE;
        int64_t llInteger = llScaledGainQ / FOC_Q_SCALE;
        int64_t llFraction = llScaledGainQ -
                             (llInteger * FOC_Q_SCALE);

        if (llInteger > INT16_MAX || llInteger < INT16_MIN) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        ptGain->nInteger = (int16_t)llInteger;
        ptGain->qFraction = (foc_scalar_t)llFraction;
    }
#else
    {
        float fGain = ((float)ptGain->nInteger + ptGain->qFraction) *
                      qSpeedBase;

        if (!isfinite(fGain)) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        return foc_gain_from_float(fGain, ptGain);
    }
#endif
    return FOC_RESULT_OK;
}

/**
 * @brief Build the Motor config bound to this App's Encoder.
 * @param ptConfig App configuration.
 * @param ptMotorConfig Output Motor configuration.
 * @param bEncoderReady Whether the Encoder was initialized.
 * @return FOC_RESULT_OK or an invalid speed configuration.
 */
static foc_result_t foc_app_BindMotorConfig(
    const foc_app_cfg_t *ptConfig,
    motor_cfg_t *ptMotorConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;

    *ptMotorConfig = ptConfig->tMotorCfg;
    ptMotorConfig->tParams.wVoltageBaseMillivolt =
        ptConfig->wVoltageBaseMillivolt;
    ptMotorConfig->qElectricalSpeedBaseTurnsPerSecond =
        ptConfig->qElectricalSpeedBaseTurnsPerSecond;
    ptMotorConfig->wControlFrequencyHz =
        ptConfig->wHighFrequencyIsrHz;
    eResult = foc_div_checked(
        ptMotorConfig->tLimits.qMaxSpeedReference,
        ptConfig->qElectricalSpeedBaseTurnsPerSecond,
        &ptMotorConfig->tLimits.qMaxSpeedReference);
    if (eResult != FOC_RESULT_OK ||
        ptMotorConfig->tLimits.qMaxSpeedReference <= FOC_ZERO ||
        ptMotorConfig->tLimits.qMaxSpeedReference > FOC_ONE) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    eResult = foc_app_ScaleSpeedGain(
        &ptMotorConfig->tSpeedPiParams.tKp,
        ptConfig->qElectricalSpeedBaseTurnsPerSecond);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_app_ScaleSpeedGain(
            &ptMotorConfig->tSpeedPiParams.tKiTs,
            ptConfig->qElectricalSpeedBaseTurnsPerSecond);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_app_ScaleSpeedGain(
            &ptMotorConfig->tSpeedPiParams.tKdOverTs,
            ptConfig->qElectricalSpeedBaseTurnsPerSecond);
    }
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    return FOC_RESULT_OK;
}

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    foc_app_cfg_t *ptConfig = (foc_app_cfg_t *)wObjectCfgAddr;
    motor_cfg_t tMotorConfig = {0};
    motor_position_cfg_t tPositionConfig = {0};
    foc_encoder_cfg_t tEncoderConfig = {0};
    uint32_t wControlPeriodNanoseconds = 0U;
    foc_result_t eEncoder = FOC_RESULT_OK;
    foc_result_t eResult = FOC_RESULT_OK;
    foc_result_t eMotor = FOC_RESULT_OK;
    int nBaseResult = MODUS_SUCCESS;

    if (ptThis == NULL || ptConfig == NULL) {
        return MODUS_EFAIL;
    }
    *ptThis = (foc_app_t){0};
    ptThis->ptBase = &s_tFocAppBase;
    s_tFocAppBaseCfg.wParent = wObjectAddr;
    ptThis->chRunPt = 0U;
    ptThis->lForegroundTimestamp = 0;
    ptThis->bEncoderEnabled = false;
    ptThis->bReady = false;

    if (ptConfig->ePositionSource == MOTOR_POSITION_SOURCE_SENSOR) {
        tEncoderConfig = ptConfig->tEncoderCfg;
        eEncoder = foc_encoder_Init(&ptThis->tEncoder,
                                    &tEncoderConfig);
        if (eEncoder != FOC_RESULT_OK && eEncoder != FOC_RESULT_DISABLED) {
            return (int)eEncoder;
        }
    } else {
        eEncoder = FOC_RESULT_DISABLED;
    }
    wControlPeriodNanoseconds = foc_app_GetControlPeriodNanoseconds(
        ptConfig->wHighFrequencyIsrHz);
    if (ptConfig->wVoltageBaseMillivolt == 0U ||
        wControlPeriodNanoseconds == 0U ||
        ptConfig->qElectricalSpeedBaseTurnsPerSecond <= FOC_ZERO) {
        return MODUS_EFAIL;
    }
    eResult = foc_app_BindMotorConfig(ptConfig, &tMotorConfig);
    if (eResult != FOC_RESULT_OK) {
        return (int)eResult;
    }
    eMotor = motor_Init(&ptThis->tMotor, &tMotorConfig);
    if (eMotor != FOC_RESULT_OK) {
        return (int)eMotor;
    }
    if (eEncoder == FOC_RESULT_OK) {
        tPositionConfig.tSensor.pContext = &ptThis->tEncoder;
#if !defined(FOC_POSITION_STATIC_BINDING)
        tPositionConfig.tSensor.fnGetPosition = foc_encoder_GetPosition;
#endif
    }
    tPositionConfig.chPolePairs = tMotorConfig.tParams.chPolePairs;
    tPositionConfig.eSource = ptConfig->ePositionSource;
    tPositionConfig.qElectricalSpeedBaseTurnsPerSecond =
        ptConfig->qElectricalSpeedBaseTurnsPerSecond;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    tPositionConfig.ptMotorParams = &ptThis->tMotor.tParams;
    tPositionConfig.tObserverCfg = ptConfig->tObserverCfg;
    tPositionConfig.tObserverCfg.tSmo.wSampleFrequencyHz =
        ptConfig->wHighFrequencyIsrHz;
#endif
    if (ptConfig->ePositionSource == MOTOR_POSITION_SOURCE_HARD_DRAG &&
        tMotorConfig.nHardDragElectricalMilliHz == 0) {
        motor_Stop(&ptThis->tMotor);
        return (int)FOC_RESULT_INVALID_ARGUMENT;
    }
    if (eEncoder == FOC_RESULT_OK ||
        ptConfig->ePositionSource == MOTOR_POSITION_SOURCE_HARD_DRAG) {
        eResult = motor_position_Init(&ptThis->tPosition,
                                      &tPositionConfig);
        if (eResult != FOC_RESULT_OK) {
            motor_Stop(&ptThis->tMotor);
            return (int)eResult;
        }
    }
    eResult = identify_Init(&ptThis->tIdentify);
    if (eResult != FOC_RESULT_OK) {
        motor_Stop(&ptThis->tMotor);
        return (int)eResult;
    }
    nBaseResult = mbase_Init(ptThis->ptBase, &s_tFocAppBaseCfg);
    if (nBaseResult != MODUS_SUCCESS) {
        motor_Stop(&ptThis->tMotor);
        return nBaseResult;
    }
    ptThis->bReady = eEncoder == FOC_RESULT_OK ||
                     ptConfig->ePositionSource == MOTOR_POSITION_SOURCE_HARD_DRAG;
    ptThis->bEncoderEnabled = eEncoder == FOC_RESULT_OK;
#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
    foc_debug_WaveformInit(
        ptThis, wControlPeriodNanoseconds);
#endif
    return MODUS_SUCCESS;
}

static int foc_app_Clock(uintptr_t wObjectAddr)
{
    if (wObjectAddr == (uintptr_t)0U) {
        return MODUS_EFAIL;
    }
    return MODUS_SUCCESS;
}

/**
 * @brief Report resistance measurement inputs and the resulting resistance.
 * @param ptResult Completed resistance identification result.
 * @return None.
 */
#if FOC_APP_LOG_RESISTANCE_ID && !defined(__NO_USE_LOG__)
static void foc_app_ReportResistance(
    const identify_resistance_result_t *ptResult)
{
    float afVoltagePu[IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT] = {0.0f};
    float afCurrentPu[IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT] = {0.0f};
    float afVoltageOutputPu[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT] = {0.0f};
    float afVoltageBaseEqMillivolt[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT] = {0.0f};
    float afCurrentBaseEqMilliamp[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT] = {0.0f};
    float afVoltageOutputMillivolt[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT] = {0.0f};
    float fDeltaCurrentPu = 0.0f;
    float fDeltaCurrentMilliamp = 0.0f;
    uint8_t chLevel = 0U;

    if (ptResult == NULL) {
        return;
    }
    for (chLevel = 0U;
         chLevel < IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT;
         chLevel++) {
        afVoltagePu[chLevel] = foc_to_float(
            ptResult->aqVoltageLevelPu[chLevel]);
        afCurrentPu[chLevel] = foc_to_float(
            ptResult->aqAverageCurrentPu[chLevel]);
        afVoltageOutputPu[chLevel] = foc_to_float(
            ptResult->aqAverageVoltageDPu[chLevel]);
        afVoltageOutputMillivolt[chLevel] = afVoltageOutputPu[chLevel] *
            (float)ptResult->wVoltageBaseMillivolt;
        afCurrentBaseEqMilliamp[chLevel] = afCurrentPu[chLevel] *
            (float)ptResult->wCurrentBaseMilliamp;
    }
    fDeltaCurrentPu = foc_to_float(ptResult->qDeltaCurrentPu);
    fDeltaCurrentMilliamp = fDeltaCurrentPu *
        (float)ptResult->wCurrentBaseMilliamp;
    MLOGF(I, "identify R VdRef=%.3f/%.3f pu VdCmd=%.3f/%.3f pu\r\n",
          (double)afVoltagePu[0U], (double)afVoltagePu[1U],
          (double)afVoltageOutputPu[0U],
          (double)afVoltageOutputPu[1U]);
    MLOGF(I, "identify R Id=%.4f/%.4f pu IdEq=%.1f/%.1f mA "
          "dId=%.4f pu (%.1f mA) R=%lu mOhm bases=%lu mV/%lu mA\r\n",
          (double)afCurrentPu[0U], (double)afCurrentPu[1U],
          (double)afCurrentBaseEqMilliamp[0U],
          (double)afCurrentBaseEqMilliamp[1U],
          (double)fDeltaCurrentPu, (double)fDeltaCurrentMilliamp,
          (unsigned long)ptResult->wResistanceMilliohm,
          (unsigned long)ptResult->wVoltageBaseMillivolt,
          (unsigned long)ptResult->wCurrentBaseMilliamp);
    MLOGF(I, "identify R VdCmdEq=%.0f/%.0f mV (not measured)\r\n",
          (double)afVoltageOutputMillivolt[0U],
          (double)afVoltageOutputMillivolt[1U]);
}
#endif

#if FOC_APP_LOG_INDUCTANCE_ID && !defined(__NO_USE_LOG__)
static const char *foc_app_InductanceFailureName(
    identify_inductance_failure_t eFailure)
{
    switch (eFailure) {
    case IDENTIFY_INDUCTANCE_FAILURE_NONE:
        return "ok";
    case IDENTIFY_INDUCTANCE_FAILURE_INSUFFICIENT_SAMPLES:
        return "samples";
    case IDENTIFY_INDUCTANCE_FAILURE_DELTA_TOO_SMALL:
        return "delta-small";
    case IDENTIFY_INDUCTANCE_FAILURE_ZERO_SLOPE:
        return "slope-zero";
    case IDENTIFY_INDUCTANCE_FAILURE_VOLTAGE_SLOPE_SIGN:
        return "V-slope-sign";
    case IDENTIFY_INDUCTANCE_FAILURE_INVALID_INDUCTANCE:
        return "L-invalid";
    default:
        return "unknown";
    }
}

static void foc_app_ReportInductanceDiagnostic(
    const identify_t *ptIdentify)
{
    identify_inductance_diagnostic_t tDiagnostic = {0};
    const identify_inductance_polarity_diagnostic_t *ptPositive = NULL;
    const identify_inductance_polarity_diagnostic_t *ptNegative = NULL;

    if (identify_GetInductanceDiagnostic(
            ptIdentify, &tDiagnostic) != FOC_RESULT_OK) {
        return;
    }
    ptPositive = &tDiagnostic.tPositive;
    ptNegative = &tDiagnostic.tNegative;
    MLOGF(I, "identify Ld cfg R=%lu mOhm minDelta=%.4f pu\r\n",
          (unsigned long)tDiagnostic.wResistanceMilliohm,
          (double)foc_to_float(tDiagnostic.qMinimumDeltaPu));
    MLOGF(I, "identify Ld + dId=%.5f pu/%ld cntEq vbus=%lu mV "
          "VdCmdEq=%ld net=%ld mV I=%ld mA fail=%s\r\n",
          (double)foc_to_float(ptPositive->qDeltaCurrentPu),
          (long)ptPositive->lDeltaCurrentAdcCodeEq,
          (unsigned long)ptPositive->wAverageBusMillivolt,
          (long)ptPositive->lCommandVoltageMillivolt,
          (long)ptPositive->lNetVoltageMillivolt,
          (long)ptPositive->lAverageCurrentMilliamp,
          foc_app_InductanceFailureName(ptPositive->eFailure));
    MLOGF(I, "identify Ld - dId=%.5f pu/%ld cntEq vbus=%lu mV "
          "VdCmdEq=%ld net=%ld mV I=%ld mA fail=%s\r\n",
          (double)foc_to_float(ptNegative->qDeltaCurrentPu),
          (long)ptNegative->lDeltaCurrentAdcCodeEq,
          (unsigned long)ptNegative->wAverageBusMillivolt,
          (long)ptNegative->lCommandVoltageMillivolt,
          (long)ptNegative->lNetVoltageMillivolt,
          (long)ptNegative->lAverageCurrentMilliamp,
          foc_app_InductanceFailureName(ptNegative->eFailure));
}
#endif

#if FOC_APP_LOG_ADC_OFFSETS && !defined(__NO_USE_LOG__)
/**
 * @brief Print the calibrated three-phase ADC offsets once.
 * @param ptThis FOC App object.
 * @return None.
 * @note Runs in the foreground after calibration; snapshots with IRQs masked.
 */
static void foc_app_ReportAdcOffsets(foc_app_t *ptThis)
{
    uint32_t wOffsetU = 0U;
    uint32_t wOffsetV = 0U;
    uint32_t wOffsetW = 0U;
    bool bCalibrated = false;
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis->bAdcOffsetReported) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    bCalibrated = ptThis->tMotor.tCalib.bIsCalibrated;
    if (bCalibrated) {
        wOffsetU = ptThis->tMotor.tCalib.wOffsetU;
        wOffsetV = ptThis->tMotor.tCalib.wOffsetV;
        wOffsetW = ptThis->tMotor.tCalib.wOffsetW;
        ptThis->bAdcOffsetReported = true;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    if (!bCalibrated) {
        return;
    }
    MLOGF(I, "FOC ADC offset U/V/W=%lu/%lu/%lu counts\r\n",
          (unsigned long)wOffsetU, (unsigned long)wOffsetV,
          (unsigned long)wOffsetW);
}
#endif

static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    foc_result_t eResult = FOC_RESULT_OK;
    foc_result_t eIdentify = FOC_RESULT_OK;
    identify_resistance_result_t tResistanceResult = {0};
    identify_inductance_result_t tInductanceResult = {0};

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }
    PERFC_PT_BEGIN(ptThis->chRunPt)
    while (true) {
        PERFC_PT_WAIT_UNTIL(perfc_is_time_out_us(
            1000U, &ptThis->lForegroundTimestamp, true))
#if (FOC_APP_LOG_TIMING_DIAGNOSTICS || \
     (FOC_APP_LOG_SMO_DIAGNOSTICS && \
      FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO)) && \
    !defined(__NO_USE_LOG__)
        foc_app_ReportDiagnostics(ptThis);
#endif
#if FOC_APP_LOG_ADC_OFFSETS && !defined(__NO_USE_LOG__)
        foc_app_ReportAdcOffsets(ptThis);
#endif
        motor_PollBreakFault(&ptThis->tMotor);
        foc_debug_CurrentStepRun(ptThis);
        if (!ptThis->bReady) {
            continue;
        }
        if (ptThis->bEncoderEnabled) {
            eResult = foc_encoder_Run(&ptThis->tEncoder);
            if (eResult != FOC_RESULT_OK) {
                continue;
            }
        }
        eIdentify = identify_Run(&ptThis->tIdentify,
                                 &ptThis->tMotor);
        if (ptThis->tIdentify.eState == IDENTIFY_STATE_ERROR &&
            ptThis->eLastIdentifyState != IDENTIFY_STATE_ERROR) {
            MLOGF(W, "identify failed (%d)\r\n", (int)eIdentify);
#if FOC_APP_LOG_INDUCTANCE_ID && !defined(__NO_USE_LOG__)
            foc_app_ReportInductanceDiagnostic(&ptThis->tIdentify);
#endif
        }
        ptThis->eLastIdentifyState = ptThis->tIdentify.eState;
        if (eIdentify == FOC_RESULT_OK &&
            identify_GetResistance(&ptThis->tIdentify,
                                   &tResistanceResult) == FOC_RESULT_OK) {
#if FOC_APP_LOG_RESISTANCE_ID && !defined(__NO_USE_LOG__)
            foc_app_ReportResistance(&tResistanceResult);
#endif
        }
        if (eIdentify == FOC_RESULT_OK &&
            identify_GetInductance(&ptThis->tIdentify,
                                   &tInductanceResult) == FOC_RESULT_OK) {
#if FOC_APP_LOG_INDUCTANCE_ID && !defined(__NO_USE_LOG__)
            foc_app_ReportInductanceDiagnostic(&ptThis->tIdentify);
            MLOGF(I, "identify Ld=%lu uH freq=%lu Hz v=%lu mV "
                  "i=%ld mA samples=%u halves=%u\r\n",
                  (unsigned long)
                      tInductanceResult.wInductanceDMicroHenry,
                  (unsigned long)tInductanceResult.wInjectionFrequencyHz,
                  (unsigned long)tInductanceResult.wEffectiveVoltageMillivolt,
                  (long)tInductanceResult.lMeanCurrentMilliamp,
                  (unsigned)tInductanceResult.hwCaptureSampleCount,
                  (unsigned)tInductanceResult.hwHalfCycleCount);
#endif
        }
    }
    PERFC_PT_END()
    return MODUS_SUCCESS;
}

#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
/**
 * @brief Accumulate trigger-to-CCR latency and direct PWM bottom crossing.
 * @param ptThis Application owning Motor and timing statistics.
 * @return None.
 * @note TIM1 CH4 rises on the down-count at CCR4; the snapshot follows
 *       the three CCR writes in the same control cycle.
 */
static void foc_app_RecordCcrTiming(foc_app_t *ptThis)
{
    const foc_port_pwm_phase_t *ptPhase =
        &ptThis->tMotor.tPwmCommitPhase;
    uint32_t wCcrLatencyTicks = 0U;

    if (!ptPhase->bValid ||
        ptPhase->wTriggerCounter == 0U ||
        (ptPhase->bCountingDown &&
         ptPhase->wCounter > ptPhase->wTriggerCounter) ||
        (!ptPhase->bCountingDown &&
         ptPhase->wCounter > UINT32_MAX -
             ptPhase->wTriggerCounter)) {
        ptThis->tHfStats.wCcrLatencyInvalidCount++;
        return;
    }
    if (ptPhase->bCountingDown) {
        wCcrLatencyTicks = ptPhase->wTriggerCounter -
            ptPhase->wCounter;
    } else {
        wCcrLatencyTicks = ptPhase->wTriggerCounter +
            ptPhase->wCounter;
    }
    ptThis->tHfStats.wCcrLatencyTicksTotal += wCcrLatencyTicks;
    ptThis->tHfStats.wCcrLatencySampleCount++;
    if (wCcrLatencyTicks > ptThis->tHfStats.wCcrLatencyMaxTicks) {
        ptThis->tHfStats.wCcrLatencyMaxTicks = wCcrLatencyTicks;
    }
    if (!ptPhase->bCountingDown) {
        ptThis->tHfStats.wCcrAfterBottomCount++;
    } else {
        if (ptPhase->wCounter <
            ptThis->tHfStats.wCcrMinimumBottomMarginTicks) {
            ptThis->tHfStats.wCcrMinimumBottomMarginTicks =
                ptPhase->wCounter;
        }
    }
}
#endif

void foc_app_HighFrequencyISR(void)
{
    uint32_t wNowTick = 0U;
#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
    int64_t lStartTicks = 0;
    int64_t lElapsedTicks = 0;
    uint32_t wElapsedTicks = 0U;
#endif
    motor_position_sample_t tSample = {0};
    motor_electrical_feedback_t tFeedback = {0};
    motor_isr_phase_t ePhase = MOTOR_ISR_NO_CONTROL;
    foc_result_t ePosition = FOC_RESULT_OK;

    wNowTick = (uint32_t)get_system_ticks();
#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
    lStartTicks = (int64_t)wNowTick;
#endif
    if (tFocApp.bReady) {
        if (tFocApp.tMotor.eState == MOTOR_STATE_ALIGN) {
            motor_position_InvalidateZero(&tFocApp.tPosition);
        }
        ePhase = motor_IsrPrepare(&tFocApp.tMotor, &tSample);
        if (ePhase == MOTOR_ISR_CONTROL_READY) {
            ePosition = FOC_POSITION_GET(&tFocApp.tPosition,
                                          wNowTick, &tSample,
                                          &tFeedback);
            motor_IsrControlStep(&tFocApp.tMotor,
                ePosition == FOC_RESULT_OK ? &tFeedback : NULL);
#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
            if (ePosition == FOC_RESULT_OK &&
                tFocApp.tMotor.eState == MOTOR_STATE_RUNNING &&
                tFocApp.tMotor.bPwmEnabled) {
                foc_app_RecordCcrTiming(&tFocApp);
            }
#endif
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
            if (ePosition == FOC_RESULT_OK) {
                motor_position_ObserverStep(&tFocApp.tPosition,
                                             &tSample);
#if FOC_APP_LOG_SMO_DIAGNOSTICS && \
    FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO && \
    !defined(__NO_USE_LOG__)
                foc_app_AccumulateSmoDiagnostic(&tFocApp, &tSample);
#endif
            }
#endif
        } else if (ePhase == MOTOR_ISR_CAPTURE_ZERO) {
            ePosition = motor_position_CaptureZero(
                &tFocApp.tPosition, wNowTick);
            motor_CompleteAlignIsr(&tFocApp.tMotor, ePosition);
        }
        // identify
        identify_operation_t eIdentifyOperation =
            tFocApp.tIdentify.eOperation;
        if (eIdentifyOperation == IDENTIFY_OPERATION_RESISTANCE ||
            eIdentifyOperation == IDENTIFY_OPERATION_INDUCTANCE) {
            identify_isr_sample_t tIdentifySample = {
                .qCurrentD = tFocApp.tMotor.tCore.tCurrent.qD,
                .qVoltageD = tFocApp.tMotor.tCore.tVoltage.qD,
            };

            if (eIdentifyOperation == IDENTIFY_OPERATION_INDUCTANCE) {
                tIdentifySample.qCurrentQ =
                    tFocApp.tMotor.tCore.tCurrent.qQ;
                tIdentifySample.qElectricalSpeedPu =
                    tFocApp.tMotor.tInput.qElectricalSpeedPu;
                tIdentifySample.bMotorFault =
                    tFocApp.tMotor.eState == MOTOR_STATE_FAULT;
                tIdentifySample.bAngleValid =
                    tFocApp.tMotor.tInput.bAngleValid;
                tIdentifySample.bPwmSaturated =
                    tFocApp.tMotor.tCore.bPwmSaturated;
                tIdentifySample.bDcBusValid =
                    foc_app_SampleDcBusMillivolt(
                        &tIdentifySample.wDcBusMillivolt) == FOC_RESULT_OK;
            }

            identify_IsrStep(&tFocApp.tIdentify, &tFocApp.tMotor,
                             &tIdentifySample);
        }
    }
#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
    foc_debug_WaveformStep();
#endif
#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
    lElapsedTicks = get_system_ticks() - lStartTicks - (int64_t)g_nOffset;
    if (lElapsedTicks > 0) {
        wElapsedTicks = (uint32_t)lElapsedTicks;
    } else {
        wElapsedTicks = 0U;
    }
    tFocApp.tHfStats.wCycleTotal =
        tFocApp.tHfStats.wCycleTotal + wElapsedTicks;
    tFocApp.tHfStats.wSampleCount =
        tFocApp.tHfStats.wSampleCount + 1U;
#endif
}

#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
/**
 * @brief Copy and reset the high-frequency cycle window.
 * @param ptThis FOC App object.
 * @param pwAverageTicks Output average perf_counter ticks per ISR.
 * @return true when the window contains at least one sample.
 * @note Interrupts are masked only while copying and resetting the counters.
 */
static bool foc_app_GetHfAverage(foc_app_t *ptThis,
                                 uint32_t *pwAverageTicks)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    uint32_t wCycleTotal = 0U;
    uint32_t wSampleCount = 0U;

    if (ptThis == NULL || pwAverageTicks == NULL) {
        return false;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    wCycleTotal = ptThis->tHfStats.wCycleTotal;
    wSampleCount = ptThis->tHfStats.wSampleCount;
    ptThis->tHfStats.wCycleTotal = 0U;
    ptThis->tHfStats.wSampleCount = 0U;
    perfc_port_resume_global_interrupt(tIrqState);
    if (wSampleCount == 0U) {
        return false;
    }
    *pwAverageTicks = wCycleTotal / wSampleCount;
    return true;
}

/**
 * @brief Report ADC trigger to PWM commit timing and bottom crossings.
 * @param ptThis FOC App object owning the ISR statistics.
 * @return None.
 * @note The late count uses captured timer direction, not rounded time.
 */
static void foc_app_ReportCcrLatency(foc_app_t *ptThis)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    uint32_t wTotalTicks = 0U;
    uint32_t wSampleCount = 0U;
    uint32_t wMaximumTicks = 0U;
    uint32_t wAfterBottomCount = 0U;
    uint32_t wInvalidCount = 0U;
    uint32_t wMinimumMarginTicks = UINT32_MAX;
    uint32_t wAverageTicks = 0U;
    uint32_t wAverageMicroseconds = 0U;
    uint32_t wMaximumMicroseconds = 0U;

    tIrqState = perfc_port_disable_global_interrupt();
    wTotalTicks = ptThis->tHfStats.wCcrLatencyTicksTotal;
    wSampleCount = ptThis->tHfStats.wCcrLatencySampleCount;
    wMaximumTicks = ptThis->tHfStats.wCcrLatencyMaxTicks;
    wAfterBottomCount = ptThis->tHfStats.wCcrAfterBottomCount;
    wInvalidCount = ptThis->tHfStats.wCcrLatencyInvalidCount;
    wMinimumMarginTicks =
        ptThis->tHfStats.wCcrMinimumBottomMarginTicks;
    ptThis->tHfStats.wCcrLatencyTicksTotal = 0U;
    ptThis->tHfStats.wCcrLatencySampleCount = 0U;
    ptThis->tHfStats.wCcrLatencyMaxTicks = 0U;
    ptThis->tHfStats.wCcrAfterBottomCount = 0U;
    ptThis->tHfStats.wCcrLatencyInvalidCount = 0U;
    ptThis->tHfStats.wCcrMinimumBottomMarginTicks = UINT32_MAX;
    perfc_port_resume_global_interrupt(tIrqState);

    if (wSampleCount == 0U && wInvalidCount == 0U) {
        return;
    }
    if (wSampleCount > 0U) {
        wAverageTicks = wTotalTicks / wSampleCount;
    }
    wAverageMicroseconds = (uint32_t)perfc_convert_ticks_to_us(
        (int64_t)wAverageTicks);
    wMaximumMicroseconds = (uint32_t)perfc_convert_ticks_to_us(
        (int64_t)wMaximumTicks);
    if (wMinimumMarginTicks == UINT32_MAX) {
        wMinimumMarginTicks = 0U;
    }
    MLOGF(T, "FOC ADCtrig->CCR avg=%lu cyc/%lu us "
          "max=%lu cyc/%lu us bottomLate=%lu/%lu "
          "marginMin=%lu cyc bad=%lu\r\n",
          (unsigned long)wAverageTicks,
          (unsigned long)wAverageMicroseconds,
          (unsigned long)wMaximumTicks,
          (unsigned long)wMaximumMicroseconds,
          (unsigned long)wAfterBottomCount,
          (unsigned long)wSampleCount,
          (unsigned long)wMinimumMarginTicks,
          (unsigned long)wInvalidCount);
}
#endif /* FOC_APP_LOG_TIMING_DIAGNOSTICS */

#if FOC_APP_LOG_SMO_DIAGNOSTICS && \
    FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO && \
    !defined(__NO_USE_LOG__)
/**
 * @brief Report SMO amplitude binning and electrical sector distributions.
 * @param pawBinBad Bad sample counts across the 4 amplitude bins.
 * @param pawBinSample Total sample counts across the 4 amplitude bins.
 * @param pawSectorBad Bad sample counts across the 8 electrical sectors.
 * @param pawSectorLow Low-amplitude sample counts across the 8 sectors.
 * @return None.
 */
static void foc_app_ReportSmoDistribution(
    const uint32_t pawBinBad[4],
    const uint32_t pawBinSample[4],
    const uint32_t pawSectorBad[8],
    const uint32_t pawSectorLow[8])
{
    MLOGF(T, "SMO bin bad/tot [0]=%lu/%lu [1]=%lu/%lu "
          "[2]=%lu/%lu [3]=%lu/%lu\r\n",
          (unsigned long)pawBinBad[0],
          (unsigned long)pawBinSample[0],
          (unsigned long)pawBinBad[1],
          (unsigned long)pawBinSample[1],
          (unsigned long)pawBinBad[2],
          (unsigned long)pawBinSample[2],
          (unsigned long)pawBinBad[3],
          (unsigned long)pawBinSample[3]);
    MLOGF(T, "SMO sec err/low s0=%lu/%lu s1=%lu/%lu s2=%lu/%lu "
          "s3=%lu/%lu s4=%lu/%lu s5=%lu/%lu s6=%lu/%lu s7=%lu/%lu\r\n",
          (unsigned long)pawSectorBad[0],
          (unsigned long)pawSectorLow[0],
          (unsigned long)pawSectorBad[1],
          (unsigned long)pawSectorLow[1],
          (unsigned long)pawSectorBad[2],
          (unsigned long)pawSectorLow[2],
          (unsigned long)pawSectorBad[3],
          (unsigned long)pawSectorLow[3],
          (unsigned long)pawSectorBad[4],
          (unsigned long)pawSectorLow[4],
          (unsigned long)pawSectorBad[5],
          (unsigned long)pawSectorLow[5],
          (unsigned long)pawSectorBad[6],
          (unsigned long)pawSectorLow[6],
          (unsigned long)pawSectorBad[7],
          (unsigned long)pawSectorLow[7]);
}

/**
 * @brief Report and reset one SMO quality window from foreground.
 * @param ptThis Application object owning the ISR statistics.
 * @return None.
 * @note The brief interrupt mask covers only the statistics snapshot.
 */
static void foc_app_ReportSmoRms(foc_app_t *ptThis)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    uint32_t wSampleCount = 0U;
    uint32_t wLargeAngleErrorCount = 0U;
    uint32_t wGoodCount = 0U;
    float fBemfSquareTotal = 0.0f;
    float fCurrentErrorSquareTotal = 0.0f;
    float fBadBemfSquareTotal = 0.0f;
    float fBadCurrentSquareTotal = 0.0f;
    float fGoodBemfSquareTotal = 0.0f;
    float fGoodCurrentSquareTotal = 0.0f;
    float fBemfRms = 0.0f;
    float fCurrentErrorRms = 0.0f;
    uint32_t awBinSampleCount[4] = {0U};
    uint32_t awBinBadCount[4] = {0U};
    uint32_t awSectorBadCount[8] = {0U};
    uint32_t awSectorLowCount[8] = {0U};
    uint32_t wIndex = 0U;

    if (ptThis == NULL) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    wSampleCount = ptThis->tHfStats.wSmoDiagnosticSampleCount;
    wLargeAngleErrorCount =
        ptThis->tHfStats.wSmoLargeAngleErrorCount;
    fBemfSquareTotal = ptThis->tHfStats.fBemfSquareTotal;
    fCurrentErrorSquareTotal =
        ptThis->tHfStats.fCurrentErrorSquareTotal;
    fBadBemfSquareTotal =
        ptThis->tHfStats.fSmoBadBemfSquareTotal;
    fBadCurrentSquareTotal =
        ptThis->tHfStats.fSmoBadCurrentSquareTotal;
    ptThis->tHfStats.wSmoDiagnosticSampleCount = 0U;
    ptThis->tHfStats.wSmoLargeAngleErrorCount = 0U;
    ptThis->tHfStats.fBemfSquareTotal = 0.0f;
    ptThis->tHfStats.fCurrentErrorSquareTotal = 0.0f;
    ptThis->tHfStats.fSmoBadBemfSquareTotal = 0.0f;
    ptThis->tHfStats.fSmoBadCurrentSquareTotal = 0.0f;
    for (wIndex = 0U; wIndex < 4U; wIndex++) {
        awBinSampleCount[wIndex] =
            ptThis->tHfStats.awSmoBinSampleCount[wIndex];
        awBinBadCount[wIndex] =
            ptThis->tHfStats.awSmoBinBadCount[wIndex];
        ptThis->tHfStats.awSmoBinSampleCount[wIndex] = 0U;
        ptThis->tHfStats.awSmoBinBadCount[wIndex] = 0U;
    }
    for (wIndex = 0U; wIndex < 8U; wIndex++) {
        awSectorBadCount[wIndex] =
            ptThis->tHfStats.awSmoSectorBadCount[wIndex];
        awSectorLowCount[wIndex] =
            ptThis->tHfStats.awSmoSectorLowCount[wIndex];
        ptThis->tHfStats.awSmoSectorBadCount[wIndex] = 0U;
        ptThis->tHfStats.awSmoSectorLowCount[wIndex] = 0U;
    }
    perfc_port_resume_global_interrupt(tIrqState);

    if (wSampleCount == 0U) {
        return;
    }
    fBemfRms = sqrtf(fBemfSquareTotal / (float)wSampleCount);
    fCurrentErrorRms = sqrtf(
        fCurrentErrorSquareTotal / (float)wSampleCount);
    MLOGF(T, "SMO rms x1e4 e=%lu iErr=%lu err25=%lu/%lu\r\n",
          (unsigned long)(fBemfRms * 10000.0f + 0.5f),
          (unsigned long)(fCurrentErrorRms * 10000.0f + 0.5f),
          (unsigned long)wLargeAngleErrorCount,
          (unsigned long)wSampleCount);
    wGoodCount = wSampleCount - wLargeAngleErrorCount;
    fGoodBemfSquareTotal = fmaxf(
        0.0f, fBemfSquareTotal - fBadBemfSquareTotal);
    fGoodCurrentSquareTotal = fmaxf(
        0.0f, fCurrentErrorSquareTotal - fBadCurrentSquareTotal);
    MLOGF(T, "SMO cond x1e4 eBad=%lu eGood=%lu "
          "iBad=%lu iGood=%lu\r\n",
          (unsigned long)(wLargeAngleErrorCount == 0U ? 0.0f :
              sqrtf(fBadBemfSquareTotal /
                    (float)wLargeAngleErrorCount) * 10000.0f + 0.5f),
          (unsigned long)(wGoodCount == 0U ? 0.0f :
              sqrtf(fGoodBemfSquareTotal / (float)wGoodCount) *
              10000.0f + 0.5f),
          (unsigned long)(wLargeAngleErrorCount == 0U ? 0.0f :
              sqrtf(fBadCurrentSquareTotal /
                    (float)wLargeAngleErrorCount) * 10000.0f + 0.5f),
          (unsigned long)(wGoodCount == 0U ? 0.0f :
              sqrtf(fGoodCurrentSquareTotal / (float)wGoodCount) *
              10000.0f + 0.5f));
    foc_app_ReportSmoDistribution(awBinBadCount,
                                  awBinSampleCount,
                                  awSectorBadCount,
                                  awSectorLowCount);
}
#endif

#if FOC_APP_LOG_TIMING_DIAGNOSTICS && !defined(__NO_USE_LOG__)
/**
 * @brief Report the average ISR cycles once per second from foreground.
 * @param ptThis FOC App object.
 * @return None.
 */
static void foc_app_ReportHfAverage(foc_app_t *ptThis)
{
    uint32_t wAverageTicks = 0U;
    uint32_t wAverageMicroseconds = 0U;
    uint32_t wDcBusMillivolt = 0U;
    foc_result_t eDcBus = FOC_RESULT_DISABLED;

    eDcBus = foc_app_SampleDcBusMillivolt(&wDcBusMillivolt);
    if (!foc_app_GetHfAverage(ptThis, &wAverageTicks)) {
        return;
    }
    wAverageMicroseconds = (uint32_t)perfc_convert_ticks_to_us(
        (int64_t)wAverageTicks);
    MLOGF(T, "FOC HF ISR avg=%lu cycles/%lu us vbus=%lu mV%s\r\n",
          (unsigned long)wAverageTicks,
          (unsigned long)wAverageMicroseconds,
          (unsigned long)wDcBusMillivolt,
          eDcBus == FOC_RESULT_OK ? "" : " (invalid)");
    foc_app_ReportCcrLatency(ptThis);
}
#endif

#if (FOC_APP_LOG_TIMING_DIAGNOSTICS || \
     (FOC_APP_LOG_SMO_DIAGNOSTICS && \
      FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO)) && \
    !defined(__NO_USE_LOG__)
/**
 * @brief Emit enabled periodic diagnostic groups once per second.
 * @param ptThis FOC App object owning diagnostic statistics.
 * @return None.
 */
static void foc_app_ReportDiagnostics(foc_app_t *ptThis)
{
    if (ptThis == NULL || !perfc_is_time_out_ms(
            1000U, &ptThis->tHfStats.lReportTimestamp, true)) {
        return;
    }
#if FOC_APP_LOG_TIMING_DIAGNOSTICS
    foc_app_ReportHfAverage(ptThis);
#endif
#if FOC_APP_LOG_SMO_DIAGNOSTICS && \
    FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    foc_app_ReportSmoRms(ptThis);
#endif
}
#endif

MODUS_DECLARE_OBJECT(foc_app, FocApp, FOC_APP_INIT_CONFIG)
