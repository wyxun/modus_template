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
static bool foc_app_GetHfAverage(foc_app_t *ptThis,
                                 uint32_t *pwAverageTicks);
static void foc_app_ReportHfAverage(foc_app_t *ptThis);

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
    ptThis->lBackoffTimestamp = 0;
    ptThis->tHfStats.lReportTimestamp = get_system_ticks();
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

static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    foc_result_t eResult = FOC_RESULT_OK;
    foc_result_t eIdentify = FOC_RESULT_OK;
    uint32_t wResistanceMilliohm = 0U;
    identify_inductance_result_t tInductanceResult = {0};

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }
    PERFC_PT_BEGIN(ptThis->chRunPt)
    while (true) {
        PERFC_PT_WAIT_UNTIL(perfc_is_time_out_us(
            1000U, &ptThis->lForegroundTimestamp, true))
        foc_app_ReportHfAverage(ptThis);
        motor_PollBreakFault(&ptThis->tMotor);
        if (!ptThis->bReady) {
            continue;
        }
        if (ptThis->lBackoffTimestamp != 0 &&
            !perfc_is_time_out_ms(100U,
                                  &ptThis->lBackoffTimestamp, false)) {
            continue;
        }
        eIdentify = identify_Run(&ptThis->tIdentify,
                                 &ptThis->tMotor);
        if (ptThis->tIdentify.eState == IDENTIFY_STATE_ERROR &&
            ptThis->eLastIdentifyState != IDENTIFY_STATE_ERROR) {
            MLOGF(W, "identify failed (%d)\r\n", (int)eIdentify);
        }
        ptThis->eLastIdentifyState = ptThis->tIdentify.eState;
        if (eIdentify == FOC_RESULT_OK &&
            identify_GetResistance(&ptThis->tIdentify,
                                   &wResistanceMilliohm) == FOC_RESULT_OK) {
            MLOGF(I, "identify R=%lu mOhm\r\n",
                  (unsigned long)wResistanceMilliohm);
        }
        if (eIdentify == FOC_RESULT_OK &&
            identify_GetInductance(&ptThis->tIdentify,
                                   &tInductanceResult) == FOC_RESULT_OK) {
            MLOGF(I, "identify Ld=%lu uH freq=%lu Hz v=%lu mV "
                  "i=%ld mA samples=%u halves=%u\r\n",
                  (unsigned long)
                      tInductanceResult.wInductanceDMicroHenry,
                  (unsigned long)tInductanceResult.wInjectionFrequencyHz,
                  (unsigned long)tInductanceResult.wEffectiveVoltageMillivolt,
                  (long)tInductanceResult.lMeanCurrentMilliamp,
                  (unsigned)tInductanceResult.hwCaptureSampleCount,
                  (unsigned)tInductanceResult.hwHalfCycleCount);
        }
        if (ptThis->tPosition.eSource == MOTOR_POSITION_SOURCE_SENSOR) {
            eResult = foc_encoder_Run(&ptThis->tEncoder);
            if (eResult != FOC_RESULT_OK) {
                ptThis->lBackoffTimestamp = get_system_ticks() +
                    perfc_convert_ms_to_ticks(100U);
                continue;
            }
        }
        ptThis->lBackoffTimestamp = 0;
    }
    PERFC_PT_END()
    return MODUS_SUCCESS;
}

void foc_app_HighFrequencyISR(void)
{
    int64_t lStartTicks = get_system_ticks();
    int64_t lElapsedTicks = 0;
    uint32_t wNowTick = 0U;
    uint32_t wElapsedTicks = 0U;
    motor_position_sample_t tSample = {0};
    motor_electrical_feedback_t tFeedback = {0};
    motor_isr_phase_t ePhase = MOTOR_ISR_NO_CONTROL;
    foc_result_t ePosition = FOC_RESULT_OK;

    wNowTick = (uint32_t)lStartTicks;
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
    lElapsedTicks = get_system_ticks() - lStartTicks -
                    (int64_t)g_nOffset;
    if (lElapsedTicks > 0) {
        wElapsedTicks = (uint32_t)lElapsedTicks;
    } else {
        wElapsedTicks = 0U;
    }
    tFocApp.tHfStats.wCycleTotal =
        tFocApp.tHfStats.wCycleTotal + wElapsedTicks;
    tFocApp.tHfStats.wSampleCount =
        tFocApp.tHfStats.wSampleCount + 1U;
}

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

    if (!perfc_is_time_out_ms(1000U,
                              &ptThis->tHfStats.lReportTimestamp, true)) {
        return;
    }
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
}

MODUS_DECLARE_OBJECT(foc_app, FocApp, FOC_APP_INIT_CONFIG)
