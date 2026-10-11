/****************************************************************************
 * @file    foc_app.c
 * @brief   MODUS composition and scheduling class for one FOC Motor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_app.h"
#include "foc_debug.h"

#include "foc_port_config.h"
#include "mdebug/mwaveform.h"

#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_NONE &&                        \
    MOTOR_CONFIG_OBSERVER_TAKEOVER
#error "Observer takeover requires an observer backend"
#endif

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
#define FOC_APP_INIT_OBSERVER_CONFIG                                            \
    .tObserverCfg = {                                                           \
        .tSmo = {                                                               \
            .wBemfCutoffRadiansPerSecond =                                      \
                MOTOR_CONFIG_SMO_BEMF_CUTOFF_RADIANS_PER_SECOND,                \
            .wSlidingGainMillivolt =                                            \
                MOTOR_CONFIG_SMO_SLIDING_GAIN_MILLIVOLT,                        \
            .qCurrentEstimateLimit = FOC_SCALAR(                                \
                MOTOR_CONFIG_SMO_CURRENT_ESTIMATE_LIMIT_PU),                    \
        },                                                                      \
    },                                                                          \
    .bObserverTakeover = MOTOR_CONFIG_OBSERVER_TAKEOVER,                        \
    .bObserverAutoTakeover = MOTOR_CONFIG_OBSERVER_AUTO_TAKEOVER,               \
    .wObserverQualificationSteps =                                              \
        MOTOR_CONFIG_OBSERVER_QUALIFICATION_STEPS,                              \
    .wObserverBlendSteps = MOTOR_CONFIG_OBSERVER_BLEND_STEPS,                   \
    .qObserverMinBemfPu = FOC_SCALAR(                                           \
        MOTOR_CONFIG_OBSERVER_MIN_BEMF_PU),                                     \
    .qObserverMinSpeedPu = FOC_SCALAR(                                          \
        MOTOR_CONFIG_OBSERVER_MIN_SPEED_PU),                                    \
    .qObserverMaxSpeedErrorRatio = FOC_SCALAR(                                  \
        MOTOR_CONFIG_OBSERVER_MAX_SPEED_ERROR_RATIO),                           \
    .qObserverMaxAngleErrorTurns = FOC_SCALAR(                                  \
        MOTOR_CONFIG_OBSERVER_MAX_ANGLE_ERROR_TURNS),
#else
#define FOC_APP_INIT_OBSERVER_CONFIG
#endif

#if FOC_PORT_HAS_POSITION
#if !defined(FOC_ENCODER_STATIC_BINDING)
extern const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface;
#define FOC_APP_INIT_ENCODER_CONFIG                                             \
    .tEncoderCfg = {                                                            \
        .qSpeedFilterAlpha = FOC_SCALAR(                                        \
            MOTOR_CONFIG_ENCODER_SPEED_FILTER_ALPHA),                           \
        .fInvalidTimeoutSeconds =                                               \
            MOTOR_CONFIG_ENCODER_INVALID_TIMEOUT_SECONDS,                       \
        .bDirectionInvert = MOTOR_CONFIG_ENCODER_DIRECTION_INVERT,              \
        .ptSensor = &g_tFocEncoderSensorInterface,                              \
    },
#else
#define FOC_APP_INIT_ENCODER_CONFIG                                             \
    .tEncoderCfg = {                                                            \
        .qSpeedFilterAlpha = FOC_SCALAR(                                        \
            MOTOR_CONFIG_ENCODER_SPEED_FILTER_ALPHA),                           \
        .fInvalidTimeoutSeconds =                                               \
            MOTOR_CONFIG_ENCODER_INVALID_TIMEOUT_SECONDS,                       \
        .bDirectionInvert = MOTOR_CONFIG_ENCODER_DIRECTION_INVERT,              \
    },
#endif
#else
#define FOC_APP_INIT_ENCODER_CONFIG                                             \
    .tEncoderCfg = {0},
#endif

static const foc_app_speed_profile_t s_tFocAppSpeedProfile = {
    .qMaximumElectricalHz =
        FOC_SCALAR(MOTOR_CONFIG_MAX_SPEED_ELECTRICAL_HZ),
    .tKpPerElectricalHz = {MOTOR_CONFIG_PI_GAIN_INTEGER,
        FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_KP_PER_ELECTRICAL_HZ)},
    .tKiTsPerElectricalHz = {MOTOR_CONFIG_PI_GAIN_INTEGER,
        FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_KI_TS_PER_ELECTRICAL_HZ)},
    .tKdOverTsPerElectricalHz = {MOTOR_CONFIG_PI_GAIN_INTEGER,
        FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_KD_OVER_TS_PER_ELECTRICAL_HZ)},
};

#define FOC_APP_INIT_CONFIG                                                     \
    .tMotorCfg = FOC_APP_INIT_MOTOR_CONFIG,                                     \
    .ptSpeedProfile = &s_tFocAppSpeedProfile,                                   \
    .ePositionSource = MOTOR_CONFIG_POSITION_SOURCE,                            \
    FOC_APP_INIT_OBSERVER_CONFIG                                                \
    FOC_APP_INIT_ENCODER_CONFIG                                                 \
    .wVoltageBaseMillivolt = MOTOR_CONFIG_BASE_VOLTAGE_MILLIVOLT,               \
    .wHighFrequencyIsrHz = FOC_HF_ISR_HZ,                                       \
    .qElectricalSpeedBaseHz =                                                   \
        FOC_SCALAR(MOTOR_CONFIG_BASE_ELECTRICAL_HZ),

#include <stddef.h>
#include <stdint.h>

#include "foc_config.h"
#include "internal/foc_units.h"
#include "mdebug/util_debug.h"
#include "perf_counter.h"
#include "perfc_task_pt.h"

static modus_base_t s_tFocAppBase;
extern foc_app_t tFocApp;
static int foc_app_Clock(uintptr_t wObjectAddr);
static int foc_app_Run(uintptr_t wObjectAddr);

static uint32_t foc_app_GetControlPeriodNanoseconds(uint32_t wFrequencyHz)
{
    if (wFrequencyHz == 0U ||
        wFrequencyHz > FOC_NANOSECONDS_PER_SECOND) {
        return 0U;
    }
    return (FOC_NANOSECONDS_PER_SECOND + (wFrequencyHz / 2U)) /
           wFrequencyHz;
}

#if IDENTIFY_LOG_ACTIVE
/** @brief Log one completed tagged Identify result in physical units.
 * @param ptIdentify Result conversion context. @param ptResult PU result.
 * @return None; called only from the foreground Run step. */
static void foc_app_ReportIdentifyResult(const identify_t *ptIdentify,
    const identify_result_t *ptResult)
{
    uint32_t wPhysical = 0U;
    foc_result_t eResult = FOC_RESULT_DISABLED;

    if (ptIdentify == NULL || ptResult == NULL) {
        return;
    }
    switch (ptResult->eOperation) {
    case IDENTIFY_OPERATION_RESISTANCE:
        eResult = identify_ResultToPhysical(ptIdentify,
            IDENTIFY_OPERATION_RESISTANCE,
            ptResult->tValue.tResistance.qResistancePu, &wPhysical);
        if (eResult == FOC_RESULT_OK) {
            MLOGF(I, "identify R=%lu mOhm\r\n", (unsigned long)wPhysical);
        }
        break;
    case IDENTIFY_OPERATION_INDUCTANCE:
        eResult = identify_ResultToPhysical(ptIdentify,
            IDENTIFY_OPERATION_INDUCTANCE,
            ptResult->tValue.tInductance.qInductanceDPu, &wPhysical);
        if (eResult == FOC_RESULT_OK) {
            MLOGF(I, "identify Ld=%lu uH\r\n", (unsigned long)wPhysical);
        }
        break;
    case IDENTIFY_OPERATION_NONE:
    default:
        break;
    }
}
#endif

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
 * @brief Bind Motor PU tuning and optional physical speed profile once.
 * @param ptConfig App configuration.
 * @param ptMotorConfig Output Motor configuration.
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
    ptMotorConfig->qElectricalSpeedBaseHz =
        ptConfig->qElectricalSpeedBaseHz;
    ptMotorConfig->wControlFrequencyHz =
        FOC_HF_ISR_HZ;
    if (ptConfig->ptSpeedProfile == NULL) {
        return FOC_RESULT_OK;
    }
    ptMotorConfig->tSpeedPiParams.tKp =
        ptConfig->ptSpeedProfile->tKpPerElectricalHz;
    ptMotorConfig->tSpeedPiParams.tKiTs =
        ptConfig->ptSpeedProfile->tKiTsPerElectricalHz;
    ptMotorConfig->tSpeedPiParams.tKdOverTs =
        ptConfig->ptSpeedProfile->tKdOverTsPerElectricalHz;
    eResult = foc_div_checked(
        ptConfig->ptSpeedProfile->qMaximumElectricalHz,
        ptConfig->qElectricalSpeedBaseHz,
        &ptMotorConfig->tLimits.qMaxSpeedReference);
    if (eResult != FOC_RESULT_OK ||
        ptMotorConfig->tLimits.qMaxSpeedReference <= FOC_ZERO ||
        ptMotorConfig->tLimits.qMaxSpeedReference > FOC_ONE) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    eResult = foc_app_ScaleSpeedGain(
        &ptMotorConfig->tSpeedPiParams.tKp,
        ptConfig->qElectricalSpeedBaseHz);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_app_ScaleSpeedGain(
            &ptMotorConfig->tSpeedPiParams.tKiTs,
            ptConfig->qElectricalSpeedBaseHz);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_app_ScaleSpeedGain(
            &ptMotorConfig->tSpeedPiParams.tKdOverTs,
            ptConfig->qElectricalSpeedBaseHz);
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
    if (ptConfig->wHighFrequencyIsrHz != FOC_HF_ISR_HZ ||
        (ptConfig->ePositionSource != MOTOR_POSITION_SOURCE_SENSOR &&
         ptConfig->ePositionSource != MOTOR_POSITION_SOURCE_HARD_DRAG)) {
        return (int)FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptThis = (foc_app_t){0};
    ptThis->ptBase = &s_tFocAppBase;
    s_tFocAppBaseCfg.wParent = wObjectAddr;
    ptThis->chRunPt = 0U;
    ptThis->lForegroundTimestamp = 0;
    ptThis->lForegroundPeriodTicks = perfc_convert_us_to_ticks(1000U);
#if !defined(__NO_USE_LOG__)
    ptThis->tIsrTiming.lReportPeriodTicks = perfc_convert_ms_to_ticks(1000U);
    ptThis->tIsrTiming.lBudgetTicks =
        ptThis->tIsrTiming.lReportPeriodTicks / FOC_HF_ISR_HZ;
    ptThis->tIsrTiming.lReportTimestamp = get_system_ticks();
#endif
    ptThis->bEncoderEnabled = false;
    ptThis->bReady = false;

    if (ptConfig->ePositionSource == MOTOR_POSITION_SOURCE_SENSOR) {
        tEncoderConfig = ptConfig->tEncoderCfg;
        eEncoder = foc_encoder_Init(&ptThis->tEncoder,
                                    &tEncoderConfig);
        if (eEncoder != FOC_RESULT_OK) {
            return (int)eEncoder;
        }
    } else {
        eEncoder = FOC_RESULT_DISABLED;
    }
    wControlPeriodNanoseconds = foc_app_GetControlPeriodNanoseconds(
        FOC_HF_ISR_HZ);
    if (ptConfig->wVoltageBaseMillivolt == 0U ||
        wControlPeriodNanoseconds == 0U ||
        ptConfig->qElectricalSpeedBaseHz <= FOC_ZERO) {
        return MODUS_EFAIL;
    }
    eResult = foc_app_BindMotorConfig(ptConfig, &tMotorConfig);
    if (eResult != FOC_RESULT_OK) {
        return (int)eResult;
    }
    if (tMotorConfig.wStartupRampSteps != 0U &&
        ptConfig->ePositionSource !=
            MOTOR_POSITION_SOURCE_HARD_DRAG) {
        return (int)FOC_RESULT_INVALID_ARGUMENT;
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
    tPositionConfig.qElectricalSpeedBaseHz =
        ptConfig->qElectricalSpeedBaseHz;
    tPositionConfig.wControlFrequencyHz =
        FOC_HF_ISR_HZ;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    tPositionConfig.ptMotorParams = &ptThis->tMotor.tParams;
    tPositionConfig.tObserverCfg = ptConfig->tObserverCfg;
    tPositionConfig.tObserverCfg.tSmo.wSampleFrequencyHz =
        FOC_HF_ISR_HZ;
    tPositionConfig.bObserverTakeover = ptConfig->bObserverTakeover;
    tPositionConfig.bAutoTakeover = ptConfig->bObserverAutoTakeover;
    tPositionConfig.wQualificationSteps =
        ptConfig->wObserverQualificationSteps;
    tPositionConfig.wBlendSteps = ptConfig->wObserverBlendSteps;
    tPositionConfig.qMinimumBemfPu =
        ptConfig->qObserverMinBemfPu;
    tPositionConfig.qMinimumSpeedPu =
        ptConfig->qObserverMinSpeedPu;
    tPositionConfig.qMaximumSpeedErrorRatio =
        ptConfig->qObserverMaxSpeedErrorRatio;
    tPositionConfig.qMaximumAngleErrorTurns =
        ptConfig->qObserverMaxAngleErrorTurns;
#endif
    if (ptConfig->ePositionSource == MOTOR_POSITION_SOURCE_HARD_DRAG &&
        tMotorConfig.nHardDragElectricalMilliHz == 0 &&
        tMotorConfig.wStartupRampSteps == 0U) {
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
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    if (ptConfig->bObserverTakeover &&
        (tMotorConfig.wStartupRampSteps == 0U ||
         tMotorConfig.wStartupMaxRunSteps <=
             tMotorConfig.wStartupRampSteps ||
         motor_position_CheckStartupBudget(&ptThis->tPosition,
             tMotorConfig.wStartupMaxRunSteps -
             tMotorConfig.wStartupRampSteps) != FOC_RESULT_OK)) {
        motor_Stop(&ptThis->tMotor);
        return (int)FOC_RESULT_INVALID_ARGUMENT;
    }
#endif
    identify_cfg_t tIdentifyConfig = {
        .wVoltageBaseMillivolt = ptConfig->wVoltageBaseMillivolt,
        .wCurrentBaseMilliamp = FOC_CURRENT_BASE_MILLIAMP,
        .wResistanceMilliohm = tMotorConfig.tParams.wResistanceMilliohm,
        .qElectricalSpeedBaseHz = ptConfig->qElectricalSpeedBaseHz,
    };
    eResult = identify_Init(&ptThis->tIdentify, &tIdentifyConfig);
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

#if !defined(__NO_USE_LOG__)
/** @brief Report mean ISR time, or this window's maximum after an overrun.
 * @param ptApp App owning the timing window.
 * @return None.
 * @note Only snapshot/reset masks interrupts; conversion/logging is foreground.
 * The measurement covers ISR work including waveform, excluding accumulation.
 */
static void foc_app_ReportIsrTiming(foc_app_t *ptApp)
{
    foc_app_isr_timing_t *ptTiming = &ptApp->tIsrTiming;
    uint64_t qwTotal = 0U;
    uint32_t wSamples = 0U;
    uint32_t wMaximum = 0U;
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (!__perfc_is_time_out(ptTiming->lReportPeriodTicks,
        &ptTiming->lReportTimestamp, true)) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    qwTotal = ptTiming->qwTotalTicks;
    wSamples = ptTiming->wSamples;
    wMaximum = ptTiming->wMaximumTicks;
    ptTiming->qwTotalTicks = 0U;
    ptTiming->wSamples = 0U;
    ptTiming->wMaximumTicks = 0U;
    perfc_port_resume_global_interrupt(tIrqState);
    if (wSamples == 0U) {
        return;
    }
    bool bOverrun = (int64_t)wMaximum > ptTiming->lBudgetTicks;
    uint32_t wTicks = bOverrun ? wMaximum : (uint32_t)(qwTotal / wSamples);
    uint32_t wTenthsUs = (uint32_t)perfc_convert_ticks_to_us(
        (int64_t)wTicks * 10);
    MLOGF(T, "foc ISR %s=%lu.%lu us%s\r\n", bOverrun ? "max" : "avg",
        (unsigned long)(wTenthsUs / 10U),
        (unsigned long)(wTenthsUs % 10U), bOverrun ? " (overrun)" : "");
}
#endif

static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    foc_result_t eResult = FOC_RESULT_OK;
    foc_result_t eIdentify = FOC_RESULT_OK;
    identify_result_t tIdentifyResult = {0};

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }
    PERFC_PT_BEGIN(ptThis->chRunPt)
    while (true) {
        PERFC_PT_WAIT_UNTIL(__perfc_is_time_out(
            ptThis->lForegroundPeriodTicks,
            &ptThis->lForegroundTimestamp, true))
        motor_PollBreakFault(&ptThis->tMotor);
        foc_debug_CurrentStepRun(ptThis);
#if !defined(__NO_USE_LOG__)
        foc_app_ReportIsrTiming(ptThis);
#endif
        if (!ptThis->bReady) {
            continue;
        }
        if (ptThis->bEncoderEnabled) {
            eResult = foc_encoder_Run(&ptThis->tEncoder);
        }
        eIdentify = identify_Run(&ptThis->tIdentify,
                                 &ptThis->tMotor,
                                 &tIdentifyResult);
#if IDENTIFY_LOG_ACTIVE
        if (eIdentify == FOC_RESULT_OK) {
            foc_app_ReportIdentifyResult(&ptThis->tIdentify,
                                         &tIdentifyResult);
        }
#endif
        if (eIdentify != FOC_RESULT_OK && eIdentify != FOC_RESULT_BUSY) {
            eResult = eIdentify;
        }
        /* Each service advances independently; report failures afterwards. */
        PERFC_PT_YIELD(eResult == FOC_RESULT_OK ? MODUS_SUCCESS : MODUS_EFAIL)
    }
    PERFC_PT_END()
    return MODUS_SUCCESS;
}

/** @brief Assemble typed PU samples and submit same-ISR safety requests.
 * @param ptApp App instance.
 * @param ptSample Current alpha-beta PU sample. @param eControl Core status.
 * @return None; control rejection latches Identify and Motor faults. */
static void _foc_app_IdentifyStep(foc_app_t *ptApp,
    const motor_position_sample_t *ptSample, foc_result_t eControl)
{
    identify_operation_t eOperation =
        identify_ActiveOperationIsr(&ptApp->tIdentify);
    identify_isr_request_t request = {0};
    bool valid = eControl == FOC_RESULT_OK && ptSample->bVoltageModelValid;

    if (eOperation == IDENTIFY_OPERATION_NONE) {
        return;
    }
    if (eOperation == IDENTIFY_OPERATION_RESISTANCE) {
        identify_resistance_sample_t sample = {
            .tCurrentPu = ptSample->tCurrentAlphaBeta,
            .qVoltageModelAlphaPu = ptSample->tVoltageModelAlphaBeta.qAlpha,
            .bValid = valid,
            .bPwmSaturated = ptSample->bPwmSaturated,
        };
        identify_ResistanceIsrStep(&ptApp->tIdentify, &sample, &request);
    } else if (eOperation == IDENTIFY_OPERATION_INDUCTANCE) {
        identify_inductance_sample_t sample = {
            .tCurrentPu = ptSample->tCurrentAlphaBeta,
            .qVoltageModelAlphaPu = ptSample->tVoltageModelAlphaBeta.qAlpha,
            .bValid = valid,
            .bPwmSaturated = ptSample->bPwmSaturated,
        };
        identify_InductanceIsrStep(&ptApp->tIdentify, &sample, &request);
    } else {
        return;
    }
    if (request.bAbort) {
        motor_IdentificationAbortIsr(&ptApp->tMotor,
                                     MOTOR_FAULT_IDENTIFICATION);
    } else if (request.bSetModulation) {
        foc_result_t eApply = motor_IdentificationApplyIsr(&ptApp->tMotor,
            &request.tModulationCommand);
        if (eApply != FOC_RESULT_OK) {
            identify_AbortIsr(&ptApp->tIdentify,
                IDENTIFY_FAILURE_CAUSE_CONTROL_REJECTED, eApply);
            motor_IdentificationAbortIsr(&ptApp->tMotor,
                                         MOTOR_FAULT_IDENTIFICATION);
        }
    } else {
        /* Capture-only step. */
    }
}

void foc_app_HighFrequencyISR(void)
{
    uint32_t wNowTick = 0U;
    motor_position_sample_t tSample = {0};
    motor_electrical_feedback_t tFeedback = {0};
    motor_isr_phase_t ePhase = MOTOR_ISR_NO_CONTROL;
    foc_result_t ePosition = FOC_RESULT_OK;
    foc_result_t eControl = FOC_RESULT_DISABLED;

    wNowTick = (uint32_t)get_system_ticks();
    if (tFocApp.bReady) {
        ePhase = motor_IsrPrepare(&tFocApp.tMotor, &tSample);
        if (tSample.bAlignActive) {
            motor_position_InvalidateZero(&tFocApp.tPosition);
        }
        if (ePhase == MOTOR_ISR_CONTROL_READY) {
            ePosition = FOC_POSITION_GET(&tFocApp.tPosition,
                                          wNowTick, &tSample,
                                          &tFeedback);
            motor_ApplyPositionEventIsr(&tFocApp.tMotor,
                motor_position_TakeEvent(&tFocApp.tPosition),
                tFeedback.qElectricalSpeedPu);
            eControl = motor_IsrControlStep(&tFocApp.tMotor,
                ePosition == FOC_RESULT_OK ? &tFeedback : NULL);
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
            if (ePosition == FOC_RESULT_OK && eControl == FOC_RESULT_OK) {
                motor_position_ObserverStep(&tFocApp.tPosition,
                                             &tSample);
            } else {
                motor_position_ResetObserver(&tFocApp.tPosition);
            }
#endif
        } else if (ePhase == MOTOR_ISR_IDENTIFICATION_READY) {
            eControl = motor_IsrControlStep(&tFocApp.tMotor, NULL);
        } else if (ePhase == MOTOR_ISR_CAPTURE_ZERO) {
            ePosition = motor_position_CaptureZero(
                &tFocApp.tPosition, wNowTick);
            motor_CompleteAlignIsr(&tFocApp.tMotor, ePosition);
        }
        _foc_app_IdentifyStep(&tFocApp, &tSample, eControl);
    }
#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
    mwaveform.Step();
#endif
#if !defined(__NO_USE_LOG__)
    /* Unsigned subtraction also handles a 32-bit cycle-counter wrap.
     * One invocation must complete within 2^32 ticks; no ISR logging. */
    uint32_t wElapsed = (uint32_t)get_system_ticks() - wNowTick;
    tFocApp.tIsrTiming.qwTotalTicks += wElapsed;
    tFocApp.tIsrTiming.wSamples += 1U;
    if (wElapsed > tFocApp.tIsrTiming.wMaximumTicks) {
        tFocApp.tIsrTiming.wMaximumTicks = wElapsed;
    }
#endif
}

MODUS_DECLARE_OBJECT(foc_app, FocApp, FOC_APP_INIT_CONFIG)
