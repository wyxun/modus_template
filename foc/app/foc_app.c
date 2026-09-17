/****************************************************************************
 * @file    foc_app.c
 * @brief   MODUS composition and scheduling class for one FOC Motor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_app.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "foc_port_config.h"
#include "foc_config.h"
#include "motor_config.h"
#include "mdebug/mwaveform.h"
#include "mdebug/util_debug.h"
#include "perf_counter.h"
#include "perfc_task_pt.h"

#if MSHELL_ENABLE
#include "mdebug/mshell.h"
#endif

static modus_base_t s_tFocAppBase;
extern foc_app_t tFocApp;
static int foc_app_Clock(uintptr_t wObjectAddr);
static int foc_app_Run(uintptr_t wObjectAddr);
static bool foc_app_GetHfAverage(foc_app_t *ptThis,
                                 uint32_t *pwAverageTicks);
static void foc_app_ReportHfAverage(foc_app_t *ptThis);
#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
static void foc_app_WaveformInit(foc_app_t *ptThis,
                                 uint32_t wPeriodNanoseconds);
static void foc_app_WaveformStep(void);
#define FOC_WAVEFORM_CHANNEL_INVALID 0xFFU
#define FOC_WAVEFORM_SINE_POINTS    40U
static const float s_afWaveSine[FOC_WAVEFORM_SINE_POINTS] = {
    0.000000f, 0.156434f, 0.309017f, 0.453990f,
    0.587785f, 0.707107f, 0.809017f, 0.891007f,
    0.951057f, 0.987688f, 1.000000f, 0.987688f,
    0.951057f, 0.891007f, 0.809017f, 0.707107f,
    0.587785f, 0.453990f, 0.309017f, 0.156434f,
    0.000000f, -0.156434f, -0.309017f, -0.453990f,
    -0.587785f, -0.707107f, -0.809017f, -0.891007f,
    -0.951057f, -0.987688f, -1.000000f, -0.987688f,
    -0.951057f, -0.891007f, -0.809017f, -0.707107f,
    -0.587785f, -0.453990f, -0.309017f, -0.156434f,
};
static float s_fWaveSine = 0.0f;
static uint8_t s_chWaveSineIndex = 0U;
static int16_t s_hwWaveSequence = 0;
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
 * @brief Build the Motor config bound to this App's Encoder.
 * @param ptConfig App configuration.
 * @param ptMotorConfig Output Motor configuration.
 * @param bEncoderReady Whether the Encoder was initialized.
 * @return FOC_RESULT_OK or an invalid speed configuration.
 */
static foc_result_t foc_app_BindMotorConfig(
    foc_app_t *ptApp,
    const foc_app_cfg_t *ptConfig,
    motor_cfg_t *ptMotorConfig,
    bool bEncoderReady)
{
    foc_result_t eResult = FOC_RESULT_OK;

    *ptMotorConfig = ptConfig->tMotorCfg;
    ptMotorConfig->tParams.wVoltageBaseMillivolt =
        ptConfig->wVoltageBaseMillivolt;
    ptMotorConfig->qElectricalSpeedBaseTurnsPerSecond =
        ptConfig->qElectricalSpeedBaseTurnsPerSecond;
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
    if (bEncoderReady) {
        ptMotorConfig->tPosition.ptOps = &g_tFocEncoderPositionOps;
        ptMotorConfig->tPosition.pContext = &ptApp->tEncoder;
    } else {
        ptMotorConfig->tPosition = (motor_position_if_t){0};
    }
    return FOC_RESULT_OK;
}

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
/**
 * @brief Register 10 kHz diagnostics and 1 kHz reference channels.
 * @param ptThis FOC App object owning the sampled speed.
 * @return None.
 */
static void foc_app_WaveformInit(foc_app_t *ptThis,
                                 uint32_t wPeriodNanoseconds)
{
    uint8_t chSine = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chSequence = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chSpeed = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chSpeedRef = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chIq = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chIqRef = FOC_WAVEFORM_CHANNEL_INVALID;
    uint32_t wActualRateHz = 0U;
    uint32_t wActualSpeedRefHz = 0U;
    uint32_t wActualIqRefHz = 0U;
    int nResult = MODUS_SUCCESS;

    if (ptThis == NULL) {
        return;
    }
    s_fWaveSine = 0.0f;
    s_chWaveSineIndex = 0U;
    s_hwWaveSequence = 0;
    nResult = mwaveform.Init(NULL);
    if (nResult != MODUS_SUCCESS) {
        MLOGF(W, "FOC waveform init failed (%d)\r\n", nResult);
        return;
    }
    chSine = mwaveform.AddVariable(
        "Sine500", 1000.0f, (void *)&s_fWaveSine,
        MWAVEFORM_VAR_FLOAT);
    chSequence = mwaveform.AddVariable(
        "WaveSeq", 1.0f, (void *)&s_hwWaveSequence,
        MWAVEFORM_VAR_RAW);
    chSpeed = mwaveform.AddVariable(
        "SpeedPU", 100.0f,
        (void *)&ptThis->tMotor.tInput.qElectricalSpeedPu,
        MWAVEFORM_VAR_FLOAT);
    chSpeedRef = mwaveform.AddVariable(
        "SpeedRefPU", 100.0f,
        (void *)&ptThis->tMotor.tCommand.qSpeedReferencePu,
        MWAVEFORM_VAR_FLOAT);
    chIq = mwaveform.AddVariable(
        "Iq", 1000.0f,
        (void *)&ptThis->tMotor.tCore.tCurrent.qQ,
        MWAVEFORM_VAR_FLOAT);
    chIqRef = mwaveform.AddVariable(
        "IqRef", 1000.0f,
        (void *)&ptThis->tMotor.tCommand.tCurrentReference.qQ,
        MWAVEFORM_VAR_FLOAT);
    if (chSine == FOC_WAVEFORM_CHANNEL_INVALID ||
        chSequence == FOC_WAVEFORM_CHANNEL_INVALID ||
        chSpeed == FOC_WAVEFORM_CHANNEL_INVALID ||
        chSpeedRef == FOC_WAVEFORM_CHANNEL_INVALID ||
        chIq == FOC_WAVEFORM_CHANNEL_INVALID ||
        chIqRef == FOC_WAVEFORM_CHANNEL_INVALID) {
        MLOGF(W, "%s\r\n", "FOC waveform channel registration failed");
        return;
    }
    mwaveform.SetRate(0U);
    wActualRateHz = mwaveform.SetStreamRate(wPeriodNanoseconds, 10000U);
    if (wActualRateHz != 10000U) {
        MLOGF(W, "%s\r\n", "FOC waveform 10 kHz stream unavailable");
        return;
    }
    wActualSpeedRefHz = mwaveform.SetChannelRate(chSpeedRef, 1000U);
    wActualIqRefHz = mwaveform.SetChannelRate(chIqRef, 1000U);
    if (wActualSpeedRefHz != 1000U || wActualIqRefHz != 1000U) {
        MLOGF(W, "%s\r\n", "FOC waveform reference rate unavailable");
        return;
    }
    mwaveform.Start();
    MLOGF(I, "FOC waveform %lu Hz; refs 1000 Hz\r\n",
          (unsigned long)wActualRateHz);
}

/**
 * @brief Update the reference sine and sequence before each 20 kHz sample.
 * @param None.
 * @return None.
 */
static void foc_app_WaveformStep(void)
{
    if (s_chWaveSineIndex >=
        (uint8_t)(FOC_WAVEFORM_SINE_POINTS - 1U)) {
        s_chWaveSineIndex = 0U;
    } else {
        s_chWaveSineIndex = (uint8_t)(s_chWaveSineIndex + 1U);
    }
    s_fWaveSine = s_afWaveSine[s_chWaveSineIndex];
    if (s_hwWaveSequence >= 29999) {
        s_hwWaveSequence = 0;
    } else {
        s_hwWaveSequence = (int16_t)(s_hwWaveSequence + 1);
    }
    mwaveform.Step();
}
#endif

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    foc_app_cfg_t *ptConfig = (foc_app_cfg_t *)wObjectCfgAddr;
    motor_cfg_t tMotorConfig = {0};
    foc_encoder_cfg_t tEncoderConfig = {0};
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

    tEncoderConfig = ptConfig->tEncoderCfg;
    eEncoder = foc_encoder_Init(&ptThis->tEncoder,
                                &tEncoderConfig);
    if (eEncoder != FOC_RESULT_OK && eEncoder != FOC_RESULT_DISABLED) {
        return (int)eEncoder;
    }
    if (ptConfig->wVoltageBaseMillivolt == 0U ||
        ptConfig->wHighFrequencyPeriodNanoseconds == 0U ||
        ptConfig->qElectricalSpeedBaseTurnsPerSecond <= FOC_ZERO) {
        return MODUS_EFAIL;
    }
    eResult = foc_app_BindMotorConfig(
        ptThis, ptConfig, &tMotorConfig, eEncoder == FOC_RESULT_OK);
    if (eResult != FOC_RESULT_OK) {
        return (int)eResult;
    }
    eMotor = motor_Init(&ptThis->tMotor, &tMotorConfig);
    if (eMotor != FOC_RESULT_OK) {
        return (int)eMotor;
    }
    nBaseResult = mbase_Init(ptThis->ptBase, &s_tFocAppBaseCfg);
    if (nBaseResult != MODUS_SUCCESS) {
        motor_Stop(&ptThis->tMotor);
        return nBaseResult;
    }
    ptThis->bReady = eEncoder == FOC_RESULT_OK;
#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
    foc_app_WaveformInit(
        ptThis, ptConfig->wHighFrequencyPeriodNanoseconds);
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
        eResult = foc_encoder_Run(&ptThis->tEncoder);
        if (eResult != FOC_RESULT_OK) {
            ptThis->lBackoffTimestamp = get_system_ticks() +
                perfc_convert_ms_to_ticks(100U);
            continue;
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

    wNowTick = (uint32_t)lStartTicks;
    if (tFocApp.bReady) {
        motor_IsrStep(&tFocApp.tMotor, wNowTick);
    }
#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
    foc_app_WaveformStep();
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

    if (!perfc_is_time_out_ms(1000U,
                              &ptThis->tHfStats.lReportTimestamp, true)) {
        return;
    }
    if (!foc_app_GetHfAverage(ptThis, &wAverageTicks)) {
        return;
    }
    wAverageMicroseconds = (uint32_t)perfc_convert_ticks_to_us(
        (int64_t)wAverageTicks);
    MLOGF(T, "FOC HF ISR avg=%lu cycles/%lu us\r\n",
          (unsigned long)wAverageTicks,
          (unsigned long)wAverageMicroseconds);
}

#if MSHELL_ENABLE
/**
 * @brief Print the Motor status without touching its internal members.
 * @param ptMotor Motor object.
 * @return None.
 */
static void foc_app_PrintStatus(const motor_t *ptMotor)
{
    motor_status_t tStatus = {0};
    foc_result_t eResult = motor_GetStatus(ptMotor, &tStatus);

    if (eResult != FOC_RESULT_OK) {
        MLOGF(E, "motor status unavailable (%d)\r\n", (int)eResult);
        return;
    }
    MLOGF(I, "motor state=%u fault=0x%08X mode=%u pwm=%u\r\n",
          (unsigned)tStatus.eState, (unsigned)tStatus.wFaults,
          (unsigned)tStatus.eMode, (unsigned)tStatus.bPwmEnabled);
}

/**
 * @brief Print the latest age-checked mechanical Encoder snapshot.
 * @param ptEncoder Encoder object.
 * @return None.
 */
static void foc_app_PrintEncoder(const foc_encoder_t *ptEncoder)
{
    foc_position_t tPosition = {0};
    uint32_t wNowTick = (uint32_t)get_system_ticks();
    foc_result_t eResult = foc_encoder_GetPosition(
        ptEncoder, wNowTick, &tPosition);

    if (eResult != FOC_RESULT_OK) {
        MLOGF(W, "encoder data unavailable (%d)\r\n", (int)eResult);
        return;
    }
    MLOGF(I, "encoder valid=%u mech=%.2f deg mech_speed=%.3f turn/s\r\n",
          (unsigned)tPosition.bValid,
          foc_angle_to_turns(tPosition.tMechanicalAngle) * 360.0f,
          foc_to_float(tPosition.qMechanicalSpeed));
}

/**
 * @brief Parse and submit one Motor command family command.
 * @param args Command arguments after the motor command name.
 * @return None.
 */
static void foc_app_CmdMotor(const char *args)
{
    float fD = 0.0f;
    float fQ = 0.0f;
    int nScanned = 0;
    bool bStarted = false;
    foc_result_t eResult = FOC_RESULT_OK;

    if (args == NULL) {
        return;
    }
    if (strncmp(args, "stop", 4U) == 0) {
        motor_Stop(&tFocApp.tMotor);
        return;
    } else if (strncmp(args, "clear", 5U) == 0) {
        eResult = motor_ClearFault(&tFocApp.tMotor);
    } else if (strncmp(args, "align", 5U) == 0) {
        eResult = motor_RequestPositionCalibration(&tFocApp.tMotor);
    } else if (strncmp(args, "speed", 5U) == 0) {
        nScanned = sscanf(args + 5, "%f", &fQ);
        if (nScanned != 1) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
        } else {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_SPEED);
            bStarted = eResult == FOC_RESULT_OK;
            if (eResult == FOC_RESULT_OK) {
                eResult = motor_SetSpeedReference(&tFocApp.tMotor,
                                                  foc_from_float(fQ));
            }
        }
    } else if (strncmp(args, "current", 7U) == 0) {
        nScanned = sscanf(args + 7, "%f %f", &fD, &fQ);
        if (nScanned != 2) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
        } else {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_CURRENT);
            bStarted = eResult == FOC_RESULT_OK;
            if (eResult == FOC_RESULT_OK) {
                eResult = motor_SetCurrentReference(&tFocApp.tMotor,
                                                    foc_from_float(fD),
                                                    foc_from_float(fQ));
            }
        }
    } else if (strncmp(args, "voltage", 7U) == 0) {
        nScanned = sscanf(args + 7, "%f %f", &fD, &fQ);
        if (nScanned != 2) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
        } else {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_VOLTAGE);
            bStarted = eResult == FOC_RESULT_OK;
            if (eResult == FOC_RESULT_OK) {
                eResult = motor_SetVoltageReference(&tFocApp.tMotor,
                                                    foc_from_float(fD),
                                                    foc_from_float(fQ));
            }
        }
    } else if (strncmp(args, "status", 6U) == 0) {
        foc_app_PrintStatus(&tFocApp.tMotor);
        return;
    } else if (strncmp(args, "encoder", 7U) == 0) {
        foc_app_PrintEncoder(&tFocApp.tEncoder);
        return;
    } else {
        MLOGF(I, "usage: motor speed <pu> | current <d> <q> | "
              "voltage <d> <q> | align | stop | clear\r\n"
              "       motor status | encoder\r\n");
        return;
    }
    if (bStarted && eResult != FOC_RESULT_OK) {
        motor_Stop(&tFocApp.tMotor);
    }
    if (eResult != FOC_RESULT_OK) {
        MLOGF(W, "motor command rejected (%d)\r\n", (int)eResult);
    }
}

MODUS_SHELL_CMD(motor, foc_app_CmdMotor,
                "FOC Motor: control/align/stop/clear/status/encoder");
#endif

#if FOC_PORT_HAS_POSITION
MODUS_DECLARE_OBJECT(foc_app, FocApp,
    .tMotorCfg = {
        .tParams = {
            .chPolePairs = MOTOR_POLE_PAIRS,
            .wResistanceMilliohm = MOTOR_RESISTANCE_MILLIOHM,
            .wInductanceDMicroHenry = MOTOR_INDUCTANCE_D_MICROHENRY,
            .wInductanceQMicroHenry = MOTOR_INDUCTANCE_Q_MICROHENRY,
        },
        .tLimits = {
            .qMaxSpeedReference = FOC_SCALAR(100.0f),
            .qMaxPhaseCurrent = FOC_SCALAR(1.0f),
            .qMaxModulation = FOC_SCALAR(0.5773502692f),
        },
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
        .tObserverCfg = {
            .tSmo = {
                .wSamplePeriodNanoseconds = 50000U,
                .wBemfCutoffRadiansPerSecond = 10000U,
                .wSlidingGainMillivolt = 3500U,
                .qCurrentEstimateLimit = FOC_ONE,
            },
        },
#endif
        .tCurrentPiParams = {
                .tKp = {0, FOC_SCALAR(0.20f)},
                .tKiTs = {0, FOC_SCALAR(0.005f)},
                .tKdOverTs = {0, FOC_ZERO},
                .qOutputMinimum = FOC_SCALAR(-0.55f),
                .qOutputMaximum = FOC_SCALAR(0.55f),
                .qIntegratorMinimum = FOC_SCALAR(-0.50f),
                .qIntegratorMaximum = FOC_SCALAR(0.50f),
        },
        .tSpeedPiParams = {
                .tKp = {0, FOC_SCALAR(0.20f)},
                .tKiTs = {0, FOC_SCALAR(0.005f)},
                .tKdOverTs = {0, FOC_ZERO},
                .qOutputMinimum = FOC_SCALAR(-0.10f),
                .qOutputMaximum = FOC_SCALAR(0.10f),
                .qIntegratorMinimum = FOC_SCALAR(-0.10f),
                .qIntegratorMaximum = FOC_SCALAR(0.10f),
        },
        .wAdcCalibrationTimeoutSteps = 2000U,
        .wAlignSteps = 30000U,
        .chSpeedLoopDiv = 20U,
        .qAlignCurrent = FOC_SCALAR(0.1f),
    },
    .tEncoderCfg = {
        .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
        .wInvalidTimeoutUs = 5000U,
        .bDirectionInvert = false,
        .ptSensor = &g_tFocEncoderSensorInterface,
    },
    .wVoltageBaseMillivolt = MOTOR_BASE_VOLTAGE_MV,
    .wHighFrequencyPeriodNanoseconds = MOTOR_HF_PERIOD_NANOSECONDS,
    .qElectricalSpeedBaseTurnsPerSecond =
        FOC_SCALAR(MOTOR_BASE_ELECTRICAL_HZ),
    )
#else
MODUS_DECLARE_OBJECT(foc_app, FocApp,
    .tMotorCfg = {
        .tParams = {
            .chPolePairs = MOTOR_POLE_PAIRS,
            .wResistanceMilliohm = MOTOR_RESISTANCE_MILLIOHM,
            .wInductanceDMicroHenry = MOTOR_INDUCTANCE_D_MICROHENRY,
            .wInductanceQMicroHenry = MOTOR_INDUCTANCE_Q_MICROHENRY,
        },
        .tLimits = {
            .qMaxSpeedReference = FOC_SCALAR(100.0f),
            .qMaxPhaseCurrent = FOC_SCALAR(1.0f),
            .qMaxModulation = FOC_SCALAR(0.5773502692f),
        },
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
        .tObserverCfg = {
            .tSmo = {
                .wSamplePeriodNanoseconds = 50000U,
                .wBemfCutoffRadiansPerSecond = 10000U,
                .wSlidingGainMillivolt = 3500U,
                .qCurrentEstimateLimit = FOC_ONE,
            },
        },
#endif
        .tCurrentPiParams = {
                .tKp = {0, FOC_SCALAR(0.20f)},
                .tKiTs = {0, FOC_SCALAR(0.005f)},
                .tKdOverTs = {0, FOC_ZERO},
                .qOutputMinimum = FOC_SCALAR(-0.55f),
                .qOutputMaximum = FOC_SCALAR(0.55f),
                .qIntegratorMinimum = FOC_SCALAR(-0.50f),
                .qIntegratorMaximum = FOC_SCALAR(0.50f),
        },
        .tSpeedPiParams = {
                .tKp = {0, FOC_SCALAR(0.20f)},
                .tKiTs = {0, FOC_SCALAR(0.005f)},
                .tKdOverTs = {0, FOC_ZERO},
                .qOutputMinimum = FOC_SCALAR(-0.10f),
                .qOutputMaximum = FOC_SCALAR(0.10f),
                .qIntegratorMinimum = FOC_SCALAR(-0.10f),
                .qIntegratorMaximum = FOC_SCALAR(0.10f),
        },
        .wAdcCalibrationTimeoutSteps = 2000U,
        .wAlignSteps = 30000U,
        .chSpeedLoopDiv = 20U,
        .qAlignCurrent = FOC_SCALAR(0.1f),
    },
    .tEncoderCfg = {0},
    .wVoltageBaseMillivolt = MOTOR_BASE_VOLTAGE_MV,
    .wHighFrequencyPeriodNanoseconds = MOTOR_HF_PERIOD_NANOSECONDS,
    .qElectricalSpeedBaseTurnsPerSecond =
        FOC_SCALAR(MOTOR_BASE_ELECTRICAL_HZ),
    )
#endif
