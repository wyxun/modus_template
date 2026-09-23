/****************************************************************************
 * @file    foc_app.c
 * @brief   MODUS composition and scheduling class for one FOC Motor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_app.h"

#include <ctype.h>
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
#if defined(FOC_POSITION_STATIC_BINDING)
        ptMotorConfig->tPosition.pContext = &ptApp->tEncoder;
#else
        ptMotorConfig->tPosition.fnGetPosition = foc_encoder_GetPosition;
        ptMotorConfig->tPosition.pContext = &ptApp->tEncoder;
#endif
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
    uint8_t chIu = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chId = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chIq = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chVdCmd = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chVd = FOC_WAVEFORM_CHANNEL_INVALID;
    uint8_t chVq = FOC_WAVEFORM_CHANNEL_INVALID;
    uint32_t wActualRateHz = 0U;
    int nResult = MODUS_SUCCESS;

    if (ptThis == NULL) {
        return;
    }
    nResult = mwaveform.Init(NULL);
    if (nResult != MODUS_SUCCESS) {
        MLOGF(W, "FOC waveform init failed (%d)\r\n", nResult);
        return;
    }
    chIu = mwaveform.AddVariable(
        "Iu", 1000.0f,
        (void *)&ptThis->tMotor.tCurrentAbc.qU,
        MWAVEFORM_VAR_FLOAT);
    chId = mwaveform.AddVariable(
        "Id", 1000.0f,
        (void *)&ptThis->tMotor.tCore.tCurrent.qD,
        MWAVEFORM_VAR_FLOAT);
    chIq = mwaveform.AddVariable(
        "Iq", 1000.0f,
        (void *)&ptThis->tMotor.tCore.tCurrent.qQ,
        MWAVEFORM_VAR_FLOAT);
    chVdCmd = mwaveform.AddVariable(
        "VdCmd", 1000.0f,
        (void *)&ptThis->tMotor.tCommand.tVoltageReference.qD,
        MWAVEFORM_VAR_FLOAT);
    chVd = mwaveform.AddVariable(
        "Vd", 1000.0f,
        (void *)&ptThis->tMotor.tCore.tVoltage.qD,
        MWAVEFORM_VAR_FLOAT);
    chVq = mwaveform.AddVariable(
        "Vq", 1000.0f,
        (void *)&ptThis->tMotor.tCore.tVoltage.qQ,
        MWAVEFORM_VAR_FLOAT);
    if (chIu == FOC_WAVEFORM_CHANNEL_INVALID ||
        chId == FOC_WAVEFORM_CHANNEL_INVALID ||
        chIq == FOC_WAVEFORM_CHANNEL_INVALID ||
        chVdCmd == FOC_WAVEFORM_CHANNEL_INVALID ||
        chVd == FOC_WAVEFORM_CHANNEL_INVALID ||
        chVq == FOC_WAVEFORM_CHANNEL_INVALID) {
        MLOGF(W, "%s\r\n", "FOC waveform channel registration failed");
        return;
    }
    mwaveform.SetRate(0U);
    wActualRateHz = mwaveform.SetStreamRate(wPeriodNanoseconds, 10000U);
    if (wActualRateHz != 10000U) {
        MLOGF(W, "%s\r\n", "FOC waveform 10 kHz stream unavailable");
        return;
    }
    mwaveform.Start();
    MLOGF(I, "FOC waveform %lu Hz\r\n",
          (unsigned long)wActualRateHz);
}

/**
 * @brief Advance the waveform stream before each 20 kHz sample.
 * @param None.
 * @return None.
 */
static void foc_app_WaveformStep(void)
{
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
        switch (tFocApp.tIdentify.eOperation) {
        case IDENTIFY_OPERATION_RESISTANCE: {
            identify_isr_sample_t tIdentifySample = {
                .qCurrentD = tFocApp.tMotor.tCore.tCurrent.qD,
            };

            identify_IsrStep(&tFocApp.tIdentify, &tFocApp.tMotor,
                             &tIdentifySample);
            break;
        }
        case IDENTIFY_OPERATION_INDUCTANCE: {
            identify_isr_sample_t tIdentifySample = {
                .qCurrentD = tFocApp.tMotor.tCore.tCurrent.qD,
                .qCurrentQ = tFocApp.tMotor.tCore.tCurrent.qQ,
                .qElectricalSpeedPu =
                    tFocApp.tMotor.tInput.qElectricalSpeedPu,
                .bMotorFault = tFocApp.tMotor.eState == MOTOR_STATE_FAULT,
                .bAngleValid = tFocApp.tMotor.tInput.bAngleValid,
                .bPwmSaturated = tFocApp.tMotor.tCore.bPwmSaturated,
            };

            tIdentifySample.bDcBusValid =
                FOC_PORT_SAMPLE_DCBUS_MILLIVOLT(
                    &tIdentifySample.wDcBusMillivolt) == FOC_RESULT_OK;
            identify_IsrStep(&tFocApp.tIdentify, &tFocApp.tMotor,
                             &tIdentifySample);
            break;
        }
        case IDENTIFY_OPERATION_NONE:
        default:
            break;
        }
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
    uint32_t wDcBusMillivolt = 0U;
    foc_result_t eDcBus = FOC_RESULT_DISABLED;

    if (!perfc_is_time_out_ms(1000U,
                              &ptThis->tHfStats.lReportTimestamp, true)) {
        return;
    }
    eDcBus = FOC_PORT_SAMPLE_DCBUS_MILLIVOLT(&wDcBusMillivolt);
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
    MLOGF(I, "motor state=%u fault=0x%08X mode=%u pwm=%u "
          "id=%.5f iq=%.5f vd_cmd=%.5f vd=%.5f vq=%.5f "
          "zero=%u angle=%u\r\n",
          (unsigned)tStatus.eState, (unsigned)tStatus.wFaults,
          (unsigned)tStatus.eMode, (unsigned)tStatus.bPwmEnabled,
          (double)foc_to_float(ptMotor->tCore.tCurrent.qD),
          (double)foc_to_float(ptMotor->tCore.tCurrent.qQ),
          (double)foc_to_float(
              ptMotor->tCommand.tVoltageReference.qD),
          (double)foc_to_float(ptMotor->tCore.tVoltage.qD),
          (double)foc_to_float(ptMotor->tCore.tVoltage.qQ),
          (unsigned)tStatus.bElectricalZeroValid,
          (unsigned)ptMotor->tInput.bAngleValid);
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
        identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
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

/**
 * @brief Compare a shell argument with a complete keyword.
 * @param pchArgs Shell argument string.
 * @param pchKeyword Keyword to compare.
 * @return true when the argument is exactly the keyword.
 */
static bool foc_app_IsKeyword(const char *pchArgs,
                              const char *pchKeyword)
{
    size_t wLength = 0U;
    const char *pchTail = NULL;

    if (pchArgs == NULL || pchKeyword == NULL) {
        return false;
    }
    wLength = strlen(pchKeyword);
    if (strncmp(pchArgs, pchKeyword, wLength) != 0) {
        return false;
    }
    pchTail = pchArgs + wLength;
    while (isspace((unsigned char)*pchTail) != 0) {
        pchTail++;
    }
    return *pchTail == '\0';
}

/**
 * @brief Print the identification status.
 * @param ptIdentify Identification object.
 * @return None.
 */
static void foc_app_PrintIdentifyStatus(const identify_t *ptIdentify)
{
    identify_status_t tStatus = {0};
    foc_result_t eResult = identify_GetStatus(ptIdentify, &tStatus);

    if (eResult != FOC_RESULT_OK) {
        MLOGF(E, "identify status unavailable (%d)\r\n", (int)eResult);
        return;
    }
    MLOGF(I, "identify state=%u operation=%u result=%d\r\n",
          (unsigned)tStatus.eState,
          (unsigned)tStatus.eOperation,
          (int)tStatus.eLastResult);
}

/**
 * @brief Parse and start a fixed parameter-identification flow.
 * @param args Command arguments after the identify command name.
 * @return None.
 */
static void foc_app_CmdIdentify(const char *args)
{
    foc_result_t eResult = FOC_RESULT_OK;
    bool bStartResistance = false;
    bool bStartInductance = false;

    if (args == NULL) {
        return;
    }
    bStartResistance = foc_app_IsKeyword(args, "resistance");
    bStartInductance = foc_app_IsKeyword(args, "inductance");
    if (bStartResistance || bStartInductance) {
        motor_status_t tMotorStatus = {0};

        eResult = motor_GetStatus(&tFocApp.tMotor, &tMotorStatus);
        if (eResult == FOC_RESULT_OK &&
            (tMotorStatus.eState != MOTOR_STATE_IDLE ||
             tMotorStatus.bPwmEnabled)) {
            MLOGF(W, "identify requires motor idle after align\r\n");
            eResult = FOC_RESULT_BUSY;
        } else if (eResult == FOC_RESULT_OK &&
                   !tMotorStatus.bElectricalZeroValid) {
            MLOGF(W, "identify requires completed motor align\r\n");
            eResult = FOC_RESULT_SAFETY;
        } else if (eResult == FOC_RESULT_OK && bStartResistance) {
            eResult = identify_StartResistance(&tFocApp.tIdentify);
        } else if (eResult == FOC_RESULT_OK) {
            identify_inductance_cfg_t tConfig = {
                .wInjectionFrequencyHz = MOTOR_IDENTIFY_LD_FREQUENCY_HZ,
                .hwCaptureDelayCycles = MOTOR_IDENTIFY_LD_CAPTURE_DELAY,
                .hwCaptureSampleCount = MOTOR_IDENTIFY_LD_CAPTURE_SAMPLES,
                .hwHalfCycleCount = MOTOR_IDENTIFY_LD_HALF_CYCLES,
                .qModulationAmplitude =
                    FOC_SCALAR(MOTOR_IDENTIFY_LD_MODULATION_PU),
                .qMaxIdentificationCurrent =
                    FOC_SCALAR(MOTOR_IDENTIFY_LD_MAX_CURRENT_PU),
                .qMinCurrentDelta =
                    FOC_SCALAR(MOTOR_IDENTIFY_LD_MIN_DELTA_PU),
                .qMaxElectricalSpeedPu =
                    FOC_SCALAR(MOTOR_IDENTIFY_LD_MAX_SPEED_PU),
                .hwMotionFaultCycles = MOTOR_IDENTIFY_LD_MOTION_CYCLES,
            };

            eResult = identify_StartInductance(&tFocApp.tIdentify,
                                               &tConfig);
        } else {
            /* Motor status failure is reported below. */
        }
    } else if (foc_app_IsKeyword(args, "status")) {
        foc_app_PrintIdentifyStatus(&tFocApp.tIdentify);
        return;
    } else if (foc_app_IsKeyword(args, "stop")) {
        identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
        return;
    } else if (foc_app_IsKeyword(args, "reset")) {
        identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
        eResult = identify_Reset(&tFocApp.tIdentify, &tFocApp.tMotor);
    } else {
        MLOGF(I, "usage: identify resistance | inductance | status | "
              "stop | reset\r\n");
        return;
    }
    if (eResult != FOC_RESULT_OK) {
        MLOGF(W, "identify command rejected (%d)\r\n", (int)eResult);
    }
}

MODUS_SHELL_CMD(identify, foc_app_CmdIdentify,
                "FOC identification: resistance/inductance/status/stop/reset");
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
#if !defined(FOC_ENCODER_STATIC_BINDING)
        .ptSensor = &g_tFocEncoderSensorInterface,
#endif
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
