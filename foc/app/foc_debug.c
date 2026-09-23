/**
 * @file    foc_debug.c
 * @brief   Debug commands and waveform hooks owned by the FOC application.
 * @note    This file is a mechanical split from foc_app.c. Debug behavior
 *          and command names remain unchanged.
 */

#include "foc_debug.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "mdebug/util_debug.h"
#include "motor_config.h"
#include "perf_counter.h"

#if MSHELL_ENABLE
#include "mdebug/mshell.h"
#endif

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
#include "mdebug/mwaveform.h"
#endif

extern foc_app_t tFocApp;

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
#define FOC_WAVEFORM_CHANNEL_INVALID 0xFFU

typedef enum {
    FOC_DEBUG_WAVEFORM_I_ALPHA = 0,
    FOC_DEBUG_WAVEFORM_I_BETA,
    FOC_DEBUG_WAVEFORM_U_ALPHA,
    FOC_DEBUG_WAVEFORM_U_BETA,
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    FOC_DEBUG_WAVEFORM_BEMF_ALPHA,
    FOC_DEBUG_WAVEFORM_BEMF_BETA,
    FOC_DEBUG_WAVEFORM_ENCODER_ANGLE,
    FOC_DEBUG_WAVEFORM_SMO_ANGLE,
#endif
    FOC_DEBUG_WAVEFORM_CHANNEL_COUNT,
} foc_debug_waveform_channel_e;

static const char *const s_achWaveformNames[] = {
    "Ialpha", "Ibeta", "UmodelAlpha", "UmodelBeta",
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    "eAlpha", "eBeta", "EncoderElec", "SmoAngle",
#endif
};

static float s_afWaveformValues[FOC_DEBUG_WAVEFORM_CHANNEL_COUNT];

static void foc_debug_CaptureWaveform(const foc_app_t *ptThis)
{
    const motor_t *ptMotor = NULL;

    if (ptThis == NULL) {
        return;
    }
    ptMotor = &ptThis->tMotor;
    s_afWaveformValues[FOC_DEBUG_WAVEFORM_I_ALPHA] =
        ptMotor->tInput.tCurrentAlphaBeta.qAlpha;
    s_afWaveformValues[FOC_DEBUG_WAVEFORM_I_BETA] =
        ptMotor->tInput.tCurrentAlphaBeta.qBeta;
    s_afWaveformValues[FOC_DEBUG_WAVEFORM_U_ALPHA] =
        ptMotor->tCore.tVoltageAlphaBeta.qAlpha;
    s_afWaveformValues[FOC_DEBUG_WAVEFORM_U_BETA] =
        ptMotor->tCore.tVoltageAlphaBeta.qBeta;
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    {
        const foc_observer_output_t *ptEstimate =
            &ptThis->tPosition.tObserver.tOutput;
        bool bEncoderAngleValid =
            ptThis->tPosition.eSource == MOTOR_POSITION_SOURCE_SENSOR &&
            ptMotor->tInput.bAngleValid;

        s_afWaveformValues[FOC_DEBUG_WAVEFORM_BEMF_ALPHA] =
            ptThis->tPosition.tObserver.tSmo.tAxis[0].qBemf;
        s_afWaveformValues[FOC_DEBUG_WAVEFORM_BEMF_BETA] =
            ptThis->tPosition.tObserver.tSmo.tAxis[1].qBemf;
        s_afWaveformValues[FOC_DEBUG_WAVEFORM_ENCODER_ANGLE] =
            bEncoderAngleValid ?
            foc_angle_to_turns(ptMotor->tInput.tElectricalAngle) : -1.0f;
        s_afWaveformValues[FOC_DEBUG_WAVEFORM_SMO_ANGLE] =
            ptEstimate->bValid ?
            foc_angle_to_turns(ptEstimate->tElectricalAngle) : -1.0f;
    }
#endif
}

void foc_debug_WaveformInit(foc_app_t *ptThis,
                            uint32_t wPeriodNanoseconds)
{
    uint32_t wIndex = 0U;
    uint8_t chChannel = FOC_WAVEFORM_CHANNEL_INVALID;
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
    for (wIndex = 0U;
         wIndex < FOC_DEBUG_WAVEFORM_CHANNEL_COUNT; wIndex++) {
        chChannel = mwaveform.AddVariable(
            s_achWaveformNames[wIndex], 1000.0f,
            &s_afWaveformValues[wIndex], MWAVEFORM_VAR_FLOAT);
        if (chChannel == FOC_WAVEFORM_CHANNEL_INVALID) {
            MLOGF(W, "%s\r\n",
                  "FOC waveform channel registration failed");
            return;
        }
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

void foc_debug_WaveformStep(void)
{
    foc_debug_CaptureWaveform(&tFocApp);
    mwaveform.Step();
}
#endif

#if MSHELL_ENABLE

/**
 * @brief Print the Motor status without touching its internal members.
 * @param ptMotor Motor object.
 * @return None.
 */
static void foc_debug_PrintStatus(const motor_t *ptMotor)
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
static void foc_debug_PrintEncoder(const foc_encoder_t *ptEncoder)
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
static void foc_debug_CmdMotor(const char *args)
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
                eResult = motor_SetSpeedReference(
                    &tFocApp.tMotor, foc_from_float(fQ));
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
                eResult = motor_SetCurrentReference(
                    &tFocApp.tMotor,
                    foc_from_float(fD), foc_from_float(fQ));
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
                eResult = motor_SetVoltageReference(
                    &tFocApp.tMotor,
                    foc_from_float(fD), foc_from_float(fQ));
            }
        }
    } else if (strncmp(args, "status", 6U) == 0) {
        foc_debug_PrintStatus(&tFocApp.tMotor);
        return;
    } else if (strncmp(args, "encoder", 7U) == 0) {
        foc_debug_PrintEncoder(&tFocApp.tEncoder);
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

MODUS_SHELL_CMD(motor, foc_debug_CmdMotor,
                "FOC Motor: control/align/stop/clear/status/encoder");

/**
 * @brief Compare a shell argument with a complete keyword.
 * @param pchArgs Shell argument string.
 * @param pchKeyword Keyword to compare.
 * @return true when the argument is exactly the keyword.
 */
static bool foc_debug_IsKeyword(const char *pchArgs,
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
static void foc_debug_PrintIdentifyStatus(const identify_t *ptIdentify)
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
static void foc_debug_CmdIdentify(const char *args)
{
    foc_result_t eResult = FOC_RESULT_OK;
    bool bStartResistance = false;
    bool bStartInductance = false;

    if (args == NULL) {
        return;
    }
    bStartResistance = foc_debug_IsKeyword(args, "resistance");
    bStartInductance = foc_debug_IsKeyword(args, "inductance");
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
                .wInjectionFrequencyHz = MOTOR_CONFIG_IDENTIFY_LD_FREQUENCY_HZ,
                .hwCaptureDelayCycles = MOTOR_CONFIG_IDENTIFY_LD_CAPTURE_DELAY,
                .hwCaptureSampleCount = MOTOR_CONFIG_IDENTIFY_LD_CAPTURE_SAMPLES,
                .hwHalfCycleCount = MOTOR_CONFIG_IDENTIFY_LD_HALF_CYCLES,
                .qModulationAmplitude =
                    FOC_SCALAR(MOTOR_CONFIG_IDENTIFY_LD_MODULATION_PU),
                .qMaxIdentificationCurrent =
                    FOC_SCALAR(MOTOR_CONFIG_IDENTIFY_LD_MAX_CURRENT_PU),
                .qMinCurrentDelta =
                    FOC_SCALAR(MOTOR_CONFIG_IDENTIFY_LD_MIN_DELTA_PU),
                .qMaxElectricalSpeedPu =
                    FOC_SCALAR(MOTOR_CONFIG_IDENTIFY_LD_MAX_SPEED_PU),
                .hwMotionFaultCycles = MOTOR_CONFIG_IDENTIFY_LD_MOTION_CYCLES,
            };

            eResult = identify_StartInductance(
                &tFocApp.tIdentify, &tConfig);
        }
    } else if (foc_debug_IsKeyword(args, "status")) {
        foc_debug_PrintIdentifyStatus(&tFocApp.tIdentify);
        return;
    } else if (foc_debug_IsKeyword(args, "stop")) {
        identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
        return;
    } else if (foc_debug_IsKeyword(args, "reset")) {
        identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
        eResult = identify_Reset(
            &tFocApp.tIdentify, &tFocApp.tMotor);
    } else {
        MLOGF(I, "usage: identify resistance | inductance | status | "
              "stop | reset\r\n");
        return;
    }
    if (eResult != FOC_RESULT_OK) {
        MLOGF(W, "identify command rejected (%d)\r\n", (int)eResult);
    }
}

MODUS_SHELL_CMD(identify, foc_debug_CmdIdentify,
                "FOC identification: resistance/inductance/status/stop/reset");

#endif /* MSHELL_ENABLE */
