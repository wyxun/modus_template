/****************************************************************************
 * @file    foc_app_encoder_command_test.c
 * @brief   Host test for the Motor encoder diagnostic command.
 * @author  Codex
 * @date    2026-09-12
 ****************************************************************************/

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../hal/foc_port.h"
#include "../observer/foc_encoder.h"

const foc_adc_if_t g_tFocAdcInterface = {0};
const foc_pwm_if_t g_tFocPwmInterface = {0};
const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface = {0};

#include "../app/foc_app.c"

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
#include "mdebug/mwaveform.h"

static uint32_t s_wWaveInitCount = 0U;
static uint32_t s_wWaveStartCount = 0U;
static uint32_t s_wWaveStepCount = 0U;
static uint32_t s_wWaveDecimation = UINT32_MAX;
static uint32_t s_wWaveIsrPeriodNs = 0U;
static uint32_t s_wWaveTargetHz = 0U;
static uint32_t s_wWaveChannelRateCount = 0U;
static uint8_t s_chWaveCount = 0U;
static char s_achWaveNames[6][16] = {{0}};
static uint8_t s_achWaveTypes[6] = {0U};
static float s_afWaveScales[6] = {0.0f};
static void *s_apvWaveValues[6] = {NULL};
static uint32_t s_awWaveChannelRates[6] = {0U};

/**
 * @brief Record waveform API initialization.
 * @param ptProtocol Waveform protocol requested by the app.
 * @return MODUS_SUCCESS.
 */
static int test_WaveInit(const mwaveform_protocol_t *ptProtocol)
{
    (void)ptProtocol;
    s_wWaveInitCount++;
    return MODUS_SUCCESS;
}

/**
 * @brief Capture waveform channel registration arguments.
 * @param pchName Channel name.
 * @param fScale Raw-value scale.
 * @param pvValue Address of the sampled value.
 * @param chType Waveform variable type.
 * @return Mock channel identifier.
 */
static uint8_t test_WaveAddVariable(const char *pchName, float fScale,
                                    void *pvValue, uint8_t chType)
{
    size_t hwLength = 0U;
    uint8_t chIndex = s_chWaveCount;

    if (chIndex >= 6U) {
        return 0xFFU;
    }
    hwLength = strlen(pchName);
    if (hwLength >= sizeof(s_achWaveNames[chIndex])) {
        return 0xFFU;
    }
    memcpy(s_achWaveNames[chIndex], pchName, hwLength + 1U);
    s_afWaveScales[chIndex] = fScale;
    s_apvWaveValues[chIndex] = pvValue;
    s_achWaveTypes[chIndex] = chType;
    s_chWaveCount = (uint8_t)(s_chWaveCount + 1U);
    return chIndex;
}

/**
 * @brief Capture per-channel waveform refresh rates.
 * @param chID Channel identifier.
 * @param wHz Requested channel rate.
 * @return Accepted channel rate, or zero for an invalid channel.
 */
static uint32_t test_WaveSetChannelRate(uint8_t chID, uint32_t wHz)
{
    s_wWaveChannelRateCount++;
    if (chID >= s_chWaveCount || chID >= 6U) {
        return 0U;
    }
    s_awWaveChannelRates[chID] = wHz;
    return (wHz == 1000U) ? 1000U : 0U;
}

/**
 * @brief Record waveform start.
 * @param None.
 * @return None.
 */
static void test_WaveStart(void)
{
    s_wWaveStartCount++;
}

/**
 * @brief Record one waveform sample step.
 * @param None.
 * @return None.
 */
static void test_WaveStep(void)
{
    s_wWaveStepCount++;
}

/**
 * @brief Capture external-drive configuration.
 * @param wDecimation Requested scheduler decimation.
 * @return None.
 */
static void test_WaveSetRate(uint32_t wDecimation)
{
    s_wWaveDecimation = wDecimation;
}

/**
 * @brief Capture stream timing and return the expected test rate.
 * @param wIsrPeriodNs ISR period in nanoseconds.
 * @param wTargetHz Requested sample rate.
 * @return Actual stream rate, or zero for unexpected arguments.
 */
static uint32_t test_WaveSetStreamRate(uint32_t wIsrPeriodNs,
                                       uint32_t wTargetHz)
{
    s_wWaveIsrPeriodNs = wIsrPeriodNs;
    s_wWaveTargetHz = wTargetHz;
    return (wIsrPeriodNs == 50000U && wTargetHz == 10000U)
        ? 10000U : 0U;
}

const mwaveform_api_t mwaveform = {
    .Init = test_WaveInit,
    .AddVariable = test_WaveAddVariable,
    .Start = test_WaveStart,
    .Step = test_WaveStep,
    .SetRate = test_WaveSetRate,
    .SetStreamRate = test_WaveSetStreamRate,
    .SetChannelRate = test_WaveSetChannelRate,
};
#endif

static uint32_t s_wPositionCallCount = 0U;
static const foc_encoder_t *s_ptReadEncoder = NULL;
static uint32_t s_wReadTick = 0U;
static foc_result_t s_ePositionResult = FOC_RESULT_OK;
static char s_chLog[160] = {0};
static int64_t s_lSystemTick = 1234;
static int64_t s_lTickStep = 0;
static uint32_t s_wShortTimeoutBudget = 0U;
static bool s_bReportTimeout = false;
static uint32_t s_wMotorStepCount = 0U;
static motor_cfg_t s_tCapturedMotorConfig = {0};

uint8_t g_chGLogMask = MLOG_MASK_ALL;
volatile int32_t g_nOffset = 0;

/**
 * @brief Provide a stable tick for the command test.
 * @param None.
 * @return Test system tick.
 */
int64_t get_system_ticks(void)
{
    int64_t lTick = s_lSystemTick;

    s_lSystemTick += s_lTickStep;
    return lTick;
}

/**
 * @brief Stub Encoder initialization for unused App initialization paths.
 * @param ptEncoder Encoder object.
 * @param ptConfig Encoder configuration.
 * @return Configured Encoder result.
 */
foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_cfg_t *ptConfig)
{
    (void)ptEncoder;
    (void)ptConfig;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub Encoder updates for the unused foreground path.
 * @param ptEncoder Encoder object.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_encoder_Run(foc_encoder_t *ptEncoder)
{
    (void)ptEncoder;
    return FOC_RESULT_OK;
}

const motor_position_ops_t g_tFocEncoderPositionOps = {0};

/**
 * @brief Stub Motor initialization for the unused App initialization path.
 * @param ptMotor Motor object.
 * @param ptConfig Motor configuration.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig)
{
    (void)ptMotor;
    s_tCapturedMotorConfig = *ptConfig;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub the Motor high-frequency path.
 * @param ptMotor Motor object.
 * @param wNowTick Current tick.
 * @return None.
 */
void motor_IsrStep(motor_t *ptMotor, uint32_t wNowTick)
{
    (void)ptMotor;
    (void)wNowTick;
    s_wMotorStepCount++;
}

/**
 * @brief Stub foreground hardware-break polling for the App test.
 * @param ptMotor Motor object.
 * @return None.
 */
void motor_PollBreakFault(motor_t *ptMotor)
{
    (void)ptMotor;
}

/**
 * @brief Stub MODUS base initialization for the unused App path.
 * @param ptBase MODUS base object.
 * @param ptConfig MODUS base configuration.
 * @return Success.
 */
int mbase_Init(modus_base_t *ptBase, modus_base_cfg_t *ptConfig)
{
    (void)ptBase;
    (void)ptConfig;
    return MODUS_SUCCESS;
}

/**
 * @brief Stub microsecond conversion for the unused App foreground path.
 * @param wMicroseconds Duration in microseconds.
 * @return Zero test duration.
 */
int64_t perfc_convert_us_to_ticks(uint32_t wMicroseconds)
{
    return (int64_t)wMicroseconds;
}

/**
 * @brief Stub millisecond conversion for the unused App foreground path.
 * @param wMilliseconds Duration in milliseconds.
 * @return Zero test duration.
 */
int64_t perfc_convert_ms_to_ticks(uint32_t wMilliseconds)
{
    return (int64_t)wMilliseconds * 1000;
}

/**
 * @brief Convert test ticks to microseconds at 100 ticks per microsecond.
 * @param lTicks Number of perf_counter ticks.
 * @return Converted test microseconds.
 */
int64_t perfc_convert_ticks_to_us(int64_t lTicks)
{
    return lTicks / 100;
}

/**
 * @brief Stub timeout checks for the unused App foreground path.
 * @param lPeriod Timeout period.
 * @param plTimestamp Timestamp storage.
 * @param bAutoReload Whether to reload after timeout.
 * @return False.
 */
bool __perfc_is_time_out(int64_t lPeriod,
                         int64_t *plTimestamp,
                         bool bAutoReload)
{
    (void)plTimestamp;
    (void)bAutoReload;
    if (lPeriod == 1000000) {
        return s_bReportTimeout;
    } else if (lPeriod == 1000) {
        if (s_wShortTimeoutBudget > 0U) {
            s_wShortTimeoutBudget--;
            return true;
        }
    } else {
        return false;
    }
    return false;
}

/**
 * @brief Capture formatted MODUS output for assertions.
 * @param pchFormat Format string.
 * @param ... Format arguments.
 * @return None.
 */
void util_debug_Printf(const char *pchFormat, ...)
{
    va_list tArgs;
    int nLength = 0;

    va_start(tArgs, pchFormat);
    nLength = vsnprintf(s_chLog, sizeof(s_chLog), pchFormat, tArgs);
    va_end(tArgs);
    assert(nLength >= 0);
}

/**
 * @brief Return a representative cached mechanical position.
 * @param ptEncoder Encoder object.
 * @param wNowTick Current tick.
 * @param ptPosition Output position.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_encoder_GetPosition(const void *pEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    s_wPositionCallCount++;
    s_ptReadEncoder = (const foc_encoder_t *)pEncoder;
    s_wReadTick = wNowTick;
    if (s_ePositionResult != FOC_RESULT_OK) {
        return s_ePositionResult;
    }
    ptPosition->tMechanicalAngle.wBam32 = 0x40000000U;
    ptPosition->qMechanicalSpeed = FOC_SCALAR(0.5f);
    ptPosition->bValid = true;
    return s_ePositionResult;
}

/**
 * @brief Stub an unused Motor control command.
 * @param ptMotor Motor object.
 * @param eMode Requested mode.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode)
{
    (void)ptMotor;
    (void)eMode;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub Motor stop for linking the command handler.
 * @param ptMotor Motor object.
 * @return None.
 */
void motor_Stop(motor_t *ptMotor)
{
    (void)ptMotor;
}

/**
 * @brief Stub an unused Motor fault command.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_ClearFault(motor_t *ptMotor)
{
    (void)ptMotor;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub an unused voltage reference command.
 * @param ptMotor Motor object.
 * @param qD D-axis voltage.
 * @param qQ Q-axis voltage.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    (void)ptMotor;
    (void)qD;
    (void)qQ;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub an unused current reference command.
 * @param ptMotor Motor object.
 * @param qD D-axis current.
 * @param qQ Q-axis current.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    (void)ptMotor;
    (void)qD;
    (void)qQ;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub an unused speed reference command.
 * @param ptMotor Motor object.
 * @param qSpeedReference Electrical speed reference.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qSpeedReference)
{
    (void)ptMotor;
    (void)qSpeedReference;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub an unused alignment command.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor)
{
    (void)ptMotor;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub the Motor status snapshot.
 * @param ptMotor Motor object.
 * @param ptStatus Output status.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    (void)ptMotor;
    *ptStatus = (motor_status_t){0};
    return FOC_RESULT_OK;
}

/**
 * @brief Verify the encoder command prints cached mechanical feedback.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    foc_app_CmdMotor("encoder");

    assert(s_wPositionCallCount == 1U);
    assert(s_ptReadEncoder == &tFocApp.tEncoder);
    assert(s_wReadTick == 1234U);
    assert(strstr(s_chLog, "mech=90.00 deg") != NULL);
    assert(strstr(s_chLog, "mech_speed=0.500 turn/s") != NULL);

    s_ePositionResult = FOC_RESULT_SAFETY;
    s_chLog[0] = '\0';
    foc_app_CmdMotor("encoder");

    assert(s_wPositionCallCount == 2U);
    assert(strstr(s_chLog, "encoder data unavailable") != NULL);
    assert(strstr(s_chLog, "mech=") == NULL);

    tFocApp.bReady = true;
    s_lSystemTick = 1000;
    s_lTickStep = 100;
    s_chLog[0] = '\0';
    foc_app_HighFrequencyISR();
    s_lTickStep = 300;
    foc_app_HighFrequencyISR();
    assert(s_wMotorStepCount == 2U);
    assert(s_chLog[0] == '\0');

    tFocApp.bReady = false;
    s_bReportTimeout = true;
    s_wShortTimeoutBudget = 1U;
    s_chLog[0] = '\0';
    (void)foc_app_Run((uintptr_t)&tFocApp);
    assert(strstr(s_chLog, "FOC HF ISR avg=200 cycles/2 us") != NULL);

    s_bReportTimeout = true;
    s_wShortTimeoutBudget = 1U;
    s_chLog[0] = '\0';
    (void)foc_app_Run((uintptr_t)&tFocApp);
    assert(s_chLog[0] == '\0');

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
    {
        foc_app_cfg_t tConfig = {0};
        tConfig.tMotorCfg.tParams.chPolePairs = 7U;
        tConfig.tMotorCfg.tParams.wResistanceMilliohm = 500U;
        tConfig.tMotorCfg.tParams.wInductanceDMicroHenry = 1000U;
        tConfig.tMotorCfg.tParams.wInductanceQMicroHenry = 1000U;
        tConfig.tMotorCfg.tLimits.qMaxSpeedReference =
            FOC_SCALAR(100.0f);
        tConfig.tMotorCfg.tLimits.qMaxPhaseCurrent = FOC_SCALAR(1.0f);
        tConfig.tMotorCfg.tLimits.qMaxModulation =
            FOC_SCALAR(0.5773502692f);
        tConfig.wVoltageBaseMillivolt = 12000U;
        tConfig.wCurrentBaseMilliamp = 7000U;
        tConfig.wHighFrequencyPeriodNanoseconds = 50000U;
        tConfig.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
        tConfig.ptAdc = &g_tFocAdcInterface;
        tConfig.ptPwm = &g_tFocPwmInterface;
        assert(foc_gain_from_float(0.2f,
            &tConfig.tMotorCfg.tSpeedPiParams.tKp) ==
               FOC_RESULT_OK);
        assert(foc_gain_from_float(0.005f,
            &tConfig.tMotorCfg.tSpeedPiParams.tKiTs) ==
               FOC_RESULT_OK);
        assert(foc_gain_from_float(0.0f,
            &tConfig.tMotorCfg.tSpeedPiParams.tKdOverTs) ==
               FOC_RESULT_OK);
        int nInitResult = foc_app_Init((uintptr_t)&tFocApp,
                                       (uintptr_t)&tConfig);

        assert(nInitResult == MODUS_SUCCESS);
        assert(s_tCapturedMotorConfig.tParams.wVoltageBaseMillivolt ==
               12000U);
        assert(s_tCapturedMotorConfig.tParams.wCurrentBaseMilliamp ==
               7000U);
        assert(s_tCapturedMotorConfig.qElectricalSpeedBaseTurnsPerSecond ==
               FOC_SCALAR(100.0f));
        assert(s_tCapturedMotorConfig.tLimits.qMaxSpeedReference == FOC_ONE);
        assert(fabsf(foc_to_float(foc_gain_apply(
                   &s_tCapturedMotorConfig.tSpeedPiParams.tKp,
                   FOC_SCALAR(0.1f))) - 2.0f) < 0.001f);
        assert(fabsf(foc_to_float(foc_gain_apply(
                   &s_tCapturedMotorConfig.tSpeedPiParams.tKiTs,
                   FOC_SCALAR(0.1f))) - 0.05f) < 0.001f);
        assert(s_wWaveInitCount == 1U);
        assert(s_chWaveCount == 6U);
        assert(strcmp(s_achWaveNames[0], "Sine500") == 0);
        assert(strcmp(s_achWaveNames[1], "WaveSeq") == 0);
        assert(strcmp(s_achWaveNames[2], "SpeedPU") == 0);
        assert(strcmp(s_achWaveNames[3], "SpeedRefPU") == 0);
        assert(strcmp(s_achWaveNames[4], "Iq") == 0);
        assert(strcmp(s_achWaveNames[5], "IqRef") == 0);
        assert(s_afWaveScales[0] == 1000.0f);
        assert(s_afWaveScales[1] == 1.0f);
        assert(s_afWaveScales[2] == 100.0f);
        assert(s_afWaveScales[3] == 100.0f);
        assert(s_afWaveScales[4] == 1000.0f);
        assert(s_afWaveScales[5] == 1000.0f);
        assert(s_achWaveTypes[0] == MWAVEFORM_VAR_FLOAT);
        assert(s_achWaveTypes[1] == MWAVEFORM_VAR_RAW);
        assert(s_achWaveTypes[2] == MWAVEFORM_VAR_FLOAT);
        assert(s_achWaveTypes[3] == MWAVEFORM_VAR_FLOAT);
        assert(s_achWaveTypes[4] == MWAVEFORM_VAR_FLOAT);
        assert(s_achWaveTypes[5] == MWAVEFORM_VAR_FLOAT);
        assert(s_apvWaveValues[0] != NULL);
        assert(s_apvWaveValues[1] != NULL);
        assert(s_apvWaveValues[2] ==
               &tFocApp.tMotor.tInput.qElectricalSpeedPu);
        assert(s_apvWaveValues[3] ==
               &tFocApp.tMotor.tCommand.qSpeedReferencePu);
        assert(s_apvWaveValues[4] ==
               &tFocApp.tMotor.tCore.tCurrent.qQ);
        assert(s_apvWaveValues[5] ==
               &tFocApp.tMotor.tCommand.tCurrentReference.qQ);
        assert(s_wWaveDecimation == 0U);
        assert(s_wWaveIsrPeriodNs == 50000U);
        assert(s_wWaveTargetHz == 10000U);
        assert(s_wWaveChannelRateCount == 2U);
        assert(s_awWaveChannelRates[3] == 1000U);
        assert(s_awWaveChannelRates[5] == 1000U);
        assert(s_wWaveStartCount == 1U);

        s_wWaveStepCount = 0U;
        {
            float fMinimum = 2.0f;
            float fMaximum = -2.0f;
            float *pfSine = (float *)s_apvWaveValues[0];
            int16_t *phwSequence = (int16_t *)s_apvWaveValues[1];
            uint32_t wIndex = 0U;

            for (wIndex = 0U; wIndex < 40U; wIndex++) {
                foc_app_HighFrequencyISR();
                if (*pfSine < fMinimum) {
                    fMinimum = *pfSine;
                }
                if (*pfSine > fMaximum) {
                    fMaximum = *pfSine;
                }
            }
            assert(fMinimum == -1.0f);
            assert(fMaximum == 1.0f);
            assert(*pfSine == 0.0f);
            assert(*phwSequence == 40);
        }
        assert(s_wWaveStepCount == 40U);
    }
#endif
    return 0;
}
