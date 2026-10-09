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
#include "../motor/motor.h"
#include "../observer/foc_encoder.h"

const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface = {0};

#include "../app/foc_app.c"
#include "../app/foc_debug.c"

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
static char s_achWaveNames[8][16] = {{0}};
static uint8_t s_achWaveTypes[8] = {0U};
static float s_afWaveScales[8] = {0.0f};
static void *s_apvWaveValues[8] = {NULL};
static uint32_t s_awWaveChannelRates[8] = {0U};

static float test_WaveformValue(const char *pchName)
{
    uint8_t chIndex = 0U;

    for (chIndex = 0U; chIndex < s_chWaveCount; chIndex++) {
        if (strcmp(s_achWaveNames[chIndex], pchName) == 0) {
            return *(float *)s_apvWaveValues[chIndex];
        }
    }
    assert(false);
    return 0.0f;
}

/**
 * @brief Record waveform API initialization.
 * @param ptProtocol Waveform protocol requested by the app.
 * @return MODUS_SUCCESS.
 */
static int test_WaveInit(const mwaveform_protocol_t *ptProtocol)
{
    (void)ptProtocol;
    s_wWaveInitCount++;
    s_chWaveCount = 0U;
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

    if (chIndex >= 8U) {
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
    if (chID >= s_chWaveCount || chID >= 8U) {
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
static char s_chLog[512] = {0};
static int64_t s_lSystemTick = 1234;
static int64_t s_lTickStep = 0;
static uint32_t s_wShortTimeoutBudget = 0U;
static bool s_bReportTimeout = false;
static uint32_t s_wMotorStepCount = 0U;
static uint32_t s_wEncoderRunCount = 0U;
static uint32_t s_wControlWithoutFeedbackCount = 0U;
static motor_isr_phase_t s_eMotorPhase = MOTOR_ISR_NO_CONTROL;
static foc_result_t s_eControlResult = FOC_RESULT_OK;
static uint32_t s_wEncoderInitCount = 0U;
static motor_cfg_t s_tCapturedMotorConfig = {0};
static motor_status_t s_tMotorStatus = {0};
static uint32_t s_wMotorStartCount = 0U;
static uint32_t s_wCurrentReferenceCount = 0U;
static uint32_t s_wMotorStopCount = 0U;
static foc_scalar_t s_qCurrentReferenceD = FOC_ZERO;
static foc_scalar_t s_qCurrentReferenceQ = FOC_ZERO;

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
    s_wEncoderInitCount++;
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
    s_wEncoderRunCount++;
    return s_ePositionResult;
}

/**
 * @brief Stub Motor initialization for the unused App initialization path.
 * @param ptMotor Motor object.
 * @param ptConfig Motor configuration.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig)
{
    *ptMotor = (motor_t){0};
    ptMotor->tParams = ptConfig->tParams;
    ptMotor->tParams.wCurrentBaseMilliamp = FOC_CURRENT_BASE_MILLIAMP;
    s_tCapturedMotorConfig = *ptConfig;
    return FOC_RESULT_OK;
}

/** @brief Stub one valid normalized DC-bus sample for App orchestration.
 * @param ptMotor Motor whose voltage base is checked.
 * @param pqBusPu Output bus voltage in pu.
 * @return FOC_RESULT_OK or a null/base error. */
foc_result_t motor_SampleDcBusPuIsr(const motor_t *ptMotor,
                                   foc_scalar_t *pqBusPu)
{
    if (ptMotor == NULL || pqBusPu == NULL) {
        return FOC_RESULT_NULL;
    }
    *pqBusPu = FOC_ZERO;
    if (ptMotor->tParams.wVoltageBaseMillivolt == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *pqBusPu = FOC_ONE;
    return FOC_RESULT_OK;
}

/**
 * @brief Stub the Motor high-frequency path.
 * @param ptMotor Motor object.
 * @param ptSample Prepared Motor sample.
 * @return None.
 */
motor_isr_phase_t motor_IsrPrepare(motor_t *ptMotor,
                                   motor_position_sample_t *ptSample)
{
    (void)ptMotor;
    ptSample->tCurrentAlphaBeta = (foc_ab_t){FOC_SCALAR(0.03f), FOC_ZERO};
    ptSample->tVoltageModelAlphaBeta.qAlpha = FOC_SCALAR(0.1f);
    ptSample->bVoltageModelValid = true;
    ptSample->bAlignActive = ptMotor->eState == MOTOR_STATE_ALIGN;
    s_wMotorStepCount++;
    return s_eMotorPhase;
}

foc_result_t motor_IsrControlStep(
    motor_t *ptMotor, const motor_electrical_feedback_t *ptFeedback)
{
    (void)ptMotor;
    if (ptFeedback == NULL) {
        s_wControlWithoutFeedbackCount++;
    }
    return s_eControlResult;
}

void motor_ApplyPositionEventIsr(motor_t *ptMotor,
    motor_position_event_t eEvent, foc_scalar_t qObservedSpeedPu)
{
    (void)ptMotor;
    (void)eEvent;
    (void)qObservedSpeedPu;
}

void motor_CompleteAlignIsr(motor_t *ptMotor, foc_result_t eCapture)
{
    (void)ptMotor;
    (void)eCapture;
}

static foc_result_t s_eIdentifyApplyResult = FOC_RESULT_OK;
static uint32_t s_wIdentifyAbortCount = 0U;
static foc_ab_t s_tIdentifyVoltageCommand = {0};

foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor, const foc_ab_t *ptVoltageCommand)
{
    (void)ptMotor;
    s_tIdentifyVoltageCommand = *ptVoltageCommand;
    return s_eIdentifyApplyResult;
}

void motor_IdentificationAbortIsr(motor_t *ptMotor,
                                  motor_fault_e eFault)
{
    (void)ptMotor;
    (void)eFault;
    s_wIdentifyAbortCount++;
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
    size_t wCurrent = strlen(s_chLog);
    if (wCurrent < sizeof(s_chLog) - 1U) {
        nLength = vsnprintf(s_chLog + wCurrent,
                            sizeof(s_chLog) - wCurrent,
                            pchFormat, tArgs);
    }
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
    s_wMotorStartCount++;
    s_tMotorStatus.eState = MOTOR_STATE_RUNNING;
    s_tMotorStatus.eMode = eMode;
    s_tMotorStatus.bPwmEnabled = true;
    return FOC_RESULT_OK;
}

foc_result_t motor_StartIdentification(motor_t *ptMotor)
{
    foc_result_t result = motor_Start(ptMotor, FOC_MODE_VOLTAGE);
    s_tMotorStatus.bIdentificationActive = true;
    return result;
}

foc_result_t motor_SetIdentificationModulation(motor_t *ptMotor,
    const foc_ab_t *ptCommand)
{
    return motor_SetVoltageReference(ptMotor,
        ptCommand->qAlpha, ptCommand->qBeta);
}

foc_result_t motor_StartSensorlessSpeed(motor_t *ptMotor,
                                        foc_scalar_t qTargetSpeedPu)
{
    (void)ptMotor;
    (void)qTargetSpeedPu;
    return FOC_RESULT_DISABLED;
}

/**
 * @brief Expose sensorless capability for the command-only fake Motor.
 * @param ptMotor Motor object queried by the shell command.
 * @return True for a supplied fake Motor.
 */
bool motor_SensorlessStartConfigured(const motor_t *ptMotor)
{
    return ptMotor != NULL;
}

/**
 * @brief Stub Motor stop for linking the command handler.
 * @param ptMotor Motor object.
 * @return None.
 */
void motor_Stop(motor_t *ptMotor)
{
    (void)ptMotor;
    s_wMotorStopCount++;
    s_tMotorStatus.eState = MOTOR_STATE_IDLE;
    s_tMotorStatus.bPwmEnabled = false;
    s_tMotorStatus.bIdentificationActive = false;
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
    s_wCurrentReferenceCount++;
    s_qCurrentReferenceD = qD;
    s_qCurrentReferenceQ = qQ;
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
    *ptStatus = s_tMotorStatus;
    return FOC_RESULT_OK;
}

/**
 * @brief Verify the encoder command prints cached mechanical feedback.
 * @param None.
 * @return Zero on success.
 */
/** @brief App submits capture requests and stops rejected/invalid steps.
 * @return None. */
static void test_IdentifyBoundary(void)
{
    for (uint32_t scenario = 0U; scenario < 4U; scenario++) {
        foc_app_t app = {0};
        identify_cfg_t model = {
            .wVoltageBaseMillivolt = 12000U,
            .wCurrentBaseMilliamp = 3500U,
            .wResistanceMilliohm = 1000U,
            .qElectricalSpeedBaseHz = FOC_SCALAR(350.0f),
        };
        identify_inductance_cfg_t cfg = {
            .wInjectionFrequencyHz = 1000U,
            .hwCaptureDelayCycles = 2U,
            .hwCaptureSampleCount = 2U,
            .hwHalfCycleCount = 4U,
            .qModulationAmplitude = FOC_SCALAR(0.1f),
            .qMaxIdentificationCurrent = FOC_SCALAR(0.5f),
            .qMinCurrentDelta = FOC_SCALAR(0.01f),
            .qMaxTransverseCurrentPu = FOC_SCALAR(0.01f),
            .hwTransverseFaultCycles = 1U,
        };
        motor_position_sample_t sample = {
            .tVoltageModelAlphaBeta = {FOC_SCALAR(0.1f), FOC_ZERO},
            .bVoltageModelValid = true,
        };
        identify_result_t tIdentifyResult = {0};
        uint32_t abort_before = s_wIdentifyAbortCount;

        s_tMotorStatus = (motor_status_t){
            .eState = MOTOR_STATE_IDLE, .bElectricalZeroValid = false};
        app.tMotor.tParams.wVoltageBaseMillivolt = 12000U;
        app.tMotor.tLimits.qMaxModulation = FOC_ONE;
        app.tMotor.tLimits.qMaxPhaseCurrent = FOC_ONE;
        assert(identify_Init(&app.tIdentify, &model) == FOC_RESULT_OK);
        assert(identify_StartInductance(&app.tIdentify, &cfg) ==
               FOC_RESULT_OK);
        assert(identify_Run(&app.tIdentify, &app.tMotor, &tIdentifyResult) ==
               FOC_RESULT_BUSY);
        assert(tIdentifyResult.eOperation == IDENTIFY_OPERATION_NONE);
        s_eIdentifyApplyResult = scenario == 1U ?
            FOC_RESULT_SAFETY : FOC_RESULT_OK;
        if (scenario == 2U) {
            sample.bVoltageModelValid = false;
        }
        _foc_app_IdentifyStep(&app, &sample,
            scenario == 3U ? FOC_RESULT_SAFETY : FOC_RESULT_OK);
        if (scenario == 0U) {
            assert(s_wIdentifyAbortCount == abort_before);
            assert(s_tIdentifyVoltageCommand.qAlpha == cfg.qModulationAmplitude);
            _foc_app_IdentifyStep(&app, &sample, FOC_RESULT_OK);
            assert(app.tIdentify.tInductance.tCapture.u_window_sum == 0);
            _foc_app_IdentifyStep(&app, &sample, FOC_RESULT_OK);
            assert(fabsf(foc_to_float((foc_scalar_t)
                app.tIdentify.tInductance.tCapture.u_window_sum) -
                0.1f) < 0.0002f);
        } else {
            assert(s_wIdentifyAbortCount == abort_before + 1U);
            assert(app.tIdentify.eState == IDENTIFY_STATE_ERROR);
            assert(app.tIdentify.eLastResult == FOC_RESULT_SAFETY);
            _foc_app_IdentifyStep(&app, &sample, FOC_RESULT_OK);
            assert(s_wIdentifyAbortCount == abort_before + 1U);
        }
    }
    s_eIdentifyApplyResult = FOC_RESULT_OK;
    s_tMotorStatus = (motor_status_t){0};
}

/** @brief Encoder failure and invalid zero cannot block stationary R/L.
 * @return None. */
static void test_EncoderlessIdentification(void)
{
    identify_cfg_t cfg = {
        .wVoltageBaseMillivolt = 12000U,
        .wCurrentBaseMilliamp = 3500U,
        .wResistanceMilliohm = 1000U,
        .qElectricalSpeedBaseHz = FOC_SCALAR(350.0f),
    };
    tFocApp = (foc_app_t){0};
    tFocApp.bReady = true;
    tFocApp.bEncoderEnabled = true;
    tFocApp.lForegroundPeriodTicks = 1000;
    tFocApp.tMotor.tParams.wVoltageBaseMillivolt = 12000U;
    tFocApp.tPosition.eSource = MOTOR_POSITION_SOURCE_SENSOR;
    tFocApp.tMotor.tOutput.tDuty = (foc_duty_abc_t){
        FOC_SCALAR(0.575f), FOC_SCALAR(0.425f), FOC_SCALAR(0.425f)};
    s_tMotorStatus = (motor_status_t){.eState = MOTOR_STATE_IDLE};
    s_ePositionResult = FOC_RESULT_SAFETY;
    assert(identify_Init(&tFocApp.tIdentify, &cfg) == FOC_RESULT_OK);
    foc_debug_CmdIdentify("resistance");
    assert(tFocApp.tIdentify.eOperation == IDENTIFY_OPERATION_RESISTANCE);
    uint32_t encoder_runs = s_wEncoderRunCount;
    uint32_t positions = s_wPositionCallCount;
    uint32_t controls = s_wControlWithoutFeedbackCount;
    s_wShortTimeoutBudget = 1U;
    assert(foc_app_Run((uintptr_t)&tFocApp) == MODUS_EFAIL);
    assert(s_wEncoderRunCount == encoder_runs + 1U);
    assert(tFocApp.tIdentify.tResistance.bMotorStarted);
    s_eMotorPhase = MOTOR_ISR_IDENTIFICATION_READY;
    foc_app_HighFrequencyISR();
    assert(s_wPositionCallCount == positions);
    assert(s_wControlWithoutFeedbackCount == controls + 1U);
    assert(tFocApp.tIdentify.eState == IDENTIFY_STATE_RUNNING);
    assert(tFocApp.tIdentify.tResistance.qLatestCurrentAlphaPu ==
           FOC_SCALAR(0.03f));
    identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
    foc_debug_CmdIdentify("inductance");
    assert(tFocApp.tIdentify.eOperation == IDENTIFY_OPERATION_INDUCTANCE);
    identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
    s_eMotorPhase = MOTOR_ISR_NO_CONTROL;
    s_ePositionResult = FOC_RESULT_OK;
}

int main(void)
{
    test_IdentifyBoundary();
    foc_debug_CmdMotor("encoder");

    assert(s_wPositionCallCount == 1U);
    assert(s_ptReadEncoder == &tFocApp.tEncoder);
    assert(s_wReadTick == 1234U);
    assert(strstr(s_chLog, "mech=90.00 deg") != NULL);
    assert(strstr(s_chLog, "mech_speed=0.500 turn/s") != NULL);

    s_ePositionResult = FOC_RESULT_SAFETY;
    s_chLog[0] = '\0';
    foc_debug_CmdMotor("encoder");

    assert(s_wPositionCallCount == 2U);
    assert(strstr(s_chLog, "encoder data unavailable") != NULL);
    assert(strstr(s_chLog, "mech=") == NULL);

    s_tMotorStatus = (motor_status_t){0};
    s_tMotorStatus.eState = MOTOR_STATE_IDLE;
    s_wMotorStartCount = 0U;
    s_wCurrentReferenceCount = 0U;
    foc_debug_CmdMotor("current 0 0");
    assert(s_wMotorStartCount == 1U);
    assert(s_wCurrentReferenceCount == 1U);
    assert(s_tMotorStatus.eMode == FOC_MODE_CURRENT);
    foc_debug_CmdMotor("current 0 0.05");
    assert(s_wMotorStartCount == 1U);
    assert(s_wCurrentReferenceCount == 2U);
    assert(fabsf(foc_to_float(s_qCurrentReferenceQ) - 0.05f) < 0.0001f);

    s_tMotorStatus = (motor_status_t){0};
    s_tMotorStatus.eState = MOTOR_STATE_RUNNING;
    s_tMotorStatus.eMode = FOC_MODE_CURRENT;
    s_tMotorStatus.bPwmEnabled = true;
    s_wMotorStartCount = 0U;
    s_wCurrentReferenceCount = 0U;
    s_wMotorStopCount = 0U;
    tFocApp.eCurrentStepState = FOC_APP_CURRENT_STEP_IDLE;
    tFocApp.bEncoderEnabled = true;
    tFocApp.tMotor.tInput.bAngleValid = true;
    s_lSystemTick = 10000;
    s_lTickStep = 0;
    s_tMotorStatus.bElectricalZeroValid = true;
    foc_debug_CmdMotor("step 0.05 100");
    assert(s_wMotorStartCount == 0U);
    assert(s_wCurrentReferenceCount == 1U);
    assert(fabsf(foc_to_float(s_qCurrentReferenceQ) - 0.05f) < 0.0001f);
    assert(tFocApp.eCurrentStepState == FOC_APP_CURRENT_STEP_PULSE);
    s_lSystemTick = 109999;
    foc_debug_CurrentStepRun(&tFocApp);
    assert(s_wMotorStopCount == 0U);
    s_lSystemTick = 110000;
    foc_debug_CurrentStepRun(&tFocApp);
    assert(s_wCurrentReferenceCount == 2U);
    assert(fabsf(foc_to_float(s_qCurrentReferenceQ)) < 0.0001f);
    assert(tFocApp.eCurrentStepState ==
           FOC_APP_CURRENT_STEP_ZERO_TAIL);
    assert(s_wMotorStopCount == 0U);
    s_lSystemTick = 114999;
    foc_debug_CurrentStepRun(&tFocApp);
    assert(s_wMotorStopCount == 0U);
    s_lSystemTick = 115000;
    foc_debug_CurrentStepRun(&tFocApp);
    assert(s_wMotorStopCount == 1U);
    assert(s_tMotorStatus.eState == MOTOR_STATE_IDLE);
    assert(tFocApp.eCurrentStepState == FOC_APP_CURRENT_STEP_IDLE);

    s_tMotorStatus.eMode = FOC_MODE_SPEED;
    s_tMotorStatus.eState = MOTOR_STATE_RUNNING;
    s_tMotorStatus.bPwmEnabled = true;
    foc_debug_CmdMotor("current 0 0.02");
    assert(s_wMotorStartCount == 0U);
    assert(s_wCurrentReferenceCount == 2U);

    tFocApp.bReady = true;
    s_lSystemTick = 1000;
    s_lTickStep = 100;
    s_chLog[0] = '\0';
    foc_app_HighFrequencyISR();
    s_lTickStep = 300;
    foc_app_HighFrequencyISR();
    assert(s_wMotorStepCount == 2U);
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
        tConfig.wHighFrequencyIsrHz = 20000U;
        tConfig.qElectricalSpeedBaseHz = FOC_SCALAR(100.0f);
        tConfig.tObserverCfg.tSmo.wBemfCutoffRadiansPerSecond = 10000U;
        tConfig.tObserverCfg.tSmo.wSlidingGainMillivolt = 3500U;
        tConfig.tObserverCfg.tSmo.qCurrentEstimateLimit = FOC_ONE;
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
        assert(s_tCapturedMotorConfig.tParams.wCurrentBaseMilliamp == 0U);
        assert(s_tCapturedMotorConfig.qElectricalSpeedBaseHz ==
               FOC_SCALAR(100.0f));
        assert(s_tCapturedMotorConfig.tLimits.qMaxSpeedReference == FOC_ONE);
        assert(fabsf(foc_to_float(foc_gain_apply(
                   &s_tCapturedMotorConfig.tSpeedPiParams.tKp,
                   FOC_SCALAR(0.1f))) - 2.0f) < 0.001f);
        assert(fabsf(foc_to_float(foc_gain_apply(
                   &s_tCapturedMotorConfig.tSpeedPiParams.tKiTs,
                   FOC_SCALAR(0.1f))) - 0.05f) < 0.001f);
        assert(s_wWaveInitCount == 1U);
        const char *const achExpectedNames[] = {
#if FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE
            "Enc_mT",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE
            "SMO_mT",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR
            "Err_mT",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_IQ
            "Iq_mpu",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA
            "Ealpha_mpu",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA
            "Ebeta_mpu",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA
            "IerrA_mpu",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA
            "IerrB_mpu",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA
            "Zalpha_mpu",
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA
            "Zbeta_mpu",
#endif
        };
        assert(s_chWaveCount ==
               sizeof(achExpectedNames) / sizeof(achExpectedNames[0]));
        assert(s_chWaveCount <= 6U);
        for (uint32_t wIndex = 0U; wIndex < s_chWaveCount; wIndex++) {
            assert(strcmp(s_achWaveNames[wIndex],
                          achExpectedNames[wIndex]) == 0);
            assert(s_achWaveTypes[wIndex] == MWAVEFORM_VAR_FLOAT);
            assert(s_apvWaveValues[wIndex] != NULL);
            assert(s_afWaveScales[wIndex] == 1000.0f);
        }
        assert(s_wWaveDecimation == 0U);
        assert(s_wWaveIsrPeriodNs == 50000U);
        assert(s_wWaveTargetHz == 10000U);
        assert(s_wWaveChannelRateCount == 0U);

        tFocApp.tPosition.eSource = MOTOR_POSITION_SOURCE_SENSOR;
        tFocApp.tMotor.tInput.tCurrentAlphaBeta =
            (foc_ab_t){FOC_SCALAR(0.1f), FOC_SCALAR(0.2f)};
        tFocApp.tMotor.tOutput.tVoltageAlphaBeta =
            (foc_ab_t){FOC_SCALAR(0.3f), FOC_SCALAR(-0.4f)};
        tFocApp.tMotor.tCommand.tCurrentReference =
            (foc_dq_t){FOC_SCALAR(0.1f), FOC_SCALAR(0.2f)};
        tFocApp.tMotor.tOutput.tCurrent =
            (foc_dq_t){FOC_SCALAR(0.3f), FOC_SCALAR(0.4f)};
        tFocApp.tMotor.tOutput.tVoltage =
            (foc_dq_t){FOC_SCALAR(0.5f), FOC_SCALAR(0.6f)};
        tFocApp.tMotor.tInput.tElectricalAngle.wBam32 = 0x20000000U;
        tFocApp.tMotor.tInput.bAngleValid = true;
        tFocApp.tPosition.tObserver.tSmo.tAxis[0].e_hat =
            FOC_SCALAR(0.5f);
        tFocApp.tPosition.tObserver.tSmo.tAxis[1].e_hat =
            FOC_SCALAR(0.6f);
        tFocApp.tPosition.tObserver.tSmo.tAxis[0].i_hat =
            FOC_SCALAR(0.04f);
        tFocApp.tPosition.tObserver.tSmo.tAxis[1].i_hat =
            FOC_SCALAR(0.05f);
        tFocApp.tPosition.tObserver.tSmo.tAxis[0].z_prev =
            FOC_SCALAR(0.7f);
        tFocApp.tPosition.tObserver.tSmo.tAxis[1].z_prev =
            FOC_SCALAR(0.8f);
        tFocApp.tPosition.tEstimates.tSmo.tEstimate.tElectricalAngle.wBam32 =
            0x60000000U;
        tFocApp.tPosition.tEstimates.tSmo.tEstimate.bValid = true;
        foc_debug_WaveformStep();
        assert(fabsf(test_WaveformValue("Enc_mT") - 0.125f) < 0.001f);
        assert(fabsf(test_WaveformValue("SMO_mT") - 0.375f) < 0.001f);
        assert(fabsf(test_WaveformValue("Err_mT") - 0.25f) < 0.001f);
        assert(fabsf(test_WaveformValue("Iq_mpu") - 0.4f) < 0.001f);
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA
        assert(fabsf(test_WaveformValue("Ealpha_mpu") - 0.5f) < 0.001f);
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA
        assert(fabsf(test_WaveformValue("Ebeta_mpu") - 0.6f) < 0.001f);
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA
        assert(fabsf(test_WaveformValue("IerrA_mpu") - 0.06f) < 0.001f);
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA
        assert(fabsf(test_WaveformValue("IerrB_mpu") - 0.15f) < 0.001f);
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA
        assert(fabsf(test_WaveformValue("Zalpha_mpu") - 0.7f) < 0.001f);
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA
        assert(fabsf(test_WaveformValue("Zbeta_mpu") - 0.8f) < 0.001f);
#endif
#if FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR
        tFocApp.tMotor.tInput.bAngleValid = false;
        foc_debug_WaveformStep();
        assert(test_WaveformValue("Err_mT") == -1.0f);
#if FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE
        assert(fabsf(test_WaveformValue("SMO_mT") - 0.375f) < 0.001f);
#endif
        tFocApp.tMotor.tInput.bAngleValid = true;
#endif
        s_wWaveStepCount = 0U;
        assert(s_wEncoderInitCount == 1U);
        tConfig.ePositionSource = MOTOR_POSITION_SOURCE_HARD_DRAG;
        tConfig.tMotorCfg.nHardDragElectricalMilliHz = 5000;
        assert(foc_app_Init((uintptr_t)&tFocApp,
                            (uintptr_t)&tConfig) == MODUS_SUCCESS);
        assert(tFocApp.bReady);
        assert(tFocApp.tPosition.eSource ==
               MOTOR_POSITION_SOURCE_HARD_DRAG);
        assert(s_wEncoderInitCount == 1U);
        assert(s_tCapturedMotorConfig.wControlFrequencyHz == 20000U);
        assert(s_wWaveStartCount == 2U);

        s_wWaveStepCount = 0U;
        for (uint32_t wIndex = 0U; wIndex < 40U; wIndex++) {
            foc_app_HighFrequencyISR();
        }
        assert(s_wWaveStepCount == 40U);

        tConfig.tMotorCfg.nHardDragElectricalMilliHz = 0;
        tConfig.tMotorCfg.wStartupRampSteps = 20U;
        tConfig.tMotorCfg.wStartupMaxRunSteps = 120U;
        tConfig.tMotorCfg.qStartupIqPu = FOC_SCALAR(0.02f);
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
        tConfig.bObserverTakeover = true;
        tConfig.bObserverAutoTakeover = false;
        tConfig.wObserverQualificationSteps = 3U;
        tConfig.wObserverBlendSteps = 4U;
        tConfig.qObserverMinBemfPu = FOC_SCALAR(0.13f);
        tConfig.qObserverMinSpeedPu = FOC_SCALAR(0.1f);
        tConfig.qObserverMaxSpeedErrorRatio = FOC_SCALAR(0.1f);
        tConfig.qObserverMaxAngleErrorTurns = FOC_SCALAR(0.125f);
#endif
        assert(foc_app_Init((uintptr_t)&tFocApp,
                            (uintptr_t)&tConfig) == MODUS_SUCCESS);
        assert(tFocApp.bReady);
        assert(!tFocApp.bEncoderEnabled);
        assert(s_wEncoderInitCount == 1U);
        assert(strcmp(s_achWaveNames[0], "Ctrl_mT") == 0);
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
        assert(tFocApp.tPosition.tHandoff.bObserverTakeover);
        assert(!tFocApp.tPosition.tHandoff.bAutoTakeover);
#endif
    }
#endif
    test_EncoderlessIdentification();
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    /* A failed control/PWM submission must not retain a qualified estimate. */
    tFocApp = (foc_app_t){0};
    tFocApp.bReady = true;
    tFocApp.tPosition.eSource = MOTOR_POSITION_SOURCE_SENSOR;
    tFocApp.tPosition.tSensor = (motor_position_provider_t){
        .fnGetPosition = foc_encoder_GetPosition,
        .pContext = &tFocApp.tEncoder};
    tFocApp.tPosition.chPolePairs = 7U;
    tFocApp.tPosition.qMechanicalToElectricalSpeedPuGain = FOC_ONE;
    tFocApp.tPosition.tEstimates.tSmo.tEstimate.bValid = true;
    tFocApp.tPosition.tSmo.bObserverSpeedReady = true;
    s_ePositionResult = FOC_RESULT_OK;
    s_eMotorPhase = MOTOR_ISR_CONTROL_READY;
    s_eControlResult = FOC_RESULT_SAFETY;
    foc_app_HighFrequencyISR();
    assert(!tFocApp.tPosition.tEstimates.tSmo.tEstimate.bValid);
    assert(!tFocApp.tPosition.tSmo.bObserverSpeedReady);
#endif
    return 0;
}
