/****************************************************************************
 * @file    foc_app_identify_test.c
 * @brief   End-to-end integration test for parameter identification.
 * @author  Antigravity
 * @date    2026-09-17
 ****************************************************************************/

#include "foc_config.h"

#if !FOC_ENABLE_EXPERIMENTAL_IDENTIFY
#error "Identify integration tests require identify enabled"
#endif

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../hal/foc_port.h"
#include "../observer/foc_encoder.h"
#include "foc_identify_plant.h"

const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface = {0};
const motor_position_ops_t g_tFocEncoderPositionOps = {0};

#include "../app/foc_app.c"

int mbase_Init(modus_base_t *ptBase, modus_base_cfg_t *ptConfig)
{
    (void)ptBase;
    (void)ptConfig;
    return MODUS_SUCCESS;
}

int mshell_RegisterCmd(uintptr_t wAddr, uintptr_t wUnused)
{
    (void)wAddr;
    (void)wUnused;
    return 0;
}

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
static int test_WaveInit(const mwaveform_protocol_t *ptProtocol)
{
    (void)ptProtocol;
    return MODUS_SUCCESS;
}

static uint8_t test_WaveAddVariable(const char *pchName, float fScale,
                                    void *pvValue, uint8_t chType)
{
    (void)pchName;
    (void)fScale;
    (void)pvValue;
    (void)chType;
    return 0U;
}

static uint32_t test_WaveSetChannelRate(uint8_t chID, uint32_t wHz)
{
    (void)chID;
    return wHz;
}

static void test_WaveStart(void)
{
}

static void test_WaveStep(void)
{
}

static void test_WaveSetRate(uint32_t wDecimation)
{
    (void)wDecimation;
}

static uint32_t test_WaveSetStreamRate(uint32_t wIsrPeriodNs,
                                       uint32_t wTargetHz)
{
    (void)wIsrPeriodNs;
    return wTargetHz;
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

uint8_t g_chGLogMask = MLOG_MASK_ALL;
volatile int32_t g_nOffset = 0;

static char s_achLogBuffer[512] = {0};

void util_debug_Printf(const char *pchFormat, ...)
{
    va_list tArgs;
    va_start(tArgs, pchFormat);
    (void)vsnprintf(s_achLogBuffer, sizeof(s_achLogBuffer), pchFormat, tArgs);
    va_end(tArgs);
}

static int64_t s_lSystemTick = 0;

int64_t get_system_ticks(void)
{
    return s_lSystemTick;
}

int64_t perfc_convert_us_to_ticks(uint32_t wMicroseconds)
{
    return (int64_t)wMicroseconds;
}

int64_t perfc_convert_ms_to_ticks(uint32_t wMilliseconds)
{
    return (int64_t)wMilliseconds * 1000;
}

int64_t perfc_convert_ticks_to_us(int64_t lTicks)
{
    return lTicks;
}

bool __perfc_is_time_out(int64_t lPeriod,
                         int64_t *plTimestamp,
                         bool bAutoReload)
{
    int64_t lNow = get_system_ticks();
    if (lNow - *plTimestamp >= lPeriod) {
        if (bAutoReload) {
            *plTimestamp = lNow;
        }
        return true;
    }
    return false;
}

static foc_angle_t s_tSimMechanicalAngle = {0};
static foc_angle_t s_tSimElectricalAngle = {0};
static foc_scalar_t s_qSimMechanicalSpeed = FOC_ZERO;
static bool s_bEncoderValid = true;

foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_cfg_t *ptConfig)
{
    (void)ptEncoder;
    (void)ptConfig;
    return FOC_RESULT_OK;
}

foc_result_t foc_encoder_Run(foc_encoder_t *ptEncoder)
{
    (void)ptEncoder;
    return FOC_RESULT_OK;
}

foc_result_t foc_encoder_GetPosition(const void *pEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    (void)pEncoder;
    (void)wNowTick;
    if (!s_bEncoderValid) {
        return FOC_RESULT_DISABLED;
    }
    ptPosition->bValid = true;
    ptPosition->tMechanicalAngle = s_tSimMechanicalAngle;
    ptPosition->qMechanicalSpeed = s_qSimMechanicalSpeed;
    return FOC_RESULT_OK;
}

static foc_identify_plant_t s_tSimPlant = {0};
static foc_duty_abc_t s_tActiveDuty = {0};
static foc_duty_abc_t s_atDutyPipeline[4] = {{0}};
static uint32_t s_wPwmDelayTicks = 1U;
static bool s_bPwmEnabled = false;
static bool s_bPwmFault = false;
static double s_dSimVbus = 12.0;

void foc_port_StartAdcTrigger(void)
{
}

foc_result_t foc_PwmEnable(void)
{
    s_bPwmEnabled = true;
    return FOC_RESULT_OK;
}

foc_result_t foc_PwmSafeStop(void)
{
    s_bPwmEnabled = false;
    return FOC_RESULT_OK;
}

bool foc_PwmGetFault(void)
{
    return s_bPwmFault;
}

foc_result_t foc_PwmClearFault(void)
{
    s_bPwmFault = false;
    return FOC_RESULT_OK;
}

foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    if (s_wPwmDelayTicks == 0U) {
        s_tActiveDuty = *ptDuty;
    } else {
        s_atDutyPipeline[s_wPwmDelayTicks - 1U] = *ptDuty;
    }
    return FOC_RESULT_OK;
}

static void sim_step_plant(void)
{
    double dDu = (double)foc_to_float(s_tActiveDuty.qU);
    double dDv = (double)foc_to_float(s_tActiveDuty.qV);
    double dDw = (double)foc_to_float(s_tActiveDuty.qW);
    double dVd = 0.0;
    double dVq = 0.0;
    double dObsId = 0.0;
    double dObsIq = 0.0;

    if (s_bPwmEnabled) {
        double dMean = (dDu + dDv + dDw) / 3.0;
        double dVu = (dDu - dMean) * s_dSimVbus;
        double dVv = (dDv - dMean) * s_dSimVbus;
        double dVw = (dDw - dMean) * s_dSimVbus;
        double dValpha = dVu;
        double dVbeta = (dVv - dVw) / 1.7320508075688772;
        double dTheta = (double)foc_angle_to_turns(s_tSimElectricalAngle) *
                        2.0 * 3.141592653589793;
        double dCos = cos(dTheta);
        double dSin = sin(dTheta);

        dVd = dCos * dValpha + dSin * dVbeta;
        dVq = dCos * dVbeta - dSin * dValpha;
    }

    foc_identify_plant_Step(&s_tSimPlant, dVd, dVq, &dObsId, &dObsIq);
}

foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    double dId = s_tSimPlant.dCurrentD;
    double dIq = s_tSimPlant.dCurrentQ;
    double dTheta = (double)foc_angle_to_turns(s_tSimElectricalAngle) *
                    2.0 * 3.141592653589793;
    double dCos = cos(dTheta);
    double dSin = sin(dTheta);
    double dIalpha = dCos * dId - dSin * dIq;
    double dIbeta = dSin * dId + dCos * dIq;
    const double dIbase = 7.0;

    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }

    double dIu = dIalpha;
    double dIv = -0.5 * dIalpha + (1.7320508075688772 / 2.0) * dIbeta;
    double dIw = -0.5 * dIalpha - (1.7320508075688772 / 2.0) * dIbeta;

    double dCountsU = (dIu / dIbase) * (double)FOC_CURRENT_COUNTS_PER_BASE;
    double dCountsV = (dIv / dIbase) * (double)FOC_CURRENT_COUNTS_PER_BASE;
    double dCountsW = (dIw / dIbase) * (double)FOC_CURRENT_COUNTS_PER_BASE;

    int32_t nRawU = 2048 - (int32_t)round(dCountsU);
    int32_t nRawV = 2048 - (int32_t)round(dCountsV);
    int32_t nRawW = 2048 - (int32_t)round(dCountsW);

    ptSample->wU = (uint32_t)nRawU;
    ptSample->wV = (uint32_t)nRawV;
    ptSample->wW = (uint32_t)nRawW;
    return FOC_RESULT_OK;
}

static void sim_tick(void)
{
    sim_step_plant();

    if (s_wPwmDelayTicks > 1U) {
        s_tActiveDuty = s_atDutyPipeline[0];
        for (uint32_t i = 0U; i < s_wPwmDelayTicks - 1U; i++) {
            s_atDutyPipeline[i] = s_atDutyPipeline[i + 1U];
        }
    } else if (s_wPwmDelayTicks == 1U) {
        s_tActiveDuty = s_atDutyPipeline[0];
    }

    s_lSystemTick += 50;
    foc_app_HighFrequencyISR();
}

static void setup_test_app(double dR, double dLd, double dLq)
{
    foc_identify_plant_cfg_t plant_cfg = {
        .dResistance = dR,
        .dInductanceD = dLd,
        .dInductanceQ = dLq,
        .dDt = 50e-6,
        .dVbase = 12.0,
        .dIbase = 7.0,
        .dVoltageOffsetD = 0.0,
        .dVoltageOffsetQ = 0.0,
        .dAdcCurrentOffsetD = 0.0,
        .dAdcCurrentOffsetQ = 0.0,
        .dAdcCurrentNoiseD = 0.0,
        .dAdcCurrentNoiseQ = 0.0,
        .dCurrentSatMax = 0.0,
        .wSubmitDelaySamples = 0U,
        .dSubmitFractionalDelay = 0.0,
    };
    foc_identify_plant_Init(&s_tSimPlant, &plant_cfg);

    s_tActiveDuty = (foc_duty_abc_t){
        .qU = FOC_HALF, .qV = FOC_HALF, .qW = FOC_HALF
    };
    memset(s_atDutyPipeline, 0, sizeof(s_atDutyPipeline));
    s_wPwmDelayTicks = 1U;
    s_bPwmEnabled = false;
    s_bPwmFault = false;
    s_dSimVbus = 12.0;
    s_tSimMechanicalAngle = (foc_angle_t){0};
    s_tSimElectricalAngle = (foc_angle_t){0};
    s_qSimMechanicalSpeed = FOC_ZERO;
    s_bEncoderValid = true;
    s_lSystemTick = 0;

    foc_app_cfg_t tConfig = {
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
        .wVoltageBaseMillivolt = 12000U,
        .wHighFrequencyPeriodNanoseconds = 50000U,
        .qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f),
    };

    int nInit = foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tConfig);
    if (nInit != MODUS_SUCCESS) {
        printf("DEBUG: foc_app_Init failed with code %d\n", nInit);
    }
    assert(nInit == MODUS_SUCCESS);

    tFocApp.tMotor.eState = MOTOR_STATE_IDLE;
    tFocApp.tMotor.bElectricalZeroValid = true;
    tFocApp.tMotor.tElectricalZero = (foc_angle_t){0};
    tFocApp.tMotor.tCalib.bIsCalibrated = true;
    tFocApp.tMotor.tCalib.wOffsetU = 2048U;
    tFocApp.tMotor.tCalib.wOffsetV = 2048U;
    tFocApp.tMotor.tCalib.wOffsetW = 2048U;
    tFocApp.tMotor.wFaults = MOTOR_FAULT_NONE;
    tFocApp.bReady = true;
}

static void test_nominal_identify(void)
{
    const double dR = 0.50;
    const double dLd = 0.0010;
    const double dLq = 0.0015;

    setup_test_app(dR, dLd, dLq);
    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);

    for (uint32_t step = 0; step < 40000U; step++) {
        sim_tick();
        if ((step % 20U) == 0U) {
            foc_app_IdentifyForegroundStep(&tFocApp);
        }
        if (!tFocApp.bIdentifyActive) {
            break;
        }
    }
    foc_app_IdentifyForegroundStep(&tFocApp);

    foc_identify_status_t tIdStatus = {0};
    (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
    assert(tIdStatus.eState == FOC_IDENTIFY_STATE_COMPLETE);
    assert(tFocApp.tMotor.eState == MOTOR_STATE_IDLE);

    foc_identify_result_t tRes = {0};
    assert(foc_identify_GetResult(&tFocApp.tIdentify, &tRes) == FOC_RESULT_OK);

    float fZbase = 12.0f / 7.0f;
    float fOmegaBase = 2.0f * 3.141592653589793f * 100.0f;
    float fLbase = fZbase / fOmegaBase;

    double dRest = (double)foc_to_float(tRes.qResistancePu) * fZbase;
    double dLdest = (double)foc_to_float(tRes.qInductanceDPu) * fLbase;
    double dLqest = (double)foc_to_float(tRes.qInductanceQPu) * fLbase;

    double dRerr = fabs(dRest - dR) / dR;
    double dLderr = fabs(dLdest - dLd) / dLd;
    double dLqerr = fabs(dLqest - dLq) / dLq;

#if defined(FOC_NUMERIC_FIXED)
    assert(dRerr <= 0.05);
    assert(dLderr <= 0.05);
    assert(dLqerr <= 0.05);
#else
    assert(dRerr <= 0.03);
    assert(dLderr <= 0.03);
    assert(dLqerr <= 0.03);
#endif
    printf("  [PASS] test_nominal_identify (R=%.4f, Ld=%.4f mH, Lq=%.4f mH)\n",
           dRest, dLdest * 1000.0, dLqest * 1000.0);
}

static void test_nonzero_electrical_angle(void)
{
    const double dR = 0.50;
    const double dLd = 0.0010;
    const double dLq = 0.0015;

    setup_test_app(dR, dLd, dLq);
    s_tSimMechanicalAngle = foc_angle_from_turns(1.0f / 42.0f);
    s_tSimElectricalAngle = foc_angle_from_turns(1.0f / 6.0f);

    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);

    for (uint32_t step = 0; step < 40000U; step++) {
        sim_tick();
        if ((step % 20U) == 0U) {
            foc_app_IdentifyForegroundStep(&tFocApp);
        }
        if (!tFocApp.bIdentifyActive) {
            break;
        }
    }
    foc_app_IdentifyForegroundStep(&tFocApp);

    foc_identify_status_t tIdStatus = {0};
    (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
    assert(tIdStatus.eState == FOC_IDENTIFY_STATE_COMPLETE);

    foc_identify_result_t tRes = {0};
    assert(foc_identify_GetResult(&tFocApp.tIdentify, &tRes) == FOC_RESULT_OK);

    float fZbase = 12.0f / 7.0f;
    float fOmegaBase = 2.0f * 3.141592653589793f * 100.0f;
    float fLbase = fZbase / fOmegaBase;

    double dRest = (double)foc_to_float(tRes.qResistancePu) * fZbase;
    double dLdest = (double)foc_to_float(tRes.qInductanceDPu) * fLbase;
    double dLqest = (double)foc_to_float(tRes.qInductanceQPu) * fLbase;

    assert(fabs(dRest - dR) / dR <= 0.05);
    assert(fabs(dLdest - dLd) / dLd <= 0.05);
    assert(fabs(dLqest - dLq) / dLq <= 0.05);
    printf("  [PASS] test_nonzero_electrical_angle\n");
}

static void test_pwm_delays(void)
{
    const double dR = 0.50;
    const double dLd = 0.0010;
    const double dLq = 0.0015;

    for (uint32_t delay = 0; delay <= 2; delay += 2) {
        setup_test_app(dR, dLd, dLq);
        s_wPwmDelayTicks = delay;

        foc_app_CmdIdentify("start");
        for (uint32_t step = 0; step < 40000U; step++) {
            sim_tick();
            if ((step % 20U) == 0U) {
                foc_app_IdentifyForegroundStep(&tFocApp);
            }
            if (!tFocApp.bIdentifyActive) {
                break;
            }
        }
        foc_app_IdentifyForegroundStep(&tFocApp);

        foc_identify_status_t tIdStatus = {0};
        (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
        assert(tIdStatus.eState == FOC_IDENTIFY_STATE_COMPLETE);
    }
    printf("  [PASS] test_pwm_delays (D=0, D=2)\n");
}

static void test_cancel_command(void)
{
    setup_test_app(0.50, 0.0010, 0.0015);
    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);

    for (uint32_t step = 0; step < 100U; step++) {
        sim_tick();
    }
    assert(tFocApp.tMotor.eState == MOTOR_STATE_RUNNING);

    foc_app_CmdIdentify("cancel");
    sim_tick();
    assert(!tFocApp.bIdentifyActive);
    assert(tFocApp.tMotor.eState == MOTOR_STATE_IDLE);
    printf("  [PASS] test_cancel_command\n");
}

static void test_motor_stop_priority(void)
{
    setup_test_app(0.50, 0.0010, 0.0015);
    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);

    for (uint32_t step = 0; step < 100U; step++) {
        sim_tick();
    }
    assert(tFocApp.tMotor.eState == MOTOR_STATE_RUNNING);

    foc_app_CmdMotor("stop");
    sim_tick();
    assert(!tFocApp.bIdentifyActive);
    assert(tFocApp.tMotor.eState == MOTOR_STATE_IDLE);
    printf("  [PASS] test_motor_stop_priority\n");
}

static void test_mutex_rejection(void)
{
    setup_test_app(0.50, 0.0010, 0.0015);
    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);

    sim_tick();
    assert(tFocApp.tMotor.eState == MOTOR_STATE_RUNNING);

    foc_app_CmdMotor("speed 1.0");
    assert(tFocApp.tMotor.tCommand.eMode == FOC_MODE_VOLTAGE);

    foc_app_CmdMotor("current 0.1 0.0");
    assert(tFocApp.tMotor.tCommand.eMode == FOC_MODE_VOLTAGE);

    foc_app_CmdMotor("voltage 0.1 0.0");
    assert(tFocApp.tMotor.tCommand.eMode == FOC_MODE_VOLTAGE);

    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);

    foc_app_CmdIdentify("cancel");
    sim_tick();
    assert(tFocApp.tMotor.eState == MOTOR_STATE_IDLE);
    printf("  [PASS] test_mutex_rejection\n");
}

static void test_isr_timeout(void)
{
    setup_test_app(0.50, 0.0010, 0.0015);
    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);

    for (uint32_t step = 0; step < 50U; step++) {
        sim_tick();
    }
    assert(tFocApp.tMotor.eState == MOTOR_STATE_RUNNING);

    s_lSystemTick += 15000;
    foc_app_IdentifyForegroundStep(&tFocApp);

    assert(!tFocApp.bIdentifyActive);
    assert(tFocApp.tMotor.eState == MOTOR_STATE_IDLE);

    foc_identify_status_t tIdStatus = {0};
    (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
    assert(tIdStatus.eState == FOC_IDENTIFY_STATE_ERROR);
    printf("  [PASS] test_isr_timeout\n");
}

static void test_bus_voltage_scale_error(void)
{
    const double dR = 0.50;
    const double dLd = 0.0010;
    const double dLq = 0.0015;

    setup_test_app(dR, dLd, dLq);
    s_dSimVbus = 11.0;

    foc_app_CmdIdentify("start");
    for (uint32_t step = 0; step < 40000U; step++) {
        sim_tick();
        if ((step % 20U) == 0U) {
            foc_app_IdentifyForegroundStep(&tFocApp);
        }
        if (!tFocApp.bIdentifyActive) {
            break;
        }
    }
    foc_app_IdentifyForegroundStep(&tFocApp);

    foc_identify_status_t tIdStatus = {0};
    (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
    assert(tIdStatus.eState == FOC_IDENTIFY_STATE_COMPLETE);

    foc_identify_result_t tRes = {0};
    assert(foc_identify_GetResult(&tFocApp.tIdentify, &tRes) == FOC_RESULT_OK);

    float fZbase = 12.0f / 7.0f;
    float fOmegaBase = 2.0f * 3.141592653589793f * 100.0f;
    float fLbase = fZbase / fOmegaBase;

    double dRest = (double)foc_to_float(tRes.qResistancePu) * fZbase;
    double dExpectedRatio = 11.0 / 12.0;
    double dExpectedR = dR * dExpectedRatio;

    assert(fabs(dRest - dExpectedR) / dExpectedR <= 0.05);
    printf("  [PASS] test_bus_voltage_scale_error (Vbus=11V -> R=%.4f)\n",
           dRest);
}

static void test_reset_and_restart(void)
{
    setup_test_app(0.50, 0.0010, 0.0015);
    foc_app_CmdIdentify("start");

    for (uint32_t step = 0; step < 40000U; step++) {
        sim_tick();
        if ((step % 20U) == 0U) {
            foc_app_IdentifyForegroundStep(&tFocApp);
        }
        if (!tFocApp.bIdentifyActive) {
            break;
        }
    }
    foc_app_IdentifyForegroundStep(&tFocApp);

    foc_identify_status_t tIdStatus = {0};
    (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
    assert(tIdStatus.eState == FOC_IDENTIFY_STATE_COMPLETE);

    foc_app_CmdIdentify("reset");
    (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
    assert(tIdStatus.eState == FOC_IDENTIFY_STATE_IDLE);
    assert(!tFocApp.bIdentifyActive);

    foc_app_CmdIdentify("start");
    assert(tFocApp.bIdentifyActive);
    for (uint32_t step = 0; step < 40000U; step++) {
        sim_tick();
        if ((step % 20U) == 0U) {
            foc_app_IdentifyForegroundStep(&tFocApp);
        }
        if (!tFocApp.bIdentifyActive) {
            break;
        }
    }
    foc_app_IdentifyForegroundStep(&tFocApp);

    (void)foc_identify_GetStatus(&tFocApp.tIdentify, &tIdStatus);
    assert(tIdStatus.eState == FOC_IDENTIFY_STATE_COMPLETE);
    printf("  [PASS] test_reset_and_restart\n");
}

int main(void)
{
    printf("Starting foc_app identify integration tests...\n");
    test_nominal_identify();
    test_nonzero_electrical_angle();
    test_pwm_delays();
    test_cancel_command();
    test_motor_stop_priority();
    test_mutex_rejection();
    test_isr_timeout();
    test_bus_voltage_scale_error();
    test_reset_and_restart();
    printf("All 9 integration tests passed!\n");
    return 0;
}
