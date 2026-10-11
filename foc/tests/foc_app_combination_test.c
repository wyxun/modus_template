/****************************************************************************
 * @file    foc_app_combination_test.c
 * @brief   Real App/Motor/Position/Core/Observer with a hardware-only fixture.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "foc_app.h"
#include "foc_port.h"

static int64_t s_lTick = 0;
static bool s_bPwm = false;
static bool s_bSubmitFail = false;
static uint16_t s_hwRawAngle = 512U;
static uint32_t s_wSubmits = 0U;
static uint32_t s_wSensorReads = 0U;
static foc_result_t s_eSensorInit = FOC_RESULT_OK;

foc_result_t test_SensorInit(void)
{
    return s_eSensorInit;
}
foc_result_t test_SensorRead(foc_angle_t *ptAngle)
{
    s_wSensorReads++;
    ptAngle->wBam32 = (uint32_t)s_hwRawAngle << 20U;
    return FOC_RESULT_OK;
}
#if !defined(FOC_ENCODER_STATIC_BINDING)
static foc_result_t test_DynamicInit(void *pContext)
{
    (void)pContext;
    return test_SensorInit();
}
static foc_result_t test_DynamicRead(void *pContext, foc_angle_t *ptAngle)
{
    (void)pContext;
    return test_SensorRead(ptAngle);
}
static const foc_encoder_sensor_ops_t s_tSensorOps = {
    .fnInit = test_DynamicInit, .fnRead = test_DynamicRead,
};
const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface = {
    .ptOps = &s_tSensorOps, .pContext = &s_hwRawAngle,
};
#endif

/* Include composition to exercise its actual ISR and foreground scheduler. */
#include "../app/foc_app.c"

int64_t get_system_ticks(void) { return s_lTick; }
uint32_t perfc_get_systimer_frequency(void) { return 1000000U; }
int64_t perfc_convert_us_to_ticks(uint32_t wUs) { return (int64_t)wUs; }
int64_t perfc_convert_ms_to_ticks(uint32_t wMs)
{
    return (int64_t)wMs * 1000;
}
bool __perfc_is_time_out(int64_t lPeriod, int64_t *plTimestamp,
                         bool bReload)
{
    if (s_lTick - *plTimestamp < lPeriod) { return false; }
    if (bReload) { *plTimestamp = s_lTick; }
    return true;
}
int mbase_Init(modus_base_t *ptBase, modus_base_cfg_t *ptConfig)
{
    assert(ptBase != NULL && ptConfig != NULL);
    return MODUS_SUCCESS;
}
void foc_debug_CurrentStepRun(foc_app_t *ptApp) { (void)ptApp; }

foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    *ptSample = (foc_current_sample_t){2048U, 2048U, 2048U};
    return FOC_RESULT_OK;
}
foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    assert(ptDuty != NULL);
    s_wSubmits++;
    return s_bSubmitFail ? FOC_RESULT_SAFETY : FOC_RESULT_OK;
}
uint32_t foc_SampleDcBusRaw(foc_port_adc_channel_e eChannel)
{
    (void)eChannel;
    return 12000U;
}
void foc_port_StartAdcTrigger(void) {}
foc_result_t foc_PwmEnable(void)
{
    s_bPwm = true;
    return FOC_RESULT_OK;
}
foc_result_t foc_PwmSafeStop(void)
{
    s_bPwm = false;
    return FOC_RESULT_OK;
}
bool foc_PwmGetFault(void) { return false; }
foc_result_t foc_PwmClearFault(void) { return FOC_RESULT_OK; }

/** @brief One full composition control tick. @return None. */
static void test_Tick(void)
{
    s_lTick += 50;
    foc_app_HighFrequencyISR();
}

/** @brief Build an explicitly PU-only config. @return App configuration. */
static foc_app_cfg_t test_Config(motor_position_source_t eSource)
{
    foc_app_cfg_t tCfg = {0};
    tCfg.ePositionSource = eSource;
    tCfg.wHighFrequencyIsrHz = FOC_HF_ISR_HZ;
    tCfg.wVoltageBaseMillivolt = 12000U;
    tCfg.qElectricalSpeedBaseHz = FOC_SCALAR(100.0f);
    tCfg.tMotorCfg = (motor_cfg_t)FOC_APP_INIT_MOTOR_CONFIG;
    tCfg.tMotorCfg.tParams.wResistanceMilliohm = 500U;
    tCfg.tMotorCfg.tParams.wInductanceDMicroHenry = 1000U;
    tCfg.tMotorCfg.tParams.wInductanceQMicroHenry = 1000U;
    tCfg.tMotorCfg.tLimits.qMaxPhaseCurrent = FOC_SCALAR(0.5f);
    tCfg.tMotorCfg.tLimits.qMaxSpeedReference = FOC_SCALAR(0.8f);
    tCfg.tMotorCfg.fAlignTimeSeconds = 0.00005f;
    tCfg.tMotorCfg.qAlignCurrent = FOC_SCALAR(0.1f);
    tCfg.tMotorCfg.wStartupRampSteps = 0U;
    tCfg.tMotorCfg.wStartupMaxRunSteps = 0U;
    tCfg.tMotorCfg.nHardDragElectricalMilliHz =
        eSource == MOTOR_POSITION_SOURCE_HARD_DRAG ? 5000 : 0;
    tCfg.tEncoderCfg.qSpeedFilterAlpha = FOC_ONE;
    tCfg.tEncoderCfg.fInvalidTimeoutSeconds = 0.002f;
#if !defined(FOC_ENCODER_STATIC_BINDING)
    tCfg.tEncoderCfg.ptSensor = &g_tFocEncoderSensorInterface;
#endif
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    tCfg.tObserverCfg.tSmo.wBemfCutoffRadiansPerSecond = 10000U;
    tCfg.tObserverCfg.tSmo.wSlidingGainMillivolt = 3500U;
    tCfg.tObserverCfg.tSmo.qCurrentEstimateLimit = FOC_ONE;
#endif
    return tCfg;
}

/** @brief Exercise once-only profile conversion and init failure gates. */
static void test_InitContracts(void)
{
    foc_app_cfg_t tCfg = test_Config(MOTOR_POSITION_SOURCE_SENSOR);
    foc_app_speed_profile_t tProfile = {
        .qMaximumElectricalHz = FOC_SCALAR(50.0f),
        .tKpPerElectricalHz = {0, FOC_SCALAR(0.02f)},
        .tKiTsPerElectricalHz = {0, FOC_SCALAR(0.001f)},
        .tKdOverTsPerElectricalHz = {0, FOC_ZERO},
    };
    motor_cfg_t tBound = {0};
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) == FOC_RESULT_OK);
    assert(tBound.wControlFrequencyHz == FOC_HF_ISR_HZ);
    assert(tBound.tLimits.qMaxSpeedReference == FOC_SCALAR(0.8f));
    assert(tBound.tSpeedPiParams.tKp.qFraction ==
           tCfg.tMotorCfg.tSpeedPiParams.tKp.qFraction);
    tCfg.ptSpeedProfile = &tProfile;
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(tBound.tLimits.qMaxSpeedReference) - 0.5f)
           < 0.001f);
    tProfile.qMaximumElectricalHz = FOC_ZERO;
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) != FOC_RESULT_OK);
    tProfile.qMaximumElectricalHz = FOC_SCALAR(101.0f);
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) != FOC_RESULT_OK);
    tProfile.qMaximumElectricalHz = FOC_SCALAR(50.0f);
    tProfile.tKpPerElectricalHz.qFraction = FOC_SCALAR(1.5f);
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) != FOC_RESULT_OK);
    tProfile.tKpPerElectricalHz.qFraction = FOC_SCALAR(0.02f);
#if defined(FOC_NUMERIC_FLOAT)
    tProfile.qMaximumElectricalHz = NAN;
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) != FOC_RESULT_OK);
    tProfile.qMaximumElectricalHz = FOC_SCALAR(50.0f);
    tProfile.tKiTsPerElectricalHz.qFraction = NAN;
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) != FOC_RESULT_OK);
    tProfile.tKiTsPerElectricalHz.qFraction = FOC_SCALAR(0.001f);
#endif
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(foc_gain_apply(&tBound.tSpeedPiParams.tKp,
           FOC_SCALAR(0.1f))) - 0.2f) < 0.001f);
    assert(tCfg.tMotorCfg.tLimits.qMaxSpeedReference == FOC_SCALAR(0.8f));
    assert(foc_app_BindMotorConfig(&tCfg, &tBound) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(tBound.tLimits.qMaxSpeedReference) - 0.5f)
           < 0.001f);
    tCfg.wHighFrequencyIsrHz = FOC_HF_ISR_HZ / 2U;
    assert(foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tCfg) !=
           MODUS_SUCCESS);
    tCfg.wHighFrequencyIsrHz = FOC_HF_ISR_HZ;
    tCfg.ePositionSource = (motor_position_source_t)-1;
    assert(foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tCfg) !=
           MODUS_SUCCESS);
    tCfg.ePositionSource = MOTOR_POSITION_SOURCE_SENSOR;
    s_eSensorInit = FOC_RESULT_DISABLED;
    assert(foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tCfg) ==
           (int)FOC_RESULT_DISABLED);
    assert(!tFocApp.bReady && !s_bPwm);
    s_eSensorInit = FOC_RESULT_OK;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    tCfg = test_Config(MOTOR_POSITION_SOURCE_HARD_DRAG);
    tCfg.tMotorCfg.tParams.wInductanceQMicroHenry = 1500U;
    tCfg.tMotorCfg.wStartupRampSteps = 4U;
    tCfg.tMotorCfg.wStartupMaxRunSteps = 1000U;
    tCfg.bObserverTakeover = true;
    tCfg.wObserverQualificationSteps = 3U;
    tCfg.wObserverBlendSteps = 4U;
    tCfg.qObserverMinBemfPu = FOC_SCALAR(0.13f);
    tCfg.qObserverMinSpeedPu = FOC_SCALAR(0.1f);
    tCfg.qObserverMaxSpeedErrorRatio = FOC_SCALAR(0.1f);
    tCfg.qObserverMaxAngleErrorTurns = FOC_SCALAR(0.125f);
    assert(foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tCfg) ==
           (int)FOC_RESULT_DISABLED);
    assert(!tFocApp.bReady && !s_bPwm);
    tCfg.bObserverTakeover = false;
    tCfg.tObserverCfg.tSmo.wBemfCutoffRadiansPerSecond = 0U;
    assert(foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tCfg) ==
           (int)FOC_RESULT_INVALID_ARGUMENT);
    assert(!tFocApp.bReady && !s_bPwm);
#endif
}

/** @brief Check safe convergence through real composition and restart. */
static void test_Composition(motor_position_source_t eSource, bool bSalient)
{
    foc_app_cfg_t tCfg = test_Config(eSource);
    motor_status_t tStatus = {0};
    uint32_t wGeneration = 0U;
    uint32_t wSubmits = 0U;
    if (bSalient) {
        tCfg.tMotorCfg.tParams.wInductanceQMicroHenry = 1500U;
    }
    assert(foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tCfg) ==
           MODUS_SUCCESS);
    assert(tFocApp.bReady);
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    assert(tFocApp.tPosition.tObserver.bInitialized == !bSalient);
#endif
    test_Tick();
    assert(motor_GetStatus(&tFocApp.tMotor, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == MOTOR_STATE_IDLE);
    /* Alignment must finish without an encoder sample or zero capture. */
    uint32_t wSensorReads = s_wSensorReads;
    assert(motor_RequestAlignment(&tFocApp.tMotor, false) == FOC_RESULT_OK);
    assert(s_bPwm && tFocApp.tMotor.eState == MOTOR_STATE_ALIGN);
    test_Tick();
    assert(tFocApp.tMotor.eState == MOTOR_STATE_IDLE && !s_bPwm);
    assert(tFocApp.tMotor.wFaults == MOTOR_FAULT_NONE);
    assert(!tFocApp.tMotor.bElectricalZeroValid);
    assert(!motor_position_ZeroValid(&tFocApp.tPosition));
    assert(s_wSensorReads == wSensorReads);
    if (eSource == MOTOR_POSITION_SOURCE_SENSOR) {
        assert(tFocApp.tPosition.tSensor.pContext == &tFocApp.tEncoder);
        assert(foc_encoder_Run(&tFocApp.tEncoder) == FOC_RESULT_OK);
        assert(motor_RequestAlignment(&tFocApp.tMotor, true) ==
               FOC_RESULT_OK);
        test_Tick();
        assert(motor_position_ZeroValid(&tFocApp.tPosition));
    }
    assert(motor_Start(&tFocApp.tMotor, FOC_MODE_VOLTAGE) == FOC_RESULT_OK);
    assert(motor_SetVoltageReference(&tFocApp.tMotor,
           FOC_SCALAR(0.05f), FOC_ZERO) == FOC_RESULT_OK);
    wSubmits = s_wSubmits;
    test_Tick();
    assert(s_bPwm && s_wSubmits > wSubmits);
    wGeneration = tFocApp.tPosition.wLastRunGeneration;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    assert(tFocApp.tPosition.tObserver.tSmo.tAxis[0].e_hat == FOC_ZERO);
#endif
    if (eSource == MOTOR_POSITION_SOURCE_SENSOR) {
        s_lTick += 3000;
        test_Tick();
        assert(!s_bPwm);
        assert(motor_GetStatus(&tFocApp.tMotor, &tStatus) == FOC_RESULT_OK);
        assert(tStatus.eState == MOTOR_STATE_FAULT);
        assert(foc_encoder_Run(&tFocApp.tEncoder) == FOC_RESULT_OK);
        assert(motor_ClearFault(&tFocApp.tMotor) == FOC_RESULT_OK);
    } else {
        motor_Stop(&tFocApp.tMotor);
    }
    assert(motor_Start(&tFocApp.tMotor, FOC_MODE_VOLTAGE) == FOC_RESULT_OK);
    test_Tick();
    assert(tFocApp.tPosition.wLastRunGeneration != wGeneration);
    s_bSubmitFail = true;
    test_Tick();
    assert(motor_GetStatus(&tFocApp.tMotor, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == MOTOR_STATE_FAULT && !s_bPwm);
    s_bSubmitFail = false;
    assert(motor_ClearFault(&tFocApp.tMotor) == FOC_RESULT_OK);
    motor_Stop(&tFocApp.tMotor);
    assert(!s_bPwm);
}
/** @brief Ramp-based hard drag ignores the diagnostic fixed frequency. */
static void test_HardDragSpeed(void)
{
    foc_app_cfg_t tCfg = test_Config(MOTOR_POSITION_SOURCE_HARD_DRAG);
    tCfg.tMotorCfg.wStartupRampSteps = 4U;
    tCfg.tMotorCfg.wStartupMaxRunSteps = 10U;
    assert(foc_app_Init((uintptr_t)&tFocApp, (uintptr_t)&tCfg) ==
           MODUS_SUCCESS);
    test_Tick();
    assert(motor_StartSensorlessSpeed(&tFocApp.tMotor,
        FOC_SCALAR(0.4f)) == FOC_RESULT_OK);
    assert(tFocApp.tMotor.eState == MOTOR_STATE_ALIGN);
    test_Tick();
    assert(tFocApp.tMotor.eState == MOTOR_STATE_RUNNING);
    for (unsigned i = 0U; i < 4U; i++) {
        test_Tick();
        assert(fabsf(foc_to_float(tFocApp.tMotor.tInput.qElectricalSpeedPu)
            - (float)(i + 1U) * 0.1f) < 0.0002f);
    }
    assert(tFocApp.tMotor.tCommand.eMode == FOC_MODE_CURRENT);
    assert(!tFocApp.tMotor.bElectricalZeroValid);
    assert(!tFocApp.tMotor.tStart.bObserverActive);
    for (unsigned i = 4U; i <= 10U; i++) {
        test_Tick();
    }
    assert(tFocApp.tMotor.eState == MOTOR_STATE_FAULT && !s_bPwm);
    assert(tFocApp.tMotor.wFaults == MOTOR_FAULT_POSITION);
}

int main(void)
{
    test_InitContracts();
    test_Composition(MOTOR_POSITION_SOURCE_SENSOR, false);
    test_Composition(MOTOR_POSITION_SOURCE_HARD_DRAG, false);
    test_Composition(MOTOR_POSITION_SOURCE_SENSOR, true);
    test_Composition(MOTOR_POSITION_SOURCE_HARD_DRAG, true);
    test_HardDragSpeed();
    puts("real FOC composition contracts passed");
    return 0;
}
