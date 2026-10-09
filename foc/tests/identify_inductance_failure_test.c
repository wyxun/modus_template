/****************************************************************************
 * @file    identify_inductance_failure_test.c
 * @brief   Host tests for failed inductance-identification runs.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "identify.h"
#include "motor.h"
#include "foc_modulation.h"
#include "perf_counter.h"

static int64_t s_lNowTicks = 1;
static uint32_t s_wAbortCount = 0U;
static identify_result_t s_tIdentifyResult = {0};
static uint32_t s_wIdentifyLogCount = 0U;
static char s_chLastIdentifyLog[160];
uint8_t g_chGLogMask = 0xFFU;

void util_debug_Printf(const char *pchFormat, ...)
{
    va_list tArgs;

    va_start(tArgs, pchFormat);
    (void)vsnprintf(s_chLastIdentifyLog, sizeof(s_chLastIdentifyLog),
                    pchFormat, tArgs);
    va_end(tArgs);
    s_wIdentifyLogCount++;
}
static motor_status_t s_tMotorStatus = {
    .eState = MOTOR_STATE_IDLE,
    .wFaults = MOTOR_FAULT_NONE,
    .eMode = FOC_MODE_CURRENT,
    .bPwmEnabled = false,
    .bElectricalZeroValid = false,
};

int64_t get_system_ticks(void)
{
    return s_lNowTicks;
}

int64_t perfc_convert_ms_to_ticks(uint32_t wMilliseconds)
{
    return (int64_t)wMilliseconds;
}

bool __perfc_is_time_out(int64_t lPeriod,
                         int64_t *plTimestamp,
                         bool bAutoReload)
{
    /* Same deadline-slot semantics as perf_counter.c, with a fake tick. */
    if (plTimestamp == NULL) {
        return false;
    }
    if (*plTimestamp == 0) {
        *plTimestamp = s_lNowTicks + lPeriod;
        return false;
    }
    if (s_lNowTicks < *plTimestamp) {
        return false;
    }
    if (bAutoReload) {
        *plTimestamp = s_lNowTicks + lPeriod;
    }
    return true;
}

foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode)
{
    s_tMotorStatus.eState = MOTOR_STATE_RUNNING;
    s_tMotorStatus.eMode = eMode;
    s_tMotorStatus.bPwmEnabled = true;
    ptMotor->eState = MOTOR_STATE_RUNNING;
    ptMotor->bPwmEnabled = true;
    return FOC_RESULT_OK;
}

static bool s_bHoldAlignment = false;

foc_result_t motor_StartIdentification(motor_t *ptMotor)
{
    foc_result_t result = motor_Start(ptMotor, FOC_MODE_VOLTAGE);
    s_tMotorStatus.bIdentificationActive = true;
    if (s_bHoldAlignment) {
        s_tMotorStatus.eState = MOTOR_STATE_ALIGN;
    }
    return result;
}

foc_result_t motor_SetIdentificationModulation(motor_t *ptMotor,
    const foc_ab_t *ptCommand)
{
    return motor_SetVoltageReference(ptMotor,
        ptCommand->qAlpha, ptCommand->qBeta);
}

void motor_Stop(motor_t *ptMotor)
{
    s_tMotorStatus.eState = ptMotor->eState == MOTOR_STATE_FAULT ?
        MOTOR_STATE_FAULT : MOTOR_STATE_IDLE;
    s_tMotorStatus.bPwmEnabled = false;
    s_tMotorStatus.bIdentificationActive = false;
    ptMotor->eState = s_tMotorStatus.eState;
    ptMotor->bPwmEnabled = false;
}

foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    (void)ptMotor;
    (void)qD;
    (void)qQ;
    return FOC_RESULT_OK;
}

foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor,
    const foc_ab_t *ptVoltageCommand)
{
    (void)ptMotor;
    (void)ptVoltageCommand;
    return FOC_RESULT_OK;
}

void motor_IdentificationAbortIsr(motor_t *ptMotor,
                                  motor_fault_e eFault)
{
    s_wAbortCount++;
    ptMotor->eState = MOTOR_STATE_FAULT;
    ptMotor->bPwmEnabled = false;
    ptMotor->wFaults |= (uint32_t)eFault;
    s_tMotorStatus.eState = MOTOR_STATE_FAULT;
    s_tMotorStatus.bPwmEnabled = false;
    s_tMotorStatus.wFaults = ptMotor->wFaults;
}

foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    (void)ptMotor;
    *ptStatus = s_tMotorStatus;
    return FOC_RESULT_OK;
}

static const identify_cfg_t s_tIdentifyConfig = {
    .wVoltageBaseMillivolt = 12000U,
    .wCurrentBaseMilliamp = 3500U,
    .wResistanceMilliohm = 1000U,
    .qElectricalSpeedBaseHz = FOC_SCALAR(350.0f),
};

/* App's same-ISR request submission is reproduced at the hardware boundary. */
static void test_IsrStep(identify_t *ptIdentify, motor_t *ptMotor,
                         const identify_inductance_sample_t *ptSample)
{
    identify_isr_request_t request = {0};
    identify_InductanceIsrStep(ptIdentify, ptSample, &request);
    if (request.bAbort) {
        motor_IdentificationAbortIsr(ptMotor, MOTOR_FAULT_IDENTIFICATION);
    } else if (request.bSetModulation) {
        assert(motor_IdentificationApplyIsr(ptMotor,
            &request.tModulationCommand) == FOC_RESULT_OK);
    } else {
        /* No control request. */
    }
}

static void test_InductanceFailure(
    foc_scalar_t qPositiveLast)
{
    identify_t tIdentify = {0};
    identify_status_t tStatus = {0};
    identify_inductance_cfg_t tConfig = {
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
    identify_inductance_sample_t tSample = {
        .bValid = true,
        .tCurrentPu.qAlpha = FOC_ZERO,
        .tCurrentPu.qBeta = FOC_ZERO,
        .bPwmSaturated = false,
    };
    motor_t tMotor = {0};
    uint8_t chHalf = 0U;
    uint8_t chCycle = 0U;
    uint32_t wLogBefore = s_wIdentifyLogCount;

    tMotor.tParams.wCurrentBaseMilliamp = 3500U;
    tMotor.tParams.wResistanceMilliohm = 1000U;
    tMotor.tLimits.qMaxModulation = FOC_ONE;
    tMotor.tLimits.qMaxPhaseCurrent = FOC_ONE;
    assert(identify_Init(&tIdentify, &s_tIdentifyConfig) == FOC_RESULT_OK);
    assert(identify_StartInductance(&tIdentify, &tConfig) ==
           FOC_RESULT_OK);
    assert(identify_Run(&tIdentify, &tMotor, &s_tIdentifyResult) ==
           FOC_RESULT_BUSY);

    for (chHalf = 0U; chHalf < 4U; chHalf++) {
        for (chCycle = 0U; chCycle < 10U; chCycle++) {
            tSample.qVoltageModelAlphaPu = (chHalf & 1U) == 0U ?
                FOC_SCALAR(0.1f) : FOC_SCALAR(-0.1f);
            tSample.tCurrentPu.qAlpha = FOC_ZERO;
            if ((chHalf & 1U) == 0U && chCycle == 3U) {
                tSample.tCurrentPu.qAlpha = qPositiveLast;
            } else if ((chHalf & 1U) != 0U && chCycle == 2U) {
                tSample.tCurrentPu.qAlpha = FOC_SCALAR(0.01f);
            }
            test_IsrStep(&tIdentify, &tMotor, &tSample);
        }
    }
    assert(identify_Run(&tIdentify, &tMotor, &s_tIdentifyResult) ==
           FOC_RESULT_OUT_OF_RANGE);
    assert(s_tIdentifyResult.eOperation == IDENTIFY_OPERATION_NONE);
    assert(identify_GetStatus(&tIdentify, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == IDENTIFY_STATE_ERROR);
    assert(tStatus.eLastResult == FOC_RESULT_OUT_OF_RANGE);
#if IDENTIFY_LOG_ENABLE
    assert(s_wIdentifyLogCount == wLogBefore + 1U);
    assert(strstr(s_chLastIdentifyLog, "identify Ld calculate") != NULL);
#else
    assert(s_wIdentifyLogCount == wLogBefore);
#endif
}

/**
 * @brief Reject a saturated PWM interval before publishing inductance.
 * @param wCleanSamples Samples accepted before clipping is reported.
 * @return None.
 */
static void test_InductancePwmSaturation(uint32_t wCleanSamples)
{
    identify_t tIdentify = {0};
    motor_t tMotor = {0};
    identify_status_t tStatus = {0};
    identify_inductance_cfg_t tConfig = {
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
    identify_inductance_sample_t tSample = {
        .bValid = true,
    };
    uint32_t wLogBefore = s_wIdentifyLogCount;
    tMotor.tParams.wCurrentBaseMilliamp = 3500U;
    tMotor.tParams.wResistanceMilliohm = 1000U;
    tMotor.tLimits.qMaxModulation = FOC_ONE;
    tMotor.tLimits.qMaxPhaseCurrent = FOC_ONE;
    const uint32_t wBefore = s_wAbortCount;
    /* Each case begins with an independently ready, calibrated Motor. */
    s_tMotorStatus.eState = MOTOR_STATE_IDLE;
    s_tMotorStatus.wFaults = MOTOR_FAULT_NONE;
    assert(identify_Init(&tIdentify, &s_tIdentifyConfig) == FOC_RESULT_OK);
    assert(identify_StartInductance(&tIdentify, &tConfig) == FOC_RESULT_OK);
    assert(identify_Run(&tIdentify, &tMotor, &s_tIdentifyResult) ==
           FOC_RESULT_BUSY);
    assert(tMotor.eState == MOTOR_STATE_RUNNING && tMotor.bPwmEnabled);
    for (uint32_t wIndex = 0U; wIndex < wCleanSamples; wIndex++) {
        test_IsrStep(&tIdentify, &tMotor, &tSample);
    }
    assert(s_wAbortCount == wBefore);
    /* Use the real modulation clipping report, as forwarded by Core/App. */
    foc_ab_t tVoltage = {FOC_SCALAR(0.8f), FOC_ZERO};
    foc_duty_abc_t tDuty = {0};
    assert(foc_svpwm(&tVoltage, &tDuty, &tSample.bPwmSaturated) ==
           FOC_RESULT_OK);
    assert(tSample.bPwmSaturated);
    test_IsrStep(&tIdentify, &tMotor, &tSample);
    assert(s_wAbortCount == wBefore + 1U);
#if IDENTIFY_LOG_ENABLE
    assert(tIdentify.bFailureLogPending);
    assert(tIdentify.eFailureOperation == IDENTIFY_OPERATION_INDUCTANCE);
    assert(tIdentify.eFailureCause ==
           IDENTIFY_FAILURE_CAUSE_PWM_SATURATED);
#endif
    assert(s_wIdentifyLogCount == wLogBefore);
    assert(tMotor.eState == MOTOR_STATE_FAULT && !tMotor.bPwmEnabled);
    assert((tMotor.wFaults & MOTOR_FAULT_IDENTIFICATION) != 0U);
    test_IsrStep(&tIdentify, &tMotor, &tSample);
    assert(s_wAbortCount == wBefore + 1U);
    assert(identify_Run(&tIdentify, &tMotor, &s_tIdentifyResult) ==
           FOC_RESULT_SAFETY);
#if IDENTIFY_LOG_ENABLE
    assert(s_wIdentifyLogCount == wLogBefore + 1U);
    assert(strstr(s_chLastIdentifyLog, "identify Ld ISR") != NULL);
#else
    assert(s_wIdentifyLogCount == wLogBefore);
#endif
    assert(s_tIdentifyResult.eOperation == IDENTIFY_OPERATION_NONE);
    assert(tMotor.eState == MOTOR_STATE_FAULT && !tMotor.bPwmEnabled);
    assert(identify_StartInductance(&tIdentify, &tConfig) == FOC_RESULT_SAFETY);
    assert(identify_GetStatus(&tIdentify, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == IDENTIFY_STATE_ERROR);
    assert(tStatus.eLastResult == FOC_RESULT_SAFETY);
#if IDENTIFY_LOG_ENABLE
    assert(!tIdentify.bFailureLogPending);
    assert(identify_Run(&tIdentify, &tMotor, &s_tIdentifyResult) ==
           FOC_RESULT_SAFETY);
    assert(s_wIdentifyLogCount == wLogBefore + 1U);
#endif
}

/** @brief Reconstruct signed phase voltage from PWM and a measured bus.
 * @return None. */
static void test_VoltageModel(void)
{
    foc_ab_t modulation = {FOC_SCALAR(0.1f), FOC_SCALAR(0.05f)};
    foc_duty_abc_t duty = {0};
    foc_ab_t voltage = {0};
    bool clipped = false;
    assert(foc_svpwm(&modulation, &duty, &clipped) == FOC_RESULT_OK);
    assert(!clipped);
    assert(foc_modulation_ModelVoltageAlphaBeta(&duty,
        FOC_SCALAR(1.25f), &voltage) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(voltage.qAlpha) - 0.125f) < 0.0002f);
    assert(fabsf(foc_to_float(voltage.qBeta) - 0.0625f) < 0.0002f);
    assert(foc_modulation_ModelVoltageAlphaBeta(&duty,
        FOC_SCALAR(0.75f), &voltage) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(voltage.qAlpha) - 0.075f) < 0.0002f);
    assert(fabsf(foc_to_float(voltage.qBeta) - 0.0375f) < 0.0002f);
    duty = (foc_duty_abc_t){FOC_HALF, FOC_HALF, FOC_HALF};
    assert(foc_modulation_ModelVoltageAlphaBeta(&duty,
        FOC_SCALAR(2.0f), &voltage) == FOC_RESULT_OK);
    assert(voltage.qAlpha == FOC_ZERO && voltage.qBeta == FOC_ZERO);
    assert(foc_modulation_ModelVoltageAlphaBeta(&duty,
        FOC_ZERO, &voltage) == FOC_RESULT_OUT_OF_RANGE);
    duty.qU = FOC_SCALAR(1.1f);
    assert(foc_modulation_ModelVoltageAlphaBeta(&duty,
        FOC_ONE, &voltage) == FOC_RESULT_OUT_OF_RANGE);
}

/** @brief A physical RL transient gives the same L at different bus voltages.
 * @param bus_voltage Actual DC voltage, distinct from the fixed 12 V base.
 * @return None. */
static void test_InductancePhysicalModel(float bus_voltage)
{
    identify_t identify = {0};
    motor_t motor = {0};
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
    identify_inductance_sample_t sample = {.bValid = true};
    identify_result_t result = {0};
    uint32_t physical = 0U;

    s_tMotorStatus.eState = MOTOR_STATE_IDLE;
    s_tMotorStatus.wFaults = MOTOR_FAULT_NONE;
    s_tMotorStatus.bPwmEnabled = false;
    motor.tLimits.qMaxModulation = FOC_ONE;
    motor.tLimits.qMaxPhaseCurrent = FOC_ONE;
    assert(identify_Init(&identify, &s_tIdentifyConfig) == FOC_RESULT_OK);
    assert(identify_StartInductance(&identify, &cfg) == FOC_RESULT_OK);
    assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_BUSY);
    for (uint32_t half = 0U; half < 4U; half++) {
        float sign = (half & 1U) == 0U ? 1.0f : -1.0f;
        for (uint32_t cycle = 0U; cycle < 10U; cycle++) {
            /* Exact physical model: i(t)=U/R*(1-exp(-t*R/L)).
             * R=1 Ohm, L=860 uH, Ts=50 us. No PU formula in the oracle. */
            float t = (float)(cycle == 0U ? 0U : cycle - 1U) / 20000.0f;
            float current = sign * bus_voltage * 0.1f *
                            (1.0f - expf(-t / 0.00086f));
            sample.tCurrentPu.qAlpha = foc_from_float(current / 3.5f);
            sample.qVoltageModelAlphaPu = foc_from_float(
                sign * bus_voltage * 0.1f / 12.0f);
            test_IsrStep(&identify, &motor, &sample);
        }
    }
    assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_OK);
    assert(result.eOperation == IDENTIFY_OPERATION_INDUCTANCE);
    assert(fabsf(foc_to_float(result.tValue.tInductance.qInductanceDPu) -
        0.00086f * 3.5f * 6.283185307f * 350.0f / 12.0f) < 0.004f);
    assert(identify_ResultToPhysical(&identify, IDENTIFY_OPERATION_INDUCTANCE,
        result.tValue.tInductance.qInductanceDPu, &physical) == FOC_RESULT_OK);
    assert(physical >= 855U && physical <= 865U);
    /* Consuming an output does not erase the module's retained PU result. */
    assert(identify.tInductance.tResult.qInductanceDPu ==
           result.tValue.tInductance.qInductanceDPu);
    assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_BUSY);
    assert(result.eOperation == IDENTIFY_OPERATION_NONE);
}

/** @brief Independent instance bases and wrong-entry dispatch are isolated.
 * @return None. */
static void test_Boundaries(void)
{
    identify_t a = {0};
    identify_t b = {0};
    identify_cfg_t cfg = s_tIdentifyConfig;
    identify_isr_request_t request = {.bAbort = true, .bSetModulation = true};
    identify_inductance_sample_t wrong = {.bValid = false};
    uint32_t physical = 0U;

    assert(identify_Init(&a, &cfg) == FOC_RESULT_OK);
    cfg.wCurrentBaseMilliamp = 500U;
    assert(identify_Init(&b, &cfg) == FOC_RESULT_OK);
    assert(identify_ResultToPhysical(&a, IDENTIFY_OPERATION_RESISTANCE,
        FOC_ONE, &physical) == FOC_RESULT_OK);
    assert(physical == 3429U);
    assert(identify_ResultToPhysical(&b, IDENTIFY_OPERATION_RESISTANCE,
        FOC_ONE, &physical) == FOC_RESULT_OK);
    assert(physical == 24000U);
    assert(identify_StartResistance(&a) == FOC_RESULT_OK);
    identify_InductanceIsrStep(&a, &wrong, &request);
    assert(!request.bAbort && !request.bSetModulation);
    assert(a.eOperation == IDENTIFY_OPERATION_RESISTANCE);
    assert(b.eState == IDENTIFY_STATE_IDLE);
    identify_ResistanceIsrStep(&a, NULL, &request);
    assert(request.bAbort);
    assert(a.eState == IDENTIFY_STATE_ERROR);
    cfg.wVoltageBaseMillivolt = 0U;
    assert(identify_Init(&b, &cfg) == FOC_RESULT_INVALID_ARGUMENT);
    assert(b.eState == IDENTIFY_STATE_UNINITIALIZED);
    assert(identify_ResultToPhysical(&a, IDENTIFY_OPERATION_NONE,
        FOC_ONE, &physical) == FOC_RESULT_INVALID_ARGUMENT);
    assert(identify_ResultToPhysical(&a, IDENTIFY_OPERATION_RESISTANCE,
        FOC_ZERO, &physical) == FOC_RESULT_OUT_OF_RANGE);
}

/** @brief Alignment gates capture, times out, stops and restarts per instance.
 * @return None. */
static void test_StationaryPreparation(void)
{
    for (unsigned scenario = 0U; scenario < 4U; scenario++) {
        identify_t identify = {0};
        motor_t motor = {0};
        identify_result_t result = {0};
        identify_inductance_cfg_t cfg = {
            .wInjectionFrequencyHz = 1000U,
            .hwCaptureDelayCycles = 2U,
            .hwCaptureSampleCount = 2U,
            .hwHalfCycleCount = 4U,
            .qModulationAmplitude = FOC_SCALAR(0.1f),
            .qMaxIdentificationCurrent = FOC_SCALAR(0.5f),
            .qMinCurrentDelta = FOC_SCALAR(0.01f),
            .qMaxTransverseCurrentPu = FOC_SCALAR(0.02f),
            .hwTransverseFaultCycles = 2U,
        };
        identify_inductance_sample_t sample = {.bValid = false};
        identify_isr_request_t request = {0};
        motor.tLimits.qMaxModulation = FOC_ONE;
        motor.tLimits.qMaxPhaseCurrent = FOC_ONE;
        s_tMotorStatus = (motor_status_t){.eState = MOTOR_STATE_IDLE};
        s_bHoldAlignment = true;
        assert(identify_Init(&identify, &s_tIdentifyConfig) == FOC_RESULT_OK);
        cfg.hwCaptureDelayCycles = 1U;
        assert(identify_StartInductance(&identify, &cfg) ==
               FOC_RESULT_INVALID_ARGUMENT);
        cfg.hwCaptureDelayCycles = 2U;
        assert(identify_StartInductance(&identify, &cfg) == FOC_RESULT_OK);
        assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_BUSY);
        identify_InductanceIsrStep(&identify, &sample, &request);
        assert(!request.bAbort && !request.bSetModulation);
        assert(!identify.tInductance.bActive);
        s_lNowTicks += IDENTIFY_ALIGNMENT_TIMEOUT_MS - 1U;
        assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_BUSY);
        if (scenario == 0U) {
            s_tMotorStatus.eState = MOTOR_STATE_RUNNING;
            assert(identify_Run(&identify, &motor, &result) ==
                   FOC_RESULT_BUSY);
            assert(identify.tInductance.bActive);
            assert(identify.tStationary.bReady);
        } else if (scenario == 3U) {
            identify_Stop(&identify, &motor);
            assert(!identify.tStationary.bReady);
            assert(identify.tStationary.chRunPt == 0U);
            s_bHoldAlignment = false;
            assert(identify_StartInductance(&identify, &cfg) == FOC_RESULT_OK);
            assert(identify_Run(&identify, &motor, &result) ==
                   FOC_RESULT_BUSY);
            assert(identify.tInductance.bActive);
        } else {
            if (scenario == 1U) {
                s_lNowTicks++;
            } else {
                s_tMotorStatus.eState = MOTOR_STATE_FAULT;
                s_tMotorStatus.wFaults = MOTOR_FAULT_ALIGN;
                motor.eState = MOTOR_STATE_FAULT;
            }
            assert(identify_Run(&identify, &motor, &result) ==
                   FOC_RESULT_SAFETY);
            assert(identify.eState == IDENTIFY_STATE_ERROR);
            assert(!s_tMotorStatus.bPwmEnabled);
            assert(!s_tMotorStatus.bIdentificationActive);
            assert(result.eOperation == IDENTIFY_OPERATION_NONE);
            uint32_t logs = s_wIdentifyLogCount;
            assert(identify_Run(&identify, &motor, &result) ==
                   FOC_RESULT_SAFETY);
            assert(s_wIdentifyLogCount == logs);
            assert(identify_Reset(&identify, &motor) == FOC_RESULT_OK);
            assert(identify.tStationary.chRunPt == 0U);
        }
        identify_Stop(&identify, &motor);
    }
    s_bHoldAlignment = false;
    s_tMotorStatus = (motor_status_t){.eState = MOTOR_STATE_IDLE};
}

/** @brief Signed transverse disturbance requires consecutive samples.
 * @return None. */
static void test_TransverseCurrent(void)
{
    identify_t identify = {0};
    motor_t motor = {0};
    identify_result_t result = {0};
    identify_inductance_cfg_t cfg = {
        .wInjectionFrequencyHz = 1000U,
        .hwCaptureDelayCycles = 2U,
        .hwCaptureSampleCount = 2U,
        .hwHalfCycleCount = 4U,
        .qModulationAmplitude = FOC_SCALAR(0.1f),
        .qMaxIdentificationCurrent = FOC_SCALAR(0.5f),
        .qMinCurrentDelta = FOC_SCALAR(0.01f),
        .qMaxTransverseCurrentPu = FOC_SCALAR(0.02f),
        .hwTransverseFaultCycles = 2U,
    };
    identify_inductance_sample_t sample = {.bValid = true};
    motor.tLimits.qMaxModulation = FOC_ONE;
    motor.tLimits.qMaxPhaseCurrent = FOC_ONE;
    s_tMotorStatus = (motor_status_t){.eState = MOTOR_STATE_IDLE};
    assert(identify_Init(&identify, &s_tIdentifyConfig) == FOC_RESULT_OK);
    assert(identify_StartInductance(&identify, &cfg) == FOC_RESULT_OK);
    assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_BUSY);
    uint32_t aborts = s_wAbortCount;
    uint32_t logs = s_wIdentifyLogCount;
    sample.tCurrentPu.qBeta = FOC_SCALAR(0.03f);
    test_IsrStep(&identify, &motor, &sample);
    assert(s_wAbortCount == aborts);
    assert(identify.tInductance.tCapture.wPhaseCycle == 1U);
    sample.tCurrentPu.qBeta = FOC_SCALAR(0.01f);
    test_IsrStep(&identify, &motor, &sample);
    assert(identify.tInductance.tCapture.hwTransverseFaultCount == 0U);
    sample.tCurrentPu.qBeta = FOC_SCALAR(-0.03f);
    test_IsrStep(&identify, &motor, &sample);
    test_IsrStep(&identify, &motor, &sample);
    assert(s_wAbortCount == aborts + 1U && !s_tMotorStatus.bPwmEnabled);
    assert(s_wIdentifyLogCount == logs);
#if IDENTIFY_LOG_ACTIVE
    assert(identify.eFailureCause ==
           IDENTIFY_FAILURE_CAUSE_TRANSVERSE_CURRENT);
#endif
    assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_SAFETY);
    assert(s_wIdentifyLogCount == logs + (IDENTIFY_LOG_ACTIVE ? 1U : 0U));
#if IDENTIFY_LOG_ACTIVE
    assert(strstr(s_chLastIdentifyLog, "transverse-current") != NULL);
#endif
    assert(identify_Run(&identify, &motor, &result) == FOC_RESULT_SAFETY);
    assert(s_wIdentifyLogCount == logs + (IDENTIFY_LOG_ACTIVE ? 1U : 0U));
}

int main(void)
{
    test_StationaryPreparation();
    test_TransverseCurrent();
    test_VoltageModel();
    test_Boundaries();
    test_InductancePhysicalModel(10.8f);
    test_InductancePhysicalModel(15.0f);
    test_InductanceFailure(FOC_SCALAR(0.004f));
    test_InductanceFailure(FOC_SCALAR(-0.02f));
    test_InductancePwmSaturation(0U);
    test_InductancePwmSaturation(2U);
    return 0;
}
