/****************************************************************************
 * @file    identify_resistance_test.c
 * @brief   Host test for D/Q-axis resistance identification.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "identify.h"
#include "motor.h"
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
    if (plTimestamp == NULL) {
        return false;
    }
    if (*plTimestamp == 0) {
        *plTimestamp = s_lNowTicks;
        return false;
    }
    if ((s_lNowTicks - *plTimestamp) < lPeriod) {
        return false;
    }
    if (bAutoReload) {
        *plTimestamp = s_lNowTicks;
    }
    return true;
}

foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode)
{
    assert(ptMotor != NULL);
    s_tMotorStatus.eState = MOTOR_STATE_RUNNING;
    s_tMotorStatus.eMode = eMode;
    s_tMotorStatus.bPwmEnabled = true;
    ptMotor->eState = MOTOR_STATE_RUNNING;
    ptMotor->bPwmEnabled = true;
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

void motor_Stop(motor_t *ptMotor)
{
    assert(ptMotor != NULL);
    s_tMotorStatus.eState = MOTOR_STATE_IDLE;
    s_tMotorStatus.bPwmEnabled = false;
    ptMotor->eState = MOTOR_STATE_IDLE;
    ptMotor->bPwmEnabled = false;
}

foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    assert(ptMotor != NULL);
    assert(qQ == FOC_ZERO);
    ptMotor->tCommand.eMode = FOC_MODE_VOLTAGE;
    ptMotor->tCommand.tVoltageReference.qD = qD;
    ptMotor->tCommand.tVoltageReference.qQ = qQ;
    return FOC_RESULT_OK;
}

/**
 * @brief Accept the unused inductance-test voltage hook.
 * @param ptMotor Motor object.
 * @param ptVoltageCommand D/Q modulation command.
 * @return FOC_RESULT_OK.
 */
foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor,
    const foc_ab_t *ptVoltageCommand)
{
    (void)ptMotor;
    (void)ptVoltageCommand;
    return FOC_RESULT_OK;
}

/**
 * @brief Record the identification abort and disable the fake PWM.
 * @param ptMotor Motor object.
 * @param eFault Requested Motor fault.
 * @return None.
 */
void motor_IdentificationAbortIsr(motor_t *ptMotor,
                                  motor_fault_e eFault)
{
    assert(ptMotor != NULL);
    s_wAbortCount++;
    s_tMotorStatus.eState = MOTOR_STATE_FAULT;
    s_tMotorStatus.wFaults |= (uint32_t)eFault;
    s_tMotorStatus.bPwmEnabled = false;
    ptMotor->eState = MOTOR_STATE_FAULT;
    ptMotor->bPwmEnabled = false;
    ptMotor->tCommand.tVoltageReference = (foc_dq_t){0};
}

foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    (void)ptMotor;
    if (ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
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
                         const identify_resistance_sample_t *ptSample)
{
    identify_isr_request_t request = {0};
    identify_ResistanceIsrStep(ptIdentify, ptSample, &request);
    if (request.bAbort) {
        motor_IdentificationAbortIsr(ptMotor, MOTOR_FAULT_IDENTIFICATION);
    } else if (request.bSetModulation) {
        assert(motor_IdentificationApplyIsr(ptMotor,
            &request.tModulationCommand) == FOC_RESULT_OK);
    } else {
        /* No control request. */
    }
}

static void test_RunCapture(identify_t *ptIdentify,
                            motor_t *ptMotor,
                            foc_scalar_t qCurrentD,
                            foc_scalar_t qVoltageD)
{
    uint32_t wIsr = 0U;
    uint32_t wTotalIsr = IDENTIFY_RESISTANCE_SAMPLE_COUNT *
                         IDENTIFY_RESISTANCE_ISR_PER_SAMPLE;
    identify_resistance_sample_t tSample = {
        .bValid = true,
        .tCurrentPu.qAlpha = FOC_ZERO,
        .qVoltageModelAlphaPu = FOC_ZERO,
    };

    tSample.tCurrentPu.qAlpha = qCurrentD;
    tSample.qVoltageModelAlphaPu = qVoltageD;
    for (wIsr = 0U; wIsr < wTotalIsr; wIsr++) {
        test_IsrStep(ptIdentify, ptMotor, &tSample);
    }
}

static void test_RunProgress(identify_t *ptIdentify,
                             motor_t *ptMotor)
{
    identify_result_t tStepResult = {0};
    identify_resistance_sample_t sample = {
        .bValid = true,
        .tCurrentPu.qAlpha = ptMotor->tOutput.tCurrent.qD,
    };
    test_IsrStep(ptIdentify, ptMotor, &sample);
    foc_result_t eResult = identify_Run(ptIdentify, ptMotor, &tStepResult);

    if (tStepResult.eOperation != IDENTIFY_OPERATION_NONE) {
        s_tIdentifyResult = tStepResult;
    }

    assert(eResult == FOC_RESULT_BUSY || eResult == FOC_RESULT_OK);
}

/**
 * @brief Reject excess current before decimation, including settled batches.
 * @return None.
 */
static void test_Overcurrent(void)
{
    const identify_resistance_state_t aeStates[] = {
        IDENTIFY_RESISTANCE_STATE_WAIT_STABLE,
        IDENTIFY_RESISTANCE_STATE_CAPTURE,
        IDENTIFY_RESISTANCE_STATE_PROCESS_LEVEL,
    };
    const foc_ab_t atCurrents[] = {
        {FOC_SCALAR(0.15f), FOC_ZERO},
        {FOC_SCALAR(-0.15f), FOC_ZERO},
        {FOC_ZERO, FOC_SCALAR(0.15f)},
        {FOC_ZERO, FOC_SCALAR(-0.15f)},
        {FOC_SCALAR(0.10f), FOC_SCALAR(0.10f)},
#if defined(FOC_NUMERIC_FLOAT)
        {NAN, FOC_ZERO},
        {FOC_ZERO, INFINITY},
#endif
    };
    uint32_t wState = 0U;
    uint32_t wCurrent = 0U;

    for (wState = 0U; wState < (sizeof(aeStates) / sizeof(aeStates[0]));
         wState++) {
        for (wCurrent = 0U;
             wCurrent < (sizeof(atCurrents) / sizeof(atCurrents[0]));
             wCurrent++) {
            identify_t tIdentify = {0};
            motor_t tMotor = {0};
            identify_result_t tResult = {0};
            identify_resistance_sample_t tSample = {
        .bValid = true,
                .tCurrentPu.qAlpha = atCurrents[wCurrent].qAlpha,
                .tCurrentPu.qBeta = atCurrents[wCurrent].qBeta,
            };
            uint32_t wAbortBefore = s_wAbortCount;
            uint32_t wLogBefore = s_wIdentifyLogCount;
#if IDENTIFY_LOG_ENABLE
            identify_failure_cause_t eExpectedCause =
                IDENTIFY_FAILURE_CAUSE_OVERCURRENT;
#if defined(FOC_NUMERIC_FLOAT)
            if (!isfinite((double)atCurrents[wCurrent].qAlpha) ||
                !isfinite((double)atCurrents[wCurrent].qBeta)) {
                eExpectedCause = IDENTIFY_FAILURE_CAUSE_INVALID_SAMPLE;
            }
#endif
#endif

            assert(identify_Init(&tIdentify, &s_tIdentifyConfig) ==
                   FOC_RESULT_OK);
            assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);
            tIdentify.tResistance.eState = aeStates[wState];
            tIdentify.tResistance.bMotorStarted = true;
            tIdentify.tResistance.bBatchReady = (wState == 2U);
            tMotor.bPwmEnabled = true;
            test_IsrStep(&tIdentify, &tMotor, &tSample);
            assert(s_wAbortCount == wAbortBefore + 1U);
#if IDENTIFY_LOG_ENABLE
            assert(s_wIdentifyLogCount == wLogBefore);
            assert(tIdentify.bFailureLogPending);
            assert(tIdentify.eFailureOperation ==
                   IDENTIFY_OPERATION_RESISTANCE);
            assert(tIdentify.eFailureCause == eExpectedCause);
#endif
            assert(!tMotor.bPwmEnabled);
            assert(tIdentify.eState == IDENTIFY_STATE_ERROR);
            assert(tIdentify.eOperation == IDENTIFY_OPERATION_NONE);
            assert(tIdentify.tResistance.eState ==
                   IDENTIFY_RESISTANCE_STATE_ERROR);
            assert(!tIdentify.tResistance.bResultPending);
            assert(tIdentify.tResistance.hwCaptureSampleCount == 0U);
            assert(identify_Run(&tIdentify, &tMotor, &tResult) ==
                   FOC_RESULT_SAFETY);
            assert(tResult.eOperation == IDENTIFY_OPERATION_NONE);
#if IDENTIFY_LOG_ENABLE
            assert(s_wIdentifyLogCount == wLogBefore + 1U);
            assert(!tIdentify.bFailureLogPending);
            assert(strstr(s_chLastIdentifyLog, "identify R ISR") != NULL);
#else
            assert(s_wIdentifyLogCount == wLogBefore);
#endif
            test_IsrStep(&tIdentify, &tMotor, &tSample);
            assert(s_wAbortCount == wAbortBefore + 1U);
            (void)identify_Run(&tIdentify, &tMotor, &tResult);
            assert(s_wIdentifyLogCount == wLogBefore +
                   (IDENTIFY_LOG_ENABLE ? 1U : 0U));
        }
    }
}

/**
 * @brief Keep valid current active and leave idle identification alone.
 * @return None.
 */
static void test_ValidCurrent(void)
{
    identify_t tIdentify = {0};
    motor_t tMotor = {0};
    identify_resistance_sample_t tSample = {
        .bValid = true, .tCurrentPu.qAlpha = FOC_SCALAR(0.13f)};
    uint32_t wAbortBefore = s_wAbortCount;

    assert(identify_Init(&tIdentify, &s_tIdentifyConfig) == FOC_RESULT_OK);
    test_IsrStep(&tIdentify, &tMotor, &tSample);
    assert(s_wAbortCount == wAbortBefore);
    assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);
    tIdentify.tResistance.eState = IDENTIFY_RESISTANCE_STATE_WAIT_STABLE;
    tIdentify.tResistance.bMotorStarted = true;
    tMotor.bPwmEnabled = true;
    test_IsrStep(&tIdentify, &tMotor, &tSample);
    assert(s_wAbortCount == wAbortBefore);
    assert(tMotor.bPwmEnabled);
    assert(tIdentify.eState == IDENTIFY_STATE_RUNNING);
}

int main(void)
{
    identify_t tIdentify = {0};
    motor_t tMotor = {0};
    identify_status_t tStatus = {0};

    const float fCurrent0 = foc_to_float(
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU) * 10.8f / 2.55f / 3.5f;
    const float fCurrent1 = foc_to_float(
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU) * 13.2f / 2.55f / 3.5f;

    tMotor.tParams.wVoltageBaseMillivolt = 12000U;
    tMotor.tParams.wCurrentBaseMilliamp = 3500U;
    assert(identify_Init(&tIdentify, &s_tIdentifyConfig) == FOC_RESULT_OK);
    s_tIdentifyResult = (identify_result_t){0};
    assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);

    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    tMotor.tOutput.tCurrent.qD = foc_from_float(fCurrent0);
    s_lNowTicks = 101;
    test_RunProgress(&tIdentify, &tMotor);
    identify_GetStatus(&tIdentify, &tStatus);
    assert(tIdentify.tResistance.eState ==
           IDENTIFY_RESISTANCE_STATE_CAPTURE);
    test_RunCapture(&tIdentify, &tMotor, foc_from_float(fCurrent0),
                    foc_from_float(foc_to_float(
                        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU) * 0.9f));
    identify_GetStatus(&tIdentify, &tStatus);
    assert(tIdentify.tResistance.hwCaptureSampleCount ==
           IDENTIFY_RESISTANCE_SAMPLE_COUNT);
    assert(identify_GetStatus(&tIdentify, &tStatus) == FOC_RESULT_OK);
    assert(!tStatus.bResultPending);
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);

    s_lNowTicks = 202;
    test_RunProgress(&tIdentify, &tMotor);
    tMotor.tOutput.tCurrent.qD = foc_from_float(fCurrent1);
    s_lNowTicks = 302;
    test_RunProgress(&tIdentify, &tMotor);
    s_lNowTicks = 403;
    test_RunProgress(&tIdentify, &tMotor);
    test_RunCapture(&tIdentify, &tMotor, foc_from_float(fCurrent1),
                    foc_from_float(foc_to_float(
                        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU) * 1.1f));
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    assert(s_tIdentifyResult.eOperation == IDENTIFY_OPERATION_RESISTANCE);
    /* Verify both captured current averages before checking the result. */
    assert(fabsf(foc_to_float(tIdentify.tResistance.aqAverageCurrent[0U]) -
                 fCurrent0) < 0.001f);
    assert(fabsf(foc_to_float(tIdentify.tResistance.aqAverageCurrent[1U]) -
                 fCurrent1) < 0.001f);
    uint32_t wPhysical = 0U;
    assert(tIdentify.qResistancePu ==
           s_tIdentifyResult.tValue.tResistance.qResistancePu);
    assert(fabsf(foc_to_float(
                     s_tIdentifyResult.tValue.tResistance.qResistancePu) -
                 2.55f * 3.5f / 12.0f) < 0.002f);
    assert(identify_ResultToPhysical(&tIdentify,
        IDENTIFY_OPERATION_RESISTANCE,
        s_tIdentifyResult.tValue.tResistance.qResistancePu,
        &wPhysical) == FOC_RESULT_OK);
    assert(wPhysical > 2540U && wPhysical < 2560U);
    assert(identify_Run(&tIdentify, &tMotor, &s_tIdentifyResult) ==
           FOC_RESULT_BUSY);
    assert(s_tIdentifyResult.eOperation == IDENTIFY_OPERATION_NONE);

    assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);
    test_RunProgress(&tIdentify, &tMotor);
    tMotor.tOutput.tCurrent.qD = FOC_ZERO;
    s_lNowTicks = 503;
    test_RunProgress(&tIdentify, &tMotor);
    s_lNowTicks = 603;
    test_RunProgress(&tIdentify, &tMotor);
    s_lNowTicks = 1103;
    uint32_t wLogBeforeTimeout = s_wIdentifyLogCount;
    assert(identify_Run(&tIdentify, &tMotor, &s_tIdentifyResult) ==
           FOC_RESULT_OUT_OF_RANGE);
    assert(s_tIdentifyResult.eOperation == IDENTIFY_OPERATION_NONE);
#if IDENTIFY_LOG_ENABLE
    assert(s_wIdentifyLogCount == wLogBeforeTimeout + 1U);
    assert(strstr(s_chLastIdentifyLog, "identify R settle") != NULL);
#else
    assert(s_wIdentifyLogCount == wLogBeforeTimeout);
#endif
    identify_Stop(&tIdentify, &tMotor);
    identify_GetStatus(&tIdentify, &tStatus);
    assert(tStatus.eState == IDENTIFY_STATE_IDLE);
    assert(identify_Reset(&tIdentify, &tMotor) == FOC_RESULT_OK);
    assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);
    test_Overcurrent();
    test_ValidCurrent();
    return 0;
}
