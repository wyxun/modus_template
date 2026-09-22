/****************************************************************************
 * @file    identify_resistance_test.c
 * @brief   Host test for D/Q-axis resistance identification.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "identify.h"
#include "perf_counter.h"

static int64_t s_lNowTicks = 1;
static motor_status_t s_tMotorStatus = {
    .eState = MOTOR_STATE_IDLE,
    .wFaults = MOTOR_FAULT_NONE,
    .eMode = FOC_MODE_CURRENT,
    .bPwmEnabled = false,
    .bElectricalZeroValid = true,
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

static void test_RunCapture(identify_t *ptIdentify,
                            foc_scalar_t qCurrentD)
{
    uint32_t wIsr = 0U;
    uint32_t wTotalIsr = IDENTIFY_RESISTANCE_SAMPLE_COUNT *
                         IDENTIFY_RESISTANCE_ISR_PER_SAMPLE;

    for (wIsr = 0U; wIsr < wTotalIsr; wIsr++) {
        identify_driver_IsrStep(ptIdentify, qCurrentD);
    }
}

static void test_RunProgress(identify_t *ptIdentify,
                             motor_t *ptMotor)
{
    foc_result_t eResult = identify_Run(ptIdentify, ptMotor);

    assert(eResult == FOC_RESULT_BUSY || eResult == FOC_RESULT_OK);
}

int main(void)
{
    identify_t tIdentify = {0};
    motor_t tMotor = {0};
    identify_status_t tStatus = {0};
    uint32_t wResistanceMilliohm = 0U;

    tMotor.tParams.wVoltageBaseMillivolt = 24000U;
    tMotor.wCurrentBaseMilliamp = 7000U;
    assert(identify_Init(&tIdentify) == FOC_RESULT_OK);
    assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);

    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    tMotor.tCore.tCurrent.qD = FOC_SCALAR(0.10f);
    s_lNowTicks = 101;
    test_RunProgress(&tIdentify, &tMotor);
    identify_GetStatus(&tIdentify, &tStatus);
    assert(tStatus.eState == IDENTIFY_STATE_RESISTANCE_CAPTURE);
    test_RunCapture(&tIdentify, FOC_SCALAR(0.10f));
    identify_GetStatus(&tIdentify, &tStatus);
    assert(tStatus.hwCaptureSampleCount ==
           IDENTIFY_RESISTANCE_SAMPLE_COUNT);
    assert(identify_GetResistance(&tIdentify,
                                  &wResistanceMilliohm) ==
           FOC_RESULT_BUSY);
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);

    s_lNowTicks = 202;
    test_RunProgress(&tIdentify, &tMotor);
    tMotor.tCore.tCurrent.qD = FOC_SCALAR(0.20f);
    s_lNowTicks = 302;
    test_RunProgress(&tIdentify, &tMotor);
    s_lNowTicks = 403;
    test_RunProgress(&tIdentify, &tMotor);
    test_RunCapture(&tIdentify, FOC_SCALAR(0.20f));
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    test_RunProgress(&tIdentify, &tMotor);
    assert(identify_GetResistance(&tIdentify,
                                  &wResistanceMilliohm) == FOC_RESULT_OK);
    /* Verify both captured current averages before checking the result. */
    assert(fabsf(foc_to_float(tIdentify.tResistance.aqAverageCurrent[0U]) -
                 0.10f) < 0.001f);
    assert(fabsf(foc_to_float(tIdentify.tResistance.aqAverageCurrent[1U]) -
                 0.20f) < 0.001f);
    assert(wResistanceMilliohm > 3400U);
    assert(wResistanceMilliohm < 3450U);
    assert(identify_GetResistance(&tIdentify,
                                  &wResistanceMilliohm) ==
           FOC_RESULT_BUSY);

    assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);
    test_RunProgress(&tIdentify, &tMotor);
    tMotor.tCore.tCurrent.qD = FOC_ZERO;
    s_lNowTicks = 503;
    test_RunProgress(&tIdentify, &tMotor);
    s_lNowTicks = 603;
    test_RunProgress(&tIdentify, &tMotor);
    s_lNowTicks = 1103;
    assert(identify_Run(&tIdentify, &tMotor) == FOC_RESULT_OUT_OF_RANGE);
    identify_Stop(&tIdentify, &tMotor);
    identify_GetStatus(&tIdentify, &tStatus);
    assert(tStatus.eState == IDENTIFY_STATE_IDLE);
    assert(identify_StartResistance(&tIdentify) == FOC_RESULT_OK);
    return 0;
}
