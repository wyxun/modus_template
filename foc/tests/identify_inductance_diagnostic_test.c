/****************************************************************************
 * @file    identify_inductance_diagnostic_test.c
 * @brief   Host tests for failed inductance-identification diagnostics.
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
    s_tMotorStatus.eState = MOTOR_STATE_RUNNING;
    s_tMotorStatus.eMode = eMode;
    s_tMotorStatus.bPwmEnabled = true;
    ptMotor->eState = MOTOR_STATE_RUNNING;
    ptMotor->bPwmEnabled = true;
    return FOC_RESULT_OK;
}

void motor_Stop(motor_t *ptMotor)
{
    s_tMotorStatus.eState = MOTOR_STATE_IDLE;
    s_tMotorStatus.bPwmEnabled = false;
    ptMotor->eState = MOTOR_STATE_IDLE;
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
    const foc_dq_t *ptVoltageCommand)
{
    (void)ptMotor;
    (void)ptVoltageCommand;
    return FOC_RESULT_OK;
}

void motor_IdentificationAbortIsr(motor_t *ptMotor,
                                  motor_fault_e eFault)
{
    (void)ptMotor;
    (void)eFault;
}

foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    (void)ptMotor;
    *ptStatus = s_tMotorStatus;
    return FOC_RESULT_OK;
}

static void test_InductanceFailure(
    foc_scalar_t qPositiveLast,
    identify_inductance_failure_t eExpectedFailure)
{
    identify_t tIdentify = {0};
    identify_inductance_diagnostic_t tDiagnostic = {0};
    identify_inductance_cfg_t tConfig = {
        .wInjectionFrequencyHz = 1000U,
        .hwCaptureDelayCycles = 1U,
        .hwCaptureSampleCount = 2U,
        .hwHalfCycleCount = 4U,
        .qModulationAmplitude = FOC_SCALAR(0.1f),
        .qMaxIdentificationCurrent = FOC_SCALAR(0.5f),
        .qMinCurrentDelta = FOC_SCALAR(0.01f),
        .qMaxElectricalSpeedPu = FOC_SCALAR(0.01f),
        .hwMotionFaultCycles = 1U,
    };
    identify_isr_sample_t tSample = {
        .qCurrentD = FOC_ZERO,
        .qCurrentQ = FOC_ZERO,
        .qElectricalSpeedPu = FOC_ZERO,
        .wDcBusMillivolt = 12000U,
        .bDcBusValid = true,
        .bMotorFault = false,
        .bAngleValid = true,
        .bPwmSaturated = false,
    };
    motor_t tMotor = {0};
    uint8_t chHalf = 0U;
    uint8_t chCycle = 0U;

    tMotor.wCurrentBaseMilliamp = 3500U;
    tMotor.tParams.wResistanceMilliohm = 1000U;
    tMotor.tLimits.qMaxModulation = FOC_ONE;
    tMotor.tLimits.qMaxPhaseCurrent = FOC_ONE;
    assert(identify_Init(&tIdentify) == FOC_RESULT_OK);
    assert(identify_StartInductance(&tIdentify, &tConfig) ==
           FOC_RESULT_OK);
    assert(identify_Run(&tIdentify, &tMotor) == FOC_RESULT_BUSY);

    for (chHalf = 0U; chHalf < 4U; chHalf++) {
        for (chCycle = 0U; chCycle < 10U; chCycle++) {
            tSample.qCurrentD = FOC_ZERO;
            if ((chHalf & 1U) == 0U && chCycle == 2U) {
                tSample.qCurrentD = qPositiveLast;
            } else if ((chHalf & 1U) != 0U && chCycle == 1U) {
                tSample.qCurrentD = FOC_SCALAR(0.01f);
            }
            identify_IsrStep(&tIdentify, &tMotor, &tSample);
        }
    }
    assert(identify_Run(&tIdentify, &tMotor) ==
           FOC_RESULT_OUT_OF_RANGE);
    assert(identify_GetInductanceDiagnostic(&tIdentify, &tDiagnostic) ==
           FOC_RESULT_OK);
    assert(tDiagnostic.bValid);
    assert(tDiagnostic.wResistanceMilliohm == 1000U);
    assert(tDiagnostic.qMinimumDeltaPu == FOC_SCALAR(0.01f));
    assert(tDiagnostic.tPositive.eFailure == eExpectedFailure);
    assert(tDiagnostic.tPositive.wAverageBusMillivolt == 12000U);
    assert(tDiagnostic.tPositive.lCommandVoltageMillivolt == 1200);
    if (eExpectedFailure ==
        IDENTIFY_INDUCTANCE_FAILURE_DELTA_TOO_SMALL) {
        assert(fabsf(foc_to_float(
                         tDiagnostic.tPositive.qDeltaCurrentPu) - 0.004f) <
               0.001f);
        assert(tDiagnostic.tPositive.lDeltaCurrentAdcCodeEq >= 5);
        assert(tDiagnostic.tPositive.lDeltaCurrentAdcCodeEq <= 6);
        assert(tDiagnostic.tPositive.lNetVoltageMillivolt > 1100);
        assert(tDiagnostic.tPositive.lNetVoltageMillivolt < 1200);
    } else {
        assert(tDiagnostic.tPositive.qDeltaCurrentPu < FOC_ZERO);
        assert(tDiagnostic.tPositive.lDeltaCurrentAdcCodeEq < 0);
        assert(tDiagnostic.tPositive.lNetVoltageMillivolt > 1200);
    }
    assert(tDiagnostic.tNegative.eFailure ==
           IDENTIFY_INDUCTANCE_FAILURE_NONE);
    assert(tDiagnostic.tNegative.qDeltaCurrentPu < FOC_ZERO);
    assert(tDiagnostic.tNegative.lCommandVoltageMillivolt == -1200);
    assert(tDiagnostic.tNegative.lNetVoltageMillivolt < -1200);
    assert(identify_GetInductanceDiagnostic(NULL, &tDiagnostic) ==
           FOC_RESULT_NULL);
    assert(identify_GetInductanceDiagnostic(&tIdentify, NULL) ==
           FOC_RESULT_NULL);
}

int main(void)
{
    test_InductanceFailure(
        FOC_SCALAR(0.004f),
        IDENTIFY_INDUCTANCE_FAILURE_DELTA_TOO_SMALL);
    test_InductanceFailure(
        FOC_SCALAR(-0.02f),
        IDENTIFY_INDUCTANCE_FAILURE_VOLTAGE_SLOPE_SIGN);
    return 0;
}
