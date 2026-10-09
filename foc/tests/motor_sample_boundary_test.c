/** @file motor_sample_boundary_test.c
 * @brief Real Motor/Core sample pairing, bus scaling and wide-current tests. */
#include <assert.h>
#include <math.h>
#include "motor.h"

static foc_current_sample_t raw = {2048U, 2048U, 2048U};
static uint32_t bus = 12000U;
static bool pwm;

foc_result_t foc_SampleCurrent(foc_current_sample_t *out)
{
    *out = raw;
    return FOC_RESULT_OK;
}
uint32_t foc_SampleDcBusRaw(foc_port_adc_channel_e channel)
{
    assert(channel == FOC_PORT_ADC_CHANNEL_DCBUS);
    return bus;
}
foc_result_t foc_SetDuty(const foc_duty_abc_t *duty)
{
    assert(duty != NULL);
    return FOC_RESULT_OK;
}
void foc_port_StartAdcTrigger(void) {}
foc_result_t foc_PwmEnable(void) { pwm = true; return FOC_RESULT_OK; }
foc_result_t foc_PwmSafeStop(void) { pwm = false; return FOC_RESULT_OK; }
bool foc_PwmGetFault(void) { return false; }
foc_result_t foc_PwmClearFault(void) { return FOC_RESULT_OK; }

static void near(foc_scalar_t actual, float expected)
{
    assert(fabsf(foc_to_float(actual) - expected) < 0.0003f);
}

int main(void)
{
    motor_t motor = {0};
    motor_position_sample_t sample = {0};
    motor_electrical_feedback_t feedback = {.bValid = true};
    motor_cfg_t cfg = {
        .tParams = {.chPolePairs = 7U, .wResistanceMilliohm = 2550U,
            .wInductanceDMicroHenry = 860U, .wInductanceQMicroHenry = 860U,
            .wVoltageBaseMillivolt = 12000U,
            .wCurrentBaseMilliamp = FOC_CURRENT_BASE_MILLIAMP},
        .tLimits = {.qMaxSpeedReference = FOC_ONE,
            .qMaxPhaseCurrent = FOC_ONE, .qMaxModulation = FOC_ONE},
        .tCurrentPiParams = {.qOutputMinimum = FOC_NEG_ONE,
            .qOutputMaximum = FOC_ONE, .qIntegratorMinimum = FOC_NEG_ONE,
            .qIntegratorMaximum = FOC_ONE},
        .tSpeedPiParams = {.qOutputMinimum = FOC_NEG_ONE,
            .qOutputMaximum = FOC_ONE, .qIntegratorMinimum = FOC_NEG_ONE,
            .qIntegratorMaximum = FOC_ONE},
        .qElectricalSpeedBaseHz = FOC_SCALAR(400.0f),
        .wControlFrequencyHz = 20000U, .wSpeedLoopFrequencyHz = 1000U,
        .fAdcCalibrationTimeoutSeconds = 0.01f,
        .fAlignTimeSeconds = 0.01f, .qAlignCurrent = FOC_SCALAR(0.05f),
    };
    assert(motor_Init(&motor, &cfg) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_NO_CONTROL);
    assert(motor_Start(&motor, FOC_MODE_VOLTAGE) == FOC_RESULT_OK);
    assert(motor_SetVoltageReference(&motor, FOC_SCALAR(0.8f), FOC_ZERO) ==
           FOC_RESULT_OK);
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_CONTROL_READY);
    near(sample.tVoltageModelAlphaBeta.qAlpha, 0.0f);
    assert(motor_IsrControlStep(&motor, &feedback) == FOC_RESULT_OK);
    /* Actual clipped duty, not the 0.8 modulation command, drives the model. */
    bus = 9600U;
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_CONTROL_READY);
    assert(sample.bVoltageModelValid && sample.bPwmSaturated);
    near(sample.tVoltageModelAlphaBeta.qAlpha, 0.8f * 2.0f / 3.0f);
    assert(motor_IsrControlStep(&motor, &feedback) == FOC_RESULT_OK);
    /* Base is a scale: valid phase current may exceed one PU. */
    int32_t sign = FOC_CURRENT_SAMPLE_INVERTED ? -1 : 1;
    uint32_t du = 5U * FOC_CURRENT_COUNTS_PER_BASE / 4U;
    uint32_t dv = FOC_CURRENT_COUNTS_PER_BASE / 4U;
    uint32_t dw = FOC_CURRENT_COUNTS_PER_BASE / 2U;
    raw = (foc_current_sample_t){2048 + sign * (int32_t)du,
        2048 + sign * (int32_t)dv, 2048 - sign * (int32_t)dw};
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_CONTROL_READY);
    float i = (float)du / FOC_CURRENT_COUNTS_PER_BASE;
    near(sample.tCurrentAlphaBeta.qAlpha, i);
    near(sample.tCurrentAlphaBeta.qBeta,
        (float)(dv + dw) / FOC_CURRENT_COUNTS_PER_BASE / sqrtf(3.0f));
    assert(motor_IsrControlStep(&motor, &feedback) == FOC_RESULT_OK);
    near(motor.tOutput.tCurrent.qD, i);
    bus = FOC_PORT_ADC_SAMPLE_INVALID;
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_CONTROL_READY);
    assert(!sample.bVoltageModelValid);
    near(sample.tVoltageModelAlphaBeta.qAlpha, 0.0f);
    assert(motor_IsrControlStep(&motor, &feedback) == FOC_RESULT_OK);
    bus = 36000U;
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_CONTROL_READY);
    assert(sample.bVoltageModelValid);
    near(sample.tVoltageModelAlphaBeta.qAlpha, 2.0f);
    assert(motor_IsrControlStep(&motor, &feedback) == FOC_RESULT_OK);
    raw.wU = UINT32_MAX;
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_NO_CONTROL);
    assert(motor.eState == MOTOR_STATE_FAULT && !pwm);
    return 0;
}
