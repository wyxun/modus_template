/** @file motor_sample_boundary_test.c
 * @brief Real Motor/Core sample pairing, bus scaling and wide-current tests. */
#include <assert.h>
#include <math.h>
#include "../motor/motor.c"

#if FOC_CURRENT_COUNTS_PER_BASE == 476U
#define TEST_CURRENT_ZERO_CODE 10236U
#else
#define TEST_CURRENT_ZERO_CODE 2048U
#endif
static foc_current_sample_t raw = {
    TEST_CURRENT_ZERO_CODE, TEST_CURRENT_ZERO_CODE, TEST_CURRENT_ZERO_CODE};
static uint32_t bus = 12000U;
static bool pwm;
static uint32_t busReads;
static bool currentReadFailed;

foc_result_t foc_SampleCurrent(foc_current_sample_t *out)
{
    if (currentReadFailed) { return FOC_RESULT_SAFETY; }
    *out = raw;
    return FOC_RESULT_OK;
}
uint32_t foc_SampleDcBusRaw(foc_port_adc_channel_e channel)
{
    assert(channel == FOC_PORT_ADC_CHANNEL_DCBUS);
    busReads++;
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

/** @brief Exercise real calibration with full-width raw samples.
 * @param motor Motor in ADC_CAL. @return None. */
static void test_Calibrate(motor_t *motor)
{
    motor_position_sample_t sample = {0};
    uint64_t sum = 0U;

    assert(!pwm && !motor->tCalib.bIsCalibrated);
    assert(motor_Start(motor, FOC_MODE_VOLTAGE) == FOC_RESULT_BUSY);
    for (uint32_t n = 0U; n < FOC_OFFSET_CALIB_TIMES; n++) {
        uint32_t code = TEST_CURRENT_ZERO_CODE + (n % 4U == 3U ? 1U : 0U);
        raw = (foc_current_sample_t){code, code, code};
        sum += code;
        assert(motor_IsrPrepare(motor, &sample) == MOTOR_ISR_NO_CONTROL);
        assert(!pwm);
    }
    assert(motor->eState == MOTOR_STATE_IDLE);
    assert(motor->tCalib.bIsCalibrated);
    assert(motor->tCalib.hwSampleCount == FOC_OFFSET_CALIB_TIMES);
    assert(motor->tCalib.ullSumU == sum);
    assert(motor->tCalib.ullSumV == sum);
    assert(motor->tCalib.ullSumW == sum);
    assert(motor->tCalib.wOffsetU == sum / FOC_OFFSET_CALIB_TIMES);
    assert(motor->tCalib.wOffsetV == sum / FOC_OFFSET_CALIB_TIMES);
    assert(motor->tCalib.wOffsetW == sum / FOC_OFFSET_CALIB_TIMES);
    raw = (foc_current_sample_t){TEST_CURRENT_ZERO_CODE,
        TEST_CURRENT_ZERO_CODE, TEST_CURRENT_ZERO_CODE};
}

/** @brief Fixed-bus identification ignores later ADC publication windows.
 * @param cfg Initialized control profile. @return None. */
static void test_IdentificationBusSnapshot(const motor_cfg_t *cfg)
{
    motor_t motor = {0};
    motor_position_sample_t sample = {0};
    uint32_t reads = 0U;

    raw = (foc_current_sample_t){TEST_CURRENT_ZERO_CODE,
        TEST_CURRENT_ZERO_CODE, TEST_CURRENT_ZERO_CODE};
    bus = 12000U;
    assert(motor_Init(&motor, cfg) == FOC_RESULT_OK);
    test_Calibrate(&motor);
    bus = FOC_PORT_ADC_SAMPLE_INVALID;
    assert(motor_StartIdentification(&motor) == FOC_RESULT_SAFETY);
    assert(!pwm);
    bus = 0U;
    assert(motor_StartIdentification(&motor) == FOC_RESULT_OUT_OF_RANGE);
    bus = FOC_DCBUS_MAX_MILLIVOLT + 1U;
    assert(motor_StartIdentification(&motor) == FOC_RESULT_OUT_OF_RANGE);
    bus = 12000U;
    reads = busReads;
    assert(motor_StartIdentification(&motor) == FOC_RESULT_OK);
    assert(busReads == reads + 1U);
    near(motor.qIdentificationDcBusPu, 1.0f);
    /* First ISR runs after the foreground ADC enters its update window. */
    bus = FOC_PORT_ADC_SAMPLE_INVALID;
    for (uint32_t step = 0U; step < motor.tAlign.wTargetSteps; step++) {
        assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_NO_CONTROL);
    }
    assert(motor_IsrPrepare(&motor, &sample) ==
           MOTOR_ISR_IDENTIFICATION_READY);
    assert(sample.bVoltageModelValid);
    assert(motor_IsrControlStep(&motor, NULL) == FOC_RESULT_OK);
    assert(motor_IdentificationApplyIsr(&motor,
        &(foc_ab_t){FOC_SCALAR(0.1f), FOC_ZERO}) == FOC_RESULT_OK);
    /* Reference reaches PWM on the next control step. */
    assert(motor_IsrPrepare(&motor, &sample) ==
           MOTOR_ISR_IDENTIFICATION_READY);
    assert(motor_IsrControlStep(&motor, NULL) == FOC_RESULT_OK);
    assert(motor_IsrPrepare(&motor, &sample) ==
           MOTOR_ISR_IDENTIFICATION_READY);
    assert(sample.bVoltageModelValid);
    near(sample.tVoltageModelAlphaBeta.qAlpha, 0.1f);
    assert(busReads == reads + 1U);
    assert(motor_IsrControlStep(&motor, NULL) == FOC_RESULT_OK);
    motor_Stop(&motor);
    near(motor.qIdentificationDcBusPu, 0.0f);
    assert(motor_StartIdentification(&motor) == FOC_RESULT_SAFETY);
    assert(!pwm);
    /* A new session must latch anew, not reuse the prior session's value. */
    bus = 9600U;
    assert(motor_StartIdentification(&motor) == FOC_RESULT_OK);
    near(motor.qIdentificationDcBusPu, 0.8f);
    /* Holding the bus snapshot must not mask a real current-sample fault. */
    raw.wU = UINT32_MAX;
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_NO_CONTROL);
    assert(motor.eState == MOTOR_STATE_FAULT && !pwm);
    near(motor.qIdentificationDcBusPu, 0.0f);
}

/** @brief Check narrowing against the original wide calculation.
 * @param None. @return None. */
static void test_CurrentConversion(void)
{
    const uint32_t values[] = {0U, 1U, 2048U, 4095U, 8192U, 8193U,
        10236U, 16380U, 65535U,
        7798783U, 7798784U, INT32_MAX, 2147483648U, UINT32_MAX - 1U,
        UINT32_MAX};
    for (unsigned i = 0U; i < sizeof(values) / sizeof(values[0]); i++) {
        for (unsigned j = 0U; j < sizeof(values) / sizeof(values[0]); j++) {
            int64_t delta = (int64_t)values[j] - (int64_t)values[i];
#if !FOC_CURRENT_SAMPLE_INVERTED
            delta = -delta;
#endif
            int64_t scaled = delta * FOC_Q_SCALE;
            bool valid = values[i] != UINT32_MAX &&
                scaled < ((int64_t)INT32_MAX + 1) *
                    FOC_CURRENT_COUNTS_PER_BASE &&
                scaled > (int64_t)INT32_MIN * FOC_CURRENT_COUNTS_PER_BASE;
            foc_scalar_t actual = FOC_ZERO;
            assert(_motor_NormalizeCurrent(values[i], values[j], &actual) ==
                   valid);
            if (valid) {
#if defined(FOC_NUMERIC_FIXED)
                assert(actual == scaled / FOC_CURRENT_COUNTS_PER_BASE);
#else
                assert(actual == (float)delta / FOC_CURRENT_COUNTS_PER_BASE);
#endif
            }
        }
    }
    if (FOC_CURRENT_COUNTS_PER_BASE == 476U) {
        for (int32_t delta = -119; delta <= 119; delta++) {
            foc_scalar_t actual = FOC_ZERO;
            int32_t signedDelta = FOC_CURRENT_SAMPLE_INVERTED ?
                                  -delta : delta;
            assert(_motor_NormalizeCurrent(
                (uint32_t)(10236 + 4 * delta), 10236U, &actual));
#if defined(FOC_NUMERIC_FIXED)
            assert(actual == (int64_t)signedDelta * FOC_Q_SCALE / 119);
#else
            assert(actual == (float)signedDelta / 119.0f);
#endif
        }
        for (uint32_t low = 1U; low <= 3U; low++) {
            foc_scalar_t actual = FOC_ZERO;
            assert(_motor_NormalizeCurrent(10236U + low, 10236U, &actual));
            assert(actual != FOC_ZERO);
        }
    }
}

int main(void)
{
    test_CurrentConversion();
    if (FOC_CURRENT_COUNTS_PER_BASE > 1000U) {
        return 0;
    }
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
    currentReadFailed = true;
    assert(motor_IsrPrepare(&motor, &sample) == MOTOR_ISR_NO_CONTROL);
    assert(motor.eState == MOTOR_STATE_FAULT && !pwm);
    assert((motor.wFaults & MOTOR_FAULT_ADC_CAL) != 0U);
    currentReadFailed = false;
    assert(motor_ClearFault(&motor) == FOC_RESULT_OK);
    assert(motor.tCalib.hwSampleCount == 0U && motor.tCalib.ullSumU == 0U);
    test_Calibrate(&motor);
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
    raw = (foc_current_sample_t){
        (uint32_t)((int32_t)TEST_CURRENT_ZERO_CODE + sign * (int32_t)du),
        (uint32_t)((int32_t)TEST_CURRENT_ZERO_CODE + sign * (int32_t)dv),
        (uint32_t)((int32_t)TEST_CURRENT_ZERO_CODE - sign * (int32_t)dw)};
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
    test_IdentificationBusSnapshot(&cfg);
    return 0;
}
