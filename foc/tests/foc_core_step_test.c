/****************************************************************************
 * @file    foc_core_step_test.c
 * @brief   Host test for the alpha-beta Core input boundary.
 * @author  Codex
 * @date    2026-09-13
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>

#include "foc_core.h"
#include "foc_numeric.h"

/**
 * @brief Check a scalar result within the backend-independent tolerance.
 * @param qActual Actual scalar value.
 * @param fExpected Expected floating-point value.
 * @return None.
 */
static void test_AssertNear(foc_scalar_t qActual, float fExpected)
{
    const float fActual = foc_to_float(qActual);

    assert(fabsf(fActual - fExpected) < 0.001f);
}

/**
 * @brief Reject invalid gains and non-finite PID bounds.
 * @param None.
 * @return None.
 */
static void test_PidValidation(void)
{
    foc_pid_t tPid = {0};
    foc_pid_params_t tPi = {
        .qOutputMinimum = FOC_NEG_ONE,
        .qOutputMaximum = FOC_ONE,
        .qIntegratorMinimum = FOC_NEG_ONE,
        .qIntegratorMaximum = FOC_ONE,
    };
    foc_gain_t *aptGain[] = {&tPi.tKp, &tPi.tKiTs, &tPi.tKdOverTs};
    for (unsigned int i = 0U; i < 3U; ++i) {
        aptGain[i]->qFraction = FOC_SCALAR(1.1f);
        assert(foc_pid_Init(&tPid, &tPi) == FOC_RESULT_INVALID_ARGUMENT);
        aptGain[i]->qFraction = FOC_ZERO;
    }
#if defined(FOC_NUMERIC_FLOAT)
    foc_scalar_t *apqBound[] = {&tPi.qOutputMinimum, &tPi.qOutputMaximum,
        &tPi.qIntegratorMinimum, &tPi.qIntegratorMaximum};
    for (unsigned int i = 0U; i < 4U; ++i) {
        foc_scalar_t qSaved = *apqBound[i];
        *apqBound[i] = NAN;
        assert(foc_pid_Init(&tPid, &tPi) == FOC_RESULT_INVALID_ARGUMENT);
        *apqBound[i] = INFINITY;
        assert(foc_pid_Init(&tPid, &tPi) == FOC_RESULT_INVALID_ARGUMENT);
        *apqBound[i] = qSaved;
    }
#endif
    assert(foc_pid_Init(&tPid, &tPi) == FOC_RESULT_OK);
}

/** @brief Check arithmetic boundaries retained by the inline numeric layer.
 * @param None. @return None. */
static void test_NumericBoundaries(void)
{
    test_AssertNear(foc_mul_wide(FOC_SCALAR(2.0f), FOC_HALF), 1.0f);
    assert(foc_sat(FOC_SCALAR(2.0f), FOC_NEG_ONE, FOC_ONE) == FOC_ONE);
    assert(foc_sat(FOC_SCALAR(-2.0f), FOC_NEG_ONE, FOC_ONE) == FOC_NEG_ONE);
#if defined(FOC_NUMERIC_FIXED)
    assert(foc_add_sat(INT32_MAX, 1) == INT32_MAX);
    assert(foc_add_sat(INT32_MIN, -1) == INT32_MIN);
    assert(foc_sub_sat(INT32_MIN, 1) == INT32_MIN);
    assert(foc_sub_sat(INT32_MAX, -1) == INT32_MAX);
    assert(foc_abs(INT32_MIN) == INT32_MAX);
    assert(foc_mul_wide(INT32_MAX, FOC_SCALAR(2.0f)) == INT32_MAX);
    assert(foc_mul_wide(INT32_MIN, FOC_SCALAR(2.0f)) == INT32_MIN);
    assert(foc_mul_pu(FOC_SCALAR(2.0f), FOC_HALF) == FOC_HALF);
    assert(foc_mul_pu(-1, FOC_HALF) == -1);
    assert(foc_mul_wide(-1, FOC_HALF) == 0);
#else
    assert(isnan(foc_sat(NAN, FOC_NEG_ONE, FOC_ONE)));
    assert(foc_mul_pu(2.0f, 0.5f) == 1.0f);
    assert(signbit(foc_abs(-0.0f)));
#endif
}

/**
 * @brief Verify Core consumes one precomputed alpha-beta current sample.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    test_PidValidation();
    test_NumericBoundaries();
    foc_core_state_t tState = {0};
    foc_core_output_t tOutput = {0};
    foc_pid_params_t tPi = {
        .qOutputMinimum = FOC_NEG_ONE,
        .qOutputMaximum = FOC_ONE,
        .qIntegratorMinimum = FOC_NEG_ONE,
        .qIntegratorMaximum = FOC_ONE,
    };
    assert(foc_core_Init(NULL, &tPi, FOC_SCALAR(0.5f)) == FOC_RESULT_NULL);
    assert(foc_core_Init(&tState, NULL, FOC_SCALAR(0.5f)) == FOC_RESULT_NULL);
    assert(foc_core_Init(&tState, &tPi, FOC_ZERO) ==
           FOC_RESULT_INVALID_ARGUMENT);
    assert(foc_core_Init(&tState, &tPi, FOC_SCALAR(1.1f)) ==
           FOC_RESULT_INVALID_ARGUMENT);
#if defined(FOC_NUMERIC_FLOAT)
    assert(foc_core_Init(&tState, &tPi, NAN) == FOC_RESULT_INVALID_ARGUMENT);
#endif
    assert(foc_core_Init(&tState, &tPi, FOC_SCALAR(0.5f)) == FOC_RESULT_OK);
    foc_core_Reset(&tState, &tOutput);
    foc_core_command_t tCommand = {0};
    foc_core_input_t tInput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tCommand.eMode = FOC_MODE_VOLTAGE;
    tCommand.tVoltageReference = (foc_dq_t){
        FOC_SCALAR(0.1f), FOC_SCALAR(0.2f)};
    tInput.tCurrentAlphaBeta = (foc_ab_t){
        FOC_SCALAR(0.5f), FOC_SCALAR(0.25f)};
    tInput.tElectricalAngle = (foc_angle_t){0x40000000U};
    tInput.bAngleValid = true;

    eResult = foc_core_step(&tState, &tCommand, &tInput, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tOutput.tCurrent.qD, 0.25f);
    test_AssertNear(tOutput.tCurrent.qQ, -0.5f);
    test_AssertNear(tOutput.tVoltageAlphaBeta.qAlpha, -0.2f);
    test_AssertNear(tOutput.tVoltageAlphaBeta.qBeta, 0.1f);
    assert(!tOutput.bVoltageLimited && !tOutput.bPwmSaturated);
    tInput.tElectricalAngle = (foc_angle_t){0U};
    tCommand.tVoltageReference = (foc_dq_t){FOC_SCALAR(0.5f), FOC_ZERO};
    assert(foc_core_step(&tState, &tCommand, &tInput, &tOutput) ==
           FOC_RESULT_OK);
    assert(tOutput.bVoltageLimited && !tOutput.bPwmSaturated);
    tCommand.tVoltageReference.qD = FOC_SCALAR(0.8f);
    assert(foc_core_step(&tState, &tCommand, &tInput, &tOutput) ==
           FOC_RESULT_OK);
    assert(tOutput.bVoltageLimited && tOutput.bPwmSaturated);
    /* PWM clipping is independent of the configured voltage criterion. */
    assert(foc_core_Init(&tState, &tPi, FOC_SCALAR(0.9f)) == FOC_RESULT_OK);
    assert(foc_core_step(&tState, &tCommand, &tInput, &tOutput) ==
           FOC_RESULT_OK);
    assert(!tOutput.bVoltageLimited && tOutput.bPwmSaturated);
    foc_core_Reset(&tState, &tOutput);
    assert(!tOutput.bVoltageLimited && !tOutput.bPwmSaturated);
    test_AssertNear(tOutput.tDuty.qU, 0.5f);
    assert(foc_core_step(&tState, &tCommand, &tInput, NULL) ==
           FOC_RESULT_NULL);
    return 0;
}
