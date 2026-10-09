/****************************************************************************
 * @file    foc_core_step_test.c
 * @brief   Host test for the alpha-beta Core input boundary.
 * @author  Codex
 * @date    2026-09-13
 ****************************************************************************/

#include <assert.h>
#include <math.h>
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
 * @brief Verify Core consumes one precomputed alpha-beta current sample.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
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
    test_AssertNear(tOutput.tCurrentAlphaBeta.qAlpha, 0.5f);
    test_AssertNear(tOutput.tCurrentAlphaBeta.qBeta, 0.25f);
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
