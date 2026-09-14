/****************************************************************************
 * @file    foc_core_step_test.c
 * @brief   Host test for the alpha-beta Core input boundary.
 * @author  Codex
 * @date    2026-09-13
 ****************************************************************************/

#include <assert.h>
#include <math.h>

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

    eResult = foc_core_step(&tState, &tCommand, &tInput);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tState.tCurrentAlphaBeta.qAlpha, 0.5f);
    test_AssertNear(tState.tCurrentAlphaBeta.qBeta, 0.25f);
    test_AssertNear(tState.tCurrent.qD, 0.25f);
    test_AssertNear(tState.tCurrent.qQ, -0.5f);
    test_AssertNear(tState.tVoltageAlphaBeta.qAlpha, -0.2f);
    test_AssertNear(tState.tVoltageAlphaBeta.qBeta, 0.1f);
    return 0;
}
