/****************************************************************************
 * @file    foc_smo_switch_feedback_test.c
 * @brief   Verify the SMO current model uses prior sliding feedback.
 * @author  Codex
 * @date    2026-09-24
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "foc_smo.h"

/**
 * @brief Distinguish raw sliding feedback from filtered BEMF feedback.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    foc_electrical_model_t tParams = {
        .wResistanceMilliohm = 2740U,
        .wInductanceDMicroHenry = 800U,
        .wInductanceQMicroHenry = 800U,
        .wVoltageBaseMillivolt = 12000U,
        .wCurrentBaseMilliamp = 3500U,
    };
    foc_smo_cfg_t tConfig = {
        .wSampleFrequencyHz = 20000U,
        .qElectricalSpeedBaseHz = FOC_SCALAR(200.0f),
        .wBemfCutoffRadiansPerSecond = 10000U,
        .wSlidingGainMillivolt = 5000U,
        .qCurrentEstimateLimit = FOC_ONE,
    };
    foc_smo_t tSmo = {0};
    foc_smo_output_t tOutput = {0};
    foc_ab_t tCurrent = {FOC_ZERO, FOC_ZERO};
    foc_ab_t tVoltage = {FOC_ZERO, FOC_ZERO};
    const foc_smo_input_t tInput = {&tCurrent, &tVoltage};
    foc_result_t eResult = FOC_RESULT_OK;
    float fExpectedCurrent = 0.0f;

    eResult = foc_smo_Init(&tSmo, &tParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    tSmo.tAxis[0].z_prev =
        tSmo.tExec.h;
    tSmo.tAxis[0].e_hat = FOC_SCALAR(0.1f);
    fExpectedCurrent = -0.5f *
        foc_to_float(tSmo.tExec.ku) *
        foc_to_float(tSmo.tExec.h);

    eResult = foc_smo_Step(&tSmo, &tInput,
                           &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(
               tSmo.tAxis[0].i_hat) -
                 fExpectedCurrent) < 0.001f);
    assert(fabsf(foc_to_float(tSmo.tAxis[0].e_hat) -
                 0.06f) < 0.001f);
    assert(tOutput.tEstimate.bValid);
    return 0;
}
