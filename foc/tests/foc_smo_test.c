/****************************************************************************
 * @file    foc_smo_test.c
 * @brief   Host tests for the normalized simple SMO.
 * @author  Codex
 * @date    2026-09-15
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "foc_smo.h"

/**
 * @brief Compare a backend scalar with a floating-point expectation.
 * @param qActual Actual scalar value.
 * @param fExpected Expected value.
 * @param fTolerance Maximum absolute error.
 * @return None.
 */
static void test_AssertNear(foc_scalar_t qActual,
                            float fExpected,
                            float fTolerance)
{
    assert(fabsf(foc_to_float(qActual) - fExpected) < fTolerance);
}
/**
 * @brief Build the provisional 12 V / 7 A theoretical test configuration.
 * @param ptMotorParams Output motor parameters.
 * @param ptConfig Output SMO configuration.
 * @return None.
 */
static void test_BuildConfig(foc_electrical_model_t *ptMotorParams,
                             foc_smo_cfg_t *ptConfig)
{
    *ptMotorParams = (foc_electrical_model_t){
        .wResistanceMilliohm = 500U,
        .wInductanceDMicroHenry = 1000U,
        .wInductanceQMicroHenry = 1000U,
        .wVoltageBaseMillivolt = 12000U,
        .wCurrentBaseMilliamp = 7000U,
    };
    *ptConfig = (foc_smo_cfg_t){
        .wSampleFrequencyHz = 20000U,
        .qElectricalSpeedBaseHz = FOC_SCALAR(200.0f),
        .wBemfCutoffRadiansPerSecond = 10000U,
        .wSlidingGainMillivolt = 3500U,
        .qCurrentEstimateLimit = FOC_ONE,
    };
}

/** @brief Exercise the boundary at zero, inside and outside both edges.
 * @param None. @return None. */
static void test_BoundarySwitch(void)
{
    const float errors[] = {0.0f, 0.025f, -0.025f, 0.05f, -0.05f,
                            0.1f, -0.1f, 0.2f, -0.2f};
    foc_electrical_model_t model = {0};
    foc_smo_cfg_t cfg = {0};
    foc_smo_t smo = {0};
    foc_smo_output_t out = {0};
    foc_ab_t current = {0};
    foc_ab_t voltage = {0};
    const foc_smo_input_t input = {&current, &voltage};

    test_BuildConfig(&model, &cfg);
    assert(foc_smo_Init(&smo, &model, &cfg) == FOC_RESULT_OK);
    for (unsigned n = 0U; n < sizeof(errors) / sizeof(errors[0]); n++) {
        float normalized = fmaxf(-1.0f, fminf(1.0f, errors[n] / 0.1f));
        float expected = foc_to_float(smo.tExec.h) * normalized;

        foc_smo_Reset(&smo);
        current.qAlpha = foc_from_float(-errors[n]);
        current.qBeta = foc_from_float(errors[n]);
        assert(foc_smo_Step(&smo, &input, &out) == FOC_RESULT_OK);
        test_AssertNear(smo.tAxis[0].z_prev, expected, 0.0003f);
        test_AssertNear(smo.tAxis[1].z_prev, -expected, 0.0003f);
        test_AssertNear(smo.tAxis[0].e_hat,
                       expected * foc_to_float(smo.tExec.b0), 0.0003f);
        assert(foc_abs(smo.tAxis[0].z_prev) <= smo.tExec.h);
        assert(out.bValid == (errors[n] != 0.0f));
    }
}

/**
 * @brief Verify the simple SMO model, filter, angle and reset behavior.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    foc_electrical_model_t tMotorParams = {0};
    foc_smo_cfg_t tConfig = {0};
    foc_smo_t tSmo = {0};
    foc_smo_output_t tOutput = {0};
    foc_ab_t tCurrent = {FOC_ZERO, FOC_ZERO};
    foc_ab_t tVoltage = {FOC_SCALAR(0.1f), FOC_ZERO};
    const foc_smo_input_t tInput = {&tCurrent, &tVoltage};
    foc_result_t eResult = FOC_RESULT_OK;

    test_BoundarySwitch();

    eResult = foc_smo_Init(NULL, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_NULL);

    test_BuildConfig(&tMotorParams, &tConfig);
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tSmo.tExec.ku, 0.0857143f, 0.0003f);
    test_AssertNear(tSmo.tExec.kr, 0.025f, 0.0003f);
    test_AssertNear(tSmo.tExec.b0, 0.2f, 0.0003f);
    test_AssertNear(tSmo.tExec.a1, -0.6f, 0.0003f);
    test_AssertNear(tSmo.tExec.h, 0.291667f, 0.0003f);

    eResult = foc_smo_Step(NULL, &tInput, &tOutput);
    assert(eResult == FOC_RESULT_NULL);
    eResult = foc_smo_Step(&tSmo, &tInput, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tSmo.tAxis[0].i_hat,
                    0.0042857f, 0.0003f);
    test_AssertNear(tSmo.tAxis[0].e_hat, 0.0025f, 0.0003f);
    assert(tOutput.bValid);
    assert(tOutput.tElectricalAngle.wBam32 > 0x80000000U);
    assert(tOutput.qElectricalSpeedPu == FOC_ZERO);

    foc_smo_Reset(&tSmo);
    assert(tSmo.tAxis[0].i_hat == FOC_ZERO);
    assert(tSmo.tAxis[0].e_hat == FOC_ZERO);
    assert(!tSmo.tAxis[0].frozen);
    assert(!tSmo.bHasPreviousElectricalAngle);

    tConfig.qCurrentEstimateLimit = FOC_SCALAR(0.01f);
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    tVoltage = (foc_ab_t){FOC_ONE, FOC_ZERO};
    eResult = foc_smo_Step(&tSmo, &tInput, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(tSmo.tAxis[0].frozen);
    test_AssertNear(tSmo.tAxis[0].i_hat, 0.01f, 0.0003f);

    tVoltage.qAlpha = FOC_NEG_ONE;
    eResult = foc_smo_Step(&tSmo, &tInput, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(!tSmo.tAxis[0].frozen);
    test_AssertNear(tSmo.tAxis[0].i_hat, 0.01f, 0.0003f);

    tMotorParams.wInductanceQMicroHenry = 1500U;
    tConfig.qCurrentEstimateLimit = FOC_ONE;
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_DISABLED);
    tMotorParams.wInductanceQMicroHenry = 1000U;

    tMotorParams.wCurrentBaseMilliamp = 0U;
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
    test_BuildConfig(&tMotorParams, &tConfig);
    tConfig.qElectricalSpeedBaseHz = FOC_ZERO;
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
#if defined(FOC_NUMERIC_FLOAT)
    test_BuildConfig(&tMotorParams, &tConfig);
    tConfig.qElectricalSpeedBaseHz = NAN;
    assert(foc_smo_Init(&tSmo, &tMotorParams, &tConfig) ==
           FOC_RESULT_INVALID_ARGUMENT);
    test_BuildConfig(&tMotorParams, &tConfig);
    assert(foc_smo_Init(&tSmo, &tMotorParams, &tConfig) == FOC_RESULT_OK);
    tCurrent.qAlpha = NAN;
    assert(foc_smo_Step(&tSmo, &tInput, &tOutput) ==
           FOC_RESULT_INVALID_ARGUMENT);
    assert(!tOutput.bValid);
    assert(tSmo.tAxis[0].i_hat == FOC_ZERO);
#endif
    return 0;
}
