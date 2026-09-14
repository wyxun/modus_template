/****************************************************************************
 * @file    foc_smo_test.c
 * @brief   Host tests for the normalized Sguan-style SMO/PLL adapter.
 * @author  Codex
 * @date    2026-09-13
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "foc_smo.h"
#include "motor.h"

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
static void test_BuildConfig(motor_params_t *ptMotorParams,
                             foc_smo_cfg_t *ptConfig)
{
    *ptMotorParams = (motor_params_t){
        .chPolePairs = 7U,
        .wResistanceMilliohm = 500U,
        .wInductanceDMicroHenry = 1000U,
        .wInductanceQMicroHenry = 1000U,
        .wVoltageBaseMillivolt = 12000U,
        .wCurrentBaseMilliamp = 7000U,
    };
    *ptConfig = (foc_smo_cfg_t){
        .wSamplePeriodNanoseconds = 50000U,
        .wBemfCutoffRadiansPerSecond = 10000U,
        .wSlidingGainMillivolt = 3500U,
        .wPllKpRadiansPerSecondPerVolt = 650U,
        .wPllKiRadiansPerSecondSquaredPerVolt = 210000U,
        .qCurrentEstimateLimit = FOC_ONE,
    };
}

/**
 * @brief Verify Init validation, normalized coefficients, step, and Reset.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    motor_params_t tMotorParams = {0};
    foc_smo_cfg_t tConfig = {0};
    foc_smo_t tSmo = {0};
    foc_smo_output_t tOutput = {0};
    foc_ab_t tCurrent = {FOC_ZERO, FOC_ZERO};
    foc_ab_t tVoltage = {FOC_SCALAR(0.1f), FOC_ZERO};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_smo_Init(NULL, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_NULL);
    test_BuildConfig(&tMotorParams, &tConfig);
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);

    eResult = foc_smo_Step(NULL, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_NULL);
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tSmo.tAxis[0].qCurrentEstimate, 0.0042857f, 0.0002f);
    test_AssertNear(tSmo.tAxis[0].qBemf, 0.0583333f, 0.001f);
    assert(!tOutput.bValid);
    assert(tOutput.tElectricalAngle.wBam32 != 0U);
    test_AssertNear(tOutput.qElectricalSpeedTurnsPerSecond,
                    -511.0f, 2.0f);

    foc_smo_Reset(&tSmo);
    assert(tSmo.tAxis[0].qCurrentEstimate == FOC_ZERO);
    assert(tSmo.tAxis[0].qBemf == FOC_ZERO);
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tSmo.tAxis[0].qCurrentEstimate, 0.0042857f, 0.0002f);

    tMotorParams.wInductanceQMicroHenry = 1500U;
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    tSmo.tAxis[0].qCurrentEstimate = FOC_SCALAR(0.2f);
    tSmo.tAxis[1].qCurrentEstimate = FOC_SCALAR(0.1f);
    tSmo.qPllMechanicalSpeed = FOC_SCALAR(0.5f / 7.0f);
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tSmo.tAxis[1].qCurrentEstimate,
                    0.12375f, 0.0003f);

    tMotorParams.wInductanceQMicroHenry = 1000U;
    tConfig.qMinimumBemf = FOC_SCALAR(0.01f);
    tConfig.qMaximumPhaseError = FOC_SCALAR(0.2f);
    tConfig.qMaximumElectricalSpeed = FOC_SCALAR(3000.0f);
    tConfig.hwQualificationSamples = 2U;
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(!tOutput.bValid);
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(tOutput.bValid);
    tSmo.tCfg.qMaximumPhaseError = FOC_SCALAR(0.0001f);
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(!tOutput.bValid);

    tMotorParams.chPolePairs = 1U;
    tConfig.wSamplePeriodNanoseconds = 10000U;
    tConfig.wBemfCutoffRadiansPerSecond = 1U;
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
#if defined(FOC_NUMERIC_FIXED)
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
#else
    assert(eResult == FOC_RESULT_OK);
#endif

    tConfig.wSamplePeriodNanoseconds = 50000U;
    tConfig.wBemfCutoffRadiansPerSecond = 10000U;
    tConfig.qMinimumBemf = FOC_ZERO;
    tConfig.qMaximumPhaseError = FOC_ZERO;
    tConfig.qMinimumElectricalSpeed = FOC_ZERO;
    tConfig.qMaximumElectricalSpeed = FOC_ZERO;
    tConfig.hwQualificationSamples = 0U;
    tConfig.qCurrentEstimateLimit = FOC_SCALAR(0.01f);
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    tSmo.tPllMechanicalAngle = (foc_angle_t){0xFFFFFFF0U};
    tSmo.qPllMechanicalSpeed = FOC_SCALAR(0.5f);
    tSmo.qPreviousPllMechanicalSpeed = FOC_SCALAR(0.5f);
    tVoltage = (foc_ab_t){FOC_ZERO, FOC_ZERO};
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(tOutput.tElectricalAngle.wBam32 < 0x80000000U);

    foc_smo_Reset(&tSmo);
    tVoltage = (foc_ab_t){FOC_ONE, FOC_ZERO};
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(tSmo.tAxis[0].bIntegratorFrozen);
    test_AssertNear(tSmo.tAxis[0].qCurrentEstimate, 0.01f, 0.0001f);
    tVoltage.qAlpha = FOC_NEG_ONE;
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(!tSmo.tAxis[0].bIntegratorFrozen);
    test_AssertNear(tSmo.tAxis[0].qCurrentEstimate, 0.01f, 0.0001f);
    eResult = foc_smo_Step(&tSmo, &tCurrent, &tVoltage, &tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(tSmo.tAxis[0].bIntegratorFrozen);
    test_AssertNear(tSmo.tAxis[0].qCurrentEstimate, -0.01f, 0.0001f);

    tMotorParams.wCurrentBaseMilliamp = 0U;
    eResult = foc_smo_Init(&tSmo, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
    return 0;
}
