/****************************************************************************
 * @file    foc_identify_test.c
 * @brief   Host tests for the independent FOC parameter identifier.
 * @author  Codex
 * @date    2026-09-14
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "foc_identify.h"

#define TEST_RESET_LIMIT FOC_SCALAR(0.025f)

/**
 * @brief Convert a result to float and check its tolerance.
 * @param qActual Actual numeric backend value.
 * @param fExpected Expected PU value.
 * @param fTolerance Absolute allowed error.
 * @return None.
 */
static void test_AssertNear(foc_scalar_t qActual,
                            float fExpected,
                            float fTolerance)
{
    assert(fabsf(foc_to_float(qActual) - fExpected) < fTolerance);
}

/**
 * @brief Make a paired alpha-beta voltage/current sample.
 * @param fCurrentAlpha Alpha current in PU.
 * @param fCurrentBeta Beta current in PU.
 * @param fVoltageAlpha Alpha Vmodel in PU.
 * @param fVoltageBeta Beta Vmodel in PU.
 * @param bValid True when the full interval is measured.
 * @return The sample in the active numeric backend.
 */
static foc_identify_sample_t test_Sample(float fCurrentAlpha,
                                         float fCurrentBeta,
                                         float fVoltageAlpha,
                                         float fVoltageBeta,
                                         bool bValid)
{
    foc_identify_sample_t tSample = {
        .tCurrentAlphaBeta = {
            FOC_SCALAR(fCurrentAlpha), FOC_SCALAR(fCurrentBeta)},
        .tVmodelAlphaBeta = {
            FOC_SCALAR(fVoltageAlpha), FOC_SCALAR(fVoltageBeta)},
        .bValid = bValid,
    };

    return tSample;
}

/**
 * @brief Build a representative signed-excitation configuration.
 * @param wTimeout Maximum samples in one phase.
 * @return A complete PU configuration.
 */
static foc_identify_cfg_t test_Config(uint16_t wTimeout)
{
    foc_identify_cfg_t tConfig = {
        .qResistanceLowVoltagePu = FOC_SCALAR(-0.1f),
        .qResistanceHighVoltagePu = FOC_SCALAR(0.1f),
        .qInductanceDVoltagePu = FOC_SCALAR(0.2f),
        .qInductanceQVoltagePu = FOC_SCALAR(-0.2f),
        .qCurrentLimitPu = FOC_SCALAR(0.75f),
        .qResetCurrentLimitPu = TEST_RESET_LIMIT,
        .qMinimumResistanceDeltaPu = FOC_SCALAR(0.05f),
        .qCurrentStabilityTolerancePu = FOC_SCALAR(0.01f),
        .qElectricalBaseTurnsPerSample = FOC_SCALAR(0.005f),
        .hwMinimumSettlingSamples = 1U,
        .hwAverageSamples = 2U,
        .hwPhaseTimeoutSamples = wTimeout,
    };

    return tConfig;
}

/**
 * @brief Step an active identifier and require success.
 * @param ptIdentify Identifier instance.
 * @param tSample Paired input sample.
 * @return The populated output record.
 */
static foc_identify_output_t test_StepOk(
    foc_identify_t *ptIdentify,
    const foc_identify_sample_t *ptSample)
{
    foc_identify_output_t tOutput = {0};
    foc_result_t eResult = foc_identify_Step(
        ptIdentify, ptSample, &tOutput);

    assert(eResult == FOC_RESULT_OK);
    return tOutput;
}

/**
 * @brief Prime the low resistance excitation and verify the boundary update.
 * @param ptIdentify Identifier instance.
 * @return None.
 */
static void test_StartLow(foc_identify_t *ptIdentify)
{
    foc_identify_sample_t tSample = test_Sample(0.0f, 0.0f, 0.0f, 0.0f,
                                                 true);
    foc_identify_output_t tOutput = {0};
    foc_result_t eResult = foc_identify_Start(ptIdentify);

    assert(eResult == FOC_RESULT_OK);
    tOutput = test_StepOk(ptIdentify, &tSample);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_RESISTANCE_SETTLE);
    assert(tOutput.bReferenceChanged);
    test_AssertNear(tOutput.tVoltageReference.qD, -0.1f, 0.0001f);
    assert(tOutput.tVoltageReference.qQ == FOC_ZERO);
}

/**
 * @brief Feed the stable low and high Rs platforms to the D reset phase.
 * @param ptIdentify Identifier instance.
 * @param fLowCurrent Stable alpha current at low voltage.
 * @param fHighCurrent Stable alpha current at high voltage.
 * @return None.
 */
static void test_PrepareDReset(foc_identify_t *ptIdentify,
                               float fLowCurrent,
                               float fHighCurrent)
{
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    foc_identify_status_e eResistanceSettle =
        FOC_IDENTIFY_STATUS_IDLE;
    foc_identify_status_e eResistanceAverage =
        FOC_IDENTIFY_STATUS_IDLE;
    uint8_t hwIndex = 0U;

    test_StartLow(ptIdentify);
    eResistanceSettle = ptIdentify->tOutput.eStatus;
    for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
        float fCurrent = hwIndex == 0U ? fLowCurrent * 3.0f
                                       : fLowCurrent;
        float fVoltage = hwIndex == 0U ? -0.3f : -0.1f;

        tSample = test_Sample(fCurrent, 0.0f, fVoltage, 0.0f, true);
        tOutput = test_StepOk(ptIdentify, &tSample);
        assert(tOutput.bReferenceChanged == (bool)(hwIndex == 2U));
        if (hwIndex == 0U) {
            eResistanceAverage = tOutput.eStatus;
        }
    }
    assert(tOutput.eStatus == eResistanceSettle);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_RESISTANCE_SETTLE);
    assert(tOutput.bReferenceChanged);
    test_AssertNear(tOutput.tVoltageReference.qD, 0.1f, 0.0001f);

    for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
        float fCurrent = hwIndex == 0U ? fHighCurrent * 3.0f
                                       : fHighCurrent;
        float fVoltage = hwIndex == 0U ? 0.3f : 0.1f;

        tSample = test_Sample(fCurrent, 0.0f, fVoltage, 0.0f, true);
        tOutput = test_StepOk(ptIdentify, &tSample);
        assert(tOutput.bReferenceChanged == (bool)(hwIndex == 2U));
        if (hwIndex == 0U) {
            assert(tOutput.eStatus == eResistanceAverage);
        }
    }
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_RESET);
    assert(tOutput.bReferenceChanged);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);
}

/**
 * @brief Drive one reset sample and enter a selected D-axis voltage step.
 * @param ptIdentify Identifier instance.
 * @param fInitialCurrent Alpha current captured before the step.
 * @return None.
 */
static void test_StartDStep(foc_identify_t *ptIdentify,
                            float fInitialCurrent)
{
    foc_identify_sample_t tSample = test_Sample(
        fInitialCurrent, 0.0f, 0.0f, 0.0f, true);
    foc_identify_output_t tOutput = test_StepOk(ptIdentify, &tSample);

    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
    assert(tOutput.bReferenceChanged);
    test_AssertNear(tOutput.tVoltageReference.qD, 0.2f, 0.0001f);

    tSample = test_Sample(fInitialCurrent, 0.0f, 0.0f, 0.0f, true);
    tOutput = test_StepOk(ptIdentify, &tSample);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
    assert(tOutput.bReferenceChanged == false);
    assert(ptIdentify->wPhaseSamples == 0U);
    assert(ptIdentify->qWindowVoltageMean == FOC_ZERO);
}

/**
 * @brief Reset Q current and enter the negative Q-axis voltage step.
 * @param ptIdentify Identifier instance.
 * @param fDCurrent Alpha current retained from the D step.
 * @param fInitialCurrent Beta current captured before the Q step.
 * @return None.
 */
static void test_StartQStep(foc_identify_t *ptIdentify,
                            float fDCurrent,
                            float fInitialCurrent)
{
    foc_identify_sample_t tSample = test_Sample(
        fDCurrent, fInitialCurrent, 0.0f, 0.0f, true);
    foc_identify_output_t tOutput = test_StepOk(ptIdentify, &tSample);

    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
    assert(tOutput.bReferenceChanged);
    test_AssertNear(tOutput.tVoltageReference.qQ, -0.2f, 0.0001f);

    tSample = test_Sample(fDCurrent, fInitialCurrent, 0.0f, 0.0f, true);
    tOutput = test_StepOk(ptIdentify, &tSample);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
    assert(tOutput.bReferenceChanged == false);
    assert(ptIdentify->wPhaseSamples == 0U);
    assert(ptIdentify->qWindowVoltageMean == FOC_ZERO);
}

/**
 * @brief Cross the Q threshold and require a safe complete output.
 * @param ptIdentify Identifier instance.
 * @param wIntervals First valid crossing interval count.
 * @param fDCurrent Alpha current retained from the D step.
 * @param fInitialCurrent Pre-step beta current.
 * @return The terminal output record.
 */
static foc_identify_output_t test_CrossQ(foc_identify_t *ptIdentify,
                                         uint16_t wIntervals,
                                         float fDCurrent,
                                         float fInitialCurrent)
{
    const float afCurrent[4] = {-0.10f, -0.25f, -0.39f, -0.39f};
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    uint16_t hwIndex = 0U;

    test_StartQStep(ptIdentify, fDCurrent, fInitialCurrent);
    for (hwIndex = 0U; hwIndex < wIntervals; hwIndex++) {
        tSample = test_Sample(fDCurrent, afCurrent[hwIndex], 0.0f,
                              -0.2f, true);
        tOutput = test_StepOk(ptIdentify, &tSample);
        assert(tOutput.bReferenceChanged ==
               (bool)(hwIndex == (uint16_t)(wIntervals - 1U)));
    }
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE);
    assert(tOutput.bReferenceChanged);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);
    assert(tOutput.tVoltageReference.qQ == FOC_ZERO);
    return tOutput;
}

/**
 * @brief Verify signed Rs, biased-current RL steps, and result publication.
 * @param None.
 * @return None.
 */
static void test_CompleteMeasurement(void)
{
    foc_identify_cfg_t tConfig = test_Config(12U);
    foc_identify_t tIdentify = {0};
    foc_identify_result_t tResult = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tConfig.qResetCurrentLimitPu = FOC_SCALAR(0.06f);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_PrepareDReset(&tIdentify, -0.2f, 0.2f);
    test_StartDStep(&tIdentify, 0.05f);
    {
        const float afDCurrent[3] = {0.12f, 0.25f, 0.39f};
        foc_identify_sample_t tSample = {0};
        foc_identify_output_t tOutput = {0};
        uint8_t hwIndex = 0U;

        for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
            tSample = test_Sample(afDCurrent[hwIndex], 0.0f, 0.2f,
                                  0.0f, true);
            tOutput = test_StepOk(&tIdentify, &tSample);
            assert(tOutput.bReferenceChanged ==
                   (bool)(hwIndex == 2U));
        }
        eResult = foc_identify_GetResult(&tIdentify, &tResult);
        assert(eResult == FOC_RESULT_BUSY);
        assert(tResult.qResistancePu == FOC_ZERO);
        assert(tResult.qInductanceDPu == FOC_ZERO);
        assert(tResult.qInductanceQPu == FOC_ZERO);
        assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_RESET);
    }
    (void)test_CrossQ(&tIdentify, 3U, 0.39f, 0.02f);

    eResult = foc_identify_GetResult(&tIdentify, &tResult);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tResult.qResistancePu, 0.5f, 0.002f);
    test_AssertNear(tResult.qInductanceDPu, 0.01573f, 0.0008f);
    test_AssertNear(tResult.qInductanceQPu, 0.01573f, 0.0008f);
    eResult = foc_identify_Start(&tIdentify);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_identify_GetResult(&tIdentify, &tResult);
    assert(eResult == FOC_RESULT_BUSY);
    assert(tResult.qResistancePu == FOC_SCALAR(0.5f));
}

/**
 * @brief Verify fixed-point Rs above one PU remains calculable.
 * @param None.
 * @return None.
 */
static void test_RsAboveOnePu(void)
{
    foc_identify_cfg_t tConfig = test_Config(12U);
    foc_identify_t tIdentify = {0};
    foc_identify_result_t tResult = {0};
    foc_result_t eResult = FOC_RESULT_OK;
    uint8_t hwIndex = 0U;
    const float afDCurrent[3] = {0.04f, 0.08f, 0.10f};
    const float afQCurrent[3] = {-0.04f, -0.08f, -0.10f};

    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_PrepareDReset(&tIdentify, -0.05f, 0.05f);
    test_StartDStep(&tIdentify, 0.0f);
    for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
        foc_identify_sample_t tSample = test_Sample(
            afDCurrent[hwIndex], 0.0f, 0.2f, 0.0f, true);
        foc_identify_output_t tOutput = test_StepOk(
            &tIdentify, &tSample);

        if (hwIndex == 2U) {
            assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_RESET);
        } else {
            assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
        }
    }
    test_StartQStep(&tIdentify, 0.10f, 0.0f);
    for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
        foc_identify_sample_t tSample = test_Sample(
            0.10f, afQCurrent[hwIndex], 0.0f, -0.2f, true);
        foc_identify_output_t tOutput = test_StepOk(
            &tIdentify, &tSample);

        if (hwIndex == 2U) {
            assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE);
        } else {
            assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
        }
    }
    eResult = foc_identify_GetResult(&tIdentify, &tResult);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tResult.qResistancePu, 2.0f, 0.002f);
    test_AssertNear(tResult.qInductanceDPu, 0.06292f, 0.001f);
    test_AssertNear(tResult.qInductanceQPu, 0.06292f, 0.001f);
}

/**
 * @brief Verify the first complete post-step interval is accepted as N=1.
 * @param None.
 * @return None.
 */
static void test_FirstIntervalCrossing(void)
{
    foc_identify_cfg_t tConfig = test_Config(8U);
    foc_identify_t tIdentify = {0};
    foc_identify_result_t tResult = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_PrepareDReset(&tIdentify, -0.2f, 0.2f);
    test_StartDStep(&tIdentify, 0.0f);
    {
        foc_identify_sample_t tSample = test_Sample(
            0.39f, 0.0f, 0.2f, 0.0f, true);
        foc_identify_output_t tOutput = test_StepOk(&tIdentify, &tSample);

        assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_RESET);
    }
    (void)test_StartQStep(&tIdentify, 0.39f, 0.0f);
    {
        foc_identify_sample_t tSample = test_Sample(
            0.39f, -0.39f, 0.0f, -0.2f, true);
        foc_identify_output_t tOutput = test_StepOk(&tIdentify, &tSample);

        assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE);
    }
    eResult = foc_identify_GetResult(&tIdentify, &tResult);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tResult.qInductanceDPu, 0.00524f, 0.0005f);
    test_AssertNear(tResult.qInductanceQPu, 0.00524f, 0.0005f);
}

/**
 * @brief Reject a full-step sample that cannot resolve the current delta.
 * @param None.
 * @return None.
 */
static void test_ZeroStepDeltaRejected(void)
{
    foc_identify_cfg_t tConfig = test_Config(8U);
    foc_identify_t tIdentify = {0};
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tConfig.qResetCurrentLimitPu = FOC_SCALAR(0.21f);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_PrepareDReset(&tIdentify, -0.2f, 0.2f);
    tSample = test_Sample(0.2f, 0.0f, 0.0f, 0.0f, true);
    tOutput = test_StepOk(&tIdentify, &tSample);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
    tSample = test_Sample(0.2f, 0.0f, 0.0f, 0.0f, true);
    tOutput = test_StepOk(&tIdentify, &tSample);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
    tSample = test_Sample(0.2f, 0.0f, 0.1f, 0.0f, true);
    eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);
    assert(tOutput.bReferenceChanged);
}

/**
 * @brief Verify invalid paired data fails safely without a partial result.
 * @param None.
 * @return None.
 */
static void test_InvalidSampleAndNoPartialResult(void)
{
    foc_identify_cfg_t tConfig = test_Config(8U);
    foc_identify_t tIdentify = {0};
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    foc_identify_result_t tResult = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_StartLow(&tIdentify);
    tSample = test_Sample(0.0f, 0.0f, 0.0f, 0.0f, false);
    eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    assert(eResult == FOC_RESULT_SAFETY);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);
    eResult = foc_identify_GetResult(&tIdentify, &tResult);
    assert(eResult != FOC_RESULT_OK);
}

/**
 * @brief Verify current protection and unstable-platform timeout.
 * @param None.
 * @return None.
 */
static void test_CurrentLimitAndStabilityTimeout(void)
{
    foc_identify_cfg_t tConfig = test_Config(6U);
    foc_identify_t tIdentify = {0};
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    foc_result_t eResult = foc_identify_Init(&tIdentify, &tConfig);

    assert(eResult == FOC_RESULT_OK);
    test_StartLow(&tIdentify);
    tSample = test_Sample(0.8f, 0.0f, -0.1f, 0.0f, true);
    eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    assert(eResult == FOC_RESULT_SAFETY);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);

    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_StartLow(&tIdentify);
    {
        uint8_t hwIndex = 0U;

        for (hwIndex = 0U; hwIndex < 6U; hwIndex++) {
            float fCurrent = (hwIndex % 2U) == 0U ? -0.2f : -0.1f;
            tSample = test_Sample(fCurrent, 0.0f, -0.1f, 0.0f,
                                  true);
            eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
        }
    }
    assert(eResult == FOC_RESULT_SAFETY);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);
}

/**
 * @brief Verify delta rejection, reset timeout, and missing-crossing timeout.
 * @param None.
 * @return None.
 */
static void test_PhaseFailures(void)
{
    foc_identify_cfg_t tConfig = test_Config(6U);
    foc_identify_t tIdentify = {0};
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    foc_result_t eResult = FOC_RESULT_OK;
    uint8_t hwIndex = 0U;

    tConfig.qMinimumResistanceDeltaPu = FOC_SCALAR(0.5f);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_StartLow(&tIdentify);
    for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
        tSample = test_Sample(-0.2f, 0.0f, -0.1f, 0.0f, true);
        eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
        assert(eResult == FOC_RESULT_OK);
    }
    for (hwIndex = 0U; hwIndex < 2U; hwIndex++) {
        tSample = test_Sample(0.2f, 0.0f, 0.1f, 0.0f, true);
        eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
        assert(eResult == FOC_RESULT_OK);
    }
    tSample = test_Sample(0.2f, 0.0f, 0.1f, 0.0f, true);
    eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    assert(eResult == FOC_RESULT_SAFETY);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);

    tConfig = test_Config(4U);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_PrepareDReset(&tIdentify, -0.2f, 0.2f);
    for (hwIndex = 0U; hwIndex < 4U; hwIndex++) {
        tSample = test_Sample(0.1f, 0.0f, 0.0f, 0.0f, true);
        eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    }
    assert(eResult == FOC_RESULT_SAFETY);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);

    tConfig = test_Config(4U);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_PrepareDReset(&tIdentify, -0.2f, 0.2f);
    test_StartDStep(&tIdentify, 0.0f);
    for (hwIndex = 0U; hwIndex < 4U; hwIndex++) {
        tSample = test_Sample(0.1f, 0.0f, 0.2f, 0.0f, true);
        eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    }
    assert(eResult == FOC_RESULT_SAFETY);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);
}

/**
 * @brief Verify out-of-range inputs are rejected with zero output.
 * @param None.
 * @return None.
 */
static void test_OutOfRangeInputs(void)
{
    foc_identify_cfg_t tConfig = test_Config(8U);
    foc_identify_t tIdentify = {0};
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

#if defined(FOC_NUMERIC_FLOAT)
    tConfig.qInductanceDVoltagePu = FOC_SCALAR(1.1f);
#else
    tConfig.qInductanceDVoltagePu = (foc_scalar_t)(FOC_ONE + 1);
#endif
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);

    tConfig = test_Config(8U);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_StartLow(&tIdentify);
    tSample = test_Sample(0.0f, 0.0f, 0.8f, 0.8f, true);
    eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);

    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    test_StartLow(&tIdentify);
#if defined(FOC_NUMERIC_FLOAT)
    tSample = test_Sample(1.1f, 0.0f, -0.1f, 0.0f, true);
#else
    tSample.tCurrentAlphaBeta.qAlpha =
        (foc_scalar_t)(FOC_ONE + 1);
    tSample.tVmodelAlphaBeta.qAlpha = FOC_SCALAR(-0.1f);
    tSample.bValid = true;
#endif
    eResult = foc_identify_Step(&tIdentify, &tSample, &tOutput);
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);
}

/**
 * @brief Verify abort, active-start rejection, and terminal restart rules.
 * @param None.
 * @return None.
 */
static void test_LifecycleAndAbort(void)
{
    foc_identify_cfg_t tConfig = test_Config(12U);
    foc_identify_t tIdentify = {0};
    foc_identify_sample_t tSample = test_Sample(
        0.0f, 0.0f, 0.0f, 0.0f, true);
    foc_identify_output_t tOutput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_identify_Start(&tIdentify);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_identify_Start(&tIdentify);
    assert(eResult == FOC_RESULT_BUSY);
    tOutput = test_StepOk(&tIdentify, &tSample);
    assert(tOutput.bReferenceChanged);

    foc_identify_Abort(&tIdentify);
    eResult = foc_identify_Start(&tIdentify);
    assert(eResult == FOC_RESULT_BUSY);
    tOutput = test_StepOk(&tIdentify, &tSample);
    assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_ABORTED);
    assert(tOutput.tVoltageReference.qD == FOC_ZERO);
    eResult = foc_identify_Start(&tIdentify);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_identify_GetResult(&tIdentify, NULL);
    assert(eResult == FOC_RESULT_NULL);
}

/**
 * @brief Verify two interleaved instances retain independent configurations.
 * @param None.
 * @return None.
 */
static void test_InterleavedInstances(void)
{
    foc_identify_cfg_t tConfigA = test_Config(12U);
    foc_identify_cfg_t tConfigB = test_Config(12U);
    foc_identify_t tIdentifyA = {0};
    foc_identify_t tIdentifyB = {0};
    foc_identify_sample_t tSample = {0};
    foc_identify_output_t tOutput = {0};
    foc_identify_result_t tResultA = {0};
    foc_identify_result_t tResultB = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tConfigB.qInductanceDVoltagePu = FOC_SCALAR(0.1f);
    tConfigB.qInductanceQVoltagePu = FOC_SCALAR(-0.1f);
    eResult = foc_identify_Init(&tIdentifyA, &tConfigA);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_identify_Init(&tIdentifyB, &tConfigB);
    assert(eResult == FOC_RESULT_OK);
    test_StartLow(&tIdentifyA);
    test_StartLow(&tIdentifyB);
    {
        const float afLow[2] = {-0.2f, -0.4f};
        const float afHigh[2] = {0.2f, 0.4f};
        const float afRise[3] = {0.1f, 0.25f, 0.39f};
        uint8_t hwIndex = 0U;
        uint8_t hwUnit = 0U;

        for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
            tSample = test_Sample(afLow[0], 0.0f, -0.1f, 0.0f,
                                  true);
            tOutput = test_StepOk(&tIdentifyA, &tSample);
            tSample = test_Sample(afLow[1], 0.0f, -0.1f, 0.0f,
                                  true);
            tOutput = test_StepOk(&tIdentifyB, &tSample);
        }
        for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
            tSample = test_Sample(afHigh[0], 0.0f, 0.1f, 0.0f,
                                  true);
            tOutput = test_StepOk(&tIdentifyA, &tSample);
            tSample = test_Sample(afHigh[1], 0.0f, 0.1f, 0.0f,
                                  true);
            tOutput = test_StepOk(&tIdentifyB, &tSample);
        }
        for (hwUnit = 0U; hwUnit < 2U; hwUnit++) {
            foc_identify_t *ptCurrent = hwUnit == 0U
                                            ? &tIdentifyA : &tIdentifyB;
            tSample = test_Sample(0.0f, 0.0f, 0.0f, 0.0f, true);
            tOutput = test_StepOk(ptCurrent, &tSample);
            assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
        }
        for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
            tSample = test_Sample(afRise[hwIndex], 0.0f, 0.2f, 0.0f,
                                  true);
            tOutput = test_StepOk(&tIdentifyA, &tSample);
            tSample = test_Sample(afRise[hwIndex], 0.0f, 0.1f, 0.0f,
                                  true);
            tOutput = test_StepOk(&tIdentifyB, &tSample);
        }
        for (hwUnit = 0U; hwUnit < 2U; hwUnit++) {
            foc_identify_t *ptCurrent = hwUnit == 0U
                                            ? &tIdentifyA : &tIdentifyB;
            float fQVoltage = hwUnit == 0U ? -0.2f : -0.1f;

            tSample = test_Sample(0.39f, 0.0f, 0.0f, 0.0f,
                                  true);
            tOutput = test_StepOk(ptCurrent, &tSample);
            assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_AXIS_STEP);
            for (hwIndex = 0U; hwIndex < 3U; hwIndex++) {
                float fQCurrent = afRise[hwIndex] * -1.0f;
                tSample = test_Sample(0.39f, fQCurrent, 0.0f,
                                      fQVoltage, true);
                tOutput = test_StepOk(ptCurrent, &tSample);
            }
            assert(tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE);
        }
    }

    eResult = foc_identify_GetResult(&tIdentifyA, &tResultA);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_identify_GetResult(&tIdentifyB, &tResultB);
    assert(eResult == FOC_RESULT_OK);
    test_AssertNear(tResultA.qResistancePu, 0.5f, 0.002f);
    test_AssertNear(tResultB.qResistancePu, 0.25f, 0.002f);
    assert(tResultA.qInductanceDPu != tResultB.qInductanceDPu);
}

/**
 * @brief Verify configuration validation and result access preconditions.
 * @param None.
 * @return None.
 */
static void test_Validation(void)
{
    foc_identify_cfg_t tConfig = test_Config(12U);
    foc_identify_t tIdentify = {0};
    foc_identify_result_t tResult = {0};
    foc_result_t eResult = foc_identify_Init(NULL, &tConfig);

    assert(eResult == FOC_RESULT_NULL);
    eResult = foc_identify_Init(&tIdentify, NULL);
    assert(eResult == FOC_RESULT_NULL);
    tConfig.qResistanceHighVoltagePu = tConfig.qResistanceLowVoltagePu;
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
    eResult = foc_identify_Start(&tIdentify);
    assert(eResult != FOC_RESULT_OK);

    tConfig = test_Config(2U);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
    tConfig = test_Config(12U);
    tConfig.qCurrentLimitPu = FOC_ZERO;
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);

    tConfig = test_Config(12U);
    tConfig.qElectricalBaseTurnsPerSample = FOC_SCALAR(1.1f);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);

    tConfig = test_Config(12U);
    eResult = foc_identify_Init(&tIdentify, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_identify_GetResult(&tIdentify, &tResult);
    assert(eResult == FOC_RESULT_BUSY);
}

/**
 * @brief Run the host test suite for the active numeric backend.
 * @param None.
 * @return Zero when all assertions pass.
 */
int main(void)
{
    test_CompleteMeasurement();
    test_RsAboveOnePu();
    test_FirstIntervalCrossing();
    test_ZeroStepDeltaRejected();
    test_InvalidSampleAndNoPartialResult();
    test_CurrentLimitAndStabilityTimeout();
    test_PhaseFailures();
    test_OutOfRangeInputs();
    test_LifecycleAndAbort();
    test_InterleavedInstances();
    test_Validation();
    return 0;
}
