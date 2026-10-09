/****************************************************************************
 * @file    foc_observer_contract_test.c
 * @brief   Candidate contracts, direct SMO equivalence and lifecycle isolation.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stddef.h>

#include "foc_observer.h"

static const foc_electrical_model_t s_tModel = {
    .wResistanceMilliohm = 500U,
    .wInductanceDMicroHenry = 1000U,
    .wInductanceQMicroHenry = 1000U,
    .wVoltageBaseMillivolt = 12000U,
    .wCurrentBaseMilliamp = 7000U,
};
static const foc_observer_cfg_t s_tConfig = {
    .tSmo = {
        .wSampleFrequencyHz = 20000U,
        .qElectricalSpeedBaseHz = FOC_SCALAR(200.0f),
        .wBemfCutoffRadiansPerSecond = 10000U,
        .wSlidingGainMillivolt = 3500U,
        .qCurrentEstimateLimit = FOC_ONE,
    },
};

#if FOC_OBSERVER_HAS_SMO
/**
 * @brief Verify every candidate field equals the standalone SMO result.
 * @param a Composition candidate.
 * @param b Independent SMO candidate.
 * @return None.
 */
static void test_Equal(const foc_smo_output_t *a, const foc_smo_output_t *b)
{
    assert(a->tEstimate.tElectricalAngle.wBam32 ==
           b->tEstimate.tElectricalAngle.wBam32);
    assert(a->tEstimate.qElectricalSpeedPu == b->tEstimate.qElectricalSpeedPu);
    assert(a->tEstimate.bValid == b->tEstimate.bValid);
    assert(a->qBemfStrengthPu == b->qBemfStrengthPu);
}

/**
 * @brief Compare 512 changing samples and repeat after resetting histories.
 * @param None.
 * @return None.
 */
static void test_Recurrence(void)
{
    foc_observer_t observer = {0};
    foc_smo_t smo = {0};
    foc_observer_estimates_t estimates = {0};
    foc_smo_output_t direct = {0};
    foc_ab_t i = {0};
    foc_ab_t u = {0};
    const foc_observer_input_t input = {.tSmo = {&i, &u}};

    assert(foc_observer_Init(&observer, &s_tModel, &s_tConfig) ==
           FOC_RESULT_OK);
    assert(foc_smo_Init(&smo, &s_tModel, &s_tConfig.tSmo) == FOC_RESULT_OK);
    for (uint32_t pass = 0U; pass < 2U; pass++) {
        for (uint32_t k = 0U; k < 512U; k++) {
            i.qAlpha = foc_from_float((float)(k % 17U) * 0.002f - 0.016f);
            i.qBeta = foc_from_float((float)(k % 13U) * 0.003f - 0.018f);
            u.qAlpha = (k % 31U) < 15U ? FOC_ONE : FOC_NEG_ONE;
            u.qBeta = (k % 23U) < 11U ? FOC_HALF : -FOC_HALF;
            assert(foc_observer_Step(&observer, &input, &estimates) ==
                   FOC_RESULT_OK);
            assert(foc_smo_Step(&smo, &input.tSmo, &direct) == FOC_RESULT_OK);
            test_Equal(&estimates.tSmo, &direct);
        }
        foc_observer_Reset(&observer);
        foc_smo_Reset(&smo);
    }
}

/**
 * @brief Reject stale candidates, failed reinit and nonfinite samples.
 * @param None.
 * @return None.
 */
static void test_Lifecycle(void)
{
    foc_observer_t a = {0};
    foc_observer_t b = {0};
    foc_observer_t control = {0};
    foc_observer_estimates_t out = {0};
    foc_observer_estimates_t other = {0};
    foc_observer_estimates_t expected = {0};
    foc_ab_t i = {0};
    foc_ab_t u = {FOC_SCALAR(0.1f), FOC_ZERO};
    foc_observer_input_t input = {.tSmo = {&i, &u}};
    foc_observer_cfg_t invalid = s_tConfig;

    assert(foc_observer_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    assert(!out.tSmo.tEstimate.bValid);
    assert(foc_observer_Init(&a, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
    assert(foc_observer_Init(&b, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
    assert(foc_observer_Init(&control, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
    assert(foc_observer_Step(&a, &input, &out) == FOC_RESULT_OK);
    assert(out.tSmo.tEstimate.bValid);
    assert(foc_observer_Step(&a, NULL, &out) == FOC_RESULT_NULL);
    assert(!out.tSmo.tEstimate.bValid);
    assert(out.tSmo.qBemfStrengthPu == FOC_ZERO);
    assert(foc_observer_Step(NULL, &input, &out) == FOC_RESULT_NULL);
    assert(foc_observer_Step(&a, &input, NULL) == FOC_RESULT_NULL);
    input.tSmo.ptVoltageModelAlphaBeta = NULL;
    assert(foc_observer_Step(&a, &input, &out) == FOC_RESULT_NULL);
    assert(!out.tSmo.tEstimate.bValid);
    input.tSmo.ptVoltageModelAlphaBeta = &u;
    foc_observer_Reset(&a);
    assert(foc_observer_Step(&a, &input, &expected) == FOC_RESULT_OK);
    assert(foc_observer_Step(&b, &input, &other) == FOC_RESULT_OK);
    test_Equal(&expected.tSmo, &other.tSmo);
    assert(foc_observer_Step(&control, &input, &out) == FOC_RESULT_OK);
    test_Equal(&out.tSmo, &other.tSmo);
    assert(foc_observer_Init(&a, NULL, &s_tConfig) == FOC_RESULT_NULL);
    assert(foc_observer_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    /* A's failed reinit must not alter B's already advancing history. */
    assert(foc_observer_Step(&b, &input, &other) == FOC_RESULT_OK);
    assert(foc_observer_Step(&control, &input, &out) == FOC_RESULT_OK);
    test_Equal(&out.tSmo, &other.tSmo);
    invalid.tSmo.wSampleFrequencyHz = 0U;
    assert(foc_observer_Init(&a, &s_tModel, &invalid) ==
           FOC_RESULT_INVALID_ARGUMENT);
    assert(foc_observer_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    assert(foc_observer_Init(&a, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
#if defined(FOC_NUMERIC_FLOAT)
    i.qAlpha = NAN;
    assert(foc_observer_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    assert(!out.tSmo.tEstimate.bValid);
    i.qAlpha = FOC_ZERO;
    foc_observer_Reset(&b);
    assert(foc_observer_Step(&a, &input, &out) == FOC_RESULT_OK);
    assert(foc_observer_Step(&b, &input, &other) == FOC_RESULT_OK);
    test_Equal(&out.tSmo, &other.tSmo);
#endif
}
#endif

/**
 * @brief Check enabled and disabled composition contracts.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    assert(foc_observer_Init(NULL, &s_tModel, &s_tConfig) == FOC_RESULT_NULL);
#if FOC_OBSERVER_HAS_SMO
    test_Recurrence();
    test_Lifecycle();
#else
    foc_observer_t observer = {0};
    foc_observer_input_t input = {0};
    foc_observer_estimates_t estimates = {0};

    assert(foc_observer_Init(&observer, &s_tModel, &s_tConfig) ==
           FOC_RESULT_DISABLED);
    estimates.tSmo.tEstimate.bValid = true;
    assert(foc_observer_Step(&observer, &input, &estimates) ==
           FOC_RESULT_DISABLED);
    assert(!estimates.tSmo.tEstimate.bValid);
    foc_observer_Reset(&observer);
    assert(!observer.bInitialized);
#endif
    return 0;
}
