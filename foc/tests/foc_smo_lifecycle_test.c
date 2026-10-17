/****************************************************************************
 * @file    foc_smo_lifecycle_test.c
 * @brief   SMO failure invalidation, reset and independent instance histories.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stddef.h>

#include "foc_config.h"
#include "foc_smo.h"

static const foc_electrical_model_t s_tModel = {
    .wResistanceMilliohm = 500U,
    .wInductanceDMicroHenry = 1000U,
    .wInductanceQMicroHenry = 1000U,
    .wVoltageBaseMillivolt = 12000U,
    .wCurrentBaseMilliamp = 7000U,
};
static const foc_smo_cfg_t s_tConfig = {
    .wSampleFrequencyHz = 20000U,
    .qElectricalSpeedBaseHz = FOC_SCALAR(200.0f),
    .wBemfCutoffRadiansPerSecond = 10000U,
    .wSlidingGainMillivolt = 3500U,
    .qCurrentEstimateLimit = FOC_ONE,
};

/**
 * @brief Verify every candidate field equals the standalone SMO result.
 * @param a First instance result.
 * @param b Independent instance result.
 * @return None.
 */
static void test_Equal(const foc_smo_output_t *a, const foc_smo_output_t *b)
{
    assert(a->tElectricalAngle.wBam32 ==
           b->tElectricalAngle.wBam32);
    assert(a->qElectricalSpeedPu == b->qElectricalSpeedPu);
    assert(a->bValid == b->bValid);
    assert(a->qBemfStrengthPu == b->qBemfStrengthPu);
}

/**
 * @brief Reject stale candidates, failed reinit and nonfinite samples.
 * @param None.
 * @return None.
 */
static void test_Lifecycle(void)
{
    foc_smo_t a = {0};
    foc_smo_t b = {0};
    foc_smo_t control = {0};
    foc_smo_output_t out = {0};
    foc_smo_output_t other = {0};
    foc_smo_output_t expected = {0};
    foc_ab_t i = {0};
    foc_ab_t u = {FOC_SCALAR(0.1f), FOC_ZERO};
    foc_smo_input_t input = {&i, &u};
    foc_smo_cfg_t invalid = s_tConfig;

    assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    assert(!out.bValid);
    assert(foc_smo_Init(&a, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
    assert(foc_smo_Init(&b, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
    assert(foc_smo_Init(&control, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
    assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_OK);
    assert(out.bValid);
    assert(foc_smo_Step(&a, NULL, &out) == FOC_RESULT_NULL);
    assert(!out.bValid);
    assert(out.qBemfStrengthPu == FOC_ZERO);
    assert(foc_smo_Step(NULL, &input, &out) == FOC_RESULT_NULL);
    assert(foc_smo_Step(&a, &input, NULL) == FOC_RESULT_NULL);
    input.ptVoltageModelAlphaBeta = NULL;
    assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_NULL);
    assert(!out.bValid);
    input.ptVoltageModelAlphaBeta = &u;
    foc_smo_Reset(&a);
    assert(foc_smo_Step(&a, &input, &expected) == FOC_RESULT_OK);
    assert(foc_smo_Step(&b, &input, &other) == FOC_RESULT_OK);
    test_Equal(&expected, &other);
    assert(foc_smo_Step(&control, &input, &out) == FOC_RESULT_OK);
    test_Equal(&out, &other);
    assert(foc_smo_Init(&a, NULL, &s_tConfig) == FOC_RESULT_NULL);
    assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    /* A's failed reinit must not alter B's already advancing history. */
    assert(foc_smo_Step(&b, &input, &other) == FOC_RESULT_OK);
    assert(foc_smo_Step(&control, &input, &out) == FOC_RESULT_OK);
    test_Equal(&out, &other);
    invalid.wSampleFrequencyHz = 0U;
    assert(foc_smo_Init(&a, &s_tModel, &invalid) ==
           FOC_RESULT_INVALID_ARGUMENT);
    assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    assert(foc_smo_Init(&a, &s_tModel, &s_tConfig) == FOC_RESULT_OK);
#if defined(FOC_NUMERIC_FLOAT)
    i.qAlpha = NAN;
    assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_INVALID_ARGUMENT);
    assert(!out.bValid);
    i.qAlpha = FOC_ZERO;
    foc_smo_Reset(&b);
    assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_OK);
    assert(foc_smo_Step(&b, &input, &other) == FOC_RESULT_OK);
    test_Equal(&out, &other);
    foc_scalar_t *fields[] = {&i.qAlpha, &i.qBeta, &u.qAlpha, &u.qBeta};
    const float invalidValues[] = {NAN, INFINITY, -INFINITY};

    for (unsigned field = 0U; field < 4U; field++) {
        for (unsigned bad = 0U; bad < 3U; bad++) {
            foc_scalar_t saved = *fields[field];

            *fields[field] = invalidValues[bad];
            assert(foc_smo_Step(&a, &input, &out) ==
                   FOC_RESULT_INVALID_ARGUMENT);
            assert(!out.bValid);
            assert(a.tAxis[0].z_prev == FOC_ZERO);
            assert(a.tAxis[1].e_hat == FOC_ZERO);
            *fields[field] = saved;
            foc_smo_Reset(&b);
            assert(foc_smo_Step(&a, &input, &out) == FOC_RESULT_OK);
            assert(foc_smo_Step(&b, &input, &other) == FOC_RESULT_OK);
            test_Equal(&out, &other);
        }
    }
#endif
}

/**
 * @brief Check direct SMO lifecycle contracts.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    assert(foc_smo_Init(NULL, &s_tModel, &s_tConfig) == FOC_RESULT_NULL);
    test_Lifecycle();
    return 0;
}
