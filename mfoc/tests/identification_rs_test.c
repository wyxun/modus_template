/**
 * @file identification_rs_test.c
 * @brief Independent Rs capture timing, slope and result validation.
 * @author Codex
 * @date 2026-10-25
 */
#include "mfoc_identification.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Drive fixed voltage levels through a resistance and voltage bias.
 * @param resistance Actual PU resistance. @param fault Injected fault kind.
 * @return Terminal progress; results must remain unchanged on failure. */
static fsm_rt_t experiment(float resistance, unsigned fault)
{
    mfoc_identification_rs_t state = {0};
    mfoc_ab_t current = {0};
    mfoc_scalar_t result = MFOC_SCALAR(0.7f);
    float modulation = MFOC_IDENTIFICATION_RS_MODULATION_LOW;
    fsm_rt_t progress = fsm_rt_on_going;
    for (unsigned n = 0U; n < 20000U; ++n) {
        float measured = (modulation - 0.015f) / resistance +
            (n % 2U == 0U ? 0.0006f : -0.0006f);
        if (fault == 1U) { measured = 0.0f; }
        if (fault == 2U) { measured = 0.2f; }
        if (fault == 3U && state.count > 4000U) {
            measured += (float)(state.count - 4000U) * 0.000006f;
        }
        current.qAlpha = mfoc_from_float(measured);
        if (fault == 4U) { current.qAlpha = -mfoc_abs(current.qAlpha); }
        if (fault == 5U) { current.qAlpha = MFOC_SCALAR(0.001f); }
        if (fault == 6U) { current.qAlpha = MFOC_SCALAR(0.015f); }
        progress = mfoc_identification_rs_Step(&state, &current);
        if (progress != fsm_rt_on_going) { break; }
        modulation = state.level == MFOC_IDENTIFICATION_RS_LOW ?
            MFOC_IDENTIFICATION_RS_MODULATION_LOW :
            MFOC_IDENTIFICATION_RS_MODULATION_HIGH;
    }
    assert(progress != fsm_rt_on_going);
    if (progress == fsm_rt_cpl) {
        assert(result == MFOC_SCALAR(0.7f));
        mfoc_identification_rs_t saved = state;
        assert(mfoc_identification_rs_Step(&state, &current) ==
            fsm_rt_cpl);
        assert(memcmp(&saved, &state, sizeof(state)) == 0);
        progress = mfoc_identification_rs_Finish(&state, MFOC_ONE, &result);
        assert(memcmp(&saved, &state, sizeof(state)) == 0);
        if (progress == fsm_rt_cpl) {
            assert(fabsf(mfoc_to_float(result) - resistance) < 0.006f);
        } else {
            assert(result == MFOC_SCALAR(0.7f));
        }
    } else {
        assert(result == MFOC_SCALAR(0.7f));
        assert(mfoc_identification_rs_Finish(&state, MFOC_ONE, &result) ==
            fsm_rt_err);
    }
    return progress;
}

/** @brief Prove both levels discard transients before accumulating samples.
 * @return None. */
static void settling(void)
{
    const unsigned settle = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_SETTLE_MS / 1000U;
    const unsigned samples = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_AVERAGE_MS / 1000U;
    mfoc_identification_rs_t state = {0};
    for (unsigned level = 0U; level < 2U; ++level) {
        mfoc_ab_t current = {MFOC_ONE, MFOC_ZERO};
        for (unsigned n = 0U; n < settle; ++n) {
            assert(mfoc_identification_rs_Step(&state, &current) ==
                fsm_rt_on_going);
            assert(state.sum_i[level] == MFOC_ZERO);
            assert(state.level == level);
        }
        current.qAlpha = MFOC_HALF;
        for (unsigned n = 0U; n < samples; ++n) {
            fsm_rt_t rc = mfoc_identification_rs_Step(&state, &current);
            assert(rc == (level == 1U && n + 1U == samples ?
                fsm_rt_cpl : fsm_rt_on_going));
        }
        assert(state.count == 0U && state.level == level + 1U);
        assert(state.sum_i[level] == MFOC_HALF * (mfoc_scalar_t)samples);
    }
}

/** @brief Check the dispatcher changes voltage after consuming each sample.
 * @return None. */
static void excitation(void)
{
    const unsigned window = MFOC_IDENTIFICATION_SAMPLE_HZ *
        (MFOC_IDENTIFICATION_RS_SETTLE_MS +
         MFOC_IDENTIFICATION_RS_AVERAGE_MS) / 1000U;
    mfoc_identification_t id = {0};
    mfoc_motor_params_t model = {0};
    assert(mfoc_identification_Start(&id, &model,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    id.stage = MFOC_IDENTIFICATION_STAGE_RS;
    mfoc_ab_t current = {MFOC_HALF, MFOC_ZERO};
    mfoc_identification_input_t input = {.i_ab_k = &current};
    mfoc_identification_output_t output = {0};
    for (unsigned n = 0U; n < 2U * window; ++n) {
        mfoc_identification_Step(&id, &input, &output);
        mfoc_scalar_t expected = MFOC_ZERO;
        if (n + 1U < window) {
            expected = MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW);
        } else if (n + 1U < 2U * window) {
            expected = MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH);
        } else { /* Complete: remove excitation. */ }
        assert(output.modulation_ab.qAlpha == expected);
    }
    assert(id.stage_done);
}

/** @brief Cover independent instances, reset and every failure boundary.
 * @return Zero after all assertions pass. */
int main(void)
{
    assert(experiment(1.1666667f, 0U) == fsm_rt_cpl);
    assert(experiment(2.0f, 0U) == fsm_rt_cpl);
    for (unsigned fault = 1U; fault <= 6U; ++fault) {
        assert(experiment(1.2f, fault) == fsm_rt_err);
    }
    assert(experiment(-1.0f, 0U) == fsm_rt_err);
    assert(experiment(9.0f, 0U) == fsm_rt_err);
    mfoc_identification_rs_t a = {0}, b = {0};
    mfoc_ab_t i = {0};
    mfoc_scalar_t r = MFOC_ONE;
    assert(mfoc_identification_rs_Finish(&a, MFOC_ONE, &r) == fsm_rt_err);
    assert(r == MFOC_ONE);
    for (unsigned n = 0U; n < 10U; ++n) {
        assert(mfoc_identification_rs_Step(&a, &i) ==
            fsm_rt_on_going);
        assert(mfoc_identification_rs_Step(&b, &i) ==
            fsm_rt_on_going);
    }
    assert(a.count == 10U && b.count == 10U);
    a = (mfoc_identification_rs_t){0};
    assert(b.count == 10U);
    assert(mfoc_identification_rs_Step(&a, &i) ==
        fsm_rt_on_going && a.count == 1U);
    settling();
    excitation();
    puts("Rs slope, voltage bias, stability, rejection and reset passed");
    return 0;
}
