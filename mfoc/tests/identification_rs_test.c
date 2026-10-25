/**
 * @file identification_rs_test.c
 * @brief Independent Rs slope, invalid input and reset checks.
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
    mfoc_identification_output_t request = {0};
    mfoc_scalar_t result = MFOC_SCALAR(0.7f);
    float modulation = 0.0f;
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
        if (fault == 6U) { current.qAlpha = MFOC_SCALAR(0.001f); }
        if (fault == 7U) { current.qAlpha = MFOC_SCALAR(0.015f); }
        if (fault == 8U) { current.qBeta = MFOC_SCALAR(0.02f); }
        const mfoc_ab_t *sample = fault == 5U && n == 100U ? NULL : &current;
        progress = mfoc_identification_rs_Step(&state, sample, &request);
        if (progress != fsm_rt_on_going) { break; }
        modulation = mfoc_to_float(request.modulation);
    }
    assert(progress != fsm_rt_on_going);
    if (progress == fsm_rt_cpl) {
        assert(result == MFOC_SCALAR(0.7f));
        assert(!request.stop && request.modulation == MFOC_ZERO);
        mfoc_identification_rs_t saved = state;
        assert(mfoc_identification_rs_Step(&state, &current, &request) ==
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
        assert(request.stop && result == MFOC_SCALAR(0.7f));
        assert(mfoc_identification_rs_Finish(&state, MFOC_ONE, &result) ==
            fsm_rt_err);
    }
    return progress;
}

/** @brief Cover independent instances, reset and every failure boundary.
 * @return Zero after all assertions pass. */
int main(void)
{
    assert(experiment(1.1666667f, 0U) == fsm_rt_cpl);
    assert(experiment(2.0f, 0U) == fsm_rt_cpl);
    for (unsigned fault = 1U; fault <= 8U; ++fault) {
        assert(experiment(1.2f, fault) == fsm_rt_err);
    }
    assert(experiment(-1.0f, 0U) == fsm_rt_err);
    assert(experiment(9.0f, 0U) == fsm_rt_err);
    mfoc_identification_rs_t a = {0}, b = {0};
    mfoc_ab_t i = {0};
    mfoc_identification_output_t output = {0};
    mfoc_scalar_t r = MFOC_ONE;
    assert(mfoc_identification_rs_Step(NULL, &i, &output) ==
        fsm_rt_err && output.stop);
    assert(mfoc_identification_rs_Finish(&a, MFOC_ONE, &r) == fsm_rt_err);
    assert(r == MFOC_ONE);
    for (unsigned n = 0U; n < 10U; ++n) {
        assert(mfoc_identification_rs_Step(&a, &i, &output) ==
            fsm_rt_on_going);
        assert(mfoc_identification_rs_Step(&b, &i, &output) ==
            fsm_rt_on_going);
    }
    assert(a.count == 10U && b.count == 10U);
    a = (mfoc_identification_rs_t){0};
    assert(b.count == 10U);
    assert(mfoc_identification_rs_Step(&a, &i, &output) ==
        fsm_rt_on_going && a.count == 1U);
    puts("Rs slope, voltage bias, stability, rejection and reset passed");
    return 0;
}
