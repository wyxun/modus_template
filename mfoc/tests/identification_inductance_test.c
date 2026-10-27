/**
 * @file identification_inductance_test.c
 * @brief Pulse capture against the analytical RL response.
 * @author Codex
 * @date 2026-10-26
 */
#include "mfoc_identification_algorithms.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Capture a physical RL transient, including changing pulse voltage.
 * @param henry Physical inductance. @param sign Pulse polarity.
 * @param initial Initial current in amperes. @return None. */
static void experiment(float henry, float sign, float initial)
{
    mfoc_identification_inductance_t state = {0};
    mfoc_scalar_t result = MFOC_ZERO;
    const float ohm = 4.0f;
    const float decay = expf(-ohm /
        ((float)MFOC_IDENTIFICATION_SAMPLE_HZ * henry));
    float amp = initial;
    assert(mfoc_identification_inductance_Step(&state,
        mfoc_from_float(amp / MFOC_MOTOR_BASE_CURRENT_AMP), MFOC_ONE) ==
        fsm_rt_on_going);
    assert(state.sum_u == MFOC_ZERO);
    assert(state.sum_i_twice == MFOC_ZERO);
    for (unsigned n = 0U; n < MFOC_IDENTIFICATION_L_SAMPLES; ++n) {
        float volt = sign * (n % 2U == 0U ? 1.2f : 1.5f);
        amp = volt / ohm + (amp - volt / ohm) * decay;
        fsm_rt_t rc = mfoc_identification_inductance_Step(&state,
            mfoc_from_float(amp / MFOC_MOTOR_BASE_CURRENT_AMP),
            mfoc_from_float(volt / MFOC_MOTOR_BASE_VOLTAGE_VOLT));
        assert(rc == (n + 1U == MFOC_IDENTIFICATION_L_SAMPLES ?
            fsm_rt_cpl : fsm_rt_on_going));
    }
    mfoc_identification_inductance_t saved = state;
    assert(mfoc_identification_inductance_Step(&state,
        MFOC_ONE, MFOC_ONE) == fsm_rt_cpl);
    assert(memcmp(&saved, &state, sizeof(state)) == 0);
    assert(mfoc_identification_inductance_Finish(&state,
        mfoc_from_float(ohm / MFOC_MOTOR_BASE_RESISTANCE_OHM),
        &result) == fsm_rt_cpl);
    float measured = mfoc_to_float(result) *
        MFOC_MOTOR_BASE_INDUCTANCE_HENRY;
    assert(fabsf(measured - henry) < henry * 0.01f);
    assert(memcmp(&saved, &state, sizeof(state)) == 0);
}

/** @brief Reject incomplete, unexcited and inconsistent frozen captures.
 * @return None. */
static void rejected_results(void)
{
    mfoc_identification_inductance_t state = {0};
    const mfoc_scalar_t sentinel = MFOC_SCALAR(0.7f);
    mfoc_scalar_t result = sentinel;
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ONE, &result) == fsm_rt_err);
    state.count = MFOC_IDENTIFICATION_L_SAMPLES + 1U;
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ONE, &result) == fsm_rt_err);
    state.i_last = MFOC_SCALAR(0.0005f);
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ONE, &result) == fsm_rt_err);
    state.i_last = MFOC_SCALAR(0.01f);
    state.sum_u = -MFOC_ONE;
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ZERO, &result) == fsm_rt_err);
    state.sum_u = MFOC_SCALAR(2.0f);
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ZERO, &result) == fsm_rt_err);
    assert(result == sentinel);
}

/** @brief Run standalone capture and foreground result tests. @return Zero. */
int main(void)
{
    experiment(0.0012f, 1.0f, 0.0f);
    experiment(0.0018f, 1.0f, 0.05f);
    experiment(0.0012f, -1.0f, 0.0f);
    experiment(0.0018f, -1.0f, -0.05f);
    rejected_results();
    puts("inductance capture and RL response passed");
    return 0;
}
