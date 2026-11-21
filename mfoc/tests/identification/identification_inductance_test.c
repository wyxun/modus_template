/**
 * @file identification_inductance_test.c
 * @brief Pulse capture against the analytical RL response.
 * @author wyxun
 * @date 2026-10-26
 */
#include "mfoc_identification_algorithms.h"
#include "mfoc_motor_config_internal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Capture a physical RL transient with varying pulse voltage. */
static mfoc_identification_inductance_t capture_rl(
    uint32_t sample_hz, float henry, float sign, float initial)
{
    mfoc_identification_inductance_t state = {0};
    const float ohm = 4.0f;
    const float decay = expf(-ohm / ((float)sample_hz * henry));
    float amp = initial;
    assert(mfoc_identification_inductance_Step(&state,
        mfoc_from_float(amp / MFOC_MOTOR_BASE_CURRENT_AMP), MFOC_ONE) ==
        MFOC_FSM_ONGOING);
    assert(state.sum_u == MFOC_ZERO);
    assert(state.sum_i_twice == MFOC_ZERO);
    for (unsigned n = 0U; n < MFOC_IDENTIFICATION_L_SAMPLES; ++n) {
        float volt = sign * (n % 2U == 0U ? 1.2f : 1.5f);
        amp = volt / ohm + (amp - volt / ohm) * decay;
        mfoc_fsm_rt_t rc =
            mfoc_identification_inductance_Step(&state,
                mfoc_from_float(amp / MFOC_MOTOR_BASE_CURRENT_AMP),
                mfoc_from_float(volt / MFOC_MOTOR_BASE_VOLTAGE_VOLT));
        assert(rc == (n + 1U == MFOC_IDENTIFICATION_L_SAMPLES ?
            MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING));
    }
    mfoc_identification_inductance_t saved = state;
    assert(mfoc_identification_inductance_Step(&state,
        MFOC_ONE, MFOC_ONE) == MFOC_FSM_COMPLETE);
    assert(memcmp(&saved, &state, sizeof(state)) == 0);
    return state;
}

/** @brief Compare the two result APIs at the default control rate. */
static void experiment(mfoc_scalar_t dt, float henry, float sign,
                       float initial)
{
    mfoc_identification_inductance_t state = capture_rl(
        MFOC_IDENTIFICATION_SAMPLE_HZ, henry, sign, initial);
    mfoc_identification_inductance_t saved = state;
    const mfoc_scalar_t rs = mfoc_from_float(4.0f /
        MFOC_MOTOR_BASE_RESISTANCE_OHM);
    mfoc_scalar_t result = MFOC_ZERO;
    mfoc_scalar_t legacy = MFOC_ZERO;
    assert(mfoc_identification_inductance_Finish(&state, rs, &legacy) ==
           MFOC_FSM_COMPLETE);
    assert(mfoc_identification_inductance_FinishPrepared(&state, rs, dt,
        &result) == MFOC_FSM_COMPLETE);
    assert(result == legacy);
    float measured = mfoc_to_float(result) *
        MFOC_MOTOR_BASE_INDUCTANCE_HENRY;
    assert(fabsf(measured - henry) < henry * 0.01f);
    assert(memcmp(&saved, &state, sizeof(state)) == 0);
}

/** @brief Recover two physical inductances with one nondefault time base. */
static void shared_timing(void)
{
    const uint32_t sample_hz = 32000U;
    const float base_hz = 337.5f;
    const double omega_base = 2.0 * acos(-1.0) * base_hz;
    const double base_henry = MFOC_MOTOR_BASE_RESISTANCE_OHM / omega_base;
    const mfoc_scalar_t rs = mfoc_from_float(4.0f /
        MFOC_MOTOR_BASE_RESISTANCE_OHM);
    mfoc_scalar_t dt = MFOC_ZERO;
    assert(mfoc_identification_inductance_InitTiming(&dt, sample_hz,
        mfoc_from_float(base_hz)) == MFOC_RESULT_OK);
    assert(fabs(mfoc_to_float(dt) - omega_base / sample_hz) <
           1.0 / MFOC_Q_SCALE);
    const float henry[2] = {0.0012f, 0.0018f};
    for (unsigned axis = 0U; axis < 2U; ++axis) {
        mfoc_identification_inductance_t capture = capture_rl(sample_hz,
            henry[axis], axis == 0U ? 1.0f : -1.0f, 0.0f);
        mfoc_scalar_t result = MFOC_ZERO;
        assert(mfoc_identification_inductance_FinishPrepared(
            &capture, rs, dt, &result) == MFOC_FSM_COMPLETE);
        const double measured = mfoc_to_float(result) * base_henry;
        assert(fabs(measured - henry[axis]) < henry[axis] * 0.01);
        printf("%c axis: %.6f mH\n", axis == 0U ? 'd' : 'q',
               measured * 1000.0);
    }
}

/** @brief Reject incomplete, unexcited and inconsistent frozen captures.
 * @return None. */
static void rejected_results(void)
{
    mfoc_identification_inductance_t state = {0};
    const mfoc_scalar_t sentinel = MFOC_SCALAR(0.7f);
    mfoc_scalar_t result = sentinel;
    mfoc_scalar_t dt = sentinel;
    assert(mfoc_identification_inductance_InitTiming(NULL, 20000U,
        MFOC_ONE) != MFOC_RESULT_OK);
    assert(mfoc_identification_inductance_InitTiming(&dt, 0U,
        MFOC_ONE) != MFOC_RESULT_OK && dt == sentinel);
    assert(mfoc_identification_inductance_InitTiming(&dt, 20000U,
        -MFOC_ONE) != MFOC_RESULT_OK && dt == sentinel);
#if defined(MFOC_NUMERIC_FLOAT)
    assert(mfoc_identification_inductance_InitTiming(&dt, 20000U,
        NAN) != MFOC_RESULT_OK && dt == sentinel);
#endif
    assert(mfoc_identification_inductance_FinishPrepared(&state, MFOC_ONE,
        dt, &result) == MFOC_FSM_FAILED && result == sentinel);
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ONE, &result) == MFOC_FSM_FAILED);
    state.count = MFOC_IDENTIFICATION_L_SAMPLES + 1U;
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ONE, &result) == MFOC_FSM_FAILED);
    state.i_last = MFOC_SCALAR(0.0005f);
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ONE, &result) == MFOC_FSM_FAILED);
    state.i_last = MFOC_SCALAR(0.01f);
    state.sum_u = -MFOC_ONE;
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ZERO, &result) == MFOC_FSM_FAILED);
    state.sum_u = MFOC_SCALAR(2.0f);
    assert(mfoc_identification_inductance_Finish(&state,
        MFOC_ZERO, &result) == MFOC_FSM_FAILED);
    assert(result == sentinel);
}

/** @brief Run standalone capture and foreground result tests. @return Zero. */
int main(void)
{
    mfoc_scalar_t dt = MFOC_ZERO;
    assert(mfoc_identification_inductance_InitTiming(
        &dt, MFOC_IDENTIFICATION_SAMPLE_HZ,
        MFOC_SCALAR(MFOC_IDENTIFICATION_SPEED_BASE_HZ)) == MFOC_RESULT_OK);
    experiment(dt, 0.0012f, 1.0f, 0.0f);
    experiment(dt, 0.0018f, 1.0f, 0.05f);
    experiment(dt, 0.0012f, -1.0f, 0.0f);
    experiment(dt, 0.0018f, -1.0f, -0.05f);
    shared_timing();
    rejected_results();
    puts("inductance capture and RL response passed");
    return 0;
}
