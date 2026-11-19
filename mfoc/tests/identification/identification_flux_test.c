/**
 * @file identification_flux_test.c
 * @brief Steady rotor-dq flux capture from applied alpha-beta voltage.
 */
#include "mfoc_identification_algorithms.h"
#include "mfoc_angle.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief A missing previous interval is skipped, a zero interval counts. */
static void voltage_history(void)
{
    const mfoc_ab_t current = {MFOC_ZERO, MFOC_HALF};
    const mfoc_ab_t zero = {0};
    mfoc_identification_flux_t capture = {0};
    mfoc_identification_flux_input_t input = {
        .i_ab_k = &current, .speed = MFOC_HALF};
    assert(mfoc_identification_flux_Step(&capture, &input) ==
        MFOC_FSM_ONGOING);
    assert(capture.count == 0U);
    input.u_ab_previous = &zero;
    assert(mfoc_identification_flux_Step(&capture, &input) ==
        MFOC_FSM_ONGOING);
    assert(capture.count == 1U && capture.sum_u_q == MFOC_ZERO);
    input.u_ab_previous = NULL;
    assert(mfoc_identification_flux_Step(&capture, &input) ==
        MFOC_FSM_ONGOING);
    assert(capture.count == 1U && capture.sum_speed == MFOC_HALF);
}

/** @brief Replay settled dq current and midpoint voltage at signed speed. */
static void steady_rotation(float speed)
{
    const mfoc_motor_params_t model = {.rs = MFOC_HALF, .ld = MFOC_ONE};
    const float flux = 0.4f;
    const float id = 0.1f, iq = 0.2f, ud = -0.3f;
    const float uq = 0.5f * iq + speed * (id + flux);
    const float turns = speed * MFOC_IDENTIFICATION_SPEED_BASE_HZ /
        MFOC_IDENTIFICATION_SAMPLE_HZ;
    mfoc_identification_flux_t capture = {0};
    mfoc_ab_t current = {0}, applied = {0};
    mfoc_identification_flux_input_t input = {
        .i_ab_k = &current, .u_ab_previous = &applied,
        .speed = mfoc_from_float(speed)};
    mfoc_fsm_rt_t progress = MFOC_FSM_ONGOING;
    for (unsigned n = 0U; n < MFOC_IDENTIFICATION_SAMPLE_HZ &&
            progress == MFOC_FSM_ONGOING; ++n) {
        float phase = turns * (float)n;
        float angle = phase * 6.28318530718f;
        float midpoint = (phase - 0.5f * turns) * 6.28318530718f;
        input.angle = mfoc_angle_from_turns(phase);
        current.qAlpha = mfoc_from_float(id * cosf(angle) - iq * sinf(angle));
        current.qBeta = mfoc_from_float(id * sinf(angle) + iq * cosf(angle));
        applied.qAlpha = mfoc_from_float(
            ud * cosf(midpoint) - uq * sinf(midpoint));
        applied.qBeta = mfoc_from_float(
            ud * sinf(midpoint) + uq * cosf(midpoint));
        progress = mfoc_identification_flux_Step(&capture, &input);
    }
    assert(progress == MFOC_FSM_COMPLETE);
    mfoc_scalar_t result = MFOC_ZERO;
    assert(mfoc_identification_flux_Finish(&capture, &model, &result) ==
        MFOC_FSM_COMPLETE);
    assert(fabsf(mfoc_to_float(result) - flux) < 0.001f);
}

int main(void)
{
    voltage_history();
    steady_rotation(0.5f);
    steady_rotation(-0.5f);
    puts("Flux identification history and midpoint projection passed");
    return 0;
}
