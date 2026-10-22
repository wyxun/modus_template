/**
 * @file motor_test.c
 * @brief Motor initialization failures and instance ownership.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Verify accepted PU data and failure without partial writes.
 * @return Zero when all assertions pass. */
int main(void)
{
    mfoc_pi_params_t pi = {MFOC_ONE, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_SCALAR(0.1f), MFOC_SCALAR(0.2f),
            MFOC_SCALAR(0.3f), MFOC_SCALAR(0.4f), MFOC_ZERO},
        .limits = {.i_max = MFOC_SCALAR(0.8f)},
        .core = {pi, pi, MFOC_SCALAR(0.55f), MFOC_ONE},
    };
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_Init(NULL, &cfg) == MFOC_RESULT_NULL);
    assert(mfoc_motor_Init(&motor, NULL) == MFOC_RESULT_NULL);
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(motor.params.pole_pairs == 7U);
    assert(motor.params.rs == cfg.params.rs);
    assert(motor.params.ld == cfg.params.ld);
    assert(motor.params.lq == cfg.params.lq);
    assert(motor.params.flux == cfg.params.flux);
    assert(motor.params.inertia == MFOC_ZERO);
    assert(motor.limits.i_max == cfg.limits.i_max);
    assert(motor.state == MFOC_MOTOR_IDLE && motor.faults == 0U);
    assert(motor.command.i_ref.qD == MFOC_ZERO);
    assert(motor.command.i_ref.qQ == MFOC_ZERO);
    mfoc_motor_t saved = motor;
    mfoc_motor_cfg_t bad = cfg;
    bad.params.pole_pairs = 0U;
    assert(mfoc_motor_Init(&motor, &bad) == MFOC_RESULT_INVALID_ARGUMENT);
    mfoc_scalar_t *fields[] = {&bad.params.rs, &bad.params.ld,
        &bad.params.lq, &bad.params.flux, &bad.limits.i_max};
    for (unsigned i = 0U; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        bad = cfg;
        *fields[i] = MFOC_ZERO;
        assert(mfoc_motor_Init(&motor, &bad) ==
               MFOC_RESULT_INVALID_ARGUMENT);
#if defined(MFOC_NUMERIC_FLOAT)
        *fields[i] = NAN;
        assert(mfoc_motor_Init(&motor, &bad) ==
               MFOC_RESULT_INVALID_ARGUMENT);
#endif
    }
    bad = cfg;
    bad.params.inertia = -MFOC_ONE;
    assert(mfoc_motor_Init(&motor, &bad) == MFOC_RESULT_INVALID_ARGUMENT);
    bad = cfg;
    bad.core.bus = MFOC_ZERO;
    assert(mfoc_motor_Init(&motor, &bad) != MFOC_RESULT_OK);
    assert(memcmp(&motor, &saved, sizeof(motor)) == 0);
    mfoc_motor_t other = {0};
    cfg.params.inertia = MFOC_SCALAR(2.0f);
    cfg.core.bus = MFOC_SCALAR(0.5f);
    assert(mfoc_motor_Init(&other, &cfg) == MFOC_RESULT_OK);
    assert(other.params.inertia == cfg.params.inertia);
    assert(other.core.bus_inv == MFOC_SCALAR(2.0f));
    assert(memcmp(&motor, &saved, sizeof(motor)) == 0);
    puts("Motor initialization and instance isolation passed");
    return 0;
}
