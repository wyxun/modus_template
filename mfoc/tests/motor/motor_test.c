/**
 * @file motor_test.c
 * @brief Motor initialization failures and instance ownership.
 * @author wyxun
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
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
        .current_base_milliamp = 10000U,
        .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
        .params = {7U, MFOC_SCALAR(0.1f), MFOC_SCALAR(0.2f),
            MFOC_SCALAR(0.3f), MFOC_SCALAR(0.4f), MFOC_ZERO},
        .limits = {.i_max = MFOC_SCALAR(0.8f)},
        .core = {pi, pi, MFOC_ONE},
    };
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_Init(NULL, &cfg) == MFOC_RESULT_NULL);
    assert(mfoc_motor_Init(&motor, NULL) == MFOC_RESULT_NULL);
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(motor_port_fake.port_cfg.current_base_milliamp ==
        cfg.current_base_milliamp);
    assert(motor_port_fake.port_cfg.voltage_base_millivolt ==
        cfg.voltage_base_millivolt);
    assert(motor_port_fake.port_cfg.electrical_speed_base_hz ==
        cfg.electrical_speed_base_hz);
    assert(motor_port_fake.port_cfg.pole_pairs == cfg.params.pole_pairs);
    assert(motor_port_fake.feedback_cfg.smo.speed_base_hz ==
        cfg.electrical_speed_base_hz);
    assert(motor_port_fake.feedback_cfg.flux.speed_base_hz ==
        cfg.electrical_speed_base_hz);
#if MFOC_FEEDBACK_ENABLE_EKF
    assert(motor_port_fake.feedback_cfg.ekf.speed_base_hz ==
        cfg.electrical_speed_base_hz);
#endif
    assert(motor.params.pole_pairs == 7U);
    assert(motor.params.rs == cfg.params.rs);
    assert(motor.params.ld == cfg.params.ld);
    assert(motor.params.lq == cfg.params.lq);
    assert(motor.params.flux == cfg.params.flux);
    assert(motor.params.inertia == MFOC_ZERO);
    assert(motor.limits.i_max == cfg.limits.i_max);
    assert(motor.state == MFOC_MOTOR_IDLE && motor.faults == 0U);
    assert(motor.i_ref.qD == MFOC_ZERO);
    assert(motor.i_ref.qQ == MFOC_ZERO);
    mfoc_motor_t saved = motor;
    mfoc_motor_cfg_t bad = cfg;
#if MFOC_FEEDBACK_ENABLE_HFI
    unsigned inits = motor_port_fake.port_inits;
    bad.electrical_speed_base_hz =
        MFOC_SCALAR(MFOC_HFI_SPEED_BASE_HZ / 2.0f);
    assert(mfoc_motor_Init(&motor, &bad) == MFOC_RESULT_OUT_OF_RANGE);
    assert(motor_port_fake.port_inits == inits);
    assert(memcmp(&motor, &saved, sizeof(motor)) == 0);
    bad = cfg;
#endif
    bad.current_base_milliamp = 0U;
    assert(mfoc_motor_Init(&motor, &bad) == MFOC_RESULT_OUT_OF_RANGE);
    bad = cfg;
    bad.voltage_base_millivolt = 0U;
    assert(mfoc_motor_Init(&motor, &bad) == MFOC_RESULT_OUT_OF_RANGE);
    bad = cfg;
    bad.electrical_speed_base_hz = MFOC_ZERO;
    assert(mfoc_motor_Init(&motor, &bad) == MFOC_RESULT_OUT_OF_RANGE);
    bad = cfg;
    bad.params.pole_pairs = 0U;
    assert(mfoc_motor_Init(&motor, &bad) == MFOC_RESULT_INVALID_ARGUMENT);
    /* i_max is not part of Motor's acceptance contract today, so only the
     * model entries that Feedback validates are exercised here. */
    mfoc_scalar_t *fields[] = {&bad.params.rs, &bad.params.ld,
        &bad.params.lq, &bad.params.flux};
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
    cfg.current_base_milliamp = 20000U;
    cfg.voltage_base_millivolt = 24000U;
#if !MFOC_FEEDBACK_ENABLE_HFI
    cfg.electrical_speed_base_hz = MFOC_SCALAR(200.0f);
#endif
    cfg.params.pole_pairs = 4U;
    cfg.params.inertia = MFOC_SCALAR(2.0f);
    cfg.core.bus = MFOC_SCALAR(0.5f);
    assert(mfoc_motor_Init(&other, &cfg) == MFOC_RESULT_OK);
    assert(motor_port_fake.port_cfg.current_base_milliamp == 20000U);
    assert(motor_port_fake.port_cfg.voltage_base_millivolt == 24000U);
    assert(motor_port_fake.port_cfg.electrical_speed_base_hz ==
        cfg.electrical_speed_base_hz);
    assert(motor_port_fake.port_cfg.pole_pairs == 4U);
    assert(other.params.inertia == cfg.params.inertia);
    assert(other.core.bus_inv == MFOC_SCALAR(2.0f));
    assert(memcmp(&motor, &saved, sizeof(motor)) == 0);
    puts("Motor initialization and instance isolation passed");
    return 0;
}
