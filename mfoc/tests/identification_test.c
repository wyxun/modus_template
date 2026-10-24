/**
 * @file identification_test.c
 * @brief Skeleton shutdown and transactional Motor model publication.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stdio.h>

/** @brief Exercise management without claiming an implemented estimator.
 * @return Zero after all assertions. */
int main(void)
{
    mfoc_pi_params_t pi = {MFOC_ONE, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE,
            MFOC_HALF, MFOC_ZERO},
        .limits = {.i_max = MFOC_ONE}, .core = {pi, pi, MFOC_ONE}};
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    mfoc_motor_params_t result = cfg.params;
    assert(mfoc_identification_GetResult(&motor.identification, &result) ==
        MFOC_RESULT_INVALID_STATE);
    assert(result.rs == cfg.params.rs);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_ELECTRICAL) ==
        MFOC_RESULT_OK);
    assert(!motor_port_fake.enabled);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_ELECTRICAL) ==
        MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    result.rs = MFOC_ONE;
    assert(mfoc_identification_ApplyResult(&motor.identification,
        &motor.params, &motor.feedback) == MFOC_RESULT_INVALID_STATE);
    unsigned writes = motor_port_fake.writes;
    unsigned reads = motor_port_fake.feedback_reads;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.state == MFOC_MOTOR_IDLE && !motor_port_fake.enabled);
    assert(motor_port_fake.writes == writes);
    assert(motor_port_fake.feedback_reads == reads);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_ERROR);
    assert(mfoc_identification_GetResult(&motor.identification, &result) ==
        MFOC_RESULT_INVALID_STATE);
    /* Seed completion only to test publication; no estimator is implemented. */
    motor.identification.result = result;
    motor.identification.state = MFOC_IDENTIFICATION_DONE;
    assert(mfoc_identification_GetResult(&motor.identification, &result) ==
        MFOC_RESULT_OK);
    motor_port_fake.model_update = MFOC_RESULT_INVALID_ARGUMENT;
    assert(mfoc_identification_ApplyResult(&motor.identification,
        &motor.params, &motor.feedback) ==
        MFOC_RESULT_INVALID_ARGUMENT);
    assert(motor.params.rs == cfg.params.rs);
    motor_port_fake.model_update = MFOC_RESULT_OK;
    assert(mfoc_identification_ApplyResult(&motor.identification,
        &motor.params, &motor.feedback) == MFOC_RESULT_OK);
    assert(motor.params.rs == result.rs);
    assert(motor_port_fake.port_inits == 1U);
    ++motor.identification.result.pole_pairs;
    assert(mfoc_identification_ApplyResult(&motor.identification,
        &motor.params, &motor.feedback) ==
        MFOC_RESULT_INVALID_ARGUMENT);
    assert(motor.params.pole_pairs == cfg.params.pole_pairs);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_ELECTRICAL) ==
        MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_IDLE);
    /* Exercise orchestration with synthetic ISR stage completion only. */
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_ELECTRICAL) ==
        MFOC_RESULT_OK);
    assert(mfoc_identification_Run(&motor.identification) ==
        fsm_rt_on_going);
    for (unsigned n = 0U; n < 4U; ++n) {
        motor.identification.stage_done = true;
        fsm_rt_t progress = mfoc_identification_Run(&motor.identification);
        assert(progress == (n == 3U ? fsm_rt_cpl : fsm_rt_on_going));
    }
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_DONE);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_ELECTRICAL) ==
        MFOC_RESULT_OK);
    assert(mfoc_identification_Run(&motor.identification) ==
        fsm_rt_on_going);
    motor_port_fake.time_ms += MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS;
    assert(mfoc_identification_Run(&motor.identification) == fsm_rt_err);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_IDLE);
    /* Bypass unsupported preparation to exercise each rotating leaf boundary. */
    for (unsigned n = 0U; n < 2U; ++n) {
        assert(mfoc_identification_Start(&motor.identification, &motor.params,
            n == 0U ? MFOC_IDENTIFICATION_FLUX :
                MFOC_IDENTIFICATION_INERTIA) == MFOC_RESULT_OK);
        assert(mfoc_identification_GetControl(&motor.identification) ==
            MFOC_IDENTIFICATION_CONTROL_FEEDBACK);
        motor.identification.stage = n == 0U ?
            MFOC_IDENTIFICATION_STAGE_FLUX :
            MFOC_IDENTIFICATION_STAGE_INERTIA;
        motor.mode = MFOC_MOTOR_MODE_SPEED;
        motor.speed.ui = MFOC_HALF; /* Sentinel: normal speed PI must not run. */
        assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
        reads = motor_port_fake.feedback_reads;
        writes = motor_port_fake.writes;
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor_port_fake.feedback_reads == reads + 1U);
        assert(motor_port_fake.writes == writes);
        assert(!motor_port_fake.enabled);
        assert(motor.speed.ui == MFOC_HALF);
        assert(mfoc_identification_GetStatus(&motor.identification) ==
            MFOC_IDENTIFICATION_ERROR);
    }
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_FLUX) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.feedback_step = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    puts("Identification skeleton and Motor model publication passed");
    return 0;
}
