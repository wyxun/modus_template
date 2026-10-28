/**
 * @file motor_handover_test.c
 * @brief Handover scheduling with independently supplied observer samples.
 * @author Codex
 * @date 2026-10-28
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stdio.h>

/** @brief Start from rest with a signed target. @return Motor instance. */
static mfoc_motor_t start(int sign)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.001f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    const mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_HALF, MFOC_HALF, MFOC_HALF, MFOC_ONE, 0},
        .limits = {MFOC_ONE, MFOC_ONE},
        .core = {pi, pi, MFOC_ONE}, .speed = pi, .speed_divider = 20U,
        .openloop = {20000U, MFOC_SCALAR(400.0f), MFOC_SCALAR(0.5f)}};
    motor_port_fake = (motor_port_fake_t){0};
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_SensorlessStart(&motor,
        sign * MFOC_HALF, MFOC_SCALAR(0.04f)) == MFOC_RESULT_OK);
    assert(motor.start_state == MFOC_START_WAIT && motor.hold_active);
    assert(motor.command.i_ref.qD == MFOC_ZERO);
    assert(motor.command.i_ref.qQ == sign * MFOC_SCALAR(0.04f));
    motor_port_fake.feedback_step = MFOC_RESULT_INVALID_STATE;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    motor_port_fake.time_ms = MFOC_MOTOR_ALIGN_TIME_MS + 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    return motor;
}

/** @brief Follow the next forced angle with a specified rotor-frame gap.
 * @param motor Instance. @param gap Control minus rotor, electrical turns.
 * @return None. This supplies geometry, not a simulated mechanical plant. */
static void sample(mfoc_motor_t *motor, float gap)
{
    mfoc_openloop_t next = motor->openloop;
    mfoc_openloop_Step(&next);
    motor_port_fake.feedback_step = MFOC_RESULT_OK;
    motor_port_fake.feedback.speed = motor->speed_ref;
    motor_port_fake.feedback.angle = mfoc_angle_add_scalar(next.angle,
        mfoc_from_float(-gap));
}

/** @brief Check both directions, light load and a continuous capture.
 * @param sign Direction. @return None. */
static void handover(int sign)
{
    mfoc_motor_t motor = start(sign);
    for (unsigned n = 0; n < 40000U &&
         motor.start_state == MFOC_START_WAIT; ++n) {
        sample(&motor, -(float)sign * 0.25f);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.start_state == MFOC_START_REDUCE);
    mfoc_scalar_t before = mfoc_abs(motor.command.i_ref.qQ);
    for (unsigned n = 0; n < 1000U; ++n) {
        sample(&motor, -(float)sign * 0.1f);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(mfoc_abs(motor.command.i_ref.qQ) < before);
    for (unsigned n = 0; n < 1000U &&
         motor.start_state == MFOC_START_REDUCE; ++n) {
        sample(&motor, -(float)sign * 0.01f);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.start_state == MFOC_START_BLEND);
    assert(motor.openloop.angle.wBam32 ==
        motor_port_fake.feedback.angle.wBam32 + (uint32_t)motor.start.offset);
    before = motor.command.i_ref.qQ;
    for (unsigned n = 0; n < 1000U &&
         motor.start_state == MFOC_START_BLEND; ++n) {
        motor_port_fake.feedback.angle.wBam32 +=
            (uint32_t)motor.openloop.target;
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.start_state == MFOC_START_NONE);
    assert(motor.mode == MFOC_MOTOR_MODE_SPEED);
    assert(mfoc_abs(motor.command.i_ref.qQ - before) < MFOC_SCALAR(0.0002f));
    assert(mfoc_motor_SetSpeed(&motor, -motor.speed_ref) != MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief Reject loss of feedback and a clipped speed-PI handoff.
 * @return None. */
static void failures(void)
{
    mfoc_motor_t motor = start(1);
    motor.start_state = MFOC_START_REDUCE;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    motor = start(1);
    motor.start_state = MFOC_START_BLEND;
    motor.start.offset = 1;
    motor_port_fake.feedback_step = MFOC_RESULT_OK;
    motor_port_fake.feedback.speed = motor.speed_ref;
    motor.speed.cfg.i_min = MFOC_ZERO;
    motor.speed.cfg.i_max = MFOC_ZERO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_SAFETY);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    motor = start(1);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(motor.start_state == MFOC_START_NONE && !motor_port_fake.enabled);
}

/** @brief Unqualified feedback must expire rather than run indefinitely.
 * @return Zero. */
int main(void)
{
    handover(1);
    handover(-1);
    failures();
    mfoc_motor_t motor = start(1);
    for (unsigned n = 0; n < 200000U &&
         motor.state == MFOC_MOTOR_RUNNING; ++n) {
        mfoc_result_t rc = mfoc_motor_Step(&motor);
        assert(rc == MFOC_RESULT_OK || rc == MFOC_RESULT_SAFETY);
    }
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    puts("Handover direction, reduction, tracking and timeout passed");
    return 0;
}
