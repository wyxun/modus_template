/** @file motor_handover_test.c
 * @brief Sensorless scheduling with injected feedback, no motor plant. */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include <assert.h>

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
    assert(mfoc_motor_SensorlessStart(&motor, sign * MFOC_HALF,
        MFOC_SCALAR(0.04f)) == MFOC_RESULT_OK);
    assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_RAMP && motor.hold_active);
    motor_port_fake.feedback_step = MFOC_RESULT_INVALID_STATE;
    mfoc_angle_t held = motor.openloop.angle;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.openloop.angle.wBam32 == held.wBam32);
    motor_port_fake.time_ms = MFOC_MOTOR_ALIGN_TIME_MS + 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    unsigned ticks = 0U;
    while (motor.sensorless_start.state == MFOC_SENSORLESS_START_RAMP && ticks < 50000U) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        ++ticks;
    }
    assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_ACQUIRE);
    assert(motor.sensorless_start.remaining == motor.sensorless_start.timeout_ticks);
    return motor;
}

static void qualified_step(mfoc_motor_t *motor)
{
    mfoc_openloop_t next = motor->openloop;
    mfoc_openloop_Step(&next);
    motor_port_fake.feedback_step = MFOC_RESULT_OK;
    motor_port_fake.feedback.speed = motor->speed_ref;
    motor_port_fake.feedback.angle = mfoc_angle_add_scalar(next.angle,
        MFOC_SCALAR(-0.01f));
    assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
}

static void handover(int sign)
{
    mfoc_motor_t motor = start(sign);
    qualified_step(&motor);
    assert(motor.sensorless_start.captured);
    assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_ACQUIRE);
    assert(motor.sensorless_start.stable == 0U);
    mfoc_angle_t forced = motor.openloop.angle;
    int32_t previous = motor.sensorless_start.offset;
    while (motor.sensorless_start.offset != 0) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor.sensorless_start.offset >= 0 && motor.sensorless_start.offset < previous);
        previous = motor.sensorless_start.offset;
    }
    assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_ACQUIRE);
    motor_port_fake.feedback.speed = MFOC_ZERO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.sensorless_start.stable == 0U);
    motor_port_fake.feedback.speed = motor.speed_ref;
    for (unsigned n = 0; n < motor.sensorless_start.stable_ticks; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_NONE);
    assert(motor.mode == MFOC_MOTOR_MODE_SPEED);
    assert(motor.openloop.angle.wBam32 == forced.wBam32);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/* Exercise adaptation through the real ISR with a controlled load angle. */
static void load_step(mfoc_motor_t *motor, int sign, float degrees,
    float lag)
{
    mfoc_openloop_t next = motor->openloop;
    mfoc_openloop_Step(&next);
    motor_port_fake.feedback_step = MFOC_RESULT_OK;
    motor_port_fake.feedback.angle = mfoc_angle_add_scalar(next.angle,
        mfoc_from_float((float)sign * (90.0f - degrees) / 360.0f));
    motor_port_fake.feedback.speed = motor->speed_ref -
        mfoc_from_float((float)sign * lag);
    assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
}

static void adjustment(int sign)
{
    mfoc_motor_t motor = start(sign);
    mfoc_scalar_t initial = mfoc_abs(motor.i_ref.qQ);
    for (unsigned n = 0; n < 2000U; ++n) {
        mfoc_scalar_t previous = mfoc_abs(motor.i_ref.qQ);
        load_step(&motor, sign, 30.0f, 0.0f);
        float change = mfoc_to_float(previous - mfoc_abs(motor.i_ref.qQ));
        assert(change >= 0.0f);
        assert(change <= mfoc_to_float(motor.sensorless_start.iq_step) + 0.000001f);
        if (n + 1U < motor.sensorless_start.iq_ticks) {
            assert(mfoc_abs(motor.i_ref.qQ) == initial);
        }
    }
    mfoc_scalar_t reduced = mfoc_abs(motor.i_ref.qQ);
    assert(reduced < initial && reduced > MFOC_ZERO);
    for (unsigned n = 0; n < 2000U; ++n) {
        load_step(&motor, sign, 110.0f, 0.0f);
    }
    assert(mfoc_abs(motor.i_ref.qQ) > reduced);
    assert(mfoc_abs(motor.i_ref.qQ) <= initial);
    assert((motor.i_ref.qQ > MFOC_ZERO) == (sign > 0));
    mfoc_motor_t lagging = motor;
    mfoc_motor_t leading = motor;
    for (unsigned n = 0; n < 1000U; ++n) {
        load_step(&lagging, sign, 60.0f, 0.1f);
        load_step(&leading, sign, 60.0f, -0.1f);
    }
    assert(mfoc_abs(lagging.i_ref.qQ) > mfoc_abs(leading.i_ref.qQ));
    for (unsigned n = 0; n < 15000U; ++n) {
        load_step(&motor, sign, 0.0f, 0.0f);
        assert(mfoc_abs(motor.i_ref.qQ) > MFOC_ZERO);
        assert(mfoc_abs(motor.i_ref.qQ) <= initial);
    }
}

int main(void)
{
    adjustment(1);
    adjustment(-1);
    handover(1);
    handover(-1);
    mfoc_motor_t motor = start(1);
    for (unsigned n = 0; n < motor.sensorless_start.timeout_ticks; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_SAFETY);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    return 0;
}
