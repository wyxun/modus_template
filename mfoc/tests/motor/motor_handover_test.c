/** @file motor_handover_test.c
 * @brief Sensorless scheduling with injected feedback, no motor plant. */
#include "mfoc_motor.h"
#include "mfoc_motor_config.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <math.h>

static const mfoc_dq_t current_command = {MFOC_HALF, -MFOC_HALF};

static mfoc_motor_t start(int sign, uint32_t sample_hz, float base_hz)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.001f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    const mfoc_motor_cfg_t cfg = {
        .current_base_milliamp = 10000U,
        .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = mfoc_from_float(base_hz),
        .params = {7U, MFOC_HALF, MFOC_HALF, MFOC_HALF, MFOC_ONE, 0},
        .limits = {MFOC_ONE, MFOC_ONE},
        .core = {pi, pi, MFOC_ONE}, .speed = pi, .speed_divider = 20U,
        .openloop = {.sample_hz = sample_hz, .accel = MFOC_SCALAR(0.5f)}};
    motor_port_fake = (motor_port_fake_t){0};
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetCurrent(&motor, &current_command) == MFOC_RESULT_OK);
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
    assert(motor.sensorless_start.remaining ==
        sample_hz * MFOC_START_HANDOVER_MS / 1000U);
    /* At half base speed, one tick advances fb / (2 * fs) turns.
     * Allow binary32 gain rounding in BAM32 counts. */
    double delta = sign * 0.5 * 4294967296.0 * base_hz / sample_hz;
    assert(fabs(motor.openloop.delta - delta) <= 4.0);
    assert(motor.current_cmd.qD == current_command.qD);
    assert(motor.current_cmd.qQ == current_command.qQ);
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

static void handover(int sign, uint32_t sample_hz, float base_hz)
{
    mfoc_motor_t motor = start(sign, sample_hz, base_hz);
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
    uint32_t stable_ticks = (sample_hz * MFOC_START_STABLE_MS + 999U) / 1000U;
    for (unsigned n = 0; n < stable_ticks; ++n) {
        assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_ACQUIRE);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_NONE);
    assert(motor.mode == MFOC_MOTOR_MODE_SPEED);
    assert(motor.openloop.angle.wBam32 == forced.wBam32);
    assert(motor.current_cmd.qD == current_command.qD);
    assert(motor.current_cmd.qQ == current_command.qQ);
    mfoc_scalar_t saved_speed = motor.speed_ref;
    assert(mfoc_motor_SetSpeed(&motor, MFOC_ZERO) ==
        MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_motor_SetSpeed(&motor, -saved_speed) ==
        MFOC_RESULT_OUT_OF_RANGE);
    assert(motor.speed_ref == saved_speed);
    assert(mfoc_motor_SetSpeed(&motor, saved_speed) == MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/* Lose feedback on either side of capture, or cancel a captured start. */
static void interrupted_capture(bool cancel)
{
    mfoc_motor_t motor = start(1, MFOC_MOTOR_RUN_FREQ_HZ, 400.0f);
    motor_port_fake.feedback_step = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(!motor.sensorless_start.captured && motor_port_fake.enabled);
    qualified_step(&motor);
    assert(motor.sensorless_start.captured && motor.sensorless_start.offset != 0);
    const unsigned writes = motor_port_fake.writes;
    if (cancel) {
        assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
        assert(motor.faults == 0U);
    } else {
        motor_port_fake.feedback_step = MFOC_RESULT_IO;
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
        assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
        assert(motor.faults == MFOC_MOTOR_FAULT_FEEDBACK);
        assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    }
    assert(!motor_port_fake.enabled && motor_port_fake.writes == writes);
    assert(mfoc_motor_SensorlessStart(&motor, -MFOC_HALF,
           MFOC_SCALAR(0.04f)) == MFOC_RESULT_OK);
    assert(motor.hold_active && motor_port_fake.enabled);
    assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_RAMP);
    assert(!motor.sensorless_start.captured && motor.sensorless_start.offset == 0);
    assert(motor.speed_iq < MFOC_ZERO && motor.speed_count == 0U);
    assert(!motor.previous_voltage_ab_valid);
    motor_port_fake.feedback_step = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.previous_voltage_ab_valid && motor_port_fake.writes == writes + 2U);
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
    mfoc_motor_t motor = start(sign, MFOC_MOTOR_RUN_FREQ_HZ, 400.0f);
    mfoc_scalar_t initial = mfoc_abs(motor.speed_iq);
    for (unsigned n = 0; n < 2000U; ++n) {
        mfoc_scalar_t previous = mfoc_abs(motor.speed_iq);
        load_step(&motor, sign, 30.0f, 0.0f);
        float change = mfoc_to_float(previous - mfoc_abs(motor.speed_iq));
        assert(change >= 0.0f);
        assert(change <= mfoc_to_float(motor.sensorless_start.iq_step) + 0.000001f);
        if (n + 1U < motor.sensorless_start.iq_ticks) {
            assert(mfoc_abs(motor.speed_iq) == initial);
        }
    }
    mfoc_scalar_t reduced = mfoc_abs(motor.speed_iq);
    assert(reduced < initial && reduced > MFOC_ZERO);
    for (unsigned n = 0; n < 2000U; ++n) {
        load_step(&motor, sign, 110.0f, 0.0f);
    }
    assert(mfoc_abs(motor.speed_iq) > reduced);
    assert(mfoc_abs(motor.speed_iq) <= initial);
    assert((motor.speed_iq > MFOC_ZERO) == (sign > 0));
    mfoc_motor_t lagging = motor;
    mfoc_motor_t leading = motor;
    for (unsigned n = 0; n < 1000U; ++n) {
        load_step(&lagging, sign, 60.0f, 0.1f);
        load_step(&leading, sign, 60.0f, -0.1f);
    }
    assert(mfoc_abs(lagging.speed_iq) > mfoc_abs(leading.speed_iq));
    for (unsigned n = 0; n < 15000U; ++n) {
        load_step(&motor, sign, 0.0f, 0.0f);
        assert(mfoc_abs(motor.speed_iq) > MFOC_ZERO);
        assert(mfoc_abs(motor.speed_iq) <= initial);
    }
}

int main(void)
{
    interrupted_capture(false);
    interrupted_capture(true);
    adjustment(1);
    adjustment(-1);
    for (int sign = -1; sign <= 1; sign += 2) {
        handover(sign, MFOC_MOTOR_RUN_FREQ_HZ, 400.0f);
#if !MFOC_FEEDBACK_ENABLE_HFI
        handover(sign, 16000U, 250.0f);
#endif
    }
    mfoc_motor_t motor = start(1, MFOC_MOTOR_RUN_FREQ_HZ, 400.0f);
    for (unsigned n = 0; n < motor.sensorless_start.timeout_ticks; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_SAFETY);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    return 0;
}
