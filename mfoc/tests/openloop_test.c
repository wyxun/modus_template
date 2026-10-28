/**
 * @file openloop_test.c
 * @brief Forced-angle ramp and Motor lifecycle integration checks.
 * @author Codex
 * @date 2026-10-23
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include "mfoc_transform.h"
#include "mfoc_trig.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/** @brief Exercise signed BAM32 ramps, wrap and reset. @return None. */
static void phase_ramp(void)
{
    const mfoc_openloop_cfg_t cfg = {20000U, MFOC_SCALAR(400.0f),
        MFOC_SCALAR(0.2f)};
    mfoc_openloop_t loop = {0};
    assert(mfoc_openloop_Init(NULL, &cfg) == MFOC_RESULT_NULL);
    assert(mfoc_openloop_Init(&loop, NULL) == MFOC_RESULT_NULL);
    assert(mfoc_openloop_Init(&loop, &cfg) == MFOC_RESULT_OK);
    assert(loop.delta_step > 0 && loop.delta == 0);
    assert(loop.gain > 85890000 && loop.gain < 85910000);
    assert(loop.delta_step >= 858 && loop.delta_step <= 860);
    assert(mfoc_openloop_SetSpeed(&loop, MFOC_HALF) == MFOC_RESULT_OK);
    int32_t goal = loop.target;
    for (unsigned i = 0U; i < 60000U; ++i) {
        int32_t before = loop.delta;
        uint32_t angle = loop.angle.wBam32;
        mfoc_openloop_Step(&loop);
        assert(loop.delta >= before && loop.delta <= goal);
        assert(loop.angle.wBam32 == angle + (uint32_t)loop.delta);
    }
    assert(loop.delta == goal);
    assert(mfoc_openloop_SetSpeed(&loop, -MFOC_HALF) == MFOC_RESULT_OK);
    for (unsigned i = 0U; i < 120000U; ++i) {
        int32_t before = loop.delta;
        uint32_t angle = loop.angle.wBam32;
        mfoc_openloop_Step(&loop);
        assert(loop.delta <= before && loop.delta >= -goal);
        assert(loop.angle.wBam32 == angle + (uint32_t)loop.delta);
    }
    assert(loop.delta == -goal);
    mfoc_openloop_Reset(&loop, (mfoc_angle_t){UINT32_MAX});
    assert(loop.delta == 0 && loop.target == 0);
    assert(loop.angle.wBam32 == UINT32_MAX);
    mfoc_openloop_t saved = loop;
    mfoc_openloop_cfg_t bad = cfg;
    bad.sample_hz = 0U;
    assert(mfoc_openloop_Init(&loop, &bad) != MFOC_RESULT_OK);
    assert(memcmp(&loop, &saved, sizeof(loop)) == 0);
    bad = cfg;
    bad.accel = MFOC_ZERO;
    assert(mfoc_openloop_Init(&loop, &bad) != MFOC_RESULT_OK);
    bad = cfg;
    bad.accel = MFOC_SCALAR(0.00003f);
    assert(mfoc_openloop_Init(&loop, &bad) == MFOC_RESULT_OUT_OF_RANGE);
    bad = cfg;
    bad.sample_hz = 1U;
    assert(mfoc_openloop_Init(&loop, &bad) == MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_openloop_SetSpeed(&loop, MFOC_SCALAR(2.0f)) ==
        MFOC_RESULT_OUT_OF_RANGE);
    assert(memcmp(&loop, &saved, sizeof(loop)) == 0);
}

/** @brief Build a current-loop Motor with no speed PI. @return Settings. */
static mfoc_motor_cfg_t config(void)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    const mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_SCALAR(0.1f), MFOC_SCALAR(0.2f),
            MFOC_SCALAR(0.2f), MFOC_ONE, MFOC_ZERO},
        .limits = {MFOC_HALF, MFOC_SCALAR(0.75f)},
        .core = {pi, pi, MFOC_ONE},
        .openloop = {20000U, MFOC_SCALAR(400.0f), MFOC_SCALAR(0.2f)},
    };
    return cfg;
}

/** @brief Check hold/ramp, feedback independence and existing shutdown.
 * @return None. */
static void lifecycle(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = config();
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_OpenLoopStart(NULL, MFOC_HALF, MFOC_HALF) ==
        MFOC_RESULT_NULL);
    cfg.openloop.sample_hz = 0U;
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_HALF) ==
        MFOC_RESULT_INVALID_STATE);
    cfg = config();
    cfg.limits.speed_max = MFOC_ZERO;
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OUT_OF_RANGE);
    assert(motor.state == MFOC_MOTOR_IDLE && motor.openloop.gain == 0);
    cfg = config();
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_ONE, MFOC_HALF) ==
        MFOC_RESULT_OUT_OF_RANGE);
    assert(motor.state == MFOC_MOTOR_IDLE);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_ZERO) ==
        MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_HALF) ==
        MFOC_RESULT_OK);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_HALF) ==
        MFOC_RESULT_INVALID_STATE);
    assert(motor.hold_active && motor.start_state == MFOC_START_FORCED);
    assert(motor.mode == MFOC_MOTOR_MODE_CURRENT);
    motor_port_fake.feedback_step = MFOC_RESULT_INVALID_STATE;
    motor_port_fake.feedback_run = MFOC_RESULT_IO;
    for (unsigned i = 0U; i < 10U; ++i) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor.openloop.angle.wBam32 == 0U);
    }
    motor_port_fake.time_ms = MFOC_MOTOR_ALIGN_TIME_MS + 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(!motor.hold_active && motor_port_fake.calibrations == 0U);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.openloop.angle.wBam32 > 0U);
    assert(motor_port_fake.feedback_reads == 11U);
    assert(motor.command.i_ref.qD == MFOC_HALF);
    assert(motor.command.i_ref.qQ == MFOC_ZERO);
    /* A different observer phase must not enter the forced current frame. */
    motor_port_fake.feedback_step = MFOC_RESULT_OK;
    motor_port_fake.feedback.angle = (mfoc_angle_t){UINT32_C(0x40000000)};
    motor_port_fake.current = (mfoc_uvw_t){MFOC_SCALAR(0.1f),
        MFOC_SCALAR(-0.05f), MFOC_SCALAR(-0.05f)};
    for (unsigned i = 0U; i < 1000U; ++i) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    mfoc_ab_t ab = {0};
    mfoc_clarke(motor_port_fake.current.qU, motor_port_fake.current.qV,
        motor_port_fake.current.qW, &ab);
    mfoc_scalar_t sine = MFOC_ZERO;
    mfoc_scalar_t cosine = MFOC_ZERO;
    mfoc_angle_sincos(motor.openloop.angle, &sine, &cosine);
    mfoc_dq_t expected = {0};
    mfoc_park_cached(&ab, sine, cosine, &expected);
    assert(motor.previous_core_output.i.qD == expected.qD);
    assert(motor.previous_core_output.i.qQ == expected.qQ);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_HALF) != MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(!motor.hold_active && motor.openloop.delta == 0);
    assert(motor.mode == MFOC_MOTOR_MODE_CURRENT);
    assert(mfoc_motor_OpenLoopStart(&motor, -MFOC_HALF, MFOC_HALF) ==
        MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(!motor.hold_active && motor.openloop.target == 0);
    assert(mfoc_motor_OpenLoopStart(&motor, -MFOC_HALF, MFOC_HALF) ==
        MFOC_RESULT_OK);
    motor_port_fake.sample = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    assert(motor.openloop.delta == 0 && motor.openloop.target == 0);
    motor_port_fake.sample = MFOC_RESULT_OK;
    motor_port_fake.write = MFOC_RESULT_IO;
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_HALF) ==
        MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor.hold_active);
    assert(!motor_port_fake.enabled && motor.openloop.target == 0);
}

/** @brief Run forced-angle checks in both scalar backends. @return Zero. */
int main(void)
{
    phase_ramp();
    lifecycle();
    puts("OpenLoop signed ramp, hold, shadow feedback and stop passed");
    return 0;
}
