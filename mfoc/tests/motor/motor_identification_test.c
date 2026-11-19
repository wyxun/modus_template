/**
 * @file motor_identification_test.c
 * @brief Motor/identification resistance path with an ideal resistive plant.
 * @author wyxun
 * @date 2026-10-27
 */
#include "mfoc_motor.h"
#include "mfoc_modulation.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const mfoc_dq_t current_command = {-MFOC_HALF, MFOC_HALF};

/** @brief Return the test drive's public Motor configuration. */
static mfoc_motor_cfg_t motor_config(void)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    return (mfoc_motor_cfg_t){
        .current_base_milliamp = 10000U,
        .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE, MFOC_HALF, 0},
        .limits = {.i_max = MFOC_SCALAR(0.8f), .speed_max = MFOC_ONE},
        .speed = {MFOC_HALF, MFOC_ZERO, -MFOC_HALF, MFOC_HALF,
                  -MFOC_HALF, MFOC_HALF}, .speed_divider = 3U,
        .core = {pi, pi, MFOC_ONE},
        .openloop = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .accel = MFOC_HALF}};
}

/** @brief Initialize, arm Rs and start the Motor. */
static void setup(mfoc_motor_t *motor)
{
    const mfoc_motor_cfg_t cfg = motor_config();
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetCurrent(motor, &current_command) == MFOC_RESULT_OK);
    assert(mfoc_identification_Start(&motor->identification, &cfg.params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    motor_port_fake.position_run = MFOC_RESULT_IO;
    motor_port_fake.feedback_step = MFOC_RESULT_IO;
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
}

/** @brief Read the measured Rs after either outcome of the final PWM stop. */
static void resistance(mfoc_result_t shutdown_status)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor_port_fake.disable = shutdown_status;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    /* Foreground position updates must survive the identification route. */
    assert(motor_port_fake.position_runs == 1U);
    mfoc_scalar_t applied_bus = MFOC_ONE;
    /* Change bus during high-level settling, after the low window closes. */
    const unsigned bus_change = MFOC_IDENTIFICATION_SAMPLE_HZ *
        (MFOC_IDENTIFICATION_RS_ALIGN_MS + MFOC_IDENTIFICATION_RS_SETTLE_MS +
         MFOC_IDENTIFICATION_RS_AVERAGE_MS +
         MFOC_IDENTIFICATION_RS_SETTLE_MS / 2U) / 1000U;
    mfoc_result_t rc = MFOC_RESULT_OK;
    for (unsigned n = 0U; n < 22000U && motor.state == MFOC_MOTOR_RUNNING; ++n) {
        mfoc_ab_t voltage = {0};
        assert(mfoc_modulation_ModelVoltageAlphaBeta(&motor_port_fake.duty,
            applied_bus, &voltage) == MFOC_RESULT_OK);
        mfoc_scalar_t current = mfoc_from_float(
            mfoc_to_float(voltage.qAlpha) / 1.2f);
        motor_port_fake.current = (mfoc_uvw_t){current,
            mfoc_mul_wide(current, MFOC_SCALAR(-0.5f)),
            mfoc_mul_wide(current, MFOC_SCALAR(-0.5f))};
        applied_bus = n < bus_change ? MFOC_SCALAR(0.8f) : MFOC_SCALAR(1.2f);
        assert(mfoc_motor_SetBus(&motor, applied_bus) == MFOC_RESULT_OK);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        if (n % 20U == 0U) {
            motor_port_fake.time_ms = (mfoc_tick_t)(n / 20U);
            rc = mfoc_motor_Run(&motor);
            if (rc != MFOC_RESULT_OK) { break; }
        }
    }
    assert(rc == shutdown_status);
    assert(motor_port_fake.feedback_reads == 0U);
    mfoc_motor_params_t result = {0};
    assert(mfoc_identification_GetResult(&motor.identification, &result) ==
        MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(result.rs) - 1.2f) < 0.01f);
    assert(motor.params.rs == MFOC_HALF);
    if (shutdown_status != MFOC_RESULT_OK) {
        assert(motor_port_fake.enabled && motor.state == MFOC_MOTOR_FAULT);
        assert((motor.faults & MFOC_MOTOR_FAULT_PWM) != 0U);
        assert(mfoc_motor_Start(&motor) == MFOC_RESULT_INVALID_STATE);
        motor_port_fake.disable = MFOC_RESULT_OK;
        assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    }
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_IDLE);
    /* Terminal result stays readable but does not block ordinary startup. */
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief A stalled experiment times out and shuts down PWM.
 * @return None. */
static void timeout(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor_port_fake.time_ms = UINT64_C(0x1fffffff0);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    motor_port_fake.time_ms += MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS - 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.enabled);
    /* A complete 32-bit interval must not disappear from timeout math. */
    motor_port_fake.time_ms += UINT64_C(0x100000000);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor_port_fake.enabled);
    assert(motor.state == MFOC_MOTOR_FAULT);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_ERROR);
}

/** @brief Manual stop cancels capture; a sampling failure disables PWM.
 * @return None. */
static void stopped_and_faulted(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(!motor_port_fake.enabled);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_IDLE);

    setup(&motor);
    motor_port_fake.sample = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_FAULT);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_IDLE);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);

    setup(&motor);
    motor_port_fake.write = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_FAULT);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_IDLE);
}

/** @brief Stage handoff is neutral even if the foreground is delayed.
 * @return None. */
static void handoff(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    const unsigned align_samples = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_ALIGN_MS / 1000U;
    for (unsigned n = 0U; n < align_samples; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.identification.stage_done);
    assert(motor_port_fake.duty.qU != MFOC_HALF);
    const unsigned writes = motor_port_fake.writes;
    for (unsigned n = 0U; n < 3U; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor.identification.work.align_samples == align_samples);
        assert(motor_port_fake.duty.qU == MFOC_HALF);
        assert(motor_port_fake.duty.qV == MFOC_HALF);
        assert(motor_port_fake.duty.qW == MFOC_HALF);
        assert(!motor.previous_core_output_valid);
    }
    assert(motor_port_fake.writes == writes + 3U);
    /* A completed stage wins over a late foreground timeout check. */
    motor_port_fake.time_ms = MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS + 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor.identification.stage == MFOC_IDENTIFICATION_STAGE_RS);
    /* The new stage gets a full deadline. Polling must not renew it. */
    motor_port_fake.time_ms += MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS - 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    ++motor_port_fake.time_ms;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_FAULT);
}

/* Exercise the last settling frame and the first accumulation frame at both
 * voltage levels. The held voltage must disappear on the rejected frame. */
static void rs_overcurrent(mfoc_identification_rs_level_t level,
                           int sign, bool mixed_axes)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    const unsigned align = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_ALIGN_MS / 1000U;
    const unsigned settle = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_SETTLE_MS / 1000U;
    const unsigned samples = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_AVERAGE_MS / 1000U;
    for (unsigned n = 0; n < align; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor.identification.stage == MFOC_IDENTIFICATION_STAGE_RS);
    if (level == MFOC_IDENTIFICATION_RS_HIGH) {
        for (unsigned n = 0; n < settle + samples; ++n) {
            assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        }
    }
    assert(motor.identification.work.rs.level == level);
    const mfoc_scalar_t limit = MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MAX);
    const mfoc_scalar_t edge = sign > 0 ? limit : -limit;
    motor_port_fake.current = (mfoc_uvw_t){edge, -edge / 2, -edge / 2};
    const unsigned accepted = mixed_axes ? settle : settle - 1U;
    for (unsigned n = 0; n < accepted; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.identification.work.rs.count == accepted);
    assert(motor_port_fake.duty.qU != MFOC_HALF);

    if (mixed_axes) {
        /* Each axis fits separately, but their magnitudes sum to > 0.12 PU. */
        const mfoc_scalar_t phase = sign > 0 ? MFOC_SCALAR(0.09375f) :
                                               -MFOC_SCALAR(0.09375f);
        motor_port_fake.current = (mfoc_uvw_t){phase, -phase, MFOC_ZERO};
    } else {
#if defined(MFOC_NUMERIC_FLOAT)
        const mfoc_scalar_t over = nextafterf(edge, sign > 0 ? INFINITY : -INFINITY);
#else
        const mfoc_scalar_t over = sign > 0 ? edge + 1 : edge - 1;
#endif
        motor_port_fake.current = (mfoc_uvw_t){over, -over / 2, -over / 2};
    }
    const unsigned writes = motor_port_fake.writes;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.identification.state == MFOC_IDENTIFICATION_ERROR);
    assert(motor.identification.work.rs.count == accepted);
    assert(motor_port_fake.writes == writes + 1U && motor_port_fake.enabled);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_FAULT);
    mfoc_motor_params_t result = {.rs = MFOC_ONE};
    assert(mfoc_identification_GetResult(&motor.identification, &result) ==
           MFOC_RESULT_INVALID_STATE);
    assert(result.rs == MFOC_ONE);
}

/** @brief A lower Motor limit must also protect direct voltage experiments. */
static void instance_current_limit(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = motor_config();
    cfg.speed_divider = 0U;
    cfg.limits.i_max = MFOC_SCALAR(0.03f);
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.current = (mfoc_uvw_t){
        MFOC_SCALAR(0.0625f), MFOC_SCALAR(-0.03125f), MFOC_SCALAR(-0.03125f)};
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor_port_fake.enabled);
}

/** @brief Invalid final data and missing model prerequisites yield no result.
 * @return None. */
static void invalid_result(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    mfoc_result_t rc = MFOC_RESULT_OK;
    for (unsigned n = 0U; n < 22000U && motor_port_fake.enabled; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        if (n % 20U == 0U) {
            motor_port_fake.time_ms = (mfoc_tick_t)(n / 20U);
            rc = mfoc_motor_Run(&motor);
            if (rc != MFOC_RESULT_OK) { break; }
        }
    }
    assert(rc == MFOC_RESULT_SAFETY && !motor_port_fake.enabled);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_ERROR);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    mfoc_motor_params_t model = {0};
    assert(mfoc_identification_Start(&motor.identification, &model,
        MFOC_IDENTIFICATION_FLUX, MFOC_ONE) == MFOC_RESULT_INVALID_STATE);
    assert(!motor_port_fake.enabled);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_ERROR);
}

/** @brief Verify each virtual stage holds the last dq command across ISR ticks.
 * @param stage Paused experiment stage. @return None. */
static void virtual_stage(mfoc_identification_stage_t stage)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    const mfoc_dq_t current = {MFOC_SCALAR(0.15f), MFOC_SCALAR(-0.1f)};
    const mfoc_angle_t angle = {UINT32_C(0x30000000)};
    motor.identification.stage = stage;
    motor.identification.stage_done = true;
    motor.identification.work.rotating.i_ref = current;
    motor.identification.angle = angle;
    const mfoc_dq_t external = {.qD = MFOC_HALF};
    assert(mfoc_motor_SetCurrent(&motor, &external) ==
           MFOC_RESULT_INVALID_STATE);
    assert(motor.current_cmd.qD == current_command.qD);
    assert(motor.current_cmd.qQ == current_command.qQ);
    mfoc_core_t expected_core = motor.core;
    const mfoc_core_input_t held = {.ref = current, .angle = angle};
    for (unsigned tick = 0U; tick < 3U; ++tick) {
        motor_port_fake.feedback.angle.wBam32 += UINT32_C(0x40000000);
#if MFOC_FEEDBACK_ENABLE_HFI
        motor_port_fake.feedback.acquiring = tick != 0U;
#endif
        mfoc_core_output_t expected = {0};
        mfoc_core_Step(&expected_core, &held, &expected);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor.previous_core_output_valid);
        assert(motor_port_fake.duty.qU == expected.duty.qU);
        assert(motor_port_fake.duty.qV == expected.duty.qV);
        assert(motor_port_fake.duty.qW == expected.duty.qW);
        assert(motor.current_cmd.qD == current_command.qD);
        assert(motor.current_cmd.qQ == current_command.qQ);
        assert(motor.identification.angle.wBam32 == angle.wBam32);
        assert(motor_port_fake.feedback_reads == 0U);
        assert(motor.identification.stage == stage);
        assert(motor.identification.stage_done);
        assert(motor.identification.state == MFOC_IDENTIFICATION_RUNNING);
    }
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief Reject a rotation limit below observable speed before opening PWM. */
static void rotation_speed_limit(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = motor_config();
    cfg.limits.speed_max = MFOC_SCALAR(0.001f);
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    unsigned enables = motor_port_fake.enables;
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_FLUX, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OUT_OF_RANGE);
    assert(motor_port_fake.enables == enables && !motor_port_fake.enabled);
}

/** @brief A pulse cannot combine volt-seconds across a missing interval. */
static void lost_pulse_interval(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_LQ;
    motor.identification.work.inductance = (mfoc_identification_inductance_t){0};
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.identification.work.inductance.count == 1U);
    motor.previous_voltage_ab_valid = false;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.identification.work.inductance.count == 1U);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
}

/** @brief Rotation and L pulses require the compiled experiment time base. */
static void different_profile(void)
{
    mfoc_motor_cfg_t cfg = motor_config();
    cfg.openloop.sample_hz = 16000U;
    mfoc_motor_t motor = {0};
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_ELECTRICAL, MFOC_ONE) == MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_INERTIA, MFOC_ONE) == MFOC_RESULT_OUT_OF_RANGE);
    assert(motor_port_fake.enables == 0U);
}

/** @brief Pending inductance stages select neutral direct PWM.
 * @return None. */
static void inductance_handoff(void)
{
    const mfoc_identification_stage_t stages[] = {
        MFOC_IDENTIFICATION_STAGE_LD, MFOC_IDENTIFICATION_STAGE_LQ};
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor.identification.stage_done = true;
    for (unsigned i = 0U; i < sizeof(stages) / sizeof(stages[0]); ++i) {
        motor.identification.stage = stages[i];
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor_port_fake.duty.qU == MFOC_HALF);
        assert(motor_port_fake.duty.qV == MFOC_HALF);
        assert(motor_port_fake.duty.qW == MFOC_HALF);
        assert(!motor.previous_core_output_valid);
        assert(motor_port_fake.feedback_reads == 0U);
    }
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief A rotating overcurrent bypasses PI and submits neutral immediately.
 * @return None. */
static void rotating_overcurrent(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_FLUX;
    motor.identification.work.rotating.i_ref = current_command;
    motor_port_fake.current = (mfoc_uvw_t){MFOC_HALF, -MFOC_HALF, 0};
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    for (unsigned tick = 0; tick < 3U; ++tick) {
        assert(motor_port_fake.duty.qU == MFOC_HALF);
        assert(motor_port_fake.duty.qV == MFOC_HALF);
        assert(motor_port_fake.duty.qW == MFOC_HALF);
        assert(!motor.previous_core_output_valid);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.current_cmd.qD == current_command.qD);
    assert(motor.current_cmd.qQ == current_command.qQ);
    assert(motor_port_fake.feedback_reads == 0U);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_ERROR);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_FAULT);
}

/** @brief An interrupt preempting foreground shutdown must see a neutral route. */
static void rotation_timeout_before_shutdown(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    mfoc_identification_t *id = &motor.identification;
    id->stage = MFOC_IDENTIFICATION_STAGE_SPINUP;
    id->work.rotating.i_ref = current_command;
    assert(mfoc_identification_Run(id) == MFOC_FSM_ONGOING);
    motor_port_fake.time_ms += MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS;
    assert(mfoc_identification_Run(id) == MFOC_FSM_FAILED);
    /* Model the IRQ between the failed Run and Motor's physical shutdown. */
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor_port_fake.enabled);
}

/** @brief Foreground service must retain an armed, not-yet-started request.
 * @return None. */
static void armed_in_idle(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    const mfoc_dq_t current = {.qD = MFOC_HALF};
    assert(mfoc_motor_SetSpeed(&motor, MFOC_HALF) == MFOC_RESULT_OK);
    mfoc_motor_params_t model = {0};
    assert(mfoc_identification_Start(&motor.identification, &model,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    unsigned char armed[sizeof motor];
    memcpy(armed, &motor, sizeof armed);
    const unsigned writes = motor_port_fake.writes;
    const unsigned enables = motor_port_fake.enables;
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, MFOC_ZERO) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_HALF) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_SensorlessStart(&motor, MFOC_HALF, MFOC_HALF) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_SetCurrent(&motor, &current) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_ZERO) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(memcmp(armed, &motor, sizeof armed) == 0);
    assert(motor_port_fake.writes == writes && motor_port_fake.enables == enables);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_IDLE);
    assert(motor.mode == MFOC_MOTOR_MODE_SPEED && motor.current_cmd.qD == MFOC_ZERO);
    assert(motor.speed_ref == MFOC_HALF);
    unsigned runs = motor_port_fake.position_runs;
    motor_port_fake.time_ms += MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS + 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.position_runs == runs + 1U);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_RUNNING);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_SCALAR(0.25f)) ==
           MFOC_RESULT_INVALID_STATE);
    assert(motor.speed_ref == MFOC_HALF);
    /* Restarting after cancellation starts timing on the first active Run. */
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor.identification.stage == MFOC_IDENTIFICATION_STAGE_ALIGN);
    assert(motor.identification.work.align_samples == 0U);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetCurrent(&motor, &current) == MFOC_RESULT_OK);
}

/** @brief A finished session stays neutral until foreground stops Motor.
 * @return None. */
static void terminal_handoff(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
    motor.identification.state = MFOC_IDENTIFICATION_DONE;
    const mfoc_dq_t current = {.qQ = MFOC_HALF};
    assert(mfoc_motor_SetCurrent(&motor, &current) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_reads == 0U);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_IDLE);
    assert(mfoc_motor_SetCurrent(&motor, &current) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief Foreground handles a terminal Rs error after a neutral ISR frame.
 * @return None. */
static void foreground_error(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor.identification.state = MFOC_IDENTIFICATION_ERROR;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(motor.state == MFOC_MOTOR_RUNNING);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_FAULT);
}

/** @brief Exercise both Motor identification terminal paths. @return Zero. */
int main(void)
{
    resistance(MFOC_RESULT_OK);
    resistance(MFOC_RESULT_IO);
    timeout();
    stopped_and_faulted();
    handoff();
    for (int sign = -1; sign <= 1; sign += 2) {
        rs_overcurrent(MFOC_IDENTIFICATION_RS_LOW, sign, false);
        rs_overcurrent(MFOC_IDENTIFICATION_RS_LOW, sign, true);
        rs_overcurrent(MFOC_IDENTIFICATION_RS_HIGH, sign, false);
        rs_overcurrent(MFOC_IDENTIFICATION_RS_HIGH, sign, true);
    }
    invalid_result();
    virtual_stage(MFOC_IDENTIFICATION_STAGE_SPINUP);
    virtual_stage(MFOC_IDENTIFICATION_STAGE_FLUX);
    virtual_stage(MFOC_IDENTIFICATION_STAGE_INERTIA);
    rotation_speed_limit();
    lost_pulse_interval();
    different_profile();
    inductance_handoff();
    instance_current_limit();
    rotating_overcurrent();
    rotation_timeout_before_shutdown();
    armed_in_idle();
    terminal_handoff();
    foreground_error();
    puts("Motor experiment limits, handoff, shutdown and timeout passed");
    return 0;
}
