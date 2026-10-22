/**
 * @file motor_control_test.c
 * @brief Motor sampling, speed-loop cadence and fault integration.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Build a common current-loop or speed-loop configuration.
 * @param divider Zero for current mode. @return Configuration value. */
static mfoc_motor_cfg_t config(uint16_t divider)
{
    mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_SCALAR(0.1f), MFOC_SCALAR(0.2f),
            MFOC_SCALAR(0.2f), MFOC_SCALAR(0.4f), MFOC_ZERO},
        .limits = {MFOC_SCALAR(0.8f)},
        .core = {pi, pi, MFOC_SCALAR(0.55f), MFOC_ONE},
        .speed = {MFOC_HALF, MFOC_SCALAR(0.1f),
            -MFOC_HALF, MFOC_HALF, -MFOC_HALF, MFOC_HALF},
        .speed_divider = divider,
    };
    return cfg;
}

/** @brief Check error reporting without partial initialization.
 * @return None. */
static void initialization(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = config(3U);
    assert(mfoc_motor_Init(NULL, &cfg) == MFOC_RESULT_NULL);
    assert(mfoc_motor_Init(&motor, NULL) == MFOC_RESULT_NULL);
    cfg.speed.out_max = MFOC_ONE;
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OUT_OF_RANGE);
    assert(motor.state == MFOC_MOTOR_UNINITIALIZED);
    cfg = config(3U);
    cfg.speed.i_min = MFOC_ONE;
    assert(mfoc_motor_Init(&motor, &cfg) != MFOC_RESULT_OK);
    cfg = config(0U);
    cfg.speed = (mfoc_pi_params_t){0};
    motor_port_fake.bus_sample = MFOC_RESULT_IO;
    unsigned reads = motor_port_fake.bus_reads;
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(motor_port_fake.bus_reads == reads);
    assert(motor.core.bus == cfg.core.bus);
    assert(mfoc_motor_UpdateBus(&motor) == MFOC_RESULT_IO);
    motor_port_fake.bus_sample = MFOC_RESULT_OK;
    assert(motor.state == MFOC_MOTOR_IDLE);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_runs == 1U);
    motor_port_fake.feedback_run = MFOC_RESULT_IO;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_IDLE);
    motor_port_fake.feedback_run = MFOC_RESULT_OK;
#if defined(MFOC_NUMERIC_FLOAT)
    cfg = config(3U);
    cfg.speed.kp = NAN;
    assert(mfoc_motor_Init(&motor, &cfg) != MFOC_RESULT_OK);
    assert(motor.speed_divider == 0U);
#endif
    mfoc_motor_t saved = motor;
    cfg = config(0U);
    motor_port_fake.port_init = MFOC_RESULT_IO;
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_IO);
    assert(memcmp(&motor, &saved, sizeof(motor)) == 0);
    motor_port_fake.port_init = MFOC_RESULT_OK;
}

/** @brief Check the single-sample current path and ADC failure shutdown.
 * @return None. */
static void current_control(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = config(0U);
    mfoc_dq_t ref = {MFOC_SCALAR(0.2f), MFOC_SCALAR(-0.1f)};
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_HALF) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(motor_port_fake.samples == 0U);
    assert(mfoc_motor_SetCurrent(&motor, &ref) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.current = (mfoc_uvw_t){MFOC_SCALAR(0.1f),
        MFOC_SCALAR(-0.05f), MFOC_SCALAR(-0.05f)};
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.samples == 1U);
    motor_port_fake.current = (mfoc_uvw_t){MFOC_SCALAR(0.1f),
        MFOC_SCALAR(0.1f), MFOC_SCALAR(-0.2f)};
    motor_port_fake.feedback.angle.wBam32 = 0x40000000U;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    unsigned writes = motor_port_fake.writes;
    motor_port_fake.sample = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(motor_port_fake.writes == writes);
    assert(!motor_port_fake.enabled);
    mfoc_motor_status_t status = {0};
    mfoc_motor_GetStatus(&motor, &status);
    assert(status.state == MFOC_MOTOR_FAULT);
    assert((status.faults & MFOC_MOTOR_FAULT_SAMPLE) != 0U);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(motor_port_fake.samples == 3U);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.disable = MFOC_RESULT_SAFETY;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_SAFETY);
    mfoc_motor_GetStatus(&motor, &status);
    uint32_t faults = MFOC_MOTOR_FAULT_SAMPLE | MFOC_MOTOR_FAULT_PWM;
    assert((status.faults & faults) == faults);
    assert(motor_port_fake.enabled);
    motor_port_fake.disable = MFOC_RESULT_OK;
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief PWM write failure trips Motor and invalidates its prior output.
 * @return None. */
static void submission_failure(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = config(0U);
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.previous_core_output_valid);
    motor_port_fake.write = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(motor.faults == MFOC_MOTOR_FAULT_PWM);
    assert(!motor.previous_core_output_valid);
    assert(!motor_port_fake.enabled && motor_port_fake.disables == 1U);
}

/** @brief Check speed PI on steps 1/4/7 and fresh state after restarting.
 * @return None. */
static void speed_control(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = config(3U);
    mfoc_dq_t ref = {0};
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetCurrent(&motor, &ref) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_SCALAR(0.4f)) ==
           MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    for (unsigned step = 0U; step < 7U; ++step) {
        if (step >= 4U) {
            motor_port_fake.feedback.speed = MFOC_SCALAR(0.4f);
        }
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        float expected = step < 3U ? 0.24f : (step < 6U ? 0.28f : 0.08f);
        assert(fabsf(mfoc_to_float(motor.command.i_ref.qQ) -
                     expected) < 0.0004f);
        assert(motor.command.i_ref.qD == MFOC_ZERO);
    }
    mfoc_scalar_t integral = motor.speed.ui;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(motor.speed.ui == integral);
    assert(mfoc_motor_Trip(&motor, MFOC_MOTOR_FAULT_EXTERNAL) ==
           MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_SCALAR(0.4f)) ==
           MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(motor.speed.ui == MFOC_ZERO && motor.speed_count == 0U);
    motor_port_fake.feedback.speed = MFOC_ZERO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(motor.command.i_ref.qQ) - 0.24f) <
           0.0004f);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(motor.speed.ui == MFOC_ZERO && motor.speed_count == 0U);
    assert(motor.speed_ref == MFOC_ZERO);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_SCALAR(0.2f)) ==
           MFOC_RESULT_OK);
    motor_port_fake.feedback.speed = MFOC_ZERO;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(motor.command.i_ref.qQ) - 0.12f) <
           0.0004f);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief Check fixed-angle current without feedback or speed-loop updates.
 * @return None. */
static void alignment(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = config(3U);
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_AlignStart(&motor, MFOC_ONE, MFOC_ZERO) ==
           MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, MFOC_ONE) ==
           MFOC_RESULT_OUT_OF_RANGE);
#if defined(MFOC_NUMERIC_FLOAT)
    assert(mfoc_motor_AlignStart(&motor, NAN, MFOC_ZERO) ==
           MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, INFINITY) ==
           MFOC_RESULT_OUT_OF_RANGE);
#endif
    assert(!motor_port_fake.enabled);
    motor_port_fake.feedback_run = MFOC_RESULT_IO;
    motor_port_fake.time_ms = 100;
    assert(mfoc_motor_AlignStart(&motor, MFOC_SCALAR(0.2f),
        MFOC_SCALAR(0.25f)) == MFOC_RESULT_OK);
    assert(motor_port_fake.enabled);
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, MFOC_ZERO) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_HALF) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_reads == 0U);
    assert(motor.speed.ui == MFOC_ZERO && motor.speed_count == 0U);
    assert(motor.command.i_ref.qD == MFOC_SCALAR(0.2f));
    assert(motor.command.i_ref.qQ == MFOC_ZERO);
    assert(fabsf(mfoc_to_float(motor.previous_core_output.m.qAlpha)) <
           0.0004f);
    assert(motor.previous_core_output.m.qBeta > MFOC_ZERO);
    motor_port_fake.time_ms += MFOC_MOTOR_ALIGN_TIME_MS - 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.enabled && motor_port_fake.feedback_runs == 1U);
    motor_port_fake.time_ms++;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(!motor_port_fake.enabled && !motor.align_active);
    assert(motor_port_fake.feedback_runs == 2U);
    assert(motor.command.i_ref.qD == MFOC_ZERO);
    assert(!motor.previous_core_output_valid);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_IO);
}

/** @brief Check rejected startup, explicit cancellation and ISR faults.
 * @return None. */
static void alignment_stop(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_cfg_t cfg = config(0U);
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    motor_port_fake.write = MFOC_RESULT_IO;
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, MFOC_ZERO) ==
           MFOC_RESULT_IO);
    assert(!motor.align_active && !motor_port_fake.enabled);
    motor_port_fake.write = MFOC_RESULT_OK;
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, MFOC_ZERO) ==
           MFOC_RESULT_OK);
    mfoc_dq_t ref = {0};
    assert(mfoc_motor_SetCurrent(&motor, &ref) ==
           MFOC_RESULT_INVALID_STATE);
    motor_port_fake.step_during_disable = &motor;
    unsigned samples = motor_port_fake.samples;
    unsigned writes = motor_port_fake.writes;
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.step_during_disable == NULL);
    assert(motor_port_fake.samples == samples);
    assert(motor_port_fake.writes == writes);
    assert(!motor.align_active && !motor_port_fake.enabled);
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, MFOC_ZERO) ==
           MFOC_RESULT_OK);
    motor_port_fake.sample = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(!motor_port_fake.enabled);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_SAFETY);
    assert(!motor.align_active && motor.state == MFOC_MOTOR_FAULT);
}

/** @brief Run both numeric-backend integration scenarios.
 * @return Zero on success. */
int main(void)
{
    initialization();
    current_control();
    submission_failure();
    speed_control();
    alignment();
    alignment_stop();
    puts("Motor control cadence and fault shutdown passed");
    return 0;
}
