/**
 * @file motor_identification_test.c
 * @brief Motor/identification resistance path with an ideal resistive plant.
 * @author Codex
 * @date 2026-10-27
 */
#include "mfoc_motor.h"
#include "mfoc_modulation.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief Initialize, arm Rs and start the Motor. @param motor Instance.
 * @return None. */
static void setup(mfoc_motor_t *motor)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    const mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE, MFOC_HALF, 0},
        .limits = {.i_max = MFOC_SCALAR(0.8f)},
        .core = {pi, pi, MFOC_ONE}};
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_identification_Start(&motor->identification, &cfg.params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    motor_port_fake.feedback_run = MFOC_RESULT_IO;
    motor_port_fake.feedback_step = MFOC_RESULT_IO;
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
}

/** @brief Drive current from the previous PWM and check the complete result.
 * @return None. */
static void resistance(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    for (unsigned n = 0U; n < 22000U && motor_port_fake.enabled; ++n) {
        mfoc_ab_t voltage = {0};
        assert(mfoc_modulation_ModelVoltageAlphaBeta(&motor_port_fake.duty,
            MFOC_ONE, &voltage) == MFOC_RESULT_OK);
        mfoc_scalar_t current = mfoc_from_float(
            mfoc_to_float(voltage.qAlpha) / 1.2f);
        motor_port_fake.current = (mfoc_uvw_t){current,
            mfoc_mul_wide(current, MFOC_SCALAR(-0.5f)),
            mfoc_mul_wide(current, MFOC_SCALAR(-0.5f))};
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        if (n % 20U == 0U) {
            motor_port_fake.time_ms = (int64_t)(n / 20U);
            assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
        }
    }
    assert(!motor_port_fake.enabled);
    assert(motor.state == MFOC_MOTOR_IDLE);
    assert(motor_port_fake.feedback_reads == 0U);
    mfoc_motor_params_t result = {0};
    assert(mfoc_identification_GetResult(&motor.identification, &result) ==
        MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(result.rs) - 1.2f) < 0.01f);
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
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    motor_port_fake.time_ms = MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS + 1U;
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
    const unsigned align_samples = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_ALIGN_MS / 1000U;
    for (unsigned n = 0U; n < align_samples; ++n) {
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    }
    assert(motor.identification.stage_done);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief Invalid final data and unsupported experiments do not energize PWM.
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
            motor_port_fake.time_ms = (int64_t)(n / 20U);
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

/** @brief A feedback-required stage retains the normal Core/PWM path.
 * @return None. */
static void feedback_stage(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor_port_fake.feedback_step = MFOC_RESULT_OK;
    motor_port_fake.feedback_run = MFOC_RESULT_OK;
    motor_port_fake.feedback.speed = MFOC_SCALAR(0.2f);
    const mfoc_dq_t current = {.qQ = MFOC_SCALAR(0.1f)};
    assert(mfoc_motor_SetCurrent(&motor, &current) == MFOC_RESULT_OK);
    /* Test the routing only; production Flux startup remains unavailable. */
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_FLUX;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_reads == 1U);
    assert(motor.previous_core_output_valid);
    assert(motor.identification.work.flux.count == 0U);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_reads == 2U);
    assert(motor.identification.work.flux.count == 1U);
    assert(motor_port_fake.duty.qV != motor_port_fake.duty.qW);
    motor_port_fake.feedback.acquiring = true;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.identification.work.flux.count == 1U);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief Foreground service must retain an armed, not-yet-started request.
 * @return None. */
static void armed_in_idle(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    mfoc_motor_params_t model = {0};
    assert(mfoc_identification_Start(&motor.identification, &model,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_IO);
    assert(mfoc_identification_GetStatus(&motor.identification) ==
        MFOC_IDENTIFICATION_RUNNING);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

/** @brief A finished session stays neutral until foreground stops Motor.
 * @return None. */
static void terminal_handoff(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
    motor.identification.state = MFOC_IDENTIFICATION_DONE;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_reads == 0U);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_IDLE);
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
    resistance();
    timeout();
    stopped_and_faulted();
    handoff();
    invalid_result();
    feedback_stage();
    armed_in_idle();
    terminal_handoff();
    foreground_error();
    puts("Motor Rs identification and timeout passed");
    return 0;
}
