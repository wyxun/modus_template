/**
 * @file motor_run_test.c
 * @brief Motor lifecycle, fault containment and current-control integration.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stdio.h>

/** @brief Prepare a current-controlled motor with a known inactive port.
 * @param motor Destination. @return None. */
static void setup(mfoc_motor_t *motor)
{
    mfoc_pi_params_t pi = {MFOC_SCALAR(0.5f), MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_SCALAR(0.1f), MFOC_SCALAR(0.2f),
            MFOC_SCALAR(0.2f), MFOC_SCALAR(0.4f), MFOC_ZERO},
        .limits = {MFOC_SCALAR(0.8f)},
        .core = {pi, pi, MFOC_SCALAR(0.55f), MFOC_ONE},
    };
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(motor, &cfg) == MFOC_RESULT_OK);
}

/** @brief Prove fault state rejects PWM and normal Stop retains the latch.
 * @param motor Faulted instance. @param fault Expected bit.
 * @return None. */
static void check_fault(mfoc_motor_t *motor, uint32_t fault)
{
    mfoc_motor_status_t status = {0};
    assert(mfoc_motor_GetStatus(motor, &status) == MFOC_RESULT_OK);
    assert(status.state == MFOC_MOTOR_FAULT);
    assert((status.faults & fault) != 0U);
    assert(!motor_port_fake.enabled);
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Stop(motor) == MFOC_RESULT_OK);
    assert(motor->state == MFOC_MOTOR_FAULT);
    assert(mfoc_motor_ClearFault(motor) == MFOC_RESULT_OK);
    assert(motor->state == MFOC_MOTOR_IDLE);
    assert(!motor_port_fake.enabled);
}

/** @brief Exercise the complete port-bound current-control path.
 * @return Zero if every assertion passes. */
int main(void)
{
    mfoc_motor_t motor = {0};
    mfoc_motor_input_t input = {.angle_valid = true};
    mfoc_core_output_t output = {0};
    mfoc_dq_t ref = {MFOC_SCALAR(0.2f), MFOC_SCALAR(-0.1f)};
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_INVALID_STATE);
    setup(&motor);
    assert(mfoc_motor_Step(&motor, &input, &output) ==
           MFOC_RESULT_INVALID_STATE);
    assert(motor_port_fake.writes == 0U);
    assert(mfoc_motor_SetCurrent(&motor, &ref) == MFOC_RESULT_OK);
    mfoc_dq_t excessive = {MFOC_SCALAR(0.7f), MFOC_SCALAR(0.7f)};
    assert(mfoc_motor_SetCurrent(&motor, &excessive) ==
           MFOC_RESULT_OUT_OF_RANGE);
    assert(motor.command.i_ref.qD == ref.qD);
    assert(motor.command.i_ref.qQ == ref.qQ);
    motor_port_fake.ready = MFOC_RESULT_SAFETY;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_SAFETY);
    assert(motor_port_fake.enables == 0U);
    motor_port_fake.ready = MFOC_RESULT_OK;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.enabled);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Step(&motor, &input, &output) == MFOC_RESULT_OK);
    assert(output.u.qD > MFOC_ZERO && output.u.qQ < MFOC_ZERO);
    assert(motor_port_fake.duty.qU == output.duty.qU);
    assert(mfoc_motor_SetBus(&motor, MFOC_SCALAR(0.9f)) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetBus(&motor, MFOC_ZERO) != MFOC_RESULT_OK);
    assert(motor.state == MFOC_MOTOR_RUNNING);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(motor.command.i_ref.qD == MFOC_ZERO);
    assert(motor.core.d.ui == MFOC_ZERO && motor.core.q.ui == MFOC_ZERO);
    assert(!motor_port_fake.enabled);

    setup(&motor);
    motor_port_fake.enable = MFOC_RESULT_IO;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_IO);
    check_fault(&motor, MFOC_MOTOR_FAULT_PWM);

    setup(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.write = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor, &input, &output) == MFOC_RESULT_IO);
    check_fault(&motor, MFOC_MOTOR_FAULT_PWM);

    setup(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.disable = MFOC_RESULT_IO;
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_IO);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT);
    motor_port_fake.disable = MFOC_RESULT_OK;
    motor_port_fake.ready = MFOC_RESULT_SAFETY;
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_SAFETY);
    motor_port_fake.ready = MFOC_RESULT_OK;
    check_fault(&motor, MFOC_MOTOR_FAULT_PWM);

    setup(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    unsigned writes = motor_port_fake.writes;
    motor_port_fake.trip_before_commit = &motor;
    assert(mfoc_motor_Step(&motor, &input, &output) == MFOC_RESULT_SAFETY);
    assert(motor_port_fake.writes == writes);
    check_fault(&motor, MFOC_MOTOR_FAULT_EXTERNAL);

    setup(&motor);
    motor_port_fake.irq_mask = 1U;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.irq_mask == 1U);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.irq_mask == 1U);
    motor_port_fake.irq_mask = 0U;
    assert(motor_port_fake.irq_mask == 0U);
    puts("Motor control, startup rejection and fault recovery passed");
    return 0;
}
