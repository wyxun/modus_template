/**
 * @file motor_start_irq_test.c
 * @brief Verify a synchronous Port that can enable the control IRQ.
 * @author Codex
 * @date 2026-11-25
 */
#include "motor_port_fake.h"
#include <assert.h>
#include <stdio.h>

/** @brief Exercise an early IRQ, its fault recovery and a Port start error.
 * @return Zero after all cases pass. */
int main(void)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    const mfoc_motor_cfg_t cfg = {
        .current_base_milliamp = 10000U,
        .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE, MFOC_HALF, MFOC_ZERO},
        .limits = {.i_max = MFOC_HALF},
        .core = {pi, pi, MFOC_ONE},
    };
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);

    motor_port_fake.step_during_enable = &motor;
    motor_port_fake.sample = MFOC_RESULT_IO;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_SAFETY);
    assert(motor_port_fake.step_during_enable_result == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT);
    assert(motor.faults == MFOC_MOTOR_FAULT_SAMPLE);
    assert(!motor_port_fake.enabled && motor_port_fake.disables == 1U);

    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    motor_port_fake.step_during_enable = &motor;
    motor_port_fake.sample = MFOC_RESULT_OK;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.step_during_enable_result == MFOC_RESULT_OK);
    assert(motor.state == MFOC_MOTOR_RUNNING && motor.faults == 0U);
    /* A successful ISR during enable owns the first published output. */
    assert(motor.previous_core_output_valid);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);

    motor_port_fake.enable = MFOC_RESULT_IO;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT);
    assert(motor.faults == MFOC_MOTOR_FAULT_PWM);
    assert(!motor_port_fake.enabled);
    puts("Motor start IRQ fault and early control output passed");
    return 0;
}
