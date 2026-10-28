/**
 * @file hfi_motor_test.c
 * @brief HFI startup and hardware-failure routing.
 * @author Codex
 * @date 2026-10-24
 */
#include "motor_port_fake.h"
#include <assert.h>
#include <stdio.h>

/** @brief Exercise HFI reset/start and Motor failure routing.
 * @return Zero on success. */
int main(void)
{
    mfoc_pi_params_t pi = {MFOC_HALF, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_HALF, MFOC_HALF, MFOC_HALF, MFOC_ONE, 0},
        .limits = {.i_max = MFOC_HALF}, .core = {pi, pi, MFOC_ONE}};
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_HfiStart(NULL) == MFOC_RESULT_NULL);
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_ENABLE_HFI
    /* No measured response must time out and remove injection. */
    mfoc_hfi_t observer = {0};
    mfoc_hfi_output_t estimate = {0};
    const mfoc_ab_t zero = {0};
    mfoc_hfi_Init(&observer);
    mfoc_hfi_Step(&observer, &zero, NULL, &estimate);
    for (unsigned j = 0U; j <
         MFOC_HFI_SAMPLE_HZ * MFOC_HFI_STARTUP_MS / 1000U; ++j) {
        mfoc_hfi_Step(&observer, &zero, &zero, &estimate);
    }
    assert(observer.state == MFOC_HFI_FAILED && !estimate.valid);
    assert(estimate.u_inject_ab.qAlpha == 0);
    assert(estimate.u_inject_ab.qBeta == 0);
    motor.feedback.hfi.state = MFOC_HFI_FAILED;
    motor.feedback.hfi_output.valid = true;
    motor.previous_core_output_valid = true;
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_OK);
    assert(motor.state == MFOC_MOTOR_RUNNING);
    assert(motor.mode == MFOC_MOTOR_MODE_CURRENT && !motor.hold_active);
    assert(!motor.previous_core_output_valid);
    assert(motor.feedback.hfi.state == MFOC_HFI_ACQUIRE);
    assert(!motor.feedback.hfi_output.valid);
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_INVALID_STATE);
    mfoc_dq_t ref = {0};
    assert(mfoc_motor_SetCurrent(&motor, &ref) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetSpeed(&motor, 0) == MFOC_RESULT_INVALID_STATE);
    /* Current acquisition holds the fundamental current at zero. */
    motor_port_fake.feedback = (mfoc_feedback_output_t){
        .acquiring = true,
        .u_inject_ab = {.qAlpha = MFOC_HFI_VOLTAGE}};
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.previous_core_output.m.qAlpha > MFOC_ZERO);
    assert(motor.command.i_ref.qD == 0 && motor.command.i_ref.qQ == 0);
    assert(motor.state == MFOC_MOTOR_RUNNING && motor_port_fake.enabled);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(!motor_port_fake.enabled && motor.state == MFOC_MOTOR_IDLE);
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_OK);
    motor_port_fake.feedback_step = MFOC_RESULT_INVALID_STATE;
    unsigned writes = motor_port_fake.writes;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    assert(motor.faults == MFOC_MOTOR_FAULT_FEEDBACK);
    assert(motor_port_fake.writes == writes);
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    motor_port_fake.feedback_step = MFOC_RESULT_OK;
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_OK);
    motor_port_fake.sample = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    assert(motor.faults == MFOC_MOTOR_FAULT_SAMPLE);
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_INVALID_STATE);
    motor_port_fake.sample = MFOC_RESULT_OK;
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_OK);
    motor_port_fake.write = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
    assert(motor.faults == MFOC_MOTOR_FAULT_PWM);
    motor_port_fake.write = MFOC_RESULT_OK;
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    motor_port_fake.enable = MFOC_RESULT_IO;
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT && !motor_port_fake.enabled);
#else
    assert(mfoc_motor_HfiStart(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(!motor_port_fake.enabled && motor_port_fake.enables == 0U);
#endif
    puts("HFI Motor startup and failure routing passed");
    return 0;
}
