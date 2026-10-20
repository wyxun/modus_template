/**
 * @file motor_port_fake.c
 * @brief Test-only implementation of the motor hardware boundary.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_motor_port.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stddef.h>

motor_port_fake_t motor_port_fake;

/** @brief Return injected readiness. @return Configured status. */
mfoc_result_t mfoc_motor_port_CheckReady(void)
{
    return motor_port_fake.ready;
}

/** @brief Return a fake completed frame.
 * @param current Destination. @return Injected status. */
mfoc_result_t mfoc_motor_port_SampleCurrent(mfoc_uvw_t *current)
{
    if (motor_port_fake.trip_on_sample != NULL) {
        assert(mfoc_motor_Trip(motor_port_fake.trip_on_sample,
            MFOC_MOTOR_FAULT_EXTERNAL) == MFOC_RESULT_OK);
        motor_port_fake.trip_on_sample = NULL;
    }
    *current = motor_port_fake.current;
    return motor_port_fake.sample;
}

/** @brief Record PWM submissions. @param duty Command.
 * @return Injected write status. */
mfoc_result_t mfoc_motor_port_SetDuty(const mfoc_uvw_t *duty)
{
    assert(motor_port_fake.irq_mask != 0U);
    ++motor_port_fake.writes;
    motor_port_fake.duty = *duty;
    return motor_port_fake.write;
}

/** @brief Enable the fake bridge. @return Injected status. */
mfoc_result_t mfoc_motor_port_EnablePwm(void)
{
    ++motor_port_fake.enables;
    motor_port_fake.enabled = true; /* Failure may leave hardware enabled. */
    return motor_port_fake.enable;
}

/** @brief Disable only on successful hardware shutdown.
 * @return Injected shutdown status. */
mfoc_result_t mfoc_motor_port_DisablePwm(void)
{
    ++motor_port_fake.disables;
    if (motor_port_fake.disable == MFOC_RESULT_OK) {
        motor_port_fake.enabled = false;
    }
    return motor_port_fake.disable;
}

/** @brief Model nestable interrupt masking. @return Previous mask. */
uint32_t mfoc_motor_port_EnterCritical(void)
{
    uint32_t previous = motor_port_fake.irq_mask;
    if (previous == 0U && motor_port_fake.trip_before_commit != NULL) {
        mfoc_motor_t *motor = motor_port_fake.trip_before_commit;
        motor_port_fake.trip_before_commit = NULL;
        assert(mfoc_motor_Trip(motor, MFOC_MOTOR_FAULT_EXTERNAL) ==
               MFOC_RESULT_OK);
    }
    motor_port_fake.irq_mask = 1U;
    return previous;
}

/** @brief Restore rather than blindly enable.
 * @param mask Previous interrupt state. @return None. */
void mfoc_motor_port_ExitCritical(uint32_t mask)
{
    motor_port_fake.irq_mask = mask;
}
