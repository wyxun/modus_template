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

#if !defined(MFOC_TEST_REAL_PORT)
/** @brief Record direct Driver current-port initialization.
 * @return Injected result. */
mfoc_result_t mfoc_port_init(void)
{
    ++motor_port_fake.port_inits;
    if (motor_port_fake.port_init == MFOC_RESULT_OK &&
        !motor_port_fake.bus_configured) {
        motor_port_fake.bus = MFOC_ONE;
        motor_port_fake.bus_configured = true;
    }
    return motor_port_fake.port_init;
}

/** @brief Read an injected normalized bus voltage for Driver tests.
 * @param bus Output valid on OK. @return Injected sampling status. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus)
{
    ++motor_port_fake.bus_reads;
    if (motor_port_fake.bus_sample == MFOC_RESULT_OK) {
        *bus = motor_port_fake.bus;
    }
    return motor_port_fake.bus_sample;
}

/** @brief Read one injected normalized frame for Driver integration.
 * @param current Output valid on OK. @return Injected sampling status. */
mfoc_result_t mfoc_port_current(mfoc_uvw_t *current)
{
    ++motor_port_fake.samples;
    if (motor_port_fake.sample == MFOC_RESULT_OK) {
        *current = motor_port_fake.current;
    }
    return motor_port_fake.sample;
}

#endif

/** @brief Initialize the test-only feedback service.
 * @param feedback Instance. @param cfg Unused settings.
 * @param pole_pairs Unused motor count. @return OK. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, uint16_t pole_pairs)
{
    (void)cfg;
    (void)pole_pairs;
    feedback->initialized = true;
    return MFOC_RESULT_OK;
}

/** @brief Record foreground service routing. @param feedback Instance.
 * @return Injected result, or INVALID_STATE before Init. */
mfoc_result_t mfoc_feedback_Run(mfoc_feedback_t *feedback)
{
    if (!feedback->initialized) { return MFOC_RESULT_INVALID_STATE; }
    ++motor_port_fake.feedback_runs;
    return motor_port_fake.feedback_run;
}

/** @brief Inject feedback to isolate Driver scheduling from its algorithm.
 * @param feedback Instance. @param input Samples. @param output Feedback.
 * @return OK. */
mfoc_result_t mfoc_feedback_Step(mfoc_feedback_t *feedback,
    const mfoc_feedback_input_t *input, mfoc_feedback_output_t *output)
{
    assert(feedback->initialized && input->i_ab_k != NULL);
    ++motor_port_fake.feedback_reads;
    *output = motor_port_fake.feedback;
    return MFOC_RESULT_OK;
}

/** @brief Return injected readiness. @return Configured status. */
mfoc_result_t mfoc_motor_port_CheckReady(void)
{
    return motor_port_fake.ready;
}

/** @brief Record PWM submissions. @param duty Command.
 * @return Injected write status. */
#if !defined(MFOC_TEST_REAL_PORT)
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty)
{
    assert(motor_port_fake.irq_mask != 0U);
    ++motor_port_fake.writes;
    motor_port_fake.duty = *duty;
    return motor_port_fake.write;
}
#endif

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
