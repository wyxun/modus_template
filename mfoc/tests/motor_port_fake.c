/**
 * @file motor_port_fake.c
 * @brief Test-only implementation of the motor hardware boundary.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_port.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stddef.h>

motor_port_fake_t motor_port_fake;

/** @brief Use milliseconds as test timer ticks. @param ms Milliseconds.
 * @return Test ticks. */
int64_t perfc_convert_ms_to_ticks(uint32_t ms)
{
    return (int64_t)ms;
}

/** @brief Model the perfc deadline using the injected foreground clock.
 * @param period Duration. @param deadline Instance timer slot.
 * @param reload Restart on expiry. @return True when elapsed. */
bool __perfc_is_time_out(int64_t period, int64_t *deadline, bool reload)
{
    if (*deadline == 0) {
        *deadline = motor_port_fake.time_ms + period;
        return false;
    }
    if (motor_port_fake.time_ms < *deadline) { return false; }
    if (reload) { *deadline = motor_port_fake.time_ms + period; }
    return true;
}

#if !defined(MFOC_TEST_REAL_PORT)
/** @brief Record direct Motor current-port initialization.
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

/** @brief Read an injected normalized bus voltage for Motor tests.
 * @param bus Output valid on OK. @return Injected sampling status. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus)
{
    ++motor_port_fake.bus_reads;
    if (motor_port_fake.bus_sample == MFOC_RESULT_OK) {
        *bus = motor_port_fake.bus;
    }
    return motor_port_fake.bus_sample;
}

/** @brief Read one injected normalized frame for Motor integration.
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
 * @param model Unused motor constants. @return OK. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, const mfoc_motor_params_t *model)
{
    (void)cfg;
    (void)model;
    feedback->initialized = true;
    return MFOC_RESULT_OK;
}

/** @brief Inject the outcome of a stopped model refresh.
 * @param feedback Instance. @param model New model. @return Injected status. */
mfoc_result_t mfoc_feedback_SetMotorParamModel(mfoc_feedback_t *feedback,
    const mfoc_motor_params_t *model)
{
    (void)feedback;
    (void)model;
    return motor_port_fake.model_update;
}

/** @brief Record foreground service routing. @param feedback Instance.
 * @return Injected result, or INVALID_STATE before Init. */
mfoc_result_t mfoc_feedback_Run(mfoc_feedback_t *feedback)
{
    if (!feedback->initialized) { return MFOC_RESULT_INVALID_STATE; }
    ++motor_port_fake.feedback_runs;
    return motor_port_fake.feedback_run;
}

/** @brief Require calibration to occur while the fixed field is energized.
 * @param feedback Instance. @param angle Held angle. @return Injected status. */
mfoc_result_t mfoc_feedback_CalibrateZero(mfoc_feedback_t *feedback,
    mfoc_angle_t angle)
{
    assert(feedback->initialized && motor_port_fake.enabled);
    ++motor_port_fake.calibrations;
    motor_port_fake.held_angle = angle;
    return motor_port_fake.calibration;
}

/** @brief Inject feedback to isolate Motor scheduling from its algorithm.
 * @param feedback Instance. @param input Samples. @param output Feedback.
 * @return OK. */
mfoc_result_t mfoc_feedback_Step(mfoc_feedback_t *feedback,
    const mfoc_feedback_input_t *input, mfoc_feedback_output_t *output)
{
    assert(feedback->initialized && input->i_ab_k != NULL);
    ++motor_port_fake.feedback_reads;
    if (motor_port_fake.feedback_step != MFOC_RESULT_OK) {
        return motor_port_fake.feedback_step;
    }
    *output = motor_port_fake.feedback;
    return MFOC_RESULT_OK;
}

#if !defined(MFOC_TEST_REAL_PORT)
/** @brief Record duty before an optional startup, with injected failures.
 * @param duty Command. @param start Whether to enable. @return Test status. */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty, bool start)
{
    ++motor_port_fake.writes;
    motor_port_fake.duty = *duty;
    if (motor_port_fake.write != MFOC_RESULT_OK) {
        return motor_port_fake.write;
    }
    if (start) {
        ++motor_port_fake.enables;
        /* A failed enable may still require shutdown. */
        motor_port_fake.enabled = true;
        return motor_port_fake.enable;
    }
    return MFOC_RESULT_OK;
}

/** @brief Disable only on successful hardware shutdown.
 * @return Injected shutdown status. */
mfoc_result_t mfoc_port_pwm_stop(void)
{
    ++motor_port_fake.disables;
    if (motor_port_fake.step_during_disable != NULL) {
        mfoc_motor_t *motor = motor_port_fake.step_during_disable;
        motor_port_fake.step_during_disable = NULL;
        assert(mfoc_motor_Step(motor) == MFOC_RESULT_INVALID_STATE);
    }
    if (motor_port_fake.disable == MFOC_RESULT_OK) {
        motor_port_fake.enabled = false;
    }
    return motor_port_fake.disable;
}

#endif
