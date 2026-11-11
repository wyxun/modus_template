/**
 * @file motor_port_fake.c
 * @brief Test-only implementation of the motor hardware boundary.
 * @author wyxun
 * @date 2026-10-20
 */
#include "mfoc_port.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stddef.h>

motor_port_fake_t motor_port_fake;

#if !defined(MFOC_TEST_REAL_PORT)
/** @brief Record direct Motor current-port initialization.
 * @param cfg Common unit configuration. @return Injected result. */
mfoc_result_t mfoc_port_init(const mfoc_port_cfg_t *cfg)
{
    assert(cfg != NULL);
    motor_port_fake.port_cfg = *cfg;
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

/** @brief Read the full injected foreground clock. @return Milliseconds. */
mfoc_tick_t mfoc_port_tick(void) { return motor_port_fake.time_ms; }

/** @brief Give the injected clock scale. @return Ticks per second. */
uint32_t mfoc_port_tick_frequency(void) { return 1000U; }

/** @brief Record foreground position refresh. @return Injected result. */
mfoc_result_t mfoc_port_position_run(void)
{
    ++motor_port_fake.position_runs;
    return motor_port_fake.position_run;
}

/** @brief Supply one coherent electrical position result.
 * @param angle Electrical BAM32. @param speed Electrical speed PU.
 * @return Injected result; failure preserves both outputs. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    ++motor_port_fake.position_reads;
    if (motor_port_fake.position_step == MFOC_RESULT_OK) {
        *angle = motor_port_fake.feedback.angle;
        *speed = motor_port_fake.feedback.speed;
    }
    return motor_port_fake.position_step;
}

/** @brief Require calibration while the fixed field is energized.
 * @param angle Held electrical angle. @return Injected status. */
mfoc_result_t mfoc_port_position_calibrate_zero(mfoc_angle_t angle)
{
    assert(motor_port_fake.enabled);
    ++motor_port_fake.calibrations;
    motor_port_fake.held_angle = angle;
    return motor_port_fake.calibration;
}

#endif

/** @brief Initialize the test-only feedback service.
 * @param feedback Instance. @param cfg Captured settings.
 * @param model Motor constants. @return OK, or INVALID_ARGUMENT for a model
 * the real Feedback would refuse through its enabled observers.
 * @note Motor delegates model validation to Feedback, so this fake repeats the
 * same acceptance checks; otherwise motor parameter cases stop being covered. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, const mfoc_motor_params_t *model)
{
    motor_port_fake.feedback_cfg = *cfg;
    if (feedback == NULL || model == NULL ||
        model->pole_pairs == 0U ||
        !mfoc_scalar_is_finite(model->rs) || model->rs <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(model->ld) || model->ld <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(model->lq) || model->lq <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(model->flux) || model->flux <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(model->inertia) || model->inertia < MFOC_ZERO) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    feedback->initialized = true;
    return MFOC_RESULT_OK;
}

/** @brief Reset HFI state before the Motor's normal startup.
 * @param feedback Initialized instance. @return Compiled capability status. */
mfoc_result_t mfoc_feedback_HfiStart(mfoc_feedback_t *feedback)
{
#if MFOC_FEEDBACK_ENABLE_HFI
    feedback->hfi = (mfoc_hfi_t){.state = MFOC_HFI_ACQUIRE};
    feedback->hfi_output = (mfoc_hfi_output_t){0};
    return MFOC_RESULT_OK;
#else
    (void)feedback;
    return MFOC_RESULT_INVALID_STATE;
#endif
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
/** @brief Record duty without changing gate state.
 * @param duty Command. @return Injected write status. */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty)
{
    ++motor_port_fake.writes;
    motor_port_fake.duty = *duty;
    if (motor_port_fake.write != MFOC_RESULT_OK) {
        return motor_port_fake.write;
    }
    return MFOC_RESULT_OK;
}

/** @brief Load initial duty before enabling phase outputs.
 * @param initial_duty Startup command. @return Injected write/enable status. */
mfoc_result_t mfoc_port_pwm_start(const mfoc_uvw_t *initial_duty)
{
    mfoc_result_t rc = mfoc_port_pwm(initial_duty);
    if (rc != MFOC_RESULT_OK) { return rc; }
    ++motor_port_fake.enables;
    /* A failed enable may still require shutdown. */
    motor_port_fake.enabled = true;
    return motor_port_fake.enable;
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
