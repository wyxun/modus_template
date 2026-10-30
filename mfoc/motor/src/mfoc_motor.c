/**
 * @file mfoc_motor.c
 * @brief Motor lifecycle and current-loop control.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include "mfoc_port.h"
#include "mfoc_transform.h"
#include "mfoc_motor_config.h"
#if MFOC_IDENTIFICATION_ENABLE
#include "mfoc_modulation.h"
#endif
#include "perf_counter.h"
#include <stddef.h>

/** @brief Compare a caller-validated finite vector without sqrt.
 * @param x First axis. @param y Second axis. @param limit Positive radius.
 * @return True within the circle; false for out-of-range values. */
static bool _mfoc_motor_InCircle(mfoc_scalar_t x, mfoc_scalar_t y,
                                 mfoc_scalar_t limit)
{
    if (x < -limit || x > limit || y < -limit || y > limit) {
        return false;
    }
#if defined(MFOC_NUMERIC_FLOAT)
    float r2 = limit * limit;
    if (r2 == 0.0f) {
        /* Only subnormal-scale configuration takes this fallback. */
        x /= limit;
        y /= limit;
        return x * x + y * y <= 1.0f;
    }
    return x * x + y * y <= r2;
#else
    /* Axis checks exclude INT32_MIN; two positive squares fit int64. */
    return (int64_t)x * x + (int64_t)y * y <= (int64_t)limit * limit;
#endif
}

/** @brief Trip on a control error, prioritizing a failed physical shutdown.
 * @param motor Instance. @param fault Cause. @param cause Original result.
 * @return Shutdown failure if any; otherwise the original error. */
static mfoc_result_t _mfoc_motor_Fail(mfoc_motor_t *motor,
                                     uint32_t fault, mfoc_result_t cause)
{
    mfoc_result_t rc = mfoc_motor_Trip(motor, fault);
    return rc == MFOC_RESULT_OK ? cause : rc;
}

/** @brief Validate fixed motor constants and initialize owned Core state.
 * @param motor Destination instance. @param cfg Initialization input.
 * @return Validation result; destination is unchanged on failure. */
mfoc_result_t mfoc_motor_Init(mfoc_motor_t *motor,
                             const mfoc_motor_cfg_t *cfg)
{
    if (motor == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    const mfoc_motor_params_t *p = &cfg->params;
#if defined(MFOC_NUMERIC_FLOAT)
    if (!mfoc_scalar_is_finite(
            2.0f * cfg->limits.i_max * cfg->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#endif
    if ((cfg->speed_divider != 0U || cfg->openloop.sample_hz != 0U) &&
        cfg->limits.speed_max <= MFOC_ZERO) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    mfoc_motor_t next = {0};
#if MFOC_IDENTIFICATION_ENABLE
    mfoc_identification_Init(&next.identification);
#endif
    mfoc_result_t rc = mfoc_core_Init(&next.core, &cfg->core);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_feedback_Init(&next.feedback, &cfg->feedback, p);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (cfg->speed_divider != 0U) {
        if (!mfoc_scalar_is_finite(cfg->limits.speed_max) ||
            cfg->speed.out_min < -cfg->limits.i_max ||
            cfg->speed.out_max > cfg->limits.i_max) {
            return MFOC_RESULT_OUT_OF_RANGE;
        }
        rc = mfoc_pi_Init(&next.speed, &cfg->speed);
        if (rc != MFOC_RESULT_OK) { return rc; }
    }
    next.speed_divider = cfg->speed_divider;
    if (cfg->openloop.sample_hz != 0U) {
        if (cfg->limits.speed_max > MFOC_ONE) {
            return MFOC_RESULT_OUT_OF_RANGE;
        }
        rc = mfoc_openloop_Init(&next.openloop, &cfg->openloop);
        if (rc != MFOC_RESULT_OK) { return rc; }
        mfoc_sensorless_start_Init(&next.sensorless_start,
                                   cfg->openloop.sample_hz);
    }
    next.params = cfg->params;
    next.limits = cfg->limits;
    next.mode = MFOC_MOTOR_MODE_CURRENT;
    next.state = MFOC_MOTOR_IDLE;
    next.irq_result = MFOC_RESULT_INVALID_STATE;
    rc = mfoc_port_init();
    if (rc != MFOC_RESULT_OK) { return rc; }
    *motor = next;
    return MFOC_RESULT_OK;
}

/** @brief Reset control and start neutral PWM before publishing RUNNING.
 * @param motor Instance. @return Control or hardware status. */
mfoc_result_t mfoc_motor_Start(mfoc_motor_t *motor)
{
    if (motor->state != MFOC_MOTOR_IDLE || motor->faults != 0U) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_result_t rc = mfoc_core_Reset(&motor->core);
    if (rc == MFOC_RESULT_OK) {
        const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
        rc = mfoc_port_pwm(&neutral, true);
    }
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_PWM, rc);
    }
#if MFOC_IDENTIFICATION_ENABLE
    motor->previous_voltage_ab = (mfoc_ab_t){0};
#endif
    motor->state = MFOC_MOTOR_RUNNING;
    return MFOC_RESULT_OK;
}

/** @brief Immediately latch fault and request shutdown, including repeats.
 * @param motor Instance. @param faults Nonzero cause bits.
 * @return Physical shutdown status. */
mfoc_result_t mfoc_motor_Trip(mfoc_motor_t *motor, uint32_t faults)
{
    motor->previous_core_output_valid = false;
    motor->faults |= faults;
    motor->state = MFOC_MOTOR_FAULT;
    motor->i_ref = (mfoc_dq_t){0};
    motor->sensorless_start.state = MFOC_SENSORLESS_START_NONE;
    motor->openloop_active = false;
    mfoc_openloop_Reset(&motor->openloop, (mfoc_angle_t){0U});
    mfoc_result_t rc = mfoc_port_pwm_stop();
    if (rc != MFOC_RESULT_OK) {
        motor->faults |= MFOC_MOTOR_FAULT_PWM;
    }
    return rc;
}

/** @brief Stop PWM and reset control while preserving fault history.
 * @param motor Instance. @return Shutdown or reset status. */
mfoc_result_t mfoc_motor_Stop(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state == MFOC_MOTOR_UNINITIALIZED) {
        return MFOC_RESULT_INVALID_STATE;
    }
    /* Prevent a preempting control ISR from entering Core during reset. */
    motor->state = motor->faults != 0U ?
        MFOC_MOTOR_FAULT : MFOC_MOTOR_IDLE;
    mfoc_result_t rc = mfoc_port_pwm_stop();
#if MFOC_IDENTIFICATION_ENABLE
    mfoc_identification_Stop(&motor->identification);
#endif
    motor->i_ref = (mfoc_dq_t){0};
    mfoc_result_t reset = mfoc_core_Reset(&motor->core);
    if (rc != MFOC_RESULT_OK) { motor->faults |= MFOC_MOTOR_FAULT_PWM; }
    if (reset != MFOC_RESULT_OK) {
        motor->faults |= MFOC_MOTOR_FAULT_MATH;
        if (rc == MFOC_RESULT_OK) { rc = reset; }
    }
    motor->state = motor->faults != 0U ?
        MFOC_MOTOR_FAULT : MFOC_MOTOR_IDLE;
    motor->hold_active = false;
    motor->hold_deadline = 0;
    motor->sensorless_start.state = MFOC_SENSORLESS_START_NONE;
    motor->openloop_active = false;
    mfoc_openloop_Reset(&motor->openloop, (mfoc_angle_t){0U});
    motor->previous_core_output_valid = false;
    if (motor->speed_divider != 0U) {
        mfoc_result_t speed_reset = mfoc_pi_Reset(&motor->speed);
        if (rc == MFOC_RESULT_OK) { rc = speed_reset; }
    }
    motor->speed_ref = MFOC_ZERO;
    motor->speed_count = 0U;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
#if MFOC_IDENTIFICATION_ENABLE
    motor->previous_voltage_ab = (mfoc_ab_t){0};
#endif
    return rc;
}

/** @brief Recover software state after physical shutdown.
 * @param motor Instance. @return Recovery status. */
mfoc_result_t mfoc_motor_ClearFault(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    mfoc_result_t rc = MFOC_RESULT_INVALID_STATE;
    if (motor->state == MFOC_MOTOR_FAULT) {
#if MFOC_IDENTIFICATION_ENABLE
        mfoc_identification_Stop(&motor->identification);
#endif
        rc = mfoc_port_pwm_stop();
        if (rc != MFOC_RESULT_OK) {
            motor->faults |= MFOC_MOTOR_FAULT_PWM;
        }
        if (rc == MFOC_RESULT_OK) { rc = mfoc_core_Reset(&motor->core); }
        if (rc == MFOC_RESULT_OK && motor->speed_divider != 0U) {
            rc = mfoc_pi_Reset(&motor->speed);
        }
        if (rc == MFOC_RESULT_OK) {
            motor->i_ref = (mfoc_dq_t){0};
            mfoc_openloop_Reset(&motor->openloop, (mfoc_angle_t){0U});
            motor->hold_active = false;
            motor->hold_deadline = 0;
            motor->sensorless_start.state = MFOC_SENSORLESS_START_NONE;
            motor->openloop_active = false;
            motor->speed_ref = MFOC_ZERO;
            motor->speed_count = 0U;
            motor->mode = MFOC_MOTOR_MODE_CURRENT;
            motor->faults = 0U;
            motor->state = MFOC_MOTOR_IDLE;
        }
    }
    return rc;
}

/** @brief Validate the current circle before publishing a complete command.
 * @param motor Instance. @param ref Requested dq current.
 * @return OK or rejection; the previous command survives rejection. */
mfoc_result_t mfoc_motor_SetCurrent(mfoc_motor_t *motor,
                                   const mfoc_dq_t *ref)
{
    if (motor == NULL || ref == NULL) { return MFOC_RESULT_NULL; }

    // 允许在空闲或者电流模式下进行控制
    if (motor->hold_active || motor->openloop_active ||
        motor->sensorless_start.state != MFOC_SENSORLESS_START_NONE ||
        (motor->state != MFOC_MOTOR_IDLE &&
        (motor->state == MFOC_MOTOR_RUNNING &&
         motor->mode != MFOC_MOTOR_MODE_CURRENT))) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_dq_t next = *ref;
    if (!_mfoc_motor_InCircle(next.qD, next.qQ, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    motor->i_ref = next;
    if (motor->state == MFOC_MOTOR_IDLE) {
        motor->mode = MFOC_MOTOR_MODE_CURRENT;
    }
    return MFOC_RESULT_OK;
}

/** @brief Forward a serialized bus update to the instance-owned Core.
 * @param motor Instance. @param bus Vdc/Vbase. @return Update status. */
mfoc_result_t mfoc_motor_SetBus(mfoc_motor_t *motor, mfoc_scalar_t bus)
{
    return mfoc_core_SetBus(&motor->core, bus);
}

/** @brief Refresh Core bus coefficients from the published MDI sample.
 * @param motor Instance. @return Sampling/update status; bus retained on error.
 * @note Control ISR while RUNNING, or foreground while IDLE. */
mfoc_result_t mfoc_motor_UpdateBus(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }

    mfoc_scalar_t bus = MFOC_ZERO;
    mfoc_result_t rc = mfoc_port_bus(&bus);
    if (rc != MFOC_RESULT_OK) { return rc; }
    return mfoc_core_SetBus(&motor->core, bus);
}

/** @brief Advance Identification or poll normal-control feedback.
 * @param motor Valid initialized instance. @return Service/stop status. */
mfoc_result_t mfoc_motor_Run(mfoc_motor_t *motor)
{
#if MFOC_IDENTIFICATION_ENABLE
    if ((motor->identification.stage &
         MFOC_IDENTIFICATION_CONTROL_MASK) !=
        MFOC_IDENTIFICATION_CONTROL_INACTIVE) {
        if (motor->state == MFOC_MOTOR_FAULT) {
            mfoc_identification_Stop(&motor->identification);
            return MFOC_RESULT_SAFETY;
        }
        if (motor->state == MFOC_MOTOR_RUNNING) {
            fsm_rt_t progress = mfoc_identification_Run(&motor->identification);
            if (motor->state != MFOC_MOTOR_RUNNING) {
                mfoc_identification_Stop(&motor->identification);
                return MFOC_RESULT_SAFETY;
            }
            if (progress == fsm_rt_err) {
                return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_EXTERNAL,
                                        MFOC_RESULT_SAFETY);
            }
            if (progress == fsm_rt_cpl) { return mfoc_motor_Stop(motor); }
            return MFOC_RESULT_OK;
        }
        return MFOC_RESULT_OK;
    }
#endif
    mfoc_result_t feedback_rc = mfoc_feedback_Run(&motor->feedback);
    /* Run ends the fixed-angle hold; ISR startup budget remains paused. */
    if (motor->hold_active) {
        mfoc_motor_state_t state = motor->state;
        if (state == MFOC_MOTOR_RUNNING &&
            !perfc_is_time_out_ms(MFOC_MOTOR_ALIGN_TIME_MS,
                                  &motor->hold_deadline, false)) {
            return MFOC_RESULT_OK;
        }
        if (state == MFOC_MOTOR_RUNNING &&
            (motor->openloop_active ||
             motor->sensorless_start.state == MFOC_SENSORLESS_START_RAMP)) {
            /* Release OpenLoop ALIGN into its prepared speed ramp. */
            mfoc_openloop_Start(&motor->openloop);
            motor->hold_active = false;
            return MFOC_RESULT_OK;
        }
        /* Capture encoder zero before releasing a calibration hold. */
        mfoc_result_t calibration = state == MFOC_MOTOR_RUNNING ?
            mfoc_feedback_CalibrateZero(&motor->feedback,
                                         motor->openloop.angle) :
            MFOC_RESULT_OK;
        mfoc_result_t rc = mfoc_motor_Stop(motor);
        if (rc != MFOC_RESULT_OK) { return rc; }
        if (calibration != MFOC_RESULT_OK) {
            return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_FEEDBACK,
                                    calibration);
        }
        return state == MFOC_MOTOR_FAULT ?
            MFOC_RESULT_SAFETY : MFOC_RESULT_OK;
    }
    return motor->state == MFOC_MOTOR_RUNNING &&
        (motor->openloop_active ||
         motor->sensorless_start.state == MFOC_SENSORLESS_START_RAMP ||
         motor->sensorless_start.state == MFOC_SENSORLESS_START_ACQUIRE) ?
        MFOC_RESULT_OK : feedback_rc;
}

/** @brief Start HFI injection without a fundamental rotating field.
 * @param motor Stopped instance. @return Validation or PWM startup status. */
mfoc_result_t mfoc_motor_HfiStart(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE || motor->faults != 0U ||
        motor->hold_active) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_result_t rc = mfoc_feedback_HfiStart(&motor->feedback);
    if (rc != MFOC_RESULT_OK) { return rc; }
    motor->i_ref = (mfoc_dq_t){0};
    mfoc_openloop_Reset(&motor->openloop, (mfoc_angle_t){0U});
    motor->previous_core_output_valid = false;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    return mfoc_motor_Start(motor);
}

/** @brief Start a fixed d-current hold and a signed forced-angle ramp.
 * @param motor Instance. @param speed Electrical PU. @param id_pu Current.
 * @return Validation or startup status. */
mfoc_result_t mfoc_motor_OpenLoopStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t id_pu)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE || motor->hold_active) {
        return MFOC_RESULT_INVALID_STATE;
    }
    const mfoc_dq_t ref = {.qD = id_pu, .qQ = MFOC_ZERO};
    if (speed == MFOC_ZERO || speed < -motor->limits.speed_max ||
        speed > motor->limits.speed_max || id_pu <= MFOC_ZERO ||
        !_mfoc_motor_InCircle(ref.qD, ref.qQ, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    mfoc_openloop_t loop = motor->openloop;
    mfoc_openloop_Reset(&loop, (mfoc_angle_t){0U});
    mfoc_result_t rc = mfoc_openloop_SetSpeed(&loop, speed);
    if (rc != MFOC_RESULT_OK) { return rc; }
    motor->i_ref = ref;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    motor->openloop_active = true;
    motor->openloop = loop;
    motor->hold_active = true;
    motor->previous_core_output_valid = false;
    rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        motor->hold_active = false;
        motor->openloop_active = false;
        return rc;
    }
    motor->hold_deadline = 0;
    (void)perfc_is_time_out_ms(MFOC_MOTOR_ALIGN_TIME_MS,
                              &motor->hold_deadline, false);
    return MFOC_RESULT_OK;
}

/** @brief Prepare sensorless hold and ramp before publishing RUNNING.
 * @param motor Instance. @param speed Electrical PU. @param current Current PU.
 * @return Startup status; adaptation is bounded by the requested current. */
mfoc_result_t mfoc_motor_SensorlessStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t current)
{
#if MFOC_FEEDBACK_ENABLE_HFI || \
    (MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_SMO && \
     MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_FLUX)
    (void)motor;
    (void)speed;
    (void)current;
    return MFOC_RESULT_INVALID_STATE;
#else
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE || motor->hold_active ||
        motor->speed_divider == 0U) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (mfoc_abs(speed) < MFOC_SCALAR(MFOC_START_MIN_SPEED_PU) ||
        mfoc_abs(speed) > motor->limits.speed_max ||
        current <= MFOC_ZERO || current > motor->limits.i_max) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    mfoc_scalar_t iq = speed > MFOC_ZERO ? current : -current;
    if (iq < motor->speed.cfg.out_min || iq > motor->speed.cfg.out_max) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    mfoc_openloop_t loop = motor->openloop;
    mfoc_angle_t angle = {speed > MFOC_ZERO ?
        UINT32_C(0xc0000000) : UINT32_C(0x40000000)};
    mfoc_openloop_Reset(&loop, angle);
    mfoc_result_t rc = mfoc_openloop_SetSpeed(&loop, speed);
    if (rc != MFOC_RESULT_OK) { return rc; }
    motor->openloop = loop;
    motor->i_ref = (mfoc_dq_t){MFOC_ZERO, iq};
    motor->speed_ref = speed;
    motor->speed_count = 0U;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    mfoc_sensorless_start_Start(&motor->sensorless_start, speed, current);
    motor->hold_active = true;
    motor->previous_core_output_valid = false;
    motor->hold_deadline = 0;
    (void)perfc_is_time_out_ms(MFOC_MOTOR_ALIGN_TIME_MS,
                              &motor->hold_deadline, false);
    rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        motor->hold_active = false;
        motor->sensorless_start.state = MFOC_SENSORLESS_START_NONE;
    }
    return rc;
#endif
}

/** @brief Start fixed-angle d-axis current alignment from IDLE.
 * @param motor Instance. @param id_pu Current. @param angle_pu Turns.
 * @return Current-command or startup status. */
mfoc_result_t mfoc_motor_AlignStart(mfoc_motor_t *motor,
    mfoc_scalar_t id_pu, mfoc_scalar_t angle_pu)
{
    if (motor->state != MFOC_MOTOR_IDLE || motor->hold_active) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (angle_pu < MFOC_ZERO || angle_pu >= MFOC_ONE) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    const mfoc_dq_t ref = {.qD = id_pu, .qQ = MFOC_ZERO};
    if (id_pu <= MFOC_ZERO ||
        !_mfoc_motor_InCircle(ref.qD, ref.qQ, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    motor->i_ref = ref;
    motor->previous_core_output_valid = false;
    mfoc_openloop_Reset(&motor->openloop,
                         mfoc_angle_from_scalar(angle_pu));
    motor->hold_active = true;
    mfoc_result_t rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        motor->hold_active = false;
        return rc;
    }
    motor->hold_deadline = 0;
    (void)perfc_is_time_out_ms(MFOC_MOTOR_ALIGN_TIME_MS,
                              &motor->hold_deadline, false);
    return MFOC_RESULT_OK;
}

/** @brief Select speed mode while idle or update its running reference.
 * @param motor Instance. @param ref Electrical speed PU.
 * @return Command status. */
mfoc_result_t mfoc_motor_SetSpeed(mfoc_motor_t *motor,
                                  mfoc_scalar_t ref)
{
    if (motor->hold_active || motor->openloop_active ||
        motor->sensorless_start.state != MFOC_SENSORLESS_START_NONE ||
        (motor->state == MFOC_MOTOR_RUNNING &&
         motor->mode != MFOC_MOTOR_MODE_SPEED)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (ref < -motor->limits.speed_max || ref > motor->limits.speed_max) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO || \
    MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_FLUX
    if (motor->state == MFOC_MOTOR_RUNNING &&
        (mfoc_abs(ref) < MFOC_SCALAR(MFOC_START_MIN_SPEED_PU) ||
         (ref > MFOC_ZERO) != (motor->speed_ref > MFOC_ZERO))) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#endif
    if (motor->state == MFOC_MOTOR_IDLE) {
        mfoc_result_t rc = mfoc_pi_Reset(&motor->speed);
        if (rc != MFOC_RESULT_OK) { return rc; }
        motor->speed_count = 0U;
        motor->mode = MFOC_MOTOR_MODE_SPEED;
    }
    motor->speed_ref = ref;
    return MFOC_RESULT_OK;
}

/** @brief Select the angle and current reference for the current step.
 * @param motor Instance. @param feedback Current feedback.
 * @param feedback_rc Observer status. @param input Core input to update.
 * @return OK or the error handled by Motor's existing fault path. */
static mfoc_result_t _mfoc_motor_SelectReference(mfoc_motor_t *motor,
    const mfoc_feedback_output_t *feedback, mfoc_result_t feedback_rc,
    mfoc_core_input_t *input)
{
    if (motor->openloop_active) {
        mfoc_openloop_Step(&motor->openloop);
        input->angle = motor->openloop.angle;
        return MFOC_RESULT_OK;
    }
    if (motor->sensorless_start.state != MFOC_SENSORLESS_START_NONE) {
        mfoc_sensorless_input_t ctx = {
            .feedback = feedback, .feedback_rc = feedback_rc,
            .speed_ref = motor->speed_ref, .hold_active = motor->hold_active};
        mfoc_result_t rc = mfoc_sensorless_start_Step(
            &motor->sensorless_start, &motor->openloop, &ctx, input);
        if (rc != MFOC_RESULT_OK) { return rc; }
        motor->i_ref = input->ref;
        if (ctx.complete) {
            rc = mfoc_pi_Track(&motor->speed, input->ref.qQ,
                               motor->speed_ref, feedback->speed);
            if (rc != MFOC_RESULT_OK) { return rc; }
            motor->speed_count = motor->speed_divider;
            motor->mode = MFOC_MOTOR_MODE_SPEED;
            motor->sensorless_start.state = MFOC_SENSORLESS_START_NONE;
        }
        return MFOC_RESULT_OK;
    }
    if (feedback_rc != MFOC_RESULT_OK) { return feedback_rc; }
    if (motor->hold_active) {
        input->angle = motor->openloop.angle;
        return MFOC_RESULT_OK;
    }
#if MFOC_FEEDBACK_ENABLE_HFI
    if (feedback->acquiring) {
        input->ref = (mfoc_dq_t){0};
        return MFOC_RESULT_OK;
    }
#endif
    if (motor->mode != MFOC_MOTOR_MODE_SPEED ||
        motor->speed_count != 0U) { return MFOC_RESULT_OK; }
    input->ref = (mfoc_dq_t){0};
    mfoc_pi_Step(&motor->speed, motor->speed_ref, feedback->speed,
                  &input->ref.qQ);
    motor->i_ref = input->ref;
    motor->speed_count = motor->speed_divider;
    return MFOC_RESULT_OK;
}

/** @brief Sample current, execute control, and submit the resulting PWM.
 * @param motor Initialized instance. @return Sampling/control/PWM status.
 * @note Feedback consumes i[k] and the last submitted Core output.
 * One RUNNING gate precedes all processing; history updates after PWM. */
mfoc_result_t mfoc_motor_Step(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_uvw_t phase = {0};
    mfoc_result_t rc = mfoc_port_current(&phase);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_SAMPLE, rc);
    }
    mfoc_ab_t i_ab = {0};
    mfoc_clarke(phase.qU, phase.qV, phase.qW, &i_ab);

#if MFOC_IDENTIFICATION_ENABLE
    const mfoc_identification_control_t control =
        (mfoc_identification_control_t)(motor->identification.stage &
            MFOC_IDENTIFICATION_CONTROL_MASK);
#endif
    mfoc_core_output_t output = {0};
#if MFOC_IDENTIFICATION_ENABLE
    mfoc_ab_t voltage_ab = {0};
#endif
    bool speed_countdown = false;
#if MFOC_IDENTIFICATION_ENABLE
    if (control != MFOC_IDENTIFICATION_CONTROL_INACTIVE) {
        mfoc_identification_output_t request = {0};
        mfoc_identification_state_t status =
            mfoc_identification_GetStatus(&motor->identification);
        const bool identification_step =
            status == MFOC_IDENTIFICATION_RUNNING &&
            !motor->identification.stage_done;
        if (identification_step) {
            const mfoc_identification_input_t sample = {
                .i_ab_k = &i_ab,
                .u_ab_previous = motor->previous_voltage_ab};
            mfoc_identification_Step(&motor->identification, &sample, &request);
        }

        switch (control) {
        case MFOC_IDENTIFICATION_CONTROL_VOLTAGE:
            mfoc_svpwm(&request.modulation_ab, &phase);
            voltage_ab.qAlpha = mfoc_mul_wide(request.modulation_ab.qAlpha,
                                              motor->core.bus);
            voltage_ab.qBeta = mfoc_mul_wide(request.modulation_ab.qBeta,
                                             motor->core.bus);
            break;
        default: {
            /* VIRTUAL: the algorithm owns the current target and angle. */
            mfoc_core_input_t core_input = {.i = i_ab};
            if (identification_step) {
                motor->i_ref = request.i_ref;
                motor->identification.angle = request.angle;
            }
            core_input.ref = motor->i_ref;
            core_input.angle = motor->identification.angle;
            mfoc_core_Step(&motor->core, &core_input, &output);
            phase = output.duty;
            voltage_ab.qAlpha = mfoc_mul_wide(output.m.qAlpha, output.bus);
            voltage_ab.qBeta = mfoc_mul_wide(output.m.qBeta, output.bus);
            break;
        }
        }
    } else
#endif
    {
        const mfoc_core_output_t *previous = motor->previous_core_output_valid ?
            &motor->previous_core_output : NULL;
        mfoc_feedback_output_t feedback = {0};
        mfoc_core_input_t core_input = {
            .i = i_ab, .ref = motor->i_ref,
            .angle = motor->openloop.angle};
        if (!motor->hold_active || motor->openloop_active ||
            motor->sensorless_start.state != MFOC_SENSORLESS_START_NONE) {
            mfoc_feedback_input_t feedback_input = {
                .i_ab_k = &i_ab, .previous_core_output = previous};
            rc = mfoc_feedback_Step(&motor->feedback, &feedback_input,
                                    &feedback);
            core_input.angle = feedback.angle;
            core_input.u_inject_ab = feedback.u_inject_ab;
        }
        rc = _mfoc_motor_SelectReference(motor, &feedback, rc, &core_input);
        if (rc != MFOC_RESULT_OK) {
            return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_FEEDBACK, rc);
        }
        mfoc_core_Step(&motor->core, &core_input, &output);
        phase = output.duty;
        speed_countdown =
            !motor->openloop_active &&
            motor->sensorless_start.state == MFOC_SENSORLESS_START_NONE &&
#if MFOC_FEEDBACK_ENABLE_HFI
            !feedback.acquiring &&
#endif
            motor->mode == MFOC_MOTOR_MODE_SPEED &&
            motor->speed_count != 0U;
    }
    rc = mfoc_port_pwm(&phase, false);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_PWM, rc);
    }
#if MFOC_IDENTIFICATION_ENABLE
    motor->previous_voltage_ab = voltage_ab;
    motor->previous_core_output_valid =
        control != MFOC_IDENTIFICATION_CONTROL_VOLTAGE;
    if (motor->previous_core_output_valid) {
        motor->previous_core_output = output;
    }
#else
    motor->previous_core_output = output;
    motor->previous_core_output_valid = true;
#endif
    if (speed_countdown) { motor->speed_count--; }
    return MFOC_RESULT_OK;
}

/** @brief Read lifecycle fields without interrupt masking.
 * @param motor Valid instance. @param status Valid destination.
 * @return None. */
void mfoc_motor_GetStatus(const mfoc_motor_t *motor,
                          mfoc_motor_status_t *status)
{
    status->state = motor->state;
    status->faults = motor->faults;
}
