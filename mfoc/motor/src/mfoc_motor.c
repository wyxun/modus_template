/**
 * @file mfoc_motor.c
 * @brief Motor lifecycle and current-loop control.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include "mfoc_port.h"
#include "mfoc_transform.h"
#include "perf_counter.h"
#include <stddef.h>

/** @brief Check finite PI parameters and a zero-output reset domain.
 * @param p PI configuration. @return True when valid at the Motor boundary. */
static bool _mfoc_motor_PiValid(const mfoc_pi_params_t *p)
{
    return mfoc_scalar_is_finite(p->kp) &&
        mfoc_scalar_is_finite(p->ki_ts) &&
        mfoc_scalar_is_finite(p->out_min) &&
        mfoc_scalar_is_finite(p->out_max) &&
        mfoc_scalar_is_finite(p->i_min) &&
        mfoc_scalar_is_finite(p->i_max) &&
        p->out_min <= MFOC_ZERO && p->out_max >= MFOC_ZERO &&
        p->i_min <= MFOC_ZERO && p->i_max >= MFOC_ZERO;
}

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
    if (p->pole_pairs == 0U ||
        !mfoc_scalar_is_finite(p->rs) || p->rs <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(p->ld) || p->ld <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(p->lq) || p->lq <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(p->flux) || p->flux <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(p->inertia) || p->inertia < MFOC_ZERO ||
        !mfoc_scalar_is_finite(cfg->limits.i_max) ||
        cfg->limits.i_max <= MFOC_ZERO ||
        !_mfoc_motor_PiValid(&cfg->core.d) ||
        !_mfoc_motor_PiValid(&cfg->core.q)) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
#if defined(MFOC_NUMERIC_FLOAT)
    if (!mfoc_scalar_is_finite(
            2.0f * cfg->limits.i_max * cfg->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#endif
    mfoc_motor_t next = {0};
    mfoc_result_t rc = mfoc_core_Init(&next.core, &cfg->core);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_feedback_Init(&next.feedback, &cfg->feedback, p->pole_pairs);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (cfg->speed_divider != 0U) {
        if (!mfoc_scalar_is_finite(cfg->limits.speed_max) ||
            cfg->limits.speed_max <= MFOC_ZERO ||
            !_mfoc_motor_PiValid(&cfg->speed) ||
            cfg->speed.out_min < -cfg->limits.i_max ||
            cfg->speed.out_max > cfg->limits.i_max) {
            return MFOC_RESULT_OUT_OF_RANGE;
        }
        rc = mfoc_pi_Init(&next.speed, &cfg->speed);
        if (rc != MFOC_RESULT_OK) { return rc; }
    }
    next.speed_divider = cfg->speed_divider;
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
    if (motor == NULL) { return MFOC_RESULT_NULL; }
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
    motor->state = MFOC_MOTOR_RUNNING;
    return MFOC_RESULT_OK;
}

/** @brief Immediately latch fault and request shutdown, including repeats.
 * @param motor Instance. @param faults Nonzero cause bits.
 * @return Physical shutdown status. */
mfoc_result_t mfoc_motor_Trip(mfoc_motor_t *motor, uint32_t faults)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (faults == 0U) { return MFOC_RESULT_INVALID_ARGUMENT; }
    if (motor->state == MFOC_MOTOR_UNINITIALIZED) {
        return MFOC_RESULT_INVALID_STATE;
    }
    motor->previous_core_output_valid = false;
    motor->faults |= faults;
    motor->state = MFOC_MOTOR_FAULT;
    motor->command.i_ref = (mfoc_dq_t){0};
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
    motor->command.i_ref = (mfoc_dq_t){0};
    mfoc_result_t reset = mfoc_core_Reset(&motor->core);
    if (rc != MFOC_RESULT_OK) { motor->faults |= MFOC_MOTOR_FAULT_PWM; }
    if (reset != MFOC_RESULT_OK) {
        motor->faults |= MFOC_MOTOR_FAULT_MATH;
        if (rc == MFOC_RESULT_OK) { rc = reset; }
    }
    motor->state = motor->faults != 0U ?
        MFOC_MOTOR_FAULT : MFOC_MOTOR_IDLE;
    motor->align_active = false;
    motor->align_deadline = 0;
    motor->previous_core_output_valid = false;
    if (motor->speed_divider != 0U) {
        mfoc_result_t speed_reset = mfoc_pi_Reset(&motor->speed);
        if (rc == MFOC_RESULT_OK) { rc = speed_reset; }
    }
    motor->speed_ref = MFOC_ZERO;
    motor->speed_count = 0U;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    return rc;
}

/** @brief Recover software state after physical shutdown.
 * @param motor Instance. @return Recovery status. */
mfoc_result_t mfoc_motor_ClearFault(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    mfoc_result_t rc = MFOC_RESULT_INVALID_STATE;
    if (motor->state == MFOC_MOTOR_FAULT) {
        rc = mfoc_port_pwm_stop();
        if (rc != MFOC_RESULT_OK) {
            motor->faults |= MFOC_MOTOR_FAULT_PWM;
        }
        if (rc == MFOC_RESULT_OK) { rc = mfoc_core_Reset(&motor->core); }
        if (rc == MFOC_RESULT_OK && motor->speed_divider != 0U) {
            rc = mfoc_pi_Reset(&motor->speed);
        }
        if (rc == MFOC_RESULT_OK) {
            motor->command.i_ref = (mfoc_dq_t){0};
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

    if (motor->align_active ||
        (motor->state != MFOC_MOTOR_IDLE &&
         motor->state != MFOC_MOTOR_RUNNING) ||
        (motor->state == MFOC_MOTOR_RUNNING &&
         motor->mode != MFOC_MOTOR_MODE_CURRENT)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_dq_t next = *ref;
    if (!_mfoc_motor_InCircle(next.qD, next.qQ, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    motor->command.i_ref = next;
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
    if (motor->state != MFOC_MOTOR_IDLE &&
        motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_scalar_t bus = MFOC_ZERO;
    mfoc_result_t rc = mfoc_port_bus(&bus);
    if (rc != MFOC_RESULT_OK) { return rc; }
    return mfoc_core_SetBus(&motor->core, bus);
}

/** @brief Poll feedback and service the existing fixed-field hold.
 * @param motor Valid initialized instance. @return Service/stop status. */
mfoc_result_t mfoc_motor_Run(mfoc_motor_t *motor)
{
    mfoc_result_t feedback_rc = mfoc_feedback_Run(&motor->feedback);

    // 执行对齐
    if (motor->align_active) {
        mfoc_motor_state_t state = motor->state;
        if (state == MFOC_MOTOR_RUNNING &&
            !perfc_is_time_out_ms(MFOC_MOTOR_ALIGN_TIME_MS,
                                  &motor->align_deadline, false)) {
            return MFOC_RESULT_OK;
        }
        /* Read before releasing the fixed field, never after Stop. */
        mfoc_result_t calibration = state == MFOC_MOTOR_RUNNING ?
            mfoc_feedback_CalibrateZero(&motor->feedback, motor->align_angle) :
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
    return feedback_rc;
}

/** @brief Start fixed-angle d-axis current alignment from IDLE.
 * @param motor Instance. @param id_pu Current. @param angle_pu Turns.
 * @return Current-command or startup status. */
mfoc_result_t mfoc_motor_AlignStart(mfoc_motor_t *motor,
    mfoc_scalar_t id_pu, mfoc_scalar_t angle_pu)
{
    if (motor->state != MFOC_MOTOR_IDLE || motor->align_active) {
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
    motor->command.i_ref = ref;
    motor->previous_core_output_valid = false;
    motor->align_angle = mfoc_angle_from_scalar(angle_pu);
    motor->align_active = true;
    mfoc_result_t rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        motor->align_active = false;
        return rc;
    }
    motor->align_deadline = 0;
    (void)perfc_is_time_out_ms(MFOC_MOTOR_ALIGN_TIME_MS,
                              &motor->align_deadline, false);
    return MFOC_RESULT_OK;
}

/** @brief Select speed mode while idle or update its running reference.
 * @param motor Instance. @param ref Electrical speed PU.
 * @return Command status. */
mfoc_result_t mfoc_motor_SetSpeed(mfoc_motor_t *motor,
                                  mfoc_scalar_t ref)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->align_active) { return MFOC_RESULT_INVALID_STATE; }
    if (motor->speed_divider == 0U) { return MFOC_RESULT_INVALID_STATE; }
    if (motor->state != MFOC_MOTOR_IDLE &&
        motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (motor->state == MFOC_MOTOR_RUNNING &&
        motor->mode != MFOC_MOTOR_MODE_SPEED) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (ref < -motor->limits.speed_max || ref > motor->limits.speed_max) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    if (motor->state == MFOC_MOTOR_IDLE) {
        mfoc_result_t rc = mfoc_pi_Reset(&motor->speed);
        if (rc != MFOC_RESULT_OK) { return rc; }
        motor->speed_count = 0U;
        motor->mode = MFOC_MOTOR_MODE_SPEED;
    }
    motor->speed_ref = ref;
    return MFOC_RESULT_OK;
}

/** @brief Sample current, execute control, and submit the resulting PWM.
 * @param motor Initialized instance. @return Sampling/control/PWM status.
 * @note Single control-ISR entry; one RUNNING gate precedes all processing. */
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
    mfoc_angle_t angle = motor->align_angle;
    if (!motor->align_active) {
        mfoc_feedback_input_t feedback_input = {
            .i_ab_k = &i_ab,
            .previous_core_output = motor->previous_core_output_valid ?
                &motor->previous_core_output : NULL};
        mfoc_feedback_output_t feedback = {0};
        rc = mfoc_feedback_Step(&motor->feedback, &feedback_input, &feedback);
        if (rc != MFOC_RESULT_OK) {
            return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_FEEDBACK, rc);
        }
        angle = feedback.angle;
        // 速度环，按照speed_divider分频
        if (motor->mode == MFOC_MOTOR_MODE_SPEED &&
            motor->speed_count == 0U) {
            mfoc_dq_t ref = {0};
            mfoc_pi_Step(&motor->speed, motor->speed_ref,
                          feedback.speed, &ref.qQ);
            motor->command.i_ref = ref;
            motor->speed_count = motor->speed_divider;
        }
    }
    // 关闭超出电流圆就触发故障功能
    // if (!_mfoc_motor_InCircle(i_ab.qAlpha, i_ab.qBeta,
    //                           motor->limits.i_max)) {
    //     return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_OVERCURRENT,
    //                             MFOC_RESULT_SAFETY);
    // }
    mfoc_core_input_t core_input = {
        .i = i_ab, .ref = motor->command.i_ref, .angle = angle};
    mfoc_core_output_t output = {0};
    mfoc_core_Step(&motor->core, &core_input, &output);
    rc = mfoc_port_pwm(&output.duty, false);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_PWM, rc);
    }
    motor->previous_core_output = output;
    motor->previous_core_output_valid = true;
    if (!motor->align_active && motor->speed_count != 0U) {
        motor->speed_count--;
    }
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
