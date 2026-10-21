/**
 * @file mfoc_motor.c
 * @brief Motor lifecycle and current-loop control.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include "mfoc_motor_port.h"
#include "mfoc_port.h"
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
    next.params = cfg->params;
    next.limits = cfg->limits;
    next.state = MFOC_MOTOR_IDLE;
    *motor = next;
    return MFOC_RESULT_OK;
}

/** @brief Perform a serialized safe start of the statically bound bridge.
 * @param motor Instance. @return Readiness, control or hardware status. */
mfoc_result_t mfoc_motor_Start(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    uint32_t mask = mfoc_motor_port_EnterCritical();
    mfoc_result_t rc = MFOC_RESULT_INVALID_STATE;
    if (motor->state == MFOC_MOTOR_IDLE && motor->faults == 0U) {
        rc = mfoc_motor_port_CheckReady();
        if (rc == MFOC_RESULT_OK) {
            rc = mfoc_core_Reset(&motor->core);
            if (rc == MFOC_RESULT_OK) {
                const mfoc_uvw_t neutral = {
                    MFOC_HALF, MFOC_HALF, MFOC_HALF};
                rc = mfoc_port_pwm(&neutral);
            }
            if (rc == MFOC_RESULT_OK) {
                rc = mfoc_motor_port_EnablePwm();
            }
            if (rc == MFOC_RESULT_OK) {
                motor->state = MFOC_MOTOR_RUNNING;
            } else {
                rc = _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_PWM, rc);
            }
        }
    }
    mfoc_motor_port_ExitCritical(mask);
    return rc;
}

/** @brief Immediately latch fault and request shutdown, including repeats.
 * @param motor Instance. @param faults Nonzero cause bits.
 * @return Physical shutdown status. */
mfoc_result_t mfoc_motor_Trip(mfoc_motor_t *motor, uint32_t faults)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (faults == 0U) { return MFOC_RESULT_INVALID_ARGUMENT; }
    uint32_t mask = mfoc_motor_port_EnterCritical();
    if (motor->state == MFOC_MOTOR_UNINITIALIZED) {
        mfoc_motor_port_ExitCritical(mask);
        return MFOC_RESULT_INVALID_STATE;
    }
    motor->faults |= faults;
    motor->state = MFOC_MOTOR_FAULT;
    motor->command.i_ref = (mfoc_dq_t){0};
    mfoc_result_t rc = mfoc_motor_port_DisablePwm();
    if (rc != MFOC_RESULT_OK) {
        motor->faults |= MFOC_MOTOR_FAULT_PWM;
    }
    mfoc_motor_port_ExitCritical(mask);
    return rc;
}

/** @brief Stop PWM and reset control while preserving fault history.
 * @param motor Instance. @return Shutdown or reset status. */
mfoc_result_t mfoc_motor_Stop(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    uint32_t mask = mfoc_motor_port_EnterCritical();
    if (motor->state == MFOC_MOTOR_UNINITIALIZED) {
        mfoc_motor_port_ExitCritical(mask);
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_result_t rc = mfoc_motor_port_DisablePwm();
    motor->command.i_ref = (mfoc_dq_t){0};
    mfoc_result_t reset = mfoc_core_Reset(&motor->core);
    if (rc != MFOC_RESULT_OK) { motor->faults |= MFOC_MOTOR_FAULT_PWM; }
    if (reset != MFOC_RESULT_OK) {
        motor->faults |= MFOC_MOTOR_FAULT_MATH;
        if (rc == MFOC_RESULT_OK) { rc = reset; }
    }
    motor->state = motor->faults != 0U ?
        MFOC_MOTOR_FAULT : MFOC_MOTOR_IDLE;
    mfoc_motor_port_ExitCritical(mask);
    return rc;
}

/** @brief Recover only after physical shutdown and prerequisite checks.
 * @param motor Instance. @return Recovery status. */
mfoc_result_t mfoc_motor_ClearFault(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    uint32_t mask = mfoc_motor_port_EnterCritical();
    mfoc_result_t rc = MFOC_RESULT_INVALID_STATE;
    if (motor->state == MFOC_MOTOR_FAULT) {
        rc = mfoc_motor_port_DisablePwm();
        if (rc != MFOC_RESULT_OK) {
            motor->faults |= MFOC_MOTOR_FAULT_PWM;
        } else {
            rc = mfoc_motor_port_CheckReady();
        }
        if (rc == MFOC_RESULT_OK) { rc = mfoc_core_Reset(&motor->core); }
        if (rc == MFOC_RESULT_OK) {
            motor->command.i_ref = (mfoc_dq_t){0};
            motor->faults = 0U;
            motor->state = MFOC_MOTOR_IDLE;
        }
    }
    mfoc_motor_port_ExitCritical(mask);
    return rc;
}

/** @brief Validate the current circle before publishing a complete command.
 * @param motor Instance. @param ref Requested dq current.
 * @return OK or rejection; the previous command survives rejection. */
mfoc_result_t mfoc_motor_SetCurrent(mfoc_motor_t *motor,
                                   const mfoc_dq_t *ref)
{
    if (motor == NULL || ref == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE &&
        motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_dq_t next = *ref;
    if (!mfoc_scalar_is_finite(next.qD) ||
        !mfoc_scalar_is_finite(next.qQ) ||
        !_mfoc_motor_InCircle(next.qD, next.qQ, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    uint32_t mask = mfoc_motor_port_EnterCritical();
    mfoc_result_t rc = MFOC_RESULT_INVALID_STATE;
    if (motor->state == MFOC_MOTOR_IDLE ||
        motor->state == MFOC_MOTOR_RUNNING) {
        motor->command.i_ref = next;
        rc = MFOC_RESULT_OK;
    }
    mfoc_motor_port_ExitCritical(mask);
    return rc;
}

/** @brief Forward a serialized bus update to the instance-owned Core.
 * @param motor Instance. @param bus Vdc/Vbase. @return Update status. */
mfoc_result_t mfoc_motor_SetBus(mfoc_motor_t *motor, mfoc_scalar_t bus)
{
    return mfoc_core_SetBus(&motor->core, bus);
}

/** @brief Execute Core, then commit only if no fault interrupted the step.
 * @param motor Instance. @param input Current/angle. @param output Results.
 * @return Control status; any failed step must be discarded by the caller. */
mfoc_result_t mfoc_motor_Step(mfoc_motor_t *motor,
                             const mfoc_motor_input_t *input,
                             mfoc_core_output_t *output)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (input == NULL || output == NULL) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_FEEDBACK,
                                MFOC_RESULT_NULL);
    }
    if (motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (!_mfoc_motor_InCircle(input->i_ab.qAlpha, input->i_ab.qBeta,
                              motor->limits.i_max)) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_OVERCURRENT,
                                MFOC_RESULT_SAFETY);
    }
    mfoc_core_input_t core_input = {
        .i = input->i_ab, .ref = motor->command.i_ref, .angle = input->angle};
    mfoc_result_t rc = mfoc_core_Step(&motor->core, &core_input, output);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_MATH,
                                MFOC_RESULT_SAFETY);
    }
    /* No mask around Core math. A preempting Trip prevents this commit. */
    uint32_t mask = mfoc_motor_port_EnterCritical();
    if (motor->state == MFOC_MOTOR_RUNNING && motor->faults == 0U) {
        rc = mfoc_port_pwm(&output->duty);
        if (rc != MFOC_RESULT_OK) {
            rc = _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_PWM, rc);
        }
    } else {
        rc = MFOC_RESULT_SAFETY;
    }
    mfoc_motor_port_ExitCritical(mask);
    return rc;
}

/** @brief Snapshot lifecycle fields without torn foreground reads.
 * @param motor Instance. @param status Destination. @return OK or NULL. */
mfoc_result_t mfoc_motor_GetStatus(const mfoc_motor_t *motor,
                                  mfoc_motor_status_t *status)
{
    if (motor == NULL || status == NULL) { return MFOC_RESULT_NULL; }
    uint32_t mask = mfoc_motor_port_EnterCritical();
    status->state = motor->state;
    status->faults = motor->faults;
    mfoc_motor_port_ExitCritical(mask);
    return MFOC_RESULT_OK;
}
