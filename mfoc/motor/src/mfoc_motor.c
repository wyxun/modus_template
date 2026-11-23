/**
 * @file mfoc_motor.c
 * @brief Motor lifecycle and current-loop control.
 * @author wyxun
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include "mfoc_feedback_internal.h"
#include "../../include/mfoc_trace_internal.h"
#if MFOC_IDENTIFICATION_ENABLE
#include "mfoc_identification_internal.h"
#endif
#include "mfoc_port.h"
#include "mfoc_transform.h"
#include "mfoc_motor_config.h"
#include "mfoc_modulation.h"
#include <stddef.h>
#include <string.h>

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
    if (cfg->current_base_milliamp == 0U ||
        cfg->voltage_base_millivolt == 0U ||
        cfg->electrical_speed_base_hz <= MFOC_ZERO) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    if (cfg->params.pole_pairs == 0U) { return MFOC_RESULT_INVALID_ARGUMENT; }
#if MFOC_FEEDBACK_ENABLE_HFI
    /* HFI retains its validated compile-time speed conversion. */
    if (cfg->electrical_speed_base_hz !=
        MFOC_SCALAR(MFOC_HFI_SPEED_BASE_HZ)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#endif
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
    mfoc_core_t core = {0};
    mfoc_result_t rc = mfoc_core_Init(&core, &cfg->core);
    if (rc != MFOC_RESULT_OK) { return rc; }
    mfoc_feedback_cfg_t feedback_cfg = cfg->feedback;
    feedback_cfg.smo.speed_base_hz = cfg->electrical_speed_base_hz;
    feedback_cfg.flux.speed_base_hz = cfg->electrical_speed_base_hz;
#if MFOC_FEEDBACK_ENABLE_EKF
    feedback_cfg.ekf.speed_base_hz = cfg->electrical_speed_base_hz;
#endif
    mfoc_feedback_prepared_t feedback = {0};
    rc = mfoc_feedback_Prepare(&feedback, &feedback_cfg, p);
    if (rc != MFOC_RESULT_OK) { return rc; }
    mfoc_pi_t speed = {0};
    if (cfg->speed_divider != 0U) {
        if (!mfoc_scalar_is_finite(cfg->limits.speed_max) ||
            cfg->speed.out_min < -cfg->limits.i_max ||
            cfg->speed.out_max > cfg->limits.i_max) {
            return MFOC_RESULT_OUT_OF_RANGE;
        }
        rc = mfoc_pi_Init(&speed, &cfg->speed);
        if (rc != MFOC_RESULT_OK) { return rc; }
    }
    mfoc_openloop_t openloop = {0};
    if (cfg->openloop.sample_hz != 0U) {
        if (cfg->limits.speed_max > MFOC_ONE) {
            return MFOC_RESULT_OUT_OF_RANGE;
        }
        mfoc_openloop_cfg_t openloop_cfg = cfg->openloop;
        openloop_cfg.speed_base_hz = cfg->electrical_speed_base_hz;
        rc = mfoc_openloop_Init(&openloop, &openloop_cfg);
        if (rc != MFOC_RESULT_OK) { return rc; }
    }
    const mfoc_port_cfg_t port_cfg = {
        .current_base_milliamp = cfg->current_base_milliamp,
        .voltage_base_millivolt = cfg->voltage_base_millivolt,
        .electrical_speed_base_hz = cfg->electrical_speed_base_hz,
        .pole_pairs = p->pole_pairs,
    };
    rc = mfoc_port_init(&port_cfg);
    if (rc != MFOC_RESULT_OK) { return rc; }
    uint32_t tick_hz = mfoc_port_tick_frequency();
    if (tick_hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    /* After Port accepts, install only the prepared state. This avoids a
     * stack copy of experiment workspace, telemetry and HFI sample history. */
    memset(motor, 0, sizeof(*motor));
    motor->core = core;
    mfoc_feedback_InitPrepared(&motor->feedback, &feedback, &feedback_cfg);
    motor->speed = speed;
    motor->speed_divider = cfg->speed_divider;
    motor->openloop = openloop;
    if (cfg->openloop.sample_hz != 0U) {
        mfoc_sensorless_start_Init(&motor->sensorless_start,
                                   cfg->openloop.sample_hz);
    }
#if MFOC_IDENTIFICATION_ENABLE
    motor->identification.timing_mismatch =
        cfg->electrical_speed_base_hz !=
            MFOC_SCALAR(MFOC_IDENTIFICATION_SPEED_BASE_HZ) ||
        (cfg->openloop.sample_hz != 0U &&
         cfg->openloop.sample_hz != MFOC_IDENTIFICATION_SAMPLE_HZ);
#endif
    motor->params = cfg->params;
    motor->limits = cfg->limits;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    motor->irq_result = MFOC_RESULT_INVALID_STATE;
    motor->hold_ticks = ((mfoc_tick_t)tick_hz * MFOC_MOTOR_ALIGN_TIME_MS +
        999U) / 1000U;
    motor->state = MFOC_MOTOR_IDLE;
    return MFOC_RESULT_OK;
}

/** @brief Invalidate telemetry and initialize the next neutral interval. */
static inline void _mfoc_motor_ResetOutputs(mfoc_motor_t *motor)
{
    motor->previous_core_output_valid = false;
    motor->previous_voltage_ab_valid = false;
#if MFOC_PWM_PRELOAD_DELAY
    motor->previous_voltage_ab = (mfoc_ab_t){0};
    motor->preloaded_voltage_ab = (mfoc_ab_t){0};
#if MFOC_IDENTIFICATION_ENABLE
    motor->previous_excitation = MFOC_IDENTIFICATION_EXCITATION_NONE;
    motor->preloaded_excitation = MFOC_IDENTIFICATION_EXCITATION_NONE;
#endif
#endif
}

/** @brief Start neutral PWM after the command has validated the idle Motor. */
static mfoc_result_t _mfoc_motor_StartPwm(mfoc_motor_t *motor)
{
    mfoc_result_t rc = mfoc_core_Reset(&motor->core);
    if (rc == MFOC_RESULT_OK) {
        const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
        rc = mfoc_port_pwm_start(&neutral);
    }
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_PWM, rc);
    }
    /* The first sample can straddle gate enable. Its predecessor is invalid. */
    _mfoc_motor_ResetOutputs(motor);
    motor->state = MFOC_MOTOR_RUNNING;
    return MFOC_RESULT_OK;
}

/** @brief Start with the HFI policy for the owner of this run. */
mfoc_result_t mfoc_motor_Start(mfoc_motor_t *motor)
{
    if (motor->state != MFOC_MOTOR_IDLE || motor->faults != 0U) {
        return MFOC_RESULT_INVALID_STATE;
    }
#if MFOC_IDENTIFICATION_ENABLE
    mfoc_identification_t *id = &motor->identification;
    if (mfoc_identification_OwnsControl(id)) {
        if (id->state != MFOC_IDENTIFICATION_RUNNING ||
            motor->limits.i_max <= MFOC_ZERO) {
            return MFOC_RESULT_INVALID_STATE;
        }
        if (motor->limits.i_max < id->current_limit) {
            id->current_limit = motor->limits.i_max;
        }
        if (motor->limits.speed_max > MFOC_ZERO &&
            motor->limits.speed_max < id->speed_limit) {
            id->speed_limit = motor->limits.speed_max;
        }
        if ((id->operation == MFOC_IDENTIFICATION_FLUX ||
             id->operation == MFOC_IDENTIFICATION_INERTIA) &&
            (id->speed_limit < MFOC_IDENTIFICATION_SPEED_MIN_PU ||
             id->current_limit <
                 MFOC_SCALAR(MFOC_IDENTIFICATION_ROTATION_CURRENT_MIN))) {
            return MFOC_RESULT_OUT_OF_RANGE;
        }
    }
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    /* Align and forced rotation both start in their fixed-angle hold. */
    bool closed_loop = !motor->hold_active;
#if MFOC_IDENTIFICATION_ENABLE
    closed_loop = closed_loop &&
        !mfoc_identification_OwnsControl(&motor->identification);
#endif
    mfoc_feedback_PrepareHfi(&motor->feedback, closed_loop);
#endif
    return _mfoc_motor_StartPwm(motor);
}

/** @brief Immediately latch fault and request shutdown, including repeats.
 * @param motor Instance. @param faults Nonzero cause bits.
 * @return Physical shutdown status. */
mfoc_result_t mfoc_motor_Trip(mfoc_motor_t *motor, uint32_t faults)
{
#if !MFOC_PWM_PRELOAD_DELAY
    _mfoc_motor_ResetOutputs(motor);
#endif
    motor->faults |= faults;
    motor->state = MFOC_MOTOR_FAULT;
#if MFOC_PWM_PRELOAD_DELAY
    /* Exclude the sampling ISR before clearing the two interval records. */
    _mfoc_motor_ResetOutputs(motor);
#endif
    motor->current_cmd = (mfoc_dq_t){0};
    mfoc_sensorless_start_Stop(&motor->sensorless_start);
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
    motor->current_cmd = (mfoc_dq_t){0};
    mfoc_result_t reset = mfoc_core_Reset(&motor->core);
    if (rc != MFOC_RESULT_OK) { motor->faults |= MFOC_MOTOR_FAULT_PWM; }
    if (reset != MFOC_RESULT_OK) {
        motor->faults |= MFOC_MOTOR_FAULT_MATH;
        if (rc == MFOC_RESULT_OK) { rc = reset; }
    }
    motor->state = motor->faults != 0U ?
        MFOC_MOTOR_FAULT : MFOC_MOTOR_IDLE;
    motor->hold_active = false;
    motor->hold_started = 0U;
    mfoc_sensorless_start_Stop(&motor->sensorless_start);
    motor->openloop_active = false;
    mfoc_openloop_Reset(&motor->openloop, (mfoc_angle_t){0U});
    _mfoc_motor_ResetOutputs(motor);
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
            motor->current_cmd = (mfoc_dq_t){0};
            mfoc_openloop_Reset(&motor->openloop, (mfoc_angle_t){0U});
            motor->hold_active = false;
            motor->hold_started = 0U;
            mfoc_sensorless_start_Stop(&motor->sensorless_start);
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

/** @brief Check ownership before an external reference command writes state.
 * @param motor Serialized instance. @param state Current lifecycle state.
 * @param mode Requested control mode. @return Whether the command may run. */
static bool _mfoc_motor_CanSetReference(const mfoc_motor_t *motor,
    mfoc_motor_state_t state, mfoc_motor_mode_t mode)
{
#if MFOC_IDENTIFICATION_ENABLE
    if (mfoc_identification_OwnsControl(&motor->identification)) {
        return false;
    }
#endif
    return (state == MFOC_MOTOR_IDLE ||
            (state == MFOC_MOTOR_RUNNING && motor->mode == mode)) &&
           !motor->hold_active && !motor->openloop_active &&
           motor->sensorless_start.state == MFOC_SENSORLESS_START_NONE;
}

/** @brief Validate the current circle before publishing a complete command.
 * @param motor Instance. @param ref Requested dq current.
 * @return OK or rejection; the previous command survives rejection. */
mfoc_result_t mfoc_motor_SetCurrent(mfoc_motor_t *motor,
                                   const mfoc_dq_t *ref)
{
    if (motor == NULL || ref == NULL) { return MFOC_RESULT_NULL; }

    mfoc_motor_state_t state = motor->state;
    if (!_mfoc_motor_CanSetReference(motor, state, MFOC_MOTOR_MODE_CURRENT)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_dq_t next = *ref;
    if (!_mfoc_motor_InCircle(next.qD, next.qQ, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    motor->current_cmd = next;
    if (state == MFOC_MOTOR_IDLE) {
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

/** @brief Refresh Core bus coefficients from the published Port sample.
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

/** @brief Service physical position, then Identification or timed startup.
 * @param motor Valid initialized instance. @return Service/stop status. */
mfoc_result_t mfoc_motor_Run(mfoc_motor_t *motor)
{
    mfoc_result_t position_rc = MFOC_RESULT_OK;
#if MFOC_FEEDBACK_ENABLE_POSITION
    position_rc = mfoc_port_position_run();
#if MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_POSITION
    /* An observational position source does not gate sensorless control. */
    position_rc = MFOC_RESULT_OK;
#endif
#endif
#if MFOC_IDENTIFICATION_ENABLE
    if (mfoc_identification_OwnsControl(&motor->identification)) {
        if (motor->state == MFOC_MOTOR_FAULT) {
            mfoc_identification_Stop(&motor->identification);
            return MFOC_RESULT_SAFETY;
        }
        if (motor->state == MFOC_MOTOR_RUNNING) {
            mfoc_fsm_rt_t progress =
                mfoc_identification_Run(&motor->identification);
            if (motor->state != MFOC_MOTOR_RUNNING) {
                mfoc_identification_Stop(&motor->identification);
                return MFOC_RESULT_SAFETY;
            }
            if (progress == MFOC_FSM_FAILED) {
                return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_EXTERNAL,
                                        MFOC_RESULT_SAFETY);
            }
            if (progress == MFOC_FSM_COMPLETE) {
                return mfoc_motor_Stop(motor);
            }
            return MFOC_RESULT_OK;
        }
        return MFOC_RESULT_OK;
    }
#endif
    /* Run ends the fixed-angle hold; ISR startup budget remains paused. */
    if (motor->hold_active) {
        mfoc_motor_state_t state = motor->state;
        if (state == MFOC_MOTOR_RUNNING &&
            mfoc_port_tick() - motor->hold_started < motor->hold_ticks) {
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
        /* Capture physical position zero before releasing the held field. */
#if MFOC_FEEDBACK_ENABLE_POSITION
        mfoc_result_t calibration = state == MFOC_MOTOR_RUNNING ?
            mfoc_port_position_calibrate_zero(motor->openloop.angle) :
            MFOC_RESULT_OK;
#else
        mfoc_result_t calibration = MFOC_RESULT_OK;
#endif
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
        MFOC_RESULT_OK : position_rc;
}

/** @brief An existing hold or identification owns the next startup. */
static inline bool _mfoc_motor_OutputReserved(const mfoc_motor_t *motor)
{
#if MFOC_IDENTIFICATION_ENABLE
    return motor->hold_active ||
        mfoc_identification_OwnsControl(&motor->identification);
#else
    return motor->hold_active;
#endif
}

#if MFOC_IDENTIFICATION_ENABLE
/** @brief Arm and start an experiment while preserving a previous result.
 * @param motor Stopped instance. @param operation Requested experiment.
 * @return Validation or PWM startup status. */
mfoc_result_t mfoc_motor_StartIdentification(mfoc_motor_t *motor,
    mfoc_identification_operation_t operation)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE || motor->faults != 0U ||
        _mfoc_motor_OutputReserved(motor)) {
        return MFOC_RESULT_INVALID_STATE;
    }

    mfoc_identification_t *id = &motor->identification;
    const mfoc_identification_state_t previous_state = id->state;
    const mfoc_identification_operation_t previous_op = id->operation;
    mfoc_motor_params_t previous_result = {0};
    if (previous_state == MFOC_IDENTIFICATION_DONE) {
        previous_result = id->result;
    }
    mfoc_result_t rc = mfoc_identification_Start(id, &motor->params, operation,
        mfoc_core_GetBus(&motor->core));
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        mfoc_identification_Stop(id);
        if (previous_state == MFOC_IDENTIFICATION_DONE) {
            id->result = previous_result;
        }
        id->operation = previous_op;
        id->state = previous_state;
    }
    return rc;
}

/** @brief Read a terminal experiment result even after a bridge fault.
 * @param motor Initialized instance. @param params Result destination.
 * @return Status; no result is written before a successful measurement. */
mfoc_result_t mfoc_motor_GetIdentificationResult(const mfoc_motor_t *motor,
    mfoc_motor_params_t *params)
{
    if (motor == NULL || params == NULL) { return MFOC_RESULT_NULL; }
    return mfoc_identification_GetResult(&motor->identification, params);
}
#endif

#if MFOC_MOTOR_MODEL_UPDATE_ENABLE
/** @brief Commit Motor parameters after Feedback accepts every coefficient. */
mfoc_result_t mfoc_motor_ApplyModel(mfoc_motor_t *motor,
                                   const mfoc_motor_params_t *params)
{
    if (motor == NULL || params == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE || _mfoc_motor_OutputReserved(motor)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (params->pole_pairs != motor->params.pole_pairs) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_result_t rc = mfoc_feedback_SetMotorParamModel(&motor->feedback, params);
    if (rc == MFOC_RESULT_OK) { motor->params = *params; }
    return rc;
}
#endif

/** @brief Start HFI injection with a zero current command.
 * @param motor Stopped instance. @return Validation or PWM startup status. */
mfoc_result_t mfoc_motor_HfiStart(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
#if !MFOC_FEEDBACK_ENABLE_HFI || (MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_HFI && \
    MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_POSITION)
    return MFOC_RESULT_INVALID_STATE;
#else
    if (motor->state != MFOC_MOTOR_IDLE || motor->faults != 0U ||
        _mfoc_motor_OutputReserved(motor)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_result_t rc = mfoc_feedback_HfiStart(&motor->feedback);
    if (rc != MFOC_RESULT_OK) { return rc; }
    motor->current_cmd = (mfoc_dq_t){0};
    mfoc_openloop_Reset(&motor->openloop, (mfoc_angle_t){0U});
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    return _mfoc_motor_StartPwm(motor);
#endif
}

/** @brief Start a fixed d-current hold and a signed forced-angle ramp.
 * @param motor Instance. @param speed Electrical PU. @param id_pu Current.
 * @return Validation or startup status. */
mfoc_result_t mfoc_motor_OpenLoopStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t id_pu)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE || _mfoc_motor_OutputReserved(motor)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (speed == MFOC_ZERO || speed < -motor->limits.speed_max ||
        speed > motor->limits.speed_max || id_pu <= MFOC_ZERO ||
        !_mfoc_motor_InCircle(id_pu, MFOC_ZERO, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    mfoc_openloop_t loop = motor->openloop;
    mfoc_openloop_Reset(&loop, (mfoc_angle_t){0U});
    mfoc_result_t rc = mfoc_openloop_SetSpeed(&loop, speed);
    if (rc != MFOC_RESULT_OK) { return rc; }
    motor->fixed_id = id_pu;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    motor->openloop_active = true;
    motor->openloop = loop;
    motor->hold_active = true;
    rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        motor->hold_active = false;
        motor->openloop_active = false;
        return rc;
    }
    motor->hold_started = mfoc_port_tick();
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
     MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_FLUX &&                             \
     MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_EKF)
    (void)motor;
    (void)speed;
    (void)current;
    return MFOC_RESULT_INVALID_STATE;
#else
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_IDLE || _mfoc_motor_OutputReserved(motor) ||
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
    motor->speed_iq = iq;
    motor->speed_ref = speed;
    motor->speed_count = 0U;
    motor->mode = MFOC_MOTOR_MODE_CURRENT;
    mfoc_sensorless_start_Start(&motor->sensorless_start, speed, current);
    motor->hold_active = true;
    motor->hold_started = mfoc_port_tick();
    rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        motor->hold_active = false;
        mfoc_sensorless_start_Stop(&motor->sensorless_start);
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
    if (motor->state != MFOC_MOTOR_IDLE || _mfoc_motor_OutputReserved(motor)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (angle_pu < MFOC_ZERO || angle_pu >= MFOC_ONE) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    if (id_pu <= MFOC_ZERO ||
        !_mfoc_motor_InCircle(id_pu, MFOC_ZERO, motor->limits.i_max)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    motor->fixed_id = id_pu;
    mfoc_openloop_Reset(&motor->openloop,
                         mfoc_angle_from_scalar(angle_pu));
    motor->hold_active = true;
    mfoc_result_t rc = mfoc_motor_Start(motor);
    if (rc != MFOC_RESULT_OK) {
        motor->hold_active = false;
        return rc;
    }
    motor->hold_started = mfoc_port_tick();
    return MFOC_RESULT_OK;
}

/** @brief Select speed mode while idle or update its running reference.
 * @param motor Instance. @param ref Electrical speed PU.
 * @return Command status. */
mfoc_result_t mfoc_motor_SetSpeed(mfoc_motor_t *motor,
                                  mfoc_scalar_t ref)
{
    mfoc_motor_state_t state = motor->state;
    if (motor->speed_divider == 0U ||
        !_mfoc_motor_CanSetReference(motor, state, MFOC_MOTOR_MODE_SPEED)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (ref < -motor->limits.speed_max || ref > motor->limits.speed_max) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO || \
    MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_FLUX ||                              \
    MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_EKF
    if (state == MFOC_MOTOR_RUNNING &&
        (mfoc_abs(ref) < MFOC_SCALAR(MFOC_START_MIN_SPEED_PU) ||
         (ref > MFOC_ZERO) != (motor->speed_ref > MFOC_ZERO))) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#endif
    if (state == MFOC_MOTOR_IDLE) {
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
 * @param speed_ticks Zero to freeze, otherwise ticks before PWM commit.
 * @return OK or the error handled by Motor's existing fault path. */
static mfoc_result_t _mfoc_motor_SelectReference(mfoc_motor_t *motor,
    const mfoc_feedback_output_t *feedback, mfoc_result_t feedback_rc,
    mfoc_core_input_t *input, uint16_t *speed_ticks)
{
    uint16_t count = motor->speed_count;
    if (motor->openloop_active) {
        mfoc_openloop_Step(&motor->openloop);
        input->ref = (mfoc_dq_t){motor->fixed_id, MFOC_ZERO};
        input->angle = motor->openloop.angle;
        return MFOC_RESULT_OK;
    }
    if (motor->sensorless_start.state != MFOC_SENSORLESS_START_NONE) {
        input->ref = (mfoc_dq_t){MFOC_ZERO, motor->speed_iq};
        mfoc_sensorless_input_t ctx = {
            .feedback = feedback, .feedback_rc = feedback_rc,
            .speed_ref = motor->speed_ref, .hold_active = motor->hold_active};
        mfoc_result_t rc = mfoc_sensorless_start_Step(
            &motor->sensorless_start, &motor->openloop, &ctx, input);
        if (rc != MFOC_RESULT_OK) { return rc; }
        motor->speed_iq = input->ref.qQ;
        if (!motor->sensorless_start.captured ||
            motor->sensorless_start.offset != 0) { return MFOC_RESULT_OK; }
        /* Regulate captured speed before requiring it to remain stable. */
        if (motor->mode != MFOC_MOTOR_MODE_SPEED) {
            rc = mfoc_pi_Track(&motor->speed, input->ref.qQ,
                               motor->speed_ref, feedback->speed);
            if (rc != MFOC_RESULT_OK) { return rc; }
            count = motor->speed_divider;
#if MFOC_SMO_STAGGER_SPEED_PI && defined(MFOC_NUMERIC_FIXED) && \
    MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
            const mfoc_smo_t *smo = &motor->feedback.smo;
            if (count > 1U && count == smo->c.speed_span) {
                /* Feedback has advanced. Seed the first PI step one frame
                 * after a window update, accounting for PWM commit's tick. */
                const uint8_t phase = smo->speed_count;
                count = phase == 0U ? 1U : (uint16_t)(count - phase + 1);
            }
#endif
            motor->mode = MFOC_MOTOR_MODE_SPEED;
        }
        if (ctx.complete) {
            mfoc_sensorless_start_Stop(&motor->sensorless_start);
        }
    }
    if (feedback_rc != MFOC_RESULT_OK) { return feedback_rc; }
#if MFOC_FEEDBACK_ENABLE_HFI
    if (feedback->acquiring) {
        input->ref = (mfoc_dq_t){0};
        return MFOC_RESULT_OK;
    }
#endif
    if (motor->mode == MFOC_MOTOR_MODE_SPEED) {
        if (count == 0U) {
            MFOC_TRACE_BEGIN(SPEED);
            mfoc_pi_Step(&motor->speed, motor->speed_ref, feedback->speed,
                         &motor->speed_iq);
            MFOC_TRACE_END(SPEED);
            MFOC_TRACE_EVENT(SPEED_PI);
            count = motor->speed_divider;
        }
        input->ref = (mfoc_dq_t){MFOC_ZERO, motor->speed_iq};
        *speed_ticks = count;
    } else {
        input->ref = motor->current_cmd;
    }
    return MFOC_RESULT_OK;
}

/** @brief Sample current, execute control, and submit the resulting PWM.
 * @param motor Initialized instance. @return Sampling/control/PWM status.
 * @note Feedback and identification consume i[k] and the voltage over
 * [k-1,k). The command source is u[k-2] on a top-update preload target.
 * One RUNNING gate precedes all processing; history updates after PWM. */
mfoc_result_t mfoc_motor_Step(mfoc_motor_t *motor)
{
    if (motor == NULL) { return MFOC_RESULT_NULL; }
    if (motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_uvw_t current;
    MFOC_TRACE_BEGIN(CURRENT);
    mfoc_result_t rc = mfoc_port_current(&current);
    MFOC_TRACE_END(CURRENT);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_SAMPLE, rc);
    }
    mfoc_core_input_t core_input;
    mfoc_clarke(current.qU, current.qV, current.qW, &core_input.i);

    const mfoc_ab_t *u_ab_previous = motor->previous_voltage_ab_valid ?
        &motor->previous_voltage_ab : NULL;
#if MFOC_IDENTIFICATION_ENABLE
    mfoc_identification_frame_t frame =
        mfoc_identification_BeginFrame(&motor->identification);
#if MFOC_PWM_PRELOAD_DELAY
    uint8_t excitation = MFOC_IDENTIFICATION_EXCITATION_NONE;
#endif
#endif
    /* Core fills its output; direct voltage publishes only duty and bus. */
    mfoc_core_output_t output;
    uint16_t speed_ticks = 0U;
#if MFOC_IDENTIFICATION_ENABLE
    if (frame.control != MFOC_IDENTIFICATION_CONTROL_INACTIVE) {
        mfoc_identification_output_t request;
        const mfoc_identification_input_t sample = {
            .i_ab_k = &core_input.i,
            .u_ab_previous = u_ab_previous,
#if MFOC_PWM_PRELOAD_DELAY
            .active_excitation = motor->preloaded_excitation,
            .applied_excitation = motor->previous_excitation,
#endif
        };
        MFOC_TRACE_BEGIN(IDENTIFICATION);
        frame.control = mfoc_identification_StepFrame(
            &motor->identification, frame, &sample, &request);
        MFOC_TRACE_END(IDENTIFICATION);
#if MFOC_PWM_PRELOAD_DELAY
        if (motor->identification.state == MFOC_IDENTIFICATION_ERROR) {
            /* Neutral preload cannot cancel the pulse already in flight. */
            return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_EXTERNAL,
                                    MFOC_RESULT_SAFETY);
        }
        excitation = request.excitation;
#endif

        switch (frame.control) {
        case MFOC_IDENTIFICATION_CONTROL_VOLTAGE:
            mfoc_svpwm(&request.modulation_ab, &output.duty);
            output.bus = mfoc_core_GetBus(&motor->core);
            break;
        default: {
            /* VIRTUAL: the algorithm owns the current target and angle. */
            core_input.u_inject_ab = (mfoc_ab_t){0};
            core_input.ref = request.i_ref;
            core_input.angle = request.angle;
            MFOC_TRACE_BEGIN(CORE);
            mfoc_core_Step(&motor->core, &core_input, &output);
            MFOC_TRACE_END(CORE);
            break;
        }
        }
    } else
#endif
    {
        if (motor->hold_active && !motor->openloop_active &&
            motor->sensorless_start.state == MFOC_SENSORLESS_START_NONE) {
            core_input.ref = (mfoc_dq_t){motor->fixed_id, MFOC_ZERO};
            core_input.angle = motor->openloop.angle;
            core_input.u_inject_ab = (mfoc_ab_t){0};
        } else {
            mfoc_feedback_output_t feedback = {0};
            mfoc_feedback_input_t feedback_input = {
                .i_ab_k = &core_input.i, .u_ab_previous = u_ab_previous};
            MFOC_TRACE_BEGIN(FEEDBACK);
            rc = mfoc_feedback_Step(&motor->feedback, &feedback_input,
                                    &feedback);
            MFOC_TRACE_END(FEEDBACK);
            core_input.angle = feedback.angle;
            core_input.u_inject_ab = feedback.u_inject_ab;
            rc = _mfoc_motor_SelectReference(motor, &feedback, rc,
                                             &core_input, &speed_ticks);
            if (rc != MFOC_RESULT_OK) {
                return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_FEEDBACK, rc);
            }
        }
        MFOC_TRACE_BEGIN(CORE);
        mfoc_core_Step(&motor->core, &core_input, &output);
        MFOC_TRACE_END(CORE);
    }
    MFOC_TRACE_BEGIN(PWM);
    rc = mfoc_port_pwm(&output.duty);
    MFOC_TRACE_END(PWM);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_PWM, rc);
    }
#if MFOC_PWM_PRELOAD_DELAY
    /* The Port accepted this write in the current frame. The previous
     * preload is active now and will finish at the next sample. Convert
     * the new duty once, with the bus snapshot that produced it. */
    motor->previous_voltage_ab = motor->preloaded_voltage_ab;
#if MFOC_IDENTIFICATION_ENABLE
    motor->previous_excitation = motor->preloaded_excitation;
    motor->preloaded_excitation = excitation;
#endif
    mfoc_ab_t *model_voltage = &motor->preloaded_voltage_ab;
#else
    mfoc_ab_t *model_voltage = &motor->previous_voltage_ab;
#endif
    rc = mfoc_modulation_ModelVoltageAlphaBeta(&output.duty, output.bus,
                                               model_voltage);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_motor_Fail(motor, MFOC_MOTOR_FAULT_MATH, rc);
    }
    motor->previous_voltage_ab_valid = true;
#if MFOC_IDENTIFICATION_ENABLE
    motor->previous_core_output_valid =
        frame.control != MFOC_IDENTIFICATION_CONTROL_VOLTAGE;
    if (motor->previous_core_output_valid) {
        motor->previous_core_output = output;
    }
#else
    motor->previous_core_output = output;
    motor->previous_core_output_valid = true;
#endif
    if (speed_ticks != 0U) {
        motor->speed_count = (uint16_t)(speed_ticks - 1U);
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
