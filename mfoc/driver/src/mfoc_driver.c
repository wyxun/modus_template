/**
 * @file mfoc_driver.c
 * @brief Top-level current acquisition and cascaded speed/current loops.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_driver.h"
#include "mfoc_port.h"
#include "mfoc_transform.h"
#include <stddef.h>

/** @brief Validate finite speed gains and current-domain output limits.
 * @param p Speed PI parameters. @param limit Motor current radius.
 * @return True when parameters fit the initialized Motor. */
static bool _mfoc_driver_SpeedValid(const mfoc_pi_params_t *p,
                                   mfoc_scalar_t limit)
{
    return mfoc_scalar_is_finite(p->kp) &&
        mfoc_scalar_is_finite(p->ki_ts) &&
        mfoc_scalar_is_finite(p->out_min) &&
        mfoc_scalar_is_finite(p->out_max) &&
        mfoc_scalar_is_finite(p->i_min) &&
        mfoc_scalar_is_finite(p->i_max) &&
        p->out_min >= -limit && p->out_max <= limit &&
        p->out_min <= MFOC_ZERO && p->out_max >= MFOC_ZERO &&
        p->i_min <= MFOC_ZERO && p->i_max >= MFOC_ZERO;
}

/** @brief Latch a scheduling failure, prioritizing shutdown errors.
 * @param driver Instance. @param fault Motor fault bit.
 * @param cause Original failure. @return Shutdown error or original cause. */
static mfoc_result_t _mfoc_driver_Fail(mfoc_driver_t *driver,
                                      uint32_t fault, mfoc_result_t cause)
{
    driver->previous_core_output_valid = false;
    mfoc_result_t rc = mfoc_motor_Trip(&driver->motor, fault);
    return rc == MFOC_RESULT_OK ? cause : rc;
}

/** @brief Validate configuration and initialize owned control instances.
 * @param driver Destination. @param cfg Control configuration.
 * @return Initialization status. @note Port Init samples offsets while gated. */
mfoc_result_t mfoc_driver_Init(mfoc_driver_t *driver,
                              const mfoc_driver_cfg_t *cfg)
{
    if (driver == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    mfoc_driver_t next = {0};
    mfoc_result_t rc = mfoc_motor_Init(&next.motor, &cfg->motor);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_feedback_Init(&next.feedback, &cfg->feedback,
                            cfg->motor.params.pole_pairs);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (cfg->speed_divider != 0U) {
        if (!_mfoc_driver_SpeedValid(&cfg->speed, cfg->motor.limits.i_max)) {
            return MFOC_RESULT_OUT_OF_RANGE;
        }
        rc = mfoc_pi_Init(&next.speed, &cfg->speed);
        if (rc != MFOC_RESULT_OK) { return rc; }
    }
    next.speed_divider = cfg->speed_divider;
    next.irq_result = MFOC_RESULT_INVALID_STATE;
    rc = mfoc_port_init();
    if (rc != MFOC_RESULT_OK) { return rc; }
    *driver = next;
    return MFOC_RESULT_OK;
}

/** @brief Poll enabled foreground feedback services at their own cadences.
 * @param driver Instance. @return Backend service status. */
mfoc_result_t mfoc_driver_Run(mfoc_driver_t *driver)
{
    if (driver == NULL) { return MFOC_RESULT_NULL; }
    return mfoc_feedback_Run(&driver->feedback);
}

/** @brief Disable Motor before clearing outer-loop history and command.
 * @param driver Instance. @return Shutdown error takes priority. */
mfoc_result_t mfoc_driver_Stop(mfoc_driver_t *driver)
{
    if (driver == NULL) { return MFOC_RESULT_NULL; }
    driver->previous_core_output_valid = false;
    mfoc_result_t rc = mfoc_motor_Stop(&driver->motor);
    if (rc == MFOC_RESULT_INVALID_STATE) { return rc; }
    if (driver->speed_divider != 0U) {
        mfoc_result_t reset = mfoc_pi_Reset(&driver->speed);
        if (rc == MFOC_RESULT_OK) { rc = reset; }
    }
    driver->speed_ref = MFOC_ZERO;
    driver->speed_count = 0U;
    return rc;
}

/** @brief Refresh the bus from the latest MDI-published ADC mean.
 * @param driver Initialized instance. @return Sampling/update status. */
mfoc_result_t mfoc_driver_UpdateBus(mfoc_driver_t *driver)
{
    if (driver == NULL) { return MFOC_RESULT_NULL; }
    mfoc_motor_status_t status = {0};
    mfoc_result_t rc = mfoc_motor_GetStatus(&driver->motor, &status);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (status.state != MFOC_MOTOR_IDLE &&
        status.state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_scalar_t next = MFOC_ZERO;
    rc = mfoc_port_bus(&next);
    if (rc != MFOC_RESULT_OK) { return rc; }
    return mfoc_motor_SetBus(&driver->motor, next);
}

/** @brief Publish a current reference in current-control configuration.
 * @param driver Instance. @param ref dq current reference.
 * @return Motor command status or wrong-mode rejection. */
mfoc_result_t mfoc_driver_SetCurrent(mfoc_driver_t *driver,
                                    const mfoc_dq_t *ref)
{
    if (driver == NULL || ref == NULL) { return MFOC_RESULT_NULL; }
    if (driver->speed_divider != 0U) { return MFOC_RESULT_INVALID_STATE; }
    return mfoc_motor_SetCurrent(&driver->motor, ref);
}

/** @brief Accept a finite speed reference without changing PI history.
 * @param driver Instance. @param ref Speed reference in PU.
 * @return Status; rejected commands leave the previous reference intact. */
mfoc_result_t mfoc_driver_SetSpeed(mfoc_driver_t *driver,
                                  mfoc_scalar_t ref)
{
    if (driver == NULL) { return MFOC_RESULT_NULL; }
    if (driver->speed_divider == 0U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_motor_status_t status = {0};
    mfoc_result_t rc = mfoc_motor_GetStatus(&driver->motor, &status);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (status.state != MFOC_MOTOR_IDLE &&
        status.state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (!mfoc_scalar_is_finite(ref)) { return MFOC_RESULT_OUT_OF_RANGE; }
    driver->speed_ref = ref;
    return MFOC_RESULT_OK;
}

/** @brief Schedule one coherent current frame and optional outer-loop tick.
 * @param driver Instance. @return Port/control status.
 * @note No runtime feedback-source dispatch or duplicated sample checks. */
mfoc_result_t mfoc_driver_Step(mfoc_driver_t *driver)
{
    if (driver == NULL) { return MFOC_RESULT_NULL; }
    mfoc_motor_status_t status = {0};
    mfoc_result_t rc = mfoc_motor_GetStatus(&driver->motor, &status);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (status.state != MFOC_MOTOR_RUNNING) {
        driver->previous_core_output_valid = false;
        return MFOC_RESULT_INVALID_STATE;
    }
    mfoc_uvw_t phase = {0};
    rc = mfoc_port_current(&phase);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_driver_Fail(driver, MFOC_MOTOR_FAULT_SAMPLE, rc);
    }
    mfoc_motor_input_t motor_input = {0};
    rc = mfoc_clarke(phase.qU, phase.qV, phase.qW, &motor_input.i_ab);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_driver_Fail(driver, MFOC_MOTOR_FAULT_MATH, rc);
    }
    mfoc_feedback_input_t feedback_input = {
        .i_ab_k = &motor_input.i_ab,
        .previous_core_output = driver->previous_core_output_valid ?
            &driver->previous_core_output : NULL};
    mfoc_feedback_output_t feedback = {0};
    rc = mfoc_feedback_Step(&driver->feedback, &feedback_input, &feedback);
    if (rc != MFOC_RESULT_OK) {
        return _mfoc_driver_Fail(driver, MFOC_MOTOR_FAULT_FEEDBACK, rc);
    }
    motor_input.angle = feedback.angle;
    motor_input.angle_valid = true;
    if (driver->speed_divider != 0U && driver->speed_count == 0U) {
        mfoc_dq_t ref = {0};
        rc = mfoc_pi_Step(&driver->speed, driver->speed_ref,
                          feedback.speed, &ref.qQ);
        if (rc != MFOC_RESULT_OK) {
            return _mfoc_driver_Fail(driver, MFOC_MOTOR_FAULT_MATH, rc);
        }
        rc = mfoc_motor_SetCurrent(&driver->motor, &ref);
        if (rc != MFOC_RESULT_OK) {
            return _mfoc_driver_Fail(driver, MFOC_MOTOR_FAULT_MATH, rc);
        }
        driver->speed_count = driver->speed_divider;
    }
    mfoc_core_output_t output = {0};
    rc = mfoc_motor_Step(&driver->motor, &motor_input, &output);
    if (rc == MFOC_RESULT_OK) {
        driver->previous_core_output = output;
        driver->previous_core_output_valid = true;
    } else {
        driver->previous_core_output_valid = false;
    }
    if (rc == MFOC_RESULT_OK && driver->speed_count != 0U) {
        driver->speed_count--;
    }
    return rc;
}
