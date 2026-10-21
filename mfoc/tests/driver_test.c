/**
 * @file driver_test.c
 * @brief Driver sampling, speed-loop cadence and fault integration.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_driver.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief Build a common current-loop or speed-loop configuration.
 * @param divider Zero for current mode. @return Configuration value. */
static mfoc_driver_cfg_t config(uint16_t divider)
{
    mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_driver_cfg_t cfg = {
        .motor = {
            .params = {7U, MFOC_SCALAR(0.1f), MFOC_SCALAR(0.2f),
                MFOC_SCALAR(0.2f), MFOC_SCALAR(0.4f), MFOC_ZERO},
            .limits = {MFOC_SCALAR(0.8f)},
            .core = {pi, pi, MFOC_SCALAR(0.55f), MFOC_ONE},
        },
        .speed = {MFOC_HALF, MFOC_SCALAR(0.1f),
            -MFOC_HALF, MFOC_HALF, -MFOC_HALF, MFOC_HALF},
        .speed_divider = divider,
    };
    return cfg;
}

/** @brief Check error reporting without partial initialization.
 * @return None. */
static void initialization(void)
{
    mfoc_driver_t driver = {0};
    mfoc_driver_cfg_t cfg = config(3U);
    assert(mfoc_driver_Init(NULL, &cfg) == MFOC_RESULT_NULL);
    assert(mfoc_driver_Init(&driver, NULL) == MFOC_RESULT_NULL);
    cfg.speed.out_max = MFOC_ONE;
    assert(mfoc_driver_Init(&driver, &cfg) == MFOC_RESULT_OUT_OF_RANGE);
    assert(driver.motor.state == MFOC_MOTOR_UNINITIALIZED);
    cfg = config(3U);
    cfg.speed.i_min = MFOC_ONE;
    assert(mfoc_driver_Init(&driver, &cfg) != MFOC_RESULT_OK);
    cfg = config(0U);
    cfg.speed = (mfoc_pi_params_t){0};
    assert(mfoc_driver_Init(&driver, &cfg) == MFOC_RESULT_OK);
    assert(driver.motor.state == MFOC_MOTOR_IDLE);
    assert(mfoc_driver_Run(NULL) == MFOC_RESULT_NULL);
    assert(mfoc_driver_Run(&driver) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_runs == 1U);
    motor_port_fake.feedback_run = MFOC_RESULT_IO;
    assert(mfoc_driver_Run(&driver) == MFOC_RESULT_IO);
    assert(driver.motor.state == MFOC_MOTOR_IDLE);
    motor_port_fake.feedback_run = MFOC_RESULT_OK;
#if defined(MFOC_NUMERIC_FLOAT)
    cfg = config(3U);
    cfg.speed.kp = NAN;
    assert(mfoc_driver_Init(&driver, &cfg) != MFOC_RESULT_OK);
    assert(driver.speed_divider == 0U);
#endif
}

/** @brief Check the single-sample current path and ADC failure shutdown.
 * @return None. */
static void current_control(void)
{
    mfoc_driver_t driver = {0};
    mfoc_driver_cfg_t cfg = config(0U);
    mfoc_dq_t ref = {MFOC_SCALAR(0.2f), MFOC_SCALAR(-0.1f)};
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_driver_Init(&driver, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_driver_SetSpeed(&driver, MFOC_HALF) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_INVALID_STATE);
    assert(motor_port_fake.samples == 0U);
    assert(mfoc_driver_SetCurrent(&driver, &ref) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&driver.motor) == MFOC_RESULT_OK);
    motor_port_fake.current = (mfoc_uvw_t){MFOC_SCALAR(0.1f),
        MFOC_SCALAR(-0.05f), MFOC_SCALAR(-0.05f)};
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_OK);
    assert(motor_port_fake.samples == 1U);
    motor_port_fake.current = (mfoc_uvw_t){MFOC_SCALAR(0.1f),
        MFOC_SCALAR(0.1f), MFOC_SCALAR(-0.2f)};
    motor_port_fake.feedback.angle.wBam32 = 0x40000000U;
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_OK);
    unsigned writes = motor_port_fake.writes;
    motor_port_fake.sample = MFOC_RESULT_IO;
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_IO);
    assert(motor_port_fake.writes == writes);
    assert(!motor_port_fake.enabled);
    mfoc_motor_status_t status = {0};
    assert(mfoc_motor_GetStatus(&driver.motor, &status) == MFOC_RESULT_OK);
    assert(status.state == MFOC_MOTOR_FAULT);
    assert((status.faults & MFOC_MOTOR_FAULT_SAMPLE) != 0U);
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_INVALID_STATE);
    assert(motor_port_fake.samples == 3U);
    assert(mfoc_motor_ClearFault(&driver.motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&driver.motor) == MFOC_RESULT_OK);
    motor_port_fake.disable = MFOC_RESULT_SAFETY;
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_SAFETY);
    assert(mfoc_motor_GetStatus(&driver.motor, &status) == MFOC_RESULT_OK);
    uint32_t faults = MFOC_MOTOR_FAULT_SAMPLE | MFOC_MOTOR_FAULT_PWM;
    assert((status.faults & faults) == faults);
    assert(motor_port_fake.enabled);
    motor_port_fake.disable = MFOC_RESULT_OK;
    assert(mfoc_driver_Stop(&driver) == MFOC_RESULT_OK);
}

/** @brief Check speed PI on steps 1/4/7 and fresh state after restarting.
 * @return None. */
static void speed_control(void)
{
    mfoc_driver_t driver = {0};
    mfoc_driver_cfg_t cfg = config(3U);
    mfoc_dq_t ref = {0};
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_driver_Init(&driver, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_driver_SetCurrent(&driver, &ref) ==
           MFOC_RESULT_INVALID_STATE);
    assert(mfoc_driver_SetSpeed(&driver, MFOC_SCALAR(0.4f)) ==
           MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&driver.motor) == MFOC_RESULT_OK);
    for (unsigned step = 0U; step < 7U; ++step) {
        if (step >= 4U) {
            motor_port_fake.feedback.speed = MFOC_SCALAR(0.4f);
        }
        assert(mfoc_driver_Step(&driver) == MFOC_RESULT_OK);
        float expected = step < 3U ? 0.24f : (step < 6U ? 0.28f : 0.08f);
        assert(fabsf(mfoc_to_float(driver.motor.command.i_ref.qQ) -
                     expected) < 0.0004f);
        assert(driver.motor.command.i_ref.qD == MFOC_ZERO);
    }
    mfoc_scalar_t integral = driver.speed.ui;
    assert(mfoc_motor_Start(&driver.motor) == MFOC_RESULT_INVALID_STATE);
    assert(driver.speed.ui == integral);
    assert(mfoc_motor_Trip(&driver.motor, MFOC_MOTOR_FAULT_EXTERNAL) ==
           MFOC_RESULT_OK);
    assert(mfoc_driver_Stop(&driver) == MFOC_RESULT_OK);
    assert(mfoc_motor_ClearFault(&driver.motor) == MFOC_RESULT_OK);
    assert(mfoc_driver_SetSpeed(&driver, MFOC_SCALAR(0.4f)) ==
           MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&driver.motor) == MFOC_RESULT_OK);
    assert(driver.speed.ui == MFOC_ZERO && driver.speed_count == 0U);
    motor_port_fake.feedback.speed = MFOC_ZERO;
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(driver.motor.command.i_ref.qQ) - 0.24f) <
           0.0004f);
    assert(mfoc_driver_Stop(&driver) == MFOC_RESULT_OK);
    assert(driver.speed.ui == MFOC_ZERO && driver.speed_count == 0U);
    assert(driver.speed_ref == MFOC_ZERO);
    assert(mfoc_driver_SetSpeed(&driver, MFOC_SCALAR(0.2f)) ==
           MFOC_RESULT_OK);
    motor_port_fake.feedback.speed = MFOC_ZERO;
    assert(mfoc_motor_Start(&driver.motor) == MFOC_RESULT_OK);
    assert(mfoc_driver_Step(&driver) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(driver.motor.command.i_ref.qQ) - 0.12f) <
           0.0004f);
    assert(mfoc_driver_Stop(&driver) == MFOC_RESULT_OK);
}

/** @brief Run both numeric-backend integration scenarios.
 * @return Zero on success. */
int main(void)
{
    initialization();
    current_control();
    speed_control();
    puts("Driver control cadence and fault shutdown passed");
    return 0;
}
