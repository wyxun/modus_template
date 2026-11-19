/**
 * @file voltage_interval_test.c
 * @brief Voltage from accepted PWM across Motor control and identification.
 * @author Codex
 * @date 2026-11-12
 */
#include "mfoc_modulation.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stdio.h>

static void init_motor(mfoc_motor_t *motor)
{
    const mfoc_pi_params_t pi = {MFOC_ONE, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    const mfoc_motor_cfg_t cfg = {
        .current_base_milliamp = 10000U, .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE, MFOC_HALF, 0},
        .limits = {.i_max = MFOC_ONE}, .core = {pi, pi, MFOC_ONE}
    };
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(motor, &cfg) == MFOC_RESULT_OK);
}

static mfoc_ab_t applied(mfoc_scalar_t bus)
{
    mfoc_ab_t voltage = {0};
    assert(mfoc_modulation_ModelVoltageAlphaBeta(&motor_port_fake.duty, bus,
        &voltage) == MFOC_RESULT_OK);
    return voltage;
}

static void same_voltage(mfoc_ab_t actual, mfoc_ab_t expected)
{
    assert(actual.qAlpha == expected.qAlpha);
    assert(actual.qBeta == expected.qBeta);
}

/** @brief First/restarted samples have no history; neutral PWM is valid. */
static void control_history(void)
{
    mfoc_motor_t motor = {0};
    init_motor(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(!motor.previous_voltage_ab_valid);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_voltage_address == NULL);
    assert(motor.previous_voltage_ab_valid);
    assert(motor.previous_core_output_valid);
    same_voltage(motor.previous_voltage_ab, (mfoc_ab_t){0});

    const mfoc_dq_t cmd = {MFOC_SCALAR(0.7f), MFOC_SCALAR(-0.2f)};
    const mfoc_scalar_t low_bus = MFOC_SCALAR(0.8f);
    assert(mfoc_motor_SetCurrent(&motor, &cmd) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetBus(&motor, low_bus) == MFOC_RESULT_OK);
    motor_port_fake.feedback.u_inject_ab.qBeta = MFOC_HALF;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_voltage_address ==
           &motor.previous_voltage_ab);
    same_voltage(motor_port_fake.feedback_voltage, (mfoc_ab_t){0});
    assert(motor.previous_core_output.limited);
    mfoc_ab_t limited = applied(low_bus);
    assert(limited.qAlpha > MFOC_ZERO && limited.qBeta > MFOC_ZERO);
    same_voltage(motor.previous_voltage_ab, limited);

    const mfoc_scalar_t high_bus = MFOC_SCALAR(1.3f);
    assert(mfoc_motor_SetBus(&motor, high_bus) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    same_voltage(motor_port_fake.feedback_voltage, limited);
    same_voltage(motor.previous_voltage_ab, applied(high_bus));

    motor_port_fake.write = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(!motor_port_fake.enabled && !motor.previous_voltage_ab_valid);
    assert(!motor.previous_core_output_valid);
    assert((motor.faults & MFOC_MOTOR_FAULT_PWM) != 0U);
    motor_port_fake.write = MFOC_RESULT_OK;
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_voltage_address == NULL);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(!motor.previous_voltage_ab_valid);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_voltage_address == NULL);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}

#if MFOC_IDENTIFICATION_ENABLE
/** @brief Replay adjacent direct, virtual and regular intervals. */
static void identification_routes(void)
{
    mfoc_motor_t motor = {0};
    init_motor(&motor);
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(!motor.previous_core_output_valid);
    assert(motor.previous_voltage_ab_valid);
    mfoc_ab_t direct = applied(MFOC_ONE);
    assert(direct.qAlpha > MFOC_ZERO);
    same_voltage(motor.previous_voltage_ab, direct);

    /* Select the regular route for one replayed tick following direct PWM.
     * This isolates the history contract from experiment stage sequencing. */
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_NONE;
    assert(mfoc_motor_SetBus(&motor, MFOC_HALF) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_reads == 1U);
    assert(motor_port_fake.feedback_voltage_address ==
           &motor.previous_voltage_ab);
    same_voltage(motor_port_fake.feedback_voltage, direct);

    /* A held virtual stage uses its current reference and the real Core. */
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_FLUX;
    motor.identification.stage_done = true;
    const mfoc_dq_t cmd = {MFOC_HALF, MFOC_SCALAR(-0.1f)};
    motor.identification.work.rotating.i_ref = cmd; /* Last reference supplied by the virtual experiment. */
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.feedback_reads == 1U);
    assert(motor.previous_core_output_valid);
    assert(motor.previous_voltage_ab_valid);
    assert(motor.previous_core_output.limited);
    mfoc_ab_t virtual_voltage = applied(MFOC_HALF);
    assert(virtual_voltage.qAlpha > MFOC_ZERO);
    assert(virtual_voltage.qBeta < MFOC_ZERO);
    same_voltage(motor.previous_voltage_ab, virtual_voltage);
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_NONE;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    same_voltage(motor_port_fake.feedback_voltage, virtual_voltage);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}
#endif

int main(void)
{
    control_history();
#if MFOC_IDENTIFICATION_ENABLE
    identification_routes();
    puts("Direct and virtual identification intervals passed");
#endif
    puts("Motor interval history passed");
    return 0;
}
