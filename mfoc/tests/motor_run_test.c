/**
 * @file motor_run_test.c
 * @brief Motor lifecycle, fault containment and current-control integration.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include "mfoc_modulation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#if !MFOC_FEEDBACK_ENABLE_HFI
_Static_assert(sizeof(mfoc_feedback_output_t) == 4U * sizeof(mfoc_scalar_t),
    "HFI-disabled feedback must contain only angle, speed and voltage");
#endif

/** @brief Prepare a current-controlled motor with a known inactive port.
 * @param motor Destination. @return None. */
static void setup(mfoc_motor_t *motor)
{
    mfoc_pi_params_t pi = {MFOC_SCALAR(0.5f), MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_SCALAR(0.1f), MFOC_SCALAR(0.2f),
            MFOC_SCALAR(0.2f), MFOC_SCALAR(0.4f), MFOC_ZERO},
        .limits = {.i_max = MFOC_SCALAR(0.8f)},
        .core = {pi, pi, MFOC_ONE},
    };
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(motor, &cfg) == MFOC_RESULT_OK);
}

/** @brief Prove fault state rejects PWM and normal Stop retains the latch.
 * @param motor Faulted instance. @param fault Expected bit.
 * @return None. */
static void check_fault(mfoc_motor_t *motor, uint32_t fault)
{
    mfoc_motor_status_t status = {0};
    mfoc_motor_GetStatus(motor, &status);
    assert(status.state == MFOC_MOTOR_FAULT);
    assert((status.faults & fault) != 0U);
    assert(!motor_port_fake.enabled);
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Stop(motor) == MFOC_RESULT_OK);
    assert(motor->state == MFOC_MOTOR_FAULT);
    assert(mfoc_motor_ClearFault(motor) == MFOC_RESULT_OK);
    assert(motor->state == MFOC_MOTOR_IDLE);
    assert(!motor_port_fake.enabled);
}

#if MFOC_FEEDBACK_ENABLE_HFI
/** @brief Freeze speed PI/cadence during acquisition and resume on readiness.
 * @return None. */
static void speed_acquisition(void)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.01f),
        -MFOC_HALF, MFOC_HALF, -MFOC_HALF, MFOC_HALF};
    const mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE, MFOC_HALF, 0},
        .limits = {MFOC_ONE, MFOC_ONE}, .core = {pi, pi, MFOC_ONE},
        .speed = pi, .speed_divider = 3U};
    mfoc_motor_t motor = {0};
    motor_port_fake = (motor_port_fake_t){0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetSpeed(&motor, MFOC_HALF) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.feedback.acquiring = true;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.speed_count == 0U && motor.speed.ui == MFOC_ZERO);
    motor_port_fake.feedback.acquiring = false;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.speed_count == 2U && motor.i_ref.qQ > MFOC_ZERO);
    mfoc_scalar_t integral = motor.speed.ui;
    motor_port_fake.feedback.acquiring = true;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.speed_count == 2U && motor.speed.ui == integral);
    motor_port_fake.feedback.acquiring = false;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.speed_count == 1U && motor.speed.ui == integral);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
}
#endif

/** @brief Regulate speed before qualification, without a current step.
 * @return None. */
static void sensorless_speed_acquisition(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        mfoc_motor_t motor = {0};
        setup(&motor);
        assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
        motor.speed.cfg = (mfoc_pi_params_t){MFOC_HALF,
            MFOC_SCALAR(0.01f), -MFOC_HALF, MFOC_HALF,
            -MFOC_HALF, MFOC_HALF};
        motor.speed_divider = 3U;
        motor.speed_ref = mfoc_from_float(0.4f * (float)direction);
        motor.i_ref.qQ = mfoc_from_float(0.05f * (float)direction);
        motor.sensorless_start.state = MFOC_SENSORLESS_START_ACQUIRE;
        motor.sensorless_start.captured = true;
        motor.sensorless_start.offset = 10;
        motor.sensorless_start.offset_step = 10;
        motor.sensorless_start.remaining = 1000U;
        motor.sensorless_start.stable_ticks = 4U;
        motor.sensorless_start.speed_window = MFOC_SCALAR(0.04f);
        motor_port_fake.feedback.speed = motor.speed_ref;
        mfoc_scalar_t initial = motor.i_ref.qQ;
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor.sensorless_start.state ==
            MFOC_SENSORLESS_START_ACQUIRE);
        assert(motor.mode == MFOC_MOTOR_MODE_SPEED);
        assert(motor.i_ref.qQ == initial && motor.speed_count == 2U);
        motor_port_fake.feedback.speed =
            mfoc_from_float(0.5f * (float)direction);
        for (unsigned n = 0; n < 3U; ++n) {
            assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        }
        assert(mfoc_to_float(motor.i_ref.qQ) * (float)direction <
               mfoc_to_float(initial) * (float)direction);
        assert(motor.sensorless_start.stable == 0U);
        motor_port_fake.feedback.speed = motor.speed_ref;
        for (unsigned n = 0; n < 4U; ++n) {
            assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        }
        assert(motor.sensorless_start.state == MFOC_SENSORLESS_START_NONE);
        motor.sensorless_start.state = MFOC_SENSORLESS_START_ACQUIRE;
        motor.sensorless_start.remaining = 0U;
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_SAFETY);
        check_fault(&motor, MFOC_MOTOR_FAULT_FEEDBACK);
        assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
        motor.sensorless_start.state = MFOC_SENSORLESS_START_ACQUIRE;
        motor.sensorless_start.captured = true;
        motor.sensorless_start.remaining = 10U;
        motor_port_fake.feedback_step = MFOC_RESULT_IO;
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
        check_fault(&motor, MFOC_MOTOR_FAULT_FEEDBACK);
    }
}

/** @brief Align without ramp coefficients; calibrate before Stop resets phase.
 * @return None. */
static void fixed_angle(void)
{
    mfoc_motor_t motor = {0};
    setup(&motor);
    assert(motor.openloop.gain == 0);
    assert(mfoc_motor_AlignStart(&motor, MFOC_SCALAR(0.2f),
        MFOC_SCALAR(0.25f)) == MFOC_RESULT_OK);
    assert(motor.openloop.mode == MFOC_OPENLOOP_ALIGN);
    for (unsigned n = 0; n < 10U; ++n) {
        mfoc_openloop_Step(&motor.openloop);
        assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
        assert(motor.openloop.angle.wBam32 == UINT32_C(0x40000000));
    }
    assert(motor_port_fake.feedback_reads == 0U);
    motor_port_fake.time_ms = MFOC_MOTOR_ALIGN_TIME_MS + 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.held_angle.wBam32 == UINT32_C(0x40000000));
    assert(motor.state == MFOC_MOTOR_IDLE && !motor_port_fake.enabled);
}

/** @brief Exercise the complete port-bound current-control path.
 * @return Zero if every assertion passes. */
int main(void)
{
    mfoc_motor_t motor = {0};
    fixed_angle();
    sensorless_speed_acquisition();
    mfoc_dq_t ref = {MFOC_SCALAR(0.2f), MFOC_SCALAR(-0.1f)};
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_INVALID_STATE);
    setup(&motor);
    assert(motor_port_fake.writes == 0U);
    assert(mfoc_motor_SetCurrent(&motor, &ref) == MFOC_RESULT_OK);
    mfoc_dq_t excessive = {MFOC_SCALAR(0.7f), MFOC_SCALAR(0.7f)};
    assert(mfoc_motor_SetCurrent(&motor, &excessive) ==
           MFOC_RESULT_OUT_OF_RANGE);
    assert(motor.i_ref.qD == ref.qD);
    assert(motor.i_ref.qQ == ref.qQ);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.enabled);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.previous_core_output.u.qD > MFOC_ZERO &&
           motor.previous_core_output.u.qQ < MFOC_ZERO);
    assert(motor_port_fake.writes == 2U); /* Start plus one Step. */
    assert(motor.previous_core_output_valid);
    assert(motor.previous_core_output.duty.qU == motor_port_fake.duty.qU);
    assert(mfoc_motor_SetBus(&motor, MFOC_SCALAR(0.9f)) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetBus(&motor, MFOC_ZERO) != MFOC_RESULT_OK);
    assert(motor.state == MFOC_MOTOR_RUNNING);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(motor.i_ref.qD == MFOC_ZERO);
    assert(motor.core.d.ui == MFOC_ZERO && motor.core.q.ui == MFOC_ZERO);
    assert(!motor_port_fake.enabled);

    setup(&motor);
    motor_port_fake.write = MFOC_RESULT_OUT_OF_RANGE;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OUT_OF_RANGE);
    assert(motor_port_fake.enables == 0U);
    motor_port_fake.write = MFOC_RESULT_OK;
    check_fault(&motor, MFOC_MOTOR_FAULT_PWM);

    setup(&motor);
    motor_port_fake.enable = MFOC_RESULT_IO;
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_IO);
    check_fault(&motor, MFOC_MOTOR_FAULT_PWM);

    setup(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.previous_core_output_valid);
    motor_port_fake.feedback_step = MFOC_RESULT_IO;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_IO);
    assert(!motor.previous_core_output_valid);
    assert(motor_port_fake.writes == 2U);
    check_fault(&motor, MFOC_MOTOR_FAULT_FEEDBACK);

    setup(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.disable = MFOC_RESULT_IO;
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_IO);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_IO);
    assert(motor.state == MFOC_MOTOR_FAULT);
    motor_port_fake.disable = MFOC_RESULT_OK;
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    check_fault(&motor, MFOC_MOTOR_FAULT_PWM);

    setup(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    unsigned writes = motor_port_fake.writes;
    assert(mfoc_motor_Trip(&motor, MFOC_MOTOR_FAULT_EXTERNAL) ==
           MFOC_RESULT_OK);
    assert(motor_port_fake.writes == writes);
    check_fault(&motor, MFOC_MOTOR_FAULT_EXTERNAL);

    setup(&motor);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.feedback.angle.wBam32 = UINT32_C(0x40000000);
    motor_port_fake.feedback.u_inject_ab =
        (mfoc_ab_t){MFOC_SCALAR(0.2f), MFOC_SCALAR(-0.1f)};
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    mfoc_ab_t voltage = {0};
    assert(mfoc_modulation_ModelVoltageAlphaBeta(&motor_port_fake.duty,
        motor.previous_core_output.bus, &voltage) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(voltage.qAlpha) - 0.2f) < 0.0003f);
    assert(fabsf(mfoc_to_float(voltage.qBeta) + 0.1f) < 0.0003f);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_ENABLE_HFI
    /* Acquisition applies excitation while deferring the stored command. */
    setup(&motor);
    assert(mfoc_motor_SetCurrent(&motor, &ref) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    motor_port_fake.feedback.acquiring = true;
    motor_port_fake.feedback.u_inject_ab.qAlpha = MFOC_SCALAR(0.1f);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(motor.previous_core_output.u.qD) -
        0.1f) < 0.0003f);
    assert(motor.previous_core_output.u.qQ == MFOC_ZERO);
    assert(motor.i_ref.qD == ref.qD);
    motor_port_fake.feedback.acquiring = false;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    assert(motor.previous_core_output.u.qD > MFOC_SCALAR(0.1f));
    assert(motor.previous_core_output.u.qQ < MFOC_ZERO);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    speed_acquisition();
#endif
    puts("Motor control, startup rejection and fault recovery passed");
    return 0;
}
