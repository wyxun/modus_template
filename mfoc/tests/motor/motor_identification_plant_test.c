/**
 * @file motor_identification_plant_test.c
 * @brief End-to-end identification from simulated PMSM phase currents.
 * @author Codex
 * @date 2026-11-19
 */
#include "mfoc_motor.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Independent SI fixture. Mild saliency distinguishes swapped pulse axes. */
#define POLE_PAIRS 7U
#define IB_A 3.5
#define VB_V 12.0
#define WB_RAD_S (6.283185307179586 * 400.0)
#define RB_OHM (VB_V / IB_A)
#define LB_H (RB_OHM / WB_RAD_S)
#define PSIB_WB (VB_V / WB_RAD_S)
#define TB_NM (1.5 * POLE_PAIRS * PSIB_WB * IB_A)
#define JB_KGM2 (TB_NM * POLE_PAIRS / (WB_RAD_S * WB_RAD_S))

typedef struct {
    double rs;
    double ld;
    double lq;
    double psi;
    double inertia;
} physical_motor_t;

/* Returning compiled defaults cannot satisfy the salient fixture tolerances. */
static const physical_motor_t salient = {
    2.75, 0.00080, 0.00102, 0.0039, 4.7e-6};
static const physical_motor_t stock_2804 = {
    2.55, 0.00086, 0.00086, 0.0035, 3.7e-6};
static const double friction = 1.5e-5; /* N m s / mechanical rad. */

typedef enum {
    SHAFT_LOCKED, /* Zero speed, d-axis physically aligned to alpha. */
    SHAFT_FREE,
    SHAFT_DRIVEN, /* External prime mover imposes speed during capture. */
} shaft_t;

typedef struct {
    double id_a;
    double iq_a;
    double angle_e;
    double omega_m;
} state_t;

typedef struct {
    const physical_motor_t *motor;
    state_t state;
    shaft_t shaft;
    double peak_current_pu;
    double peak_speed_e;
} plant_t;

typedef struct {
    double alpha;
    double beta;
} volts_t;

typedef struct {
    mfoc_motor_t motor;
    mfoc_motor_params_t model; /* Caller-owned result and next operation seed. */
} session_t;

/** @brief Convert a physical boundary value to the selected scalar.
 * @param value Unquantized PU value. @return Port or configuration scalar. */
static mfoc_scalar_t pu(double value)
{
    return mfoc_from_float((float)value);
}

/** @brief Compute winding and shaft derivatives in amperes, radians, seconds.
 * @param s Physical state. @param v Stator voltages after neutral removal.
 * @param p Fixture parameters and constraint. @return State derivatives. */
static state_t derivative(const state_t *s, volts_t v, const plant_t *p)
{
    const physical_motor_t *m = p->motor;
    const double c = cos(s->angle_e);
    const double sn = sin(s->angle_e);
    const double vd = c * v.alpha + sn * v.beta;
    const double vq = -sn * v.alpha + c * v.beta;
    const double omega_e = POLE_PAIRS * s->omega_m;
    const double torque = 1.5 * POLE_PAIRS *
        (m->psi + (m->ld - m->lq) * s->id_a) * s->iq_a;
    return (state_t){
        .id_a = (vd - m->rs * s->id_a + omega_e * m->lq * s->iq_a) / m->ld,
        .iq_a = (vq - m->rs * s->iq_a -
                 omega_e * (m->ld * s->id_a + m->psi)) / m->lq,
        .angle_e = omega_e,
        .omega_m = p->shaft == SHAFT_FREE ?
            (torque - friction * s->omega_m) / m->inertia : 0.0,
    };
}

/** @brief Integrate a derivative from a known state.
 * @param s Starting state. @param d Derivatives. @param h Seconds.
 * @return Advanced state. */
static state_t advance(state_t s, state_t d, double h)
{
    return (state_t){s.id_a + h * d.id_a, s.iq_a + h * d.iq_a,
        s.angle_e + h * d.angle_e, s.omega_m + h * d.omega_m};
}

/** @brief Apply real PWM duty for one complete interval then sample phases.
 * @param p Plant with one simulated PWM period of delay. @return None. */
static void plant_interval(plant_t *p)
{
    const double du = mfoc_to_float(motor_port_fake.duty.qU);
    const double dv = mfoc_to_float(motor_port_fake.duty.qV);
    const double dw = mfoc_to_float(motor_port_fake.duty.qW);
    assert(du >= 0.0 && du <= 1.0 && dv >= 0.0 && dv <= 1.0);
    assert(dw >= 0.0 && dw <= 1.0);
    const double vdc = VB_V * mfoc_to_float(motor_port_fake.bus);
    const volts_t v = {vdc * (2.0 * du - dv - dw) / 3.0,
        vdc * (dv - dw) / sqrt(3.0)};
    const double h = 1.0 / (4.0 * MFOC_MOTOR_RUN_FREQ_HZ);
    for (unsigned k = 0U; k < 4U; ++k) {
        state_t d = derivative(&p->state, v, p);
        const state_t mid = advance(p->state, d, 0.5 * h);
        d = derivative(&mid, v, p);
        p->state = advance(p->state, d, h);
        p->peak_speed_e = fmax(p->peak_speed_e,
            fabs(POLE_PAIRS * p->state.omega_m));
        p->peak_current_pu = fmax(p->peak_current_pu,
            hypot(p->state.id_a, p->state.iq_a) / IB_A);
    }
    const state_t *s = &p->state;
    const double i_alpha = (s->id_a * cos(s->angle_e) -
                            s->iq_a * sin(s->angle_e)) / IB_A;
    const double i_beta = (s->id_a * sin(s->angle_e) +
                           s->iq_a * cos(s->angle_e)) / IB_A;
    motor_port_fake.current = (mfoc_uvw_t){pu(i_alpha),
        pu((-i_alpha + sqrt(3.0) * i_beta) * 0.5),
        pu((-i_alpha - sqrt(3.0) * i_beta) * 0.5)};
}

/** @brief Verify the integrator against both analytical locked-winding rises.
 * @return None. */
static void check_locked_windings(void)
{
    plant_t p = {.motor = &salient, .shaft = SHAFT_LOCKED};
    /* Fixed duties give vd = 3 V, vq = sqrt(3) V at theta = 0. */
    motor_port_fake = (motor_port_fake_t){.bus = MFOC_ONE,
        .duty = {pu(0.75), pu(0.5), pu(0.25)}};
    const unsigned intervals = 8U;
    for (unsigned n = 0U; n < intervals; ++n) { plant_interval(&p); }
    const double t = (double)intervals / MFOC_MOTOR_RUN_FREQ_HZ;
    const double expected_d = 3.0 *
        (1.0 - exp(-t * salient.rs / salient.ld)) / salient.rs;
    const double expected_q = sqrt(3.0) *
        (1.0 - exp(-t * salient.rs / salient.lq)) / salient.rs;
    assert(fabs(p.state.id_a - expected_d) < expected_d * 0.0003);
    assert(fabs(p.state.iq_a - expected_q) < expected_q * 0.0003);
    assert(p.state.angle_e == 0.0 && p.state.omega_m == 0.0);
}

/** @brief Start with wrong seeds for exactly the parameters being measured.
 * @param op Public identification operation. @param m SI fixture.
 * @return PU model. */
static mfoc_motor_params_t seed_model(mfoc_identification_operation_t op,
                                      const physical_motor_t *m)
{
    const bool electrical = op == MFOC_IDENTIFICATION_ELECTRICAL;
    const bool resistance = op == MFOC_IDENTIFICATION_RESISTANCE;
    return (mfoc_motor_params_t){
        .pole_pairs = POLE_PAIRS,
        .rs = pu(m->rs / RB_OHM * (electrical || resistance ? 0.7 : 1.0)),
        .ld = pu(m->ld / LB_H * (electrical ? 1.4 : 1.0)),
        .lq = pu(m->lq / LB_H * (electrical ? 0.7 : 1.0)),
        .flux = pu(m->psi / PSIB_WB *
            (op == MFOC_IDENTIFICATION_FLUX ? 0.75 : 1.0)),
        .inertia = pu(m->inertia / JB_KGM2 *
            (op == MFOC_IDENTIFICATION_INERTIA ? 0.65 : 1.0)),
    };
}

/** @brief Initialize Motor once per fixture without a source of rotor feedback.
 * @param s Test session with the initial model. @return None. */
static void init(session_t *s)
{
    const mfoc_pi_params_t pi = {pu(0.472810), pu(0.070097),
        pu(-0.55), pu(0.55), -MFOC_HALF, MFOC_HALF};
    const mfoc_motor_cfg_t cfg = {
        .current_base_milliamp = 3500U, .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = MFOC_SCALAR(400.0f), .params = s->model,
        .limits = {.i_max = pu(0.12), .speed_max = pu(200.0 / WB_RAD_S)},
        .core = {pi, pi, MFOC_ONE},
    };
    motor_port_fake = (motor_port_fake_t){
        .position_run = MFOC_RESULT_IO, .position_step = MFOC_RESULT_IO,
        .feedback_step = MFOC_RESULT_IO,
    };
    assert(mfoc_motor_Init(&s->motor, &cfg) == MFOC_RESULT_OK);
}

/** @brief Check recovered SI values against independently defined constants.
 * @param m Published model. @param op Experiment.
 * @param seed Initial parameters, preserved outside the measurement.
 * @param actual SI fixture. @return None. */
static void check_result(mfoc_motor_params_t m,
    mfoc_identification_operation_t op, mfoc_motor_params_t seed,
    const physical_motor_t *actual)
{
    const double r = mfoc_to_float(m.rs) * RB_OHM;
    const double d = mfoc_to_float(m.ld) * LB_H;
    const double q = mfoc_to_float(m.lq) * LB_H;
    const double flux = mfoc_to_float(m.flux) * PSIB_WB;
    const double j = mfoc_to_float(m.inertia) * JB_KGM2;
    assert(m.pole_pairs == seed.pole_pairs);
    if (op == MFOC_IDENTIFICATION_ELECTRICAL ||
        op == MFOC_IDENTIFICATION_RESISTANCE) {
        printf("  Rs=%.5f ohm\n", r);
        assert(fabs(r - actual->rs) < actual->rs * 0.005);
    } else {
        assert(m.rs == seed.rs);
    }
    if (op == MFOC_IDENTIFICATION_ELECTRICAL) {
        printf("  Ld=%.5f mH Lq=%.5f mH\n", d * 1e3, q * 1e3);
        assert(fabs(d - actual->ld) < actual->ld * 0.01);
        assert(fabs(q - actual->lq) < actual->lq * 0.01);
    } else {
        assert(m.ld == seed.ld && m.lq == seed.lq);
    }
    if (op == MFOC_IDENTIFICATION_FLUX) {
        printf("  flux=%.5f mWb\n", flux * 1e3);
        assert(fabs(flux - actual->psi) < actual->psi * 0.02);
    } else {
        assert(m.flux == seed.flux);
    }
    if (op == MFOC_IDENTIFICATION_INERTIA) {
        printf("  J=%.5g kg m^2\n", j);
        assert(fabs(j - actual->inertia) < actual->inertia * 0.05);
    } else {
        assert(m.inertia == seed.inertia);
    }
}

/** @brief Run Motor against the SI plant until the foreground disables PWM.
 * @param s Session, with caller-supplied seed and persistent Motor.
 * @param op Public operation. @param plant Physical fixture and peak readings.
 * @param drive_speed_e Prime mover speed during measurement, electrical rad/s.
 * @param cancel_stage Stage at which to interrupt capture; NONE runs to completion.
 * @return Foreground status after PWM shutdown. */
static mfoc_result_t run(session_t *s, mfoc_identification_operation_t op,
                         plant_t *plant, double drive_speed_e,
                         mfoc_identification_stage_t cancel_stage)
{
    static const char *const names[] = {
        [MFOC_IDENTIFICATION_ELECTRICAL] = "ELECTRICAL",
        [MFOC_IDENTIFICATION_RESISTANCE] = "RESISTANCE",
        [MFOC_IDENTIFICATION_FLUX] = "FLUX",
        [MFOC_IDENTIFICATION_INERTIA] = "INERTIA",
    };
    mfoc_motor_t *motor = &s->motor;
    const unsigned disables = motor_port_fake.disables;
    const mfoc_tick_t start = motor_port_fake.time_ms;
    printf("%s Rs=%.2f ohm theta0=%.2f rad omega_e0=%.1f rad/s\n",
        names[op], plant->motor->rs, plant->state.angle_e,
        plant->state.omega_m * POLE_PAIRS);
    assert(mfoc_identification_Start(&motor->identification, &s->model, op,
        MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Run(motor) == MFOC_RESULT_OK);
    mfoc_result_t rc = MFOC_RESULT_OK;
    unsigned n = 0U;
    const unsigned samples_per_ms = MFOC_MOTOR_RUN_FREQ_HZ / 1000U;
    const mfoc_identification_stage_t stages[] = {
        MFOC_IDENTIFICATION_STAGE_ALIGN, MFOC_IDENTIFICATION_STAGE_RS,
        MFOC_IDENTIFICATION_STAGE_LD, MFOC_IDENTIFICATION_STAGE_LQ};
    unsigned stage_index = 0U;
    for (; n < MFOC_MOTOR_RUN_FREQ_HZ * 18U && motor_port_fake.enabled; ++n) {
        /* The free rotor must reach measurement before overspeed/reversal.
         * Coupling the external shaft changes speed with continuous angle. */
        if (drive_speed_e != 0.0 && plant->shaft == SHAFT_FREE &&
            (motor->identification.stage == MFOC_IDENTIFICATION_STAGE_FLUX ||
             motor->identification.stage == MFOC_IDENTIFICATION_STAGE_INERTIA)) {
            plant->shaft = SHAFT_DRIVEN;
            plant->state.omega_m = drive_speed_e / POLE_PAIRS;
        }
        plant_interval(plant);
        assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
        if (mfoc_identification_GetStatus(&motor->identification) ==
            MFOC_IDENTIFICATION_ERROR) {
            assert(motor_port_fake.duty.qU == MFOC_HALF);
            assert(motor_port_fake.duty.qV == MFOC_HALF);
            assert(motor_port_fake.duty.qW == MFOC_HALF);
        }
        if (cancel_stage != MFOC_IDENTIFICATION_STAGE_NONE &&
            motor->identification.stage == cancel_stage) {
            rc = mfoc_motor_Stop(motor);
            break;
        }
        if ((n + 1U) % samples_per_ms == 0U) {
            motor_port_fake.time_ms = start + (n + 1U) / samples_per_ms;
            rc = mfoc_motor_Run(motor);
            if (op == MFOC_IDENTIFICATION_ELECTRICAL &&
                motor_port_fake.enabled &&
                motor->identification.stage != stages[stage_index]) {
                ++stage_index;
                assert(stage_index < sizeof stages / sizeof stages[0]);
                assert(motor->identification.stage == stages[stage_index]);
            }
        }
    }
    printf("  drive=%g samples=%u peakI=%.6f PU "
           "peak|omega_e|=%.3f rad/s stage=%d result=%d\n",
        drive_speed_e, n, plant->peak_current_pu,
        plant->peak_speed_e, (int)motor->identification.stage, (int)rc);
    assert(!motor_port_fake.enabled && motor_port_fake.disables == disables + 1U);
    assert(motor_port_fake.position_reads == 0U);
    assert(motor_port_fake.feedback_reads == 0U);
#if !MFOC_FEEDBACK_ENABLE_POSITION
    assert(motor_port_fake.position_runs == 0U);
#endif
    if (op == MFOC_IDENTIFICATION_ELECTRICAL && rc == MFOC_RESULT_OK &&
        cancel_stage == MFOC_IDENTIFICATION_STAGE_NONE) {
        assert(stage_index == 3U);
    }
    return rc;
}

/** @brief Verify a successful measurement and retain its published parameters.
 * @param s Persistent session. @param op Experiment. @param plant SI fixture.
 * @return None. */
static void measure(session_t *s, mfoc_identification_operation_t op,
                    plant_t plant)
{
    const mfoc_motor_params_t seed = s->model;
    assert(run(s, op, &plant, 0.0, MFOC_IDENTIFICATION_STAGE_NONE) == MFOC_RESULT_OK);
    assert(s->motor.state == MFOC_MOTOR_IDLE);
    assert(mfoc_identification_GetResult(&s->motor.identification,
        &s->model) == MFOC_RESULT_OK);
    assert(plant.peak_current_pu <= 0.12 && plant.peak_speed_e <= 200.0);
    if (plant.shaft == SHAFT_FREE) { assert(plant.peak_speed_e > 40.0); }
    check_result(s->model, op, seed, plant.motor);
}

/** @brief Isolate one successful or rejected experiment from earlier results.
 * @param op Experiment. @param plant SI fixture. @param drive_speed_e rad/s.
 * @param success Expected publication outcome. @return None. */
static void experiment(mfoc_identification_operation_t op, plant_t plant,
                       double drive_speed_e, bool success)
{
    session_t s = {.model = seed_model(op, plant.motor)};
    init(&s);
    if (success) {
        measure(&s, op, plant);
        return;
    }
    assert(run(&s, op, &plant, drive_speed_e, MFOC_IDENTIFICATION_STAGE_NONE) == MFOC_RESULT_SAFETY);
    assert(s.motor.state == MFOC_MOTOR_FAULT);
    if (drive_speed_e != 0.0) { assert(plant.shaft == SHAFT_DRIVEN); }
    assert(mfoc_identification_GetStatus(&s.motor.identification) ==
        MFOC_IDENTIFICATION_ERROR);
    mfoc_motor_params_t result = {.pole_pairs = 19U};
    assert(mfoc_identification_GetResult(&s.motor.identification,
        &result) == MFOC_RESULT_INVALID_STATE);
    assert(result.pole_pairs == 19U);
}

/** @brief Stop an active capture before it can publish a partial model. */
static void cancel_capture(mfoc_identification_operation_t op, plant_t plant,
                           mfoc_identification_stage_t stage)
{
    session_t s = {.model = seed_model(op, plant.motor)};
    init(&s);
    assert(run(&s, op, &plant, 0.0, stage) == MFOC_RESULT_OK);
    assert(s.motor.state == MFOC_MOTOR_IDLE);
    assert(mfoc_identification_GetStatus(&s.motor.identification) ==
        MFOC_IDENTIFICATION_IDLE);
    mfoc_motor_params_t result = {.pole_pairs = 19U};
    assert(mfoc_identification_GetResult(&s.motor.identification, &result) ==
        MFOC_RESULT_INVALID_STATE && result.pole_pairs == 19U);
}

/** @brief Carry the measured model across three public operations on one Motor.
 * @param locked Fixed, aligned rotor. @param free_rotor Released, initially
 * resting rotor for each no-load rotating measurement. @return None. */
static void measure_chain(plant_t locked, plant_t free_rotor)
{
    session_t s = {.model = seed_model(MFOC_IDENTIFICATION_ELECTRICAL,
                                       locked.motor)};
    s.model.flux = pu(0.75 * locked.motor->psi / PSIB_WB);
    s.model.inertia = pu(0.65 * locked.motor->inertia / JB_KGM2);
    const mfoc_motor_params_t original = s.model;
    init(&s);
    puts("Sequential measurements using the preceding published model");
    measure(&s, MFOC_IDENTIFICATION_ELECTRICAL, locked);
    measure(&s, MFOC_IDENTIFICATION_FLUX, free_rotor);
    measure(&s, MFOC_IDENTIFICATION_INERTIA, free_rotor);
    assert(motor_port_fake.port_inits == 1U && motor_port_fake.disables == 3U);
    assert(s.motor.params.rs == original.rs && s.motor.params.ld == original.ld);
    assert(s.motor.params.lq == original.lq && s.motor.params.flux ==
        original.flux && s.motor.params.inertia == original.inertia);
}

/** @brief Exercise the four public operations and rejected rotor conditions.
 * @return Zero after successful verification. */
int main(void)
{
    check_locked_windings();
    const plant_t stock = {.motor = &stock_2804, .shaft = SHAFT_LOCKED};
    const plant_t locked = {.motor = &salient, .shaft = SHAFT_LOCKED};
    const plant_t free_rotor = {.motor = &salient, .shaft = SHAFT_FREE,
        .state = {.angle_e = 0.55}};
    plant_t moving = free_rotor;
    experiment(MFOC_IDENTIFICATION_ELECTRICAL, stock, 0.0, true);
    experiment(MFOC_IDENTIFICATION_RESISTANCE, locked, 0.0, true);
    experiment(MFOC_IDENTIFICATION_ELECTRICAL, locked, 0.0, true);
    experiment(MFOC_IDENTIFICATION_FLUX, free_rotor, 0.0, true);
    experiment(MFOC_IDENTIFICATION_INERTIA, free_rotor, 0.0, true);
    moving.state = (state_t){.angle_e = -2.5, .omega_m = -50.0 / POLE_PAIRS};
    experiment(MFOC_IDENTIFICATION_FLUX, moving, 0.0, true);
    moving.state = (state_t){.angle_e = 2.6, .omega_m = 50.0 / POLE_PAIRS};
    experiment(MFOC_IDENTIFICATION_INERTIA, moving, 0.0, true);
    experiment(MFOC_IDENTIFICATION_FLUX, locked, 0.0, false);
    experiment(MFOC_IDENTIFICATION_INERTIA, locked, 0.0, false);
    experiment(MFOC_IDENTIFICATION_FLUX, free_rotor, 230.0, false);
    experiment(MFOC_IDENTIFICATION_INERTIA, free_rotor, -110.0, false);
    cancel_capture(MFOC_IDENTIFICATION_ELECTRICAL, locked,
        MFOC_IDENTIFICATION_STAGE_LQ);
    cancel_capture(MFOC_IDENTIFICATION_FLUX, free_rotor,
        MFOC_IDENTIFICATION_STAGE_FLUX);
    cancel_capture(MFOC_IDENTIFICATION_INERTIA, free_rotor,
        MFOC_IDENTIFICATION_STAGE_INERTIA);
    measure_chain(locked, free_rotor);
    return 0;
}
