/**
 * @file per_unit_invariance_test.c
 * @brief Replay one SI trajectory with interleaved Motor and observer states.
 * @author Codex
 * @date 2026-11-20
 */
#include "motor_port_fake.h"
#include "mfoc_ekf.h"
#include "mfoc_flux.h"
#include "mfoc_smo.h"
#include "mfoc_transform.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TAU 6.283185307179586
#define SQRT3 1.732050807568877
#define RS_OHM 2.55
#define LS_H 0.00086
#define PM_WB 0.0035
#define BUS_V 12.0
#define SPEED_HZ 72.0
#define ID_A 0.24
#define IQ_A 0.31
#define KP_V_PER_A 1.7
#define KI_V_PER_AS 460.0

typedef struct {
    unsigned ma;
    unsigned mv;
    unsigned speed_hz;
    unsigned sample_hz;
} bases_t;

typedef struct {
    double alpha;
    double beta;
} vector_t;

typedef struct {
    double angle;
    double speed;
} observer_error_t;

typedef struct {
    observer_error_t smo;
    observer_error_t flux;
#if MFOC_FEEDBACK_ENABLE_EKF
    observer_error_t ekf;
#endif
} observer_errors_t;

typedef struct {
    bases_t bases;
    mfoc_motor_t motor;
    motor_port_fake_t port;
    mfoc_smo_t smo;
    mfoc_flux_t flux;
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_ekf_t ekf;
#endif
    observer_errors_t errors;
    double integral_d;
    double integral_q;
    double peak_voltage_error;
    mfoc_smo_output_t smo_out;
    mfoc_flux_output_t flux_out;
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_ekf_output_t ekf_out;
#endif
} replay_t;

/** @brief Encode a bounded PU input for the selected numeric backend.
 * @param value Input PU. @return Encoded value. */
static mfoc_scalar_t encode(double value)
{
    return mfoc_from_float((float)value);
}

/** @brief Initialize normalized instances from identical physical tuning.
 * @param r Replay state and chosen bases. @return None. */
static void start(replay_t *r)
{
    const bases_t b = r->bases;
    const double ib = b.ma / 1000.0;
    const double vb = b.mv / 1000.0;
    const double fb = b.speed_hz;
    const double psi_base = vb / (TAU * fb);
    const mfoc_motor_params_t model = {
        .pole_pairs = 7U, .rs = encode(RS_OHM * ib / vb),
        .ld = encode(LS_H * ib / psi_base),
        .lq = encode(LS_H * ib / psi_base),
        .flux = encode(PM_WB / psi_base),
    };
    const mfoc_pi_params_t pi = {
        encode(KP_V_PER_A * ib / vb),
        encode(KI_V_PER_AS * ib / (vb * b.sample_hz)),
        encode(-5.0 / vb), encode(5.0 / vb),
        encode(-4.0 / vb), encode(4.0 / vb),
    };
    const mfoc_motor_cfg_t motor_cfg = {
        .params = model, .current_base_milliamp = b.ma,
        .voltage_base_millivolt = b.mv,
        .electrical_speed_base_hz = encode(fb),
        .core = {pi, pi, encode(BUS_V / vb)},
        .limits = {
            .i_max = encode(1.0 / ib), .speed_max = encode(100.0 / fb),
        },
    };
    motor_port_fake = (motor_port_fake_t){.bus_configured = true,
        .bus = encode(BUS_V / vb)};
    assert(mfoc_motor_Init(&r->motor, &motor_cfg) == MFOC_RESULT_OK);
    assert(motor_port_fake.port_cfg.current_base_milliamp == b.ma);
    assert(motor_port_fake.port_cfg.voltage_base_millivolt == b.mv);
    assert(motor_port_fake.port_cfg.electrical_speed_base_hz == encode(fb));
    assert(mfoc_motor_Start(&r->motor) == MFOC_RESULT_OK);
    r->port = motor_port_fake;

    const mfoc_smo_cfg_t smo_cfg = {
        .sample_hz = b.sample_hz, .cutoff_rad_s = 4000U,
        .speed_base_hz = encode(fb), .h = encode(5.0 / vb),
        .boundary = encode(0.60 / ib), .i_limit = encode(2.0 / ib),
    };
    assert(mfoc_smo_Init(&r->smo, &smo_cfg, &model) == MFOC_RESULT_OK);
    const double flux_scale = psi_base / (12.0 / (TAU * 400.0));
    const mfoc_flux_cfg_t flux_cfg = {
        .sample_hz = b.sample_hz, .speed_base_hz = encode(fb),
        .gain = encode(0.1 * 400.0 / fb * flux_scale * flux_scale),
        .pll_kp = encode(0.5 * 400.0 / fb),
        .pll_ki = encode(100.0 * 400.0 / fb),
    };
    assert(mfoc_flux_Init(&r->flux, &flux_cfg, &model) == MFOC_RESULT_OK);
    assert(r->smo.c.speed_span == b.sample_hz / 1000U);
    assert(fabs(mfoc_to_float(r->flux.c.angle_step) -
        fb / b.sample_hz) <= 1.0 / 32768.0);
#if MFOC_FEEDBACK_ENABLE_EKF
    const float current_variance_scale = (float)(3.5 * 3.5 / (ib * ib));
    const float speed_variance_scale = (float)(400.0 * 400.0 / (fb * fb));
    const float q_dt = 20000.0f / (float)b.sample_hz;
    const mfoc_ekf_cfg_t ekf_cfg = {
        .sample_hz = b.sample_hz, .speed_base_hz = encode(fb),
        .q = {1e-6f * q_dt * current_variance_scale,
              1e-6f * q_dt * current_variance_scale,
              1e-7f * q_dt * speed_variance_scale, 1e-8f * q_dt},
        .r = {1e-5f * current_variance_scale, 0.0f,
              1e-5f * current_variance_scale},
        .p0 = {0.01f * current_variance_scale, 0.01f * current_variance_scale,
               0.25f * speed_variance_scale, 0.08333333f},
        .min_speed = (float)(8.0 / fb), .max_angle_var = 0.01f,
        .max_speed_var = 0.1f * speed_variance_scale,
    };
    assert(mfoc_ekf_Init(&r->ekf, &ekf_cfg, &model) == MFOC_RESULT_OK);
    assert(fabs(r->ekf.c.kt - fb / b.sample_hz) < 1e-8);
#endif
}

/** @brief Rotate an independently defined physical dq pair to alpha-beta.
 * @param d First component. @param q Second component. @param rad Angle.
 * @return Rotated components. */
static vector_t rotate(double d, double q, double rad)
{
    return (vector_t){d * cos(rad) - q * sin(rad),
                      d * sin(rad) + q * cos(rad)};
}

/** @brief Convert a physical alpha-beta value at the interface.
 * @param vector Physical quantity. @param base Amperes or volts per unit.
 * @return Alpha-beta PU. */
static mfoc_ab_t normalize(vector_t vector, double base)
{
    return (mfoc_ab_t){encode(vector.alpha / base),
                       encode(vector.beta / base)};
}

/** @brief Accumulate wrapped angle and signed electrical speed errors.
 * @param error Peak errors. @param angle Estimated angle.
 * @param speed_hz Estimated frequency. @param turns Reference angle.
 * @param expected_hz Reference frequency.
 * @return None. */
static void observe_error(observer_error_t *error, mfoc_angle_t angle,
                          double speed_hz, double turns, double expected_hz)
{
    const double delta = mfoc_angle_to_turns(angle) - turns;
    error->angle = fmax(error->angle, fabs(delta - floor(delta + 0.5)));
    error->speed = fmax(error->speed, fabs(speed_hz - expected_hz));
}

/** @brief Sample the same PMSM trajectory and step actual Motor/controller.
 * @param r One instance and its fake power stage. @param k Current frame.
 * @return None. */
static void step(replay_t *r, unsigned k)
{
    const bases_t b = r->bases;
    const double ib = b.ma / 1000.0;
    const double vb = b.mv / 1000.0;
    const double dt = 1.0 / b.sample_hz;
    const double t = k * dt;
    const double turns = 0.07 + SPEED_HZ * t;
    const double angle = TAU * turns;
    const double delta = TAU * SPEED_HZ * dt;
    const vector_t current = rotate(ID_A, IQ_A, angle);
    const vector_t r_drop = rotate(RS_OHM * ID_A, RS_OHM * IQ_A,
                                   angle - delta * 0.5);
    const vector_t psi = rotate(LS_H * ID_A + PM_WB, LS_H * IQ_A, angle);
    const vector_t prev_psi = rotate(LS_H * ID_A + PM_WB, LS_H * IQ_A,
                                     angle - delta);
    /* The mean R*i over the interval is its midpoint value times sinc. */
    const double sinc = sin(delta * 0.5) / (delta * 0.5);
    const vector_t voltage = {r_drop.alpha * sinc +
        (psi.alpha - prev_psi.alpha) / dt, r_drop.beta * sinc +
        (psi.beta - prev_psi.beta) / dt};

    motor_port_fake = r->port;
    motor_port_fake.current = (mfoc_uvw_t){
        encode(current.alpha / ib),
        encode((SQRT3 * current.beta - current.alpha) / (2.0 * ib)),
        encode((-SQRT3 * current.beta - current.alpha) / (2.0 * ib)),
    };
    motor_port_fake.feedback.angle = mfoc_angle_from_turns((float)turns);
    motor_port_fake.feedback.speed = encode(SPEED_HZ / b.speed_hz);
    const double error_d = 0.08 * cos(TAU * 19.0 * t);
    const double error_q = 0.11 * cos(TAU * 23.0 * t);
    const mfoc_dq_t command = {encode((ID_A + error_d) / ib),
                             encode((IQ_A + error_q) / ib)};
    assert(mfoc_motor_SetCurrent(&r->motor, &command) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&r->motor) == MFOC_RESULT_OK);
    assert(r->motor.previous_core_output_valid);
    const mfoc_core_output_t *out = &r->motor.previous_core_output;
    assert(!out->limited);
    assert(fabs(mfoc_to_float(out->i.qD) * ib - ID_A) < 0.0012);
    assert(fabs(mfoc_to_float(out->i.qQ) * ib - IQ_A) < 0.0012);
    r->port = motor_port_fake;
    assert(r->port.samples == k && r->port.writes == k + 1U);
    assert(r->motor.previous_voltage_ab_valid);
    const double du = mfoc_to_float(r->port.duty.qU);
    const double dv = mfoc_to_float(r->port.duty.qV);
    const double dw = mfoc_to_float(r->port.duty.qW);
    const mfoc_ab_t model_u = r->motor.previous_voltage_ab;
    assert(fabs(mfoc_to_float(model_u.qAlpha) * vb -
        BUS_V * (2.0 * du - dv - dw) / 3.0) < 0.002);
    assert(fabs(mfoc_to_float(model_u.qBeta) * vb -
        BUS_V * (dv - dw) / SQRT3) < 0.002);

    r->integral_d += KI_V_PER_AS * error_d * dt;
    r->integral_q += KI_V_PER_AS * error_q * dt;
    const double u_d = KP_V_PER_A * error_d + r->integral_d;
    const double u_q = KP_V_PER_A * error_q + r->integral_q;
    r->peak_voltage_error = fmax(r->peak_voltage_error, fmax(
        fabs(mfoc_to_float(out->u.qD) * vb - u_d),
        fabs(mfoc_to_float(out->u.qQ) * vb - u_q)));

    mfoc_ab_t i = {0};
    const mfoc_uvw_t phase = r->port.current;
    mfoc_clarke(phase.qU, phase.qV, phase.qW, &i);
    const mfoc_ab_t u = normalize(voltage, vb);
    mfoc_smo_Step(&r->smo, &i, &u, &r->smo_out);
    mfoc_flux_Step(&r->flux, &i, &u, &r->flux_out);
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_ekf_Step(&r->ekf, &i, &u, &r->ekf_out);
#endif
    if (t >= 0.4) {
        assert(r->smo_out.valid && r->flux_out.valid);
        observe_error(&r->errors.smo, r->smo_out.angle,
            mfoc_to_float(r->smo_out.speed) * b.speed_hz, turns, SPEED_HZ);
        observe_error(&r->errors.flux, r->flux_out.angle,
            mfoc_to_float(r->flux_out.speed) * b.speed_hz, turns, SPEED_HZ);
#if MFOC_FEEDBACK_ENABLE_EKF
        assert(r->ekf_out.valid);
        observe_error(&r->errors.ekf, r->ekf_out.angle,
            mfoc_to_float(r->ekf_out.speed) * b.speed_hz, turns, SPEED_HZ);
#endif
    }
}

/** @brief Compare the same-rate PWM commands after scaling the inputs.
 * @param first First set of bases. @param second Second set.
 * @return None. */
static void compare_duty(const replay_t *first, const replay_t *second)
{
    const mfoc_uvw_t a = first->port.duty;
    const mfoc_uvw_t b = second->port.duty;
    assert(fabs(mfoc_to_float(a.qU) - mfoc_to_float(b.qU)) < 0.002);
    assert(fabs(mfoc_to_float(a.qV) - mfoc_to_float(b.qV)) < 0.002);
    assert(fabs(mfoc_to_float(a.qW) - mfoc_to_float(b.qW)) < 0.002);
}

/** @brief Compare same-rate observers after undoing their PU scaling.
 * @param errors Peak differences. @param a First set of bases.
 * @param b Second set of bases. @return None. */
static void compare_observers(observer_errors_t *errors, const replay_t *a,
                              const replay_t *b)
{
    const double fa = a->bases.speed_hz;
    const double fb = b->bases.speed_hz;
    observe_error(&errors->smo, a->smo_out.angle,
        mfoc_to_float(a->smo_out.speed) * fa,
        mfoc_angle_to_turns(b->smo_out.angle),
        mfoc_to_float(b->smo_out.speed) * fb);
    observe_error(&errors->flux, a->flux_out.angle,
        mfoc_to_float(a->flux_out.speed) * fa,
        mfoc_angle_to_turns(b->flux_out.angle),
        mfoc_to_float(b->flux_out.speed) * fb);
#if MFOC_FEEDBACK_ENABLE_EKF
    observe_error(&errors->ekf, a->ekf_out.angle,
        mfoc_to_float(a->ekf_out.speed) * fa,
        mfoc_angle_to_turns(b->ekf_out.angle),
        mfoc_to_float(b->ekf_out.speed) * fb);
#endif
}

/** @brief Check physical-unit errors independently across both sample rates.
 * @param r Completed replay. @return None. */
static void check(const replay_t *r)
{
    printf("%u Hz Ibase=%u mA Vbase=%u mV fbase=%u Hz "
           "Core %.6f V SMO %.4f deg/%.3f Hz Flux %.4f deg/%.3f Hz\n",
        r->bases.sample_hz, r->bases.ma, r->bases.mv, r->bases.speed_hz,
        r->peak_voltage_error, 360.0 * r->errors.smo.angle,
        r->errors.smo.speed, 360.0 * r->errors.flux.angle,
        r->errors.flux.speed);
#if MFOC_FEEDBACK_ENABLE_EKF
    printf("  EKF %.4f deg/%.3f Hz\n",
        360.0 * r->errors.ekf.angle, r->errors.ekf.speed);
#endif
    fflush(stdout);
#if defined(MFOC_NUMERIC_FIXED)
    /* The integral accumulates the current quantization over one second. */
    assert(r->peak_voltage_error < 0.07);
#else
    assert(r->peak_voltage_error < 0.001);
#endif
    assert(r->errors.smo.angle < 2.0 / 360.0 && r->errors.smo.speed < 1.0);
    assert(r->errors.flux.angle < 1.0 / 360.0 && r->errors.flux.speed < 1.0);
#if MFOC_FEEDBACK_ENABLE_EKF
    assert(r->errors.ekf.angle < 2.0 / 360.0 && r->errors.ekf.speed < 1.0);
#endif
}

/** @brief Execute independent instances for the same physical duration.
 * @return Zero after successful comparisons. */
int main(void)
{
    replay_t first = {.bases = {3500U, 12000U, 400U, 20000U}};
    replay_t second = {.bases = {4000U, 14000U, 320U, 20000U}};
    replay_t slower = {.bases = {4000U, 14000U, 320U, 16000U}};
    start(&first);
    start(&second);
    start(&slower);
    observer_errors_t differences = {0};
    unsigned other = 1U;
    for (unsigned k = 1U; k <= first.bases.sample_hz; ++k) {
        step(&first, k);
        step(&second, k);
        compare_duty(&first, &second);
        if (k >= first.bases.sample_hz * 2U / 5U) {
            compare_observers(&differences, &first, &second);
        }
        if (other * first.bases.sample_hz <= k * slower.bases.sample_hz) {
            step(&slower, other);
            ++other;
        }
    }
    assert(other == slower.bases.sample_hz + 1U);
    check(&first);
    check(&second);
    check(&slower);
    printf("Same-rate base differences SMO %.5f deg/%.5f Hz "
           "Flux %.5f deg/%.5f Hz\n",
        360.0 * differences.smo.angle, differences.smo.speed,
        360.0 * differences.flux.angle, differences.flux.speed);
#if MFOC_FEEDBACK_ENABLE_EKF
    printf("  EKF %.5f deg/%.5f Hz\n",
        360.0 * differences.ekf.angle, differences.ekf.speed);
#endif
    fflush(stdout);
    assert(differences.smo.angle < 0.2 / 360.0);
    assert(differences.smo.speed < 0.5);
    assert(differences.flux.angle < 0.2 / 360.0);
    assert(differences.flux.speed < 0.5);
#if MFOC_FEEDBACK_ENABLE_EKF
    assert(differences.ekf.angle < 0.01 / 360.0);
    assert(differences.ekf.speed < 0.01);
#endif
    return 0;
}
