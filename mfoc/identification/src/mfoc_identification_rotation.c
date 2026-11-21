/**
 * @file mfoc_identification_rotation.c
 * @brief Encoderless bounded spinup, block EMF observation and flux capture.
 * @author Codex
 * @date 2026-11-19
 */
#include "mfoc_identification_internal.h"
#include "mfoc_transform.h"
#include <stddef.h>

#define BLOCK_COUNT(ms) ((ms) * MFOC_IDENTIFICATION_SAMPLE_HZ /                 \
    (1000U * MFOC_ROTATION_BLOCK))
#define RAMP_BLOCKS BLOCK_COUNT(1000U)
#define SETTLE_BLOCKS BLOCK_COUNT(100U)
#define FLUX_BLOCKS BLOCK_COUNT(120U)
#define MAX_RUN_BLOCKS BLOCK_COUNT(5000U)
#define BAM_STEP_BASE (UINT64_C(4294967296) * MFOC_IDENTIFICATION_SPEED_BASE_HZ  \
    / MFOC_IDENTIFICATION_SAMPLE_HZ)
#define BLOCK_SPEED_GAIN MFOC_SCALAR((float)MFOC_IDENTIFICATION_SAMPLE_HZ /     \
    ((float)MFOC_ROTATION_BLOCK * (float)MFOC_IDENTIFICATION_SPEED_BASE_HZ))

typedef struct {
    mfoc_angle_t angle; /* Rotor at the center of this integrated block. */
    mfoc_scalar_t speed; /* Rotor speed at the preceding block boundary. */
    mfoc_dq_t current;
    mfoc_scalar_t emf;
} rotation_sample_t;

/** @brief Convert positive electrical PU speed to BAM advancement per ISR.
 * @param speed Bounded electrical speed. @return BAM32 increment. */
static uint32_t _rotation_StepFromSpeed(mfoc_scalar_t speed)
{
#if defined(MFOC_NUMERIC_FIXED)
    return (uint32_t)(((uint64_t)(uint32_t)speed * BAM_STEP_BASE) / MFOC_Q_SCALE);
#else
    return (uint32_t)(speed * (float)BAM_STEP_BASE);
#endif
}

void mfoc_identification_rotation_Init(mfoc_identification_rotation_t *s,
    mfoc_scalar_t current_max, mfoc_scalar_t speed_max)
{
    *s = (mfoc_identification_rotation_t){0};
    s->current_max = current_max;
    s->speed_max = speed_max;
    s->target_speed = mfoc_mul_wide(s->speed_max, MFOC_SCALAR(0.58f));
    s->ramp_increment = _rotation_StepFromSpeed(s->target_speed) / RAMP_BLOCKS;
    s->i_ref.qD = s->current_max / 2;
}

/** @brief Integrate extended EMF from preceding voltages and current endpoints.
 * @param o Observer with exact baseline current. @param model Stator parameters.
 * @param i Final current. @param flux_capture True to project the EMF magnitude.
 * @param sample Receives midpoint current, angle, optional EMF
 * and independently differentiated electrical speed. @return True after two
 * blocks with observable EMF, including reverse rotation or overspeed. */
static bool _rotation_Observe(mfoc_rotation_observer_t *o,
    const mfoc_motor_params_t *model, const mfoc_ab_t *i,
    bool flux_capture, rotation_sample_t *sample)
{
    const mfoc_ab_t di = {i->qAlpha - o->start_i.qAlpha,
        i->qBeta - o->start_i.qBeta};
    /* Retain B times the midpoint vector for phase extraction. Dividing the
     * components first costs low-order FIXED information at low speed.
     * E = <u> - Rs <i> - Lq (i_end-i_start)/dt_pu. For steady id its
     * q component is omega*(psi + (Ld-Lq)*id), even when Ld differs from Lq. */
    const mfoc_scalar_t n = (mfoc_scalar_t)MFOC_ROTATION_BLOCK;
    const mfoc_ab_t total_i = {o->sum_i.qAlpha - di.qAlpha / 2,
        o->sum_i.qBeta - di.qBeta / 2};
    const mfoc_scalar_t k = mfoc_mul_wide(model->lq, MFOC_SCALAR(
        (float)MFOC_IDENTIFICATION_SAMPLE_HZ / MFOC_MOTOR_BASE_ELECTRICAL_RAD_S));
    const mfoc_ab_t e = {o->sum_u.qAlpha -
        mfoc_mul_wide(model->rs, total_i.qAlpha) - mfoc_mul_wide(k, di.qAlpha),
        o->sum_u.qBeta - mfoc_mul_wide(model->rs, total_i.qBeta) -
        mfoc_mul_wide(k, di.qBeta)};
    o->sum_i = (mfoc_ab_t){0};
    o->sum_u = (mfoc_ab_t){0};
    o->start_i = *i;
    o->count = 0U;
    if (mfoc_abs(e.qAlpha) + mfoc_abs(e.qBeta) < MFOC_SCALAR(0.002f) * n) {
        o->angle_valid = false;
        return false;
    }
    sample->angle = mfoc_angle_atan2(-e.qAlpha, e.qBeta);
    sample->speed = mfoc_mul_wide(BLOCK_SPEED_GAIN,
        mfoc_angle_diff(sample->angle, o->previous_angle));
    const bool valid_speed = o->angle_valid;
    o->previous_angle = sample->angle;
    o->angle_valid = true;
    if (valid_speed) {
        mfoc_scalar_t sine = MFOC_ZERO, cosine = MFOC_ZERO;
        mfoc_angle_sincos(sample->angle, &sine, &cosine);
        const mfoc_ab_t mean_i = {total_i.qAlpha / n, total_i.qBeta / n};
        mfoc_park_cached(&mean_i, sine, cosine, &sample->current);
        if (flux_capture) {
            sample->emf = (mfoc_mul_wide(e.qBeta, cosine) -
                mfoc_mul_wide(e.qAlpha, sine)) / n;
        }
    }
    return valid_speed;
}

/** @brief Qualify positive synchronous motion and rotor-frame current stability.
 * @param s Virtual drive. @param m Observed midpoint quantities. @return Lock. */
static bool _rotation_Steady(const mfoc_identification_rotation_t *s,
    const rotation_sample_t *m)
{
    const mfoc_angle_t midpoint = {s->drive_angle.wBam32 -
        s->drive_step * MFOC_ROTATION_BLOCK / 2U};
    return s->elapsed_blocks >= RAMP_BLOCKS &&
        mfoc_abs(mfoc_angle_diff(m->angle, midpoint)) < MFOC_SCALAR(0.08f) &&
        mfoc_abs(m->speed - s->target_speed) <= s->target_speed / 8 &&
        mfoc_abs(m->speed - s->previous_speed) <= s->target_speed / 16 &&
        m->current.qD >= s->i_ref.qD / 2 &&
        mfoc_abs(m->current.qD - s->previous_current.qD) < MFOC_SCALAR(0.002f) &&
        mfoc_abs(m->current.qQ - s->previous_current.qQ) < MFOC_SCALAR(0.002f);
}

/** @brief Evaluate one integrated block and select the next excitation.
 * @param s Rotating state. @param stage Selected experiment. @param model Known
 * motor constants. @param i Current endpoint. @return Stage progress. */
static mfoc_fsm_rt_t _rotation_Block(mfoc_identification_rotation_t *s,
    mfoc_rotation_stage_t stage, const mfoc_motor_params_t *model,
    const mfoc_ab_t *i)
{
    rotation_sample_t m; /* Observe supplies exactly the fields for this stage. */
    const bool observed = _rotation_Observe(&s->observer, model, i,
        stage == MFOC_ROTATION_FLUX, &m);
    ++s->elapsed_blocks;
    if (s->elapsed_blocks >= MAX_RUN_BLOCKS) { return MFOC_FSM_FAILED; }
    if (observed && mfoc_abs(m.speed) > s->speed_max) { return MFOC_FSM_FAILED; }
    if (stage == MFOC_ROTATION_SPINUP) {
        if (s->elapsed_blocks <= RAMP_BLOCKS) {
            s->drive_step += s->ramp_increment;
        }
    }
    if (!observed) {
        s->stable_blocks = 0U;
        return stage == MFOC_ROTATION_SPINUP ? MFOC_FSM_ONGOING : MFOC_FSM_FAILED;
    }
    const bool steady = stage != MFOC_ROTATION_INERTIA && _rotation_Steady(s, &m);
    s->previous_speed = m.speed;
    s->previous_current = m.current;
    if (stage == MFOC_ROTATION_SPINUP) {
        s->stable_blocks = steady ? s->stable_blocks + 1U : 0U;
        return s->stable_blocks >= SETTLE_BLOCKS ?
            MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING;
    }
    if (stage == MFOC_ROTATION_FLUX) {
        if (!steady) { return MFOC_FSM_FAILED; }
        s->flux.sum_speed += m.speed;
        s->flux.sum_emf += m.emf;
        s->flux.sum_speed_id += mfoc_mul_wide(m.speed, m.current.qD);
        ++s->flux.count;
        return s->flux.count >= FLUX_BLOCKS ?
            MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING;
    }
    if (stage != MFOC_ROTATION_INERTIA || m.speed < s->speed_max / 4) {
        return MFOC_FSM_FAILED;
    }
    /* Inertia excitation follows the measured rotor. Advance the midpoint
     * phase to k before Core applies this block's virtual dq command. */
    s->drive_step = _rotation_StepFromSpeed(m.speed);
    s->drive_angle.wBam32 = m.angle.wBam32 +
        s->drive_step * MFOC_ROTATION_BLOCK / 2U;
    if (s->i_ref.qD > MFOC_ZERO) {
        /* Removing id in one step would create a saliency d-EMF impulse that
         * rotates the observed vector away from the magnet during handoff. */
        s->i_ref.qD = mfoc_sat(s->i_ref.qD - s->current_max / 64, MFOC_ZERO,
            s->current_max);
        s->i_ref.qQ = mfoc_sat((s->target_speed - m.speed) / 2,
            -s->current_max / 16, s->current_max / 16);
        return MFOC_FSM_ONGOING;
    }
    const mfoc_scalar_t active_flux = model->flux +
        mfoc_mul_wide(model->ld - model->lq, m.current.qD);
    return mfoc_identification_inertia_Block(s, m.speed,
        mfoc_mul_wide(active_flux, m.current.qQ));
}

mfoc_fsm_rt_t mfoc_identification_rotation_Step(mfoc_identification_rotation_t *s,
    mfoc_rotation_stage_t stage, const mfoc_motor_params_t *model,
    const mfoc_ab_t *current, const mfoc_ab_t *previous_voltage)
{
    mfoc_rotation_observer_t *o = &s->observer;
    /* SampleStage has already bounded |alpha| + |beta| and checked i[k]. */
    mfoc_fsm_rt_t status = MFOC_FSM_ONGOING;
    s->drive_angle.wBam32 += s->drive_step;
    if (!o->baseline) {
        o->start_i = *current;
        o->baseline = true;
    } else if (previous_voltage == NULL ||
        mfoc_abs(previous_voltage->qAlpha) > MFOC_SCALAR(8.0f) ||
        mfoc_abs(previous_voltage->qBeta) > MFOC_SCALAR(8.0f)) {
        return MFOC_FSM_FAILED;
    } else {
        o->sum_i.qAlpha += current->qAlpha;
        o->sum_i.qBeta += current->qBeta;
        o->sum_u.qAlpha += previous_voltage->qAlpha;
        o->sum_u.qBeta += previous_voltage->qBeta;
        ++o->count;
        if (o->count == MFOC_ROTATION_BLOCK) {
            status = _rotation_Block(s, stage, model, current);
        }
    }
    return status;
}

mfoc_fsm_rt_t mfoc_identification_rotation_Finish(
    const mfoc_identification_rotation_t *s, mfoc_rotation_stage_t stage,
    const mfoc_motor_params_t *model, mfoc_scalar_t *result)
{
    if (stage == MFOC_ROTATION_INERTIA) {
        return mfoc_identification_inertia_Finish(&s->inertia, result);
    }
    if (stage != MFOC_ROTATION_FLUX || s->flux.count != FLUX_BLOCKS ||
        s->flux.sum_speed <= MFOC_ZERO) { return MFOC_FSM_FAILED; }
    const mfoc_scalar_t residual = s->flux.sum_emf - mfoc_mul_wide(
        model->ld - model->lq, s->flux.sum_speed_id);
    mfoc_scalar_t flux = MFOC_ZERO;
    if (mfoc_div_checked(residual, s->flux.sum_speed, &flux) != MFOC_RESULT_OK ||
        flux <= MFOC_ZERO || flux > MFOC_SCALAR(MFOC_MOTOR_FLUX_MAX_PU)) {
        return MFOC_FSM_FAILED;
    }
    *result = flux;
    return MFOC_FSM_COMPLETE;
}
