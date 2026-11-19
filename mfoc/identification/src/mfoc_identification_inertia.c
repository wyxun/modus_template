/**
 * @file mfoc_identification_inertia.c
 * @brief Matched accelerating and braking torque impulses on a free shaft.
 * @author Codex
 * @date 2026-11-19
 */
#include "mfoc_identification_rotation.h"

enum { SEEK_TOP, SEEK_DOWN, CAPTURE_DOWN, SEEK_BOTTOM, SEEK_UP, CAPTURE_UP,
       CAPTURE_DONE };
enum { BRAKING, ACCELERATING, MAX_BLOCKS = 512U };

/** @brief Integrate midpoint torque over intervals between speed estimates.
 * @param s Capture. @param leg Braking or accelerating. @return Window fits. */
static bool _inertia_Accumulate(mfoc_rotation_inertia_t *s, unsigned leg)
{
    if (s->blocks[leg] >= MAX_BLOCKS) { return false; }
    s->sum_torque[leg] += s->previous_torque;
    ++s->blocks[leg];
    return true;
}

mfoc_fsm_rt_t mfoc_identification_inertia_Block(mfoc_identification_rotation_t *s,
    mfoc_scalar_t speed, mfoc_scalar_t torque)
{
    mfoc_rotation_inertia_t *w = &s->inertia;
    const mfoc_scalar_t high = mfoc_mul_wide(s->speed_max, MFOC_SCALAR(0.66f));
    const mfoc_scalar_t low = s->speed_max / 2;
    const mfoc_scalar_t margin = s->speed_max / 16;
    const mfoc_scalar_t excitation = mfoc_sat(s->current_max / 16,
        MFOC_ZERO, MFOC_SCALAR(0.006f));
    /* The electrical model supplies torque at this block's midpoint. Speed
     * from consecutive EMF angles belongs half a block earlier. The previous
     * midpoint torque thus covers the interval ending at this speed sample. */
    switch (w->phase) {
    case SEEK_TOP:
        s->i_ref = (mfoc_dq_t){MFOC_ZERO, excitation};
        if (speed >= high + margin) {
            s->i_ref.qQ = -excitation;
            w->phase = SEEK_DOWN;
        }
        break;
    case SEEK_DOWN:
        if (speed <= high) { w->start_speed = speed; w->phase = CAPTURE_DOWN; }
        break;
    case CAPTURE_DOWN:
        if (!_inertia_Accumulate(w, BRAKING)) { return MFOC_FSM_FAILED; }
        if (speed <= low) {
            w->delta_speed[BRAKING] = speed - w->start_speed;
            w->phase = SEEK_BOTTOM;
        }
        break;
    case SEEK_BOTTOM:
        if (speed <= low - margin) {
            s->i_ref.qQ = excitation;
            w->phase = SEEK_UP;
        }
        break;
    case SEEK_UP:
        if (speed >= low) { w->start_speed = speed; w->phase = CAPTURE_UP; }
        break;
    case CAPTURE_UP:
        if (!_inertia_Accumulate(w, ACCELERATING)) { return MFOC_FSM_FAILED; }
        if (speed >= high) {
            w->delta_speed[ACCELERATING] = speed - w->start_speed;
            w->phase = CAPTURE_DONE;
            return MFOC_FSM_COMPLETE;
        }
        break;
    default:
        return MFOC_FSM_FAILED;
    }
    w->previous_torque = torque;
    return MFOC_FSM_ONGOING;
}

mfoc_fsm_rt_t mfoc_identification_inertia_Finish(
    const mfoc_rotation_inertia_t *w, mfoc_scalar_t *result)
{
    const uint32_t up = w->blocks[ACCELERATING], down = w->blocks[BRAKING];
    if (w->phase != CAPTURE_DONE || up < 3U || down < 3U ||
        up > MAX_BLOCKS || down > MAX_BLOCKS ||
        w->delta_speed[ACCELERATING] <= MFOC_ZERO ||
        w->delta_speed[BRAKING] >= MFOC_ZERO) { return MFOC_FSM_FAILED; }
    /* Subtract the mean torque equations for two traversals of the same
     * positive speed band. A constant drag cancels even for unequal times. */
#if defined(MFOC_NUMERIC_FIXED)
    const int64_t impulse = (int64_t)w->sum_torque[ACCELERATING] * down -
        (int64_t)w->sum_torque[BRAKING] * up;
    const int64_t delta = (int64_t)w->delta_speed[ACCELERATING] * down -
        (int64_t)w->delta_speed[BRAKING] * up;
    if (impulse <= 0 || impulse > INT64_MAX / MFOC_ROTATION_BLOCK_DT) {
        return MFOC_FSM_FAILED;
    }
    const int64_t value = impulse * MFOC_ROTATION_BLOCK_DT / delta;
#else
    const float impulse = w->sum_torque[ACCELERATING] * (float)down -
        w->sum_torque[BRAKING] * (float)up;
    const float delta = w->delta_speed[ACCELERATING] * (float)down -
        w->delta_speed[BRAKING] * (float)up;
    const float value = impulse * MFOC_ROTATION_BLOCK_DT / delta;
#endif
    if (value <= MFOC_ZERO || value > MFOC_SCALAR(MFOC_MOTOR_INERTIA_MAX_PU)) {
        return MFOC_FSM_FAILED;
    }
    *result = (mfoc_scalar_t)value;
    return MFOC_FSM_COMPLETE;
}
