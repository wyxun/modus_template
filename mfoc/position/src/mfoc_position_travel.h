/**
 * @file mfoc_position_travel.h
 * @brief Angular travel over a measured tick interval.
 * @author wyxun
 * @date 2026-11-11
 */
#ifndef MFOC_POSITION_TRAVEL_H
#define MFOC_POSITION_TRAVEL_H

#include "mfoc_types.h"

/** @brief Extrapolate travel at a constant angular speed.
 * @param speed_turns_per_s Signed mechanical or electrical turns per second.
 * @param elapsed_ticks Interval bounded by the caller before narrowing.
 * @param tick_hz Nonzero clock frequency in ticks per second.
 * @return Turns in the same angle domain, truncated toward zero and
 * saturated to the scalar range in FIXED. */
static inline mfoc_scalar_t mfoc_position_travel(mfoc_scalar_t speed_turns_per_s,
    uint32_t elapsed_ticks, uint32_t tick_hz)
{
#if defined(MFOC_NUMERIC_FIXED)
    int64_t turns = (int64_t)speed_turns_per_s * (int64_t)elapsed_ticks /
        (int64_t)tick_hz;
    if (turns > INT32_MAX) { turns = INT32_MAX; }
    else if (turns < INT32_MIN) { turns = INT32_MIN; }
    else { /* Representable Q15 turns. */ }
    return (mfoc_scalar_t)turns;
#else
    return speed_turns_per_s * (mfoc_scalar_t)elapsed_ticks /
        (mfoc_scalar_t)tick_hz;
#endif
}

#endif
