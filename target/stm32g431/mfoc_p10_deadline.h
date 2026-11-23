/**
 * @file mfoc_p10_deadline.h
 * @brief Same-frame timing relative to TIM2's hardware-latched TIM1 UEV.
 * @author Codex
 * @date 2026-11-21
 */
#ifndef MFOC_P10_DEADLINE_H
#define MFOC_P10_DEADLINE_H

#include <stdbool.h>
#include <stdint.h>

/** The sampling trigger T precedes the apex by ARR-CCR4 counts. U is
 * always the scheduled update for that T, including when W is already late.
 * CCR1 and I use the same 32-bit free-running timer. If another UEV
 * intervenes between I and the CCR1 read, reject and count the frame. */
static inline bool mfoc_p10_frame_times(uint32_t phase, uint32_t arr,
    uint32_t ccr4, uint32_t capture, uint32_t entry,
    uint32_t *trigger, uint32_t *deadline)
{
    const uint32_t period = 2U * arr;
    if ((phase != 1U && phase != 2U) || arr == 0U || arr > 65535U ||
        ccr4 >= arr || entry - capture >= period) {
        return false;
    }
    /* Trigger within the period that starts at the captured UEV. */
    const uint32_t offset = phase == 1U ? ccr4 : arr + ccr4;
    uint32_t t = capture + offset;
    if (entry - capture < offset) { t -= period; }
    *trigger = t;
    *deadline = t + (phase == 1U ? period - ccr4 : period + arr - ccr4);
    return true;
}

/** Remaining ticks relative to the 80% deadline. Keeping the arithmetic
 * relative to T handles TIM2 wrap across J, W and the following frame.
 * J lower bound and both guards must come from the same bench calibration. */
static inline int32_t mfoc_p10_reserve(uint32_t window, uint32_t write_age,
    uint32_t j_lower, uint32_t guard)
{
    if (guard >= window || j_lower >= window - guard ||
        window > 3U * 65535U || write_age > INT32_MAX) {
        return INT32_MIN;
    }
    const uint32_t permitted = j_lower + 4U * (window - guard - j_lower) / 5U;
    return (int32_t)permitted - (int32_t)write_age;
}

#endif
