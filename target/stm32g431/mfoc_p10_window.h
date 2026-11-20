/**
 * @file mfoc_p10_window.h
 * @brief Arithmetic for the G431 live-compare timing probe.
 * @author Codex
 * @date 2026-11-20
 */
#ifndef MFOC_G431_P10_WINDOW_H
#define MFOC_G431_P10_WINDOW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief Measured age, conservative window and signed edge slack in ticks. */
typedef struct {
    uint32_t age_at_entry;
    uint32_t trigger_to_write;
    uint32_t available;
    int32_t slack;
} mfoc_p10_window_t;

/** @brief Bound the last compare write against a descending compare edge.
 * @param arr Center-aligned timer half-period.
 * @param trigger_earliest Count at the earliest possible trigger, counting up.
 * @param count Timer count at IRQ entry.
 * @param down True if the count is descending.
 * @param irq_to_write Core ticks from that entry sample to the last write.
 * @param changed_max_compare Max(old,new) across changed phase compares.
 * @param out Result, unchanged if an input or frame age is invalid.
 * @return True if the result fits in a single control period.
 * @note Timer and core must have the same tick rate. Entry must belong to the
 * assumed frame; the caller is responsible for detecting entire lost frames.
 */
static inline bool mfoc_p10_window(uint32_t arr, uint32_t trigger_earliest,
    uint32_t count, bool down, uint32_t irq_to_write,
    uint32_t changed_max_compare, mfoc_p10_window_t *out)
{
    if (out == NULL || arr == 0U || arr > (uint32_t)INT32_MAX / 2U ||
        trigger_earliest > arr || count > arr || changed_max_compare > arr) {
        return false;
    }
    const uint32_t period = 2U * arr;
    uint32_t phase = down ? period - count : count;
    if (phase < trigger_earliest) { phase += period; }
    const uint32_t age = phase - trigger_earliest;
    if (irq_to_write >= period - age) { return false; }
    const uint32_t elapsed = age + irq_to_write;
    const uint32_t available = period - changed_max_compare - trigger_earliest;
    *out = (mfoc_p10_window_t){.age_at_entry = age,
        .trigger_to_write = elapsed, .available = available,
        .slack = (int32_t)available - (int32_t)elapsed};
    return true;
}

/** @brief Check the conservative 20% reserve, rounded up to a timer tick.
 * @param window A valid result from mfoc_p10_window.
 * @return True for a nonempty window with at least the required slack.
 */
static inline bool mfoc_p10_has_reserve(const mfoc_p10_window_t *window)
{
    if (window == NULL || window->available == 0U || window->slack < 0) {
        return false;
    }
    const uint32_t minimum = (window->available + 4U) / 5U;
    return (uint32_t)window->slack >= minimum;
}

#endif
