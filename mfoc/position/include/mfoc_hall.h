/**
 * @file mfoc_hall.h
 * @brief Three digital Hall inputs to electrical angle and speed.
 */
#ifndef MFOC_HALL_H
#define MFOC_HALL_H

#include "mfoc_angle.h"

/** @brief Raw Hall code and its positive-direction entry angle (BAM32). */
typedef struct {
    uint8_t code;
    mfoc_angle_t start;
} mfoc_hall_sector_t;

/** @brief Six sectors in increasing electrical angle, one full turn.
 * Codes 1..6 occur once each; adjacent codes differ by one bit.
 * Each sector spans less than half a turn. Board calibration owns phase zero.
 */
typedef struct {
    mfoc_hall_sector_t sectors[6];
    mfoc_scalar_t electrical_speed_base_hz;
    uint32_t timeout_ms;
} mfoc_hall_cfg_t;

/** @brief One published sector, electrical anchor and measured velocity. */
typedef struct {
    mfoc_tick_t tick;
    mfoc_angle_t anchor;
    mfoc_scalar_t speed_hz;  /**< Signed electrical turns per second. */
    mfoc_scalar_t speed_pu;
    uint8_t sector;
    int8_t direction;        /**< 0 at initial acquisition, otherwise +/-1. */
} mfoc_hall_sample_t;

/** @brief Board-owned instance, one ordered writer and one ISR reader.
 * The writer fills an inactive slot then publishes its index. Serialize
 * calls or let Step preempt the writer on one core; the writer must not
 * preempt Step. Supply capture timestamps on the same clock as Step.
 */
typedef struct {
    mfoc_hall_sector_t sectors[6];
    volatile mfoc_hall_sample_t samples[2];
    volatile uint8_t published;
    mfoc_tick_t last_event_tick; /**< Writer clock, including invalid edges. */
    uint32_t tick_hz;
    uint32_t timeout_ticks;
#if defined(MFOC_NUMERIC_FIXED)
    mfoc_scalar_t speed_base_hz;
#else
    mfoc_scalar_t tick_hz_per_base_hz;
#endif
} mfoc_hall_t;

/** @brief Validate and copy board calibration before enabling the reader.
 * @param hall Instance. @param cfg Sector table and positive finite base.
 * @param tick_hz Ticks per second. @return Status; failure preserves hall. */
mfoc_result_t mfoc_hall_Init(mfoc_hall_t *hall, const mfoc_hall_cfg_t *cfg,
    uint32_t tick_hz);

/** @brief Supply the initial code once, then each captured transition.
 * @param hall Initialized instance. @param code Raw three-bit Hall level.
 * @param tick Capture timestamp. @return OK, SAFETY, INVALID_ARGUMENT for
 * an older capture, or OUT_OF_RANGE for an unrepresentable speed.
 * An invalid transition withdraws the sample. The next valid code reseeds
 * the sector midpoint. Duplicate codes do not refresh the timestamp. */
mfoc_result_t mfoc_hall_OnEdge(mfoc_hall_t *hall, uint8_t code,
    mfoc_tick_t tick);

/** @brief Extrapolate within the measured sector, preserving outputs on error.
 * @param hall Initialized instance. @param now Current capture-clock tick.
 * @param angle Electrical BAM32. @param speed Measured electrical speed PU.
 * @return OK, INVALID_STATE before acquisition, or SAFETY after timeout.
 * Valid pointers are required. The first level yields its midpoint at zero
 * speed. Two successive edges in one direction establish measured speed. */
mfoc_result_t mfoc_hall_Step(const mfoc_hall_t *hall, mfoc_tick_t now,
    mfoc_angle_t *angle, mfoc_scalar_t *speed);

#endif
