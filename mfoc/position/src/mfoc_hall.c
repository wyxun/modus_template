/**
 * @file mfoc_hall.c
 * @brief Electrical sector decoding, edge timing and bounded extrapolation.
 */
#include "mfoc_hall.h"
#include "mfoc_position_travel.h"
#include <limits.h>
#include <stddef.h>

enum { HALL_SECTORS = 6, HALL_UNSEEN = 2, HALL_REACQUIRE = 3 };

/** @brief Next sector in the calibrated positive electrical direction. */
static uint8_t _mfoc_hall_Next(uint8_t sector)
{
    return sector == HALL_SECTORS - 1U ? 0U : sector + 1U;
}

/** @brief Validate the calibration once, leaving edge and ISR paths small. */
mfoc_result_t mfoc_hall_Init(mfoc_hall_t *hall, const mfoc_hall_cfg_t *cfg,
    uint32_t tick_hz)
{
    if (hall == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    if (tick_hz == 0U || cfg->timeout_ms == 0U ||
        cfg->electrical_speed_base_hz <= MFOC_ZERO) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_tick_t window = ((mfoc_tick_t)tick_hz * cfg->timeout_ms + 999U) /
        1000U;
    if (window > INT32_MAX) { return MFOC_RESULT_OUT_OF_RANGE; }
    mfoc_hall_t next = {.published = HALL_UNSEEN, .tick_hz = tick_hz,
        .timeout_ticks = (uint32_t)window};
#if defined(MFOC_NUMERIC_FIXED)
    next.speed_base_hz = cfg->electrical_speed_base_hz;
#else
    mfoc_result_t rc = mfoc_div_checked((mfoc_scalar_t)tick_hz,
        cfg->electrical_speed_base_hz, &next.tick_hz_per_base_hz);
    if (rc != MFOC_RESULT_OK) { return rc; }
#endif
    uint32_t codes = 0U;
    uint64_t total = 0U;
    for (uint8_t i = 0U; i < HALL_SECTORS; ++i) {
        mfoc_hall_sector_t sector = cfg->sectors[i];
        mfoc_hall_sector_t after = cfg->sectors[_mfoc_hall_Next(i)];
        uint8_t bits = sector.code ^ after.code;
        uint32_t width = after.start.wBam32 - sector.start.wBam32;
        if (sector.code == 0U || sector.code > 6U ||
            (codes & (1U << sector.code)) != 0U ||
            (bits != 1U && bits != 2U && bits != 4U) ||
            width == 0U || width > INT32_MAX) {
            return MFOC_RESULT_INVALID_ARGUMENT;
        }
        codes |= 1U << sector.code;
        total += width;
        next.sectors[i] = sector;
    }
    if (total != (UINT64_C(1) << 32U)) { return MFOC_RESULT_INVALID_ARGUMENT; }
    *hall = next;
    return MFOC_RESULT_OK;
}

/** @brief Speed over the completed sector; width is signed electrical BAM32. */
static mfoc_result_t _mfoc_hall_Speed(mfoc_hall_sample_t *sample,
    int32_t width, uint32_t interval, const mfoc_hall_t *hall)
{
#if defined(MFOC_NUMERIC_FIXED)
    int64_t rate = (int64_t)width * hall->tick_hz / interval / INT64_C(131072);
    if (rate > INT32_MAX || rate < INT32_MIN) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    sample->speed_hz = (mfoc_scalar_t)rate;
    return mfoc_div_checked(sample->speed_hz, hall->speed_base_hz,
        &sample->speed_pu);
#else
    mfoc_scalar_t turns_per_tick = (mfoc_scalar_t)width /
        (4294967296.0f * (mfoc_scalar_t)interval);
    sample->speed_hz = turns_per_tick * (mfoc_scalar_t)hall->tick_hz;
    sample->speed_pu = turns_per_tick * hall->tick_hz_per_base_hz;
    return MFOC_RESULT_OK;
#endif
}

/** @brief Decode each captured edge before publishing one complete estimate. */
mfoc_result_t mfoc_hall_OnEdge(mfoc_hall_t *hall, uint8_t code,
    mfoc_tick_t tick)
{
    uint8_t published = hall->published;
    if (published != HALL_UNSEEN && tick - hall->last_event_tick > INT64_MAX) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    hall->last_event_tick = tick;
    mfoc_hall_sample_t previous = {0};
    mfoc_tick_t elapsed = 0U;
    if (published < HALL_UNSEEN) {
        previous = hall->samples[published];
        elapsed = tick - previous.tick;
    }
    uint8_t sector = 0U;
    while (sector < HALL_SECTORS && hall->sectors[sector].code != code) {
        ++sector;
    }
    if (sector == HALL_SECTORS) {
        hall->published = HALL_REACQUIRE;
        return MFOC_RESULT_SAFETY;
    }
    if (published < HALL_UNSEEN && sector == previous.sector) {
        return MFOC_RESULT_OK;
    }
    uint8_t after = _mfoc_hall_Next(sector);
    uint32_t start = hall->sectors[sector].start.wBam32;
    uint32_t end = hall->sectors[after].start.wBam32;
    mfoc_hall_sample_t sample = {.tick = tick, .sector = sector};
    if (published >= HALL_UNSEEN || elapsed > hall->timeout_ticks) {
        sample.anchor.wBam32 = start + (end - start) / 2U;
    } else {
        if (sector == _mfoc_hall_Next(previous.sector)) {
            sample.direction = 1;
            sample.anchor.wBam32 = start;
        } else if (after == previous.sector) {
            sample.direction = -1;
            sample.anchor.wBam32 = end;
        }
        if (sample.direction == 0 || elapsed == 0U) {
            hall->published = HALL_REACQUIRE;
            return MFOC_RESULT_SAFETY;
        }
        if (sample.direction == previous.direction) {
            uint32_t old = previous.anchor.wBam32;
            uint32_t now = sample.anchor.wBam32;
            int32_t width = sample.direction > 0 ?
                (int32_t)(now - old) : -(int32_t)(old - now);
            mfoc_result_t rc = _mfoc_hall_Speed(&sample, width,
                (uint32_t)elapsed, hall);
            if (rc != MFOC_RESULT_OK) {
                hall->published = HALL_REACQUIRE;
                return rc;
            }
        }
    }
    uint8_t next = published == 0U ? 1U : 0U;
    hall->samples[next] = sample;
    hall->published = next;
    return MFOC_RESULT_OK;
}

/** @brief Predict from the last edge up to the exit of the observed sector. */
mfoc_result_t mfoc_hall_Step(const mfoc_hall_t *hall, mfoc_tick_t now,
    mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    uint8_t published = hall->published;
    if (published >= HALL_UNSEEN) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_hall_sample_t sample = hall->samples[published];
    mfoc_tick_t age = now - sample.tick;
    if (age > hall->timeout_ticks) { return MFOC_RESULT_SAFETY; }
    uint32_t width = hall->sectors[_mfoc_hall_Next(sample.sector)].start.wBam32
        - hall->sectors[sample.sector].start.wBam32;
    mfoc_scalar_t travel = mfoc_position_travel(mfoc_abs(sample.speed_hz),
        (uint32_t)age, hall->tick_hz);
    /* Clamp before converting a multi-turn displacement to wrapped BAM32. */
    uint32_t distance = travel >= MFOC_HALF ?
        width : mfoc_angle_from_scalar(travel).wBam32;
    if (distance > width) { distance = width; }
    angle->wBam32 = sample.direction < 0 ? sample.anchor.wBam32 - distance :
        sample.anchor.wBam32 + distance;
    *speed = sample.speed_pu;
    return MFOC_RESULT_OK;
}
