/**
 * @file hall_test.c
 * @brief Hall edge timing, sector geometry and position publication.
 * @author wyxun
 * @date 2026-11-11
 */
#include "mfoc_hall.h"
#include "mfoc_feedback.h"
#include "mfoc_port.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const mfoc_hall_cfg_t cfg = {
    .sectors = {
        {1U, {0xe0000000U}}, {5U, {0x10000000U}}, {4U, {0x30000000U}},
        {6U, {0x70000000U}}, {2U, {0x90000000U}}, {3U, {0xb0000000U}}
    },
    .electrical_speed_base_hz = MFOC_SCALAR(128.0f),
    .timeout_ms = 40U,
};

enum { TICK_HZ = 1024000, TIMEOUT_TICKS = 40960 };

static mfoc_hall_t board_hall;
static mfoc_tick_t board_tick;

/** @brief Read the test board's Hall instance on its capture clock.
 * @param angle Electrical BAM32. @param speed Electrical speed PU.
 * @return Hall sample status. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    return mfoc_hall_Step(&board_hall, board_tick, angle, speed);
}

/** @brief Route the board-owned Hall estimate through real Feedback.
 * @return None. */
static void feedback_position(void)
{
    mfoc_feedback_t feedback = {0};
    mfoc_feedback_cfg_t settings = {0};
    mfoc_motor_params_t model = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE,
        MFOC_HALF, MFOC_ZERO};
    mfoc_feedback_output_t out = {0};
    mfoc_hall_cfg_t board_cfg = cfg;
    board_cfg.electrical_speed_base_hz = MFOC_SCALAR(400.0f);
    assert(mfoc_hall_Init(&board_hall, &board_cfg, TICK_HZ) == MFOC_RESULT_OK);
    assert(mfoc_feedback_Init(&feedback, &settings, &model) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&board_hall, 1U, 0U) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&board_hall, 5U, 1000U) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&board_hall, 4U, 3000U) == MFOC_RESULT_OK);
    board_tick = 4000U;
    assert(mfoc_feedback_Step(&feedback, NULL, &out) == MFOC_RESULT_OK);
    assert(out.angle.wBam32 == 0x40000000U);
    /* 64 Hz / 400 Hz, accurate to one Q15 count. */
    assert(fabsf(mfoc_to_float(out.speed) - 0.16f) <= 1.0f / 32768.0f);
    mfoc_scalar_t speed = out.speed;
    assert(out.u_inject_ab.qAlpha == MFOC_ZERO && out.u_inject_ab.qBeta == 0);
    out.u_inject_ab = (mfoc_ab_t){MFOC_HALF, -MFOC_HALF};
    board_tick += TIMEOUT_TICKS;
    assert(mfoc_feedback_Step(&feedback, NULL, &out) == MFOC_RESULT_SAFETY);
    assert(out.angle.wBam32 == 0x40000000U && out.speed == speed);
    assert(out.u_inject_ab.qAlpha == MFOC_HALF);
    assert(out.u_inject_ab.qBeta == -MFOC_HALF);
}

/** @brief Compare a published angle and speed to the electrical test trace.
 * @param hall Source. @param now Capture time. @param bam Electrical BAM32.
 * @param omega Expected electrical speed PU. @return None. */
static void expect_position(const mfoc_hall_t *hall, mfoc_tick_t now,
                            uint32_t bam, float omega)
{
    mfoc_angle_t angle = {0U};
    mfoc_scalar_t speed = MFOC_ZERO;
    assert(mfoc_hall_Step(hall, now, &angle, &speed) == MFOC_RESULT_OK);
    /* Allow 1/65536 turn for interpolation in either numerical backend. */
    uint32_t delta = angle.wBam32 - bam;
    assert(delta <= 0x10000U || 0U - delta <= 0x10000U);
    assert(fabsf(mfoc_to_float(speed) - omega) < 0.0001f);
}

/** @brief Check a failed read leaves both caller-owned values untouched.
 * @param hall Source. @param now Capture time. @param expected Status.
 * @return None. */
static void expect_failure(const mfoc_hall_t *hall, mfoc_tick_t now,
                           mfoc_result_t expected)
{
    mfoc_angle_t angle = {0x12345678U};
    mfoc_scalar_t speed = -MFOC_HALF;
    assert(mfoc_hall_Step(hall, now, &angle, &speed) == expected);
    assert(angle.wBam32 == 0x12345678U && speed == -MFOC_HALF);
}

/** @brief Reconstruct a turn across both angular zero and tick rollover.
 * @return None. */
static void rotation(void)
{
    mfoc_hall_t hall = {0};
    mfoc_tick_t now = UINT64_MAX - 1000U;
    assert(mfoc_hall_Init(&hall, &cfg, TICK_HZ) == MFOC_RESULT_OK);
    expect_failure(&hall, now, MFOC_RESULT_INVALID_STATE);
    assert(mfoc_hall_OnEdge(&hall, 3U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0xc8000000U, 0.0f);
    now += 500U;
    assert(mfoc_hall_OnEdge(&hall, 1U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now + 100U, 0xe0000000U, 0.0f);

    /* Sector 1 covers 3/16 turn in 3000 ticks: 64 Hz at 1.024 MHz. */
    assert(mfoc_hall_OnEdge(&hall, 1U, now + 2500U) == MFOC_RESULT_OK);
    now += 3000U;
    assert(mfoc_hall_OnEdge(&hall, 5U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0x10000000U, 0.5f);
    expect_position(&hall, now + 1000U, 0x20000000U, 0.5f);
    expect_position(&hall, now + 2500U, 0x30000000U, 0.5f);

    /* Actual reversal anchors at the 5/1 boundary, even after clamping. */
    now += 3000U;
    assert(mfoc_hall_OnEdge(&hall, 1U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now + 500U, 0x10000000U, 0.0f);
    now += 3000U;
    assert(mfoc_hall_OnEdge(&hall, 3U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0xe0000000U, -0.5f);
    expect_position(&hall, now + 1000U, 0xd0000000U, -0.5f);
    expect_position(&hall, now + 4000U, 0xb0000000U, -0.5f);

    now += 6000U;
    assert(mfoc_hall_OnEdge(&hall, 2U, now) == MFOC_RESULT_OK);
    now += 1000U;
    assert(mfoc_hall_OnEdge(&hall, 3U, now) == MFOC_RESULT_OK);
    now += 3000U;
    assert(mfoc_hall_OnEdge(&hall, 1U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now + 2500U, 0x08000000U, 0.5f);
    /* Two turns of predicted travel are still bounded by the next edge. */
    expect_position(&hall, now + 32000U, 0x10000000U, 0.5f);
}

/** @brief Freshness uses real edge time, including full low-word cycles.
 * @return None. */
static void stale_and_reacquisition(void)
{
    mfoc_hall_t hall = {0};
    mfoc_tick_t now = UINT64_C(0x1fffffff0);
    assert(mfoc_hall_Init(&hall, &cfg, TICK_HZ) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&hall, 1U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0xf8000000U, 0.0f);
    now += 1000U;
    assert(mfoc_hall_OnEdge(&hall, 5U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0x10000000U, 0.0f);
    assert(mfoc_hall_OnEdge(&hall, 5U, now + TIMEOUT_TICKS - 1U) ==
        MFOC_RESULT_OK);
    expect_position(&hall, now + TIMEOUT_TICKS, 0x10000000U, 0.0f);
    expect_failure(&hall, now + TIMEOUT_TICKS + 1U, MFOC_RESULT_SAFETY);
    now += UINT64_C(0x100000000) + 500U;
    expect_failure(&hall, now, MFOC_RESULT_SAFETY);
    assert(mfoc_hall_OnEdge(&hall, 5U, now) == MFOC_RESULT_OK);
    expect_failure(&hall, now, MFOC_RESULT_SAFETY);
    assert(mfoc_hall_OnEdge(&hall, 4U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0x50000000U, 0.0f);
    now += 1000U;
    assert(mfoc_hall_OnEdge(&hall, 6U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0x70000000U, 0.0f);
    now += 2000U;
    assert(mfoc_hall_OnEdge(&hall, 2U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0x90000000U, 0.5f);
}

/** @brief A delayed capture cannot invalidate or rewind a newer position.
 * @return None. */
static void delayed_event(void)
{
    mfoc_hall_t hall = {0};
    mfoc_tick_t now = UINT64_C(0x100000050);
    assert(mfoc_hall_Init(&hall, &cfg, TICK_HZ) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&hall, 1U, now - 2500U) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&hall, 5U, now - 2000U) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&hall, 4U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now + 1000U, 0x40000000U, 0.5f);
    assert(mfoc_hall_OnEdge(&hall, 5U, now - 2000U) ==
        MFOC_RESULT_INVALID_ARGUMENT);
    expect_position(&hall, now + 1000U, 0x40000000U, 0.5f);
    now += 4000U;
    assert(mfoc_hall_OnEdge(&hall, 6U, now) == MFOC_RESULT_OK);
    expect_position(&hall, now, 0x70000000U, 0.5f);
}

/** @brief Failed transitions require reacquisition, preserving read outputs.
 * @return None. */
static void invalid_edges(void)
{
    const uint8_t bad_code[] = {0U, 7U, 8U, 4U, 5U};
    const mfoc_tick_t elapsed[] = {1U, 1U, 1U, 500U, 0U};
    for (unsigned n = 0U; n < sizeof(bad_code) / sizeof(bad_code[0]); ++n) {
        mfoc_hall_t hall = {0};
        assert(mfoc_hall_Init(&hall, &cfg, TICK_HZ) == MFOC_RESULT_OK);
        assert(mfoc_hall_OnEdge(&hall, 1U, 0U) == MFOC_RESULT_OK);
        assert(mfoc_hall_OnEdge(&hall, bad_code[n], elapsed[n]) ==
            MFOC_RESULT_SAFETY);
        expect_failure(&hall, elapsed[n], MFOC_RESULT_INVALID_STATE);
        assert(mfoc_hall_OnEdge(&hall, 2U, elapsed[n] + 1U) ==
            MFOC_RESULT_OK);
        expect_position(&hall, elapsed[n] + 1U, 0xa0000000U, 0.0f);
    }
}

/** @brief A failed capture establishes the earliest time for reacquisition.
 * @return None. */
static void delayed_after_fault(void)
{
    mfoc_hall_t hall = {0};
    assert(mfoc_hall_Init(&hall, &cfg, TICK_HZ) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&hall, 1U, 100U) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&hall, 7U, 200U) == MFOC_RESULT_SAFETY);
    assert(mfoc_hall_OnEdge(&hall, 5U, 150U) == MFOC_RESULT_INVALID_ARGUMENT);
    expect_failure(&hall, 200U, MFOC_RESULT_INVALID_STATE);
    assert(mfoc_hall_OnEdge(&hall, 5U, 201U) == MFOC_RESULT_OK);
    expect_position(&hall, 201U, 0x20000000U, 0.0f);
}

/** @brief Reject invalid configuration without replacing a live estimate.
 * @return None. */
static void configuration(void)
{
    mfoc_hall_t hall = {0};
    assert(mfoc_hall_Init(&hall, &cfg, TICK_HZ) == MFOC_RESULT_OK);
    assert(mfoc_hall_OnEdge(&hall, 5U, 100U) == MFOC_RESULT_OK);
    for (unsigned n = 0U; n < 9U; ++n) {
        mfoc_hall_cfg_t bad = cfg;
        uint32_t hz = TICK_HZ;
        switch (n) {
        case 0U: bad.sectors[0].code = bad.sectors[1].code; break;
        case 1U: bad.sectors[0].code = 7U; break;
        case 2U: bad.sectors[1].start = bad.sectors[0].start; break;
        case 3U:
            /* Individually valid widths, but two turns around the table. */
            for (unsigned i = 0U; i < 6U; ++i) {
                bad.sectors[i].start.wBam32 = i * UINT32_C(0x50000000);
            }
            break;
        case 4U: bad.electrical_speed_base_hz = MFOC_ZERO; break;
        case 5U: bad.timeout_ms = 0U; break;
        case 6U:
            bad.sectors[1].code = 4U;
            bad.sectors[2].code = 5U;
            break;
        case 7U:
            /* Exactly one full turn, but the first sector spans half a turn. */
            bad.sectors[0].start.wBam32 = 0x10000000U;
            bad.sectors[1].start.wBam32 = 0x90000000U;
            bad.sectors[2].start.wBam32 = 0xa0000000U;
            bad.sectors[3].start.wBam32 = 0xb0000000U;
            bad.sectors[4].start.wBam32 = 0xc0000000U;
            bad.sectors[5].start.wBam32 = 0xe0000000U;
            break;
        default: hz = 0U; break;
        }
        assert(mfoc_hall_Init(&hall, &bad, hz) != MFOC_RESULT_OK);
        expect_position(&hall, 100U, 0x20000000U, 0.0f);
    }
#if defined(MFOC_NUMERIC_FLOAT)
    const float invalid_base[] = {NAN, INFINITY};
    for (unsigned n = 0U; n < 2U; ++n) {
        mfoc_hall_cfg_t bad = cfg;
        bad.electrical_speed_base_hz = invalid_base[n];
        assert(mfoc_hall_Init(&hall, &bad, TICK_HZ) != MFOC_RESULT_OK);
        expect_position(&hall, 100U, 0x20000000U, 0.0f);
    }
#endif
}

/** @brief Run Hall geometry, timing and failure checks. @return Zero. */
int main(void)
{
    rotation();
    stale_and_reacquisition();
    delayed_event();
    invalid_edges();
    delayed_after_fault();
    configuration();
    feedback_position();
    puts("Hall sector timing, direction, rollover and reacquisition passed");
    return 0;
}
