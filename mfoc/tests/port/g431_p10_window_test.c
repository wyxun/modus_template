/**
 * @file g431_p10_window_test.c
 * @brief Boundary cases for TIM1 phase and PWM margin reconstruction.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_p10_window.h"
#include "mfoc_p10_deadline.h"
#include <assert.h>
#include <stdio.h>

/** Recreate three successive frames from real timer geometry, including
 * late IRQ service, APB capture races and the 32-bit TIM2 rollover. */
static void candidate_deadlines(void)
{
    const uint32_t period = 8500U, arr = 4250U, ccr4 = 4240U;
    const uint32_t initial = UINT32_MAX - 4600U;
    for (unsigned phase = 1U; phase <= 2U; ++phase) {
        const uint32_t offset = phase == 1U ? ccr4 : arr + ccr4;
        const uint32_t window = phase == 1U ? 4260U : 8510U;
        for (unsigned k = 0U; k < 3U; ++k) {
            uint32_t u0 = initial + k * period;
            uint32_t t = u0 + offset;
            for (unsigned age = 0U; age < period; ++age) {
                uint32_t i = t + age;
                uint32_t cap = u0 + ((offset + age) / period) * period;
                uint32_t got_t = 0U, got_u = 0U;
                assert(mfoc_p10_frame_times(phase, arr, ccr4, cap, i,
                    &got_t, &got_u));
                assert(got_t == t && got_u == t + window);
            }
        }
    }
    uint32_t t = 0U, u = 0U;
    assert(!mfoc_p10_frame_times(1U, arr, ccr4, 105U, 100U, &t, &u));
    assert(!mfoc_p10_frame_times(2U, arr, ccr4, 10U, 8510U, &t, &u));
    assert(!mfoc_p10_frame_times(0U, arr, ccr4, 0U, 0U, &t, &u));
    assert(mfoc_p10_reserve(4260U, 3408U, 0U, 0U) == 0);
    assert(mfoc_p10_reserve(4260U, 3409U, 0U, 0U) == -1);
    assert(mfoc_p10_reserve(8510U, 6808U, 0U, 0U) == 0);
    assert(mfoc_p10_reserve(4260U, 3520U, 600U, 10U) == 0);
    assert(mfoc_p10_reserve(4260U, 0U, 4260U, 1U) == INT32_MIN);
    assert(mfoc_p10_reserve(4260U, 0U, 0U, 4260U) == INT32_MIN);
}

/** @brief Run direction, frame rollover and 20% reserve checks.
 * @return Zero when all assertions pass.
 */
int main(void)
{
    candidate_deadlines();
    mfoc_p10_window_t result = {0};
    assert(mfoc_p10_window(4250U, 4240U, 4000U, true, 748U, 3000U,
        &result));
    assert(result.age_at_entry == 260U);
    assert(result.trigger_to_write == 1008U && result.available == 1260U);
    assert(result.slack == 252 && mfoc_p10_has_reserve(&result));
    assert(mfoc_p10_window(4250U, 4240U, 4000U, true, 749U, 3000U,
        &result));
    assert(result.slack == 251 && !mfoc_p10_has_reserve(&result));
    assert(mfoc_p10_window(4250U, 4240U, 3640U, true, 2000U, 3000U,
        &result));
    assert(result.slack == -1360 && !mfoc_p10_has_reserve(&result));

    assert(mfoc_p10_window(4250U, 4240U, 4245U, false, 75U, 4159U,
        &result));
    assert(result.age_at_entry == 5U && result.available == 101U);
    assert(result.slack == 21 && mfoc_p10_has_reserve(&result));
    assert(mfoc_p10_window(4250U, 4240U, 4245U, false, 76U, 4159U,
        &result));
    assert(result.slack == 20 && !mfoc_p10_has_reserve(&result));

    assert(mfoc_p10_window(4250U, 4240U, 4220U, false, 19U, 0U, &result));
    assert(result.age_at_entry == 8480U && result.trigger_to_write == 8499U);
    assert(!mfoc_p10_window(4250U, 4240U, 4220U, false, 20U, 0U, &result));
    assert(result.trigger_to_write == 8499U);
    assert(!mfoc_p10_window(4250U, 4240U, 4240U, false, UINT32_MAX, 0U,
        &result));
    assert(!mfoc_p10_window(0U, 0U, 0U, false, 0U, 0U, &result));
    assert(!mfoc_p10_window(UINT32_MAX, 0U, 0U, false, 0U, 0U, &result));
    assert(!mfoc_p10_window(4250U, 4251U, 0U, false, 0U, 0U, &result));
    assert(!mfoc_p10_window(4250U, 4240U, 4251U, false, 0U, 0U, &result));
    assert(!mfoc_p10_window(4250U, 4240U, 4250U, true, 0U, 4251U,
        &result));
    assert(!mfoc_p10_window(4250U, 4240U, 4250U, true, 0U, 0U, NULL));
    assert(mfoc_p10_window(4250U, 4250U, 4250U, true, 0U, 4250U,
        &result));
    assert(result.available == 0U && !mfoc_p10_has_reserve(&result));
    puts("G431 P10 PWM timing window passed");
    return 0;
}
