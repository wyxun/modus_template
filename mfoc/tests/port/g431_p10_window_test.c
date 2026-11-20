/**
 * @file g431_p10_window_test.c
 * @brief Boundary cases for TIM1 phase and PWM margin reconstruction.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_p10_window.h"
#include <assert.h>
#include <stdio.h>

/** @brief Run direction, frame rollover and 20% reserve checks.
 * @return Zero when all assertions pass.
 */
int main(void)
{
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
