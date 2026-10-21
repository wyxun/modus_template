/**
 * @file driver_irq_test.c
 * @brief No-argument IRQ vector alias, frame ordering and cleanup.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_driver_irq_binding.h"
#include "motor_port_fake.h"
#include <assert.h>
#include <stdio.h>

mfoc_driver_t mfoc_driver_instance;
static bool pending = false;
static unsigned finishes = 0U;

/** @brief Claim an IRQ without clearing the current frame.
 * @param token Target token. @return Whether this IRQ belongs to control. */
bool test_irq_begin(uint32_t *token)
{
    if (!pending) { return false; }
    *token = 123U;
    return true;
}

/** @brief Model cleanup, including an idle or failed control step.
 * @param token Matching entry token. @return None. */
void test_irq_end(uint32_t token)
{
    assert(token == 123U);
    finishes++;
    pending = false;
}

/** @brief Exercise the actual aliased, no-argument vector entry.
 * @return Zero when aliasing, scheduling and cleanup hold. */
int main(void)
{
    mfoc_driver_irq_handler_t *volatile vector = mfoc_test_ADC_IRQHandler;
    assert(vector == mfoc_driver_IRQHandler);
    mfoc_driver_IRQHandler();
    assert(finishes == 0U);
    mfoc_pi_params_t pi = {MFOC_HALF, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_driver_cfg_t cfg = {
        .motor = {
            .params = {7U, MFOC_ONE, MFOC_ONE, MFOC_ONE, MFOC_ONE, MFOC_ZERO},
            .limits = {MFOC_ONE},
            .core = {pi, pi, MFOC_SCALAR(0.55f), MFOC_ONE},
        },
    };
    assert(mfoc_driver_Init(&mfoc_driver_instance, &cfg) == MFOC_RESULT_OK);
    pending = true;
    mfoc_test_ADC_IRQHandler();
    assert(finishes == 1U && !pending);
    assert(motor_port_fake.samples == 0U);
    assert(mfoc_driver_instance.irq_result == MFOC_RESULT_INVALID_STATE);
    mfoc_dq_t ref = {MFOC_SCALAR(0.1f), MFOC_ZERO};
    assert(mfoc_driver_SetCurrent(&mfoc_driver_instance, &ref) ==
           MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&mfoc_driver_instance.motor) == MFOC_RESULT_OK);
    pending = true;
    mfoc_test_ADC_IRQHandler();
    assert(mfoc_driver_instance.irq_result == MFOC_RESULT_OK);
    assert(finishes == 2U && !pending);
    assert(motor_port_fake.samples == 1U && motor_port_fake.writes == 2U);
    assert(motor_port_fake.feedback_reads == 1U);
    motor_port_fake.sample = MFOC_RESULT_IO;
    pending = true;
    mfoc_test_ADC_IRQHandler();
    assert(mfoc_driver_instance.irq_result == MFOC_RESULT_IO);
    assert(finishes == 3U && !pending);
    assert(!motor_port_fake.enabled && motor_port_fake.writes == 2U);
    pending = true;
    mfoc_test_ADC_IRQHandler();
    assert(mfoc_driver_instance.irq_result == MFOC_RESULT_INVALID_STATE);
    assert(finishes == 4U && !pending);
    assert(motor_port_fake.samples == 2U);
    puts("Driver vector alias, frame order and IRQ cleanup passed");
    return 0;
}
