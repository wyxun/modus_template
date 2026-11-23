/**
 * @file mfoc_pwm_phase.c
 * @brief Qualify G431 preload transfers with TIM2.
 * @author Codex
 * @date 2026-11-22
 */
#include "mfoc_pwm_phase.h"
#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"

mfoc_g431_pwm_phase_t g_mfoc_g431_pwm_phase;

/* Margin for sequential APB reads and internal trigger synchronization.
 * Much smaller than the 4250-tick distance between the two extremums. */
#define PWM_PHASE_SYNC_TICKS 64U

/* TIM2_CH1 captures the rising TIM1_TRGO pulse through ITR0 and TRC.
 * One complete natural UEV must follow baseline. The poll count also bounds
 * waiting if the capture timer's own clock fails. */
static bool phase_wait(uint32_t baseline, uint32_t *capture)
{
    uint32_t arr = TIM1->ARR;
    uint32_t start = TIM2->CNT;
    for (uint32_t attempt = 0U; attempt < 4U * arr; ++attempt) {
        uint32_t stamp = TIM2->CCR1;
        uint32_t now = TIM2->CNT;
        if (now - start > 4U * arr) { break; }
        if (stamp != baseline) {
            uint32_t count = TIM1->CNT;
            bool down = (TIM1->CR1 & TIM_CR1_DIR) != 0U;
            uint32_t latest = TIM2->CNT - stamp;
            uint32_t earliest = now - stamp;
#if MFOC_G431_PWM_PHASE == 1
            uint32_t travel = count;
            bool direction_ok = !down;
#else
            uint32_t travel = arr - count;
            bool direction_ok = down;
#endif
            if (!direction_ok || latest >= arr / 2U ||
                travel + PWM_PHASE_SYNC_TICKS < earliest ||
                travel > latest + PWM_PHASE_SYNC_TICKS) { break; }
            *capture = stamp;
            return true;
        }
    }
    return false;
}

void mfoc_g431_pwm_StartCounter(void)
{
    uint32_t arr = TIM1->ARR;
    bool valid = false;
    g_mfoc_g431_pwm_phase.locked = false;
    /* The trigger is enabled only after the last software UG, so CCR1
     * cannot confuse the prescaler/repetition load with a natural UEV. */
    TIM1->SR = ~TIM_SR_UIF;
    TIM1->EGR = TIM_EGR_UG;
    for (unsigned n = 0U; n < PWM_PHASE_SYNC_TICKS; ++n) {
        if ((TIM1->SR & TIM_SR_UIF) != 0U) { valid = true; break; }
    }
    TIM1->SR = ~TIM_SR_UIF;
    valid = valid && TIM1->CNT == 0U &&
        (TIM1->CR1 & (TIM_CR1_CEN | TIM_CR1_DIR | TIM_CR1_UDIS)) == 0U;
    if (!valid) { ++g_mfoc_g431_pwm_phase.failures; return; }

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    TIM2->PSC = 0U;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SMCR = 0U;               /* ITR0 = TIM1_TRGO, free-running. */
    TIM2->CCMR1 = TIM_CCMR1_CC1S;   /* 11: capture TRC on rising edge. */
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->CR1 = TIM_CR1_CEN;
    uint32_t previous = TIM2->CCR1;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t run = TIM1->CR1 | TIM_CR1_CEN;
    uint32_t start = TIM2->CNT;
    TIM1->CR1 = run;
#if MFOC_G431_PWM_PHASE == 2
    /* Adjacent volatile writes. All cold UGs loaded REP=0. The first
     * overflow transfers the pending RCR=1 for the following periods. */
    TIM1->RCR = 1U;
#endif
    __DSB();
    uint32_t count = TIM1->CNT;
    valid = (TIM1->CR1 & TIM_CR1_DIR) == 0U && count < arr &&
        (TIM1->SR & TIM_SR_UIF) == 0U && TIM2->CNT - start < arr;
    __set_PRIMASK(primask);
    g_mfoc_g431_pwm_phase.cold_count = count;

    uint32_t *first = &g_mfoc_g431_pwm_phase.first_capture;
    uint32_t *second = &g_mfoc_g431_pwm_phase.second_capture;
    uint32_t expected = MFOC_G431_PWM_PHASE == 1 ? 2U * arr : arr;
    valid = valid && phase_wait(previous, first) &&
        *first - start >= expected - PWM_PHASE_SYNC_TICKS &&
        *first - start <= expected + PWM_PHASE_SYNC_TICKS &&
        phase_wait(*first, second) && *second - *first == 2U * arr;
    g_mfoc_g431_pwm_phase.locked = valid;
    if (!valid) { ++g_mfoc_g431_pwm_phase.failures; }
}

bool mfoc_g431_pwm_WaitTransfer(void)
{
    if (!g_mfoc_g431_pwm_phase.locked) {
        ++g_mfoc_g431_pwm_phase.failures;
        return false;
    }
    /* Complete all preceding CCR writes before sampling the capture.
     * An event between the last write and this read costs only one wait. */
    __DSB();
    uint32_t previous = TIM2->CCR1;
    if (!phase_wait(previous, &g_mfoc_g431_pwm_phase.last_capture)) {
        ++g_mfoc_g431_pwm_phase.failures;
        return false;
    }
    ++g_mfoc_g431_pwm_phase.startup_captures;
    return true;
}
