/**
 * @file pwm_runtime.c
 * @brief G431 PWM gate and write-zero-to-clear command contracts.
 * @author Codex
 * @date 2026-10-10
 * @note SR is a write-command capture, not an emulated hardware status
 * register. Tests explicitly apply rc_w0 semantics to the captured command.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t EGR;
    volatile uint32_t BDTR;
    volatile uint32_t CCER;
    volatile uint32_t SR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
} test_pwm_timer_t;

typedef struct { volatile uint32_t CSR; } test_comp_t;
static test_pwm_timer_t s_tTimer;
static test_comp_t s_tComp1;
static test_comp_t s_tComp2;
static test_comp_t s_tComp4;
static uint32_t s_wPrimask;
static uint32_t s_wSourceAfterReads;
static uint32_t s_wBreakAfterReads;
#define TIM1 (&s_tTimer)
#define COMP1 (&s_tComp1)
#define COMP2 (&s_tComp2)
#define COMP4 test_comp4()
#define TIM_CR1_CEN UINT32_C(1)
#define TIM_BDTR_MOE UINT32_C(0x8000)
#define TIM_CCER_CC1E UINT32_C(1)
#define TIM_CCER_CC1NE UINT32_C(4)
#define TIM_CCER_CC2E UINT32_C(0x10)
#define TIM_CCER_CC2NE UINT32_C(0x40)
#define TIM_CCER_CC3E UINT32_C(0x100)
#define TIM_CCER_CC3NE UINT32_C(0x400)
#define TIM_CCER_CC4E UINT32_C(0x1000)
#define TIM_SR_BIF UINT32_C(0x80)
#define COMP_CSR_VALUE UINT32_C(0x40000000)

static inline uint32_t __get_PRIMASK(void) { return s_wPrimask; }
static inline void __disable_irq(void) { s_wPrimask = 1U; }
static inline void __set_PRIMASK(uint32_t wMask) { s_wPrimask = wMask; }

/** @brief Inject a fault between provider reads without faking SR clearing. */
static inline test_comp_t *test_comp4(void)
{
    if (s_wSourceAfterReads != 0U) {
        --s_wSourceAfterReads;
        if (s_wSourceAfterReads == 0U) { s_tComp4.CSR = COMP_CSR_VALUE; }
    }
    if (s_wBreakAfterReads != 0U) {
        --s_wBreakAfterReads;
        if (s_wBreakAfterReads == 0U) { TIM1->SR |= TIM_SR_BIF; }
    }
    return &s_tComp4;
}

#include "mdi/pwm.h"

volatile bool g_bG431FaultLatched;
#define TEST_PHASE_MASK UINT32_C(0x555)
#define TEST_CHANNELS(X)                                                       \
    X(u, TIM1->CCR1, TIM1->ARR)                                                 \
    X(v, TIM1->CCR2, TIM1->ARR)                                                 \
    X(w, TIM1->CCR3, TIM1->ARR)
MDI_PWM_REG_BIND(bridge, TEST_CHANNELS)
MDI_STM32_PWM_TIMING_BIND(bridge, TIM1, 170000000U, 2U, TEST_CHANNELS)
MDI_G431_PWM_LIFECYCLE_BIND(bridge)

#if defined(TEST_UNSUPPORTED_COMMIT)
void test_unsupported_commit(void)
{
    assert(MDI_PWM_Commit(bridge) == MDI_OK);
}
#endif

/** @brief Exercise phase stop without disturbing ongoing sampling or faults. */
static void test_safe_stop(void)
{
    TIM1->CR1 = 0x61U;
    TIM1->CCER = TEST_PHASE_MASK | TIM_CCER_CC4E | 0x222U;
    TIM1->BDTR = TIM_BDTR_MOE | 0x40U;
    TIM1->SR = TIM_SR_BIF | 1U;
    g_bG431FaultLatched = true;
    assert(MDI_PWM_SafeStop(bridge) == MDI_OK);
    assert(TIM1->CR1 == 0x61U);
    assert(TIM1->CCER == (TIM_CCER_CC4E | 0x222U));
    assert(TIM1->BDTR == (TIM_BDTR_MOE | 0x40U));
    assert(TIM1->SR == (TIM_SR_BIF | 1U));
    assert(g_bG431FaultLatched);
    TIM1->BDTR = 0x40U;
    TIM1->CR1 = 0x60U;
    TIM1->CCER = TEST_PHASE_MASK;
    assert(MDI_PWM_Enable(bridge, false) == MDI_OK);
    assert(TIM1->CCER == 0U);
    assert(TIM1->CR1 == 0x60U);
    assert(TIM1->BDTR == 0x40U);
}

/** @brief Verify complete enable, fault guards and no duty update event. */
static void test_enable_and_duty(void)
{
    MDI_PWM_DutyFrame(bridge) tDuty = {32768U, 0U, 65536U};
    TIM1->ARR = 4250U;
    TIM1->EGR = 0x1234U;
    assert(MDI_PWM_SetDuty(bridge, &tDuty) == MDI_OK);
    assert(TIM1->CCR1 == 2125U && TIM1->CCR2 == 0U);
    assert(TIM1->CCR3 == 4250U && TIM1->EGR == 0x1234U);
    assert(MDI_PWM_Enable(bridge, true) == MDI_BUSY);
    g_bG431FaultLatched = false;
    assert(MDI_PWM_Enable(bridge, true) == MDI_BUSY);
    TIM1->SR = 0U;
    COMP1->CSR = COMP_CSR_VALUE;
    assert(MDI_PWM_Enable(bridge, true) == MDI_BUSY);
    COMP1->CSR = 0U;
    COMP2->CSR = COMP_CSR_VALUE;
    assert(MDI_PWM_Enable(bridge, true) == MDI_BUSY);
    COMP2->CSR = 0U;
    COMP4->CSR = COMP_CSR_VALUE;
    assert(MDI_PWM_Enable(bridge, true) == MDI_BUSY);
    COMP4->CSR = 0U;
    assert(MDI_PWM_Enable(bridge, true) == MDI_OK);
    assert((TIM1->CCER & (TEST_PHASE_MASK | TIM_CCER_CC4E)) ==
           (TEST_PHASE_MASK | TIM_CCER_CC4E));
    assert((TIM1->CR1 & TIM_CR1_CEN) != 0U);
    assert((TIM1->BDTR & TIM_BDTR_MOE) != 0U);
    assert(MDI_PWM_IsEnabled(bridge));
    assert(MDI_PWM_AnyOutputEnabled(bridge));
    TIM1->CCER &= ~TIM_CCER_CC3NE;
    assert(!MDI_PWM_IsEnabled(bridge));
    assert(MDI_PWM_AnyOutputEnabled(bridge));
}

/** @brief Gate state remains open with a stopped counter or active Break. */
static void test_any_output_enabled(void)
{
    TIM1->CR1 = 0U;
    TIM1->BDTR = 0U;
    TIM1->CCER = TIM_CCER_CC1E;
    assert(MDI_PWM_AnyOutputEnabled(bridge));
    TIM1->CCER = TIM_CCER_CC4E;
    assert(!MDI_PWM_AnyOutputEnabled(bridge));
}

/** @brief Acknowledge only BIF and never reopen the phase outputs. */
static void test_clear_fault(void)
{
    const uint32_t wUnrelated = 0x1001U;
    uint32_t wCommand = 0U;
    TIM1->SR = TIM_SR_BIF | wUnrelated;
    g_bG431FaultLatched = true;
    assert(MDI_PWM_ClearFault(bridge) == MDI_BUSY);
    assert(TIM1->SR == (TIM_SR_BIF | wUnrelated));
    assert(MDI_PWM_SafeStop(bridge) == MDI_OK);
    COMP2->CSR = COMP_CSR_VALUE;
    assert(MDI_PWM_ClearFault(bridge) == MDI_BUSY);
    assert(TIM1->SR == (TIM_SR_BIF | wUnrelated));
    COMP2->CSR = 0U;
    TIM1->BDTR = 0x40U;
    s_wPrimask = 1U;
    assert(MDI_PWM_ClearFault(bridge) == MDI_OK);
    assert(s_wPrimask == 1U);
    assert(!g_bG431FaultLatched);
    wCommand = TIM1->SR;
    assert(wCommand == ~TIM_SR_BIF);
    /* rc_w0: writing one preserves every other flag, including a flag
     * arriving after the earlier status read but before the write. */
    assert(((TIM_SR_BIF | wUnrelated | 0x200U) & wCommand) ==
           (wUnrelated | 0x200U));
    assert((TIM1->CCER & TEST_PHASE_MASK) == 0U);
    assert(TIM1->BDTR == 0x40U);
    TIM1->SR = wUnrelated;
    assert(!MDI_PWM_FaultActive(bridge));
    mdi_g431_fault_notify_break();
    assert(MDI_PWM_FaultActive(bridge));
}

/** @brief Preserve faults that arrive during clear or gate rearming. */
static void test_fault_races(void)
{
    s_wPrimask = 0U;
    s_wSourceAfterReads = 2U;
    assert(MDI_PWM_ClearFault(bridge) == MDI_IO_ERROR);
    assert(s_wPrimask == 0U && g_bG431FaultLatched);
    s_tComp4.CSR = 0U;
    s_wBreakAfterReads = 2U;
    assert(MDI_PWM_ClearFault(bridge) == MDI_IO_ERROR);
    assert(g_bG431FaultLatched);
    TIM1->SR = 0U;
    g_bG431FaultLatched = false;
    s_wSourceAfterReads = 2U;
    assert(MDI_PWM_Enable(bridge, true) == MDI_BUSY);
    assert((TIM1->CCER & TEST_PHASE_MASK) == 0U);
    assert(g_bG431FaultLatched);
    TIM1->CCER |= TEST_PHASE_MASK;
    assert(MDI_PWM_Enable(bridge, true) == MDI_BUSY);
    assert((TIM1->CCER & TEST_PHASE_MASK) == 0U);
}

/** @brief Run isolated G431 register contracts. */
int main(void)
{
    test_safe_stop();
    test_enable_and_duty();
    test_clear_fault();
    test_fault_races();
    test_any_output_enabled();
    puts("PASS G431 PWM duty, phase gates and rc_w0 command contracts");
    return 0;
}
