/**
 * @file mdi_pwm.h
 * @brief Optional 16-bit STM32 timer frequency and Q16 duty capabilities.
 * @author Codex
 * @date 2026-09-18
 * @note This is a register backend, not timer initialization. Requires PWM1,
 * configured polarity and serialized group ownership. Stage/StageFast and
 * SetDuty write CCR directly: preload configuration decides when they apply.
 * SetFrequency requires stopped counter and externally gated outputs. It
 * clears all bound compares and issues UG; duty must then be resubmitted.
 * RCR/trigger side effects of UG belong to board integration, not this API.
 * Only the optional CYCLE binding exposes StageCycle/Commit and requires
 * ARR/CCR preload. G431 bridge preload is disabled and has no Commit.
 */
#ifndef STM32G431_MDI_PWM_H
#define STM32G431_MDI_PWM_H
#include "mdi/core/bind.h"

extern volatile bool g_bG431FaultLatched;

/** @brief Return the live COMP fault source; it must be clear before reset. */
MDI_INLINE bool mdi_g431_fault_source_active(void)
{
    return ((COMP1->CSR | COMP2->CSR | COMP4->CSR) & COMP_CSR_VALUE) != 0U;
}

/** @brief Return the software or timer break fault state. */
MDI_INLINE bool mdi_g431_fault_active(void)
{
    return g_bG431FaultLatched || ((TIM1->SR & TIM_SR_BIF) != 0U);
}

#define MDI_G431_PWM_PHASE_MASK                                                 \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |          \
     TIM_CCER_CC3E | TIM_CCER_CC3NE)

/** @brief Acknowledge a break only with closed phases and no physical fault.
 * @return BUSY for unsafe reset; IO_ERROR if the source or break reasserts.
 * @note IRQ masking serializes the software latch with the break ISR. The
 * hardware break remains effective; reset never changes CCER, CEN or MOE.
 */
MDI_INLINE mdi_status_t mdi_g431_fault_clear(void)
{
    const uint32_t wPrimask = __get_PRIMASK();
    mdi_status_t eStatus = MDI_BUSY;
    __disable_irq();
    if ((TIM1->CCER & MDI_G431_PWM_PHASE_MASK) == 0U &&
        !mdi_g431_fault_source_active()) {
        /* SR is rc_w0: never use a read-modify-write to acknowledge BIF. */
        TIM1->SR = ~TIM_SR_BIF;
        g_bG431FaultLatched = false;
        if (mdi_g431_fault_source_active() ||
            ((TIM1->SR & TIM_SR_BIF) != 0U)) {
            g_bG431FaultLatched = true;
            eStatus = MDI_IO_ERROR;
        } else {
            eStatus = MDI_OK;
        }
    }
    __set_PRIMASK(wPrimask);
    return eStatus;
}

/** @brief Latch a break event from the timer interrupt. */
MDI_INLINE void mdi_g431_fault_notify_break(void)
{
    g_bG431FaultLatched = true;
}

/** @brief Bind the board's TIM1 bridge and its CH4 ADC trigger gate.
 * @param NAME Existing three-phase TIM1 PWM resource token.
 * @return Generates complete lifecycle and fault capabilities.
 * @note Stop closes only CH1-3 E/NE. Existing CH4/CEN/MOE and break state
 * survive. Enable checks both physical and latched faults before opening
 * the phase gate, then checks again for a break during the register writes.
 */
#define MDI_G431_PWM_LIFECYCLE_BIND(NAME)                                       \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SafeStop)(void)                   \
    {                                                                           \
        TIM1->CCER &= ~MDI_G431_PWM_PHASE_MASK;                                 \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_Enable)(bool bEnable)             \
    {                                                                           \
        if (!bEnable) { return MDI_OP(NAME, _pwm_SafeStop)(); }                 \
        if (mdi_g431_fault_active() || mdi_g431_fault_source_active()) {        \
            TIM1->CCER &= ~MDI_G431_PWM_PHASE_MASK;                             \
            return MDI_BUSY;                                                    \
        }                                                                       \
        if ((TIM1->CR1 & TIM_CR1_CEN) != 0U &&                                  \
            (TIM1->BDTR & TIM_BDTR_MOE) != 0U &&                                \
            (TIM1->CCER & MDI_G431_PWM_PHASE_MASK) ==                           \
                MDI_G431_PWM_PHASE_MASK) {                                      \
            return MDI_OK;                                                      \
        }                                                                       \
        TIM1->CCER &= ~MDI_G431_PWM_PHASE_MASK;                                 \
        TIM1->CR1 |= TIM_CR1_CEN;                                               \
        TIM1->CCER |= TIM_CCER_CC4E;                                            \
        TIM1->BDTR |= TIM_BDTR_MOE;                                             \
        if (mdi_g431_fault_active() || mdi_g431_fault_source_active()) {        \
            g_bG431FaultLatched = true;                                         \
            return MDI_BUSY;                                                    \
        }                                                                       \
        TIM1->CCER |= MDI_G431_PWM_PHASE_MASK;                                  \
        if (mdi_g431_fault_active() || mdi_g431_fault_source_active() ||        \
            ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)) {                              \
            TIM1->CCER &= ~MDI_G431_PWM_PHASE_MASK;                             \
            g_bG431FaultLatched = true;                                         \
            return MDI_BUSY;                                                    \
        }                                                                       \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE bool MDI_OP(NAME, _pwm_IsEnabled)(void)                          \
    {                                                                           \
        return ((TIM1->CR1 & TIM_CR1_CEN) != 0U) &&                             \
               ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) &&                           \
               ((TIM1->CCER & MDI_G431_PWM_PHASE_MASK) ==                       \
                MDI_G431_PWM_PHASE_MASK);                                       \
    }                                                                           \
    MDI_INLINE bool MDI_OP(NAME, _pwm_FaultActive)(void)                        \
    {                                                                           \
        return mdi_g431_fault_active();                                         \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_ClearFault)(void)                 \
    {                                                                           \
        return mdi_g431_fault_clear();                                          \
    }

/** @brief Calculate a near-target frequency with maximum timer resolution.
 * @param wClockHz Timer input clock, not the APB bus clock.
 * @param wHz Requested nonzero output frequency.
 * @param wFactor One for edge aligned, two for center aligned.
 * @param ptTiming Output divider and ticks; untouched on failure.
 * @return Status; achieved Hz is clock / divider / ticks / factor.
 */
MDI_INLINE mdi_status_t mdi_stm32_PlanFrequency(
    uint32_t wClockHz, uint32_t wHz, uint32_t wFactor,
    mdi_pwm_timing_t *ptTiming)
{
    uint64_t qwBase = (uint64_t)wHz * wFactor;
    uint64_t qwDivider = 0U;
    uint64_t qwPeriod = 0U;
    uint64_t qwDenominator = 0U;
    if (ptTiming == NULL || (wFactor != 1U && wFactor != 2U)) {
        return MDI_INVALID;
    }
    if (wHz == 0U || wClockHz == 0U || qwBase * 2U > wClockHz) {
        return MDI_RANGE;
    }
    qwDenominator = qwBase * 65535U;
    qwDivider = ((uint64_t)wClockHz + qwDenominator - 1U) / qwDenominator;
    if (qwDivider == 0U || qwDivider > 65536U) {
        return MDI_RANGE;
    }
    qwDenominator = qwBase * qwDivider;
    qwPeriod = ((uint64_t)wClockHz + qwDenominator / 2U) / qwDenominator;
    if (qwPeriod < 2U || qwPeriod > 65535U) {
        return MDI_RANGE;
    }
    ptTiming->wDivider = (uint32_t)qwDivider;
    ptTiming->wPeriodTicks = (uint32_t)qwPeriod;
    return MDI_OK;
}

#define MDI_STM32_PWM_PERIOD(T, F) ((T)->ARR + (((F) == 1U) ? 1U : 0U))
#define MDI_CORE_PWM_ZERO(N, R, M) (R) = 0U;
#define MDI_CORE_DUTY_INVALID(N, R, M) || (ptDuty->N > 65536U)
#define MDI_CORE_DUTY_CONVERT(N, R, M)                                          \
    tCounts.N = (uint32_t)(((uint64_t)ptDuty->N * wPeriod + 32768U) >> 16U);
#define MDI_CORE_CYCLE_INVALID(N, R, M) || (ptCycle->tCompare.N > wPeriod)

/** @brief Bind timer run control and an explicit output gate.
 * @param NAME Existing PWM resource token.
 * @param TIMER Timer register block expression containing CR1.CEN.
 * @param GATE Output gate lvalue, such as an advanced timer BDTR register.
 * @param GATE_MASK Mask that opens the safe output gate.
 * @return Generates Enable, SafeStop and IsEnabled capabilities.
 * @note Disable clears the output gate before stopping the counter. Enable
 * starts the counter while outputs remain gated, then opens the gate.
 */
#define MDI_STM32_PWM_LIFECYCLE_BIND(NAME, TIMER, GATE, GATE_MASK)              \
    _Static_assert((GATE_MASK) != 0U, "PWM output gate mask must be nonzero");  \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_Enable)(bool bEnable)             \
    {                                                                           \
        if (bEnable) {                                                          \
            (TIMER)->CR1 |= 1U;                                                 \
            (GATE) |= (GATE_MASK);                                              \
        } else {                                                                \
            (GATE) &= ~(GATE_MASK);                                             \
            (TIMER)->CR1 &= ~1U;                                                \
        }                                                                       \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SafeStop)(void)                   \
    {                                                                           \
        (GATE) &= ~(GATE_MASK);                                                 \
        (TIMER)->CR1 &= ~1U;                                                    \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE bool MDI_OP(NAME, _pwm_IsEnabled)(void)                          \
    {                                                                           \
        return (((TIMER)->CR1 & 1U) != 0U) &&                                   \
               (((GATE) & (GATE_MASK)) != 0U);                                  \
    }

/** @brief Bind lifecycle control with a latched fault and live fault source.
 * @param FAULT_ACTIVE Latched hardware/software fault expression.
 * @param FAULT_SOURCE_ACTIVE Live source expression; must be clear to reset.
 * @param CLEAR_FAULT Statement that acknowledges the latched fault.
 * @return Generates lifecycle and fault status operations.
 * @note Enable is rejected while the latch is active. SafeStop never clears
 * a fault; ClearFault is the only operation allowed to acknowledge it.
 */
#define MDI_STM32_PWM_LIFECYCLE_FAULT_BIND(                                     \
    NAME, TIMER, GATE, GATE_MASK, FAULT_ACTIVE, FAULT_SOURCE_ACTIVE,            \
    CLEAR_FAULT)                                                                \
    _Static_assert((GATE_MASK) != 0U, "PWM output gate mask must be nonzero");  \
    MDI_INLINE bool MDI_OP(NAME, _pwm_FaultActive)(void)                        \
    {                                                                           \
        return (FAULT_ACTIVE) != 0U;                                            \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_ClearFault)(void)                 \
    {                                                                           \
        if ((FAULT_SOURCE_ACTIVE) != 0U) { return MDI_BUSY; }                   \
        (CLEAR_FAULT);                                                          \
        return (FAULT_ACTIVE) != 0U ? MDI_IO_ERROR : MDI_OK;                    \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_Enable)(bool bEnable)             \
    {                                                                           \
        if (bEnable) {                                                          \
            if ((FAULT_ACTIVE) != 0U) { return MDI_BUSY; }                      \
            (TIMER)->CR1 |= 1U;                                                 \
            (GATE) |= (GATE_MASK);                                              \
        } else {                                                                \
            (GATE) &= ~(GATE_MASK);                                             \
            (TIMER)->CR1 &= ~1U;                                                \
        }                                                                       \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SafeStop)(void)                   \
    {                                                                           \
        (GATE) &= ~(GATE_MASK);                                                 \
        (TIMER)->CR1 &= ~1U;                                                    \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE bool MDI_OP(NAME, _pwm_IsEnabled)(void)                          \
    {                                                                           \
        return (((TIMER)->CR1 & 1U) != 0U) &&                                   \
               (((GATE) & (GATE_MASK)) != 0U);                                  \
    }

/** @brief Add frequency and normalized duty to a timer's frame provider.
 * @param NAME Existing MDI_PWM_REG_BIND resource token.
 * @param TIMER Pointer expression to a configured STM32 timer register block.
 * @param CLOCK_HZ Stable timer clock in Hz; clock changes need a new binding.
 * @param FACTOR One=edge/upcounter; two=center. Must match configured CMS.
 * @param CHANNELS Same X-list as the frame provider; maxima must track ARR.
 * @return Generates optional SetFrequency and SetDuty operations.
 * @note Duty frames and tick frames are distinct C types. Changing frequency
 * while running returns BUSY; no partial reconfiguration occurs on error.
 */
#define MDI_STM32_PWM_TIMING_BIND(NAME, TIMER, CLOCK_HZ, FACTOR, CHANNELS)      \
    _Static_assert((FACTOR) == 1U || (FACTOR) == 2U, "invalid PWM alignment");  \
    _Static_assert((CLOCK_HZ) > 0U, "invalid timer clock");                     \
    typedef struct {                                                            \
        CHANNELS(MDI_CORE_PWM_FIELD)                                            \
    } MDI_PWM_DutyFrame(NAME);                                                  \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SetFrequency)(                    \
        uint32_t wHz)                                                           \
    {                                                                           \
        mdi_pwm_timing_t tTiming = {0};                                         \
        mdi_status_t eStatus = MDI_OK;                                          \
        const uint32_t wControl = (TIMER)->CR1;                                 \
        if ((wControl & 3U) != 0U) { return MDI_BUSY; }                         \
        if (((wControl & 0x60U) != 0U) != ((FACTOR) == 2U)) {                   \
            return MDI_INVALID;                                                 \
        }                                                                       \
        if ((FACTOR) == 1U && (wControl & 0x10U) != 0U) {                       \
            return MDI_INVALID;                                                 \
        }                                                                       \
        eStatus = mdi_stm32_PlanFrequency(                                      \
            (CLOCK_HZ), wHz, (FACTOR), &tTiming);                               \
        if (eStatus != MDI_OK) { return eStatus; }                              \
        (TIMER)->PSC = tTiming.wDivider - 1U;                                   \
        (TIMER)->ARR = tTiming.wPeriodTicks - (((FACTOR) == 1U) ? 1U : 0U);     \
        CHANNELS(MDI_CORE_PWM_ZERO)                                             \
        (TIMER)->EGR = 1U;                                                      \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SetDuty)(                         \
        const MDI_PWM_DutyFrame(NAME) *ptDuty)                                  \
    {                                                                           \
        MDI_PWM_Frame(NAME) tCounts = {0};                                      \
        uint32_t wPeriod = 0U;                                                  \
        if (ptDuty == NULL) { return MDI_INVALID; }                             \
        if (false CHANNELS(MDI_CORE_DUTY_INVALID)) { return MDI_RANGE; }        \
        wPeriod = MDI_STM32_PWM_PERIOD(TIMER, FACTOR);                          \
        if (wPeriod < 2U || wPeriod > 65535U) { return MDI_RANGE; }             \
        CHANNELS(MDI_CORE_DUTY_CONVERT)                                         \
        MDI_PWM_StageFast(NAME, &tCounts);                                      \
        return MDI_OK;                                                          \
    }

/** @brief Opt in to an ARR/CCR preload cycle with explicit update events.
 * @param NAME Existing frame/timing resource token.
 * @param TIMER Configured STM32 timer with ARR and all bound CCR preloads.
 * @param FACTOR One for edge alignment; two for center alignment.
 * @param CHANNELS Same compare channel list as the frame provider.
 * @return Generates Cycle type, StageCycle and Commit capabilities.
 * @note The owner must choose a safe update window and account for UG's
 * counter/repetition/trigger side effects. Do not bind to the G431 bridge.
 */
#define MDI_STM32_PWM_CYCLE_BIND(NAME, TIMER, FACTOR, CHANNELS)                 \
    typedef struct {                                                            \
        uint32_t wPeriodTicks;                                                  \
        MDI_PWM_Frame(NAME) tCompare;                                           \
    } MDI_PWM_Cycle(NAME);                                                      \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_StageCycle)(                      \
        const MDI_PWM_Cycle(NAME) *ptCycle)                                     \
    {                                                                           \
        uint32_t wPeriod = 0U;                                                  \
        if (ptCycle == NULL) { return MDI_INVALID; }                            \
        wPeriod = ptCycle->wPeriodTicks;                                        \
        if (wPeriod < 2U || wPeriod > 65535U) { return MDI_RANGE; }             \
        if (false CHANNELS(MDI_CORE_CYCLE_INVALID)) { return MDI_RANGE; }       \
        if (((TIMER)->CR1 & 0x80U) == 0U) { return MDI_INVALID; }               \
        (TIMER)->ARR = wPeriod - (((FACTOR) == 1U) ? 1U : 0U);                  \
        MDI_PWM_StageFast(NAME, &ptCycle->tCompare);                            \
        return MDI_OK;                                                          \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_Commit)(                          \
        void)                                                                   \
    {                                                                           \
        (TIMER)->EGR = 1U;                                                      \
        return MDI_OK;                                                          \
    }
#endif
