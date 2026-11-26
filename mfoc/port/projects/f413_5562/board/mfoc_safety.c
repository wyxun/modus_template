/**
 * @file mfoc_safety.c
 * @brief AT-MOTOR-EVB V1.1 control frame and bridge protection.
 * @author Codex
 * @date 2026-11-25
 */
#include "mfoc_safety.h"
#include "mdi/instance.h"

/* Measured 20 kHz window: 3900..4600 counts down from a top of 5000.
 * The 200 MHz timer clock fixes ADC conversion delay in timer counts. */
#define FRAME_LEAD_COUNTS_LOW  UINT32_C(400)
#define FRAME_LEAD_COUNTS_HIGH UINT32_C(1100)
#define START_LEAD_COUNTS      UINT32_C(1500)
#define TIMER_COUNT_DOWN       (UINT32_C(1) << 4U)
#define START_TIMEOUT_CYCLES   (AT32F413_TMR1_HZ / 500U)
#define MISSING_FRAME_TICKS    UINT32_C(3)

/* 123 ADC counts is approximately 3 A with the board's 33 mV/A gain.
 * This raw backstop stays independent of the motor's PU command limits. */
#ifndef AT32F413_SAFETY_RAW_DELTA_LIMIT
#define AT32F413_SAFETY_RAW_DELTA_LIMIT UINT32_C(123)
#endif
#define RAW_DELTA_MAX (AT32F413_SAFETY_RAW_DELTA_LIMIT - 1U)
_Static_assert(AT32F413_SAFETY_RAW_DELTA_LIMIT > 0U &&
               AT32F413_SAFETY_RAW_DELTA_LIMIT <= 1024U,
               "raw margin must fit the calibrated ADC offset range");
_Static_assert(PWM_PERIOD > START_LEAD_COUNTS,
               "PWM period must leave room for the ADC completion window");

static volatile bool s_armed;
static volatile mfoc_at32_safety_fault_t s_fault;
static uint32_t s_min_u, s_min_v, s_min_w;
/* ADC alone publishes the sequence. SysTick alone tracks its age. */
static volatile uint32_t s_frame;
static uint32_t s_seen;
static uint32_t s_age;

/** @brief Close the bridge and save the cause for Motor.
 * @param fault First detected guard fault. @return False for the ADC owner. */
static inline bool _mfoc_at32_safety_Trip(mfoc_at32_safety_fault_t fault)
{
    (void)MDI_PWM_SafeStop(bridge);
    s_fault = fault;
    s_armed = false;
    return false;
}

bool mfoc_at32_safety_StartWindowOpen(void)
{
    return (TMR1->ctrl1 & TIMER_COUNT_DOWN) != 0U &&
        PWM_PERIOD - TMR1->cval <= START_LEAD_COUNTS;
}

mfoc_result_t mfoc_at32_safety_WaitStart(void)
{
    if (MDI_PWM_IsEnabled(bridge) || MDI_PWM_FaultActive(bridge) ||
        (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        return MFOC_RESULT_SAFETY;
    }
    TMR1->ists = ~(uint32_t)TMR_OVF_FLAG;
    const uint32_t start = DWT->CYCCNT;
    while ((TMR1->ists & TMR_OVF_FLAG) == 0U &&
           DWT->CYCCNT - start < START_TIMEOUT_CYCLES) {}
    if ((TMR1->ists & TMR_OVF_FLAG) == 0U ||
        !mfoc_at32_safety_StartWindowOpen()) {
        return MFOC_RESULT_SAFETY;
    }
    /* A conversion from the prior PWM period may have completed during wait. */
    mdi_at32_adc_AckCurrentFrame();
    while (!MDI_Sample_IsReady(phase_current) &&
           DWT->CYCCNT - start < START_TIMEOUT_CYCLES) {}
    if (!MDI_Sample_IsReady(phase_current)) { return MFOC_RESULT_IO; }
    mdi_at32_adc_AckCurrentFrame();
    if (MDI_PWM_FaultActive(bridge) || !mfoc_at32_safety_StartWindowOpen()) {
        return MFOC_RESULT_SAFETY;
    }
    return MFOC_RESULT_OK;
}

void mfoc_at32_safety_Arm(uint32_t offset_u, uint32_t offset_v,
                         uint32_t offset_w)
{
    s_min_u = offset_u - RAW_DELTA_MAX;
    s_min_v = offset_v - RAW_DELTA_MAX;
    s_min_w = offset_w - RAW_DELTA_MAX;
    s_frame = 0U;
    s_seen = 0U;
    s_age = 0U;
    s_fault = MFOC_AT32_SAFETY_NONE;
    s_armed = true;
}

void mfoc_at32_safety_Disarm(void)
{
    s_armed = false;
}

mfoc_at32_safety_fault_t mfoc_at32_safety_GetFault(void)
{
    return s_fault;
}

bool mfoc_at32_safety_Frame(void)
{
    if (!s_armed) { return true; }
    const uint32_t after_top = PWM_PERIOD - TMR1->cval;
    if ((TMR1->ctrl1 & TIMER_COUNT_DOWN) == 0U ||
        after_top - FRAME_LEAD_COUNTS_LOW >
            FRAME_LEAD_COUNTS_HIGH - FRAME_LEAD_COUNTS_LOW) {
        return _mfoc_at32_safety_Trip(MFOC_AT32_SAFETY_PHASE);
    }
#if AT32F413_BOARD_HALL
    const uint32_t hall = MDI_IO_Read(hall_inputs);
    if (hall == 0U || hall == 7U) {
        return _mfoc_at32_safety_Trip(MFOC_AT32_SAFETY_HALL);
    }
#endif
    if (ADC1->pdt1 - s_min_u > 2U * RAW_DELTA_MAX ||
        ADC1->pdt2 - s_min_v > 2U * RAW_DELTA_MAX ||
        ADC1->pdt3 - s_min_w > 2U * RAW_DELTA_MAX) {
        return _mfoc_at32_safety_Trip(MFOC_AT32_SAFETY_OVERCURRENT);
    }
    ++s_frame;
    return true;
}

void mfoc_at32_safety_Tick(void)
{
    if (!s_armed) { return; }
    const uint32_t frame = s_frame;
    mfoc_at32_safety_fault_t fault = MFOC_AT32_SAFETY_NONE;
    if (MDI_PWM_FaultActive(bridge)) {
        fault = MFOC_AT32_SAFETY_BREAK;
    } else if (!MDI_PWM_IsEnabled(bridge)) {
        fault = MFOC_AT32_SAFETY_GATE;
    } else {
        s_age = frame == s_seen ? s_age + 1U : 1U;
        s_seen = frame;
        if (s_age >= MISSING_FRAME_TICKS) {
            fault = MFOC_AT32_SAFETY_MISSING_FRAME;
        }
    }
    if (fault == MFOC_AT32_SAFETY_NONE) { return; }

    /* The ADC can preempt SysTick before its fault decision is committed. */
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (s_armed &&
        (fault != MFOC_AT32_SAFETY_MISSING_FRAME || frame == s_frame)) {
        (void)_mfoc_at32_safety_Trip(fault);
    }
    __set_PRIMASK(mask);
}
