/**
 * @file mfoc_p10_timing.c
 * @brief G431 timing capture and stopped Motor shell reporting.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_p10_timing.h"
#include "mfoc_p10_window.h"
#include "mfoc_app.h"
#include "mdebug/mshell.h"
#include "stm32g4xx.h"

#if !MSHELL_ENABLE || defined(__NO_USE_LOG__) || MWAVEFORM_ENABLE || \
    MFOC_ISR_MEASURE
#error "P10 needs shell and logging, with waveform and ISR timing disabled"
#endif

#define P10_TIMER_HZ 170000000U

enum {
    P10_ALL,
    P10_LIMITED,
#if MFOC_FEEDBACK_ENABLE_SMO
    P10_SMO_WINDOW,
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    P10_HFI_GROUP,
#endif
    P10_GROUP_COUNT
};

typedef struct {
    uint32_t writes;
    uint32_t changed;
    uint32_t late;
    uint32_t reserve_fail;
    uint32_t max_irq;
    uint32_t max_trigger;
    int32_t min_slack;
} p10_group_t;

typedef struct {
    uint32_t count;
    uint32_t edge_compare;
    uint32_t available;
    uint32_t trigger_to_write;
    uint32_t irq_to_write;
    bool down;
} p10_worst_t;

static p10_group_t s_groups[P10_GROUP_COUNT];
static p10_worst_t s_worst;
static uint32_t s_arr;
static uint32_t s_trigger;
static uint32_t s_invalid;
static uint32_t s_gap;
static uint32_t s_no_write;
static uint32_t s_adc2_pending;
static uint32_t s_enter;
static uint32_t s_count;
static uint32_t s_write;
static uint32_t s_last_enter;
static uint32_t s_last_age;
static uint32_t s_previous[3];
static uint32_t s_submitted[3];
static bool s_ready;
static bool s_frame;
static bool s_down;
static bool s_wrote;
static bool s_have_previous;
static bool s_have_last_enter;
#if MFOC_FEEDBACK_ENABLE_SMO
static bool s_smo_window;
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
static bool s_hfi_group;
#endif

extern mfoc_app_t tMfocApp;

void mfoc_p10_Init(void)
{
    s_arr = TIM1->ARR;
    s_trigger = TIM1->CCR4;
    s_ready = SystemCoreClock == P10_TIMER_HZ && TIM1->PSC == 0U &&
        s_arr == P10_TIMER_HZ / MFOC_MOTOR_RUN_FREQ_HZ / 2U &&
        s_trigger < s_arr &&
        (TIM1->CR1 & TIM_CR1_CMS) == TIM_CR1_CMS_0 &&
        (TIM1->CCMR1 & (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE)) == 0U &&
        (TIM1->CCMR2 & TIM_CCMR2_OC3PE) == 0U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_ready = s_ready && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

void mfoc_p10_Begin(const mfoc_motor_t *motor)
{
    if (!s_ready || motor->state != MFOC_MOTOR_RUNNING) { return; }
    s_enter = DWT->CYCCNT;
    uint32_t direction = TIM1->CR1 & TIM_CR1_DIR;
    s_count = TIM1->CNT;
    if ((TIM1->CR1 & TIM_CR1_DIR) != direction) {
        ++s_invalid;
        return;
    }
    s_down = direction != 0U;
    s_wrote = false;
    s_frame = true;
    if ((ADC2->ISR & ADC_ISR_JEOS) == 0U) { ++s_adc2_pending; }
#if MFOC_FEEDBACK_ENABLE_SMO
    const mfoc_smo_t *smo = &motor->feedback.smo;
    s_smo_window = smo->has_angle &&
        (uint32_t)smo->speed_count + 1U == smo->c.speed_span;
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    s_hfi_group = (uint32_t)motor->feedback.hfi.carrier + 1U ==
        MFOC_HFI_CARRIER_SAMPLES;
#endif
}

void mfoc_p10_PwmWritten(void)
{
    if (s_frame) {
        /* Include the APB write completion and the probe's own delay. */
        __DSB();
        s_write = DWT->CYCCNT;
        s_submitted[0] = TIM1->CCR1;
        s_submitted[1] = TIM1->CCR2;
        s_submitted[2] = TIM1->CCR3;
        s_wrote = true;
    } else {
        /* The startup frame seeds the previous compares before enabling IRQ. */
        s_previous[0] = TIM1->CCR1;
        s_previous[1] = TIM1->CCR2;
        s_previous[2] = TIM1->CCR3;
        s_have_previous = true;
    }
}

/** @brief Group latency and conservative edge margin for one accepted frame.
 * @param group Target group. @param window Measured timing.
 * @param irq Cycles from the probe's entry timestamp. @param changed Duty change.
 */
static void _p10_Accumulate(p10_group_t *group, const mfoc_p10_window_t *window,
    uint32_t irq, bool changed)
{
    ++group->writes;
    if (irq > group->max_irq) { group->max_irq = irq; }
    if (window->trigger_to_write > group->max_trigger) {
        group->max_trigger = window->trigger_to_write;
    }
    if (!changed) { return; }
    if (group->changed == 0U || window->slack < group->min_slack) {
        group->min_slack = window->slack;
    }
    ++group->changed;
    if (window->slack <= 0) { ++group->late; }
    if (!mfoc_p10_has_reserve(window)) { ++group->reserve_fail; }
}

void mfoc_p10_End(const mfoc_motor_t *motor)
{
    if (!s_frame) { return; }
    s_frame = false;
    if (!s_wrote) {
        ++s_no_write;
        return;
    }

    uint32_t highest = 0U;
    bool previous_valid = s_have_previous;
    for (unsigned i = 0U; i < 3U; ++i) {
        uint32_t old = s_previous[i];
        uint32_t next = s_submitted[i];
        if (old != next) {
            uint32_t edge = old > next ? old : next;
            if (edge > highest) { highest = edge; }
        }
        s_previous[i] = next;
    }
    s_have_previous = true;
    mfoc_p10_window_t window = {0};
    uint32_t irq = s_write - s_enter;
    if (!previous_valid || !mfoc_p10_window(s_arr, s_trigger, s_count,
        s_down, irq, highest, &window)) {
        ++s_invalid;
        return;
    }

    if (s_have_last_enter) {
        const int64_t interval = (int64_t)(s_enter - s_last_enter) +
            s_last_age - window.age_at_entry;
        if (interval < (int64_t)s_arr || interval > (int64_t)(3U * s_arr)) {
            ++s_gap;
        }
    }
    s_last_enter = s_enter;
    s_last_age = window.age_at_entry;
    s_have_last_enter = true;
    const bool changed = highest != 0U;
    if (changed && (s_groups[P10_ALL].changed == 0U ||
        window.slack < s_groups[P10_ALL].min_slack)) {
        s_worst = (p10_worst_t){.count = s_count, .edge_compare = highest,
            .available = window.available,
            .trigger_to_write = window.trigger_to_write,
            .irq_to_write = irq, .down = s_down};
    }
    _p10_Accumulate(&s_groups[P10_ALL], &window, irq, changed);
    if (motor->previous_core_output_valid &&
        motor->previous_core_output.limited) {
        _p10_Accumulate(&s_groups[P10_LIMITED], &window, irq, changed);
    }
#if MFOC_FEEDBACK_ENABLE_SMO
    if (s_smo_window && motor->feedback.smo.has_angle &&
        motor->feedback.smo.speed_count == 0U) {
        _p10_Accumulate(&s_groups[P10_SMO_WINDOW], &window, irq, changed);
    }
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    if (s_hfi_group && motor->feedback.hfi.carrier == 0U) {
        _p10_Accumulate(&s_groups[P10_HFI_GROUP], &window, irq, changed);
    }
#endif
}

/** @brief Report or reset the capture while the motor is stopped.
 * @param args Arguments after p10.
 */
static void _p10_Command(const char *args)
{
    if (tMfocApp.motor.state == MFOC_MOTOR_RUNNING) {
        MLOGF(W, "p10: stop motor before report or clear\r\n");
        return;
    }
    if (mshell_ConsumeWord(&args, "clear") && mshell_AtEnd(args)) {
        for (unsigned i = 0U; i < P10_GROUP_COUNT; ++i) {
            s_groups[i] = (p10_group_t){0};
        }
        s_invalid = 0U;
        s_gap = 0U;
        s_no_write = 0U;
        s_adc2_pending = 0U;
        s_have_last_enter = false;
        MLOGF(I, "p10 cleared\r\n");
        return;
    }
    if (!mshell_AtEnd(args)) {
        MLOGF(W, "usage: p10 [clear]\r\n");
        return;
    }
    MLOGF(I, "p10 hz=%lu arr=%lu ccr4=%lu ready=%u invalid=%lu gap=%lu "
        "no_write=%lu adc2_pending=%lu faults=0x%lx\r\n",
        (unsigned long)P10_TIMER_HZ, (unsigned long)s_arr,
        (unsigned long)s_trigger, (unsigned)s_ready, (unsigned long)s_invalid,
        (unsigned long)s_gap, (unsigned long)s_no_write,
        (unsigned long)s_adc2_pending, (unsigned long)tMfocApp.motor.faults);
    static const char *const names[P10_GROUP_COUNT] = {
        "all", "limited",
#if MFOC_FEEDBACK_ENABLE_SMO
        "smo_window",
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
        "hfi_group",
#endif
    };
    for (unsigned i = 0U; i < P10_GROUP_COUNT; ++i) {
        const p10_group_t *group = &s_groups[i];
        MLOGF(I, "p10 %s writes=%lu changed=%lu late=%lu reserve_fail=%lu "
            "max_irq=%lu max_trigger=%lu min_slack=%ld\r\n", names[i],
            (unsigned long)group->writes, (unsigned long)group->changed,
            (unsigned long)group->late, (unsigned long)group->reserve_fail,
            (unsigned long)group->max_irq, (unsigned long)group->max_trigger,
            (long)group->min_slack);
    }
    if (s_groups[P10_ALL].changed != 0U) {
        MLOGF(I, "p10 worst count=%lu down=%u edge_compare=%lu "
            "available=%lu trigger_to_write=%lu irq_to_write=%lu\r\n",
            (unsigned long)s_worst.count, (unsigned)s_worst.down,
            (unsigned long)s_worst.edge_compare,
            (unsigned long)s_worst.available,
            (unsigned long)s_worst.trigger_to_write,
            (unsigned long)s_worst.irq_to_write);
    }
}

MODUS_SHELL_CMD(p10, _p10_Command, "P10 stopped capture report or clear");
