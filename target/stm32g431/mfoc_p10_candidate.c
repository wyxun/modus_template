/**
 * @file mfoc_p10_candidate.c
 * @brief G431 A/B preload timing. All times except section durations are
 * TIM2 ticks, referenced to TIM1's hardware-captured update.
 * @author Codex
 * @date 2026-11-21
 */
#include "mfoc_p10_timing.h"
#include "mfoc/mfoc_pwm_phase.h"
#include "mfoc_p10_deadline.h"
#include "mfoc_trace_binding.h"
#include "mfoc_app.h"
#if MFOC_P10_HFI_MPU != 0
#include "mfoc_p10_hfi_guard.h"
#endif
#include "mdebug/mshell.h"
#include <string.h>
#if MFOC_P10_EXIT_WITNESS
#include "mfoc_p10_exit.h"
#endif

#if !MSHELL_ENABLE || defined(__NO_USE_LOG__) || MWAVEFORM_ENABLE || \
    MFOC_ISR_MEASURE || !MFOC_TRACE_ENABLE
#error "Candidate requires shell, probe hooks and exclusive timing"
#endif

/* Event bits 0..4 are recorded at the branch itself. Others use the Motor
 * state bracketing the step. Every group can overlap every other group. */
enum {
    GROUP_ALL = 5, GROUP_ORDINARY, GROUP_LIMIT_ENTRY, GROUP_LIMIT_RECOVER,
    GROUP_SPEED_HEAVY, GROUP_HEAVY_LIMIT, GROUP_ALL_THREE, GROUP_START,
    GROUP_HANDOVER, GROUP_HFI_LOCK, GROUP_HFI_FIRST, GROUP_HFI_SECOND,
    GROUP_ID_ALIGN, GROUP_RS_LOW, GROUP_RS_HIGH, GROUP_LD, GROUP_LQ,
    GROUP_L_START, GROUP_L_LAST, GROUP_ID_SPINUP, GROUP_ID_FLUX,
    GROUP_ID_INERTIA, GROUP_ID_HANDOFF, GROUP_HFI_ACQUIRE, GROUP_HFI_TRACK,
    GROUP_HFI_POLARITY, GROUP_ID_PROTECT, GROUP_COUNT
};
#define GROUP_BIT(n) (UINT32_C(1) << (n))
_Static_assert(GROUP_COUNT <= 32, "group mask capacity");

static const char *const s_names[GROUP_COUNT] = {
    "speed_pi", "smo_window", "hfi_8_ok", "hfi_8_weak", "limited",
    "all", "ordinary", "limit_entry", "limit_recover", "speed_heavy",
    "heavy_limit", "speed_heavy_limit", "startup", "handover", "hfi_lock",
    "hfi_first", "hfi_second", "id_align", "rs_low", "rs_high", "ld", "lq",
    "l_start", "l_last", "id_spinup", "id_flux", "id_inertia", "id_handoff",
    "hfi_acquire", "hfi_track", "hfi_polarity", "id_protect"
};

typedef struct {
    uint32_t frames;
    uint32_t write_age;
    uint32_t exit_age;
} p10_group_t;

typedef struct {
    uint32_t seq, capture, trigger, update, enter, seen, write;
    volatile uint32_t tail;
    uint32_t events, adc1, adc2, duty[3];
    int32_t rc;
    uint16_t count;
    bool down, wrote, timed, valid;
#if MFOC_RUNTIME_BUS
    bool bus_applied;
#endif
#if MFOC_P10_SEGMENTS
    uint32_t sections[P10_SECTION_COUNT];
#endif
} p10_frame_t;

typedef struct {
    uint32_t aircr, iser[4], prio[5], systick_ctrl, ier1, ier2, dier;
    uint32_t primask, basepri;
    bool valid;
} p10_nvic_t;

volatile uint32_t g_p10_irq_entry;
static volatile uint32_t s_sink;
volatile uint32_t *g_p10_irq_tail = &s_sink;
uint32_t g_p10_events;
#if MFOC_P10_SEGMENTS
uint32_t g_p10_start[P10_SECTION_COUNT];
uint32_t g_p10_end[P10_SECTION_COUNT];
static uint32_t s_max_sections[P10_SECTION_COUNT];
static const char *const s_sections[P10_SECTION_COUNT] = {
    "port_current", "feedback_total", "position", "smo", "flux", "ekf",
    "hfi", "speed_pi", "ident", "core", "port_pwm"
};
#endif
#if MFOC_RUNTIME_BUS
bool g_p10_bus_applied;
static const char *const s_bus_names[] = {
    "bus_new", "bus_speed", "bus_smo_window", "bus_hfi_8", "bus_limit",
    "bus_speed_heavy", "bus_speed_heavy_limit",
#if MFOC_P10_LIMIT
    "bus_limit_speed_only", "bus_limit_smo_only",
#endif
};
enum { P10_BUS_GROUP_COUNT = sizeof(s_bus_names) / sizeof(s_bus_names[0]) };
static p10_group_t s_bus_groups[P10_BUS_GROUP_COUNT];
#endif
static p10_group_t s_groups[GROUP_COUNT];
static p10_frame_t s_first_error;
static uint32_t s_id_protect;
static p10_frame_t s_frame[2], s_worst_write, s_worst_exit;
static p10_nvic_t s_nvic;
static uint32_t s_slot, s_arr, s_ccr4, s_seq, s_prev_trigger;
static uint32_t s_window_fail, s_tail_fail, s_gaps, s_missing_write;
static uint32_t s_incomplete, s_invalid_time, s_max_response, s_max_seen;
static uint32_t s_errors, s_post, s_calibration, s_first_run, s_empty_irq;
static uint32_t s_max_error_tail;
static bool s_active, s_pending, s_have_prev, s_prev_limited, s_captured;
#if MFOC_FEEDBACK_ENABLE_HFI
static bool s_hfi_valid;
#endif
extern mfoc_app_t tMfocApp;

#if MFOC_P10_HFI_CAPTURE
#if !MFOC_FEEDBACK_ENABLE_HFI || MFOC_P10_PHASE != 2
#error "HFI capture requires the HFI feedback and B timing probe"
#endif
#define P10_HFI_CAPTURE_MAGIC UINT32_C(0x32494648) /* HFI2, little endian. */
#define P10_HFI_CAPTURE_PREFIX 128U
#define P10_HFI_CAPTURE_TAIL 128U
#define P10_HFI_CAPTURE_STRIDE 8U

typedef struct {
    uint32_t seq, events, angle_bam32;
    mfoc_ab_t delta_i[2], delta_u[2];
} p10_hfi_record_t;

/* Stopped SWD readout. Prefix retains all early groups; the tail rings at
 * one record per eight complete groups. P10 counters still count all groups. */
typedef struct {
    uint32_t magic, fractional_bits, groups, tail_next, injection_mpu;
    p10_hfi_record_t prefix[P10_HFI_CAPTURE_PREFIX];
    p10_hfi_record_t tail[P10_HFI_CAPTURE_TAIL];
} p10_hfi_capture_t;

_Static_assert(sizeof(p10_hfi_record_t) == 44U, "HFI capture record layout");
_Static_assert(sizeof(p10_hfi_capture_t) == 11284U, "HFI capture layout");
p10_hfi_capture_t g_p10_hfi_capture;
#if MFOC_P10_HFI_MPU != 0
mfoc_p10_hfi_guard_t g_p10_hfi_guard;
#endif

/** @brief Latch both coherent Di/Du columns after PWM service. */
static void p10_HfiCapture(const mfoc_hfi_t *hfi, uint32_t seq, uint32_t events)
{
    uint32_t group = g_p10_hfi_capture.groups++;
    p10_hfi_record_t *record;
    if (group < P10_HFI_CAPTURE_PREFIX) {
        record = &g_p10_hfi_capture.prefix[group];
    } else if (group % P10_HFI_CAPTURE_STRIDE != 0U) {
        return;
    } else {
        uint32_t index = g_p10_hfi_capture.tail_next;
        record = &g_p10_hfi_capture.tail[index];
        ++index;
        g_p10_hfi_capture.tail_next = index == P10_HFI_CAPTURE_TAIL ? 0U : index;
    }
    record->seq = seq;
    record->events = events;
    record->angle_bam32 = hfi->angle.wBam32;
    record->delta_i[0] = hfi->delta_i[0];
    record->delta_i[1] = hfi->delta_i[1];
    record->delta_u[0] = hfi->delta_u[0];
    record->delta_u[1] = hfi->delta_u[1];
}
#endif

/** Check live IRQ configuration after the bridge has enabled JEOS. */
static void p10_ReadNvic(void)
{
    static const IRQn_Type irqs[] = {ADC1_2_IRQn, DMA1_Channel1_IRQn,
        TIM1_BRK_TIM15_IRQn, SysTick_IRQn, USART2_IRQn};
    uint32_t expected[4] = {0};
    s_nvic.aircr = SCB->AIRCR;
    s_nvic.ier1 = ADC1->IER;
    s_nvic.ier2 = ADC2->IER;
    s_nvic.dier = TIM1->DIER;
    s_nvic.systick_ctrl = SysTick->CTRL;
    s_nvic.primask = __get_PRIMASK();
    s_nvic.basepri = __get_BASEPRI();
    for (unsigned i = 0U; i < 5U; ++i) {
        s_nvic.prio[i] = NVIC_GetPriority(irqs[i]);
        if (irqs[i] >= 0) {
            unsigned n = (unsigned)irqs[i];
            expected[n / 32U] |= GROUP_BIT(n % 32U);
        }
    }
    bool ok = NVIC_GetPriorityGrouping() == 3U && s_nvic.prio[0] == 1U &&
        s_nvic.prio[1] == 2U && s_nvic.prio[2] == 2U &&
        s_nvic.prio[3] == 3U && s_nvic.prio[4] == 4U &&
        (s_nvic.systick_ctrl & SysTick_CTRL_TICKINT_Msk) != 0U &&
        s_nvic.ier1 == ADC_IER_JEOSIE && s_nvic.ier2 == 0U &&
        s_nvic.dier == TIM_DIER_BIE;
#if MFOC_P10_EXIT_WITNESS
    expected[(unsigned)TIM2_IRQn / 32U] |=
        GROUP_BIT((unsigned)TIM2_IRQn % 32U);
    ok = ok && NVIC_GetPriority(TIM2_IRQn) == 2U;
#endif
    for (unsigned i = 0U; i < 4U; ++i) {
        s_nvic.iser[i] = NVIC->ISER[i];
        if (s_nvic.iser[i] != expected[i]) { ok = false; }
    }
    s_nvic.valid = ok;
}

/** Decode the identification stage before the sample advances it. */
#if MFOC_IDENTIFICATION_ENABLE
static uint32_t p10_IdGroup(const mfoc_identification_t *id)
{
    if (id->state != MFOC_IDENTIFICATION_RUNNING) { return 0U; }
    if (id->stage_done) { return GROUP_BIT(GROUP_ID_HANDOFF); }
    switch (id->stage) {
    case MFOC_IDENTIFICATION_STAGE_ALIGN:
        return GROUP_BIT(GROUP_ID_ALIGN);
    case MFOC_IDENTIFICATION_STAGE_RS:
        return id->work.rs.level == MFOC_IDENTIFICATION_RS_LOW ?
            GROUP_BIT(GROUP_RS_LOW) : GROUP_BIT(GROUP_RS_HIGH);
    case MFOC_IDENTIFICATION_STAGE_LD:
    case MFOC_IDENTIFICATION_STAGE_LQ: {
        uint32_t bits = id->stage == MFOC_IDENTIFICATION_STAGE_LD ?
            GROUP_BIT(GROUP_LD) : GROUP_BIT(GROUP_LQ);
        if (id->work.inductance.count == 0U) {
            bits |= GROUP_BIT(GROUP_L_START);
        } else if (id->work.inductance.count ==
                   MFOC_IDENTIFICATION_L_SERIES_INTERVALS) {
            bits |= GROUP_BIT(GROUP_L_LAST);
        }
        return bits;
    }
    case MFOC_IDENTIFICATION_STAGE_SPINUP: return GROUP_BIT(GROUP_ID_SPINUP);
    case MFOC_IDENTIFICATION_STAGE_FLUX: return GROUP_BIT(GROUP_ID_FLUX);
    case MFOC_IDENTIFICATION_STAGE_INERTIA: return GROUP_BIT(GROUP_ID_INERTIA);
    default: return GROUP_BIT(GROUP_ID_HANDOFF);
    }
}
#endif

void mfoc_p10_Init(void)
{
#if MFOC_P10_HFI_CAPTURE
    g_p10_hfi_capture.magic = P10_HFI_CAPTURE_MAGIC;
    g_p10_hfi_capture.injection_mpu =
        (uint32_t)(MFOC_HFI_VOLTAGE_PU * 1000.0f + 0.5f);
#if defined(MFOC_NUMERIC_FIXED)
    g_p10_hfi_capture.fractional_bits = MFOC_Q_FRACTION_BITS;
#endif
#endif
    s_arr = TIM1->ARR;
    s_ccr4 = TIM1->CCR4;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t t = DWT->CYCCNT;
    s_calibration = DWT->CYCCNT - t;
#if MFOC_P10_EXIT_WITNESS
    mfoc_p10_exit_Init();
#endif
}

void mfoc_p10_Begin(const mfoc_motor_t *motor)
{
#if MFOC_RUNTIME_BUS
    g_p10_bus_applied = false;
#endif
    s_active = motor->state == MFOC_MOTOR_RUNNING;
    if (!s_active) { ++s_empty_irq; return; }
    p10_frame_t *f = &s_frame[s_slot];
    f->enter = g_p10_irq_entry;
    f->adc1 = ADC1->ISR;
    f->adc2 = ADC2->ISR;
    f->seen = TIM2->CNT;
    f->capture = TIM2->CCR1;
    f->seq = ++s_seq;
    f->down = (TIM1->CR1 & TIM_CR1_DIR) != 0U;
    f->count = (uint16_t)TIM1->CNT;
    f->trigger = f->update = 0U;
    f->timed = g_mfoc_g431_pwm_phase.locked && mfoc_p10_frame_times(
        MFOC_P10_PHASE, s_arr, s_ccr4, f->capture, f->enter,
        &f->trigger, &f->update);
    f->valid = (f->adc1 & f->adc2 & ADC_ISR_JEOS) != 0U;
    f->wrote = false;
    f->write = 0U;
    f->duty[0] = f->duty[1] = f->duty[2] = 0U;
    if (!f->valid) { ++s_incomplete; }
    if (!f->timed) { ++s_invalid_time; }
    g_p10_events = GROUP_BIT(GROUP_ALL);
    if (motor->hold_active || motor->openloop_active ||
        motor->sensorless_start.state != MFOC_SENSORLESS_START_NONE) {
        g_p10_events |= GROUP_BIT(GROUP_START);
    }
    s_captured = motor->sensorless_start.captured;
#if MFOC_IDENTIFICATION_ENABLE
    g_p10_events |= p10_IdGroup(&motor->identification);
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    s_hfi_valid = motor->feedback.hfi_output.valid;
    const mfoc_hfi_t *h = &motor->feedback.hfi;
    if (motor->feedback.hfi_mode == MFOC_FEEDBACK_HFI_INJECT) {
        if (h->state == MFOC_HFI_ACQUIRE || h->state == MFOC_HFI_TRACK) {
            g_p10_events |= h->state == MFOC_HFI_TRACK ?
                GROUP_BIT(GROUP_HFI_TRACK) : GROUP_BIT(GROUP_HFI_ACQUIRE);
#if MFOC_PWM_PRELOAD_DELAY
            if (h->has_current && motor->previous_voltage_ab_valid &&
                h->applied_tag >= 1U &&
                h->applied_tag <= MFOC_HFI_CARRIER_SAMPLES) {
                g_p10_events |= (h->applied_tag & 1U) != 0U ?
                    GROUP_BIT(GROUP_HFI_FIRST) : GROUP_BIT(GROUP_HFI_SECOND);
            }
#else
            if (h->has_current && motor->previous_voltage_ab_valid) {
                g_p10_events |= (h->carrier & 1U) == 0U ?
                    GROUP_BIT(GROUP_HFI_FIRST) : GROUP_BIT(GROUP_HFI_SECOND);
            }
#endif
        } else {
            g_p10_events |= GROUP_BIT(GROUP_HFI_POLARITY);
        }
    }
#endif
#if MFOC_P10_SEGMENTS
    memset(g_p10_start, 0, sizeof(g_p10_start));
    memset(g_p10_end, 0, sizeof(g_p10_end));
#endif
}

void mfoc_p10_PwmWritten(void)
{
    if (s_active) {
        __DSB();
        p10_frame_t *f = &s_frame[s_slot];
        f->write = TIM2->CNT;
        f->wrote = true;
        f->duty[0] = TIM1->CCR1;
        f->duty[1] = TIM1->CCR2;
        f->duty[2] = TIM1->CCR3;
    }
}

/** Called after the following PWM submission (or by the stopped command).
 * The preceding tail stamp thus includes the previous frame's accounting. */
static void p10_Finalize(const p10_frame_t *f)
{
    if ((f->events & GROUP_BIT(GROUP_ID_PROTECT)) != 0U) { ++s_id_protect; }
    if (s_first_error.seq == 0U && (!f->timed || !f->valid || !f->wrote ||
        f->rc != MFOC_RESULT_OK ||
        (f->events & GROUP_BIT(GROUP_ID_PROTECT)) != 0U)) { s_first_error = *f; }
    if (!f->timed) { return; }
    const uint32_t period = 2U * s_arr;
    const uint32_t e = f->tail - f->trigger;
    if (e >= period) { ++s_tail_fail; }
    if (!f->valid || !f->wrote || f->rc != MFOC_RESULT_OK) {
        if (e > s_max_error_tail) { s_max_error_tail = e; }
        return;
    }
    const uint32_t w = f->write - f->trigger;
    const uint32_t window = f->update - f->trigger;
    if (mfoc_p10_reserve(window, w, 0U, 0U) < 0) { ++s_window_fail; }
    if (f->enter - f->trigger > s_max_response) {
        s_max_response = f->enter - f->trigger;
    }
    if (f->seen - f->trigger > s_max_seen) {
        s_max_seen = f->seen - f->trigger;
    }
    if (e - w > s_post) { s_post = e - w; }
    if (s_groups[GROUP_ALL].frames == 0U ||
        w > s_groups[GROUP_ALL].write_age) { s_worst_write = *f; }
    if (s_groups[GROUP_ALL].frames == 0U ||
        e > s_groups[GROUP_ALL].exit_age) { s_worst_exit = *f; }
#if MFOC_RUNTIME_BUS
    if (f->bus_applied) {
        const uint32_t ev = f->events;
        const bool hits[P10_BUS_GROUP_COUNT] = {true, (ev & P10_SPEED_PI) != 0U,
            (ev & P10_SMO_WINDOW) != 0U,
            (ev & (P10_HFI_EIGHTH_OK | P10_HFI_EIGHTH_WEAK)) != 0U,
            (ev & P10_LIMITED) != 0U,
            (ev & GROUP_BIT(GROUP_SPEED_HEAVY)) != 0U,
            (ev & GROUP_BIT(GROUP_ALL_THREE)) != 0U,
#if MFOC_P10_LIMIT
            (ev & (P10_LIMITED | P10_SPEED_PI | P10_SMO_WINDOW)) ==
                (P10_LIMITED | P10_SPEED_PI),
            (ev & (P10_LIMITED | P10_SPEED_PI | P10_SMO_WINDOW)) ==
                (P10_LIMITED | P10_SMO_WINDOW),
#endif
        };
        for (unsigned i = 0U; i < P10_BUS_GROUP_COUNT; ++i) {
            if (hits[i]) {
                p10_group_t *g = &s_bus_groups[i];
                ++g->frames;
                if (w > g->write_age) { g->write_age = w; }
                if (e > g->exit_age) { g->exit_age = e; }
            }
        }
    }
#endif
    uint32_t groups = f->events;
    while (groups != 0U) {
        unsigned bit = (unsigned)__builtin_ctz(groups);
        p10_group_t *g = &s_groups[bit];
        ++g->frames;
        if (w > g->write_age) { g->write_age = w; }
        if (e > g->exit_age) { g->exit_age = e; }
        groups &= groups - 1U;
    }
#if MFOC_P10_SEGMENTS
    for (unsigned i = 0U; i < P10_SECTION_COUNT; ++i) {
        if (f->sections[i] > s_max_sections[i]) {
            s_max_sections[i] = f->sections[i];
        }
    }
#endif
}

void mfoc_p10_End(const mfoc_motor_t *motor)
{
    g_p10_irq_tail = &s_sink;
    if (!s_active) { return; }
    if (s_pending) { p10_Finalize(&s_frame[s_slot ^ 1U]); }
    p10_frame_t *f = &s_frame[s_slot];
    f->rc = motor->irq_result;
    if (!f->wrote) { ++s_missing_write; }
    if (f->rc != MFOC_RESULT_OK) { ++s_errors; }
    if (s_first_run != g_mfoc_g431_pwm_phase.startup_captures) {
        p10_ReadNvic();
        s_have_prev = false;
        s_prev_limited = false;
        s_first_run = g_mfoc_g431_pwm_phase.startup_captures;
    }
    if (f->timed) {
        if (s_have_prev && f->trigger - s_prev_trigger != 2U * s_arr) {
            ++s_gaps;
        }
        s_prev_trigger = f->trigger;
        s_have_prev = true;
    }
    const bool limited = motor->previous_core_output_valid &&
        motor->previous_core_output.limited && f->rc == MFOC_RESULT_OK;
    if (limited) {
        g_p10_events |= P10_LIMITED;
        if (!s_prev_limited) { g_p10_events |= GROUP_BIT(GROUP_LIMIT_ENTRY); }
    } else if (s_prev_limited) { g_p10_events |= GROUP_BIT(GROUP_LIMIT_RECOVER); }
    s_prev_limited = limited;
    if (!s_captured && motor->sensorless_start.captured) {
        g_p10_events |= GROUP_BIT(GROUP_HANDOVER);
    }
#if MFOC_IDENTIFICATION_ENABLE
    if (motor->identification.state == MFOC_IDENTIFICATION_ERROR) {
        g_p10_events |= GROUP_BIT(GROUP_ID_PROTECT);
    }
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    if (!s_hfi_valid && motor->feedback.hfi_output.valid) {
        g_p10_events |= GROUP_BIT(GROUP_HFI_LOCK);
    }
#endif
    if ((g_p10_events & (P10_SMO_WINDOW | P10_HFI_EIGHTH_OK |
                         P10_HFI_EIGHTH_WEAK)) != 0U) {
        if (limited) { g_p10_events |= GROUP_BIT(GROUP_HEAVY_LIMIT); }
        if ((g_p10_events & P10_SPEED_PI) != 0U) {
            g_p10_events |= GROUP_BIT(GROUP_SPEED_HEAVY);
            if (limited) { g_p10_events |= GROUP_BIT(GROUP_ALL_THREE); }
        }
    } else if ((g_p10_events & ~(GROUP_BIT(GROUP_ALL) |
        GROUP_BIT(GROUP_HFI_FIRST) | GROUP_BIT(GROUP_HFI_SECOND) |
        GROUP_BIT(GROUP_HFI_TRACK))) == 0U) {
        g_p10_events |= GROUP_BIT(GROUP_ORDINARY);
    }
#if MFOC_P10_HFI_CAPTURE
    if ((g_p10_events & (P10_HFI_EIGHTH_OK | P10_HFI_EIGHTH_WEAK)) != 0U) {
        p10_HfiCapture(&motor->feedback.hfi, f->seq, g_p10_events);
    }
#endif
    f->events = g_p10_events;
#if MFOC_RUNTIME_BUS
    f->bus_applied = g_p10_bus_applied;
#endif
#if MFOC_P10_SEGMENTS
    for (unsigned i = 0U; i < P10_SECTION_COUNT; ++i) {
        f->sections[i] = g_p10_end[i] - g_p10_start[i];
    }
#endif
#if MFOC_P10_EXIT_WITNESS
    if (f->timed) {
        uint32_t flags = (!f->valid || !f->wrote || f->rc != MFOC_RESULT_OK) ?
            MFOC_P10_EXIT_ABNORMAL : 0U;
        if ((f->events & P10_SPEED_PI) != 0U) { flags |= MFOC_P10_EXIT_SPEED; }
        if ((f->events & P10_SMO_WINDOW) != 0U) { flags |= MFOC_P10_EXIT_SMO; }
        if ((f->events & (P10_HFI_EIGHTH_OK | P10_HFI_EIGHTH_WEAK)) != 0U) {
            flags |= MFOC_P10_EXIT_HFI_8;
        }
        if (limited) { flags |= MFOC_P10_EXIT_LIMIT; }
#if MFOC_RUNTIME_BUS
        if (f->bus_applied) { flags |= MFOC_P10_EXIT_BUS; }
#endif
        mfoc_p10_exit_Pend(f->seq, f->trigger, 2U * s_arr, flags);
    }
#endif
    g_p10_irq_tail = &f->tail;
    s_slot ^= 1U;
    s_pending = true;
    s_active = false;
}


/** Print an intact frame so nested segments can be interpreted together. */
static void p10_PrintFrame(const char *label, const p10_frame_t *f)
{
    MLOGF(I, "p10 %s seq=%lu mask=%08lx cap=%lx T=%lx I=%lx Jseen=%lx "
        "W=%lx U=%lx Esoft=%lx\r\n", label, (unsigned long)f->seq,
        (unsigned long)f->events, (unsigned long)f->capture,
        (unsigned long)f->trigger, (unsigned long)f->enter,
        (unsigned long)f->seen, (unsigned long)f->write,
        (unsigned long)f->update, (unsigned long)f->tail);
    MLOGF(I, "p10 %s cnt=%u down=%u adc=%lx/%lx rc=%ld duty=%lu/%lu/%lu\r\n",
        label, (unsigned)f->count, (unsigned)f->down, (unsigned long)f->adc1,
        (unsigned long)f->adc2, (long)f->rc, (unsigned long)f->duty[0],
        (unsigned long)f->duty[1], (unsigned long)f->duty[2]);
#if MFOC_P10_SEGMENTS
    for (unsigned i = 0U; i < P10_SECTION_COUNT; ++i) {
        MLOGF(I, "p10 %s %s=%lu max=%lu\r\n", label, s_sections[i],
            (unsigned long)f->sections[i], (unsigned long)s_max_sections[i]);
    }
#endif
}

/** A stopped calibration consumes only JEOS (W1C), after both converters
 * complete. Regular DMA, other ADC flags and the bridge keep their state. */
typedef struct {
    uint32_t accepted, rejected, lower_min, upper_max, bracket_max;
    uint32_t first_trigger, first_lower, first_upper, last_trigger;
} p10_j_bounds_t;
static p10_j_bounds_t s_cal;

/** Poll successive triggered frames with gates closed. A read pair brackets
 * each observed complete event even if an interrupt separates the reads. */
static __attribute__((noinline)) bool p10_CalibrateFrame(void)
{
    const uint32_t gates = TIM_CCER_CC1E | TIM_CCER_CC1NE |
        TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    const uint32_t period = 2U * s_arr;
    s_cal = (p10_j_bounds_t){.lower_min = UINT32_MAX};
    if (tMfocApp.motor.state == MFOC_MOTOR_RUNNING ||
        (TIM1->CCER & gates) != 0U ||
        ((ADC1->IER | ADC2->IER) & ADC_IER_JEOSIE) != 0U ||
        !g_mfoc_g431_pwm_phase.locked || period == 0U ||
        (TIM1->CR1 & TIM_CR1_CEN) == 0U ||
        (TIM2->CR1 & TIM_CR1_CEN) == 0U) {
        return false;
    }
    const uint32_t start = TIM2->CNT;
    uint32_t lower = 0U, previous = 0U;
    bool synced = false, saw_missing = false;
    /* The first complete pair can include stopped, sticky JEOS. Discard it
     * before using any timestamps. The poll bound also handles a dead clock. */
    for (uint32_t n = 0U; n < 32U * period; ++n) {
        const uint32_t early = TIM2->CNT;
        const bool a = (ADC1->ISR & ADC_ISR_JEOS) != 0U;
        const bool b = (ADC2->ISR & ADC_ISR_JEOS) != 0U;
        const uint32_t late = TIM2->CNT;
        if (late - start > 24U * period) { break; }
        if (!a || !b) {
            saw_missing = true;
            lower = early;
            continue;
        }
        const uint32_t capture = TIM2->CCR1;
        ADC1->ISR = ADC_ISR_JEOS;
        ADC2->ISR = ADC_ISR_JEOS;
        __DSB();
        uint32_t t = 0U, u = 0U;
        bool timed = mfoc_p10_frame_times(MFOC_P10_PHASE, s_arr, s_ccr4,
            capture, late, &t, &u);
        if (synced) {
            if (!timed || !saw_missing || t - previous != period ||
                late - lower >= period) {
                ++s_cal.rejected;
            } else {
                /* If last incomplete predates T, T is the stronger bound. */
                uint32_t hi = late - t;
                uint32_t lo = late - lower > hi ? 0U : lower - t;
                if (s_cal.accepted == 0U) {
                    s_cal.first_trigger = t;
                    s_cal.first_lower = lo;
                    s_cal.first_upper = hi;
                }
                s_cal.last_trigger = t;
                if (lo < s_cal.lower_min) { s_cal.lower_min = lo; }
                if (hi > s_cal.upper_max) { s_cal.upper_max = hi; }
                if (hi - lo > s_cal.bracket_max) {
                    s_cal.bracket_max = hi - lo;
                }
                ++s_cal.accepted;
            }
        }
        previous = t;
        synced = timed;
        saw_missing = false;
        if (s_cal.accepted == 16U) { break; }
    }
    return s_cal.accepted == 16U && s_cal.rejected == 0U;
}

#if MFOC_P10_LIMIT
static bool s_stagger_speed;
#endif

#if MFOC_P10_SYNC_SPEED
static bool p10_SpeedRouteReady(const mfoc_motor_t *motor)
{
    const mfoc_sensorless_start_t *start = &motor->sensorless_start;
    return motor->state == MFOC_MOTOR_RUNNING &&
        motor->mode == MFOC_MOTOR_MODE_SPEED &&
        (start->state == MFOC_SENSORLESS_START_NONE ||
         (start->state == MFOC_SENSORLESS_START_ACQUIRE && start->captured &&
          start->offset == 0)) && motor->feedback.smo_output.valid;
}

/** @brief Phase the next speed PI relative to the next SMO window. */
static bool p10_SetSpeedPhase(mfoc_motor_t *motor, bool stagger)
{
    const mfoc_smo_t *smo = &motor->feedback.smo;
    const bool ok = p10_SpeedRouteReady(motor) && motor->speed_divider >= 2U &&
        motor->speed_divider == smo->c.speed_span &&
        smo->speed_count < smo->c.speed_span;
    if (ok) {
        const uint16_t next = (uint16_t)(smo->c.speed_span - smo->speed_count);
        motor->speed_count = stagger ? (smo->speed_count == 0U ? 0U : next) :
                                       (uint16_t)(next - 1U);
#if MFOC_P10_LIMIT
        s_stagger_speed = stagger;
#endif
    }
    return ok;
}

static void p10_SyncSpeed(bool stagger)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool ok = p10_SetSpeedPhase(&tMfocApp.motor, stagger);
    __set_PRIMASK(primask);
    MLOGF(I, "p10 %s ok=%u\r\n", stagger ? "stagger_speed" : "sync_speed",
          (unsigned)ok);
}
#endif

#if MFOC_P10_LIMIT
#define P10_LIMIT_FRAMES 50U
#define P10_LIMIT_DURATION (MFOC_PERFC_HZ * 2U)
#define P10_LIMIT_SPACING (MFOC_PERFC_HZ / 100U)
static volatile uint32_t s_limit_remaining;
static uint32_t s_limit_start, s_limit_last;

bool mfoc_p10_LimitFrame(const mfoc_motor_t *motor)
{
    if (s_limit_remaining == 0U) { return false; }
    const uint32_t now = TIM2->CNT;
    if (now - s_limit_start >= P10_LIMIT_DURATION ||
        !p10_SpeedRouteReady(motor)) {
        s_limit_remaining = 0U;
        return false;
    }
    const mfoc_smo_t *smo = &motor->feedback.smo;
    const bool speed_due = motor->speed_count == 0U;
    const bool smo_due = (unsigned)smo->speed_count + 1U == smo->c.speed_span;
    bool heavy = speed_due && smo_due;
    if (s_stagger_speed) {
        const bool alternate = (s_limit_remaining & 1U) == 0U ?
            speed_due : smo_due;
        heavy = g_p10_bus_applied ? speed_due || smo_due : alternate;
    }
    const uint32_t elapsed = now - s_limit_last;
    /* Collisions take priority. Sparse single frames leave the loop time
     * to recover, and never raise the allowed physical duty. */
    if (s_limit_remaining != P10_LIMIT_FRAMES &&
        !(heavy && (elapsed >= P10_LIMIT_SPACING ||
                   (g_p10_bus_applied && elapsed >= P10_LIMIT_SPACING / 2U)))) {
        return false;
    }
    s_limit_last = now;
    --s_limit_remaining;
    return true;
}

static void p10_ArmLimit(bool set_phase, bool stagger)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mfoc_motor_t *motor = &tMfocApp.motor;
    const bool ok = s_limit_remaining == 0U &&
        (set_phase ? p10_SetSpeedPhase(motor, stagger) :
                     p10_SpeedRouteReady(motor));
    if (ok) {
        s_limit_start = TIM2->CNT;
        s_limit_last = s_limit_start;
        s_limit_remaining = P10_LIMIT_FRAMES;
    }
    __set_PRIMASK(primask);
    MLOGF(I, "p10 limit ok=%u\r\n", (unsigned)ok);
}
#endif

/** Read stopped statistics. The hardware capture continues with gates off. */
static void p10_Command(const char *args)
{
#if MFOC_P10_SYNC_SPEED
    const char *command = args;
    if (mshell_ConsumeWord(&command, "sync_speed") && mshell_AtEnd(command)) {
        p10_SyncSpeed(false);
        return;
    }
#if MFOC_P10_LIMIT
    const char *stagger = args;
    if (mshell_ConsumeWord(&stagger, "stagger_speed") &&
        mshell_AtEnd(stagger)) {
        p10_SyncSpeed(true);
        return;
    }
#endif
#endif
#if MFOC_P10_LIMIT
    const char *limit = args;
    if (mshell_ConsumeWord(&limit, "limit")) {
        if (mshell_AtEnd(limit)) {
            p10_ArmLimit(false, false);
            return;
        }
        bool stagger = mshell_ConsumeWord(&limit, "staggered");
        if ((stagger || mshell_ConsumeWord(&limit, "same")) &&
            mshell_AtEnd(limit)) {
            p10_ArmLimit(true, stagger);
            return;
        }
    }
#endif
    if (tMfocApp.motor.state == MFOC_MOTOR_RUNNING) {
        MLOGF(W, "p10: stop motor for readout\r\n");
        return;
    }
    if (mshell_ConsumeWord(&args, "frame") && mshell_AtEnd(args)) {
        bool ok = p10_CalibrateFrame();
        MLOGF(I, "p10 stopped_J ok=%u n=%lu rejected=%lu lower_min=%lu "
            "upper_max=%lu max_bracket=%lu\r\n", (unsigned)ok,
            (unsigned long)s_cal.accepted, (unsigned long)s_cal.rejected,
            (unsigned long)(s_cal.accepted != 0U ? s_cal.lower_min : 0U),
            (unsigned long)s_cal.upper_max, (unsigned long)s_cal.bracket_max);
        MLOGF(I, "p10 stopped_J first_T=%lx first_lo=%lu first_hi=%lu "
            "last_T=%lx\r\n", (unsigned long)s_cal.first_trigger,
            (unsigned long)s_cal.first_lower, (unsigned long)s_cal.first_upper,
            (unsigned long)s_cal.last_trigger);
        return;
    }
    if (s_pending) {
        p10_Finalize(&s_frame[s_slot ^ 1U]);
        s_pending = false;
    }
    if (mshell_ConsumeWord(&args, "clear") && mshell_AtEnd(args)) {
        memset(s_groups, 0, sizeof(s_groups));
        s_first_error = (p10_frame_t){0};
        s_id_protect = 0U;
#if MFOC_P10_EXIT_WITNESS
        mfoc_p10_exit_Reset();
#endif
#if MFOC_P10_LIMIT
        s_limit_remaining = 0U;
#if MFOC_SMO_STAGGER_SPEED_PI
        s_stagger_speed = true;
#else
        s_stagger_speed = false;
#endif
#endif
#if MFOC_RUNTIME_BUS
        memset(s_bus_groups, 0, sizeof(s_bus_groups));
#endif
#if MFOC_P10_SEGMENTS
        memset(s_max_sections, 0, sizeof(s_max_sections));
#endif
#if MFOC_P10_HFI_CAPTURE
        /* Reset the readable range; stale slots are outside it. */
        g_p10_hfi_capture.groups = 0U;
        g_p10_hfi_capture.tail_next = 0U;
#if MFOC_P10_HFI_MPU != 0
        g_p10_hfi_guard = (mfoc_p10_hfi_guard_t){0};
#endif
#endif
        s_seq = s_window_fail = s_tail_fail = s_gaps = s_missing_write = 0U;
        s_incomplete = s_invalid_time = s_max_response = s_max_seen = 0U;
        s_max_error_tail = s_errors = s_post = s_empty_irq = 0U;
        s_have_prev = false;
        s_nvic = (p10_nvic_t){0};
        s_first_run = UINT32_MAX;
        MLOGF(I, "p10 cleared\r\n");
        return;
    }
    if (!mshell_AtEnd(args)) {
        MLOGF(W, "usage: p10 [clear|frame]\r\n");
        return;
    }
    MLOGF(I, "p10 phase=%u segments=%u hz=%lu arr=%lu ccr4=%lu "
        "dwt_pair=%lu locked=%u failures=%lu\r\n",
        (unsigned)MFOC_P10_PHASE, (unsigned)MFOC_P10_SEGMENTS,
        (unsigned long)SystemCoreClock, (unsigned long)s_arr,
        (unsigned long)s_ccr4, (unsigned long)s_calibration,
        (unsigned)g_mfoc_g431_pwm_phase.locked,
        (unsigned long)g_mfoc_g431_pwm_phase.failures);
    MLOGF(I, "p10 cold_count=%lu u1=%lx u2=%lx starts=%lu last_start=%lx\r\n",
        (unsigned long)g_mfoc_g431_pwm_phase.cold_count,
        (unsigned long)g_mfoc_g431_pwm_phase.first_capture,
        (unsigned long)g_mfoc_g431_pwm_phase.second_capture,
        (unsigned long)g_mfoc_g431_pwm_phase.startup_captures,
        (unsigned long)g_mfoc_g431_pwm_phase.last_capture);
    MLOGF(I, "p10 nvic_ok=%u aircr=%08lx iser=%lx/%lx/%lx/%lx\r\n",
        (unsigned)s_nvic.valid, (unsigned long)s_nvic.aircr,
        (unsigned long)s_nvic.iser[0], (unsigned long)s_nvic.iser[1],
        (unsigned long)s_nvic.iser[2], (unsigned long)s_nvic.iser[3]);
    MLOGF(I, "p10 irqpri adc/dma/brk/systick/uart=%lu/%lu/%lu/%lu/%lu "
        "systcsr=%lx ier1/2=%lx/%lx dier=%lx primask/basepri=%lu/%lu\r\n",
        (unsigned long)s_nvic.prio[0], (unsigned long)s_nvic.prio[1],
        (unsigned long)s_nvic.prio[2], (unsigned long)s_nvic.prio[3],
        (unsigned long)s_nvic.prio[4], (unsigned long)s_nvic.systick_ctrl,
        (unsigned long)s_nvic.ier1, (unsigned long)s_nvic.ier2,
        (unsigned long)s_nvic.dier, (unsigned long)s_nvic.primask,
        (unsigned long)s_nvic.basepri);
    MLOGF(I, "p10 frames=%lu gap=%lu phase_invalid=%lu adc_incomplete=%lu "
        "no_write=%lu failed=%lu idle_ready=%lu faults=%lx\r\n",
        (unsigned long)s_seq, (unsigned long)s_gaps,
        (unsigned long)s_invalid_time, (unsigned long)s_incomplete,
        (unsigned long)s_missing_write, (unsigned long)s_errors,
        (unsigned long)s_empty_irq, (unsigned long)tMfocApp.motor.faults);
    MLOGF(I, "p10 preliminary J_lower=T E=software_tail setup=0 "
        "reserve_miss=%lu tail_miss=%lu max_T_I=%lu max_T_Jseen=%lu "
        "max_W_E=%lu max_T_error_tail=%lu\r\n", (unsigned long)s_window_fail,
        (unsigned long)s_tail_fail, (unsigned long)s_max_response,
        (unsigned long)s_max_seen, (unsigned long)s_post,
        (unsigned long)s_max_error_tail);
    for (unsigned i = 0U; i < GROUP_COUNT; ++i) {
        const p10_group_t *g = &s_groups[i];
        if (g->frames != 0U) {
            MLOGF(I, "p10 %s n=%lu max_T_W=%lu max_T_E=%lu\r\n", s_names[i],
                (unsigned long)g->frames, (unsigned long)g->write_age,
                (unsigned long)g->exit_age);
        }
    }
#if MFOC_RUNTIME_BUS
    for (unsigned i = 0U; i < P10_BUS_GROUP_COUNT; ++i) {
        const p10_group_t *g = &s_bus_groups[i];
        MLOGF(I, "p10 %s n=%lu max_T_W=%lu max_T_E=%lu\r\n", s_bus_names[i],
            (unsigned long)g->frames, (unsigned long)g->write_age,
            (unsigned long)g->exit_age);
    }
#endif
    if (s_first_error.seq != 0U) { p10_PrintFrame("first_error", &s_first_error); }
    MLOGF(I, "p10 id_protect=%lu\r\n", (unsigned long)s_id_protect);
    if (s_groups[GROUP_ALL].frames != 0U) {
        p10_PrintFrame("worst_write", &s_worst_write);
        p10_PrintFrame("worst_tail", &s_worst_exit);
    }
}
MODUS_SHELL_CMD(p10, p10_Command, "P10 stopped A/B timing report or clear");
