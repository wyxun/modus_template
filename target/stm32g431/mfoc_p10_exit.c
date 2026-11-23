/** @file mfoc_p10_exit.c
 * The lower-priority TIM2 IRQ witnesses return from the G431 ADC IRQ.
 */
#include "mfoc_p10_exit.h"
#include "stm32g4xx.h"
#include <stdbool.h>

#if MFOC_P10_PHASE != 2 || MFOC_P10_KCL || MFOC_P10_PHYSICAL
#error "Exit witness needs B top updates and exclusive TIM2 IRQ"
#endif

/* Conservative allowance for the TIM1 -> TIM2 phase reference. */
#define P10_EXIT_TRIGGER_GUARD 64U
#define P10_EXIT_WITNESS_PRIORITY 2U
#define P10_EXIT_ADC_PRIORITY 1U
#define P10_EXIT_MAGIC UINT32_C(0x50314558)

typedef struct {
    uint32_t seq;
    uint32_t trigger;
    uint32_t pending_tick;
    uint32_t period;
    uint32_t flags;
} p10_exit_request_t;

_Static_assert(sizeof(mfoc_p10_exit_summary_t) == 32U, "Exit group layout");
_Static_assert(sizeof(mfoc_p10_exit_evidence_t) == 548U, "Exit evidence layout");

mfoc_p10_exit_evidence_t g_p10_exit_witness;
static volatile p10_exit_request_t s_exit_request;

void mfoc_p10_exit_Reset(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM2->DIER &= ~TIM_DIER_CC2IE;
    TIM2->SR = ~TIM_SR_CC2IF;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    g_p10_exit_witness = (mfoc_p10_exit_evidence_t){
        .magic = P10_EXIT_MAGIC, .trigger_guard_ticks = P10_EXIT_TRIGGER_GUARD,
    };
    __set_PRIMASK(primask);
}

void mfoc_p10_exit_Init(void)
{
    mfoc_p10_exit_Reset();
    NVIC_SetPriority(TIM2_IRQn, P10_EXIT_WITNESS_PRIORITY);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void mfoc_p10_exit_Pend(uint32_t seq, uint32_t trigger, uint32_t period,
                       uint32_t flags)
{
    if (g_p10_exit_witness.outstanding != 0U) {
        ++g_p10_exit_witness.replaced;
        NVIC_ClearPendingIRQ(TIM2_IRQn);
    }
    s_exit_request.seq = seq;
    s_exit_request.trigger = trigger;
    s_exit_request.period = period;
    s_exit_request.flags = flags;
    s_exit_request.pending_tick = TIM2->CNT;
    ++g_p10_exit_witness.armed;
    g_p10_exit_witness.outstanding = 1U;
    /* The ADC has higher priority. TIM2 can run only after it returns. */
    NVIC_SetPendingIRQ(TIM2_IRQn);
}

static void record(const p10_exit_request_t *request, uint32_t observed)
{
    static const uint32_t masks[MFOC_P10_EXIT_GROUP_COUNT] = {
        [MFOC_P10_EXIT_ALL] = 0U,
        [MFOC_P10_EXIT_ERRORS] = MFOC_P10_EXIT_ABNORMAL,
        [MFOC_P10_EXIT_SMO_WINDOW] = MFOC_P10_EXIT_SMO,
        [MFOC_P10_EXIT_HFI_EIGHTH] = MFOC_P10_EXIT_HFI_8,
        [MFOC_P10_EXIT_LIMITED] = MFOC_P10_EXIT_LIMIT,
        [MFOC_P10_EXIT_BUS_NEW] = MFOC_P10_EXIT_BUS,
        [MFOC_P10_EXIT_SPEED_SMO] = MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_SMO,
        [MFOC_P10_EXIT_SPEED_HFI] = MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_HFI_8,
        [MFOC_P10_EXIT_BUS_SPEED_SMO] =
            MFOC_P10_EXIT_BUS | MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_SMO,
        [MFOC_P10_EXIT_BUS_SPEED_HFI] =
            MFOC_P10_EXIT_BUS | MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_HFI_8,
        [MFOC_P10_EXIT_BUS_SPEED_SMO_LIMIT] =
            MFOC_P10_EXIT_BUS | MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_SMO |
            MFOC_P10_EXIT_LIMIT,
        [MFOC_P10_EXIT_BUS_SPEED_HFI_LIMIT] =
            MFOC_P10_EXIT_BUS | MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_HFI_8 |
            MFOC_P10_EXIT_LIMIT,
        [MFOC_P10_EXIT_BUS_SPEED_SMO_HFI] =
            MFOC_P10_EXIT_BUS | MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_SMO |
            MFOC_P10_EXIT_HFI_8,
        [MFOC_P10_EXIT_BUS_SPEED_SMO_HFI_LIMIT] =
            MFOC_P10_EXIT_BUS | MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_SMO |
            MFOC_P10_EXIT_HFI_8 | MFOC_P10_EXIT_LIMIT,
        [MFOC_P10_EXIT_LIMIT_ONLY] = MFOC_P10_EXIT_LIMIT,
        [MFOC_P10_EXIT_SPEED_SMO_LIMIT] =
            MFOC_P10_EXIT_SPEED | MFOC_P10_EXIT_SMO | MFOC_P10_EXIT_LIMIT,
    };
    const uint32_t age = observed - request->trigger;
    const bool late = request->period <= P10_EXIT_TRIGGER_GUARD ||
        age >= request->period - P10_EXIT_TRIGGER_GUARD;
    for (unsigned i = 0U; i < MFOC_P10_EXIT_GROUP_COUNT; ++i) {
        if ((request->flags & masks[i]) != masks[i]) { continue; }
        if (i == MFOC_P10_EXIT_LIMIT_ONLY &&
            request->flags != MFOC_P10_EXIT_LIMIT) { continue; }
        mfoc_p10_exit_summary_t *group = &g_p10_exit_witness.groups[i];
        if (group->witnessed == 0U || age > group->max_age_ticks) {
            group->max_age_ticks = age;
            group->seq = request->seq;
            group->trigger = request->trigger;
            group->compare = request->pending_tick;
            group->observed = observed;
            group->flags = request->flags;
        }
        ++group->witnessed;
        if (late) { ++group->late; }
    }
}

void TIM2_IRQHandler(void)
{
    /* An ADC IRQ before this short claim can replace the request. An ADC
     * IRQ after it can arm again without changing the claimed snapshot. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t observed = TIM2->CNT;
    const bool match = g_p10_exit_witness.outstanding != 0U;
    p10_exit_request_t request;
    if (match) {
        request = s_exit_request;
        g_p10_exit_witness.outstanding = 0U;
    }
    const bool adc_active = NVIC_GetActive(ADC1_2_IRQn) != 0U;
    __set_PRIMASK(primask);

    if (!match) { return; }
    if (adc_active) { ++g_p10_exit_witness.active_adc; return; }
    if (NVIC_GetPriorityGrouping() != 3U ||
        NVIC_GetPriority(ADC1_2_IRQn) != P10_EXIT_ADC_PRIORITY ||
        NVIC_GetPriority(TIM2_IRQn) != P10_EXIT_WITNESS_PRIORITY) {
        ++g_p10_exit_witness.bad_priority;
        return;
    }
    if (observed - request.pending_tick >= UINT32_C(0x80000000)) {
        ++g_p10_exit_witness.bad_compare;
        return;
    }
    record(&request, observed);
}
