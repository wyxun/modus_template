/** @file mfoc_board_bus.c
 * Apply one immutable set of DC-link coefficients at a control frame boundary.
 */
#include "mfoc_board_bus.h"
#include "mfoc_port.h"
#include <stdatomic.h>
#if MFOC_P10_PHASE
#include "mfoc_trace_binding.h"
#endif

typedef struct {
    mfoc_tick_t started;
    mfoc_result_t status;
    uint32_t expires;
    mfoc_core_bus_coeff_t coeff;
} bus_slot_t;

static bus_slot_t slots[2];
static volatile uint32_t published;
/* IRQ is the sole writer while running. Foreground timestamps publications
 * relative to this counter. Unsigned addition covers control-frame wrap. */
static volatile uint32_t control_frame;
static uint32_t applied;
static uint32_t expires;
static bool first_frame;

#define BUS_MAX_TICKS ((UINT64_C(30) * MFOC_PERFC_HZ + 999U) / 1000U)

/* Exclusive deadline: the frame at exactly 30 ms must already fail.
 * When publication falls between two IRQs, rounding can reject at most one
 * frame early. Both target clocks have a whole number of ticks per frame. */
static uint32_t frames_left(mfoc_tick_t age)
{
    if (age >= BUS_MAX_TICKS) { return 0U; }
    const uint32_t remaining = (uint32_t)(BUS_MAX_TICKS - age);
#if MFOC_PERFC_HZ % MFOC_MOTOR_RUN_FREQ_HZ == 0
    return 1U + (remaining - 1U) /
        (MFOC_PERFC_HZ / MFOC_MOTOR_RUN_FREQ_HZ);
#else
    return 1U + (uint32_t)(((uint64_t)remaining *
        MFOC_MOTOR_RUN_FREQ_HZ - 1U) / MFOC_PERFC_HZ);
#endif
}

void mfoc_board_bus_Reset(void)
{
    /* Board initialization, with the motor stopped. */
    slots[0].status = MFOC_RESULT_INVALID_STATE;
    published = 0U;
    control_frame = 0U;
    applied = UINT32_MAX;
    expires = 0U;
    first_frame = true;
}

mfoc_result_t mfoc_board_bus_Publish(const mfoc_board_bus_sample_t *s)
{
    uint32_t next = published + 1U;
    bus_slot_t *slot = &slots[next & 1U];
    slot->started = s->started;
    slot->status = s->status;
    if (slot->status == MFOC_RESULT_OK) {
        slot->status = mfoc_core_PrepareBus(s->bus, &slot->coeff);
        if (slot->status == MFOC_RESULT_OK) {
            /* Read the IRQ epoch first. Preemption may shorten the remaining
             * budget, but can never give an old acquisition extra frames. */
            const uint32_t frame = control_frame;
            const uint32_t left = frames_left(mfoc_port_tick() - s->started);
            slot->expires = frame + left;
            if (left == 0U) { slot->status = MFOC_RESULT_SAFETY; }
        }
    }
    atomic_signal_fence(memory_order_release);
    published = next;
    return slot->status;
}

mfoc_result_t mfoc_board_bus_Arm(mfoc_tick_t now)
{
    const uint32_t seq = published;
    const bus_slot_t *slot = &slots[seq & 1U];
    if (slot->status != MFOC_RESULT_OK) { return slot->status; }
    if (now - slot->started >= BUS_MAX_TICKS) { return MFOC_RESULT_SAFETY; }
    /* Stopped: the control-frame counter did not advance during downtime. */
    first_frame = true;
    applied = seq - 1U;
    return MFOC_RESULT_OK;
}

mfoc_result_t mfoc_board_bus_Apply(mfoc_core_t *core)
{
    const uint32_t frame = control_frame + 1U;
    control_frame = frame;
    const uint32_t seq = published;
    if (seq == applied) {
        return (int32_t)(expires - frame) > 0 ?
            MFOC_RESULT_OK : MFOC_RESULT_SAFETY;
    }
    atomic_signal_fence(memory_order_acquire);
    const bus_slot_t *slot = &slots[seq & 1U];
    if (slot->status != MFOC_RESULT_OK) { return slot->status; }
    uint32_t deadline = slot->expires;
    if (first_frame) {
        const uint32_t left = frames_left(mfoc_port_tick() - slot->started);
        deadline = frame + left;
    }
    if ((int32_t)(deadline - frame) <= 0) { return MFOC_RESULT_SAFETY; }
    mfoc_core_ApplyBus(core, &slot->coeff);
    first_frame = false;
    expires = deadline;
    applied = seq;
#if MFOC_P10_PHASE
    g_p10_bus_applied = true;
#endif
    return MFOC_RESULT_OK;
}
