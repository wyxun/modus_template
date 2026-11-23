/** @file board_bus_test.c
 * @brief Replay frame age, startup handoff and foreground publication.
 */
#include "mfoc_board_bus.h"
#include "mfoc_port.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Exact target periods for the two host configurations (16 and 20 kHz). */
#define FRAME_TICKS ((mfoc_tick_t)MFOC_PERFC_HZ / MFOC_MOTOR_RUN_FREQ_HZ)
#define TIMEOUT_TICKS ((mfoc_tick_t)MFOC_PERFC_HZ * 30U / 1000U)
_Static_assert(MFOC_PERFC_HZ % MFOC_MOTOR_RUN_FREQ_HZ == 0U, "test period");

static mfoc_core_t live;
static mfoc_tick_t now;
static unsigned int clock_reads;
static unsigned int interrupts_in_prepare;
static mfoc_scalar_t old_bus;
static bool interrupt_clock_read;

static mfoc_tick_t read_clock(void)
{
    const mfoc_tick_t captured = now;
    ++clock_reads;
    if (interrupt_clock_read) {
        interrupt_clock_read = false;
        now += FRAME_TICKS;
        assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    }
    return captured;
}

static mfoc_result_t prepare(mfoc_scalar_t bus, mfoc_core_bus_coeff_t *coeff)
{
    mfoc_result_t rc = mfoc_core_PrepareBus(bus, coeff);
    for (unsigned int i = 0U; i < interrupts_in_prepare; ++i) {
        now += FRAME_TICKS;
        assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
        assert(mfoc_core_GetBus(&live) == old_bus);
    }
    return rc;
}

#define mfoc_port_tick read_clock
#ifdef mfoc_core_PrepareBus
#undef mfoc_core_PrepareBus
#endif
#define mfoc_core_PrepareBus prepare
#include "../../../class/mfoc_board_bus.c"
#undef mfoc_core_PrepareBus
#undef mfoc_port_tick

static mfoc_result_t publish(mfoc_tick_t started, mfoc_scalar_t bus)
{
    const mfoc_board_bus_sample_t sample = {started, bus, MFOC_RESULT_OK};
    return mfoc_board_bus_Publish(&sample);
}

static void start_fresh(void)
{
    mfoc_board_bus_Reset();
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_INVALID_STATE);
    assert(publish(now, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
}

static void check_expiry(mfoc_tick_t started)
{
    const unsigned int reads = clock_reads;
    while (now + FRAME_TICKS < started + TIMEOUT_TICKS) {
        now += FRAME_TICKS;
        assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    }
    now += FRAME_TICKS;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_SAFETY);
    assert(clock_reads == reads);
}

static void start_and_restart(void)
{
    now = UINT32_MAX - 10U;
    start_fresh();
    check_expiry(now);
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_SAFETY);

    /* The time before the bridge opens belongs to the sample's age. */
    const mfoc_tick_t started = now;
    assert(publish(started, MFOC_HALF) == MFOC_RESULT_OK);
    now += TIMEOUT_TICKS - 4U * FRAME_TICKS;
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_OK);
    now += 2U * FRAME_TICKS;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    assert(mfoc_core_GetBus(&live) == MFOC_HALF);
    check_expiry(started);

    /* One raw clock tick of life remains on the actual first frame. */
    assert(publish(now, MFOC_HALF) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_OK);
    now += TIMEOUT_TICKS - 1U;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    now += FRAME_TICKS;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_SAFETY);

    /* A delayed first IRQ must reject a formerly valid startup frame. */
    assert(publish(now, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_OK);
    now += TIMEOUT_TICKS;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_SAFETY);
    assert(mfoc_core_GetBus(&live) == MFOC_HALF);

    /* Invalid age from the old run must not carry into a fresh re-arm. */
    assert(publish(now, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    assert(mfoc_core_GetBus(&live) == MFOC_ONE);
    check_expiry(now);
}

static void running_publication(void)
{
    mfoc_board_bus_Reset();
    published = UINT32_MAX - 1U;
    control_frame = UINT32_MAX - 2U;
    assert(publish(now, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Arm(now) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    old_bus = MFOC_ONE;
    interrupts_in_prepare = 1U;
    assert(publish(now, MFOC_HALF) == MFOC_RESULT_OK);
    interrupts_in_prepare = 0U;
    assert(published == 0U);
    /* Two newer publications without a control tick must select the last one. */
    assert(publish(now, MFOC_SCALAR(0.75f)) == MFOC_RESULT_OK);
    const mfoc_scalar_t last = MFOC_SCALAR(0.85f);
    assert(publish(now, last) == MFOC_RESULT_OK);
    mfoc_core_t ref = live;
    assert(mfoc_core_SetBus(&ref, last) == MFOC_RESULT_OK);
    const unsigned int reads = clock_reads;
    now += FRAME_TICKS;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    assert(clock_reads == reads);
    assert(memcmp(&live, &ref, sizeof live) == 0);
    const mfoc_core_input_t in = {.angle = {UINT32_C(0x12345678)},
        .u_inject_ab = {MFOC_ONE, MFOC_HALF}};
    mfoc_core_output_t a = {0}, b = {0};
    mfoc_core_Step(&live, &in, &a);
    mfoc_core_Step(&ref, &in, &b);
    assert(a.limited && a.bus == last && memcmp(&a, &b, sizeof a) == 0);

    /* The acquisition budget has exactly two nominal frame periods left. */
    const mfoc_tick_t started = now - TIMEOUT_TICKS + 2U * FRAME_TICKS;
    assert(publish(started, last) == MFOC_RESULT_OK);
    now += FRAME_TICKS;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_OK);
    check_expiry(started);
    assert(publish(started, MFOC_HALF) == MFOC_RESULT_SAFETY);
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_SAFETY);
    assert(memcmp(&live, &ref, sizeof live) == 0);
    const mfoc_board_bus_sample_t io = {now, MFOC_ZERO, MFOC_RESULT_IO};
    assert(mfoc_board_bus_Publish(&io) == MFOC_RESULT_IO);
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_IO);
    assert(publish(now, MFOC_ZERO) != MFOC_RESULT_OK);
    assert(mfoc_board_bus_Arm(now) != MFOC_RESULT_OK);

    start_fresh();
    old_bus = MFOC_ONE;
    interrupts_in_prepare = 3U;
    /* A near-expiry conversion becomes stale while preparing. Foreground
     * timing or the running frame counter must charge all three IRQs. */
    (void)publish(now - TIMEOUT_TICKS + 2U * FRAME_TICKS, MFOC_HALF);
    interrupts_in_prepare = 0U;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_SAFETY);
    assert(mfoc_core_GetBus(&live) == MFOC_ONE);

    /* The publisher reads the IRQ counter and the absolute clock separately.
     * A frame between the clock sample and its return costs one whole frame. */
    start_fresh();
    interrupt_clock_read = true;
    assert(publish(now - TIMEOUT_TICKS + 2U * FRAME_TICKS, MFOC_HALF) ==
           MFOC_RESULT_OK);
    assert(!interrupt_clock_read);
    now += FRAME_TICKS;
    assert(mfoc_board_bus_Apply(&live) == MFOC_RESULT_SAFETY);
    assert(mfoc_core_GetBus(&live) == MFOC_ONE);
}

/** Exercise foreground publication between IRQs, including either side of
 * an exact frame age. Rejection may precede expiry by less than one period. */
static void fractional_publication(void)
{
    const mfoc_tick_t phases[] = {0U, 1U, FRAME_TICKS / 2U, FRAME_TICKS - 1U};
    const mfoc_tick_t remaining[] = {1U, FRAME_TICKS - 1U, FRAME_TICKS,
        FRAME_TICKS + 1U, 2U * FRAME_TICKS, 2U * FRAME_TICKS + 1U};
    for (size_t i = 0U; i < sizeof phases / sizeof phases[0]; ++i) {
        for (size_t j = 0U; j < sizeof remaining / sizeof remaining[0]; ++j) {
            start_fresh();
            now += phases[i];
            const mfoc_tick_t deadline = now + remaining[j];
            assert(publish(deadline - TIMEOUT_TICKS, MFOC_HALF) ==
                   MFOC_RESULT_OK);
            now += FRAME_TICKS - phases[i];
            for (;;) {
                mfoc_result_t rc = mfoc_board_bus_Apply(&live);
                if (rc == MFOC_RESULT_SAFETY) {
                    assert(now >= deadline || deadline - now < FRAME_TICKS);
                    break;
                }
                assert(rc == MFOC_RESULT_OK && now < deadline);
                now += FRAME_TICKS;
            }
        }
    }
}

int main(void)
{
    start_and_restart();
    running_publication();
    fractional_publication();
    printf("Bus arm, age, IRQ clock reads, wrap and coefficients passed at %u Hz\n",
           (unsigned)MFOC_MOTOR_RUN_FREQ_HZ);
    return 0;
}
