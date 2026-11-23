/**
 * @file port_test.c
 * @brief Exercise the AT-MOTOR-EVB V1.1 Port against simulated registers.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_port.h"
#include "mdi/instance.h"
#if MFOC_RUNTIME_BUS
#include "mfoc_board_bus.h"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>

#if defined(AT32F413_BUS_NOMINAL_MV)
#define TEST_BUS_BASE_MV 24000U
#define TEST_BUS_RAW 1536U
#else
#define TEST_BUS_BASE_MV 12000U
#define TEST_BUS_RAW 768U
#endif

uint32_t SystemCoreClock = 200000000U;
test_adc_t test_adc = {.pdt1 = 2048U, .pdt2 = 2052U, .pdt3 = 2044U};
test_dma_t test_dma;
test_timer_t test_timer;
test_gpio_t test_gpio = {.idt = GPIO_PINS_4 | GPIO_PINS_12};
volatile uint16_t g_ahwAt32AdcOrdinary[AT32_ADC_ORDINARY_COUNT];

#define s_gate (test_timer.brk_bit.oen)
static int64_t s_ticks;
static bool s_frame_ready;
static bool s_irq;
#if AT32F413_BOARD_HALL
void TMR3_GLOBAL_IRQHandler(void);
test_timer_t test_hall_timer;
static bool s_manual_clock, s_edge_after_read, s_edge_during_seed;
static unsigned s_clock_preemptions;

uint32_t tmr_channel_value_get(tmr_type *tmr, unsigned channel)
{
    assert(tmr == TMR3 && channel == TMR_SELECT_CHANNEL_1);
    tmr->ists &= ~TMR_C1_FLAG;
    if (s_edge_after_read) { tmr->ists |= TMR_C1_FLAG; }
    return tmr->c1dt;
}
#endif

int64_t get_system_ticks(void)
{
#if AT32F413_BOARD_HALL
    if (s_manual_clock) {
        if (s_edge_during_seed) { TMR3->ists |= TMR_C1_FLAG; }
        if (s_clock_preemptions != 0U) {
            --s_clock_preemptions;
            TMR3->cval_bit.cval += 20U;
            s_ticks += 20U * AT32_HALL_TICKS_PER_US;
        }
        return s_ticks;
    }
#endif
    s_ticks += 1000;
    return s_ticks;
}

uint32_t perfc_get_systimer_frequency(void)
{
    return SystemCoreClock;
}

bool perfc_is_time_out_ms(uint32_t ms, int64_t *timestamp, bool repeat)
{
    if (s_frame_ready) { test_adc.sts |= ADC_PCCE_FLAG; }
    const int64_t now = get_system_ticks();
    if (*timestamp == 0) { *timestamp = now; }
    if (now - *timestamp < (int64_t)(SystemCoreClock / 1000U) * ms) {
        return false;
    }
    if (repeat) { *timestamp = now; }
    return true;
}

void mdi_at32_adc_EnableControlIRQ(void)
{
    assert(s_gate && test_timer.swevt == TMR_OVERFLOW_SWTRIG);
    s_irq = true;
}

#if AT32F413_BOARD_HALL
static void hall_edge(int64_t edge_us, unsigned delay_us, uint8_t code)
{
    assert((TMR3->iden & TMR_C1_INT) != 0U);
    test_gpio.idt = GPIO_PINS_12 | ((code & 3U) << 4U) | (code >> 2U);
    TMR3->c1dt = (uint16_t)edge_us;
    TMR3->cval_bit.cval = (uint16_t)(edge_us + delay_us);
    s_ticks = (edge_us + delay_us) * AT32_HALL_TICKS_PER_US;
    TMR3->ists |= TMR_C1_FLAG;
    TMR3_GLOBAL_IRQHandler();
}

static void check_hall_loss(void)
{
    mfoc_angle_t angle = {42U};
    mfoc_scalar_t speed = MFOC_ONE;
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_SAFETY);
    assert(angle.wBam32 == 42U && speed == MFOC_ONE);
    assert(mfoc_port_position_run() == MFOC_RESULT_SAFETY);
    assert((TMR3->iden & TMR_C1_INT) == 0U);
}

static void check_hall_capture(const mfoc_uvw_t *duty)
{
    mfoc_angle_t angle = {0U};
    mfoc_scalar_t speed = MFOC_ONE;
    static const uint8_t forward[] = {1U, 3U, 2U, 6U, 4U, 5U};
    s_manual_clock = true;
    int64_t t = ((s_ticks / AT32_HALL_TICKS_PER_US + 131071) /
        65536) * 65536 - 4010;
    s_ticks = t * AT32_HALL_TICKS_PER_US;
    assert(mfoc_port_position_calibrate_zero((mfoc_angle_t){0U}) ==
           MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == UINT32_C(0x15555555) && speed == MFOC_ZERO);
    assert(mfoc_port_position_calibrate_zero((mfoc_angle_t){0x80000000U}) ==
           MFOC_RESULT_SAFETY);
    /* The second capture straddles a 16-bit wrap and an ADC preemption.
     * Six queued sectors must produce 1/(6*2 ms) Hz regardless of lag. */
    for (unsigned i = 1; i <= 6U; ++i) {
        t += 2000;
        s_clock_preemptions = i == 2U;
        hall_edge(t, i == 2U ? 30U : 4U * i, forward[i % 6U]);
    }
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(speed) - 1.0f / 4.8f) < 0.00005f);
    for (unsigned i = 1; i <= 6U; ++i) {
        t += 2000;
        hall_edge(t, i * 5U, forward[(6U - i) % 6U]);
    }
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(speed) + 1.0f / 4.8f) < 0.00005f);
    /* The timeout is measured from the capture, not IRQ service. */
    s_ticks = (t + 100000) * AT32_HALL_TICKS_PER_US;
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    ++s_ticks;
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_SAFETY);
    t += INT64_C(25000000); /* Multiple TMR3 and low Port tick wraps. */
    hall_edge(t, 50U, 3U);
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(speed == MFOC_ZERO);
    /* Advance 8-bit queue indices through wrap with full batches. */
    for (unsigned i = 0; i < 264U; ++i) {
        t += 2000;
        hall_edge(t, 8U, forward[(i + 2U) % 6U]);
        if ((i + 1U) % 8U == 0U) {
            assert(mfoc_port_position_run() == MFOC_RESULT_OK);
        }
    }
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(speed) - 1.0f / 4.8f) < 0.00005f);
    for (unsigned i = 0; i < 9U; ++i) {
        t += 2000;
        hall_edge(t, 8U, forward[(i + 2U) % 6U]);
    }
    check_hall_loss();
    test_gpio.idt = GPIO_PINS_4 | GPIO_PINS_12;
    s_edge_during_seed = true;
    assert(mfoc_port_pwm_start(duty) == MFOC_RESULT_SAFETY && !s_gate);
    s_edge_during_seed = false;
    assert(mfoc_port_pwm_start(duty) == MFOC_RESULT_OK);
    TMR3->ists |= TMR_C1_RECAPTURE_FLAG;
    hall_edge(t += 2000, 3U, 3U);
    check_hall_loss();
    assert(mfoc_port_position_calibrate_zero((mfoc_angle_t){0x30000000U}) ==
           MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_start(duty) == MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_start(duty) == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    s_edge_after_read = true;
    hall_edge(t += 2000, 4U, 2U);
    s_edge_after_read = false;
    check_hall_loss();
    assert(mfoc_port_pwm_start(duty) == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    s_clock_preemptions = 2U;
    hall_edge(t += 2000, 4U, 3U);
    check_hall_loss();
    assert(mfoc_port_pwm_start(duty) == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    hall_edge(t += 2000, 6U, 0U);
    assert(mfoc_port_position_run() == MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_start(duty) == MFOC_RESULT_SAFETY && !s_gate);
}
#endif

/** @brief Verify configuration, signed ADC scaling and gate interlocks.
 * @return Zero on success.
 */
int main(void)
{
    const mfoc_port_cfg_t cfg = {
        .current_base_milliamp = 3500U,
        .voltage_base_millivolt = TEST_BUS_BASE_MV,
        .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
        .pole_pairs = 7U,
    };
    const mfoc_uvw_t duty = {MFOC_SCALAR(0.25f), MFOC_HALF,
                             MFOC_SCALAR(0.75f)};
    mfoc_uvw_t current = {MFOC_ONE, MFOC_ONE, MFOC_ONE};
    assert(mfoc_port_current(&current) == MFOC_RESULT_INVALID_STATE);
    assert(current.qU == MFOC_ONE && current.qW == MFOC_ONE);
    assert(mfoc_port_init(NULL) == MFOC_RESULT_NULL);
    SystemCoreClock = 48000000U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_SAFETY);
    assert(!s_gate && !s_irq);
    SystemCoreClock = 200000000U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_IO);
    assert(!s_gate && !s_irq);
    s_frame_ready = true;
    test_adc.pdt1 = 0U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_SAFETY);
    test_adc.pdt1 = 2048U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_OK);
    assert(!s_gate && !s_irq);
    assert(!MDI_Sample_IsReady(phase_current));
    assert(mfoc_port_tick_frequency() == 200000000U);
    const mfoc_tick_t before = mfoc_port_tick();
    assert(mfoc_port_tick() > before);
#if MFOC_RUNTIME_BUS
    mfoc_board_bus_Reset();
    mfoc_board_bus_sample_t sample;
    assert(!mfoc_board_bus_Poll(&sample)); /* issue SW trigger */
    assert(!mfoc_board_bus_Poll(&sample)); /* still in flight */
    g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE] = TEST_BUS_RAW;
    test_dma.sts = DMA1_FDT1_FLAG;
    assert(mfoc_board_bus_Poll(&sample) && sample.status == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Publish(&sample) == MFOC_RESULT_OK);
    assert(!mfoc_board_bus_Poll(&sample)); /* old FDT cannot refresh age */
    s_ticks += SystemCoreClock / 100U;
    assert(!mfoc_board_bus_Poll(&sample));
    test_dma.sts = DMA1_DTERR1_FLAG;
    assert(mfoc_board_bus_Poll(&sample) && sample.status == MFOC_RESULT_IO);
    test_dma.sts = 0U;
#endif

    test_adc.pdt1 = 2008U;
    test_adc.pdt2 = 2092U;
    test_adc.pdt3 = 2044U;
    test_adc.sts = ADC_PCCE_FLAG;
    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
    assert(test_adc.sts == ADC_PCCE_FLAG);
    const float sample_pu = 40.0f / (40.96f * 3.5f);
    assert(fabsf(mfoc_to_float(current.qU) - sample_pu) < 0.0001f);
    assert(fabsf(mfoc_to_float(current.qV) + sample_pu) < 0.0001f);
    assert(current.qW == MFOC_ZERO);

    mfoc_scalar_t bus = MFOC_ZERO;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_INVALID_STATE);
    assert(bus == MFOC_ZERO);
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_IO);
    assert(!s_gate && !s_irq && test_adc.ctrl2_bit.ocswtrg == 1U);
    assert(test_dma.clr == (DMA1_FDT1_FLAG | DMA1_DTERR1_FLAG));
    test_dma.sts = DMA1_FDT1_FLAG | DMA1_DTERR1_FLAG;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_INVALID_STATE);
    assert(bus == MFOC_ZERO);
    test_dma.sts = DMA1_FDT1_FLAG;
    g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE] = 100U;
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY);
    assert(!s_gate && !s_irq);
#if defined(AT32F413_BUS_NOMINAL_MV)
    /* ADC codes immediately outside 18–30 V, plus the 12 V and 48 V buses. */
    const uint16_t rejected_bus_raw[] = {1152U, 1921U, 768U, 3072U};
    for (unsigned i = 0; i < sizeof(rejected_bus_raw) / sizeof(uint16_t); ++i) {
        g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE] = rejected_bus_raw[i];
        assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY);
        assert(!s_gate && !s_irq);
    }
#endif
    g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE] = TEST_BUS_RAW;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(bus) - 1.0f) < 0.005f);

    test_gpio.idt &= ~GPIO_PINS_12;
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY);
    assert(!s_gate && !s_irq);
    assert(test_timer.swevt == 0U);
    test_gpio.idt |= GPIO_PINS_12;
    test_timer.ists = TMR_BRK_FLAG;
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_OK);
    assert(s_gate && s_irq);
    assert((test_timer.ists & TMR_BRK_FLAG) == 0U);
#if MFOC_RUNTIME_BUS
    mfoc_core_t control = {0};
    /* The bridge accepted a new DMA transaction. Delay the first control
     * sample until that transaction expires, then re-arm on a fresh scan. */
    s_ticks += (int64_t)(SystemCoreClock / 1000U) * 30;
    assert(mfoc_board_bus_Apply(&control) == MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK && !s_gate);
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_OK);
    assert(mfoc_board_bus_Apply(&control) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(mfoc_core_GetBus(&control)) - 1.0f) < 0.005f);
#endif
    assert(test_timer.c1dt == PWM_PERIOD / 4U);
    assert(test_timer.c2dt == PWM_PERIOD / 2U);
    assert(test_timer.c3dt == 3U * PWM_PERIOD / 4U);
    const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
    assert(mfoc_port_pwm(&neutral) == MFOC_RESULT_OK);
    assert(s_gate && test_timer.c3dt == PWM_PERIOD / 2U);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(!s_gate && s_irq);
#if defined(AT32F413_BUS_NOMINAL_MV)
    /* First and last ADC codes inside the 18–30 V window. */
    const uint16_t accepted_bus_raw[] = {1153U, 1920U};
    for (unsigned i = 0; i < sizeof(accepted_bus_raw) / sizeof(uint16_t); ++i) {
        g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE] = accepted_bus_raw[i];
        assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_OK && s_gate);
        assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK && !s_gate);
    }
    g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE] = TEST_BUS_RAW;
#endif
    assert(mfoc_port_pwm(&duty) == MFOC_RESULT_SAFETY);
    /* A recurring physical fault must survive duty and repeated Stop. */
    test_timer.ists |= TMR_BRK_FLAG;
    test_gpio.idt &= ~GPIO_PINS_12;
    assert(mfoc_port_pwm(&duty) == MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY);
    assert((test_timer.ists & TMR_BRK_FLAG) != 0U && !s_gate);
    test_gpio.idt |= GPIO_PINS_12;
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_OK && s_gate);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_ENABLE_POSITION
    check_hall_capture(&duty);
#endif
    puts("AT32F413 Port voltage, current and gate contract passed");
    return 0;
}
