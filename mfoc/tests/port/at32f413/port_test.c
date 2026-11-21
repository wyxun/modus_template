/**
 * @file port_test.c
 * @brief Exercise the AT-MOTOR-EVB V1.1 Port against simulated registers.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_port.h"
#include "mdi/instance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

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

int64_t get_system_ticks(void)
{
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

/** @brief Verify configuration, signed ADC scaling and gate interlocks.
 * @return Zero on success.
 */
int main(void)
{
    const mfoc_port_cfg_t cfg = {
        .current_base_milliamp = 3500U,
        .voltage_base_millivolt = 12000U,
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
    g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE] = 768U;
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
    assert(test_timer.c1dt == PWM_PERIOD / 4U);
    assert(test_timer.c2dt == PWM_PERIOD / 2U);
    assert(test_timer.c3dt == 3U * PWM_PERIOD / 4U);
    const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
    assert(mfoc_port_pwm(&neutral) == MFOC_RESULT_OK);
    assert(s_gate && test_timer.c3dt == PWM_PERIOD / 2U);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(!s_gate && s_irq);
    assert(mfoc_port_pwm(&duty) == MFOC_RESULT_SAFETY);
#if MFOC_FEEDBACK_ENABLE_POSITION
    mfoc_angle_t angle = {0U};
    mfoc_scalar_t speed = MFOC_ONE;
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == UINT32_C(0x15555555) && speed == MFOC_ZERO);
    assert(mfoc_port_position_calibrate_zero((mfoc_angle_t){0U}) ==
        MFOC_RESULT_OK);
    assert(mfoc_port_position_calibrate_zero((mfoc_angle_t){0x80000000U}) ==
        MFOC_RESULT_SAFETY);
    s_ticks += 400000U;
    test_gpio.idt = GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_12;
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    s_ticks += 400000U;
    test_gpio.idt = GPIO_PINS_5 | GPIO_PINS_12;
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(speed > MFOC_ZERO);
    test_gpio.idt = GPIO_PINS_12;
    assert(mfoc_port_position_run() == MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY && !s_gate);
#endif
    puts("AT32F413 Port voltage, current and gate contract passed");
    return 0;
}
