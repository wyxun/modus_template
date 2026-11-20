/**
 * @file port_test.c
 * @brief Exercise the AT-MOTOR-EVB V1.1 Port against simulated HAL samples.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_port.h"
#include "at32f413.h"
#include "haladc.h"
#include "halpwm.h"
#include "perf_counter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

uint32_t SystemCoreClock = 200000000U;
test_adc_t test_adc;
test_timer_t test_timer;
test_gpio_t test_gpio = {.idt = GPIO_PINS_4};
#define s_gate (test_timer.brk_bit.oen)
static int64_t s_ticks;
static bool s_frame_ready;
static bool s_bus_ready;
static bool s_break;
static bool s_irq;
static uint16_t s_adc[3] = {2048U, 2052U, 2044U};
static uint16_t s_bus_adc = 768U;
static uint32_t s_compare[3];
static unsigned s_adc_reads;
static unsigned s_bus_requests;

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

void haladc_GetPreemptRaw(uint16_t *u, uint16_t *v, uint16_t *w)
{
    *u = s_adc[0];
    *v = s_adc[1];
    *w = s_adc[2];
    ++s_adc_reads;
}

void haladc_EnableControlIRQ(void)
{
    s_irq = true;
}

void haladc_RequestBus(void)
{
    ++s_bus_requests;
}

bool haladc_ReadBusRaw(uint16_t *raw)
{
    if (!s_bus_ready) { return false; }
    *raw = s_bus_adc;
    return true;
}

void halpwm_SetDuty(uint32_t u, uint32_t v, uint32_t w)
{
    s_compare[0] = u;
    s_compare[1] = v;
    s_compare[2] = w;
}

bool halpwm_Arm(uint32_t u, uint32_t v, uint32_t w)
{
    if (s_break) { return false; }
    assert(!s_gate);
    halpwm_SetDuty(u, v, w);
    s_gate = true;
    return true;
}

void halpwm_Stop(void)
{
    s_gate = false;
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
    assert(!s_gate && !s_irq && s_adc_reads == 0U);
    SystemCoreClock = 200000000U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_IO);
    assert(!s_gate && !s_irq && s_adc_reads == 0U);
    s_frame_ready = true;
    s_adc[0] = 0U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_SAFETY);
    s_adc[0] = 2048U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_OK);
    assert(!s_gate && !s_irq && s_adc_reads > 0U);
    assert(mfoc_port_tick_frequency() == 200000000U);
    const mfoc_tick_t before = mfoc_port_tick();
    assert(mfoc_port_tick() > before);

    const unsigned reads = s_adc_reads;
    s_adc[0] = 2008U;
    s_adc[1] = 2092U;
    s_adc[2] = 2044U;
    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
    assert(s_adc_reads == reads + 1U);
    const float sample_pu = 40.0f / (40.96f * 3.5f);
    assert(fabsf(mfoc_to_float(current.qU) - sample_pu) < 0.0001f);
    assert(fabsf(mfoc_to_float(current.qV) + sample_pu) < 0.0001f);
    assert(current.qW == MFOC_ZERO);

    mfoc_scalar_t bus = MFOC_ZERO;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_INVALID_STATE);
    assert(bus == MFOC_ZERO);
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_IO);
    assert(!s_gate && !s_irq && s_bus_requests == 1U);
    s_bus_ready = true;
    s_bus_adc = 100U;
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY);
    assert(!s_gate && !s_irq);
    s_bus_adc = 768U;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(bus) - 1.0f) < 0.005f);

    s_break = true;
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY);
    assert(!s_gate && !s_irq);
    s_break = false;
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_OK);
    assert(s_gate && s_irq);
    assert(s_compare[0] == PWM_PERIOD / 4U);
    assert(s_compare[1] == PWM_PERIOD / 2U);
    assert(s_compare[2] == 3U * PWM_PERIOD / 4U);
    const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
    assert(mfoc_port_pwm(&neutral) == MFOC_RESULT_OK);
    assert(s_gate && s_compare[2] == PWM_PERIOD / 2U);
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
    test_gpio.idt = GPIO_PINS_4 | GPIO_PINS_5;
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    s_ticks += 400000U;
    test_gpio.idt = GPIO_PINS_5;
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(speed > MFOC_ZERO);
    test_gpio.idt = 0U;
    assert(mfoc_port_position_run() == MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_start(&duty) == MFOC_RESULT_SAFETY && !s_gate);
#endif
    puts("AT32F413 Port voltage, current and gate contract passed");
    return 0;
}
