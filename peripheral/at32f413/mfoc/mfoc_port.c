/**
 * @file mfoc_port.c
 * @brief MFOC port for AT-MOTOR-EVB V1.1 and AT32F413RCT7.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_port.h"
#include "at32f413.h"
#include "haladc.h"
#include "halpwm.h"
#include "perf_counter.h"
#if MFOC_FEEDBACK_ENABLE_POSITION
#include "mfoc_at32_hall_config.h"
#include "mfoc_hall.h"
#endif
#include <limits.h>
#include <stddef.h>

#define ADC_SCALE UINT32_C(4096)
#define OFFSET_SAMPLES UINT32_C(8)
#define PHASE_MV_PER_AMP UINT32_C(33)
#define ADC_REFERENCE_MV UINT32_C(3300)
/* (180k + 3.9k + 10k) / 10k on the 19.39x DC-link divider. */
#define BUS_DIVIDER_X100 UINT32_C(1939)
#define BUS_MIN_MV UINT32_C(9000)
#define BUS_MAX_MV UINT32_C(15000)

typedef struct {
    uint32_t offset_u;
    uint32_t offset_v;
    uint32_t offset_w;
    uint32_t voltage_base_millivolt;
#if defined(MFOC_NUMERIC_FIXED)
    int32_t gain_q30;
#else
    mfoc_scalar_t gain;
#endif
    bool configured;
} at32_mfoc_port_t;

static at32_mfoc_port_t s_port = {0};

/** @brief Read board time for Port users. @return Full perf-counter ticks. */
mfoc_tick_t mfoc_port_tick(void)
{
    return (mfoc_tick_t)get_system_ticks();
}

/** @brief Read the initialized Port clock rate. @return Ticks per second. */
uint32_t mfoc_port_tick_frequency(void)
{
    return perfc_get_systimer_frequency();
}

#if MFOC_FEEDBACK_ENABLE_POSITION
#ifndef MFOC_AT32_HALL_SECTORS
#error "Provide the measured MFOC_AT32_HALL_SECTORS table for this motor"
#endif

static const mfoc_hall_sector_t s_hall_sectors[6] = MFOC_AT32_HALL_SECTORS;
static mfoc_hall_t s_hall = {0};
static mfoc_scalar_t s_hall_speed_base = MFOC_ZERO;
static uint8_t s_hall_code = UINT8_MAX;

/** @brief Read the three J1 Hall inputs in one GPIO snapshot.
 * @return Raw code with H1 in bit zero, H2 in bit one and H3 in bit two. */
static uint8_t _mfoc_port_HallCode(void)
{
    const uint32_t levels = GPIOB->idt;
    return (uint8_t)(((levels >> 4U) & 3U) | ((levels & 1U) << 2U));
}

/** @brief Validate the measured Hall map and acquire the present sector.
 * @param base Electrical frequency in hertz.
 * @return Map validation and initial Hall level status. */
static mfoc_result_t _mfoc_port_HallSeed(mfoc_scalar_t base)
{
    mfoc_hall_cfg_t cfg = {.electrical_speed_base_hz = base,
        .timeout_ms = 100U};
    for (unsigned int i = 0U; i < 6U; ++i) {
        cfg.sectors[i] = s_hall_sectors[i];
    }
    mfoc_hall_t next = {0};
    mfoc_result_t rc = mfoc_hall_Init(&next, &cfg,
        mfoc_port_tick_frequency());
    if (rc != MFOC_RESULT_OK) { return rc; }
    uint8_t code = _mfoc_port_HallCode();
    rc = mfoc_hall_OnEdge(&next, code, mfoc_port_tick());
    if (rc != MFOC_RESULT_OK) { return rc; }
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    s_hall = next;
    s_hall_code = code;
    __set_PRIMASK(irq_mask);
    return MFOC_RESULT_OK;
}

/** @brief Configure the J1 Hall inputs; use the board's external pulls.
 * @param base Electrical frequency in hertz.
 * @return Hall map and initial acquisition status. */
static mfoc_result_t _mfoc_port_HallInit(mfoc_scalar_t base)
{
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
    gpio_init_type pins = {0};
    gpio_default_para_init(&pins);
    pins.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_0;
    pins.gpio_mode = GPIO_MODE_INPUT;
    pins.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &pins);
    return _mfoc_port_HallSeed(base);
}

/** @brief Publish a new Hall sector when the foreground sees a change.
 * @return Transition status. The foreground must service every sector. */
mfoc_result_t mfoc_port_position_run(void)
{
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    const uint8_t code = _mfoc_port_HallCode();
    if (code == s_hall_code) { return MFOC_RESULT_OK; }
    s_hall_code = code;
    return mfoc_hall_OnEdge(&s_hall, code, mfoc_port_tick());
}

/** @brief Use the last published Hall capture for the control tick.
 * @param angle Electrical BAM32. @param speed Electrical speed PU.
 * @return Sample status. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    return mfoc_hall_Step(&s_hall, mfoc_port_tick(), angle, speed);
}

/** @brief Verify that the measured Hall sector contains the held angle.
 * @param held_angle Known electrical BAM32.
 * @return OK when the map agrees, otherwise calibration or safety status. */
mfoc_result_t mfoc_port_position_calibrate_zero(mfoc_angle_t held_angle)
{
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    const uint8_t code = _mfoc_port_HallCode();
    for (unsigned int i = 0U; i < 6U; ++i) {
        if (s_hall_sectors[i].code != code) { continue; }
        const unsigned int next = (i + 1U) % 6U;
        const uint32_t start = s_hall_sectors[i].start.wBam32;
        const uint32_t width = s_hall_sectors[next].start.wBam32 - start;
        if (held_angle.wBam32 - start < width) {
            return _mfoc_port_HallSeed(s_hall_speed_base);
        }
        break;
    }
    return MFOC_RESULT_SAFETY;
}
#endif

/** @brief Average fresh samples while the phase gates are closed.
 * @param port Candidate instance to receive three calibration offsets.
 * @return OK, IO on timeout or SAFETY for an implausible midpoint. */
static mfoc_result_t _mfoc_port_Calibrate(at32_mfoc_port_t *port)
{
    uint32_t sum_u = 0U;
    uint32_t sum_v = 0U;
    uint32_t sum_w = 0U;
    int64_t timeout = 0;
    haladc_DiscardPreempt();
    for (uint32_t i = 0U; i < OFFSET_SAMPLES; ++i) {
        while (!haladc_PreemptReady()) {
            if (perfc_is_time_out_ms(20U, &timeout, false)) {
                return MFOC_RESULT_IO;
            }
        }
        uint16_t u = 0U;
        uint16_t v = 0U;
        uint16_t w = 0U;
        haladc_GetPreemptRaw(&u, &v, &w);
        haladc_DiscardPreempt();
        sum_u += u;
        sum_v += v;
        sum_w += w;
    }
    port->offset_u = (sum_u + OFFSET_SAMPLES / 2U) / OFFSET_SAMPLES;
    port->offset_v = (sum_v + OFFSET_SAMPLES / 2U) / OFFSET_SAMPLES;
    port->offset_w = (sum_w + OFFSET_SAMPLES / 2U) / OFFSET_SAMPLES;
    const uint32_t min = ADC_SCALE / 4U;
    const uint32_t max = ADC_SCALE * 3U / 4U;
    if (port->offset_u < min || port->offset_u > max ||
        port->offset_v < min || port->offset_v > max ||
        port->offset_w < min || port->offset_w > max) {
        return MFOC_RESULT_SAFETY;
    }
    return MFOC_RESULT_OK;
}

/** @brief Configure the EVB conversion gains and calibrate current zero.
 * @param cfg Motor normalization bases.
 * @return Configuration or calibration status.
 * @note Phase gates and the ADC vector remain disabled during calibration. */
mfoc_result_t mfoc_port_init(const mfoc_port_cfg_t *cfg)
{
    if (cfg == NULL) { return MFOC_RESULT_NULL; }
    if (cfg->current_base_milliamp == 0U ||
        cfg->voltage_base_millivolt == 0U) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    if (halpwm_OutputsEnabled()) { return MFOC_RESULT_INVALID_STATE; }
    /* An HEXT failure leaves the board on the incompatible HICK timebase. */
    if (SystemCoreClock != 200000000U) { return MFOC_RESULT_SAFETY; }
    at32_mfoc_port_t next = {
        .voltage_base_millivolt = cfg->voltage_base_millivolt,
    };
    /* 2 mΩ shunt x 16.5 gain gives 33 mV/A. Direction changes at the
     * subtractor because the low-side shunt senses bridge-to-ground. */
#if defined(MFOC_NUMERIC_FIXED)
    const uint64_t denominator = (uint64_t)PHASE_MV_PER_AMP * ADC_SCALE *
        cfg->current_base_milliamp;
    const uint64_t gain = ((UINT64_C(1) << 30U) * ADC_REFERENCE_MV *
        1000U + denominator / 2U) / denominator;
    if (gain == 0U || gain > INT32_MAX) { return MFOC_RESULT_OUT_OF_RANGE; }
    next.gain_q30 = (int32_t)gain;
#else
    next.gain = ((float)ADC_REFERENCE_MV * 1000.0f) /
        ((float)(PHASE_MV_PER_AMP * ADC_SCALE) *
         (float)cfg->current_base_milliamp);
#endif
    mfoc_result_t rc = _mfoc_port_Calibrate(&next);
    if (rc != MFOC_RESULT_OK) { return rc; }
#if MFOC_FEEDBACK_ENABLE_POSITION
    rc = _mfoc_port_HallInit(cfg->electrical_speed_base_hz);
    if (rc != MFOC_RESULT_OK) { return rc; }
    s_hall_speed_base = cfg->electrical_speed_base_hz;
#endif
    next.configured = true;
    s_port = next;
    return MFOC_RESULT_OK;
}

/** @brief Convert one low-side shunt count delta to motor phase current.
 * @param delta Signed calibrated offset minus ADC reading.
 * @return I/Ibase in the selected numeric backend. */
static inline mfoc_scalar_t _mfoc_port_ConvertCurrent(int32_t delta)
{
#if defined(MFOC_NUMERIC_FIXED)
    return (mfoc_scalar_t)(delta * (int64_t)s_port.gain_q30 /
        INT64_C(32768));
#else
    return (mfoc_scalar_t)delta * s_port.gain;
#endif
}

/** @brief Read the coherent frame made available by the bound ADC vector.
 * @param current I/Ibase phase currents. @return Status.
 * @note The ADC vector owns completion checking and acknowledgement. */
mfoc_result_t mfoc_port_current(mfoc_uvw_t *current)
{
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    uint16_t raw_u = 0U;
    uint16_t raw_v = 0U;
    uint16_t raw_w = 0U;
    haladc_GetPreemptRaw(&raw_u, &raw_v, &raw_w);
    *current = (mfoc_uvw_t){
        _mfoc_port_ConvertCurrent((int32_t)s_port.offset_u - (int32_t)raw_u),
        _mfoc_port_ConvertCurrent((int32_t)s_port.offset_v - (int32_t)raw_v),
        _mfoc_port_ConvertCurrent((int32_t)s_port.offset_w - (int32_t)raw_w),
    };
    return MFOC_RESULT_OK;
}

/** @brief Undo the Vbus divider and ADC quantization.
 * @param raw ADC count. @return Rounded millivolts. */
static uint32_t _mfoc_port_BusMillivolt(uint16_t raw)
{
    const uint32_t full_scale_mv = ADC_REFERENCE_MV * BUS_DIVIDER_X100 / 100U;
    return ((uint32_t)raw * full_scale_mv + ADC_SCALE / 2U) / ADC_SCALE;
}

/** @brief Return a completed DC-link sample in Motor voltage units.
 * @param bus Vdc/Vbase, unchanged on failure. @return Sample status. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus)
{
    if (bus == NULL) { return MFOC_RESULT_NULL; }
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    uint16_t raw = 0U;
    if (!haladc_ReadBusRaw(&raw)) { return MFOC_RESULT_INVALID_STATE; }
    if (raw == 0U || raw >= ADC_SCALE - 1U) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    const uint32_t mv = _mfoc_port_BusMillivolt(raw);
#if defined(MFOC_NUMERIC_FIXED)
    const uint64_t base = s_port.voltage_base_millivolt;
    const uint64_t pu = ((uint64_t)mv * MFOC_Q_SCALE + base / 2U) / base;
    if (pu == 0U || pu > INT32_MAX) { return MFOC_RESULT_OUT_OF_RANGE; }
    *bus = (mfoc_scalar_t)pu;
#else
    *bus = (mfoc_scalar_t)mv / (mfoc_scalar_t)s_port.voltage_base_millivolt;
#endif
    return MFOC_RESULT_OK;
}

/** @brief Scale one trusted phase duty into TMR1 compare counts.
 * @param duty Normalized 0..1 duty. @return Rounded compare count. */
static inline uint32_t _mfoc_port_Compare(mfoc_scalar_t duty)
{
#if defined(MFOC_NUMERIC_FIXED)
    return ((uint32_t)duty * PWM_PERIOD + MFOC_Q_SCALE / 2U) >>
        MFOC_Q_FRACTION_BITS;
#else
    return (uint32_t)(duty * (mfoc_scalar_t)PWM_PERIOD + 0.5f);
#endif
}

/** @brief Submit three coherent compare preloads without changing the gates.
 * @param duty Valid phase duties. @return OK or gate fault. */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty)
{
    if (!halpwm_OutputsEnabled()) { return MFOC_RESULT_SAFETY; }
    halpwm_SetDuty(_mfoc_port_Compare(duty->qU),
                  _mfoc_port_Compare(duty->qV),
                  _mfoc_port_Compare(duty->qW));
    return MFOC_RESULT_OK;
}

/** @brief Validate a new bus sample and apply the startup frame before MOE.
 * @param initial_duty Valid phase duties. @return Startup status. */
mfoc_result_t mfoc_port_pwm_start(const mfoc_uvw_t *initial_duty)
{
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    haladc_RequestBus();
    int64_t timeout = 0;
    uint16_t raw = 0U;
    while (!haladc_ReadBusRaw(&raw)) {
        if (perfc_is_time_out_ms(2U, &timeout, false)) {
            return MFOC_RESULT_IO;
        }
    }
    uint32_t mv = _mfoc_port_BusMillivolt(raw);
    if (mv < BUS_MIN_MV || mv > BUS_MAX_MV) { return MFOC_RESULT_SAFETY; }
#if MFOC_FEEDBACK_ENABLE_POSITION
    mfoc_result_t rc = _mfoc_port_HallSeed(s_hall_speed_base);
    if (rc != MFOC_RESULT_OK) { return rc; }
#endif
    if (!halpwm_Arm(_mfoc_port_Compare(initial_duty->qU),
                    _mfoc_port_Compare(initial_duty->qV),
                    _mfoc_port_Compare(initial_duty->qW))) {
        return MFOC_RESULT_SAFETY;
    }
    haladc_EnableControlIRQ();
    return MFOC_RESULT_OK;
}

/** @brief Turn off phase outputs and keep ADC synchronization alive.
 * @return OK after the gate request. */
mfoc_result_t mfoc_port_pwm_stop(void)
{
    halpwm_Stop();
    return MFOC_RESULT_OK;
}
