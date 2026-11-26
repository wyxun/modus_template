/**
 * @file mfoc_port.c
 * @brief MFOC port for AT-MOTOR-EVB V1.1 and AT32F413RCT7.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_port.h"
#include "mfoc_safety.h"
#if MFOC_RUNTIME_BUS
#include "mfoc_board_bus.h"
#endif
#include "mdi/instance.h"
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
#ifndef AT32F413_BUS_NOMINAL_MV
#define AT32F413_BUS_NOMINAL_MV UINT32_C(12000)
#endif
#define BUS_MIN_MV (AT32F413_BUS_NOMINAL_MV * 3U / 4U)
#define BUS_MAX_MV (AT32F413_BUS_NOMINAL_MV * 5U / 4U)

typedef struct {
    uint32_t offset_u;
    uint32_t offset_v;
    uint32_t offset_w;
    uint32_t voltage_base_millivolt;
#if MFOC_RUNTIME_BUS
    mfoc_tick_t bus_started;
    bool bus_pending;
    bool bus_primed;
#endif
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
    return MDI_TICK_Now(raw_tick);
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
#ifndef MFOC_AT32_HALL_TIMEOUT_MS
#define MFOC_AT32_HALL_TIMEOUT_MS 100U
#endif

static const mfoc_hall_sector_t s_hall_sectors[6] = MFOC_AT32_HALL_SECTORS;
static mfoc_hall_t s_hall = {0};
static mfoc_scalar_t s_hall_speed_base = MFOC_ZERO;
#define HALL_QUEUE_SIZE 8U
#define HALL_CAPTURE_FLAGS (TMR_C1_FLAG | TMR_C1_RECAPTURE_FLAG)

/* A single capture IRQ publishes slots; foreground advances read only
 * after copying both members. The ADC IRQ only consumes s_hall. */
static volatile struct {
    mfoc_tick_t tick[HALL_QUEUE_SIZE];
    uint8_t code[HALL_QUEUE_SIZE];
    uint8_t write;
    uint8_t read;
    uint32_t overruns;
} s_edges;
static volatile mfoc_result_t s_hall_status = MFOC_RESULT_INVALID_STATE;

/** @brief Read the CH1 latch before sampling all three GPIOs together. */
void TMR3_GLOBAL_IRQHandler(void)
{
    if ((TMR3->ists & TMR3->iden & TMR_C1_FLAG) == 0U) { return; }
    const uint16_t captured = (uint16_t)tmr_channel_value_get(
        TMR3, TMR_SELECT_CHANNEL_1); /* Reading C1DT clears C1IF only. */
    const uint8_t code = (uint8_t)MDI_IO_Read(hall_inputs);
    const uint32_t flags = TMR3->ists;
    const uint8_t write = s_edges.write;
    /* Bracket the Port clock read. Retry across a long ADC preemption.
     * No extra IRQ masking; entry latency must be below one TMR3 wrap. */
    if ((flags & HALL_CAPTURE_FLAGS) == 0U &&
        (uint8_t)(write - s_edges.read) < HALL_QUEUE_SIZE) {
        for (unsigned int attempt = 0; attempt < 2U; ++attempt) {
            const uint16_t before = (uint16_t)TMR3->cval_bit.cval;
            const mfoc_tick_t now = mfoc_port_tick();
            const uint16_t count = (uint16_t)TMR3->cval_bit.cval;
            if ((uint16_t)(count - before) <= 1U) {
                const uint16_t lag = (uint16_t)(count - captured);
                const uint8_t slot = write % HALL_QUEUE_SIZE;
                s_edges.tick[slot] = now - (uint32_t)lag * AT32_HALL_TICKS_PER_US;
                s_edges.code[slot] = code;
                s_edges.write = (uint8_t)(write + 1U);
                return;
            }
        }
    }
    s_hall_status = MFOC_RESULT_SAFETY;
    ++s_edges.overruns;
    tmr_interrupt_enable(TMR3, TMR_C1_INT, FALSE);
}

/** @brief Validate the measured Hall map and acquire the present sector.
 * @param base Electrical frequency in hertz.
 * @return Map validation and initial Hall level status. */
static mfoc_result_t _mfoc_port_HallSeed(mfoc_scalar_t base)
{
    mfoc_hall_cfg_t cfg = {.electrical_speed_base_hz = base,
        .timeout_ms = MFOC_AT32_HALL_TIMEOUT_MS};
    for (unsigned int i = 0U; i < 6U; ++i) {
        cfg.sectors[i] = s_hall_sectors[i];
    }
    mfoc_hall_t next = {0};
    const uint32_t tick_hz = mfoc_port_tick_frequency();
    if (tick_hz != 1000000U * AT32_HALL_TICKS_PER_US) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_result_t rc = mfoc_hall_Init(&next, &cfg, tick_hz);
    if (rc != MFOC_RESULT_OK) { return rc; }
    tmr_interrupt_enable(TMR3, TMR_C1_INT, FALSE);
    if (s_hall_status == MFOC_RESULT_SAFETY && MDI_PWM_IsEnabled(bridge)) {
        return MFOC_RESULT_SAFETY;
    }
    tmr_flag_clear(TMR3, HALL_CAPTURE_FLAGS);
    const uint8_t code = (uint8_t)MDI_IO_Read(hall_inputs);
    rc = mfoc_hall_OnEdge(&next, code, mfoc_port_tick());
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    if ((TMR3->ists & HALL_CAPTURE_FLAGS) != 0U) { rc = MFOC_RESULT_SAFETY; }
    if (rc == MFOC_RESULT_OK) {
        s_hall = next;
        s_edges.read = s_edges.write;
        tmr_interrupt_enable(TMR3, TMR_C1_INT, TRUE);
    }
    s_hall_status = rc;
    __set_PRIMASK(irq_mask);
    return rc;
}

/** @brief Publish queued transitions in capture order in the foreground. */
mfoc_result_t mfoc_port_position_run(void)
{
    /* Bound one foreground visit even if more edges arrive. */
    const uint8_t end = s_edges.write;
    while (s_hall_status == MFOC_RESULT_OK && s_edges.read != end) {
        const uint8_t read = s_edges.read;
        const uint8_t slot = read % HALL_QUEUE_SIZE;
        const uint8_t code = s_edges.code[slot];
        const mfoc_tick_t tick = s_edges.tick[slot];
        s_edges.read = (uint8_t)(read + 1U);
        mfoc_result_t rc = mfoc_hall_OnEdge(&s_hall, code, tick);
        if (rc != MFOC_RESULT_OK) {
            s_hall_status = MFOC_RESULT_SAFETY;
            tmr_interrupt_enable(TMR3, TMR_C1_INT, FALSE);
        }
    }
    return s_hall_status;
}

/** @brief Use the last published Hall capture for the control tick.
 * @param angle Electrical BAM32. @param speed Electrical speed PU.
 * @return Sample status. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    const mfoc_result_t rc = s_hall_status;
    if (rc != MFOC_RESULT_OK) { return rc; }
    return mfoc_hall_Step(&s_hall, mfoc_port_tick(), angle, speed);
}

/** @brief Verify that the measured Hall sector contains the held angle.
 * @param held_angle Known electrical BAM32.
 * @return OK when the map agrees, otherwise calibration or safety status. */
mfoc_result_t mfoc_port_position_calibrate_zero(mfoc_angle_t held_angle)
{
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    const uint8_t code = (uint8_t)MDI_IO_Read(hall_inputs);
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
    mdi_at32_adc_AckCurrentFrame();
    for (uint32_t i = 0U; i < OFFSET_SAMPLES; ++i) {
        while (!MDI_Sample_IsReady(phase_current)) {
            if (perfc_is_time_out_ms(20U, &timeout, false)) {
                return MFOC_RESULT_IO;
            }
        }
        MDI_Sample_Frame(phase_current) frame;
        MDI_Sample_Read(phase_current, &frame);
        mdi_at32_adc_AckCurrentFrame();
        sum_u += frame.u;
        sum_v += frame.v;
        sum_w += frame.w;
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
    if (MDI_PWM_IsEnabled(bridge)) { return MFOC_RESULT_INVALID_STATE; }
    /* An HEXT failure leaves the board on the incompatible HICK timebase. */
    if (!mdi_at32f413_control_clock_ready()) { return MFOC_RESULT_SAFETY; }
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
    rc = _mfoc_port_HallSeed(cfg->electrical_speed_base_hz);
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
    MDI_Sample_Frame(phase_current) frame;
    MDI_Sample_Read(phase_current, &frame);
    *current = (mfoc_uvw_t){
        _mfoc_port_ConvertCurrent((int32_t)s_port.offset_u - (int32_t)frame.u),
        _mfoc_port_ConvertCurrent((int32_t)s_port.offset_v - (int32_t)frame.v),
        _mfoc_port_ConvertCurrent((int32_t)s_port.offset_w - (int32_t)frame.w),
    };
    return MFOC_RESULT_OK;
}

/** @brief Undo the Vbus divider and ADC quantization.
 * @param raw ADC count. @return Rounded millivolts. */
static uint32_t _mfoc_port_BusMillivolt(uint32_t raw)
{
    const uint32_t full_scale_mv = ADC_REFERENCE_MV * BUS_DIVIDER_X100 / 100U;
    return (raw * full_scale_mv + ADC_SCALE / 2U) / ADC_SCALE;
}

static inline mfoc_result_t convert_bus(uint32_t code, mfoc_scalar_t *bus)
{
    if (code == 0U || code >= ADC_SCALE - 1U) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    const uint32_t mv = _mfoc_port_BusMillivolt(code);
#if MFOC_RUNTIME_BUS
    if (mv < BUS_MIN_MV || mv > BUS_MAX_MV) { return MFOC_RESULT_SAFETY; }
#endif
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

/** @brief Return a completed DC-link sample in Motor voltage units.
 * @param bus Vdc/Vbase, unchanged on failure. @return Sample status. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus)
{
    if (bus == NULL) { return MFOC_RESULT_NULL; }
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
    mdi_adc_value_t sample;
    if (MDI_ADC_Read(adc_bus_voltage, &sample) != MDI_OK) {
        return MFOC_RESULT_INVALID_STATE;
    }
    return convert_bus(sample.wCode, bus);
}

#if MFOC_RUNTIME_BUS
static void start_bus_scan(void)
{
    s_port.bus_started = mfoc_port_tick();
    s_port.bus_pending = true;
    s_port.bus_primed = true;
    (void)MDI_ADC_Start(adc_bus_voltage);
}

/* Consume FDT only once per trigger. The startup interlock and the 100 Hz
 * scheduler are serialized foreground clients of this transaction. */
static bool finish_bus_scan(mfoc_board_bus_sample_t *sample)
{
    mdi_adc_value_t raw;
    mdi_status_t status = MDI_ADC_Read(adc_bus_voltage, &raw);
    if (status == MDI_BUSY) { return false; }
    sample->started = s_port.bus_started;
    sample->status = status == MDI_OK ?
        convert_bus(raw.wCode, &sample->bus) : MFOC_RESULT_IO;
    s_port.bus_pending = false;
    return true;
}

bool mfoc_board_bus_Poll(mfoc_board_bus_sample_t *sample)
{
    if (s_port.bus_pending) { return finish_bus_scan(sample); }
    if (!s_port.bus_primed ||
        mfoc_port_tick() - s_port.bus_started >= MFOC_PERFC_HZ / 100U) {
        start_bus_scan();
    }
    return false;
}

static mfoc_result_t fresh_bus_before_start(void)
{
    int64_t deadline = 0;
    mfoc_board_bus_sample_t sample;
    if (s_port.bus_pending) {
        while (!finish_bus_scan(&sample)) {
            if (perfc_is_time_out_ms(2U, &deadline, false)) { return MFOC_RESULT_IO; }
        }
        if (sample.status != MFOC_RESULT_OK) { return mfoc_board_bus_Publish(&sample); }
    }
    start_bus_scan();
    while (!finish_bus_scan(&sample)) {
        if (perfc_is_time_out_ms(2U, &deadline, false)) { return MFOC_RESULT_IO; }
    }
    return mfoc_board_bus_Publish(&sample);
}
#endif

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
    if (!MDI_PWM_IsEnabled(bridge)) { return MFOC_RESULT_SAFETY; }
    const MDI_PWM_Frame(bridge) frame = {
        .u = _mfoc_port_Compare(duty->qU),
        .v = _mfoc_port_Compare(duty->qV),
        .w = _mfoc_port_Compare(duty->qW),
    };
    MDI_PWM_StageFast(bridge, &frame);
    return MFOC_RESULT_OK;
}

/** @brief Validate a new bus sample and apply the startup frame before MOE.
 * @param initial_duty Valid phase duties. @return Startup status. */
mfoc_result_t mfoc_port_pwm_start(const mfoc_uvw_t *initial_duty)
{
    if (!s_port.configured) { return MFOC_RESULT_INVALID_STATE; }
#if MFOC_RUNTIME_BUS
    mfoc_result_t bus_rc = fresh_bus_before_start();
    if (bus_rc != MFOC_RESULT_OK) { return bus_rc; }
#else
    (void)MDI_ADC_Start(adc_bus_voltage);
    int64_t timeout = 0;
    mdi_adc_value_t sample;
    while (MDI_ADC_Read(adc_bus_voltage, &sample) != MDI_OK) {
        if (perfc_is_time_out_ms(2U, &timeout, false)) {
            return MFOC_RESULT_IO;
        }
    }
    const uint32_t mv = _mfoc_port_BusMillivolt(sample.wCode);
    if (mv < BUS_MIN_MV || mv > BUS_MAX_MV) { return MFOC_RESULT_SAFETY; }
#endif
#if MFOC_FEEDBACK_ENABLE_POSITION
    mfoc_result_t rc = _mfoc_port_HallSeed(s_hall_speed_base);
    if (rc != MFOC_RESULT_OK) { return rc; }
#endif
    (void)MDI_PWM_SafeStop(bridge);
    if (!mdi_at32f413_control_clock_ready() ||
        MDI_PWM_ClearFault(bridge) != MDI_OK) {
        return MFOC_RESULT_SAFETY;
    }
    /* The foreground owns the next ADC completion during synchronization. */
    mdi_at32_adc_DisableControlIRQ();
    const MDI_PWM_Frame(bridge) frame = {
        .u = _mfoc_port_Compare(initial_duty->qU),
        .v = _mfoc_port_Compare(initial_duty->qV),
        .w = _mfoc_port_Compare(initial_duty->qW),
    };
    MDI_PWM_StageFast(bridge, &frame);
#if MFOC_RUNTIME_BUS
    bus_rc = mfoc_board_bus_Arm(mfoc_port_tick());
    if (bus_rc != MFOC_RESULT_OK) { return bus_rc; }
#endif
    mfoc_result_t ready = mfoc_at32_safety_WaitStart();
    if (ready != MFOC_RESULT_OK) { return ready; }

    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (mfoc_at32_safety_StartWindowOpen()) {
        mfoc_at32_safety_Arm(s_port.offset_u, s_port.offset_v, s_port.offset_w);
        if (MDI_PWM_Enable(bridge, true) == MDI_OK) {
            mdi_at32_adc_EnableControlIRQ();
            __set_PRIMASK(mask);
            return MFOC_RESULT_OK;
        }
        mfoc_at32_safety_Disarm();
    }
    __set_PRIMASK(mask);
    return MFOC_RESULT_SAFETY;
}

/** @brief Turn off phase outputs and keep ADC synchronization alive.
 * @return OK after the gate request. */
mfoc_result_t mfoc_port_pwm_stop(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    (void)MDI_PWM_SafeStop(bridge);
    mfoc_at32_safety_Disarm();
    mdi_at32_adc_DisableControlIRQ();
    __set_PRIMASK(mask);
    return MFOC_RESULT_OK;
}
