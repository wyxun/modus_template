/**
 * @file mfoc_p10_physical.c
 * @brief B-top PWM electrical fingerprints on the G431 bench.
 * @author Codex
 * @date 2026-11-22
 */
#include "mfoc_p10_physical.h"
#include "mfoc_p10_deadline.h"
#include "mfoc/mfoc_pwm_phase.h"
#include "mfoc_p10_timing.h"
#include "mfoc_app.h"
#include "mfoc_modulation.h"
#include "mfoc_port.h"
#include "mdebug/mshell.h"
#include "mdi/instance.h"
#include <math.h>
#include <string.h>

#define P10_PHY_MAGIC UINT32_C(0x50313042)
#define P10_PHY_PULSE_ROWS 8U
#define P10_PHY_PULSE_INDEX 2U
#define P10_PHY_BINS 32U
#define P10_PHY_I_MAX 0.12f
/* Bench for 12 V/2804. Increase only with a new electrical limit review. */
#define P10_PHY_PULSE_MAX_MPU 60
#define P10_PHY_SWEEP_MAX_MPU 100U
#define P10_PHY_MIN_HZ 2U
#define P10_PHY_MAX_HZ 20U
#define P10_PHY_TIMEOUT_TICKS (MFOC_PERFC_HZ / 5000U)
/* A fresh, ordered write is also required before the next preload event. */
#define P10_PHY_WRITE_GUARD 64U
/* Moduli are used for timestamps. This one bounds the latest ADC entry. */
#define P10_PHY_MAX_ENTRY_TICKS (MFOC_PERFC_HZ / 50000U)

enum {
    PHY_IDLE, PHY_RUNNING, PHY_DONE,
    PHY_OVERCURRENT = 10, PHY_SAMPLE, PHY_PHASE, PHY_PWM,
    PHY_ABORT, PHY_BREAK, PHY_TIMEOUT
};
enum { PHY_PULSE = 1, PHY_SWEEP };

typedef struct {
    uint32_t n, seq, trigger, capture, entry, write, angle_after;
    uint32_t raw[3];
    float current[3], residual;
    uint32_t before[3], after[3], command[3];
    uint32_t max_v_ccr;
    float sum_residual_sq;
} p10_physical_row_t;

typedef struct {
    uint32_t magic;
    volatile uint32_t status;
    uint32_t mode;
    int32_t mpu;
    uint32_t hz, period_frames, frame_count, last_trigger;
    float max_current, bus;
    uint32_t arr, record_count;
    p10_physical_row_t rows[P10_PHY_BINS];
} p10_physical_data_t;

_Static_assert(sizeof(p10_physical_row_t) == 25U * sizeof(uint32_t),
    "Physical evidence row must be a fixed DWORD layout");
_Static_assert(offsetof(p10_physical_data_t, rows) == 12U * sizeof(uint32_t),
    "Physical evidence header must be a fixed DWORD layout");

/* Read only after terminal status and closed gates. No live RTT in the IRQ. */
p10_physical_data_t g_p10_physical;

static const mfoc_uvw_t s_neutral = {0.5f, 0.5f, 0.5f};
static mfoc_uvw_t s_pulse;
static uint32_t s_ccr_before[3], s_ccr_after[3];
static uint32_t s_angle_after, s_angle_step, s_ccr4;
static float s_modulation;
extern mfoc_app_t tMfocApp;

/** @brief Keep CH1's phase capture untouched while scheduling the watchdog. */
static void p10_PhysicalWatchdog(void)
{
    TIM2->CCR2 = TIM2->CNT + P10_PHY_TIMEOUT_TICKS;
    TIM2->SR = ~TIM_SR_CC2IF;
}

/** @brief Close phase gates before disarming the timeout or publishing data.
 * @param status Terminal result visible to the host; caller excludes ADC IRQ
 * when not already in it. */
static void p10_PhysicalStop(uint32_t status)
{
    mfoc_result_t rc = mfoc_port_pwm_stop();
    TIM2->DIER &= ~TIM_DIER_CC2IE;
    TIM2->SR = ~TIM_SR_CC2IF;
    NVIC_DisableIRQ(TIM2_IRQn);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    __DMB();
    g_p10_physical.status = rc == MFOC_RESULT_OK ? status : PHY_PWM;
}

/** @brief Shut the bridge if the ADC path stops taking valid samples. */
void TIM2_IRQHandler(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if ((TIM2->SR & TIM_SR_CC2IF) != 0U &&
        g_p10_physical.status == PHY_RUNNING) {
        p10_PhysicalStop(PHY_TIMEOUT);
    }
    __set_PRIMASK(primask);
}

/** @brief Read the CCR preload contents corresponding to a submitted duty.
 * @param ccr Three compare counts in the U/V/W order. */
static void p10_PhysicalDuty(uint32_t ccr[3])
{
    ccr[0] = TIM1->CCR1;
    ccr[1] = TIM1->CCR2;
    ccr[2] = TIM1->CCR3;
}

/** @brief Update the max-residual sample and RMS accumulators for its bin.
 * @param row Complete frame; pulse records retain their sequential indices. */
static void p10_PhysicalRecord(const p10_physical_row_t *row)
{
    p10_physical_data_t *data = &g_p10_physical;
    if (data->mode == PHY_PULSE) {
        data->rows[row->seq] = *row;
        return;
    }
    /* Sample entered after the top. ADC2 Rank 2 is on the after side per
     * the trigger model. Keep both CCR sets for later interpretation. */
    if (row->seq <= data->period_frames ||
        row->seq > 2U * data->period_frames) { return; }
    p10_physical_row_t *bin = &data->rows[row->angle_after >> 27U];
    uint32_t n = bin->n + 1U;
    uint32_t peak = bin->max_v_ccr;
    float sum = bin->sum_residual_sq + row->sum_residual_sq;
    if (n == 1U || fabsf(row->residual) > fabsf(bin->residual)) {
        *bin = *row;
    }
    bin->n = n;
    bin->max_v_ccr = row->after[1] > peak ? row->after[1] : peak;
    bin->sum_residual_sq = sum;
}

bool mfoc_p10_PhysicalStep(void)
{
    p10_physical_data_t *data = &g_p10_physical;
    if (data->status != PHY_RUNNING) { return false; }
    if (mdi_g431_fault_active() || (TIM1->BDTR & TIM_BDTR_MOE) == 0U ||
        (TIM1->CCER & MDI_G431_PWM_PHASE_MASK) != MDI_G431_PWM_PHASE_MASK) {
        p10_PhysicalStop(PHY_BREAK);
        return true;
    }
    uint32_t adc1 = ADC1->ISR;
    uint32_t adc2 = ADC2->ISR;
    if ((adc2 & ADC_ISR_JEOS) == 0U ||
        ((adc1 | adc2) & (ADC_ISR_OVR | ADC_ISR_JQOVF)) != 0U) {
        p10_PhysicalStop(PHY_SAMPLE);
        return true;
    }
    p10_physical_row_t row = {
        .n = 1U, .seq = data->frame_count, .capture = TIM2->CCR1,
        .entry = g_p10_irq_entry, .angle_after = s_angle_after,
        .raw = {ADC1->JDR1 & 0x3fffU, ADC2->JDR2 & 0x3fffU,
                ADC2->JDR1 & 0x3fffU}
    };
    uint32_t deadline = 0U;
    const uint32_t period = 2U * data->arr;
    if (!mfoc_p10_frame_times(2U, data->arr, s_ccr4, row.capture,
            row.entry, &row.trigger, &deadline) ||
        row.entry - row.capture > P10_PHY_MAX_ENTRY_TICKS ||
        (row.seq != 0U && row.trigger - data->last_trigger != period)) {
        p10_PhysicalStop(PHY_PHASE);
        return true;
    }
    mfoc_uvw_t current = {0};
    if (mfoc_port_current(&current) != MFOC_RESULT_OK) {
        p10_PhysicalStop(PHY_SAMPLE);
        return true;
    }
    row.current[0] = current.qU;
    row.current[1] = current.qV;
    row.current[2] = current.qW;
    row.residual = current.qU + current.qV + current.qW;
    row.sum_residual_sq = row.residual * row.residual;
    memcpy(row.before, s_ccr_before, sizeof row.before);
    memcpy(row.after, s_ccr_after, sizeof row.after);
    row.max_v_ccr = row.after[1];
    float peak = fmaxf(fabsf(current.qU), fabsf(current.qV));
    peak = fmaxf(peak, fabsf(current.qW));
    if (peak > data->max_current) { data->max_current = peak; }
    data->frame_count = row.seq + 1U;
    data->last_trigger = row.trigger;
    if (peak > P10_PHY_I_MAX) {
        p10_PhysicalRecord(&row);
        p10_PhysicalStop(PHY_OVERCURRENT);
        return true;
    }
    mfoc_uvw_t duty = s_neutral;
    uint32_t next_angle = 0U;
    if (data->mode == PHY_PULSE) {
        if (row.seq == P10_PHY_PULSE_INDEX) { duty = s_pulse; }
    } else if (row.seq < 2U * data->period_frames) {
        next_angle = (row.seq % data->period_frames) * s_angle_step;
        mfoc_ab_t vector = {0};
        mfoc_angle_sincos((mfoc_angle_t){next_angle}, &vector.qBeta,
                          &vector.qAlpha);
        vector.qAlpha *= s_modulation;
        vector.qBeta *= s_modulation;
        mfoc_svpwm(&vector, &duty);
    } else {
        /* Flush the last in-flight duty before closing the bridge. */
    }
    if ((int32_t)(deadline - TIM2->CNT) <= (int32_t)P10_PHY_WRITE_GUARD ||
        mfoc_port_pwm(&duty) != MFOC_RESULT_OK) {
        p10_PhysicalStop(PHY_PWM);
        return true;
    }
    __DSB();
    row.write = TIM2->CNT;
    if ((int32_t)(deadline - row.write) <= 0 ||
        TIM2->CCR1 != row.capture) {
        p10_PhysicalStop(PHY_PHASE);
        return true;
    }
    p10_PhysicalDuty(row.command);
    memcpy(s_ccr_before, s_ccr_after, sizeof s_ccr_before);
    memcpy(s_ccr_after, row.command, sizeof s_ccr_after);
    s_angle_after = next_angle;
    p10_PhysicalRecord(&row);
    if ((data->mode == PHY_PULSE && data->frame_count == P10_PHY_PULSE_ROWS) ||
        (data->mode == PHY_SWEEP && row.seq == 2U * data->period_frames + 1U)) {
        p10_PhysicalStop(PHY_DONE);
    } else {
        p10_PhysicalWatchdog();
    }
    return true;
}

/** @brief Calibrate the command constants and hand the closed bridge to ADC.
 * @param mode Pulse or one captured sweep turn after a warmup turn.
 * @param mpu Modulation command in milli-PU. @param hz Sweep electrical Hz.
 * @return Invalid state, bench voltage fault or Port startup result. */
static mfoc_result_t p10_PhysicalStart(uint32_t mode, int32_t mpu, uint32_t hz)
{
    p10_physical_data_t *data = &g_p10_physical;
    if (data->status == PHY_RUNNING ||
        tMfocApp.motor.state != MFOC_MOTOR_IDLE ||
        (TIM1->CCER & MDI_G431_PWM_PHASE_MASK) != 0U ||
        !g_mfoc_g431_pwm_phase.locked) {
        return MFOC_RESULT_INVALID_STATE;
    }
    float bus = 0.0f;
    if (mfoc_port_bus(&bus) != MFOC_RESULT_OK || bus < 0.1f || bus > 1.1f) {
        return MFOC_RESULT_SAFETY;
    }
    memset(data, 0, sizeof *data);
    data->magic = P10_PHY_MAGIC;
    data->mode = mode;
    data->mpu = mpu;
    data->hz = hz;
    data->bus = bus;
    data->arr = TIM1->ARR;
    s_ccr4 = TIM1->CCR4;
    s_angle_after = 0U;
    s_modulation = (float)mpu * 0.001f;
    if (mode == PHY_SWEEP) {
        data->record_count = P10_PHY_BINS;
        data->period_frames = (MFOC_MOTOR_RUN_FREQ_HZ + hz - 1U) / hz;
        s_angle_step = (uint32_t)(UINT64_C(0x100000000) / data->period_frames);
    } else {
        data->record_count = P10_PHY_PULSE_ROWS;
        const mfoc_ab_t vector = {.qAlpha = s_modulation};
        mfoc_svpwm(&vector, &s_pulse);
    }
    for (unsigned i = 0U; i < 3U; ++i) {
        s_ccr_before[i] = data->arr / 2U;
        s_ccr_after[i] = s_ccr_before[i];
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    p10_PhysicalWatchdog();
    NVIC_SetPriority(TIM2_IRQn, 2U);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    TIM2->DIER |= TIM_DIER_CC2IE;
    NVIC_EnableIRQ(TIM2_IRQn);
    __DMB();
    data->status = PHY_RUNNING;
    mfoc_result_t rc = mfoc_port_pwm_start(&s_neutral);
    if (rc == MFOC_RESULT_OK) {
        p10_PhysicalDuty(s_ccr_after);
        memcpy(s_ccr_before, s_ccr_after, sizeof s_ccr_before);
    } else {
        p10_PhysicalStop(PHY_PWM);
    }
    __set_PRIMASK(primask);
    return rc;
}

/** @brief Control a bounded phase experiment through the foreground shell.
 * @param args Command arguments. */
static void p10_PhysicalCommand(const char *args)
{
    int32_t mpu = 0;
    uint32_t hz = 0U;
    mfoc_result_t rc = MFOC_RESULT_INVALID_ARGUMENT;
    if (mshell_ConsumeWord(&args, "stop") && mshell_AtEnd(args)) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (g_p10_physical.status == PHY_RUNNING) {
            p10_PhysicalStop(PHY_ABORT);
        }
        __set_PRIMASK(primask);
        rc = MFOC_RESULT_OK;
    } else if (mshell_ConsumeWord(&args, "pulse") &&
        mshell_ParseInt32(&args, &mpu) == MSHELL_PARSE_OK &&
        mshell_AtEnd(args) && mpu >= -P10_PHY_PULSE_MAX_MPU &&
        mpu <= P10_PHY_PULSE_MAX_MPU) {
        rc = p10_PhysicalStart(PHY_PULSE, mpu, 0U);
    } else if (mshell_ConsumeWord(&args, "sweep") &&
        mshell_ParseInt32(&args, &mpu) == MSHELL_PARSE_OK && mpu > 0 &&
        (uint32_t)mpu <= P10_PHY_SWEEP_MAX_MPU &&
        mshell_ParseUint32(&args, &hz) == MSHELL_PARSE_OK && mshell_AtEnd(args)
        && hz >= P10_PHY_MIN_HZ && hz <= P10_PHY_MAX_HZ) {
        rc = p10_PhysicalStart(PHY_SWEEP, mpu, hz);
    } else if (mshell_AtEnd(args) ||
        (mshell_ConsumeWord(&args, "status") && mshell_AtEnd(args))) {
        rc = MFOC_RESULT_OK;
    } else {
        /* Invalid commands leave the previous stopped capture available. */
    }
    MLOGF(I, "p10phy rc=%d status=%lu frames=%lu rows=%lu\r\n", (int)rc,
          (unsigned long)g_p10_physical.status,
          (unsigned long)g_p10_physical.frame_count,
          (unsigned long)g_p10_physical.record_count);
}
MODUS_SHELL_CMD(p10phy, p10_PhysicalCommand, "P10 bounded pulse/sweep");
