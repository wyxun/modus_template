/** @file mfoc_p10_kcl.c
 * Check sampling-window coverage with bounded sensorless runs.
 */
#include "mfoc_p10_kcl.h"
#include "mfoc_app.h"
#include "mfoc_port.h"
#include "mdebug/mshell.h"
#include "mdebug/util_debug.h"
#include "stm32g4xx.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#if !defined(MFOC_NUMERIC_FLOAT) || MFOC_P10_PHASE != 2 || MFOC_P10_PHYSICAL
#error "P10 KCL requires FLOAT top-update PWM and exclusive use of TIM2 CH2"
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_FLUX
typedef mfoc_flux_output_t p10_kcl_feedback_t;
#define P10_KCL_FEEDBACK(motor) (&(motor)->feedback.flux_output)
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
typedef mfoc_smo_output_t p10_kcl_feedback_t;
#define P10_KCL_FEEDBACK(motor) (&(motor)->feedback.smo_output)
#else
#error "P10 KCL requires Flux or SMO sensorless startup"
#endif

#define P10_KCL_BINS 32U
#define P10_KCL_FRAMES 512U
#define P10_KCL_TIMEOUT_TICKS (MFOC_PERFC_HZ / 10U)
#define P10_KCL_MAX_PHASE 0.12f
#define P10_KCL_MIN_SPEED 0.38f
#define P10_KCL_MAX_REF 0.50f
#define P10_KCL_MAX_OBSERVED_SPEED 0.52f

typedef enum {
    KCL_IDLE, KCL_RECORDING, KCL_DONE, KCL_ABORTED, KCL_TIMEOUT,
    KCL_CURRENT, KCL_CONTROL, KCL_SPEED, KCL_SHUTDOWN,
} p10_kcl_status_t;

typedef struct {
    uint32_t frames;
    float sum_residual_sq;
    float peak_abs_residual;
    float peak_abs_current[3];
    /* Unnormalized squared modulation, in compare ticks squared.
     * Divide by 9*ARR^2 before taking the square root on the host. */
    float peak_mod_energy;
    float residual_at_max_v;
    uint16_t max_compare[3];
    uint16_t raw_at_peak_residual[3];
    float i_at_peak_residual[3];
    uint16_t compare_before_at_peak[3];
    uint16_t compare_after_at_peak[3];
} p10_kcl_bin_t;

typedef struct {
    volatile uint32_t status;
    uint32_t frames;
    uint32_t arr;
    uint32_t capture_first;
    uint32_t capture_last;
    float peak_phase_current;
    float measured_bus;
    p10_kcl_bin_t bins[P10_KCL_BINS];
} p10_kcl_capture_t;

_Static_assert(sizeof(p10_kcl_bin_t) == 68U, "KCL bin layout");
_Static_assert(offsetof(p10_kcl_capture_t, bins) == 28U, "KCL capture layout");

/* Stable after a terminal status. Read only after the bridge closes. */
p10_kcl_capture_t g_p10_kcl;
extern mfoc_app_t tMfocApp;

static float s_current[3];
static uint16_t s_raw[3];
static uint32_t s_capture;
/* At frame k, cached compare history starts with d[k-2] and d[k-1]. */
static uint16_t s_compare[3][3];
static uint8_t s_write;
static uint8_t s_count;
static bool s_overcurrent;

void mfoc_p10_kcl_Reset(void)
{
    s_write = 0U;
    s_count = 0U;
    s_overcurrent = false;
}

bool mfoc_p10_kcl_Current(uint32_t capture, const mfoc_uvw_t *current,
                          uint16_t u, uint16_t v, uint16_t w)
{
    const float peak = fmaxf(fabsf(current->qU),
        fmaxf(fabsf(current->qV), fabsf(current->qW)));
    if (g_p10_kcl.status == KCL_RECORDING) {
        g_p10_kcl.peak_phase_current = fmaxf(g_p10_kcl.peak_phase_current,
                                            peak);
        s_capture = capture;
        s_current[0] = current->qU;
        s_current[1] = current->qV;
        s_current[2] = current->qW;
        s_raw[0] = u;
        s_raw[1] = v;
        s_raw[2] = w;
    }
    if (peak > P10_KCL_MAX_PHASE) { s_overcurrent = true; return false; }
    return true;
}

void mfoc_p10_kcl_Duty(uint16_t u, uint16_t v, uint16_t w)
{
    s_compare[s_write][0] = u;
    s_compare[s_write][1] = v;
    s_compare[s_write][2] = w;
    s_write = s_write == 2U ? 0U : (uint8_t)(s_write + 1U);
    if (s_count < 3U) { ++s_count; }
}

/* Caller excludes ADC preemption. TIM2 CH1 is the PWM phase capture. */
static void finish(mfoc_motor_t *motor, p10_kcl_status_t status)
{
    if (motor->state == MFOC_MOTOR_RUNNING &&
        mfoc_motor_Stop(motor) != MFOC_RESULT_OK) { status = KCL_SHUTDOWN; }
    TIM2->DIER &= ~TIM_DIER_CC2IE;
    TIM2->SR = ~TIM_SR_CC2IF;
    NVIC_DisableIRQ(TIM2_IRQn);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    __DMB();
    g_p10_kcl.status = status;
}

void TIM2_IRQHandler(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if ((TIM2->SR & TIM_SR_CC2IF) != 0U &&
        g_p10_kcl.status == KCL_RECORDING) {
        finish(&tMfocApp.motor, KCL_TIMEOUT);
    }
    __set_PRIMASK(primask);
}

static void record(uint32_t angle)
{
    p10_kcl_bin_t *bin = &g_p10_kcl.bins[angle >> 27U];
    /* One command was just issued. The oldest slot is d[k-2]; the
     * next slot d[k-1] is active during ADC2 rank 2 after this top. */
    const uint16_t *before = s_compare[s_write];
    const uint16_t *after = s_compare[s_write == 2U ? 0U : s_write + 1U];
    const float residual = s_current[0] + s_current[1] + s_current[2];
    const float abs_residual = fabsf(residual);
    bin->sum_residual_sq += residual * residual;
    if (bin->frames == 0U || abs_residual > bin->peak_abs_residual) {
        bin->peak_abs_residual = abs_residual;
        memcpy(bin->i_at_peak_residual, s_current, sizeof s_current);
        memcpy(bin->raw_at_peak_residual, s_raw, sizeof s_raw);
        memcpy(bin->compare_before_at_peak, before, sizeof s_compare[0]);
        memcpy(bin->compare_after_at_peak, after, sizeof s_compare[0]);
    }
    if (after[1] > bin->max_compare[1]) { bin->residual_at_max_v = residual; }
    for (unsigned i = 0U; i < 3U; ++i) {
        bin->peak_abs_current[i] = fmaxf(bin->peak_abs_current[i],
                                         fabsf(s_current[i]));
        if (after[i] > bin->max_compare[i]) { bin->max_compare[i] = after[i]; }
    }
    const float a = (float)(2 * after[0] - after[1] - after[2]);
    const float b = (float)(after[1] - after[2]);
    bin->peak_mod_energy = fmaxf(bin->peak_mod_energy, a * a + 3.0f * b * b);
    ++bin->frames;
}

static bool eligible(const mfoc_motor_t *motor)
{
    return motor->state == MFOC_MOTOR_RUNNING && !motor->hold_active &&
        motor->sensorless_start.state == MFOC_SENSORLESS_START_NONE &&
        motor->mode == MFOC_MOTOR_MODE_SPEED &&
        fabsf(motor->speed_ref) <= P10_KCL_MAX_REF &&
        P10_KCL_FEEDBACK(motor)->valid && motor->previous_core_output_valid;
}

void mfoc_p10_kcl_AfterStep(mfoc_motor_t *motor)
{
    if (g_p10_kcl.status != KCL_RECORDING) { return; }
    if (s_overcurrent) { finish(motor, KCL_CURRENT); return; }
    if (motor->irq_result != MFOC_RESULT_OK || !eligible(motor)) {
        finish(motor, KCL_CONTROL);
        return;
    }
    const p10_kcl_feedback_t *feedback = P10_KCL_FEEDBACK(motor);
    const float speed = fabsf(feedback->speed);
    if (speed < P10_KCL_MIN_SPEED || speed > P10_KCL_MAX_OBSERVED_SPEED) {
        finish(motor, KCL_SPEED);
        return;
    }
    if (g_p10_kcl.frames == 0U) { g_p10_kcl.capture_first = s_capture; }
    g_p10_kcl.capture_last = s_capture;
    record(feedback->angle.wBam32);
    if (++g_p10_kcl.frames == P10_KCL_FRAMES) { finish(motor, KCL_DONE); }
}

static mfoc_result_t start(void)
{
    if (g_p10_kcl.status == KCL_RECORDING || !eligible(&tMfocApp.motor)) {
        return MFOC_RESULT_INVALID_STATE;
    }
    float bus = 0.0f;
    if (mfoc_port_bus(&bus) != MFOC_RESULT_OK || bus < 0.90f || bus > 1.10f) {
        return MFOC_RESULT_SAFETY;
    }
    /* ISR ignores the data until the state below publishes the new run. */
    memset(g_p10_kcl.bins, 0, sizeof g_p10_kcl.bins);
    g_p10_kcl.frames = 0U;
    g_p10_kcl.arr = TIM1->ARR;
    g_p10_kcl.capture_first = 0U;
    g_p10_kcl.capture_last = 0U;
    g_p10_kcl.peak_phase_current = 0.0f;
    g_p10_kcl.measured_bus = bus;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mfoc_result_t result = MFOC_RESULT_INVALID_STATE;
    const float speed = fabsf(P10_KCL_FEEDBACK(&tMfocApp.motor)->speed);
    if (eligible(&tMfocApp.motor) && !s_overcurrent && s_count == 3U &&
        speed >= P10_KCL_MIN_SPEED && speed <= P10_KCL_MAX_OBSERVED_SPEED) {
        TIM2->CCR2 = TIM2->CNT + P10_KCL_TIMEOUT_TICKS;
        TIM2->SR = ~TIM_SR_CC2IF;
        NVIC_SetPriority(TIM2_IRQn, 2U);
        NVIC_ClearPendingIRQ(TIM2_IRQn);
        TIM2->DIER |= TIM_DIER_CC2IE;
        NVIC_EnableIRQ(TIM2_IRQn);
        __DMB();
        g_p10_kcl.status = KCL_RECORDING;
        result = MFOC_RESULT_OK;
    }
    __set_PRIMASK(primask);
    return result;
}

static void command(const char *args)
{
    mfoc_result_t rc = MFOC_RESULT_INVALID_ARGUMENT;
    if (mshell_ConsumeWord(&args, "start") && mshell_AtEnd(args)) {
        rc = start();
    } else if (mshell_ConsumeWord(&args, "stop") && mshell_AtEnd(args)) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (g_p10_kcl.status == KCL_RECORDING) {
            finish(&tMfocApp.motor, KCL_ABORTED);
        }
        __set_PRIMASK(primask);
        rc = MFOC_RESULT_OK;
    } else if (mshell_AtEnd(args) ||
               (mshell_ConsumeWord(&args, "status") && mshell_AtEnd(args))) {
        rc = MFOC_RESULT_OK;
    } else {
        /* Preserve any previous capture for an unrecognized command. */
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const mfoc_motor_t *motor = &tMfocApp.motor;
    const bool ready = eligible(motor);
    const unsigned handover = (unsigned)motor->sensorless_start.state;
    const float speed = P10_KCL_FEEDBACK(motor)->speed;
    __set_PRIMASK(primask);
    MLOGF(I, "p10kcl rc=%d status=%lu frames=%lu/%u peak_phase_mpu=%ld "
          "ready=%u handover=%u speed_mpu=%ld\r\n",
          (int)rc, (unsigned long)g_p10_kcl.status,
          (unsigned long)g_p10_kcl.frames, P10_KCL_FRAMES,
          (long)(1000.0f * g_p10_kcl.peak_phase_current), (unsigned)ready,
          handover, (long)(1000.0f * speed));
}

MODUS_SHELL_CMD(p10kcl, command, "P10 sensorless KCL start/status/stop");
