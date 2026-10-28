/**
 * @file mfoc_app.c
 * @brief Stopped G431 MFOC application with one owned Motor.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_app.h"
#include "mfoc_debug.h"
#include "mfoc_numeric.h"
#include "mfoc_motor_config.h"
#include "mfoc_port.h"
#include "mdebug/mshell.h"
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
#include "perf_counter.h"
#endif
#include <math.h>
#include <stddef.h>

static modus_base_t s_base;
static modus_base_cfg_t s_base_cfg = {
    .wId = FOC_APP,
    .FcnInterface = {.Clock = mfoc_app_Clock, .Run = mfoc_app_Run},
};

int mfoc_app_Init(uintptr_t object, uintptr_t cfg)
{
    mfoc_app_t *app = (mfoc_app_t *)object;
    if (app == NULL || cfg == 0U) { return MODUS_EFAIL; }
    *app = (mfoc_app_t){0};
    app->ptBase = &s_base;
    const mfoc_app_cfg_t *config = (const mfoc_app_cfg_t *)cfg;
    app->result = mfoc_motor_Init(&app->motor, &config->motor);
    if (app->result != MFOC_RESULT_OK) {
        app->state = MFOC_APP_ERROR;
        return MODUS_EFAIL;
    }
    s_base_cfg.wParent = object;
    int status = mbase_Init(app->ptBase, &s_base_cfg);
    app->state = status == MODUS_SUCCESS ? MFOC_APP_IDLE : MFOC_APP_ERROR;
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
    app->tIsrTiming.lReportPeriodTicks = perfc_convert_ms_to_ticks(1000U);
    app->tIsrTiming.lReportTimestamp = get_system_ticks();
#endif
#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
    if (status == MODUS_SUCCESS) { mfoc_debug_WaveformInit(); }
#endif
    return status;
}

int mfoc_app_Clock(uintptr_t object)
{
    return object != 0U ? MODUS_SUCCESS : MODUS_EFAIL;
}

#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
/** @brief Report the maximum ISR ticks in the preceding one-second window.
 * @param app Application owning the timing window. @return None.
 * @note Snapshot and reset are atomic with respect to the control IRQ. */
static void _mfoc_app_ReportIsrTiming(mfoc_app_t *app)
{
    mfoc_app_isr_timing_t *timing = &app->tIsrTiming;
    uint32_t maximum = 0U;
    bool overrun = false;
    perfc_global_interrupt_status_t irq_state = 0U;

    if (!__perfc_is_time_out(timing->lReportPeriodTicks,
        &timing->lReportTimestamp, true)) { return; }
    irq_state = perfc_port_disable_global_interrupt();
    maximum = timing->wMaximumTicks;
    overrun = timing->bOverrun;
    timing->wMaximumTicks = 0U;
    timing->bOverrun = false;
    perfc_port_resume_global_interrupt(irq_state);
    if (maximum == 0U) { return; }
    uint32_t tenths_us = (uint32_t)perfc_convert_ticks_to_us(
        (int64_t)maximum * 10);
    MLOGF(T, "mfoc ISR max=%lu ticks (%lu.%lu us)%s\r\n",
        (unsigned long)maximum,
        (unsigned long)(tenths_us / 10U),
        (unsigned long)(tenths_us % 10U),
        overrun ? " (overrun)" : "");
}
#endif

int mfoc_app_Run(uintptr_t object)
{
    mfoc_app_t *app = (mfoc_app_t *)object;
    if (app == NULL || app->state != MFOC_APP_IDLE) {
        return MODUS_EFAIL;
    }
    app->result = mfoc_motor_Run(&app->motor);
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
    _mfoc_app_ReportIsrTiming(app);
#endif
    return app->result == MFOC_RESULT_OK ? MODUS_SUCCESS : MODUS_EFAIL;
}

/* Motor data and PU bases come from the 2804 datasheet in motor_config.
 * Ld/Lq share its phase inductance; encoder zero/direction need validation.
 * Inherited PI gains are starting values, not validated bench tuning. */
#define MFOC_APP_CURRENT_PI { \
    .kp = MFOC_SCALAR(0.472810f), \
    .ki_ts = MFOC_SCALAR(0.070097f), \
    .out_min = MFOC_SCALAR(-0.55f), .out_max = MFOC_SCALAR(0.55f), \
    .i_min = MFOC_SCALAR(-0.50f), .i_max = MFOC_SCALAR(0.50f)}

/* Reuse the 1 kHz speed profile from motor_config.h on a 400 Hz base. */
#define MFOC_APP_SPEED_PI { \
    .kp = MFOC_SCALAR(0.6489304f), \
    .ki_ts = MFOC_SCALAR(0.010193372f), \
    .out_min = MFOC_SCALAR(-0.10f), .out_max = MFOC_SCALAR(0.10f), \
    .i_min = MFOC_SCALAR(-0.10f), .i_max = MFOC_SCALAR(0.10f)}

#define MFOC_APP_SPEED_MAX MFOC_SCALAR(0.7583333f)

MODUS_DECLARE_OBJECT(mfoc_app, MfocApp,
    .motor = {
        .params = {
            .pole_pairs = MFOC_MOTOR_POLE_PAIRS,
            .rs = MFOC_SCALAR(MFOC_MOTOR_RS_PU),
            .ld = MFOC_SCALAR(MFOC_MOTOR_LD_PU),
            .lq = MFOC_SCALAR(MFOC_MOTOR_LQ_PU),
            .flux = MFOC_SCALAR(MFOC_MOTOR_FLUX_PU),
            .inertia = MFOC_SCALAR(MFOC_MOTOR_INERTIA_PU),
        },
        .limits = {.i_max = MFOC_SCALAR(0.14f),
            .speed_max = MFOC_APP_SPEED_MAX},
        .core = {
            .d = MFOC_APP_CURRENT_PI, .q = MFOC_APP_CURRENT_PI,
            .bus = MFOC_ONE,
        },
        .feedback = {.encoder = {
            .electrical_speed_base_hz = MFOC_SCALAR(
                MFOC_MOTOR_BASE_ELECTRICAL_HZ),
            .speed_filter_alpha = MFOC_SCALAR(0.25f),
            .sample_period_us = 1000U, .timeout_ms = 20U,
        }, .smo = {
            .sample_hz = 20000U,
            .speed_base_hz = MFOC_SCALAR(MFOC_MOTOR_BASE_ELECTRICAL_HZ),
            .cutoff_rad_s = 4000U,
            .h = MFOC_SCALAR(5.0f / MFOC_MOTOR_BASE_VOLTAGE_VOLT),
            .boundary = MFOC_SCALAR(0.1f), .i_limit = MFOC_ONE,
        }, .flux = {
            .sample_hz = 20000U,
            .speed_base_hz = MFOC_SCALAR(MFOC_MOTOR_BASE_ELECTRICAL_HZ),
            .gain = MFOC_SCALAR(MFOC_FLUX_GAIN),
            .pll_kp = MFOC_SCALAR(MFOC_FLUX_PLL_KP),
            .pll_ki = MFOC_SCALAR(MFOC_FLUX_PLL_KI),
        }},
        .speed = MFOC_APP_SPEED_PI,
        .speed_divider = 20U,
        .openloop = {20000U,
            MFOC_SCALAR(MFOC_MOTOR_BASE_ELECTRICAL_HZ),
            MFOC_SCALAR(0.2f)},
    },
)

#if MSHELL_ENABLE
/** @brief Print one mechanical encoder sample without changing Motor state.
 * @return None.
 * @note Foreground I2C access only; degrees use integer output. */
static void _mfoc_app_PrintEncoder(void)
{
    mfoc_angle_t angle = {0U};
    mfoc_result_t rc = mfoc_port_encoder(&angle);
    if (rc != MFOC_RESULT_OK) {
        MLOGF(W, "encoder read failed result=%d\r\n", (int)rc);
        return;
    }
    uint32_t raw = angle.wBam32 >> 20U;
    uint32_t degrees100 = (raw * 36000U + 2048U) / 4096U;
    MLOGF(I, "encoder raw=%lu angle=%lu.%02lu deg\r\n",
        (unsigned long)raw, (unsigned long)(degrees100 / 100U),
        (unsigned long)(degrees100 % 100U));
}

/** @brief Apply dq current and start only from IDLE.
 * @param d d-axis PU. @param q q-axis PU. @return Command/start result. */
static mfoc_result_t _mfoc_app_Current(float d, float q)
{
    const mfoc_dq_t ref = {mfoc_from_float(d), mfoc_from_float(q)};
    mfoc_motor_status_t status = {0};
    mfoc_result_t rc = MFOC_RESULT_INVALID_STATE;
    __IRQ_SAFE {
        mfoc_motor_GetStatus(&tMfocApp.motor, &status);
        rc = mfoc_motor_SetCurrent(&tMfocApp.motor, &ref);
    }
    if (rc == MFOC_RESULT_OK && status.state == MFOC_MOTOR_IDLE) {
        rc = mfoc_motor_Start(&tMfocApp.motor);
    }
    return rc;
}

/** @brief Apply an electrical-speed PU command and start from IDLE.
 * @param speed_ref Speed reference within the configured limit.
 * @return Command/start result. */
static mfoc_result_t _mfoc_app_Speed(float speed_ref)
{
    mfoc_motor_status_t status = {0};
    mfoc_result_t rc = MFOC_RESULT_INVALID_STATE;
    mfoc_scalar_t ref = mfoc_from_float(speed_ref);
    __IRQ_SAFE {
        mfoc_motor_GetStatus(&tMfocApp.motor, &status);
        rc = mfoc_motor_SetSpeed(&tMfocApp.motor, ref);
    }
    if (rc == MFOC_RESULT_OK && status.state == MFOC_MOTOR_IDLE) {
        rc = mfoc_motor_Start(&tMfocApp.motor);
    }
    return rc;
}

/** @brief Print a consistent HFI snapshot from the foreground.
 * @return None. No algorithm, encoder read or periodic log is added. */
static void _mfoc_app_PrintHfi(void)
{
#if MFOC_FEEDBACK_ENABLE_HFI
    mfoc_hfi_state_t state = MFOC_HFI_ACQUIRE;
    mfoc_hfi_output_t output = {0};
    mfoc_motor_state_t motor_state = MFOC_MOTOR_UNINITIALIZED;
    uint32_t steps = 0U;
    uint16_t lock = 0U;
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
    uint32_t maximum = 0U;
    bool overrun = false;
#endif
    __IRQ_SAFE {
        state = tMfocApp.motor.feedback.hfi.state;
        output = tMfocApp.motor.feedback.hfi_output;
        motor_state = tMfocApp.motor.state;
        steps = tMfocApp.motor.feedback.hfi.startup_count;
        lock = tMfocApp.motor.feedback.hfi.lock_count;
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
        maximum = tMfocApp.tIsrTiming.wMaximumTicks;
        overrun = tMfocApp.tIsrTiming.bOverrun;
#endif
    }
    const char *name = "polarity";
    if (state == MFOC_HFI_ACQUIRE) { name = "acquire"; }
    else if (state == MFOC_HFI_TRACK) { name = "track"; }
    else if (state == MFOC_HFI_FAILED) { name = "failed"; }
    else { /* Polarity identification substate. */ }
    uint32_t angle_mt = (uint32_t)(
        ((uint64_t)output.angle.wBam32 * 1000U) >> 32U);
    int32_t speed_mpu = (int32_t)(mfoc_to_float(output.speed) * 1000.0f);
    MLOGF(I, "hfi state=%s motor=%u valid=%u "
        "angle_mT=%lu speed_mpu=%ld polarity=%u steps=%lu lock=%u"
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
        " isr_win_max=%lu overrun=%u budget=%lu"
#endif
        "\r\n",
        name, (unsigned)motor_state,
        (unsigned)(output.valid && motor_state == MFOC_MOTOR_RUNNING),
        (unsigned long)angle_mt, (long)speed_mpu,
        (unsigned)MFOC_HFI_ENABLE_POLARITY, (unsigned long)steps,
        (unsigned)lock
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
        , (unsigned long)maximum, (unsigned)overrun,
        (unsigned long)MFOC_ISR_BUDGET_TICKS
#endif
        );
#else
    MLOGF(I, "hfi state=disabled (MFOC_FEEDBACK_ENABLE_HFI=0)\r\n");
#endif
}

/** @brief Print the independent Flux estimate and its configured scales.
 * @return None. This diagnostic does not start or reconfigure control. */
static void _mfoc_app_PrintFlux(void)
{
#if MFOC_FEEDBACK_ENABLE_FLUX
    mfoc_flux_output_t output = {0};
    mfoc_flux_cfg_t cfg = {0};
    mfoc_scalar_t psi = MFOC_ZERO;
    mfoc_motor_state_t state = MFOC_MOTOR_UNINITIALIZED;
    bool available = false;
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
    uint32_t maximum = 0U;
    bool overrun = false;
#endif
    __IRQ_SAFE {
        const mfoc_motor_t *motor = &tMfocApp.motor;
        output = motor->feedback.flux_output;
        cfg = motor->feedback.flux_cfg;
        psi = motor->params.flux;
        state = motor->state;
        available = state == MFOC_MOTOR_RUNNING && !motor->hold_active &&
            motor->previous_core_output_valid && output.valid;
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
        maximum = tMfocApp.tIsrTiming.wMaximumTicks;
        overrun = tMfocApp.tIsrTiming.bOverrun;
#endif
    }
    uint32_t angle_mt = (uint32_t)(
        ((uint64_t)output.angle.wBam32 * 1000U) >> 32U);
    MLOGF(I, "flux motor=%u valid=%u angle_mT=%lu speed_mpu=%ld "
        "psi_mpu=%ld gain_mpu=%ld kp_mpu=%ld ki_mpu=%ld"
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
        " isr_win_max=%lu overrun=%u budget=%lu"
#endif
        "\r\n", (unsigned)state, (unsigned)available,
        (unsigned long)angle_mt, (long)(mfoc_to_float(output.speed) * 1000.0f),
        (long)(mfoc_to_float(psi) * 1000.0f),
        (long)(mfoc_to_float(cfg.gain) * 1000.0f),
        (long)(mfoc_to_float(cfg.pll_kp) * 1000.0f),
        (long)(mfoc_to_float(cfg.pll_ki) * 1000.0f)
#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
        , (unsigned long)maximum, (unsigned)overrun,
        (unsigned long)MFOC_ISR_BUDGET_TICKS
#endif
        );
#else
    MLOGF(I, "flux state=disabled (MFOC_FEEDBACK_ENABLE_FLUX=0)\r\n");
#endif
}

/** @brief Parse Motor commands and execute control or diagnostics.
 * @param args Arguments after motor. @return None.
 */
static void _mfoc_app_CmdMotor(const char *args)
{
    if (args == NULL) { return; }
    const char *cursor = args;
    float d = 0.0f;
    float q = 0.0f;
    bool valid = false;

    if (mshell_ConsumeWord(&cursor, "flux")) {
        if ((mshell_AtEnd(cursor) ||
             mshell_ConsumeWord(&cursor, "status")) && mshell_AtEnd(cursor)) {
            _mfoc_app_PrintFlux();
        } else {
            MLOGF(W, "invalid motor arguments\r\n");
        }
        return;
    } else if (mshell_ConsumeWord(&cursor, "hfi")) {
        if (mshell_AtEnd(cursor) || mshell_ConsumeWord(&cursor, "start")) {
            if (!mshell_AtEnd(cursor)) {
                MLOGF(W, "invalid motor arguments\r\n");
                return;
            }
            tMfocApp.result = mfoc_motor_HfiStart(&tMfocApp.motor);
            MLOGF(I, "motor hfi result=%d\r\n", (int)tMfocApp.result);
        } else if (mshell_ConsumeWord(&cursor, "status") &&
                   mshell_AtEnd(cursor)) {
            _mfoc_app_PrintHfi();
        } else {
            MLOGF(W, "invalid motor arguments\r\n");
        }
        return;
    } else if (mshell_ConsumeWord(&cursor, "encoder")) {
        valid = mshell_AtEnd(cursor);
        if (valid) {
            _mfoc_app_PrintEncoder();
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "current")) {
        valid = mshell_Parse(&cursor, &d) == MSHELL_PARSE_OK &&
            mshell_Parse(&cursor, &q) == MSHELL_PARSE_OK &&
            mshell_AtEnd(cursor);
        if (valid) {
            tMfocApp.result = _mfoc_app_Current(d, q);
            MLOGF(I, "motor current result=%d\r\n", (int)tMfocApp.result);
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "speed")) {
        valid = mshell_Parse(&cursor, &q) == MSHELL_PARSE_OK &&
            mshell_AtEnd(cursor) && isfinite(q);
        if (valid) {
            tMfocApp.result = _mfoc_app_Speed(q);
            MLOGF(I, "motor speed result=%d\r\n", (int)tMfocApp.result);
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "openloop")) {
        valid = mshell_Parse(&cursor, &q) == MSHELL_PARSE_OK &&
            mshell_Parse(&cursor, &d) == MSHELL_PARSE_OK &&
            mshell_AtEnd(cursor);
        if (valid) {
            tMfocApp.result = mfoc_motor_OpenLoopStart(&tMfocApp.motor,
                mfoc_from_float(q), mfoc_from_float(d));
            MLOGF(I, "motor openloop result=%d\r\n", (int)tMfocApp.result);
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "sensorless")) {
        valid = mshell_Parse(&cursor, &q) == MSHELL_PARSE_OK &&
            mshell_Parse(&cursor, &d) == MSHELL_PARSE_OK &&
            mshell_AtEnd(cursor) && isfinite(q) && isfinite(d);
        if (valid) {
            tMfocApp.result = mfoc_motor_SensorlessStart(&tMfocApp.motor,
                mfoc_from_float(q), mfoc_from_float(d));
            MLOGF(I, "motor sensorless result=%d\r\n",
                  (int)tMfocApp.result);
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "align")) {
        valid = mshell_AtEnd(cursor);
        if (valid) {
            tMfocApp.result = mfoc_motor_AlignStart(&tMfocApp.motor,
                MFOC_SCALAR(0.04f), MFOC_ZERO);
            MLOGF(I, "motor align result=%d\r\n", (int)tMfocApp.result);
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "stop")) {
        valid = mshell_AtEnd(cursor);
        if (valid) {
            tMfocApp.result = mfoc_motor_Stop(&tMfocApp.motor);
            MLOGF(I, "motor stop result=%d\r\n", (int)tMfocApp.result);
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "clear")) {
        valid = mshell_AtEnd(cursor);
        /* TODO: mfoc_motor_ClearFault(). */
    } else if (mshell_ConsumeWord(&cursor, "status")) {
        valid = mshell_AtEnd(cursor);
        if (valid) {
            /* TODO: print Motor state and fault details. */
            MLOGF(I, "mfoc app state=%u result=%d\r\n",
                (unsigned)tMfocApp.state, (int)tMfocApp.result);
            return;
        }
    } else {
        MLOGF(I, "usage: motor current <id> <iq> | speed <pu> | align\r\n"
            "       motor openloop <speed_pu> <id_pu>\r\n"
            "       motor sensorless <speed_pu> <current_pu>\r\n"
            "       motor hfi [start|status]; motor stop to end\r\n"
            "       motor flux [status]; observer only\r\n"
            "       motor encoder | stop | clear | status | help\r\n"
            "       clear: parse only; execution pending\r\n");
        return;
    }
    if (valid) {
        MLOGF(I, "motor %s: parsed only; execution pending\r\n", args);
    } else {
        MLOGF(W, "invalid motor arguments\r\n");
    }
}

MODUS_SHELL_CMD(motor, _mfoc_app_CmdMotor,
    "MFOC: current/speed/openloop/sensorless/hfi/flux/encoder/align/stop/status");
#endif
