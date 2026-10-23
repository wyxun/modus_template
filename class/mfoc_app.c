/**
 * @file mfoc_app.c
 * @brief Stopped G431 MFOC application with one owned Motor.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_app.h"
#include "mfoc_debug.h"
#include "mfoc_numeric.h"
#include "mfoc_port.h"
#include "mdebug/mshell.h"
#if !defined(__NO_USE_LOG__) && !defined(FOC_ISR_TIMING_DWT_ONLY)
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
#if !defined(__NO_USE_LOG__) && !defined(FOC_ISR_TIMING_DWT_ONLY)
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

#if !defined(__NO_USE_LOG__) && !defined(FOC_ISR_TIMING_DWT_ONLY)
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
#if !defined(__NO_USE_LOG__) && !defined(FOC_ISR_TIMING_DWT_ONLY)
    _mfoc_app_ReportIsrTiming(app);
#endif
    return app->result == MFOC_RESULT_OK ? MODUS_SUCCESS : MODUS_EFAIL;
}

/* Integration profile uses Ib=3.5 A, Vb=12 V, and 400 Hz base.
 * R=4 ohm, Ld=Lq=1.2 mH are carried from the previous motor profile.
 * Flux=1 PU is unmeasured; encoder zero/direction need bench validation.
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
            .pole_pairs = 7U,
            .rs = MFOC_SCALAR(4.0f * 3.5f / 12.0f),
            .ld = MFOC_SCALAR(0.0012f * 2513.274123f * 3.5f / 12.0f),
            .lq = MFOC_SCALAR(0.0012f * 2513.274123f * 3.5f / 12.0f),
            .flux = MFOC_ONE,
            .inertia = MFOC_ZERO,
        },
        .limits = {.i_max = MFOC_SCALAR(0.14f),
            .speed_max = MFOC_APP_SPEED_MAX},
        .core = {
            .d = MFOC_APP_CURRENT_PI, .q = MFOC_APP_CURRENT_PI,
            .bus = MFOC_ONE,
        },
        .feedback = {.encoder = {
            .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
            .speed_filter_alpha = MFOC_SCALAR(0.25f),
            .sample_period_us = 1000U, .timeout_ms = 20U,
        }, .smo = {
            .sample_hz = 20000U, .speed_base_hz = MFOC_SCALAR(400.0f),
            .cutoff_rad_s = 4000U, .h = MFOC_SCALAR(5.0f / 12.0f),
            .boundary = MFOC_SCALAR(0.1f), .i_limit = MFOC_ONE,
        }},
        .speed = MFOC_APP_SPEED_PI,
        .speed_divider = 20U,
        .openloop = {20000U, MFOC_SCALAR(400.0f), MFOC_SCALAR(0.2f)},
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

/** @brief Parse Motor commands and execute current/speed/align/stop.
 * @param args Arguments after motor. @return None.
 */
static void _mfoc_app_CmdMotor(const char *args)
{
    if (args == NULL) { return; }
    const char *cursor = args;
    float d = 0.0f;
    float q = 0.0f;
    bool valid = false;

    if (mshell_ConsumeWord(&cursor, "encoder")) {
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
    "MFOC: current/speed/openloop/encoder/align/stop/status");
#endif
