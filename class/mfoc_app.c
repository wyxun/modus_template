/**
 * @file mfoc_app.c
 * @brief Stopped G431 MFOC application with one owned Motor.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_app.h"
#include "mfoc_numeric.h"
#include "mdebug/mshell.h"
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
    return status;
}

int mfoc_app_Clock(uintptr_t object)
{
    return object != 0U ? MODUS_SUCCESS : MODUS_EFAIL;
}

int mfoc_app_Run(uintptr_t object)
{
    mfoc_app_t *app = (mfoc_app_t *)object;
    if (app == NULL || app->state != MFOC_APP_IDLE) {
        return MODUS_EFAIL;
    }
    app->result = mfoc_motor_Run(&app->motor);
    return app->result == MFOC_RESULT_OK ? MODUS_SUCCESS : MODUS_EFAIL;
}

/* Stopped integration profile: Ib=3.5 A, Vb=12 V, electrical base=400 Hz.
 * R=4 ohm, Ld=Lq=1.2 mH are carried from the previous motor profile.
 * Flux=1 PU is unmeasured and unused here; zero/direction are uncalibrated.
 * These settings do not authorize a Motor Start or observer operation. */
#define MFOC_APP_CURRENT_PI {                                                  \
    .kp = MFOC_SCALAR(0.472810f), .ki_ts = MFOC_SCALAR(0.070097f),              \
    .out_min = MFOC_SCALAR(-0.55f), .out_max = MFOC_SCALAR(0.55f),              \
    .i_min = MFOC_SCALAR(-0.50f), .i_max = MFOC_SCALAR(0.50f)}

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
        .limits = {.i_max = MFOC_SCALAR(0.14f)},
        .core = {
            .d = MFOC_APP_CURRENT_PI, .q = MFOC_APP_CURRENT_PI,
            .m_max = MFOC_SCALAR(0.55f), .bus = MFOC_ONE,
        },
        .feedback = {.encoder = {
            .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
            .speed_filter_alpha = MFOC_ONE,
            .sample_period_us = 1000U, .timeout_ms = 20U,
        }},
        .speed_divider = 0U,
    },
)

#if MSHELL_ENABLE
/** @brief Parse Motor commands and start/cancel fixed-field alignment.
 * @param args Arguments after motor. @return None.
 * @note Alignment/Stop require the target control IRQ integration. */
static void _mfoc_app_CmdMotor(const char *args)
{
    if (args == NULL) { return; }
    const char *cursor = args;
    float d = 0.0f;
    float q = 0.0f;
    bool valid = false;

    if (mshell_ConsumeWord(&cursor, "current")) {
        valid = mshell_Parse(&cursor, &d) == MSHELL_PARSE_OK &&
            mshell_Parse(&cursor, &q) == MSHELL_PARSE_OK &&
            mshell_AtEnd(cursor);
        /* TODO: mfoc_motor_SetCurrent() and Motor Start. */
    } else if (mshell_ConsumeWord(&cursor, "speed")) {
        valid = mshell_Parse(&cursor, &q) == MSHELL_PARSE_OK &&
            mshell_AtEnd(cursor) && fabsf(q) <= 1.0f;
        /* TODO: configure speed PI, mfoc_motor_SetSpeed() and Motor Start. */
    } else if (mshell_ConsumeWord(&cursor, "align")) {
        valid = mshell_AtEnd(cursor);
        if (valid) {
#if defined(MFOC_MOTOR_IRQ_ENABLED) && MFOC_MOTOR_IRQ_ENABLED
            tMfocApp.result = mfoc_motor_AlignStart(&tMfocApp.motor,
                MFOC_SCALAR(0.04f), MFOC_ZERO);
            MLOGF(I, "motor align result=%d\r\n", (int)tMfocApp.result);
#else
            MLOGF(W, "motor align unavailable: control IRQ not enabled\r\n");
#endif
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "stop")) {
        valid = mshell_AtEnd(cursor);
        if (valid) {
#if defined(MFOC_MOTOR_IRQ_ENABLED) && MFOC_MOTOR_IRQ_ENABLED
            tMfocApp.result = mfoc_motor_Stop(&tMfocApp.motor);
            MLOGF(I, "motor stop result=%d\r\n", (int)tMfocApp.result);
#else
            MLOGF(W, "motor stop unavailable: control IRQ not enabled\r\n");
#endif
            return;
        }
    } else if (mshell_ConsumeWord(&cursor, "clear")) {
        valid = mshell_AtEnd(cursor);
        /* TODO: mfoc_motor_ClearFault(). */
    } else if (mshell_ConsumeWord(&cursor, "status")) {
        valid = mshell_AtEnd(cursor);
        if (valid) {
            /* TODO: add Motor status after its lifecycle ports are connected. */
            MLOGF(I, "mfoc app state=%u result=%d\r\n",
                (unsigned)tMfocApp.state, (int)tMfocApp.result);
            return;
        }
    } else {
        MLOGF(I, "usage: motor current <id> <iq> | speed <pu> | align\r\n"
            "       motor stop | clear | status | help\r\n"
            "       current/speed/clear: parse only; execution pending\r\n");
        return;
    }
    if (valid) {
        MLOGF(I, "motor %s: parsed only; execution pending\r\n", args);
    } else {
        MLOGF(W, "invalid motor arguments\r\n");
    }
}

MODUS_SHELL_CMD(motor, _mfoc_app_CmdMotor,
    "MFOC: align/stop/status; current/speed/clear parse only");
#endif
