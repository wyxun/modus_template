/**
 * @file app_init_test.c
 * @brief Application initialization order and failure-registration checks.
 * @author Codex
 * @date 2026-10-21
 */
#include "../../class/mfoc_app.c"
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static mfoc_result_t s_init_result;
static unsigned s_init_count;
static unsigned s_run_count;
static unsigned s_register_count;
static int s_register_result;

#if MSHELL_ENABLE
static char s_log[512];
static unsigned s_align_count;
static unsigned s_stop_count;
static mfoc_result_t s_align_result;
uint8_t g_chGLogMask = MLOG_MASK_ALL;

/** @brief Capture command responses without a hardware stream.
 * @param format Format string. @param ... Arguments. @return None. */
void util_debug_Printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(s_log, sizeof(s_log), format, args);
    va_end(args);
}

/** @brief Supply the Shell registration boundary.
 * @param address Command descriptor. @param unused Unused. @return Success. */
int mshell_RegisterCmd(uintptr_t address, uintptr_t unused)
{
    (void)unused;
    assert(address == (uintptr_t)&shell_cmd_motor);
    return MODUS_SUCCESS;
}

/** @brief Verify command parsing and response without real hardware.
 * @param args Command arguments. @param response Expected response fragment.
 * @return None. */
static void command(const char *args, const char *response)
{
    s_log[0] = '\0';
    shell_cmd_motor.pfcnHandle(args);
    assert(strstr(s_log, response) != NULL);
    assert(tMfocApp.state == MFOC_APP_IDLE);
}

/** @brief Exercise parsing and alignment routing at the Motor boundary.
 * @return None. */
static void shell_commands(void)
{
    tMfocApp.state = MFOC_APP_IDLE;
    assert(strcmp(shell_cmd_motor.pchName, "motor") == 0);
    command("current 0.01 -0.02", "current 0.01 -0.02: parsed only");
    command("speed -0.1", "speed -0.1: parsed only");
#if defined(MFOC_MOTOR_IRQ_ENABLED) && MFOC_MOTOR_IRQ_ENABLED
    command("align", "motor align result=0");
    command("stop", "motor stop result=0");
    assert(s_align_count == 1U && s_stop_count == 1U);
    s_align_result = MFOC_RESULT_IO;
    command("align", "motor align result=6");
    assert(s_align_count == 2U && tMfocApp.result == MFOC_RESULT_IO);
#else
    command("align", "align unavailable: control IRQ not enabled");
    command("stop", "stop unavailable: control IRQ not enabled");
    assert(s_align_count == 0U && s_stop_count == 0U);
#endif
    unsigned align_count = s_align_count;
    unsigned stop_count = s_stop_count;
    command("clear", "clear: parsed only");
    command("status", "mfoc app state=");
    command("", "usage: motor");
    command("help", "usage: motor");
    command("current 0.01", "invalid motor arguments");
    command("current nan 0", "invalid motor arguments");
    command("speed inf", "invalid motor arguments");
    command("speed 1.1", "invalid motor arguments");
    command("speed 0.1 trailing", "invalid motor arguments");
    command("align trailing", "invalid motor arguments");
    command("stop trailing", "invalid motor arguments");
    command("clear trailing", "invalid motor arguments");
    command("status trailing", "invalid motor arguments");
    assert(s_align_count == align_count && s_stop_count == stop_count);
}

/** @brief Record the alignment request and verify its app defaults.
 * @param motor App instance. @param id_pu Current. @param angle_pu Turns.
 * @return Injected status. */
mfoc_result_t mfoc_motor_AlignStart(mfoc_motor_t *motor,
    mfoc_scalar_t id_pu, mfoc_scalar_t angle_pu)
{
    assert(motor == &tMfocApp.motor);
    assert(id_pu == MFOC_SCALAR(0.04f) && angle_pu == MFOC_ZERO);
    ++s_align_count;
    return s_align_result;
}

/** @brief Record explicit cancellation. @param motor App instance.
 * @return OK. */
mfoc_result_t mfoc_motor_Stop(mfoc_motor_t *motor)
{
    assert(motor == &tMfocApp.motor);
    ++s_stop_count;
    return MFOC_RESULT_OK;
}
#endif

/** @brief Supply Motor initialization and record invocation.
 * @param motor Instance. @param cfg Configuration. @return Test status. */
mfoc_result_t mfoc_motor_Init(mfoc_motor_t *motor,
                               const mfoc_motor_cfg_t *cfg)
{
    assert(motor != NULL && cfg != NULL);
    ++s_init_count;
    return s_init_result;
}

/** @brief Supply foreground service without hardware.
 * @param motor Instance. @return OK. */
mfoc_result_t mfoc_motor_Run(mfoc_motor_t *motor)
{
    assert(motor != NULL);
    ++s_run_count;
    return MFOC_RESULT_OK;
}

/** @brief Verify registration follows successful Motor initialization.
 * @param base Base object. @param cfg Base configuration.
 * @return Selected framework status. */
int mbase_Init(modus_base_t *base, modus_base_cfg_t *cfg)
{
    assert(base != NULL && cfg != NULL);
    assert(s_init_count != 0U && s_init_result == MFOC_RESULT_OK);
    assert(cfg->FcnInterface.Clock != NULL);
    assert(cfg->FcnInterface.Run != NULL);
    assert(cfg->FcnInterface.Clock(cfg->wParent) == MODUS_SUCCESS);
    ++s_register_count;
    return s_register_result;
}

/** @brief Check success, failure and Run without deferred initialization.
 * @return Zero on success. */
int main(void)
{
    mfoc_app_t app = {0};
    mfoc_app_cfg_t cfg = {0};
    assert(mfoc_app_Init(0U, (uintptr_t)&cfg) == MODUS_EFAIL);
    assert(mfoc_app_Run((uintptr_t)&app) == MODUS_EFAIL);
    s_init_result = MFOC_RESULT_IO;
    assert(mfoc_app_Init((uintptr_t)&app, (uintptr_t)&cfg) == MODUS_EFAIL);
    assert(app.state == MFOC_APP_ERROR && s_register_count == 0U);
    s_init_result = MFOC_RESULT_OK;
    s_register_result = MODUS_SUCCESS;
    assert(mfoc_app_Init((uintptr_t)&app, (uintptr_t)&cfg) == MODUS_SUCCESS);
    assert(app.state == MFOC_APP_IDLE && s_register_count == 1U);
    unsigned count = s_init_count;
    assert(mfoc_app_Run((uintptr_t)&app) == MODUS_SUCCESS);
    assert(s_run_count == 1U && s_init_count == count);
    s_register_result = MODUS_EFAIL;
    assert(mfoc_app_Init((uintptr_t)&app, (uintptr_t)&cfg) == MODUS_EFAIL);
    assert(app.state == MFOC_APP_ERROR);
#if MSHELL_ENABLE
    unsigned init_count = s_init_count;
    unsigned run_count = s_run_count;
    shell_commands();
    assert(s_init_count == init_count && s_run_count == run_count);
#endif
    puts("Application Init order, Clock binding and failures passed");
    return 0;
}
