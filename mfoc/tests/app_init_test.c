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

#if MFOC_ISR_MEASURE && !defined(__NO_USE_LOG__)
static int64_t s_ticks;
/** @brief Supply a 170 MHz test clock. @return Current ticks. */
int64_t get_system_ticks(void) { return s_ticks; }
/** @brief Convert milliseconds to test ticks.
 * @param ms Milliseconds. @return Clock ticks. */
int64_t perfc_convert_ms_to_ticks(uint32_t ms)
{
    return (int64_t)ms * 170000;
}
/** @brief Decode test ticks. @param ticks Clock ticks.
 * @return Whole microseconds. */
int64_t perfc_convert_ticks_to_us(int64_t ticks) { return ticks / 170; }
/** @brief Model periodic foreground reporting.
 * @param period Interval. @param stamp Previous report. @param reload Reload.
 * @return Whether the interval expired. */
bool __perfc_is_time_out(int64_t period, int64_t *stamp, bool reload)
{
    if (s_ticks - *stamp < period) { return false; }
    if (reload) { *stamp = s_ticks; }
    return true;
}
#endif

#if MSHELL_ENABLE
static char s_log[512];
static unsigned s_align_count;
static unsigned s_stop_count;
static mfoc_result_t s_align_result;
static unsigned s_encoder_reads;
static uint32_t s_encoder_raw;
static mfoc_result_t s_encoder_result;
static unsigned s_current_count;
static unsigned s_speed_count;
static unsigned s_openloop_count;
static unsigned s_hfi_count;
static unsigned s_start_count;
static mfoc_dq_t s_current_ref;
static mfoc_scalar_t s_speed_ref;
static mfoc_result_t s_current_result;
static mfoc_result_t s_speed_result;
static mfoc_result_t s_start_result;
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

#if MFOC_ISR_MEASURE
/** @brief Check one-second peak windows, overrun clearing and idle silence.
 * @return None. */
static void timing_windows(void)
{
    mfoc_app_t app = {0};
    mfoc_app_isr_timing_t *timing = &app.tIsrTiming;
    timing->lReportPeriodTicks = perfc_convert_ms_to_ticks(1000U);
    timing->wMaximumTicks = MFOC_ISR_BUDGET_TICKS;
    s_log[0] = '\0';
    s_ticks = timing->lReportPeriodTicks - 1;
    _mfoc_app_ReportIsrTiming(&app);
    assert(s_log[0] == '\0');
    assert(timing->wMaximumTicks == MFOC_ISR_BUDGET_TICKS);
    ++s_ticks;
    _mfoc_app_ReportIsrTiming(&app);
    assert(strstr(s_log, "max=8500 ticks (50.0 us)") != NULL);
    assert(strstr(s_log, "overrun") == NULL);
    assert(timing->wMaximumTicks == 0U && !timing->bOverrun);
    timing->wMaximumTicks = MFOC_ISR_BUDGET_TICKS + 170U;
    timing->bOverrun = true;
    s_ticks += timing->lReportPeriodTicks;
    _mfoc_app_ReportIsrTiming(&app);
    assert(strstr(s_log, "max=8670 ticks (51.0 us) (overrun)") != NULL);
    assert(timing->wMaximumTicks == 0U && !timing->bOverrun);
    timing->wMaximumTicks = 170U;
    s_ticks += timing->lReportPeriodTicks;
    _mfoc_app_ReportIsrTiming(&app);
    assert(strstr(s_log, "max=170 ticks (1.0 us)") != NULL);
    assert(strstr(s_log, "overrun") == NULL);
    s_log[0] = '\0';
    s_ticks += timing->lReportPeriodTicks;
    _mfoc_app_ReportIsrTiming(&app);
    assert(s_log[0] == '\0');
}
#endif

/** @brief Exercise parsing and alignment routing at the Motor boundary.
 * @return None. */
static void shell_commands(void)
{
    tMfocApp.state = MFOC_APP_IDLE;
    assert(strcmp(shell_cmd_motor.pchName, "motor") == 0);
    tMfocApp.motor.state = MFOC_MOTOR_IDLE;
    command("current 0.01 -0.02", "motor current result=0");
    assert(s_current_count == 1U && s_start_count == 1U);
    assert(s_current_ref.qD == mfoc_from_float(0.01f));
    assert(s_current_ref.qQ == mfoc_from_float(-0.02f));
    command("current 0 0.02", "motor current result=0");
    assert(s_start_count == 1U && s_current_count == 2U);
    tMfocApp.motor.state = MFOC_MOTOR_IDLE;
    command("speed 0.02", "motor speed result=0");
    assert(s_speed_count == 1U && s_start_count == 2U);
    assert(s_speed_ref == mfoc_from_float(0.02f));
    command("speed -0.03", "motor speed result=0");
    assert(s_speed_count == 2U && s_start_count == 2U);
    assert(fabsf(mfoc_to_float(s_speed_ref) + 0.03f) < 0.0001f);
    command("speed 0.8", "motor speed result=3");
    assert(s_speed_count == 3U && s_start_count == 2U);
    s_current_result = MFOC_RESULT_OUT_OF_RANGE;
    command("current 1 1", "motor current result=3");
    assert(s_start_count == 2U);
    s_current_result = MFOC_RESULT_OK;
    tMfocApp.motor.state = MFOC_MOTOR_IDLE;
    s_start_result = MFOC_RESULT_IO;
    command("current 0 0.02", "motor current result=6");
    assert(s_start_count == 3U);
    s_start_result = MFOC_RESULT_OK;
    tMfocApp.motor.state = MFOC_MOTOR_FAULT;
    command("current 0 0.02", "motor current result=5");
    assert(s_start_count == 3U);
    tMfocApp.motor.state = MFOC_MOTOR_IDLE;
    command("speed -0.1", "motor speed result=0");
    command("openloop 0.1 0.04", "motor openloop result=0");
    assert(s_openloop_count == 1U);
    command("openloop 0.1", "invalid motor arguments");
    command("openloop nan 0.04", "invalid motor arguments");
    command("openloop 0.1 inf", "invalid motor arguments");
    command("openloop 0.1 0.04 trailing", "invalid motor arguments");
    assert(s_openloop_count == 1U);
    tMfocApp.motor.state = MFOC_MOTOR_IDLE;
    command("hfi start", "motor hfi result=0");
    command("hfi", "motor hfi result=5");
    assert(s_hfi_count == 2U);
    command("hfi start 0.04", "invalid motor arguments");
    command("hfi start status", "invalid motor arguments");
    command("hfi off", "invalid motor arguments");
    assert(s_hfi_count == 2U);
    command("hfi status", "hfi state=");
#if MFOC_FEEDBACK_ENABLE_HFI
    tMfocApp.motor.feedback.hfi.startup_count = 17U;
    tMfocApp.motor.feedback.hfi.lock_count = 3U;
    command("hfi status", "steps=17 lock=3");
#if MFOC_ISR_MEASURE
    tMfocApp.tIsrTiming.wMaximumTicks = MFOC_ISR_BUDGET_TICKS + 1U;
    tMfocApp.tIsrTiming.bOverrun = true;
    command("hfi status", "isr_win_max=8501 overrun=1 budget=8500");
    assert(tMfocApp.tIsrTiming.wMaximumTicks == 8501U);
    assert(tMfocApp.tIsrTiming.bOverrun);
#endif
#endif
    command("hfi status trailing", "invalid motor arguments");
    unsigned starts = s_start_count;
#if MFOC_FEEDBACK_ENABLE_FLUX
    tMfocApp.motor.state = MFOC_MOTOR_RUNNING;
    tMfocApp.motor.previous_core_output_valid = true;
    tMfocApp.motor.feedback.flux_output = (mfoc_flux_output_t){
        .angle = {0x40000000U}, .speed = MFOC_HALF, .valid = true};
    command("flux status", "valid=1 angle_mT=250 speed_mpu=500");
    tMfocApp.motor.hold_active = true;
    command("flux", "valid=0");
    tMfocApp.motor.hold_active = false;
    tMfocApp.motor.state = MFOC_MOTOR_IDLE;
    command("flux status", "valid=0");
#if MFOC_ISR_MEASURE
    command("flux status", "isr_win_max=8501 overrun=1 budget=8500");
    assert(tMfocApp.tIsrTiming.wMaximumTicks == 8501U);
#endif
#else
    command("flux status", "flux state=disabled");
#endif
    command("flux status trailing", "invalid motor arguments");
    command("flux start", "invalid motor arguments");
    assert(s_start_count == starts);
    command("align", "motor align result=0");
    command("stop", "motor stop result=0");
    assert(s_align_count == 1U && s_stop_count == 1U);
    s_align_result = MFOC_RESULT_IO;
    command("align", "motor align result=6");
    assert(s_align_count == 2U && tMfocApp.result == MFOC_RESULT_IO);
    unsigned align_count = s_align_count;
    unsigned stop_count = s_stop_count;
    unsigned current_count = s_current_count;
    command("encoder", "encoder raw=0 angle=0.00 deg");
    s_encoder_raw = 2048U;
    command("encoder", "encoder raw=2048 angle=180.00 deg");
    s_encoder_raw = 4095U;
    command("encoder", "encoder raw=4095 angle=359.91 deg");
    s_encoder_result = MFOC_RESULT_IO;
    command("encoder", "encoder read failed result=6");
    s_encoder_result = MFOC_RESULT_OK;
    unsigned reads = s_encoder_reads;
    command("encoder trailing", "invalid motor arguments");
    assert(s_encoder_reads == reads);
    tMfocApp.motor.state = MFOC_MOTOR_FAULT;
    command("clear", "motor clear result=0");
    assert(tMfocApp.motor.state == MFOC_MOTOR_IDLE);
    command("clear", "motor clear result=5");
    command("status", "mfoc app state=");
    command("", "usage: motor");
    command("help", "usage: motor");
    command("current 0.01", "invalid motor arguments");
    command("current nan 0", "invalid motor arguments");
    command("speed inf", "invalid motor arguments");
    command("speed 1.1", "motor speed result=3");
    command("speed 0.1 trailing", "invalid motor arguments");
    command("align trailing", "invalid motor arguments");
    command("stop trailing", "invalid motor arguments");
    command("clear trailing", "invalid motor arguments");
    command("status trailing", "invalid motor arguments");
    assert(s_align_count == align_count && s_stop_count == stop_count);
    assert(s_current_count == current_count);
}

/** @brief Capture current submission and reject invalid lifecycle states.
 * @param motor App Motor. @param ref dq command. @return Test result. */
mfoc_result_t mfoc_motor_SetCurrent(mfoc_motor_t *motor, const mfoc_dq_t *ref)
{
    ++s_current_count;
    if (motor->state != MFOC_MOTOR_IDLE && motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    s_current_ref = *ref;
    return s_current_result;
}

/** @brief Capture a bounded speed request for Shell routing tests.
 * @param motor App Motor. @param ref Electrical speed PU.
 * @return Injected status. */
mfoc_result_t mfoc_motor_SetSpeed(mfoc_motor_t *motor, mfoc_scalar_t ref)
{
    ++s_speed_count;
    if (motor->state != MFOC_MOTOR_IDLE &&
        motor->state != MFOC_MOTOR_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (mfoc_abs(ref) > mfoc_from_float(0.7583333f)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    s_speed_ref = ref;
    return s_speed_result;
}

/** @brief Capture startup without hardware. @param motor Motor.
 * @return Injected result. */
mfoc_result_t mfoc_motor_Start(mfoc_motor_t *motor)
{
    ++s_start_count;
    if (s_start_result == MFOC_RESULT_OK) { motor->state = MFOC_MOTOR_RUNNING; }
    return s_start_result;
}

/** @brief Supply lifecycle snapshot. @param motor Motor.
 * @param status Snapshot output. @return None. */
void mfoc_motor_GetStatus(const mfoc_motor_t *motor, mfoc_motor_status_t *status)
{
    status->state = motor->state;
    status->faults = motor->faults;
}

/** @brief Supply one raw AS5600 sample without hardware.
 * @param angle Mechanical BAM32. @return Injected status. */
mfoc_result_t mfoc_port_encoder(mfoc_angle_t *angle)
{
    ++s_encoder_reads;
    if (s_encoder_result == MFOC_RESULT_OK) {
        angle->wBam32 = s_encoder_raw << 20U;
    }
    return s_encoder_result;
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

/** @brief Simulate fault recovery. @param motor App instance.
 * @return Recovery status. */
mfoc_result_t mfoc_motor_ClearFault(mfoc_motor_t *motor)
{
    if (motor->state != MFOC_MOTOR_FAULT) {
        return MFOC_RESULT_INVALID_STATE;
    }
    motor->state = MFOC_MOTOR_IDLE;
    return MFOC_RESULT_OK;
}

/** @brief Record explicit cancellation. @param motor App instance.
 * @return OK. */
mfoc_result_t mfoc_motor_Stop(mfoc_motor_t *motor)
{
    assert(motor == &tMfocApp.motor);
    ++s_stop_count;
    return MFOC_RESULT_OK;
}

/** @brief Stub sensorless startup for the Shell test link.
 * @param motor App instance. @param speed Target. @param current Current.
 * @return Invalid state. */
mfoc_result_t mfoc_motor_SensorlessStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t current)
{
    (void)motor;
    (void)speed;
    (void)current;
    return MFOC_RESULT_INVALID_STATE;
}

/** @brief Verify the forced startup Shell argument order.
 * @param motor App instance. @param speed Electrical PU. @param id Current.
 * @return OK. */
mfoc_result_t mfoc_motor_OpenLoopStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t id)
{
    assert(motor == &tMfocApp.motor);
    assert(speed == mfoc_from_float(0.1f));
    assert(id == mfoc_from_float(0.04f));
    ++s_openloop_count;
    return MFOC_RESULT_OK;
}

/** @brief Capture HFI startup routing without touching hardware.
 * @param motor App instance. @return State-dependent result. */
mfoc_result_t mfoc_motor_HfiStart(mfoc_motor_t *motor)
{
    ++s_hfi_count;
    if (motor->state != MFOC_MOTOR_IDLE) {
        return MFOC_RESULT_INVALID_STATE;
    }
    motor->state = MFOC_MOTOR_RUNNING;
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
#if MFOC_ISR_MEASURE
    timing_windows();
#endif
#endif
    puts("Application Init order, Clock binding and failures passed");
    return 0;
}
