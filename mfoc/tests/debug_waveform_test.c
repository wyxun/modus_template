/**
 * @file debug_waveform_test.c
 * @brief Full-carrier diagnostic latching across both decimation phases.
 * @author Codex
 * @date 2026-10-24
 */
#include "../../class/mfoc_debug.c"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
mfoc_app_t tMfocApp;
static const char *s_names[4];
static float *s_values[4];
static float s_sample[4];
static unsigned s_channels;
static unsigned s_steps;
static bool s_started;
static mfoc_result_t s_encoder_result;
static unsigned s_encoder_calls;

/** @brief Reset fake registration. @param protocol Unused protocol.
 * @return OK. */
static int init(const mwaveform_protocol_t *protocol)
{
    (void)protocol;
    s_channels = 0U;
    s_started = false;
    return MODUS_SUCCESS;
}

/** @brief Capture registration and enforce four bound float channels.
 * @param name Name. @param scale Transport scale. @param value Bound value.
 * @param type Numeric format. @return Next channel ID. */
static uint8_t add(const char *name, float scale, void *value, uint8_t type)
{
    assert(s_channels < 4U);
#if MFOC_FEEDBACK_ENABLE_HFI
    assert(scale == 10000.0f);
#else
    assert(scale == 1000.0f);
#endif
    assert(strlen(name) <= 7U); /* Wire descriptor reserves eight bytes. */
    assert(type == MWAVEFORM_VAR_FLOAT);
    uint8_t index = (uint8_t)s_channels;
    s_names[index] = name;
    s_values[index] = value;
    ++s_channels;
    return index;
}

/** @brief Verify external ISR drive. @param rate External-drive selector.
 * @return None. */
static void set_rate(uint32_t rate) { assert(rate == 0U); }

/** @brief Verify the existing frequency settings.
 * @param period ISR period ns. @param rate Stream rate Hz.
 * @return Accepted rate. */
static uint32_t set_stream(uint32_t period, uint32_t rate)
{
    assert(period == 50000U && rate == 10000U);
    return rate;
}

/** @brief Record stream startup. @return None. */
static void start(void) { s_started = true; }

/** @brief Simulate both possible 2:1 waveform decimation phases.
 * @return None. */
static void step(void)
{
    ++s_steps;
    if ((s_steps & 1U) == 0U) {
        mfoc_debug_CaptureWaveform();
        for (unsigned j = 0U; j < 4U; ++j) { s_sample[j] = *s_values[j]; }
    }
}

const mwaveform_api_t mwaveform = {.Init = init, .AddVariable = add,
    .SetRate = set_rate, .SetStreamRate = set_stream, .Start = start,
    .Step = step};

/** @brief Supply a cached angle without accessing hardware.
 * @param encoder Unused instance. @param angle Angle. @param speed Speed.
 * @return Injected status. */
mfoc_result_t mfoc_encoder_Step(const mfoc_encoder_t *encoder,
    mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    (void)encoder;
    ++s_encoder_calls;
    *angle = (mfoc_angle_t){0x40000000U};
    *speed = 0.0f;
    return s_encoder_result;
}

#if MFOC_FEEDBACK_ENABLE_HFI
/** @brief Check a latched frame without depending on the encoder.
 * @param expected Four current/voltage PU values. @return None. */
static void assert_frame(const float expected[4])
{
    for (unsigned j = 0U; j < 4U; ++j) {
        assert(fabsf(*s_values[j] - expected[j]) < 0.00001f);
    }
}

/** @brief Verify unavailable markers, rest holds and restart cleanup.
 * @param saved Complete frame. @param expected Its four channel values.
 * @return None. */
static void invalid_states(const mfoc_hfi_t *saved, const float expected[4])
{
    mfoc_hfi_t *hfi = &tMfocApp.motor.feedback.hfi;
    const float invalid[4] = {-3.2f, -3.2f, -3.2f, -3.2f};
    *hfi = *saved;
    tMfocApp.motor.align_active = true;
    mfoc_debug_WaveformStep();
    assert_frame(invalid);
    tMfocApp.motor.align_active = false;
    hfi->state = MFOC_HFI_FAILED;
    mfoc_debug_WaveformStep();
    assert_frame(invalid);
    *hfi = *saved;
    mfoc_debug_WaveformStep();
    assert_frame(expected);
#if MFOC_HFI_ENABLE_POLARITY
    hfi->state = MFOC_HFI_SETTLE_POSITIVE;
    hfi->delta_i[MFOC_DEBUG_HFI_AXIS] = (mfoc_ab_t){1.0f, 1.0f};
    mfoc_debug_WaveformStep();
    assert_frame(expected);
    *hfi = *saved;
#endif
    hfi->startup_count = 1U;
    mfoc_debug_WaveformStep();
    assert_frame(invalid);
    *hfi = *saved;
    hfi->has_current = false;
    mfoc_debug_WaveformStep();
    assert_frame(invalid);
    *hfi = *saved;
    tMfocApp.motor.state = MFOC_MOTOR_IDLE;
    mfoc_debug_WaveformStep();
    assert_frame(invalid);
    assert(s_encoder_calls == 0U);
}

/** @brief Check complete-group pairing under both 2:1 transport phases.
 * @param phase Initial decimation phase. @return None. */
static void full_cycles(unsigned phase)
{
    const float invalid[4] = {-3.2f, -3.2f, -3.2f, -3.2f};
    tMfocApp = (mfoc_app_t){0};
    tMfocApp.motor.state = MFOC_MOTOR_RUNNING;
    mfoc_hfi_t *hfi = &tMfocApp.motor.feedback.hfi;
    hfi->has_current = true;
    hfi->startup_count = 1U;
    mfoc_debug_WaveformStep();
    assert_frame(invalid);
    hfi->startup_count = MFOC_HFI_CARRIER_SAMPLES;
    mfoc_debug_WaveformStep();
    assert_frame(invalid);
    hfi->startup_count = MFOC_HFI_CARRIER_SAMPLES + 1U;
    hfi->delta_i[0] = (mfoc_ab_t){0.02f, 0.004f};
    hfi->delta_i[1] = (mfoc_ab_t){-0.006f, 0.03f};
    hfi->delta_u[0] = (mfoc_ab_t){0.08f, -0.002f};
    hfi->delta_u[1] = (mfoc_ab_t){0.003f, 0.09f};
    const unsigned axis = MFOC_DEBUG_HFI_AXIS;
    const float expected[4] = {hfi->delta_i[axis].qAlpha,
        hfi->delta_i[axis].qBeta, hfi->delta_u[axis].qAlpha,
        hfi->delta_u[axis].qBeta};
    mfoc_hfi_t saved = *hfi;
    s_steps = phase;
    mfoc_debug_WaveformStep();
    assert_frame(expected);
    assert(memcmp(&saved, hfi, sizeof(saved)) == 0);
    for (uint8_t carrier = 1U; carrier < MFOC_HFI_CARRIER_SAMPLES; ++carrier) {
        hfi->carrier = carrier;
        hfi->delta_i[axis] = (mfoc_ab_t){1.0f, -1.0f};
        hfi->delta_u[axis] = (mfoc_ab_t){2.0f, -2.0f};
        mfoc_debug_WaveformStep();
        assert_frame(expected);
        for (unsigned j = 0U; j < 4U; ++j) {
            assert(fabsf(s_sample[j] - expected[j]) < 0.00001f);
        }
    }
    /* Capture callback cannot replace a complete group with partial data,
     * touch observer state, or make encoder availability a prerequisite. */
    mfoc_hfi_t partial = *hfi;
    s_encoder_result = MFOC_RESULT_INVALID_STATE;
    mfoc_debug_CaptureWaveform();
    assert_frame(expected);
    assert(memcmp(&partial, hfi, sizeof(partial)) == 0);
    assert(s_encoder_calls == 0U);
    s_encoder_result = MFOC_RESULT_OK;
    *hfi = saved;
    hfi->state = MFOC_HFI_TRACK;
    hfi->delta_i[axis] = (mfoc_ab_t){-0.015f, 0.0f};
    hfi->delta_u[axis] = (mfoc_ab_t){0.0f, -0.075f};
    const float changed[4] = {-0.015f, 0.0f, 0.0f, -0.075f};
    mfoc_debug_WaveformStep();
    assert_frame(changed);
    hfi->delta_i[axis] = (mfoc_ab_t){0};
    hfi->delta_u[axis] = (mfoc_ab_t){0};
    const float zero[4] = {0};
    mfoc_debug_WaveformStep();
    assert_frame(zero);
    hfi->delta_i[axis] = (mfoc_ab_t){40.0f, -40.0f};
    hfi->delta_u[axis] = (mfoc_ab_t){50.0f, -50.0f};
    const float clipped[4] = {3.1f, -3.1f, 3.1f, -3.1f};
    mfoc_debug_WaveformStep();
    assert_frame(clipped);
    invalid_states(&saved, expected);
}
#endif
#endif

/** @brief Verify registration and full-cycle diagnostics. @return Success. */
int main(void)
{
#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
    mfoc_debug_WaveformInit();
    assert(s_started && s_channels == 4U);
#if MFOC_FEEDBACK_ENABLE_HFI
    assert(strcmp(s_names[0], "DiA_pu") == 0);
    assert(strcmp(s_names[1], "DiB_pu") == 0);
    assert(strcmp(s_names[2], "DuA_pu") == 0);
    assert(strcmp(s_names[3], "DuB_pu") == 0);
    full_cycles(0U);
    full_cycles(1U);
#else
    assert(strcmp(s_names[0], "Enc_mT") == 0);
    assert(strcmp(s_names[1], "SMO_mT") == 0);
    assert(strcmp(s_names[2], "Err_mT") == 0);
    assert(strcmp(s_names[3], "Iq_mpu") == 0);
    tMfocApp.motor.state = MFOC_MOTOR_RUNNING;
    tMfocApp.motor.previous_core_output_valid = true;
    tMfocApp.motor.previous_core_output.i.qQ = 0.02f;
#if MFOC_FEEDBACK_ENABLE_SMO
    tMfocApp.motor.feedback.smo_output.valid = true;
    tMfocApp.motor.feedback.smo_output.angle.wBam32 = 0x60000000U;
#endif
    mfoc_debug_WaveformStep();
    mfoc_debug_CaptureWaveform();
    assert(*s_values[0] == 0.25f && *s_values[3] == 0.02f);
#if MFOC_FEEDBACK_ENABLE_SMO
    assert(*s_values[1] == 0.375f && *s_values[2] == 0.125f);
#endif
#endif
#endif
    puts("Waveform full-carrier latch and four-channel layout passed");
    return 0;
}
