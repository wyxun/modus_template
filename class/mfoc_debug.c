/**
 * @file mfoc_debug.c
 * @brief MFOC angle and current waveform diagnostics.
 * @author Codex
 * @date 2026-10-22
 */
#include "mfoc_debug.h"

#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
#include "mdebug/mwaveform.h"
#include "mdebug/util_debug.h"
#include <stddef.h>

#define MFOC_DEBUG_WAVEFORM_PERIOD_NS 50000U
#define MFOC_DEBUG_WAVEFORM_RATE_HZ 10000U
#define MFOC_DEBUG_WAVEFORM_SCALE 1000.0f
#define MFOC_DEBUG_CHANNEL_INVALID 0xFFU

enum {
    MFOC_DEBUG_ENCODER_ANGLE = 0,
    MFOC_DEBUG_CONTROL_ANGLE,
    MFOC_DEBUG_ANGLE_ERROR,
    MFOC_DEBUG_IQ,
    MFOC_DEBUG_CHANNEL_COUNT,
};

static float s_waveform[MFOC_DEBUG_CHANNEL_COUNT];
extern mfoc_app_t tMfocApp;

void mfoc_debug_CaptureWaveform(void)
{
    mfoc_motor_t *motor = &tMfocApp.motor;
    mfoc_angle_t encoder_angle = {0U};
    mfoc_scalar_t speed = MFOC_ZERO;
    s_waveform[MFOC_DEBUG_ENCODER_ANGLE] = -1.0f;
    s_waveform[MFOC_DEBUG_CONTROL_ANGLE] = -1.0f;
    s_waveform[MFOC_DEBUG_ANGLE_ERROR] = -1.0f;
    s_waveform[MFOC_DEBUG_IQ] = -1.0f;
    if (motor->state != MFOC_MOTOR_RUNNING) { return; }

    mfoc_angle_t control_angle = motor->align_angle;
    if (motor->align_active) {
        s_waveform[MFOC_DEBUG_CONTROL_ANGLE] =
            mfoc_angle_to_turns(control_angle);
        s_waveform[MFOC_DEBUG_IQ] =
            mfoc_to_float(motor->previous_core_output.i.qQ);
    }
    mfoc_result_t rc = mfoc_encoder_Step(&motor->feedback.encoder,
        &encoder_angle, &speed);
    if (rc != MFOC_RESULT_OK) { return; }
    if (!motor->align_active) { control_angle = encoder_angle; }
    s_waveform[MFOC_DEBUG_ENCODER_ANGLE] =
        mfoc_angle_to_turns(encoder_angle);
    s_waveform[MFOC_DEBUG_CONTROL_ANGLE] =
        mfoc_angle_to_turns(control_angle);
    s_waveform[MFOC_DEBUG_ANGLE_ERROR] =
        mfoc_to_float(mfoc_angle_diff(control_angle, encoder_angle));
    s_waveform[MFOC_DEBUG_IQ] =
        mfoc_to_float(motor->previous_core_output.i.qQ);
}

void mfoc_debug_WaveformInit(void)
{
    static const char *const names[MFOC_DEBUG_CHANNEL_COUNT] = {
        "Enc_mT", "Ctrl_mT", "Err_mT", "Iq_mpu"};
    int rc = mwaveform.Init(NULL);
    if (rc != MODUS_SUCCESS) {
        MLOGF(W, "MFOC waveform init failed (%d)\r\n", rc);
        return;
    }
    for (uint8_t index = 0U; index < MFOC_DEBUG_CHANNEL_COUNT; ++index) {
        if (mwaveform.AddVariable(names[index], MFOC_DEBUG_WAVEFORM_SCALE,
            &s_waveform[index], MWAVEFORM_VAR_FLOAT) ==
            MFOC_DEBUG_CHANNEL_INVALID) {
            MLOGF(W, "MFOC waveform channel registration failed\r\n");
            return;
        }
    }
    mwaveform.SetRate(0U);
    uint32_t rate = mwaveform.SetStreamRate(
        MFOC_DEBUG_WAVEFORM_PERIOD_NS, MFOC_DEBUG_WAVEFORM_RATE_HZ);
    if (rate != MFOC_DEBUG_WAVEFORM_RATE_HZ) {
        MLOGF(W, "MFOC waveform rate unavailable (%lu Hz)\r\n",
            (unsigned long)rate);
        return;
    }
    mwaveform.Start();
    MLOGF(I, "MFOC waveform %lu Hz, 4 channels\r\n",
        (unsigned long)rate);
}

void mfoc_debug_WaveformStep(void)
{
    mwaveform.Step();
}
#endif
