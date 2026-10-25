/**
 * @file mfoc_debug.c
 * @brief MFOC complete-carrier demodulation and angle diagnostics.
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
#if MFOC_FEEDBACK_ENABLE_HFI
/* 0.0001 PU transport resolution, below the current ADC's 1/476 PU/count. */
#define MFOC_DEBUG_WAVEFORM_SCALE 10000.0f
#else
#define MFOC_DEBUG_WAVEFORM_SCALE 1000.0f
#endif
#define MFOC_DEBUG_CHANNEL_INVALID 0xFFU

enum {
#if MFOC_FEEDBACK_ENABLE_HFI
    MFOC_DEBUG_DI_ALPHA = 0,
    MFOC_DEBUG_DI_BETA,
    MFOC_DEBUG_DU_ALPHA,
    MFOC_DEBUG_DU_BETA,
#else
    MFOC_DEBUG_ENCODER_ANGLE = 0,
    MFOC_DEBUG_OBSERVER_ANGLE,
    MFOC_DEBUG_ANGLE_ERROR,
    MFOC_DEBUG_IQ,
#endif
    MFOC_DEBUG_CHANNEL_COUNT,
};

static float s_waveform[MFOC_DEBUG_CHANNEL_COUNT];
extern mfoc_app_t tMfocApp;

void mfoc_debug_CaptureWaveform(void)
{
#if !MFOC_FEEDBACK_ENABLE_HFI
    mfoc_motor_t *motor = &tMfocApp.motor;
    s_waveform[MFOC_DEBUG_ENCODER_ANGLE] = -1.0f;
    s_waveform[MFOC_DEBUG_OBSERVER_ANGLE] = -1.0f;
    s_waveform[MFOC_DEBUG_ANGLE_ERROR] = -1.0f;
    s_waveform[MFOC_DEBUG_IQ] = -1.0f;
    if (motor->state != MFOC_MOTOR_RUNNING) { return; }

    if (motor->previous_core_output_valid) {
        s_waveform[MFOC_DEBUG_IQ] =
            mfoc_to_float(motor->previous_core_output.i.qQ);
    }
#if MFOC_FEEDBACK_ENABLE_SMO
    mfoc_angle_t observer_angle = {0U};
    observer_angle = motor->feedback.smo_output.angle;
    bool observer_available = !motor->align_active &&
        motor->feedback.smo_status == MFOC_RESULT_OK &&
        motor->feedback.smo_output.valid;
    if (observer_available) {
        s_waveform[MFOC_DEBUG_OBSERVER_ANGLE] =
            mfoc_angle_to_turns(observer_angle);
    }
#endif
#if MFOC_FEEDBACK_ENABLE_ENCODER
    mfoc_angle_t encoder_angle = {0U};
    mfoc_scalar_t speed = MFOC_ZERO;
    mfoc_result_t rc = mfoc_encoder_Step(&motor->feedback.encoder,
        &encoder_angle, &speed);
    if (rc != MFOC_RESULT_OK) { return; }
    s_waveform[MFOC_DEBUG_ENCODER_ANGLE] =
        mfoc_angle_to_turns(encoder_angle);
#if MFOC_FEEDBACK_ENABLE_SMO
    if (observer_available) {
        s_waveform[MFOC_DEBUG_ANGLE_ERROR] = mfoc_to_float(
            mfoc_angle_diff(observer_angle, encoder_angle));
    }
#endif
#endif
#endif
}

void mfoc_debug_WaveformInit(void)
{
    static const char *const names[MFOC_DEBUG_CHANNEL_COUNT] = {
#if MFOC_FEEDBACK_ENABLE_HFI
        "DiA_pu", "DiB_pu", "DuA_pu", "DuB_pu"};
#else
        "Enc_mT", "SMO_mT", "Err_mT", "Iq_mpu"};
#endif
    int rc = mwaveform.Init(NULL);
    if (rc != MODUS_SUCCESS) {
        MLOGF(W, "MFOC waveform init failed (%d)\r\n", rc);
        return;
    }
    for (uint8_t index = 0U; index < MFOC_DEBUG_CHANNEL_COUNT; ++index) {
#if MFOC_FEEDBACK_ENABLE_HFI
        s_waveform[index] = -3.2f;
#else
        s_waveform[index] = -1.0f;
#endif
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
#if MFOC_FEEDBACK_ENABLE_HFI
    MLOGF(I, "HFI Di/Du excitation=%s, PU, unavailable=-3.2\r\n",
        MFOC_DEBUG_HFI_AXIS == 0 ? "alpha" : "beta");
#endif
}

void mfoc_debug_WaveformStep(void)
{
#if MFOC_FEEDBACK_ENABLE_HFI
    const mfoc_motor_t *motor = &tMfocApp.motor;
    const mfoc_hfi_t *hfi = &motor->feedback.hfi;
    if (motor->state != MFOC_MOTOR_RUNNING || motor->align_active ||
        !hfi->has_current ||
        hfi->startup_count <= MFOC_HFI_CARRIER_SAMPLES ||
        hfi->state == MFOC_HFI_FAILED) {
        s_waveform[MFOC_DEBUG_DI_ALPHA] = -3.2f;
        s_waveform[MFOC_DEBUG_DI_BETA] = -3.2f;
        s_waveform[MFOC_DEBUG_DU_ALPHA] = -3.2f;
        s_waveform[MFOC_DEBUG_DU_BETA] = -3.2f;
    } else if (hfi->carrier == 0U &&
               (hfi->state == MFOC_HFI_ACQUIRE ||
                hfi->state == MFOC_HFI_TRACK)) {
        /* Latch before 10 kHz decimation: only every eighth control step
         * contains two complete pulse pairs. Reuse the bound slots. */
        const mfoc_ab_t *i = &hfi->delta_i[MFOC_DEBUG_HFI_AXIS];
        const mfoc_ab_t *u = &hfi->delta_u[MFOC_DEBUG_HFI_AXIS];
        /* Preserve signed PU values; no angle extraction or normalization.
         * Reserve -3.2 for missing data, +/-3.1 for diagnostic clipping. */
        s_waveform[MFOC_DEBUG_DI_ALPHA] = mfoc_sat(i->qAlpha, -3.1f, 3.1f);
        s_waveform[MFOC_DEBUG_DI_BETA] = mfoc_sat(i->qBeta, -3.1f, 3.1f);
        s_waveform[MFOC_DEBUG_DU_ALPHA] = mfoc_sat(u->qAlpha, -3.1f, 3.1f);
        s_waveform[MFOC_DEBUG_DU_BETA] = mfoc_sat(u->qBeta, -3.1f, 3.1f);
    } else { /* Partial pairs and polarity pauses retain the last full group. */ }
#endif
    mwaveform.Step();
}
#endif
