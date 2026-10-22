/**
 * @file mfoc_encoder.h
 * @brief Foreground encoder sampling and ISR electrical feedback.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_ENCODER_H
#define MFOC_ENCODER_H

#include "mfoc_angle.h"

/** @brief Encoder configuration on the Motor's electrical speed base.
 * electrical_zero is subtracted after direction and pole-pair conversion.
 * speed_filter_alpha is in [0,1]; 1 disables speed filtering.
 * Sample often enough to resolve less than half a mechanical turn.
 * The supplied zero must already be calibrated; Init does not align a rotor. */
typedef struct {
    mfoc_scalar_t electrical_speed_base_hz;
    mfoc_scalar_t speed_filter_alpha;
    mfoc_angle_t electrical_zero;
    uint32_t sample_period_us;
    uint32_t timeout_ms;
    bool inverted;
} mfoc_encoder_cfg_t;

/** @brief One published mechanical sample, with read-completion timestamp. */
typedef struct {
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /**< Mechanical turns/s, before PU conversion. */
    mfoc_scalar_t speed_pu; /**< Electrical speed PU, prepared by Run. */
    uint32_t tick;
} mfoc_encoder_sample_t;

/** @brief Single foreground writer and higher-priority ISR reader.
 * Volatile slots are fully written before the byte index is published.
 * The reader must complete before foreground resumes; no concurrent cores,
 * DMA writers, or ISR calls to Run are supported. Init precedes IRQ enable.
 * Tick arithmetic uses low 32 bits; service within one full clock wrap. */
typedef struct {
    volatile mfoc_encoder_sample_t samples[2];
    volatile uint8_t published; /**< 0/1 slot index, 2 means no sample. */
    mfoc_scalar_t speed_gain;
    mfoc_scalar_t filter_alpha;
    mfoc_angle_t electrical_zero;
    uint32_t tick_hz;
    uint32_t period_ticks;
    uint32_t timeout_ticks;
    uint32_t last_poll_tick;
    uint16_t pole_pairs;
    bool inverted;
} mfoc_encoder_t;

/** @brief Initialize coefficients; failure preserves the destination.
 * @param encoder Instance. @param cfg Encoder settings.
 * @param pole_pairs Motor pole-pair count. @return Configuration status. */
mfoc_result_t mfoc_encoder_Init(mfoc_encoder_t *encoder,
    const mfoc_encoder_cfg_t *cfg, uint16_t pole_pairs);

/** @brief Read and publish when the sample period elapses; no delay loop.
 * @param encoder Instance. @return OK, INVALID_STATE, or port read error.
 * @note Foreground only. Read errors retain the prior snapshot; Step rejects
 * it after timeout. First Run reads immediately and publishes zero speed. */
mfoc_result_t mfoc_encoder_Run(mfoc_encoder_t *encoder);

/** @brief Capture electrical zero from a rotor held at a known angle.
 * @param encoder Initialized instance. @param angle Held electrical angle.
 * @return Sensor/state status; failed reads preserve the calibration.
 * @note Foreground only, while Step is excluded or uses a fixed angle.
 * One fresh read; direction must already match the phase sequence.
 * Clears velocity history to avoid extrapolating the held position. */
mfoc_result_t mfoc_encoder_CalibrateZero(mfoc_encoder_t *encoder,
    mfoc_angle_t angle);

/** @brief Copy fresh feedback, extrapolating angle to the current tick.
 * @param encoder Instance. @param angle Electrical BAM32 output.
 * @param speed Electrical speed PU output. @return OK or sample/state error.
 * @note Valid pointers and initialized encoder are required. ISR only;
 * speed is copied from Run's snapshot without conversion.
 * No hardware bus access. Failure preserves both outputs. */
mfoc_result_t mfoc_encoder_Step(const mfoc_encoder_t *encoder,
    mfoc_angle_t *angle, mfoc_scalar_t *speed);

#endif
