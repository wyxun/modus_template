/**
 * @file mfoc_feedback.h
 * @brief Independent position algorithms and control feedback selection.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_FEEDBACK_H
#define MFOC_FEEDBACK_H

#include "mfoc_core.h"
#include "mfoc_encoder.h"

#define MFOC_FEEDBACK_NONE 0
#define MFOC_FEEDBACK_ENCODER 1
#ifndef MFOC_FEEDBACK_ENABLE_ENCODER
#define MFOC_FEEDBACK_ENABLE_ENCODER 1
#endif
#ifndef MFOC_FEEDBACK_BACKEND
#define MFOC_FEEDBACK_BACKEND MFOC_FEEDBACK_ENCODER
#endif
#if MFOC_FEEDBACK_ENABLE_ENCODER != 0 && MFOC_FEEDBACK_ENABLE_ENCODER != 1
#error "MFOC_FEEDBACK_ENABLE_ENCODER must be 0 or 1"
#endif
#if MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_NONE &&                              \
    MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_ENCODER
#error "MFOC_FEEDBACK_BACKEND must select an implemented position algorithm"
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER &&                           \
    !MFOC_FEEDBACK_ENABLE_ENCODER
#error "The selected encoder output requires MFOC_FEEDBACK_ENABLE_ENCODER"
#endif

/** @brief Independent algorithm settings; enable flags control execution. */
typedef struct {
    mfoc_encoder_cfg_t encoder;
} mfoc_feedback_cfg_t;

/** @brief Borrowed samples and prior interval values for one Step. */
typedef struct {
    const mfoc_ab_t *i_ab_k; /**< Current sample i[k], alpha-beta I/Ibase. */
    /** Core output from step k-1; NULL before the first successful Step. */
    const mfoc_core_output_t *previous_core_output;
} mfoc_feedback_input_t;

/** @brief Electrical feedback, valid only when Step returns OK. */
typedef struct {
    mfoc_angle_t angle;   /**< Electrical BAM32. */
    mfoc_scalar_t speed;  /**< omega_e/omega_base. */
} mfoc_feedback_output_t;

/** @brief Instance-owned algorithm states, independent of output selection. */
typedef struct {
#if MFOC_FEEDBACK_ENABLE_ENCODER
    mfoc_encoder_t encoder;
    mfoc_result_t encoder_status; /**< Last ISR encoder sample status. */
#endif
    bool initialized;
} mfoc_feedback_t;

/** @brief Initialize enabled algorithms before enabling the ISR reader.
 * @param feedback Instance. @param cfg Algorithm configurations.
 * @param pole_pairs Motor pole-pair count. @return Initialization status. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, uint16_t pole_pairs);

/** @brief Service enabled foreground algorithms at their own cadences.
 * @param feedback Instance. @return Sampling/state status.
 * @note Call repeatedly while idle or running; do not call from ISR. */
mfoc_result_t mfoc_feedback_Run(mfoc_feedback_t *feedback);

/** @brief Calibrate the selected encoder against a held electrical angle.
 * @param feedback Initialized instance. @param angle Held electrical angle.
 * @return Calibration status; NONE has no position reference to calibrate.
 * @note Foreground only while Motor uses fixed-angle alignment. */
mfoc_result_t mfoc_feedback_CalibrateZero(mfoc_feedback_t *feedback,
    mfoc_angle_t angle);

/** @brief Run enabled ISR algorithms, then select the configured output.
 * @param feedback Instance. @param input Current/prior Core samples; encoder
 * ignores this argument, which may be NULL for that backend.
 * @param output Electrical feedback, unchanged on failure.
 * @return Backend status; NONE returns INVALID_STATE. No I2C access here.
 * @note Valid initialized feedback and output pointers are preconditions. */
mfoc_result_t mfoc_feedback_Step(mfoc_feedback_t *feedback,
    const mfoc_feedback_input_t *input, mfoc_feedback_output_t *output);

#endif
