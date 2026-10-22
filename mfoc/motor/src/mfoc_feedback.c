/**
 * @file mfoc_feedback.c
 * @brief Compile-time position selection and foreground/ISR dispatch.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_feedback.h"
#include <stddef.h>

/** @brief Initialize enabled algorithms without runtime callbacks.
 * @param feedback Instance. @param cfg Settings. @param pole_pairs Motor.
 * @return Initialization status; failure preserves the destination. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, uint16_t pole_pairs)
{
    if (feedback == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    mfoc_feedback_t next = {0};
#if MFOC_FEEDBACK_ENABLE_ENCODER
    mfoc_result_t rc = mfoc_encoder_Init(&next.encoder, &cfg->encoder,
                                         pole_pairs);
    if (rc != MFOC_RESULT_OK) { return rc; }
    next.encoder_status = MFOC_RESULT_INVALID_STATE;
#else
    (void)pole_pairs;
#endif
    next.initialized = true;
    *feedback = next;
    return MFOC_RESULT_OK;
}

/** @brief Advance enabled foreground services without ISR bus access.
 * @param feedback Instance. @return Service status. */
mfoc_result_t mfoc_feedback_Run(mfoc_feedback_t *feedback)
{
    if (feedback == NULL) { return MFOC_RESULT_NULL; }
    if (!feedback->initialized) { return MFOC_RESULT_INVALID_STATE; }
#if MFOC_FEEDBACK_ENABLE_ENCODER
    return mfoc_encoder_Run(&feedback->encoder);
#else
    return MFOC_RESULT_OK;
#endif
}

/** @brief Execute enabled algorithms before selecting the control feedback.
 * @param feedback Instance. @param input Samples for model-based algorithms.
 * @param output Electrical feedback. @return Snapshot/state status. */
mfoc_result_t mfoc_feedback_Step(mfoc_feedback_t *feedback,
    const mfoc_feedback_input_t *input, mfoc_feedback_output_t *output)
{
    (void)input;
#if MFOC_FEEDBACK_ENABLE_ENCODER
    mfoc_feedback_output_t encoder_output = {0};
    feedback->encoder_status = mfoc_encoder_Step(&feedback->encoder,
        &encoder_output.angle, &encoder_output.speed);
#else
    (void)feedback;
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
    if (feedback->encoder_status != MFOC_RESULT_OK) {
        return feedback->encoder_status;
    }
    *output = encoder_output;
    return MFOC_RESULT_OK;
#else
    (void)output;
    return MFOC_RESULT_INVALID_STATE;
#endif
}
