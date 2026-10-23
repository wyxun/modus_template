/**
 * @file mfoc_feedback.c
 * @brief Compile-time position selection and foreground/ISR dispatch.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_feedback.h"
#include "mfoc_modulation.h"
#include <stddef.h>

/** @brief Initialize enabled algorithms without runtime callbacks.
 * @param feedback Instance. @param cfg Settings. @param model Motor data.
 * @return Initialization status; failure preserves the destination. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, const mfoc_motor_params_t *model)
{
    if (feedback == NULL || cfg == NULL || model == NULL) {
        return MFOC_RESULT_NULL;
    }
    mfoc_feedback_t next = {0};
#if MFOC_FEEDBACK_ENABLE_ENCODER
    mfoc_result_t rc = mfoc_encoder_Init(&next.encoder, &cfg->encoder,
                                         model->pole_pairs);
    if (rc != MFOC_RESULT_OK) { return rc; }
    next.encoder_status = MFOC_RESULT_INVALID_STATE;
#endif
#if MFOC_FEEDBACK_ENABLE_SMO
    mfoc_result_t smo_rc = mfoc_smo_Init(&next.smo, &cfg->smo, model);
    if (smo_rc != MFOC_RESULT_OK) { return smo_rc; }
    next.smo_status = MFOC_RESULT_INVALID_STATE;
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

/** @brief Commit the selected encoder zero while the rotor is held.
 * @param feedback Instance. @param angle Held electrical angle.
 * @return Sampling/configuration status. No ISR work is added. */
mfoc_result_t mfoc_feedback_CalibrateZero(mfoc_feedback_t *feedback,
    mfoc_angle_t angle)
{
    if (feedback == NULL) { return MFOC_RESULT_NULL; }
    if (!feedback->initialized) { return MFOC_RESULT_INVALID_STATE; }
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
    return mfoc_encoder_CalibrateZero(&feedback->encoder, angle);
#else
    (void)angle;
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
#if MFOC_FEEDBACK_ENABLE_SMO
    mfoc_ab_t voltage = {0};
    feedback->smo_status = MFOC_RESULT_INVALID_STATE;
    if (input != NULL && input->i_ab_k != NULL &&
        input->previous_core_output != NULL) {
        const mfoc_core_output_t *previous = input->previous_core_output;
        feedback->smo_status = mfoc_modulation_ModelVoltageAlphaBeta(
            &previous->duty, previous->bus, &voltage);
        if (feedback->smo_status == MFOC_RESULT_OK) {
            mfoc_smo_Step(&feedback->smo, input->i_ab_k, &voltage,
                           &feedback->smo_output);
        }
    }
    if (feedback->smo_status != MFOC_RESULT_OK) {
        mfoc_smo_Reset(&feedback->smo);
        feedback->smo_output = (mfoc_smo_output_t){0};
    }
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
    if (feedback->encoder_status != MFOC_RESULT_OK) {
        return feedback->encoder_status;
    }
    *output = encoder_output;
    return MFOC_RESULT_OK;
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
    if (feedback->smo_status != MFOC_RESULT_OK) {
        return feedback->smo_status;
    }
    if (!feedback->smo_output.valid) { return MFOC_RESULT_INVALID_STATE; }
    output->angle = feedback->smo_output.angle;
    output->speed = feedback->smo_output.speed;
    return MFOC_RESULT_OK;
#else
    (void)output;
    return MFOC_RESULT_INVALID_STATE;
#endif
}
