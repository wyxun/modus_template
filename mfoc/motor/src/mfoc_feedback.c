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
    next.smo_cfg = cfg->smo;
    next.smo_status = MFOC_RESULT_INVALID_STATE;
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    mfoc_hfi_Init(&next.hfi);
#endif
    next.initialized = true;
    *feedback = next;
    return MFOC_RESULT_OK;
}

/** @brief Refresh SMO coefficients without resetting encoder/HFI state.
 * @param feedback Stopped instance. @param model New motor model.
 * @return OK or rejected model; previous coefficients survive rejection. */
mfoc_result_t mfoc_feedback_SetMotorParamModel(mfoc_feedback_t *feedback,
    const mfoc_motor_params_t *model)
{
    if (feedback == NULL || model == NULL) { return MFOC_RESULT_NULL; }
    if (!feedback->initialized) { return MFOC_RESULT_INVALID_STATE; }
    if (model->pole_pairs == 0U ||
        model->rs < MFOC_SCALAR(MFOC_MOTOR_RS_MIN_PU) ||
        model->rs > MFOC_SCALAR(MFOC_MOTOR_RS_MAX_PU) ||
        model->ld < MFOC_SCALAR(MFOC_MOTOR_L_MIN_PU) ||
        model->ld > MFOC_SCALAR(MFOC_MOTOR_L_MAX_PU) ||
        model->lq < MFOC_SCALAR(MFOC_MOTOR_L_MIN_PU) ||
        model->lq > MFOC_SCALAR(MFOC_MOTOR_L_MAX_PU) ||
        model->flux <= MFOC_ZERO ||
        model->flux > MFOC_SCALAR(MFOC_MOTOR_FLUX_MAX_PU) ||
        model->inertia < MFOC_ZERO ||
        model->inertia > MFOC_SCALAR(MFOC_MOTOR_INERTIA_MAX_PU)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#if MFOC_FEEDBACK_ENABLE_SMO
    mfoc_result_t rc = mfoc_smo_Init(&feedback->smo, &feedback->smo_cfg, model);
    if (rc != MFOC_RESULT_OK) { return rc; }
    feedback->smo_output = (mfoc_smo_output_t){0};
    feedback->smo_status = MFOC_RESULT_INVALID_STATE;
#else
    (void)feedback;
    (void)model;
#endif
    return MFOC_RESULT_OK;
}

/** @brief Advance enabled foreground services without ISR bus access.
 * @param feedback Instance. @return Service status. */
mfoc_result_t mfoc_feedback_Run(mfoc_feedback_t *feedback)
{
    if (feedback == NULL) { return MFOC_RESULT_NULL; }
    if (!feedback->initialized) { return MFOC_RESULT_INVALID_STATE; }
#if MFOC_FEEDBACK_ENABLE_ENCODER
    mfoc_result_t rc = mfoc_encoder_Run(&feedback->encoder);
    return rc;
#else
    return MFOC_RESULT_OK;
#endif
}

/** @brief Reset HFI and acquire position independently of backend selection.
 * @param feedback Initialized, stopped instance. @return Capability status. */
mfoc_result_t mfoc_feedback_HfiStart(mfoc_feedback_t *feedback)
{
#if MFOC_FEEDBACK_ENABLE_HFI
    mfoc_hfi_Init(&feedback->hfi);
    feedback->hfi_output = (mfoc_hfi_output_t){0};
    return MFOC_RESULT_OK;
#else
    (void)feedback;
    return MFOC_RESULT_INVALID_STATE;
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
    mfoc_feedback_output_t next = {0};
#if MFOC_FEEDBACK_ENABLE_ENCODER
    feedback->encoder_status = mfoc_encoder_Step(&feedback->encoder,
        &next.angle, &next.speed);
#else
    (void)feedback;
#endif
#if MFOC_FEEDBACK_ENABLE_SMO || MFOC_FEEDBACK_ENABLE_HFI
    mfoc_ab_t voltage = {0};
    mfoc_result_t voltage_status = MFOC_RESULT_INVALID_STATE;
    bool has_current = input != NULL && input->i_ab_k != NULL;
    if (has_current && input->previous_core_output != NULL) {
        const mfoc_core_output_t *previous = input->previous_core_output;
        voltage_status = mfoc_modulation_ModelVoltageAlphaBeta(
            &previous->duty, previous->bus, &voltage);
    }
#endif
#if MFOC_FEEDBACK_ENABLE_SMO
    feedback->smo_status = voltage_status;
    if (voltage_status == MFOC_RESULT_OK) {
        mfoc_smo_Step(&feedback->smo, input->i_ab_k, &voltage,
                       &feedback->smo_output);
    } else {
        mfoc_smo_Reset(&feedback->smo);
        feedback->smo_output = (mfoc_smo_output_t){0};
    }
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    const mfoc_ab_t *u_previous = voltage_status == MFOC_RESULT_OK ?
        &voltage : NULL;
    feedback->hfi_output = (mfoc_hfi_output_t){0};
    if (has_current) {
        mfoc_hfi_Step(&feedback->hfi, input->i_ab_k, u_previous,
                       &feedback->hfi_output);
    } else {
        mfoc_hfi_Init(&feedback->hfi);
    }
    next.u_inject_ab = feedback->hfi_output.u_inject_ab;
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
    if (feedback->encoder_status != MFOC_RESULT_OK) {
        return feedback->encoder_status;
    }
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
    if (feedback->smo_status != MFOC_RESULT_OK) {
        return feedback->smo_status;
    }
    if (!feedback->smo_output.valid) { return MFOC_RESULT_INVALID_STATE; }
    next.angle = feedback->smo_output.angle;
    next.speed = feedback->smo_output.speed;
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_HFI
    if (!has_current || feedback->hfi.state == MFOC_HFI_FAILED) {
        return MFOC_RESULT_INVALID_STATE;
    }
    next.acquiring = !feedback->hfi_output.valid;
    next.angle = feedback->hfi_output.angle;
    next.speed = feedback->hfi_output.speed;
#else
    return MFOC_RESULT_INVALID_STATE;
#endif
    *output = next;
    return MFOC_RESULT_OK;
}
