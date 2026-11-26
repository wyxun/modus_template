/**
 * @file mfoc_feedback.c
 * @brief Compile-time position selection and foreground/ISR dispatch.
 * @author wyxun
 * @date 2026-10-21
 */
#include "mfoc_feedback_internal.h"
#include "mfoc_motor_config_internal.h"
#include "mfoc_port.h"
#include <stddef.h>
#include <string.h>

mfoc_result_t mfoc_feedback_Prepare(mfoc_feedback_prepared_t *prepared,
    const mfoc_feedback_cfg_t *cfg, const mfoc_motor_params_t *model)
{
    (void)prepared;
    (void)cfg;
    (void)model;
#if MFOC_FEEDBACK_ENABLE_SMO
    mfoc_result_t smo_rc = mfoc_smo_Init(&prepared->smo, &cfg->smo, model);
    if (smo_rc != MFOC_RESULT_OK) { return smo_rc; }
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
    mfoc_result_t flux_rc = mfoc_flux_Init(&prepared->flux, &cfg->flux, model);
    if (flux_rc != MFOC_RESULT_OK) { return flux_rc; }
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_result_t ekf_rc = mfoc_ekf_Init(&prepared->ekf, &cfg->ekf, model);
    if (ekf_rc != MFOC_RESULT_OK) { return ekf_rc; }
#endif
    return MFOC_RESULT_OK;
}

void mfoc_feedback_InitPrepared(mfoc_feedback_t *feedback,
    const mfoc_feedback_prepared_t *prepared, const mfoc_feedback_cfg_t *cfg)
{
    (void)prepared;
    (void)cfg;
#if MFOC_FEEDBACK_ENABLE_SMO
    feedback->smo = prepared->smo;
    feedback->smo_cfg = cfg->smo;
    feedback->smo_status = MFOC_RESULT_INVALID_STATE;
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    mfoc_hfi_Init(&feedback->hfi);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_HFI
    feedback->normal_hfi_mode = MFOC_FEEDBACK_HFI_INJECT;
#else
    feedback->normal_hfi_mode = cfg->observe_hfi ? MFOC_FEEDBACK_HFI_OBSERVE :
                                                  MFOC_FEEDBACK_HFI_OFF;
#endif
    feedback->hfi_mode = feedback->normal_hfi_mode;
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
    feedback->flux = prepared->flux;
    feedback->flux_cfg = cfg->flux;
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
    feedback->ekf = prepared->ekf;
    feedback->ekf_cfg = cfg->ekf;
#endif
    feedback->initialized = true;
}

/** @brief Initialize enabled algorithms without runtime callbacks.
 * @param feedback Instance. @param cfg Settings. @param model Motor data.
 * @return Initialization status; failure preserves the destination. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, const mfoc_motor_params_t *model)
{
    if (feedback == NULL || cfg == NULL || model == NULL) {
        return MFOC_RESULT_NULL;
    }
    mfoc_feedback_prepared_t prepared = {0};
    mfoc_result_t rc = mfoc_feedback_Prepare(&prepared, cfg, model);
    if (rc != MFOC_RESULT_OK) { return rc; }
    memset(feedback, 0, sizeof(*feedback));
    mfoc_feedback_InitPrepared(feedback, &prepared, cfg);
    return MFOC_RESULT_OK;
}

/** @brief Refresh model-dependent observers without resetting Port position/HFI.
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
    mfoc_smo_t smo = {0};
    mfoc_result_t rc = mfoc_smo_Init(&smo, &feedback->smo_cfg, model);
    if (rc != MFOC_RESULT_OK) { return rc; }
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
    mfoc_flux_t flux = {0};
    mfoc_result_t flux_rc = mfoc_flux_Init(&flux, &feedback->flux_cfg, model);
    if (flux_rc != MFOC_RESULT_OK) { return flux_rc; }
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_ekf_t ekf = {0};
    mfoc_result_t ekf_rc = mfoc_ekf_Init(&ekf, &feedback->ekf_cfg, model);
    if (ekf_rc != MFOC_RESULT_OK) { return ekf_rc; }
#endif
    /* Commit only after every enabled model has accepted the parameters. */
#if MFOC_FEEDBACK_ENABLE_SMO
    feedback->smo = smo;
    feedback->smo_output = (mfoc_smo_output_t){0};
    feedback->smo_status = MFOC_RESULT_INVALID_STATE;
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
    feedback->flux = flux;
    feedback->flux_output = (mfoc_flux_output_t){0};
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
    feedback->ekf = ekf;
    feedback->ekf_output = (mfoc_ekf_output_t){0};
#endif
    return MFOC_RESULT_OK;
}

/** @brief Start HFI injection for HFI control or physical-position comparison.
 * @param feedback Initialized, stopped instance. @return Capability status. */
mfoc_result_t mfoc_feedback_HfiStart(mfoc_feedback_t *feedback)
{
#if MFOC_FEEDBACK_ENABLE_HFI && (MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_HFI || \
    MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION)
    feedback->hfi_mode = MFOC_FEEDBACK_HFI_INJECT;
    mfoc_hfi_Init(&feedback->hfi);
    feedback->hfi_output = (mfoc_hfi_output_t){0};
    return MFOC_RESULT_OK;
#else
    (void)feedback;
    return MFOC_RESULT_INVALID_STATE;
#endif
}

#if MFOC_FEEDBACK_ENABLE_HFI
/** @brief Advance HFI or reset its history. @return True for a sampled frame. */
static inline bool _mfoc_feedback_StepHfi(mfoc_feedback_t *feedback,
    const mfoc_ab_t *current, const mfoc_ab_t *voltage)
{
    if (current == NULL) {
        mfoc_hfi_Init(&feedback->hfi);
        feedback->hfi_output = (mfoc_hfi_output_t){0};
        return false;
    }
    mfoc_hfi_Step(&feedback->hfi, current, voltage, &feedback->hfi_output);
    return true;
}
#endif

/** @brief Execute enabled algorithms before selecting the control feedback.
 * @param feedback Instance. @param input Samples for model-based algorithms.
 * @param output Electrical feedback. @return Snapshot/state status. */
mfoc_result_t mfoc_feedback_Step(mfoc_feedback_t *feedback,
    const mfoc_feedback_input_t *input, mfoc_feedback_output_t *output)
{
    (void)input;
    (void)feedback;
    mfoc_feedback_output_t next = {0};
#if MFOC_FEEDBACK_ENABLE_POSITION &&                                         \
    (MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION ||                       \
     MFOC_DEBUG_POSITION_WAVEFORM)
    mfoc_result_t position_status = mfoc_port_position(
        &next.angle, &next.speed);
    (void)position_status;
#endif
#if MFOC_FEEDBACK_ENABLE_SMO || MFOC_FEEDBACK_ENABLE_HFI ||                     \
    MFOC_FEEDBACK_ENABLE_FLUX || MFOC_FEEDBACK_ENABLE_EKF
    const mfoc_ab_t *current = input != NULL ? input->i_ab_k : NULL;
    const mfoc_ab_t *voltage = current != NULL ? input->u_ab_previous : NULL;
#endif
#if MFOC_FEEDBACK_ENABLE_SMO
    if (voltage != NULL) {
        mfoc_smo_Step(&feedback->smo, current, voltage, &feedback->smo_output);
        feedback->smo_status = MFOC_RESULT_OK;
    } else {
        mfoc_smo_Reset(&feedback->smo);
        feedback->smo_output = (mfoc_smo_output_t){0};
        feedback->smo_status = MFOC_RESULT_INVALID_STATE;
    }
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
    if (voltage != NULL) {
        mfoc_flux_Step(&feedback->flux, current, voltage,
                       &feedback->flux_output);
    } else {
        mfoc_flux_Reset(&feedback->flux);
        feedback->flux_output = (mfoc_flux_output_t){0};
    }
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
    if (voltage != NULL) {
        mfoc_ekf_Step(&feedback->ekf, current, voltage, &feedback->ekf_output);
    } else {
        mfoc_ekf_Reset(&feedback->ekf);
        feedback->ekf_output = (mfoc_ekf_output_t){0};
    }
#endif
#if MFOC_FEEDBACK_ENABLE_HFI && MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_HFI
    if (feedback->hfi_mode != MFOC_FEEDBACK_HFI_OFF) {
        (void)_mfoc_feedback_StepHfi(feedback, current, voltage);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
        if (feedback->hfi_mode == MFOC_FEEDBACK_HFI_INJECT) {
            next.u_inject_ab = feedback->hfi_output.u_inject_ab;
        }
#endif
    }
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
    if (position_status != MFOC_RESULT_OK) {
        return position_status;
    }
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
    if (!feedback->smo_output.valid) { return MFOC_RESULT_INVALID_STATE; }
    next.angle = feedback->smo_output.angle;
    next.speed = feedback->smo_output.speed;
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_HFI
    if (feedback->hfi_mode != MFOC_FEEDBACK_HFI_INJECT) {
        return MFOC_RESULT_INVALID_STATE;
    }
    const bool sampled = _mfoc_feedback_StepHfi(feedback, current, voltage);
    next.u_inject_ab = feedback->hfi_output.u_inject_ab;
    if (!sampled || feedback->hfi.state == MFOC_HFI_FAILED) {
        return MFOC_RESULT_INVALID_STATE;
    }
    next.acquiring = !feedback->hfi_output.valid;
    next.angle = feedback->hfi_output.angle;
    next.speed = feedback->hfi_output.speed;
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_FLUX
    if (!feedback->flux_output.valid) { return MFOC_RESULT_INVALID_STATE; }
    next.angle = feedback->flux_output.angle;
    next.speed = feedback->flux_output.speed;
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_EKF
    if (!feedback->ekf_output.valid) { return MFOC_RESULT_INVALID_STATE; }
    next.angle = feedback->ekf_output.angle;
    next.speed = feedback->ekf_output.speed;
#else
    return MFOC_RESULT_INVALID_STATE;
#endif
    *output = next;
    return MFOC_RESULT_OK;
}
