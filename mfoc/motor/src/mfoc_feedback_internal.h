/**
 * @file mfoc_feedback_internal.h
 * @brief Foreground-only observer initialization shared by Motor and Feedback.
 * @author Codex
 * @date 2026-11-23
 */
#ifndef MFOC_FEEDBACK_INTERNAL_H
#define MFOC_FEEDBACK_INTERNAL_H

#include "mfoc_feedback.h"

#ifdef MFOC_ABI_ENTRY
/* Non-ELF callers also have to agree on the prepared state layout. */
#define mfoc_feedback_Prepare MFOC_ABI_ENTRY(mfoc_feedback_Prepare)
#define mfoc_feedback_InitPrepared MFOC_ABI_ENTRY(mfoc_feedback_InitPrepared)
#endif

/** @brief Prepared state of the observers that can reject initialization. */
typedef struct {
#if MFOC_FEEDBACK_ENABLE_SMO
    mfoc_smo_t smo;
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
    mfoc_flux_t flux;
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_ekf_t ekf;
#endif
#if !MFOC_FEEDBACK_ENABLE_SMO && !MFOC_FEEDBACK_ENABLE_FLUX && \
    !MFOC_FEEDBACK_ENABLE_EKF
    bool unused;
#endif
} mfoc_feedback_prepared_t;

/** @brief Build fallible observer state without changing the active Feedback.
 * @param prepared Scratch storage. @param cfg Effective settings.
 * @param model Shared Motor data. @return Observer initialization status. */
mfoc_result_t mfoc_feedback_Prepare(mfoc_feedback_prepared_t *prepared,
    const mfoc_feedback_cfg_t *cfg, const mfoc_motor_params_t *model);

/** @brief Initialize zeroed Feedback using observer state from Prepare.
 * @param feedback Zeroed destination. @param prepared Validated state.
 * @param cfg The same effective settings supplied to Prepare. @return None. */
void mfoc_feedback_InitPrepared(mfoc_feedback_t *feedback,
    const mfoc_feedback_prepared_t *prepared, const mfoc_feedback_cfg_t *cfg);

#endif
