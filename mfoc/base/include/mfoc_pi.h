/**
 * @file mfoc_pi.h
 * @brief Generic parallel PI controller: u = kp * err + ui.
 * @author Codex
 * @date 2026-10-20
 */
#ifndef MFOC_PI_H
#define MFOC_PI_H

#include "mfoc_numeric.h"

/** @brief PI gains and limits, in caller-selected input/output units.
 * ki_ts = Ki * Ts; both ui and output limits use output units.
 * Signed gains are allowed; FIXED fields use the common int32 Q15 format. */
typedef struct {
    mfoc_scalar_t kp;
    mfoc_scalar_t ki_ts;
    mfoc_scalar_t out_min;
    mfoc_scalar_t out_max;
    mfoc_scalar_t i_min;
    mfoc_scalar_t i_max;
} mfoc_pi_params_t;

/** @brief One PI instance, owning its configuration and integral output.
 * Initialize before use; serialize operations on the same instance.
 * Caller supplies finite values and keeps FLOAT intermediate math in range.
 * PI does not check NaN/Inf. FIXED arithmetic saturates its int32 carrier. */
typedef struct {
    mfoc_pi_params_t cfg;
    mfoc_scalar_t ui;
} mfoc_pi_t;

/** @brief Copy configuration and reset ui; failure leaves p unchanged.
 * @param p PI instance. @param c Gains and ordered limits.
 * @return OK, NULL, or INVALID_ARGUMENT for unordered limits. */
mfoc_result_t mfoc_pi_Init(mfoc_pi_t *p, const mfoc_pi_params_t *c);

/** @brief Reset ui to zero, clipped to its configured limits.
 * @param p Initialized PI. @return OK or NULL. */
mfoc_result_t mfoc_pi_Reset(mfoc_pi_t *p);

/** @brief Apply external output correction after Step: ui += applied - out.
 * @param p Initialized PI. @param out Output returned by Step.
 * @param applied Externally limited output, in the same units.
 * @return None. Integral limits still apply; no next-step preload.
 * @note p must be valid and initialized. */
void mfoc_pi_ApplyLimit(mfoc_pi_t *p, mfoc_scalar_t out,
                        mfoc_scalar_t applied);

/** @brief Set ui = clamp(out - kp*err - ki_ts*err) for the next Step.
 * @param p Initialized PI. @param out Target output, clipped to output limits.
 * @param ref Reference. @param fbk Feedback.
 * @return OK or NULL.
 * @note Same next-step inputs reproduce out if ui limits and numeric
 * representation permit. */
mfoc_result_t mfoc_pi_Track(mfoc_pi_t *p, mfoc_scalar_t out,
                           mfoc_scalar_t ref, mfoc_scalar_t fbk);

/** @brief Run parallel PI with conditional integration and output clamping.
 * @param p Initialized PI. @param ref Reference. @param fbk Feedback.
 * @param out Output; must not overlap p. @return None.
 * @note Freeze ui when the previous integral plus current proportional term
 * is saturated and di drives further outward; reverse di may recover it.
 * Valid pointers are required. No implicit [-1,1] error clipping.
 * See mfoc_pi_t for numeric preconditions. */
void mfoc_pi_Step(mfoc_pi_t *p, mfoc_scalar_t ref,
                  mfoc_scalar_t fbk, mfoc_scalar_t *out);

#endif
