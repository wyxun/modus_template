/**
 * @file mfoc_core.h
 * @brief Current PI, circular voltage limit and SVPWM.
 * @author Codex
 * @date 2026-10-20
 */
#ifndef MFOC_CORE_H
#define MFOC_CORE_H

#include "mfoc_config.h"
#include "mfoc_pi.h"
#include "mfoc_angle.h"

/** @brief Two independent axis gains/limits and initial bus voltage.
 * PI input is I/Ibase, output and integral are V/Vbase.
 * This core supports axis output bounds within [-1,1] V/Vbase.
 * Configure integral bounds to allow the required saturation recovery. */
typedef struct {
    mfoc_pi_params_t d;
    mfoc_pi_params_t q;
    mfoc_scalar_t bus; /* Initial Vdc/Vbase, strictly positive. */
} mfoc_core_params_t;

/** @brief Instance-owned state; access through Core operations only. */
typedef struct {
    mfoc_pi_t d;
    mfoc_pi_t q;
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t u_max_sq; /* Squared voltage radius, (V/Vbase)^2. */
#else
    mfoc_scalar_t u_max;    /* Voltage radius, V/Vbase. */
#endif
    mfoc_scalar_t bus;     /* Vdc/Vbase used for the current coefficients. */
    mfoc_scalar_t bus_inv; /* Vbase/Vdc. */
} mfoc_core_t;

/** @brief Synchronous current sample and command for this control step.
 * i/ref use Ibase; bus coefficients come from Init/SetBus.
 * angle is the valid electrical angle for i[k].
 * Caller guarantees finite values, representable intermediate arithmetic,
 * initialized state, and nonoverlapping input/output/state buffers.
 * No sampling, angle-source selection or fault handling occurs here. */
typedef struct {
    mfoc_ab_t i;
    mfoc_dq_t ref;
    mfoc_angle_t angle;
    mfoc_ab_t u_inject_ab; /* Added stationary voltage V/Vbase; zero disables. */
} mfoc_core_input_t;

/** @brief Computed command for the next PWM interval, not measured voltage.
 * i is dq current; u includes injection, limited dq V/Vbase.
 * m is alpha-beta V/Vdc.
 * bus is the Vdc/Vbase snapshot used for this output.
 * limited reports circular limiting.
 * PI axis limits are configured separately and remain active. */
typedef struct {
    mfoc_dq_t i;
    mfoc_dq_t u;
    mfoc_ab_t m;
    mfoc_uvw_t duty;
    mfoc_scalar_t bus;
    bool limited;
} mfoc_core_output_t;

/** @brief Copy valid configuration; failure leaves c unchanged.
 * @param c Instance. @param cfg Finite configuration.
 * @return OK, NULL, INVALID_ARGUMENT or OUT_OF_RANGE. */
mfoc_result_t mfoc_core_Init(mfoc_core_t *c,
                            const mfoc_core_params_t *cfg);

/** @brief Update bus coefficients without changing PI state.
 * @param c Initialized instance. @param bus Finite positive Vdc/Vbase.
 * @return OK, NULL, INVALID_ARGUMENT or OUT_OF_RANGE.
 * @note Failure leaves state unchanged. Radius and reciprocal must be
 * representable and positive (FIXED uses Q15, with truncated reciprocal).
 * FLOAT also requires a finite, normal squared voltage radius.
 * Serialize with Step/Reset; no hardware access or synchronization here.
 * Fixed bus: call only at Init. Dynamic bus: update before the next Step. */
mfoc_result_t mfoc_core_SetBus(mfoc_core_t *c, mfoc_scalar_t bus);

/** @brief Reset both integral states, retaining configuration.
 * @param c Initialized instance. @return OK or NULL. */
mfoc_result_t mfoc_core_Reset(mfoc_core_t *c);

/** @brief Execute Park, two PI steps, vector limit, inverse Park and SVPWM.
 * @param c Initialized instance. @param in Valid synchronous inputs.
 * @param out Command output. @return None.
 * All pointers must be valid; caller guarantees the input/state preconditions.
 * sin/cos is evaluated once. No speed/decoupling or voltage mode is added. */
void mfoc_core_Step(mfoc_core_t *c,
                    const mfoc_core_input_t *in,
                    mfoc_core_output_t *out);

#endif
