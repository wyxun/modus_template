/**
 * @file mfoc_feedback.h
 * @brief Independent position algorithms and control feedback selection.
 * @author wyxun
 * @date 2026-10-21
 */
#ifndef MFOC_FEEDBACK_H
#define MFOC_FEEDBACK_H

#include "mfoc_config.h"
#include "mfoc_smo.h"
#include "mfoc_hfi.h"
#include "mfoc_flux.h"
#include "mfoc_ekf.h"

#if MFOC_FEEDBACK_ENABLE_POSITION != 0 && MFOC_FEEDBACK_ENABLE_POSITION != 1
#error "MFOC_FEEDBACK_ENABLE_POSITION must be 0 or 1"
#endif
#if MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_NONE &&                              \
    MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_POSITION &&                           \
    MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_SMO &&                             \
    MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_HFI &&                               \
    MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_FLUX &&                              \
    MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_EKF
#error "MFOC_FEEDBACK_BACKEND must select an implemented position algorithm"
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION &&                           \
    !MFOC_FEEDBACK_ENABLE_POSITION
#error "POSITION backend requires MFOC_FEEDBACK_ENABLE_POSITION"
#endif
#if MFOC_FEEDBACK_ENABLE_SMO != 0 && MFOC_FEEDBACK_ENABLE_SMO != 1
#error "MFOC_FEEDBACK_ENABLE_SMO must be 0 or 1"
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO && !MFOC_FEEDBACK_ENABLE_SMO
#error "The selected SMO output requires MFOC_FEEDBACK_ENABLE_SMO"
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_HFI && !MFOC_FEEDBACK_ENABLE_HFI
#error "The selected HFI output requires MFOC_FEEDBACK_ENABLE_HFI"
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX != 0 && MFOC_FEEDBACK_ENABLE_FLUX != 1
#error "MFOC_FEEDBACK_ENABLE_FLUX must be 0 or 1"
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_FLUX && !MFOC_FEEDBACK_ENABLE_FLUX
#error "The selected Flux output requires MFOC_FEEDBACK_ENABLE_FLUX"
#endif
#if MFOC_FEEDBACK_ENABLE_EKF != 0 && MFOC_FEEDBACK_ENABLE_EKF != 1
#error "MFOC_FEEDBACK_ENABLE_EKF must be 0 or 1"
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_EKF && !MFOC_FEEDBACK_ENABLE_EKF
#error "The selected EKF output requires MFOC_FEEDBACK_ENABLE_EKF"
#endif

#if MFOC_FEEDBACK_ENABLE_HFI
/** @brief HFI execution and permission to publish its voltage request. */
enum {
    MFOC_FEEDBACK_HFI_OFF,
    MFOC_FEEDBACK_HFI_OBSERVE,
    MFOC_FEEDBACK_HFI_INJECT
};
#endif

/** @brief Instance settings for the compiled observers. */
typedef struct {
    mfoc_smo_cfg_t smo;
    mfoc_flux_cfg_t flux;
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_ekf_cfg_t ekf;
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    bool observe_hfi; /**< Run HFI diagnostics in ordinary non-HFI control. */
#endif
} mfoc_feedback_cfg_t;

/** @brief Borrowed samples and prior interval values for one Step. */
typedef struct {
    const mfoc_ab_t *i_ab_k; /**< Current sample i[k], alpha-beta I/Ibase. */
    /** Model voltage over [k-1,k), alpha-beta V/Vbase; NULL without history. */
    const mfoc_ab_t *u_ab_previous;
} mfoc_feedback_input_t;

/** @brief Selected control feedback and next-interval injection request.
 * During HFI acquisition, Motor uses the estimated angle and freezes speed
 * control. The current PIs run at zero dq reference with injection enabled. */
typedef struct {
    mfoc_angle_t angle;   /**< Electrical BAM32. */
    mfoc_scalar_t speed;  /**< omega_e/omega_base. */
    mfoc_ab_t u_inject_ab; /**< Next-interval V/Vbase; zero when inactive. */
#if MFOC_FEEDBACK_ENABLE_HFI
    bool acquiring; /**< Primary HFI is acquiring its control angle. */
#endif
} mfoc_feedback_output_t;

/** @brief Instance-owned algorithm states, independent of output selection. */
typedef struct {
#if MFOC_FEEDBACK_ENABLE_SMO
    mfoc_smo_t smo;
    mfoc_smo_cfg_t smo_cfg; /**< Retained for stopped model updates. */
    mfoc_smo_output_t smo_output;
    mfoc_result_t smo_status;
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    mfoc_hfi_t hfi;
    mfoc_hfi_output_t hfi_output;
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
    mfoc_flux_t flux;
    mfoc_flux_cfg_t flux_cfg; /**< Retained for stopped model updates. */
    mfoc_flux_output_t flux_output;
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
    mfoc_ekf_t ekf;
    mfoc_ekf_cfg_t ekf_cfg; /**< Retained for stopped model updates. */
    mfoc_ekf_output_t ekf_output;
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
    uint8_t normal_hfi_mode; /**< Init policy for ordinary closed-loop starts. */
    uint8_t hfi_mode;        /**< Policy for this run, set before PWM starts. */
#endif
    bool initialized;
} mfoc_feedback_t;

/** @brief Initialize enabled algorithms before enabling the ISR reader.
 * @param feedback Instance. @param cfg Algorithm configurations.
 * @param model Shared Motor constants, borrowed only during Init.
 * @return Initialization status. */
mfoc_result_t mfoc_feedback_Init(mfoc_feedback_t *feedback,
    const mfoc_feedback_cfg_t *cfg, const mfoc_motor_params_t *model);

/** @brief Rebuild observers, preserving Port position calibration.
 * @param feedback Instance. @param model New motor PU parameters.
 * @return OK or rejected model; failure preserves all observer state.
 * @note Motor must be stopped. Pole pairs and PU bases remain unchanged. */
mfoc_result_t mfoc_feedback_SetMotorParamModel(mfoc_feedback_t *feedback,
    const mfoc_motor_params_t *model);

#if MFOC_FEEDBACK_ENABLE_HFI
/** @brief Prepare HFI before a normal Motor start.
 * @param feedback Stopped instance. @param closed_loop Use its Init policy.
 * A forced field or identification owns excitation when closed_loop is false. */
static inline void mfoc_feedback_PrepareHfi(mfoc_feedback_t *feedback,
                                            bool closed_loop)
{
    feedback->hfi_mode = closed_loop ? feedback->normal_hfi_mode :
                                       MFOC_FEEDBACK_HFI_OFF;
    feedback->hfi_output.valid = false;
}
#endif

/** @brief Reset HFI and authorize injection with POSITION or HFI feedback.
 * @param feedback Initialized instance.
 * @return OK or INVALID_STATE for an unsupported backend/build. */
mfoc_result_t mfoc_feedback_HfiStart(mfoc_feedback_t *feedback);

/** @brief Run enabled ISR algorithms, then select the configured output.
 * @param feedback Instance. @param input Current and prior-interval voltage.
 * Physical position ignores input; absent voltage resets model histories.
 * An authorized HFI run can request its first injection without voltage.
 * @param output Feedback/excitation, unchanged on failure.
 * @return Backend status; NONE returns INVALID_STATE. No I2C access here.
 * @note Valid initialized feedback and output pointers are preconditions. */
mfoc_result_t mfoc_feedback_Step(mfoc_feedback_t *feedback,
    const mfoc_feedback_input_t *input, mfoc_feedback_output_t *output);

#endif
