/**
 * @file mfoc_ekf.h
 * @brief FLOAT four-state equal-inductance PMSM extended Kalman filter.
 * @author Codex
 * @date 2026-10-31
 */
#ifndef MFOC_EKF_H
#define MFOC_EKF_H
#include "mfoc_config.h"
#include "mfoc_angle.h"
#include "mfoc_motor_params.h"

#if MFOC_FEEDBACK_ENABLE_EKF
#if defined(MFOC_NUMERIC_FIXED)
#error "MFOC EKF requires FLOAT; FIXED EKF is not implemented"
#endif

/** @brief Init-only tuning for x=[i_alpha, i_beta, speed_pu, theta_turn].
 * Q is diagonal process covariance per sample; P0 is initial covariance.
 * R=[Raa,Rab,Rbb] is symmetric current-measurement covariance in PU squared.
 * Q/P0 units are [current PU squared, current PU squared, speed PU squared,
 * turns squared]. Caller supplies finite representable model/tuning values. */
typedef struct {
    uint32_t sample_hz;
    mfoc_scalar_t speed_base_hz;
    float q[4];
    float r[3];
    float p0[4];
    float min_speed;          /* Positive electrical speed PU magnitude. */
    float max_angle_var;      /* Positive posterior variance, turns squared. */
    float max_speed_var;      /* Positive posterior variance, speed PU squared. */
} mfoc_ekf_cfg_t;

/** @brief Precomputed normalized-time recurrence and whitened noise. */
typedef struct {
    float ki;                /* 1 - rs * ku; Euler decay requires |ki| < 1. */
    float ku;                /* (2*pi*speed_base_hz/sample_hz) / ld. */
    float ke;                /* flux * ku. */
    float kt;                /* speed_base_hz/sample_hz, turns per speed PU. */
    float q[4];
    float p0[4];
    float r1;                /* Raa. */
    float r2;                /* Rbb - Rab*Rab/Raa. */
    float rho;               /* Rab/Raa; h2=[-rho,1,0,0]. */
    float min_speed;
    float max_angle_var;
    float max_speed_var;
} mfoc_ekf_coeff_t;

/** @brief Posterior k|k estimate; validity is speed/covariance qualification. */
typedef struct {
    mfoc_angle_t angle;
    mfoc_scalar_t speed;      /* Electrical speed / omega_base. */
    bool valid;
} mfoc_ekf_output_t;

/** @brief Instance-owned mean/covariance; no hardware or parent dependencies.
 * P uses upper-triangle order [00,01,02,03,11,12,13,22,23,33].
 * Angle mean is BAM32; angle covariance and increments use electrical turns. */
typedef struct {
    mfoc_ekf_coeff_t c;
    mfoc_ab_t i_hat;
    mfoc_scalar_t speed;
    mfoc_angle_t angle;
    float p[10];
    bool has_sample;
} mfoc_ekf_t;

/** @brief Validate tuning/model and initialize an equal-Ld/Lq filter.
 * @param ekf Instance. @param cfg Tuning. @param model PU motor constants.
 * @return Status; failure leaves the instance unchanged. */
mfoc_result_t mfoc_ekf_Init(mfoc_ekf_t *ekf, const mfoc_ekf_cfg_t *cfg,
                           const mfoc_motor_params_t *model);

/** @brief Restore zero mean/P0 and clear sample history, preserving tuning.
 * @param ekf Initialized instance. @return None. */
void mfoc_ekf_Reset(mfoc_ekf_t *ekf);

/** @brief Predict from u[k-1], then assimilate current i[k] in two updates.
 * @param ekf Instance. @param i Present alpha-beta current, I/Ibase.
 * @param u_previous Prior-interval model voltage, V/Vbase.
 * @param out Present-sample posterior angle/speed/quality. @return None.
 * @note Valid pointers and finite bounded inputs are caller preconditions.
 * First sample seeds current only and is invalid. Nonpositive innovation
 * variance or negative covariance diagonal resets the filter and output.
 * One old-state sin/cos pair; no PLL, encoder seed or takeover policy. */
void mfoc_ekf_Step(mfoc_ekf_t *ekf, const mfoc_ab_t *i,
                   const mfoc_ab_t *u_previous, mfoc_ekf_output_t *out);
#endif
#endif
