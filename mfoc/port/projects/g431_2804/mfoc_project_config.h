/** @file mfoc_project_config.h
 * @brief Operating limits and control tuning for the G431 + 2804 project. */
#ifndef MFOC_G431_2804_PROJECT_CONFIG_H
#define MFOC_G431_2804_PROJECT_CONFIG_H

/* Current loop at 20 kHz. ki_ts includes the sampling period.
 * Output and integrator limits are in voltage PU. */
#ifndef MFOC_APP_D_PI
#define MFOC_APP_D_PI { \
    .kp = MFOC_SCALAR(0.472810f * MFOC_APP_CURRENT_BASE_RATIO), \
    .ki_ts = MFOC_SCALAR(0.070097f * MFOC_APP_CURRENT_BASE_RATIO), \
    .out_min = MFOC_SCALAR(-0.55f), \
    .out_max = MFOC_SCALAR(0.55f), \
    .i_min = MFOC_SCALAR(-0.50f), \
    .i_max = MFOC_SCALAR(0.50f)}
#endif
#ifndef MFOC_APP_Q_PI
#define MFOC_APP_Q_PI { \
    .kp = MFOC_SCALAR(0.472810f * MFOC_APP_CURRENT_BASE_RATIO), \
    .ki_ts = MFOC_SCALAR(0.070097f * MFOC_APP_CURRENT_BASE_RATIO), \
    .out_min = MFOC_SCALAR(-0.55f), \
    .out_max = MFOC_SCALAR(0.55f), \
    .i_min = MFOC_SCALAR(-0.50f), \
    .i_max = MFOC_SCALAR(0.50f)}
#endif

/* Speed loop at 1 kHz. ki_ts includes the sampling period.
 * The PI output and its integrator are in current PU. */
#ifndef MFOC_SPEED_RUN_FREQ_HZ
#define MFOC_SPEED_RUN_FREQ_HZ 1000U
#endif
#ifndef MFOC_APP_CURRENT_LIMIT_PU
#define MFOC_APP_CURRENT_LIMIT_PU (0.14f / MFOC_APP_CURRENT_BASE_RATIO)
#endif
#ifndef MFOC_APP_SPEED_CURRENT_LIMIT_PU
#define MFOC_APP_SPEED_CURRENT_LIMIT_PU (0.10f / MFOC_APP_CURRENT_BASE_RATIO)
#endif
#ifndef MFOC_APP_SPEED_MAX
#define MFOC_APP_SPEED_MAX MFOC_SCALAR(0.7583333f)
#endif
#ifndef MFOC_APP_SPEED_PI
#define MFOC_APP_SPEED_PI { \
    .kp = MFOC_SCALAR(0.6489304f / MFOC_APP_CURRENT_BASE_RATIO), \
    .ki_ts = MFOC_SCALAR(0.010193372f / MFOC_APP_CURRENT_BASE_RATIO), \
    .out_min = MFOC_SCALAR(-MFOC_APP_SPEED_CURRENT_LIMIT_PU), \
    .out_max = MFOC_SCALAR(MFOC_APP_SPEED_CURRENT_LIMIT_PU), \
    .i_min = MFOC_SCALAR(-MFOC_APP_SPEED_CURRENT_LIMIT_PU), \
    .i_max = MFOC_SCALAR(MFOC_APP_SPEED_CURRENT_LIMIT_PU)}
#endif

/* Flux observer and speed PLL. */
#ifndef MFOC_FLUX_GAIN
#define MFOC_FLUX_GAIN 0.1f
#endif
#ifndef MFOC_FLUX_PLL_KP
#define MFOC_FLUX_PLL_KP 0.5f
#endif
#ifndef MFOC_FLUX_PLL_KI
#define MFOC_FLUX_PLL_KI 100.0f
#endif

/* SMO cutoff in rad/s, switching gain in V and current bounds in PU. */
#ifndef MFOC_APP_SMO_CUTOFF_RAD_S
#define MFOC_APP_SMO_CUTOFF_RAD_S 4000U
#endif
#ifndef MFOC_APP_SMO_GAIN_V
#define MFOC_APP_SMO_GAIN_V 5.0f
#endif
#ifndef MFOC_APP_SMO_BOUNDARY_PU
#define MFOC_APP_SMO_BOUNDARY_PU 0.1f
#endif
#ifndef MFOC_APP_SMO_CURRENT_LIMIT_PU
#define MFOC_APP_SMO_CURRENT_LIMIT_PU 1.0f
#endif

/* HFI injection, axis qualification and PLL per eight-sample group. */
#ifndef MFOC_HFI_VOLTAGE_PU
#define MFOC_HFI_VOLTAGE_PU 0.04f
#endif
#ifndef MFOC_HFI_RESPONSE_MIN
#define MFOC_HFI_RESPONSE_MIN MFOC_SCALAR(0.002f)
#endif
#ifndef MFOC_HFI_SALIENCY_MIN
#define MFOC_HFI_SALIENCY_MIN MFOC_SCALAR(0.04f)
#endif
#ifndef MFOC_HFI_SALIENCY_SIGN
#define MFOC_HFI_SALIENCY_SIGN 1
#endif
#ifndef MFOC_HFI_PLL_KP
#define MFOC_HFI_PLL_KP MFOC_SCALAR(0.05f)
#endif
#ifndef MFOC_HFI_PLL_KI
#define MFOC_HFI_PLL_KI MFOC_SCALAR(0.02f)
#endif
#ifndef MFOC_HFI_SPEED_MAX
#define MFOC_HFI_SPEED_MAX MFOC_SCALAR(0.05f)
#endif
#ifndef MFOC_HFI_LOCK_ERROR
#define MFOC_HFI_LOCK_ERROR MFOC_SCALAR(0.01f) /* Electrical turns. */
#endif
#ifndef MFOC_HFI_LOCK_CYCLES
#define MFOC_HFI_LOCK_CYCLES 64U
#endif
#ifndef MFOC_HFI_STARTUP_MS
#define MFOC_HFI_STARTUP_MS 500U
#endif

/* Optional HFI polarity pulses, disabled by default. */
#ifndef MFOC_HFI_ENABLE_POLARITY
#define MFOC_HFI_ENABLE_POLARITY 0
#endif
#ifndef MFOC_HFI_POLARITY_VOLTAGE
#define MFOC_HFI_POLARITY_VOLTAGE MFOC_SCALAR(0.10f)
#endif
#ifndef MFOC_HFI_POLARITY_SAMPLES
#define MFOC_HFI_POLARITY_SAMPLES 4U
#endif
#ifndef MFOC_HFI_POLARITY_PAIRS
#define MFOC_HFI_POLARITY_PAIRS 3U
#endif
#ifndef MFOC_HFI_POLARITY_SIGN
#define MFOC_HFI_POLARITY_SIGN 1
#endif
#ifndef MFOC_HFI_POLARITY_CONTRAST
#define MFOC_HFI_POLARITY_CONTRAST MFOC_SCALAR(0.05f)
#endif
#ifndef MFOC_HFI_POLARITY_CURRENT_MAX
#define MFOC_HFI_POLARITY_CURRENT_MAX MFOC_SCALAR(0.10f)
#endif
#ifndef MFOC_HFI_REST_CURRENT
#define MFOC_HFI_REST_CURRENT MFOC_SCALAR(0.003f)
#endif
#ifndef MFOC_HFI_REST_VOLTAGE
#define MFOC_HFI_REST_VOLTAGE MFOC_SCALAR(0.01f)
#endif
#ifndef MFOC_HFI_REST_MS
#define MFOC_HFI_REST_MS 3U
#endif
#ifndef MFOC_HFI_REST_SPEED
#define MFOC_HFI_REST_SPEED MFOC_SCALAR(0.0005f)
#endif

/* FLOAT EKF covariance for [i_alpha, i_beta, speed_pu, angle_turns]. */
#ifndef MFOC_EKF_Q
#define MFOC_EKF_Q {1e-6f, 1e-6f, 1e-7f, 1e-8f}
#endif
#ifndef MFOC_EKF_R
#define MFOC_EKF_R {1e-6f, 0.0f, 1e-6f}
#endif
#ifndef MFOC_EKF_P0
#define MFOC_EKF_P0 {0.01f, 0.01f, 0.25f, 0.08333333f}
#endif
#ifndef MFOC_EKF_MIN_SPEED
#define MFOC_EKF_MIN_SPEED 0.05f
#endif
#ifndef MFOC_EKF_MAX_ANGLE_VAR
#define MFOC_EKF_MAX_ANGLE_VAR 0.000625f /* Electrical turns squared. */
#endif
#ifndef MFOC_EKF_MAX_SPEED_VAR
#define MFOC_EKF_MAX_SPEED_VAR 0.0025f /* Speed PU squared. */
#endif

/* Alignment, forced ramp and sensorless handover.
 * Speed and current are PU. Angles are in electrical turns. */
#ifndef MFOC_MOTOR_ALIGN_TIME_MS
#define MFOC_MOTOR_ALIGN_TIME_MS 1500U
#endif
#ifndef MFOC_APP_OPENLOOP_ACCEL_PU_S
#define MFOC_APP_OPENLOOP_ACCEL_PU_S 0.2f
#endif
#ifndef MFOC_START_MIN_SPEED_PU
#define MFOC_START_MIN_SPEED_PU 0.4f
#endif
#ifndef MFOC_START_SPEED_WINDOW
#define MFOC_START_SPEED_WINDOW 0.1f
#endif
#ifndef MFOC_START_HANDOVER_MS
#define MFOC_START_HANDOVER_MS 2000U
#endif
#ifndef MFOC_START_STABLE_MS
#define MFOC_START_STABLE_MS 20U
#endif
#ifndef MFOC_START_CAPTURE_TURNS
#define MFOC_START_CAPTURE_TURNS (10.0f / 360.0f)
#endif
#ifndef MFOC_START_BLEND_TURNS_S
#define MFOC_START_BLEND_TURNS_S 1U
#endif
#ifndef MFOC_START_IQ_UPDATE_MS
#define MFOC_START_IQ_UPDATE_MS 5U
#endif
#ifndef MFOC_START_IQ_ANGLE_GAIN
#define MFOC_START_IQ_ANGLE_GAIN 0.5f
#endif
#ifndef MFOC_START_IQ_SPEED_GAIN
#define MFOC_START_IQ_SPEED_GAIN 0.05f
#endif
#ifndef MFOC_START_IQ_SLEW_PU_S
#define MFOC_START_IQ_SLEW_PU_S 0.2f
#endif
#ifndef MFOC_START_IQ_MIN_PU
#define MFOC_START_IQ_MIN_PU 0.001f
#endif

#endif
