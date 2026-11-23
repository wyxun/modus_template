/**
 * @file mfoc_config.h
 * @brief Common project options for MFOC.
 * @author wyxun
 * @date 2026-10-23
 */
#ifndef MFOC_CONFIG_H
#define MFOC_CONFIG_H

/* Speed-loop rate in Hz. Set to 0 to disable the default speed mode.
 * A nonzero rate must divide the board's control interrupt rate. */
#ifndef MFOC_SPEED_RUN_FREQ_HZ
#define MFOC_SPEED_RUN_FREQ_HZ 1000U
#endif

/* Main control feedback and the sources compiled into the project. */
#ifndef MFOC_FEEDBACK_BACKEND
#define MFOC_FEEDBACK_BACKEND MFOC_FEEDBACK_FLUX
#endif
#if !defined(MFOC_FEEDBACK_ENABLE_POSITION) &&                              \
    !defined(MFOC_FEEDBACK_ENABLE_ENCODER)
#define MFOC_FEEDBACK_ENABLE_POSITION 1
#endif
#ifndef MFOC_FEEDBACK_ENABLE_SMO
#define MFOC_FEEDBACK_ENABLE_SMO 0
#endif
#ifndef MFOC_FEEDBACK_ENABLE_HFI
/* Motor decides whether to permit injection at startup. */
#define MFOC_FEEDBACK_ENABLE_HFI 0
#endif
#ifndef MFOC_FEEDBACK_ENABLE_FLUX
/* Requires confirmed motor flux and configured observer/PLL gains. */
#define MFOC_FEEDBACK_ENABLE_FLUX 1
#endif
#ifndef MFOC_FEEDBACK_ENABLE_EKF
/* FLOAT only; start with physical position control and EKF observation. */
#define MFOC_FEEDBACK_ENABLE_EKF 0
#endif

/* 0 locks the axis only, 1 requires magnetic polarity identification. */
#ifndef MFOC_HFI_ENABLE_POLARITY
#define MFOC_HFI_ENABLE_POLARITY 0
#endif

/* Include the experiment workspace in Motor only when enabled. */
#ifndef MFOC_IDENTIFICATION_ENABLE
#define MFOC_IDENTIFICATION_ENABLE 0
#endif

/* Enable the foreground API for replacing an idle Motor's PU parameters. */
#ifndef MFOC_MOTOR_MODEL_UPDATE_ENABLE
#define MFOC_MOTOR_MODEL_UPDATE_ENABLE 0
#endif

#include "include/mfoc_config_internal.h"

#endif
