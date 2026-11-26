/**
 * @file mfoc_sensorless_start_config.h
 * @brief Compile-time defaults for sensorless startup and handover.
 */
#ifndef MFOC_SENSORLESS_START_CONFIG_H
#define MFOC_SENSORLESS_START_CONFIG_H

#include "mfoc_motor_config.h"

/* Handover speed/window are in PU, angles are in electrical turns. */
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
