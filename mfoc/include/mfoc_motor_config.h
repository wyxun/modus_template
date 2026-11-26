/**
 * @file mfoc_motor_config.h
 * @brief Read an optional motor profile shared by the build.
 */
#ifndef MFOC_MOTOR_CONFIG_SELECTOR_H
#define MFOC_MOTOR_CONFIG_SELECTOR_H

#if defined(MFOC_MOTOR_CONFIG)
#include MFOC_MOTOR_CONFIG
#endif

/* Compile-time speed normalization for HFI and identification. */
#ifndef MFOC_MOTOR_BASE_ELECTRICAL_HZ
#define MFOC_MOTOR_BASE_ELECTRICAL_HZ 400U
#endif

#endif
