/**
 * @file mfoc_motor_config.h
 * @brief Select one motor profile for all translation units in the build.
 */
#ifndef MFOC_MOTOR_CONFIG_SELECTOR_H
#define MFOC_MOTOR_CONFIG_SELECTOR_H

#if defined(MFOC_MOTOR_CONFIG)
#include MFOC_MOTOR_CONFIG
#else
#include "../config/motors/motor_2804.h"
#endif

#endif
