/**
 * @file foc_port_config.h
 * @brief Portable test configuration for the FOC application command test.
 */
#ifndef FOC_TEST_PORT_CONFIG_H
#define FOC_TEST_PORT_CONFIG_H

#define FOC_PORT_HAS_POSITION 1

#include "foc_encoder.h"
#include "foc_port.h"

extern const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface;

#endif /* FOC_TEST_PORT_CONFIG_H */
