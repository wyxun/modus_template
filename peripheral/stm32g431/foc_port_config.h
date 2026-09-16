/****************************************************************************
 * @file    foc_port_config.h
 * @brief   STM32G431 FOC hardware-source declarations.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_PORT_CONFIG_H
#define FOC_PORT_CONFIG_H

#define FOC_PORT_HAS_POSITION 1

#include "foc_encoder.h"
#include "foc_port.h"

extern const foc_adc_if_t g_tFocAdcInterface;
extern const foc_pwm_if_t g_tFocPwmInterface;
extern const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface;

void *foc_port_GetPwmContext(void);
void foc_port_NotifyBreak(void *pContext);

#endif /* FOC_PORT_CONFIG_H */
