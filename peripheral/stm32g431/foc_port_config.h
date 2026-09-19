/****************************************************************************
 * @file    foc_port_config.h
 * @brief   STM32G431 FOC hardware-source declarations.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_PORT_CONFIG_H
#define FOC_PORT_CONFIG_H

#define FOC_PORT_HAS_POSITION 1

#include "foc_port.h"

void foc_port_NotifyBreak(void);

#endif /* FOC_PORT_CONFIG_H */
