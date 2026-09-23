/**
 * @file foc_port.h
 * @brief G431 FOC port entry and MDI binding.
 * @note This header is the only target port entry. The MDI adapter provides
 *       the static implementations; no G431 foc_port.c is required.
 */
#ifndef STM32G431_FOC_PORT_H
#define STM32G431_FOC_PORT_H

#define FOC_PORT_HAS_POSITION 1

#include "foc/hal/foc_port.h"
#include "halcordic.h"

#define FOC_PORT_TRIG_SINCOS_BAM32(W, S, C) \
    hal_cordic_SinCosBam32((W), (S), (C))
#define FOC_PORT_TRIG_ATAN2(Y, X) hal_cordic_Atan2((Y), (X))

#include "mdi/foc_adapter.h"

#endif /* STM32G431_FOC_PORT_H */
