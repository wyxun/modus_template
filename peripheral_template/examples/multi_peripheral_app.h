/**
 * @file multi_peripheral_app.h
 * @brief Application-facing entry points for the MDI reference instance.
 */
#ifndef PERIPHERAL_TEMPLATE_MULTI_PERIPHERAL_APP_H
#define PERIPHERAL_TEMPLATE_MULTI_PERIPHERAL_APP_H

#include <stdint.h>

#include "mdi/instance.h"

/** Set the ADC frequency and update the compatibility raw-tick origin. */
mdi_status_t template_SetAdcSampleFrequency(uint32_t wHz,
                                             uint32_t wCurrentTick);

/**
 * Run the board MDI service for compatibility with pre-0.6.1.2 callers.
 * New applications should let modus_Run() invoke mdi_Service().
 */
mdi_status_t template_AdcService(uint32_t wCurrentTick);

/** Read the filtered bus-voltage channel. */
mdi_status_t template_ReadBusVoltage(uint32_t *pwCode);

#endif /* PERIPHERAL_TEMPLATE_MULTI_PERIPHERAL_APP_H */
