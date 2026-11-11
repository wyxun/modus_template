/**
 * @file service.h
 * @brief G431 ADC scheduling, DMA interrupt handoff and encoder readout.
 * @author Codex
 * @date 2026-10-10
 */
#ifndef STM32G431_MDI_SERVICE_H
#define STM32G431_MDI_SERVICE_H
#include "mdi/core/contract.h"

/** @brief Read AS5600 RAW ANGLE in one foreground I2C transaction.
 * @param raw Mechanical position in counts (0..4095); unchanged on failure.
 * @return OK, INVALID for null output, or the I2C transfer error. */
mdi_status_t mdi_g431_encoder_ReadRaw(uint16_t *raw);

/**
 * @brief Change foreground scan rate, not the ADC conversion clock.
 * @param wHz Scans/second, 1..1000000; period is ceil(1e6/wHz) us.
 * @return OK or RANGE; failure preserves the current schedule.
 * @note Foreground serialized. Effective now; active DMA finishes. First
 *       Service starts immediately. Later starts depend on foreground latency
 *       and DMA availability; the range is not a hardware rate guarantee.
 */
mdi_status_t mdi_service_SetAdcFrequency(uint32_t wHz);
void mdi_g431_adc_dma_complete(void);
#endif
