/**
 * @file service.h
 * @brief Board software ADC scheduling and DMA interrupt handoff.
 * @author Codex
 * @date 2026-10-10
 */
#ifndef TEMPLATE_MDI_SERVICE_H
#define TEMPLATE_MDI_SERVICE_H
#include "mdi/core/contract.h"

/**
 * @brief Change foreground scan rate, not the ADC conversion clock.
 * @param wHz Scans/second, 1..1000000; period is ceil(1e6/wHz) us.
 * @return OK or RANGE; failure preserves the current schedule.
 * @note Foreground serialized. Effective now; active DMA finishes. First
 *       Service starts immediately. Later starts depend on foreground latency
 *       and DMA availability; the range is not a hardware rate guarantee.
 */
mdi_status_t mdi_service_SetAdcFrequency(uint32_t wHz);
void pt32_AdcDmaCompleteIrq(void);
#endif
