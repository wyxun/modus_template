/** @file mfoc_board_bus.h
 * Board application handoff for the optional 100 Hz DC-link monitor.
 * One foreground producer and one higher-priority, non-reentrant control IRQ.
 */
#ifndef MFOC_BOARD_BUS_H
#define MFOC_BOARD_BUS_H
#include "mfoc_core.h"

/** A completed conversion (or sampling failure), with its acquisition time. */
typedef struct {
    mfoc_tick_t started;
    mfoc_scalar_t bus;
    mfoc_result_t status;
} mfoc_board_bus_sample_t;

/** Foreground board provider. True only for a new sample or a hard failure. */
bool mfoc_board_bus_Poll(mfoc_board_bus_sample_t *sample);
/** Foreground preparation and publication, also used by the AT32 start gate. */
mfoc_result_t mfoc_board_bus_Publish(const mfoc_board_bus_sample_t *sample);
/** Foreground freshness interlock immediately before enabling phase outputs. */
mfoc_result_t mfoc_board_bus_Arm(mfoc_tick_t now);
/** Consume one running control frame. Recheck absolute age on the first IRQ. */
mfoc_result_t mfoc_board_bus_Apply(mfoc_core_t *core);
void mfoc_board_bus_Reset(void);
#endif
