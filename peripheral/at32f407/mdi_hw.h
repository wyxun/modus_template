/**
 * @file mdi_hw.h
 * @brief Global peripheral resource definition (MDI hardware pool) — AT32F407
 *
 * Provides a unified hardware structure for the application layer,
 * avoiding exposure of chip-specific headers.
 */

#ifndef __MDI_HW_H__
#define __MDI_HW_H__

#include "mdi/legacy/mdi.h"
#include "mdi/legacy/mdi_static.h"

#define MDI_STREAM_WRITE_ASSOCIATIONS \
    mdi_stream_t *: mdi_stream_Write

#define MDI_STREAM_READ_ASSOCIATIONS \
    mdi_stream_t *: mdi_stream_Read

#define MDI_STREAM_IS_BUSY_ASSOCIATIONS \
    mdi_stream_t *: mdi_stream_IsBusy

/*============================================================================
 * Project hardware resource pool
 *===========================================================================*/

typedef struct {
    /* ---------- LED / Status ---------- */
    mdi_gpio_t   *ptLedStatus;    /**< PD13, active-low (AT-START-F407 LED2) */

    /* ---------- Stream ---------- */
    mdi_stream_t *ptSerial;       /**< USART1 PA9/PA10 (on-board ST-Link VCP) */

} mdi_hardware_t;

extern const mdi_hardware_t HW;

#endif /* __MDI_HW_H__ */
