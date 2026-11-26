/**
 * @file mfoc_at32_hall_config.h
 * @brief Measured Hall sectors for the 5562 Hall project wiring.
 * @author Codex
 * @date 2026-11-25
 */
#ifndef MFOC_BLDC5562D048_HALL_CONFIG_H
#define MFOC_BLDC5562D048_HALL_CONFIG_H

#include <stdint.h>

/* Measured against back EMF using the motor's U/V/W and H1/H2/H3 order.
 * Positive rotation is CCW from the shaft end. Entry angles are
 * 28.63, 96.79, 152.68, 203.64, 268.09 and 330.87 electrical degrees.
 * H1/H2/H3 map to bits 0/1/2. Recheck angles after changing phase order. */
#define MFOC_AT32_HALL_SECTORS { \
    {1U, {UINT32_C(0x145beeb5)}}, {3U, {UINT32_C(0x44d414ef)}}, \
    {2U, {UINT32_C(0x6c928bb8)}}, {6U, {UINT32_C(0x90cf87da)}}, \
    {4U, {UINT32_C(0xbea44b8c)}}, {5U, {UINT32_C(0xeb490b9b)}} }

/* Preserve the qualified standstill/slow-start timeout for this motor. */
#define MFOC_AT32_HALL_TIMEOUT_MS 1000U

#endif
