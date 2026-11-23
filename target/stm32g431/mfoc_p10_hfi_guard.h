/** @file mfoc_p10_hfi_guard.h
 * Phase-current interlock for the higher-injection G431 HFI bench images.
 */
#ifndef MFOC_G431_P10_HFI_GUARD_H
#define MFOC_G431_P10_HFI_GUARD_H

#include "mfoc_numeric.h"

#define MFOC_P10_HFI_PHASE_LIMIT MFOC_SCALAR(0.12f)

/** Read from the matching ELF with the bridge closed. Values are in PU. */
typedef struct {
    uint32_t samples;
    uint32_t first_trip;
    mfoc_scalar_t peak_abs_phase;
    mfoc_uvw_t trip_current;
} mfoc_p10_hfi_guard_t;

extern mfoc_p10_hfi_guard_t g_p10_hfi_guard;

/** @brief Check fresh normalized U/V/W before the frame reaches Motor/Core.
 * @param current Valid and bounded phase currents from the G431 ADC port.
 * @return False to make Motor close the bridge via its sample fault path. */
static inline bool mfoc_p10_hfi_Current(const mfoc_uvw_t *current)
{
    mfoc_scalar_t peak = mfoc_abs(current->qU);
    mfoc_scalar_t v = mfoc_abs(current->qV);
    mfoc_scalar_t w = mfoc_abs(current->qW);
    if (v > peak) { peak = v; }
    if (w > peak) { peak = w; }
    ++g_p10_hfi_guard.samples;
    if (peak > g_p10_hfi_guard.peak_abs_phase) {
        g_p10_hfi_guard.peak_abs_phase = peak;
    }
    if (peak > MFOC_P10_HFI_PHASE_LIMIT) {
        if (g_p10_hfi_guard.first_trip == 0U) {
            g_p10_hfi_guard.first_trip = g_p10_hfi_guard.samples;
            g_p10_hfi_guard.trip_current = *current;
        }
        return false;
    }
    return true;
}

#endif
