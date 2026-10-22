/**
 * @file mfoc_debug.h
 * @brief MFOC waveform diagnostics.
 * @author Codex
 * @date 2026-10-22
 */
#ifndef MFOC_DEBUG_H
#define MFOC_DEBUG_H

#include "mfoc_app.h"

#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
/** @brief Register four MFOC angle/current waveform channels.
 * @return None. @note Called once after Motor initialization. */
void mfoc_debug_WaveformInit(void);

/** @brief Advance the waveform stream from the Motor control IRQ.
 * @return None. @note MWaveform applies the 10 kHz decimation. */
void mfoc_debug_WaveformStep(void);

/** @brief Refresh bound values for one accepted MWaveform sample.
 * @return None. @note Uses only the published encoder cache. */
void mfoc_debug_CaptureWaveform(void);
#endif

#endif
