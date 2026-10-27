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
/* Angle comparison takes priority: Flux, then SMO, then HFI diagnostics. */
#define MFOC_DEBUG_HFI_WAVEFORM (MFOC_FEEDBACK_ENABLE_HFI &&                     \
    !MFOC_FEEDBACK_ENABLE_FLUX && !MFOC_FEEDBACK_ENABLE_SMO)
#if MFOC_DEBUG_HFI_WAVEFORM
/* Temporary demodulation observation: 0 selects alpha excitation, 1 beta.
 * Selection changes the observed column only, never the injection sequence. */
#ifndef MFOC_DEBUG_HFI_AXIS
#define MFOC_DEBUG_HFI_AXIS 0
#endif
#if MFOC_DEBUG_HFI_AXIS != 0 && MFOC_DEBUG_HFI_AXIS != 1
#error "MFOC_DEBUG_HFI_AXIS must be 0 (alpha) or 1 (beta)"
#endif
#endif

/** @brief Register four HFI Di/Du or encoder/observer/current channels.
 * @return None. @note Called once after Motor initialization. */
void mfoc_debug_WaveformInit(void);

/** @brief Advance the waveform stream from the Motor control IRQ.
 * @return None. @note Called at 20 kHz. All four HFI values latch complete
 * carriers into existing waveform slots before the 10 kHz decimation. */
void mfoc_debug_WaveformStep(void);

/** @brief Refresh bound values for one accepted MWaveform sample.
 * @return None. @note HFI Di/Du slots are maintained by WaveformStep;
 * this callback leaves them untouched. Otherwise reads cached encoder/observer.
 * HFI channels use PU at 0.0001 PU/count transport resolution;
 * -3.2 marks unavailable, +/-3.1 diagnostic clipping.
 * Di is the difference of positive/negative second-interval increments;
 * Du is the matching total model-voltage difference, not measured voltage. */
void mfoc_debug_CaptureWaveform(void);
#endif

#endif
