/**
 * @file mfoc_modulation.h
 * @brief Centered three-phase modulation with explicit voltage units.
 * @author Codex
 * @date 2026-10-19
 */
#ifndef MFOC_MODULATION_H
#define MFOC_MODULATION_H
#include "mfoc_types.h"

/** @brief SVPWM through min/max common-mode injection.
 * @param voltage Finite voltage/bus modulation, each component in [-1,1].
 * @param duty Output [0,1]. @param saturated Optional endpoint/clipping flag.
 * @return None. Linear vector limit is 1/sqrt(3).
 * @note Voltage/duty pointers and input ranges must be valid.
 * Common-mode shift needs no trig. */
void mfoc_svpwm(const mfoc_ab_t *voltage,
                mfoc_uvw_t *duty, bool *saturated);
/** @brief Centered SPWM. @param voltage Finite voltage/bus modulation [-1,1].
 * @param duty Output [0,1]. @return NULL or OK.
 * @note Caller validates inputs. Linear vector limit is 0.5; may clip. */
mfoc_result_t mfoc_spwm(const mfoc_ab_t *voltage, mfoc_uvw_t *duty);
/** @brief Reconstruct ideal stationary voltage from an effective interval.
 * @param duty Finite prior interval duties [0,1].
 * @param bus Finite measured bus/baseV > 0.
 * @param voltage Output voltage/baseV; may exceed 1 PU.
 * @return NULL or OK. Caller aligns i[k] and u[k-1].
 * @note Caller validates input ranges. Excludes deadtime and device drops. */
mfoc_result_t mfoc_modulation_ModelVoltageAlphaBeta(
    const mfoc_uvw_t *duty, mfoc_scalar_t bus, mfoc_ab_t *voltage);
#endif
