/****************************************************************************
 * @file    foc_observer.h
 * @brief   Compile-time estimator composition; Position selects candidates.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/

#ifndef FOC_OBSERVER_H
#define FOC_OBSERVER_H

#include "foc_config.h"
#include "foc_smo.h"

/* HFI + SMO extension contract (design only; HFI is not implemented):
 * - Add independent HFI cfg/input/output/state alongside the SMO members.
 *   Own both states concurrently; each algorithm keeps its own evidence.
 * - Match PU bases, axes, signs and estimate timestamps before comparison.
 *   HFI inputs must identify the injection phase and corresponding samples.
 * - HFI publishes a PU injection request for Motor/Core to apply; it does
 *   not submit PWM. Define how SMO rejects or models injection components.
 * - Run both algorithms during qualification/blending; starting an estimator
 *   at the switching threshold does not make its estimate qualified.
 * - Before dual mode, add per-algorithm failure reporting so one failed
 *   estimator cannot erase an otherwise valid peer candidate. The current
 *   single-SMO Step contract clears the entire output on error.
 * - Position owns selection, bidirectional blending and fallback policy;
 *   sensor and hard-drag candidates remain outside these algorithm types.
 * Full design: ../docs/2026-10-09-observer-composition-boundary.md.
 */

/** @brief Independent algorithm configurations consumed only by Init. */
typedef struct {
    foc_smo_cfg_t tSmo;
} foc_observer_cfg_t;

/** @brief Algorithm-specific inputs; borrowed pointers expire after Step. */
typedef struct {
    foc_smo_input_t tSmo;
} foc_observer_input_t;

/** @brief Candidate snapshots with independent evidence, never selected here.
 * Add new algorithm members alongside tSmo, not inside its input/output types.
 * Pure data types remain available in observer-disabled builds.
 */
typedef struct {
    foc_smo_output_t tSmo;
} foc_observer_estimates_t;

/** @brief Instance-owned algorithm states; concurrent estimators need no union.
 * Future enabled algorithms get independent #if blocks in Init/Step/Reset.
 * No parent pointer, runtime callbacks, configuration or input is retained.
 */
typedef struct {
#if FOC_OBSERVER_HAS_SMO
    foc_smo_t tSmo;
#endif
    bool bInitialized;
} foc_observer_t;

/**
 * @brief Initialize every algorithm in the compile-time set.
 * @param ptObserver Position-owned Observer instance.
 * @param ptModel Electrical model snapshot consumed only during Init.
 * @param ptConfig Independent algorithm configurations.
 * @return OK, DISABLED, or an initialization error.
 * @note Failure leaves the whole composition uninitialized.
 */
foc_result_t foc_observer_Init(
    foc_observer_t *ptObserver,
    const foc_electrical_model_t *ptModel,
    const foc_observer_cfg_t *ptConfig);

/**
 * @brief Advance enabled algorithms and publish candidate snapshots.
 * @param ptObserver Instance-owned algorithm state.
 * @param ptInput Independent inputs with documented PU bases and timing.
 * @param ptEstimates Caller-owned candidate output for this sample.
 * @return OK, DISABLED, or an input/state error.
 * @note No selection, blending, PWM submission, logging or unit conversion.
 *       Any error clears output validity; a failed set is not a partial success.
 */
foc_result_t foc_observer_Step(
    foc_observer_t *ptObserver,
    const foc_observer_input_t *ptInput,
    foc_observer_estimates_t *ptEstimates);

/**
 * @brief Clear all algorithm history while preserving validated coefficients.
 * @param ptObserver Observer instance.
 * @return None.
 * @note Caller discards previous snapshots at its own lifecycle boundary.
 */
void foc_observer_Reset(foc_observer_t *ptObserver);

#endif /* FOC_OBSERVER_H */
