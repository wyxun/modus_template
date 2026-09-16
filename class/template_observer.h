#ifndef MODUS_TEMPLATE_OBSERVER_H
#define MODUS_TEMPLATE_OBSERVER_H

/*
 * Observer interface example:
 *
 * This is a template_interface family example, not an Observer Driver object.
 * Concrete algorithm objects such as smo_t, flux_t or ekf_t implement the
 * operations and are bound through template_observer_if_t. A parent Motor
 * Driver owns the
 * interface and does not know the concrete algorithm type.
 *
 * The interface is intentionally limited to the fast control path:
 *
 *     ISR physical values -> fnIsrStep() -> observer output
 *
 * It has no hardware dependency, no PT cursor and no Run() requirement.
 * Algorithm parameters and mutable algorithm state belong to the concrete
 * object pointed to by pContext. Replace the template prefix and physical
 * quantities with domain-specific names when copying this file.
 *
 * For a single-core target, fnIsrStep() must be bounded, non-blocking and
 * free of logging, bus access, dynamic allocation and hidden static state.
 * Bind the selected implementation during initialization; do not use a
 * global registry or switch algorithm implementations from the ISR.
 */

#include <stdbool.h>
#include <stdint.h>

#include "template_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Physical quantities supplied to one observer control step.
 *
 * The concrete module may replace this structure with quantities matching
 * its plant model. All fields must document units and numeric range.
 */
typedef struct {
    float fIAlpha;
    float fIBeta;
    float fUAlpha;
    float fUBeta;
} template_observer_input_t;

/**
 * @brief Common observer result consumed by a parent Motor Driver.
 */
typedef struct {
    float    fTheta;
    float    fOmega;
    bool     bValid;
    uint32_t wFaults;
} template_observer_output_t;

/**
 * @brief Operations implemented by one concrete observer algorithm.
 *
 * fnInit and fnReset execute in foreground context. fnIsrStep executes in
 * the caller's ISR/control-loop context and must not call the PT scheduler.
 * A concrete algorithm may remove fnReset when reset is not meaningful.
 */
typedef struct {
    template_interface_result_t (*fnInit)(void *pContext);
    void (*fnIsrStep)(void *pContext,
                      const template_observer_input_t *ptInput,
                      template_observer_output_t *ptOutput);
    template_interface_result_t (*fnReset)(void *pContext);
} template_observer_ops_t;

/**
 * @brief Bind one concrete observer object to its operations.
 */
typedef struct {
    const template_observer_ops_t *ptOps;
    void                           *pContext;
} template_observer_if_t;

#ifdef __cplusplus
}
#endif

#endif /* MODUS_TEMPLATE_OBSERVER_H */
