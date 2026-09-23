#ifndef MODUS_TEMPLATE_DRIVER_H
#define MODUS_TEMPLATE_DRIVER_H

/*
 * Template reading guide:
 *
 * 1. This file is a capability template, not a mandatory API base class.
 * 2. template_driver_cfg_t is a boundary input in developer-facing units.
 *    Do not embed the whole configuration in template_driver_t.
 * 3. Init validates and converts configuration into compact execution data;
 *    template_driver_t owns that data and all mutable per-instance state.
 * 4. The ops/context pair injects one external dependency instance.
 * 5. Public APIs operate on an explicit template_driver_t pointer.
 *
 * Capability selection:
 *
 * - Init is common to every concrete Driver.
 * - Run/Start/Stop/Reset and the PT cursor are the optional foreground
 *   service profile. Keep them for startup, calibration or asynchronous I/O;
 *   remove them from a concrete algorithm that is only called by an ISR.
 * - Clock is an optional fixed-rate service and is not a second Run().
 * - IsrStep below is an optional short hardware service with no data model.
 *   A concrete ISR consumes prevalidated execution fields in its object;
 *   it must not read cfg or convert physical units on each invocation.
 *   An algorithm that consumes physical values should define a typed
 *   interface such as template_observer_if_t from template_observer.h and
 *   expose its own typed xxx_IsrStep() function. Do not pass algorithm data
 *   through void pointers.
 *
 * The current template keeps the sample/apply operations so that the PT
 * service profile remains buildable and testable. They are examples, not a
 * universal Driver contract. A concrete SMO, flux or EKF module should delete
 * that example section and keep only its own typed calculation interface.
 *
 * Internal function convention:
 * `_template_driver_*` names identify file-private helper functions in the
 * implementation. The `static` keyword is the actual C linkage boundary;
 * the underscore is only a project convention and is not an access control.
 * Internal variables follow the same ownership rule: a local variable is
 * temporary operation data, while persistent mutable data belongs in the
 * template_driver_t object. Do not hide persistent state in a file-static or
 * function-static variable.
 *
 * Static data policy:
 *
 * - static const tables and immutable default ops are allowed.
 * - MODUS-required base objects/configuration are allowed and documented.
 * - an explicitly owned single-instance resource may be static when its
 *   ownership and lifetime are obvious from the surrounding template.
 * - mutable status, timers, caches, faults, commands and PT cursors must be
 *   members of template_driver_t, never hidden file-static runtime state.
 *
 * When this file is copied, choose the required capability profile first.
 * Replace template_driver_* and the placeholder hardware interface with the
 * concrete module prefix and typed dependencies, then delete unused sections.
 */

#include <stdbool.h>
#include <stdint.h>

#include "perf_counter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Detailed result returned by template Driver control APIs.
 */
typedef enum {
    TEMPLATE_DRIVER_RESULT_OK = 0,
    TEMPLATE_DRIVER_RESULT_NULL,
    TEMPLATE_DRIVER_RESULT_INVALID_ARGUMENT,
    TEMPLATE_DRIVER_RESULT_NOT_READY,
    TEMPLATE_DRIVER_RESULT_BUSY,
    TEMPLATE_DRIVER_RESULT_TIMEOUT,
    TEMPLATE_DRIVER_RESULT_IO,
    TEMPLATE_DRIVER_RESULT_FAULT,
} template_driver_result_t;

/**
 * @brief Runtime state of the template Driver.
 */
typedef enum {
    TEMPLATE_DRIVER_STATE_UNINITIALIZED = 0,
    TEMPLATE_DRIVER_STATE_IDLE,
    TEMPLATE_DRIVER_STATE_RUNNING,
    TEMPLATE_DRIVER_STATE_ERROR,
} template_driver_state_t;

#define TEMPLATE_DRIVER_FAULT_ARGUMENT  (1UL << 0)
#define TEMPLATE_DRIVER_FAULT_TIMEOUT   (1UL << 1)
#define TEMPLATE_DRIVER_FAULT_IO        (1UL << 2)
#define TEMPLATE_DRIVER_FAULT_STATE     (1UL << 3)

typedef template_driver_result_t (*template_driver_fn_init_t)(void *pContext);
typedef template_driver_result_t (*template_driver_fn_sample_t)(
    void *pContext,
    uint32_t *pwValue);
typedef template_driver_result_t (*template_driver_fn_apply_t)(
    void *pContext,
    uint32_t wValue);
typedef template_driver_result_t (*template_driver_fn_stop_t)(void *pContext);
typedef template_driver_result_t (*template_driver_fn_clock_t)(
    void *pContext);
typedef template_driver_result_t (*template_driver_fn_isr_step_t)(
    void *pContext);

/**
 * @brief Template-only placeholder for one concrete external dependency.
 *
 * A copied hardware module should replace this type with a semantic interface
 * such as motor_adc_ops_t or motor_pwm_ops_t. Algorithm modules normally do
 * not need this interface; use a typed family interface instead.
 */
typedef struct {
    template_driver_fn_init_t    fnInit;
    template_driver_fn_sample_t  fnSample;
    template_driver_fn_apply_t   fnApply;
    template_driver_fn_stop_t    fnStop;
    template_driver_fn_clock_t   fnClock;
    template_driver_fn_isr_step_t fnIsrStep;
} template_driver_hw_ops_t;

/**
 * @brief Pair one operations table with one dependency instance.
 */
typedef struct {
    const template_driver_hw_ops_t *ptOps;
    void                            *pContext;
} template_driver_hw_if_t;

/**
 * @brief Boundary configuration in units meaningful to the caller.
 *
 * wTickRateHz describes the tick source passed to Run(). wUpdateRateHz is
 * the requested service frequency. Init rejects unrepresentable rates and
 * rounds the period up to a whole tick, so the actual rate never exceeds the
 * request. The configuration is not retained after initialization.
 */
typedef struct {
    template_driver_hw_if_t tHw;
    uint32_t                wInitialValue;
    uint32_t                wTickRateHz;
    uint32_t                wUpdateRateHz;
} template_driver_cfg_t;

/**
 * @brief Read-only state snapshot returned to a parent Class.
 */
typedef struct {
    template_driver_state_t  eState;
    template_driver_result_t eLastError;
    uint32_t                 wFaults;
    uint32_t                 wValue;
    bool                     bTimerActive;
} template_driver_status_t;

/**
 * @brief Runtime object owned by a parent Class.
 *
 * wPeriodTicks is the checked execution parameter used by the service path.
 * Keep only fields needed by later operations; never add a cfg member here.
 */
typedef struct {
    template_driver_hw_if_t tHw;
    template_driver_state_t eState;
    template_driver_result_t eLastError;
    uint32_t                 wFaults;
    uint8_t                  chRunPt;
    bool                     bTimerActive;
    uint32_t                 wStartTick;
    uint32_t                 wPeriodTicks;
    uint32_t                 wValue;
} template_driver_t;

/**
 * @brief Initialize one caller-owned Driver object.
 * @param ptThis Caller-owned Driver object.
 * @param ptCfg Initialization configuration and dependency bindings.
 * @return Detailed initialization result; no handle is returned.
 */
template_driver_result_t template_driver_Init(
    template_driver_t *ptThis,
    const template_driver_cfg_t *ptCfg);

/**
 * @brief Advance one non-blocking Driver PT step.
 *
 * This function belongs to the optional foreground service profile. It is
 * intentionally absent from a concrete Driver that has no asynchronous flow.
 * @param ptThis Caller-owned Driver object.
 * @param wNowTick Explicit soft-clock tick sampled by the parent Class.
 * @return PT progress; detailed failures remain in the Driver status.
 */
fsm_rt_t template_driver_Run(template_driver_t *ptThis,
                             uint32_t wNowTick);

/**
 * @brief Start the optional periodic Driver flow.
 * @param ptThis Caller-owned Driver object.
 * @return Detailed state-transition result.
 */
template_driver_result_t template_driver_Start(template_driver_t *ptThis);

/**
 * @brief Stop the optional Driver service and request safe external output.
 * @param ptThis Caller-owned Driver object.
 * @return None; failures are latched in the Driver status.
 */
void template_driver_Stop(template_driver_t *ptThis);

/**
 * @brief Execute one optional fixed-rate hard-clock service.
 * @param ptThis Caller-owned Driver object.
 * @return None; failures are latched in the Driver status.
 * @note This entry must not advance the foreground PT or block.
 */
void template_driver_Clock(template_driver_t *ptThis);

/**
 * @brief Execute one optional bounded hardware ISR service.
 * @param ptThis Caller-owned Driver object.
 * @return None; failures are latched in the Driver status.
 * @note This is not the typed algorithm entry used by SMO/flux/EKF modules.
 * @note Do not use PT, logs, bus transactions, dynamic memory or blocking.
 */
void template_driver_IsrStep(template_driver_t *ptThis);

/**
 * @brief Clear latched Driver faults after a safe stop.
 * @param ptThis Caller-owned Driver object.
 * @return Detailed reset result.
 */
template_driver_result_t template_driver_Reset(template_driver_t *ptThis);

/**
 * @brief Copy a read-only status snapshot.
 * @param ptThis Caller-owned Driver object.
 * @param ptStatus Destination status snapshot.
 * @return Detailed query result.
 */
template_driver_result_t template_driver_GetStatus(
    const template_driver_t *ptThis,
    template_driver_status_t *ptStatus);

/*
 * The Clock and hardware IsrStep entries are intentionally present in this
 * hardware/PT example. A concrete Driver that does not need one removes its
 * unused declaration, callback and implementation during specialization.
 *
 * A concrete observer interface is defined separately in template_observer.h
 * using the rules in template_interface.h. It is implemented by SMO, flux and
 * EKF objects; it is not another Driver object and it is not a global
 * registration mechanism.
 */

#ifdef __cplusplus
}
#endif

#endif /* MODUS_TEMPLATE_DRIVER_H */
