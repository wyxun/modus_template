#ifndef MODUS_TEMPLATE_CLASS_H
#define MODUS_TEMPLATE_CLASS_H

/*
 * Template reading guide:
 *
 * template_class_t is the MODUS object and owns one template_driver_t by
 * value. The Class owns product policy and Class state; the Driver owns its
 * own process state and external dependency calls.
 *
 * A concrete Class may own several Drivers by value or bind a child through a
 * typed interface. For example, motor_driver_t may own an observer_if_t whose
 * context points to an SMO, flux or EKF algorithm object. The interface is a
 * dependency boundary; it is not a second generic Driver base class.
 *
 * Internal function convention:
 * `_template_class_*` names identify file-private helpers. `static` provides
 * the actual file-local linkage; the underscore only communicates intent.
 * Public template_class_* functions are the only functions placed in the
 * MODUS callback table.
 *
 * Static data policy:
 *
 * - MODUS base bookkeeping, immutable default ops and explicitly owned
 *   template resources may be file-static.
 * - Class business state, Driver state, PT cursors, timers, faults, caches
 *   and commands must remain in template_class_t or its child Driver.
 * - A static variable must have a visible owner and lifetime. If a second
 *   Class instance could accidentally share it, move it into the object or
 *   inject it through the configuration.
 *
 * A concrete Class may call template_driver_IsrStep(&ptThis->tDriver) from
 * its own ISR entry when the hardware Driver declares an ISR-safe hook.
 * Algorithm children should instead be called through their typed IsrStep
 * interface with the physical values captured by the ISR.
 */

#include "modus.h"
#include "template_driver.h"

/**
 * @brief Business state of the parent Class.
 */
typedef enum {
    TEMPLATE_CLASS_STATE_UNINITIALIZED = 0,
    TEMPLATE_CLASS_STATE_IDLE,
    TEMPLATE_CLASS_STATE_RUNNING,
    TEMPLATE_CLASS_STATE_ERROR,
} template_class_state_t;

/**
 * @brief Initialization configuration for template_class.
 *
 * Child cfg is passed to template_driver_Init() and is not embedded in the
 * runtime template_class_t or template_driver_t objects.
 */
typedef struct {
    uint8_t *pchRingBuffer;
    uint16_t hwRingSize;
    const volatile uint32_t *pwSharedSystemTick;
    template_driver_cfg_t tDriverCfg;
} template_class_cfg_t;

/**
 * @brief MODUS Class object that owns one Driver instance.
 */
typedef struct {
    modus_base_t *ptBase;
    const volatile uint32_t *pwSharedSystemTick;
    template_driver_t tDriver;
    template_class_state_t eState;
    template_driver_result_t eDriverError;
    uint8_t chRunPt;
} template_class_t;

/**
 * @brief Initialize one MODUS Class object and its child Driver.
 * @param wObjectAddr Address of the caller-owned Class object.
 * @param wObjectCfgAddr Address of the Class configuration.
 * @return MODUS_SUCCESS on success, otherwise MODUS_EFAIL.
 */
int template_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/**
 * @brief Advance the parent Class and child Driver foreground PTs.
 * @param wObjectAddr Address of the Class object.
 * @return Driver PT progress or MODUS_EFAIL on a latched child error.
 */
int template_class_Run(uintptr_t wObjectAddr);

/**
 * @brief Execute the optional short Class clock callback.
 * @param wObjectAddr Address of the Class object.
 * @return MODUS_SUCCESS on success, otherwise MODUS_EFAIL.
 */
int template_class_Clock(uintptr_t wObjectAddr);

#endif /* MODUS_TEMPLATE_CLASS_H */
