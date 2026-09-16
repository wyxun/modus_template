#ifndef MODUS_TEMPLATE_INTERFACE_H
#define MODUS_TEMPLATE_INTERFACE_H

/*
 * Strong interface template for C11:
 *
 * This file defines the interface pattern, not a concrete object and not a
 * universal Driver base class. A family interface owns its semantic input,
 * output and operations types. template_observer.h is one example.
 *
 * TEMPLATE_INTERFACE_SELECT() is compile-time dispatch, not runtime
 * polymorphism. Keep its concrete type list in a family or application
 * adapter, not in this common header. A new algorithm must therefore change
 * the adapter instead of changing the common interface template.
 *
 * Use _Generic when the implementation is fixed by the build and direct
 * calls or inlining matter on the hot path. Use an ops/context interface when
 * the implementation is selected by configuration. Do not maintain both
 * dispatch mechanisms for the same call chain.
 */

/**
 * @brief Minimal result shared by interface lifecycle operations.
 *
 * A concrete interface family may extend this enum when it needs domain
 * specific detail. Driver-specific results remain in template_driver.h.
 */
typedef enum {
    TEMPLATE_INTERFACE_RESULT_OK = 0,
    TEMPLATE_INTERFACE_RESULT_INVALID_ARGUMENT,
    TEMPLATE_INTERFACE_RESULT_NOT_READY,
    TEMPLATE_INTERFACE_RESULT_FAULT,
} template_interface_result_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Select one function from a caller-owned C11 type list.
 *
 * Example:
 *
 * #define observer_IsrStep(ptObject, ...) \
 *     TEMPLATE_INTERFACE_SELECT(ptObject, \
 *         smo_t *: smo_IsrStep, \
 *         flux_t *: flux_IsrStep, \
 *         ekf_t *: ekf_IsrStep)(ptObject, __VA_ARGS__)
 *
 * The selected function still receives compiler-checked arguments. Different
 * concrete functions may consequently have different parameter lists.
 */
#define TEMPLATE_INTERFACE_SELECT(ptObject, ...) \
    _Generic((ptObject), __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* MODUS_TEMPLATE_INTERFACE_H */
