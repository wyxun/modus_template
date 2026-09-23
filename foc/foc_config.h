/****************************************************************************
 * @file    foc_config.h
 * @brief   User-selectable FOC build features and compile-time validation.
 *
 * Numeric and trig backends are selected in foc_numeric.h and foc_trig.h.
 * Board settings belong to the selected target; motor and application
 * settings belong to app/motor_config.h.
 ****************************************************************************/

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* Public build options. Set these in the target build configuration. */
#ifndef FOC_OFFSET_CALIB_TIMES
#define FOC_OFFSET_CALIB_TIMES              200U
#endif

#ifndef FOC_ENABLE_EXPERIMENTAL_NSD
#define FOC_ENABLE_EXPERIMENTAL_NSD       0
#endif

/* User feature switch. foc.mk always lists foc_smo.c;
 * this macro selects the Observer backend.
 */
#ifndef FOC_ENABLE_SMO
#define FOC_ENABLE_SMO                    1
#endif

#ifndef FOC_ENABLE_HFI
#define FOC_ENABLE_HFI                    0
#endif

/* Observer implementation selected for this FOC build. */
#define FOC_OBSERVER_BACKEND_NONE          0
#define FOC_OBSERVER_BACKEND_SMO           1

#ifndef FOC_OBSERVER_BACKEND
#if FOC_ENABLE_SMO
#define FOC_OBSERVER_BACKEND               FOC_OBSERVER_BACKEND_SMO
#else
#define FOC_OBSERVER_BACKEND               FOC_OBSERVER_BACKEND_NONE
#endif
#endif

#if (FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE) && \
    (FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_SMO)
#error "Unsupported FOC_OBSERVER_BACKEND"
#endif

#if (FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO) && !FOC_ENABLE_SMO
#error "SMO observer backend requires FOC_ENABLE_SMO=1"
#endif

#endif /* FOC_CONFIG_H */
