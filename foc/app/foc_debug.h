/**
 * @file    foc_debug.h
 * @brief   Debug command and waveform bindings for the FOC application.
 * @note    Shell commands use the existing single foc_app instance; this is
 *          a mechanical split from foc_app.c and does not add a command API.
 *
 * @section foc_debug_shell_usage Shell command usage
 *
 * The commands below are available when MSHELL_ENABLE is enabled:
 *
 *   motor speed <pu>       Start speed control with a per-unit reference.
 *   motor current <d> <q>  Start current control, or update live refs.
 *   motor step <q> <ms>    Apply a bounded q-current pulse, then stop PWM.
 *   motor voltage <d> <q>  Start voltage control with d/q references.
 *   motor align             Request electrical position calibration.
 *   motor stop              Stop the motor and identification operation.
 *   motor clear             Clear motor faults.
 *   motor status            Print motor state and control values.
 *   motor encoder           Print the latest encoder position and speed.
 *
 *   identify resistance     Start resistance identification.
 *   identify inductance     Start inductance identification.
 *   identify status         Print identification state and result.
 *   identify stop           Stop identification.
 *   identify reset          Stop and reset identification.
 */
#ifndef FOC_DEBUG_H
#define FOC_DEBUG_H

#include "foc_app.h"

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
/*
 * Compile-time channel selection: set switches to 0/1 here or with -D.
 * Default: active angle, SMO angle/error, Iq and SMO alpha/beta back-EMF.
 * Replace the back-EMF pair with current-error or sliding-voltage channels
 * when inspecting those SMO internals; the total must stay at six or less.
 */
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE
#define FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR
#define FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_IQ
#define FOC_DEBUG_WAVEFORM_ENABLE_IQ 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA
#if FOC_OBSERVER_HAS_SMO
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA 1
#else
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA 0
#endif
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA
#if FOC_OBSERVER_HAS_SMO
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA 1
#else
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA 0
#endif
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA 0
#endif

#define FOC_DEBUG_WAVEFORM_CHANNEL_COUNT                                    \
    (FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE +                              \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE +                                  \
     FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR +                                \
     FOC_DEBUG_WAVEFORM_ENABLE_IQ +                                         \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA +                             \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA +                              \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA +                    \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA +                     \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA +                          \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA)

#if FOC_DEBUG_WAVEFORM_CHANNEL_COUNT > 6
#error "FOC waveform supports at most six channels"
#endif
#if FOC_DEBUG_WAVEFORM_CHANNEL_COUNT == 0
#error "Enable at least one FOC waveform channel"
#endif
#if !FOC_OBSERVER_HAS_SMO &&                                                    \
    (FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA ||                            \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA ||                             \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA ||                   \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA ||                    \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA ||                         \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA)
#error "SMO internal waveform channels require the SMO observer backend"
#endif
#endif

/**
 * @brief Advance the foreground-timed current pulse test, if active.
 * @param ptApp FOC application object owning the pulse state.
 * @return None.
 */
void foc_debug_CurrentStepRun(foc_app_t *ptApp);

#if MWAVEFORM_ENABLE && defined(FOC_NUMERIC_FLOAT)
void foc_debug_WaveformInit(foc_app_t *ptApp,
                            uint32_t wPeriodNanoseconds);
void foc_debug_WaveformStep(void);
#endif

#endif /* FOC_DEBUG_H */
