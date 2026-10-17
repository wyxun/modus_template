/**
 * @file    foc_debug.h
 * @brief   Debug command and waveform bindings for the FOC application.
 * @note    Shell commands use the existing single foc_app instance; this is
 *          a mechanical split from foc_app.c and does not add a command API.
 *
 * @section foc_debug_shell_usage Shell command usage
 *
 * The commands below are available when MSHELL_ENABLE is enabled:
 * Floating arguments are decimal per-unit values, e.g. -0.05, +1.25, .5, 1.
 * Use spaces or tabs between arguments; exponents, hex floats, NaN/Inf and
 * unit suffixes are rejected. Scripts should emit fixed decimal notation.
 * Speed has a shell bound of [-1, 1] pu and Motor's configured speed limit.
 * Current/voltage vectors are checked against Motor's configured limits.
 * Step requires 0 < q <= 0.05 pu and unsigned decimal ms in [25, 150].
 * Identification commands accept no numerical arguments. Extra arguments
 * are rejected. Numeric parsing alone does not establish a safe motor limit.
 *
 *   motor speed <pu>       Start speed control with a per-unit reference.
 *   motor current <d> <q>  Start current control, or update live refs.
 *   motor step <q> <ms>    Apply a bounded q-current pulse, then stop PWM.
 *   motor voltage <d> <q>  Start voltage control with d/q references.
 *   motor align             Hold the rotor without capturing encoder zero.
 *   motor calibrate         Align and capture the encoder electrical zero.
 *   motor stop              Stop the motor and identification operation.
 *   motor clear             Clear motor faults.
 *   motor status            Print motor state and control values.
 *   motor encoder           Print the latest encoder position and speed.
 *   motor encoder config    Read the encoder's volatile filter configuration.
 *   motor encoder filter N  Select SF 0..3 while stopped; verify readback.
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
 * Residual diagnosis: raw SMO angle, measured-minus-estimated alpha/beta
 * current and independent encoder electrical angle. EncAbs has no zero offset;
 * infer that constant offset offline from the initial alignment hold.
 * The encoder probe reads only its published cache, never I2C in ISR.
 * Angles use turns, current residuals use PU; streamer scale is 1000.
 * Speed probes emit -2 outside RUNNING/PWM-on or when invalid/not ready.
 * Values beyond +/-32 PU are invalid, avoiding int16 overflow at scale 1000.
 * Optional current probes emit -2 outside RUNNING/PWM-on or if nonfinite.
 * Keep at most six channels; this trial selects exactly four at 10 kHz.
 */
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE
#define FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_SPEED
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_SPEED 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_FILTERED_SPEED
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_FILTERED_SPEED 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_REFERENCE
#define FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_REFERENCE 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR
#define FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_STRENGTH
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_STRENGTH 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_IQ
#define FOC_DEBUG_WAVEFORM_ENABLE_IQ 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_CURRENT_ALPHA
#define FOC_DEBUG_WAVEFORM_ENABLE_CURRENT_ALPHA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_CURRENT_BETA
#define FOC_DEBUG_WAVEFORM_ENABLE_CURRENT_BETA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA 1
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA 0
#endif
#ifndef FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA
#define FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA 0
#endif

#define FOC_DEBUG_WAVEFORM_CHANNEL_COUNT                                        \
    (FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE +                                  \
     FOC_DEBUG_WAVEFORM_ENABLE_ENCODER_REFERENCE +                              \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_ANGLE +                                      \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SPEED +                                      \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_FILTERED_SPEED +                             \
     FOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR +                                    \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_STRENGTH +                              \
     FOC_DEBUG_WAVEFORM_ENABLE_IQ +                                             \
     FOC_DEBUG_WAVEFORM_ENABLE_CURRENT_ALPHA +                                  \
     FOC_DEBUG_WAVEFORM_ENABLE_CURRENT_BETA +                                   \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA +                                 \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA +                                  \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA +                        \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA +                         \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA +                              \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_BETA)

#if FOC_DEBUG_WAVEFORM_CHANNEL_COUNT > 6
#error "FOC waveform supports at most six channels"
#endif
#if FOC_DEBUG_WAVEFORM_CHANNEL_COUNT == 0
#error "Enable at least one FOC waveform channel"
#endif
#if !FOC_OBSERVER_HAS_SMO &&                                                    \
    (FOC_DEBUG_WAVEFORM_ENABLE_SMO_SPEED ||                                     \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_FILTERED_SPEED ||                            \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_STRENGTH ||                             \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA ||                                \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA ||                                 \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA ||                       \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA ||                        \
     FOC_DEBUG_WAVEFORM_ENABLE_SMO_SLIDING_ALPHA ||                             \
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
/** @brief Prepare bound values on an accepted waveform sample.
 * @param None. @return None. @note Called synchronously by mwaveform.Step. */
void foc_debug_CaptureWaveform(void);
#endif

#endif /* FOC_DEBUG_H */
