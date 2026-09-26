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
