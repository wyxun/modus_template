/****************************************************************************
 * @file    foc_log_config.h
 * @brief   Compile-time switches for FOC diagnostic-only data and logs.
 * @author  Codex
 * @date    2026-09-26
 ****************************************************************************/

#ifndef FOC_LOG_CONFIG_H
#define FOC_LOG_CONFIG_H

/* Print the three ADC zero-current offsets once after startup calibration. */
#ifndef FOC_APP_LOG_ADC_OFFSETS
#define FOC_APP_LOG_ADC_OFFSETS 1
#endif

/* Periodic SMO quality statistics; observer state remains unconditional. */
#ifndef FOC_APP_LOG_SMO_DIAGNOSTICS
#define FOC_APP_LOG_SMO_DIAGNOSTICS 0
#endif

/* Resistance input snapshot and identification result detail. */
#ifndef FOC_APP_LOG_RESISTANCE_ID
#define FOC_APP_LOG_RESISTANCE_ID 1
#endif

/* Inductance polarity snapshots and identification result detail. */
#ifndef FOC_APP_LOG_INDUCTANCE_ID
#define FOC_APP_LOG_INDUCTANCE_ID 1
#endif

/* Periodic HF ISR runtime and ADC-trigger-to-CCR timing statistics. */
#ifndef FOC_APP_LOG_TIMING_DIAGNOSTICS
#define FOC_APP_LOG_TIMING_DIAGNOSTICS 0
#endif

#endif /* FOC_LOG_CONFIG_H */
