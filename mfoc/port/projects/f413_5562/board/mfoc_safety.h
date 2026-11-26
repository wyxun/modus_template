/**
 * @file mfoc_safety.h
 * @brief AT32F413 EVB phase, raw-current and missing-frame protection.
 * @author Codex
 * @date 2026-11-25
 */
#ifndef MFOC_AT32_SAFETY_H
#define MFOC_AT32_SAFETY_H

#include "mfoc_types.h"

/** @brief First guard fault in the current run. */
typedef enum {
    MFOC_AT32_SAFETY_NONE = 0,
    MFOC_AT32_SAFETY_PHASE,
    MFOC_AT32_SAFETY_OVERCURRENT,
    MFOC_AT32_SAFETY_HALL,
    MFOC_AT32_SAFETY_MISSING_FRAME,
    MFOC_AT32_SAFETY_BREAK,
    MFOC_AT32_SAFETY_GATE,
} mfoc_at32_safety_fault_t;

/** @brief Wait for a natural top update and its first ADC frame.
 * @return OK, SAFETY for an invalid gate/phase, or IO for missing ADC data.
 * @note Foreground, bridge stopped, ADC consumer disabled, trigger running. */
mfoc_result_t mfoc_at32_safety_WaitStart(void);

/** @brief Check the descending near-top gate-enable window.
 * @return True if the timer is still within the window.
 * @note Port rechecks under IRQ mask before opening the bridge. */
bool mfoc_at32_safety_StartWindowOpen(void);

/** @brief Publish calibrated raw thresholds and clear the previous fault.
 * @param offset_u U midpoint. @param offset_v V midpoint.
 * @param offset_w W midpoint. @return None.
 * @note Serialize with ADC and SysTick. Call after a successful WaitStart. */
void mfoc_at32_safety_Arm(uint32_t offset_u, uint32_t offset_v,
                         uint32_t offset_w);

/** @brief Disarm the guard before closing the outputs. @return None. */
void mfoc_at32_safety_Disarm(void);

/** @brief Read the first guard fault. @return Latched fault or NONE. */
mfoc_at32_safety_fault_t mfoc_at32_safety_GetFault(void);

/** @brief Check phase and calibrated raw signals for one ready ADC frame.
 * @return True to continue control or false after a guard fault. */
bool mfoc_at32_safety_Frame(void);

/** @brief Check frame freshness and bridge faults every 1 ms. @return None. */
void mfoc_at32_safety_Tick(void);

#endif
