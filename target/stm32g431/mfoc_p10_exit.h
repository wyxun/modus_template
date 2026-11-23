/** @file mfoc_p10_exit.h
 * Low-priority hardware witness for completion of the G431 ADC exception.
 */
#ifndef MFOC_G431_P10_EXIT_H
#define MFOC_G431_P10_EXIT_H

#include <stdint.h>

#define MFOC_P10_EXIT_SPEED    (1U << 0)
#define MFOC_P10_EXIT_SMO      (1U << 1)
#define MFOC_P10_EXIT_HFI_8    (1U << 2)
#define MFOC_P10_EXIT_LIMIT    (1U << 3)
#define MFOC_P10_EXIT_BUS      (1U << 4)
#define MFOC_P10_EXIT_ABNORMAL (1U << 5)

typedef enum {
    MFOC_P10_EXIT_ALL = 0,
    MFOC_P10_EXIT_ERRORS,
    MFOC_P10_EXIT_SMO_WINDOW,
    MFOC_P10_EXIT_HFI_EIGHTH,
    MFOC_P10_EXIT_LIMITED,
    MFOC_P10_EXIT_BUS_NEW,
    MFOC_P10_EXIT_SPEED_SMO,
    MFOC_P10_EXIT_SPEED_HFI,
    MFOC_P10_EXIT_BUS_SPEED_SMO,
    MFOC_P10_EXIT_BUS_SPEED_HFI,
    MFOC_P10_EXIT_BUS_SPEED_SMO_LIMIT,
    MFOC_P10_EXIT_BUS_SPEED_HFI_LIMIT,
    MFOC_P10_EXIT_BUS_SPEED_SMO_HFI,
    MFOC_P10_EXIT_BUS_SPEED_SMO_HFI_LIMIT,
    MFOC_P10_EXIT_LIMIT_ONLY,
    MFOC_P10_EXIT_SPEED_SMO_LIMIT,
    MFOC_P10_EXIT_GROUP_COUNT,
} mfoc_p10_exit_group_t;

typedef struct {
    uint32_t witnessed;
    uint32_t late;
    uint32_t max_age_ticks;
    uint32_t seq;
    uint32_t trigger;
    uint32_t compare; /* Request timestamp in the capture format. */
    uint32_t observed;
    uint32_t flags;
} mfoc_p10_exit_summary_t;

typedef struct {
    uint32_t magic;
    uint32_t trigger_guard_ticks;
    uint32_t armed;
    uint32_t replaced;
    uint32_t untagged; /* Kept for decoding earlier compare captures. */
    uint32_t bad_compare;
    uint32_t active_adc;
    uint32_t bad_priority;
    uint32_t outstanding; /* Pending witness at the time a stopped dump reads. */
    mfoc_p10_exit_summary_t groups[MFOC_P10_EXIT_GROUP_COUNT];
} mfoc_p10_exit_evidence_t;

extern mfoc_p10_exit_evidence_t g_p10_exit_witness;

/** @brief Initialize/reset the witness with the phase timer already running. */
void mfoc_p10_exit_Init(void);
/** @brief Clear stopped statistics and the pending witness. */
void mfoc_p10_exit_Reset(void);
/** @brief Pend the lower-priority witness at the end of the ADC path.
 * @param seq Frame sequence. @param trigger ADC trigger reference.
 * @param period Frame period in TIM2 ticks. @param flags Recorded paths. */
void mfoc_p10_exit_Pend(uint32_t seq, uint32_t trigger, uint32_t period,
                       uint32_t flags);

#endif
