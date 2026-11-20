#ifndef __HALPWM_H__
#define __HALPWM_H__

#include <stdint.h>
#include <stdbool.h>

#include "at32f413.h"
#include "mfoc_config.h"

/* TMR1 is clocked at 2 * PCLK2. Center-aligned counting takes 2 * ARR. */
#define AT32F413_TMR1_HZ      200000000U
#define PWM_PERIOD            (AT32F413_TMR1_HZ / MFOC_MOTOR_RUN_FREQ_HZ / 2U)
#define DEADTIME_CLK_DIV      TMR_CLOCK_DIV2
#define DEADTIME_VAL          100U  /* 1 us at a 100 MHz dead-time clock. */

/** @brief Read the main output gate, which Break asynchronously clears.
 * @return True when phase outputs are enabled. */
static inline bool halpwm_OutputsEnabled(void)
{
    return TMR1->brk_bit.oen != 0U;
}

void halpwm_Init(void);
void halpwm_StartSampling(void);
bool halpwm_Start(void);
bool halpwm_Arm(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w);
void halpwm_Stop(void);
void halpwm_SetDuty(uint32_t wDutyU, uint32_t wDutyV, uint32_t wDutyW);

#endif /* __HALPWM_H__ */
