#ifndef __HALPWM_H__
#define __HALPWM_H__

#include <stdint.h>
#include <stdbool.h>

#include "mfoc_config.h"

/* PWM frequency: center-aligned, PCLK2 = 100MHz. Both the PWM and the ADC
 * trigger follow the MFOC control rate, so it has a single source. */
#define PWM_FREQ_HZ           MFOC_MOTOR_RUN_FREQ_HZ
#define PWM_PERIOD            ((100000000U / PWM_FREQ_HZ / 2U) - 1U)  /* 2499 */
#define HALF_PWM_PERIOD       (PWM_PERIOD / 2U)

/* Dead-time: Tdts = 1 / (PCLK2 / DIV) */
#define DEADTIME_CLK_DIV      TMR_CLOCK_DIV1
#define DEADTIME_NS           1000U         /* 1us */
/* DT = DEADTIME_NS / (1 / 100MHz) = 100 clocks */
#define DEADTIME_VAL          100U

void halpwm_Init(void);
void halpwm_Start(void);
void halpwm_Stop(void);
void halpwm_SetDuty(uint32_t wDutyU, uint32_t wDutyV, uint32_t wDutyW);

#endif /* __HALPWM_H__ */
