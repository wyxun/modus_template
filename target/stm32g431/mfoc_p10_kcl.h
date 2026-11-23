/** @file mfoc_p10_kcl.h
 * Bounded electrical sampling check for the G431 FLOAT Flux bench image.
 */
#ifndef MFOC_G431_P10_KCL_H
#define MFOC_G431_P10_KCL_H

#include "mfoc_motor.h"

/** @brief Start a new compare history with the newly opened neutral PWM. */
void mfoc_p10_kcl_Reset(void);

/** @brief Check and retain the three independent channels of one ADC frame. */
bool mfoc_p10_kcl_Current(uint32_t capture, const mfoc_uvw_t *current,
                          uint16_t u, uint16_t v, uint16_t w);

/** @brief Record the compare group accepted for the next top transfer. */
void mfoc_p10_kcl_Duty(uint16_t u, uint16_t v, uint16_t w);

/** @brief Store the sample after the board accepts the complete Motor step. */
void mfoc_p10_kcl_AfterStep(mfoc_motor_t *motor);

#endif
