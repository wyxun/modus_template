/****************************************************************************
 * @file    identify_resistance.h
 * @brief   Private resistance-identification declarations.
 ****************************************************************************/

#ifndef FOC_IDENTIFY_RESISTANCE_H
#define FOC_IDENTIFY_RESISTANCE_H

#include "identify.h"
#include "perfc_task_pt.h"

foc_result_t _identify_resistance_Start(identify_t *ptThis);

void _identify_resistance_IsrStep(identify_t *ptThis,
                                  foc_scalar_t qCurrentD,
                                  foc_scalar_t qVoltageD);

fsm_rt_t _identify_resistance_RunPt(identify_t *ptThis,
                                     motor_t *ptMotor);

void _identify_resistance_Stop(identify_t *ptThis);

void _identify_resistance_Reset(identify_t *ptThis);

#endif /* FOC_IDENTIFY_RESISTANCE_H */
