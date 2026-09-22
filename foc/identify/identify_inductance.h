/****************************************************************************
 * @file    identify_inductance.h
 * @brief   Private Phase 1 Ld identification declarations.
 * @author  Codex
 * @date    2026-09-22
 ****************************************************************************/

#ifndef IDENTIFY_INDUCTANCE_H
#define IDENTIFY_INDUCTANCE_H

#include "identify.h"
#include "perfc_task_pt.h"

foc_result_t _identify_inductance_Start(
    identify_t *ptThis,
    const identify_inductance_cfg_t *ptConfig);

void _identify_inductance_IsrStep(
    identify_t *ptThis,
    motor_t *ptMotor,
    const identify_isr_sample_t *ptSample);

fsm_rt_t _identify_inductance_RunPt(identify_t *ptThis,
                                     motor_t *ptMotor);

void _identify_inductance_Stop(identify_t *ptThis);

void _identify_inductance_Reset(identify_t *ptThis);

#endif /* IDENTIFY_INDUCTANCE_H */
