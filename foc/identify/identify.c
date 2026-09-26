/****************************************************************************
 * @file    identify.c
 * @brief   Parameter-identification parent flow and dispatch.
 * @author  Modus project
 * @date    2026-09-22
 ****************************************************************************/

#include "identify.h"

#include <stddef.h>

#include "identify_inductance.h"
#include "identify_resistance.h"
#include "perf_counter.h"
#include "perfc_task_pt.h"

/**
 * @brief Initialize one identification object.
 * @param ptThis Caller-owned identification object.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Init(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptThis = (identify_t){0};
    ptThis->eState = IDENTIFY_STATE_IDLE;
    ptThis->eLastResult = FOC_RESULT_OK;
    return FOC_RESULT_OK;
}

/**
 * @brief Start the resistance child through the parent lifecycle boundary.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t identify_StartResistance(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptThis->eState == IDENTIFY_STATE_ERROR ||
        ptThis->eLastResult != FOC_RESULT_OK) {
        return FOC_RESULT_SAFETY;
    }
    if (ptThis->eState != IDENTIFY_STATE_IDLE ||
        ptThis->tResistance.bResultPending ||
        ptThis->tInductance.bResultPending) {
        return FOC_RESULT_BUSY;
    }
    return _identify_resistance_Start(ptThis);
}

foc_result_t identify_StartInductance(
    identify_t *ptThis,
    const identify_inductance_cfg_t *ptConfig)
{
    return _identify_inductance_Start(ptThis, ptConfig);
}

/**
 * @brief Record one high-frequency identification sample.
 * @param ptThis Identification object.
 * @param ptMotor Motor object that owns PWM and fault state.
 * @param ptSample Synchronized current sample.
 * @return None.
 */
void identify_IsrStep(identify_t *ptThis,
                      motor_t *ptMotor,
                      const identify_isr_sample_t *ptSample)
{
    if (ptThis == NULL || ptMotor == NULL || ptSample == NULL) {
        return;
    }
    switch (ptThis->eOperation) {
    case IDENTIFY_OPERATION_RESISTANCE:
        _identify_resistance_IsrStep(ptThis, ptSample->qCurrentD,
                                     ptSample->qVoltageD);
        break;
    case IDENTIFY_OPERATION_INDUCTANCE:
        _identify_inductance_IsrStep(ptThis, ptMotor, ptSample);
        break;
    case IDENTIFY_OPERATION_NONE:
    default:
        break;
    }
}

/**
 * @brief Advance the foreground identification state machine.
 * @param ptThis Identification object.
 * @param ptMotor Motor object controlled by this identification run.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or a measurement error.
 */
foc_result_t identify_Run(identify_t *ptThis, motor_t *ptMotor)
{
    fsm_rt_t ePtResult = fsm_rt_on_going;

    if (ptThis == NULL || ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptThis->eState == IDENTIFY_STATE_ERROR) {
        return ptThis->eLastResult;
    }
    if (ptThis->eState == IDENTIFY_STATE_IDLE) {
        return (ptThis->tResistance.bResultPending ||
                ptThis->tInductance.bResultPending) ?
            FOC_RESULT_OK : FOC_RESULT_BUSY;
    }
    switch (ptThis->eOperation) {
    case IDENTIFY_OPERATION_RESISTANCE:
        ePtResult = _identify_resistance_RunPt(ptThis, ptMotor);
        break;
    case IDENTIFY_OPERATION_INDUCTANCE:
        ePtResult = _identify_inductance_RunPt(ptThis, ptMotor);
        break;
    case IDENTIFY_OPERATION_NONE:
    default:
        return FOC_RESULT_DISABLED;
    }
    if (ePtResult == fsm_rt_err) {
        return ptThis->eLastResult;
    }
    return (ePtResult == fsm_rt_cpl) ? FOC_RESULT_OK : FOC_RESULT_BUSY;
}

/**
 * @brief Stop identification and the controlled Motor.
 * @param ptThis Identification object.
 * @param ptMotor Motor object, or NULL when no Motor is available.
 * @return None.
 */
void identify_Stop(identify_t *ptThis, motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis == NULL) {
        if (ptMotor != NULL) {
            motor_Stop(ptMotor);
        }
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptThis->eState != IDENTIFY_STATE_UNINITIALIZED) {
        ptThis->eOperation = IDENTIFY_OPERATION_NONE;
        ptThis->eState = IDENTIFY_STATE_IDLE;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    if (ptMotor != NULL) {
        motor_Stop(ptMotor);
    }
    _identify_resistance_Stop(ptThis);
    _identify_inductance_Stop(ptThis);
}

/**
 * @brief Reset identification state after stop or error.
 * @param ptThis Identification object.
 * @param ptMotor Motor whose PWM state is checked before reset.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Reset(identify_t *ptThis,
                            const motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    motor_status_t tStatus = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL || ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = motor_GetStatus(ptMotor, &tStatus);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (tStatus.bPwmEnabled) {
        return FOC_RESULT_BUSY;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptThis->eOperation != IDENTIFY_OPERATION_NONE) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptThis->eState = IDENTIFY_STATE_IDLE;
    ptThis->eLastResult = FOC_RESULT_OK;
    perfc_port_resume_global_interrupt(tIrqState);
    _identify_resistance_Reset(ptThis);
    _identify_inductance_Reset(ptThis);
    return FOC_RESULT_OK;
}

/**
 * @brief Copy a status snapshot.
 * @param ptThis Identification object.
 * @param ptStatus Destination status snapshot.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_GetStatus(const identify_t *ptThis,
                                identify_status_t *ptStatus)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptThis == NULL || ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptStatus->eState = ptThis->eState;
    ptStatus->eOperation = ptThis->eOperation;
    ptStatus->eLastResult = ptThis->eLastResult;
    ptStatus->bResultPending =
        ptThis->tResistance.bResultPending ||
        ptThis->tInductance.bResultPending;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
