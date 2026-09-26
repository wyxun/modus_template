/****************************************************************************
 * @file    motor_position.c
 * @brief   Sensor conversion and optional shadow position estimation.
 ****************************************************************************/
#include "motor_position.h"

#include <stddef.h>

#include "foc_port.h"

static foc_angle_t _motor_position_ToElectrical(
    const motor_position_t *ptPosition, foc_angle_t tMechanical)
{
    foc_angle_t tElectrical = {0U};
    tElectrical.wBam32 = (uint32_t)(
        (uint64_t)tMechanical.wBam32 * ptPosition->chPolePairs);
    return tElectrical;
}

foc_result_t motor_position_Init(motor_position_t *ptPosition,
                                  const motor_position_cfg_t *ptConfig)
{
    foc_scalar_t qPolePairs = FOC_ZERO;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptConfig->chPolePairs == 0U ||
        ptConfig->qElectricalSpeedBaseTurnsPerSecond <= FOC_ZERO ||
        ptConfig->eSource > MOTOR_POSITION_SOURCE_HARD_DRAG ||
        (ptConfig->eSource == MOTOR_POSITION_SOURCE_SENSOR &&
         ptConfig->tSensor.pContext == NULL)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if !defined(FOC_POSITION_STATIC_BINDING)
    if (ptConfig->eSource == MOTOR_POSITION_SOURCE_SENSOR &&
        ptConfig->tSensor.fnGetPosition == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#endif
    *ptPosition = (motor_position_t){0};
    ptPosition->pSensorState = ptConfig->tSensor.pContext;
#if !defined(FOC_POSITION_STATIC_BINDING)
    ptPosition->tSensor = ptConfig->tSensor;
#endif
    ptPosition->chPolePairs = ptConfig->chPolePairs;
    ptPosition->eSource = ptConfig->eSource;
    qPolePairs = foc_from_float((float)ptConfig->chPolePairs);
    eResult = foc_div_checked(
        qPolePairs, ptConfig->qElectricalSpeedBaseTurnsPerSecond,
        &ptPosition->qMechanicalToElectricalSpeedPuGain);
    if (eResult != FOC_RESULT_OK ||
        ptPosition->qMechanicalToElectricalSpeedPuGain <= FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    if (ptConfig->ptMotorParams == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = foc_observer_Init(&ptPosition->tObserver,
                                ptConfig->ptMotorParams,
                                &ptConfig->tObserverCfg);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
#endif
    return FOC_RESULT_OK;
}

foc_result_t motor_position_Step(
    motor_position_t *ptPosition, uint32_t wNowTick,
    const motor_position_sample_t *ptSample,
    motor_electrical_feedback_t *ptFeedback)
{
    foc_position_t tMechanical = {0};
    foc_angle_t tElectrical = {0U};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL || ptSample == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptFeedback = (motor_electrical_feedback_t){0};
    if (ptPosition->wLastRunGeneration != ptSample->wRunGeneration) {
        motor_position_ResetObserver(ptPosition);
        ptPosition->wLastRunGeneration = ptSample->wRunGeneration;
    }
    if (ptPosition->eSource == MOTOR_POSITION_SOURCE_HARD_DRAG) {
        if (!ptSample->tHardDragCandidate.bValid) {
            return FOC_RESULT_SAFETY;
        }
        *ptFeedback = ptSample->tHardDragCandidate;
        return FOC_RESULT_OK;
    }
    eResult = FOC_SENSOR_POSITION_GET(ptPosition, wNowTick, &tMechanical);
    if (eResult != FOC_RESULT_OK || !tMechanical.bValid) {
        return eResult == FOC_RESULT_OK ? FOC_RESULT_SAFETY : eResult;
    }
    tElectrical = _motor_position_ToElectrical(
        ptPosition, tMechanical.tMechanicalAngle);
    ptFeedback->tElectricalAngle = foc_angle_add(
        tElectrical,
        (foc_angle_t){0U - ptPosition->tElectricalZero.wBam32});
    ptFeedback->qElectricalSpeedPu = foc_mul_wide(
        tMechanical.qMechanicalSpeed,
        ptPosition->qMechanicalToElectricalSpeedPuGain);
    ptFeedback->bValid = true;
    return FOC_RESULT_OK;
}

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
void motor_position_ObserverStep(
    motor_position_t *ptPosition,
    const motor_position_sample_t *ptSample)
{
    foc_observer_input_t tObserverInput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL || ptSample == NULL) {
        return;
    }
    tObserverInput.ptCurrentAlphaBeta = &ptSample->tCurrentAlphaBeta;
    tObserverInput.ptVoltageModelAlphaBeta =
        &ptSample->tVoltageModelAlphaBeta;
    eResult = foc_observer_Step(&ptPosition->tObserver,
                                 &tObserverInput);
    if (eResult != FOC_RESULT_OK) {
        ptPosition->tObserver.tOutput.bValid = false;
    }
}
#endif

foc_result_t motor_position_CaptureZero(motor_position_t *ptPosition,
                                        uint32_t wNowTick)
{
    foc_position_t tMechanical = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptPosition == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptPosition->eSource != MOTOR_POSITION_SOURCE_SENSOR) {
        return FOC_RESULT_DISABLED;
    }
    motor_position_InvalidateZero(ptPosition);
    eResult = FOC_SENSOR_POSITION_CAPTURE_ZERO(
        ptPosition, wNowTick, &tMechanical);
    if (eResult != FOC_RESULT_OK || !tMechanical.bValid) {
        return eResult == FOC_RESULT_OK ? FOC_RESULT_SAFETY : eResult;
    }
    ptPosition->tElectricalZero = _motor_position_ToElectrical(
        ptPosition, tMechanical.tMechanicalAngle);
    ptPosition->bElectricalZeroValid = true;
    return FOC_RESULT_OK;
}

void motor_position_InvalidateZero(motor_position_t *ptPosition)
{
    if (ptPosition != NULL) {
        ptPosition->bElectricalZeroValid = false;
        ptPosition->tElectricalZero = (foc_angle_t){0U};
    }
}

bool motor_position_ZeroValid(const motor_position_t *ptPosition)
{
    return ptPosition != NULL && ptPosition->bElectricalZeroValid;
}

void motor_position_ResetObserver(motor_position_t *ptPosition)
{
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    if (ptPosition != NULL) {
        foc_observer_Reset(&ptPosition->tObserver);
    }
#else
    (void)ptPosition;
#endif
}
