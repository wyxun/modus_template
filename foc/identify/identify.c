/**
 * @file identify.c
 * @brief Empty parameter-identification object implementation.
 * @author Modus project
 * @date 2026-09-19
 */

#include "identify.h"

#include <stddef.h>

/**
 * @brief Initialize one parameter-identification object.
 * @param ptThis Caller-owned identification object.
 * @param ptConfig Initialization configuration.
 * @return FOC_RESULT_OK or an argument/configuration error.
 */
foc_result_t identify_Init(identify_t *ptThis,
                           const identify_cfg_t *ptConfig)
{
    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }

    *ptThis = (identify_t){0};
    ptThis->eState = IDENTIFY_STATE_UNINITIALIZED;

    if (ptConfig == NULL) {
        ptThis->eState = IDENTIFY_STATE_ERROR;
        ptThis->eLastResult = FOC_RESULT_NULL;
        return FOC_RESULT_NULL;
    }
    if (ptConfig->wSamplePeriodTicks == 0U) {
        ptThis->eState = IDENTIFY_STATE_ERROR;
        ptThis->eLastResult = FOC_RESULT_INVALID_ARGUMENT;
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    ptThis->tConfig = *ptConfig;
    ptThis->eState = IDENTIFY_STATE_IDLE;
    ptThis->eLastResult = FOC_RESULT_OK;
    return FOC_RESULT_OK;
}

/**
 * @brief Start the empty identification flow.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t identify_Start(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptThis->eState == IDENTIFY_STATE_ERROR) {
        return FOC_RESULT_SAFETY;
    }
    if (ptThis->eState != IDENTIFY_STATE_IDLE) {
        return FOC_RESULT_BUSY;
    }

    ptThis->eState = IDENTIFY_STATE_RUNNING;
    ptThis->eLastResult = FOC_RESULT_OK;
    return FOC_RESULT_OK;
}

/**
 * @brief Advance one non-blocking identification step.
 * @param ptThis Identification object.
 * @param wNowTick Current foreground soft-clock tick.
 * @return FOC_RESULT_DISABLED until the algorithm is implemented.
 */
foc_result_t identify_Run(identify_t *ptThis, uint32_t wNowTick)
{
    (void)wNowTick;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptThis->eState != IDENTIFY_STATE_RUNNING) {
        return FOC_RESULT_BUSY;
    }

    /* The first sampling and estimation step will be added incrementally. */
    ptThis->eLastResult = FOC_RESULT_DISABLED;
    return FOC_RESULT_DISABLED;
}

/**
 * @brief Stop the identification flow without clearing a latched error.
 * @param ptThis Identification object.
 * @return None.
 */
void identify_Stop(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return;
    }
    if (ptThis->eState != IDENTIFY_STATE_UNINITIALIZED &&
        ptThis->eState != IDENTIFY_STATE_ERROR) {
        ptThis->eState = IDENTIFY_STATE_IDLE;
    }
}

/**
 * @brief Reset runtime state after a stop or an error.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Reset(identify_t *ptThis)
{
    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->eState == IDENTIFY_STATE_UNINITIALIZED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    ptThis->eState = IDENTIFY_STATE_IDLE;
    ptThis->eLastResult = FOC_RESULT_OK;
    ptThis->wSampleCount = 0U;
    return FOC_RESULT_OK;
}

/**
 * @brief Copy a read-only status snapshot.
 * @param ptThis Identification object.
 * @param ptStatus Destination status snapshot.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_GetStatus(const identify_t *ptThis,
                                identify_status_t *ptStatus)
{
    if (ptThis == NULL || ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }

    ptStatus->eState = ptThis->eState;
    ptStatus->eLastResult = ptThis->eLastResult;
    ptStatus->wSampleCount = ptThis->wSampleCount;
    return FOC_RESULT_OK;
}
