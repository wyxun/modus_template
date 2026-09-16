/**
 * @file template_driver.c
 * @brief Class-owned non-blocking Driver template implementation.
 * @author Modus project
 * @date 2026-09-16
 */

/*
 * This implementation deliberately has no static runtime object. Each
 * mutable value belongs to the caller-owned template_driver_t, so two Driver
 * instances can be interleaved and tested without shared hidden state.
 *
 * A static helper function is different from a static runtime variable:
 * `_template_driver_*` is file-private by convention and by static linkage;
 * the PT cursor and all timer data remain in the Driver object.
 *
 * The optional Clock and hardware IsrStep entries are object services. Their
 * callbacks may be absent, but they must never create hidden static state.
 *
 * This file demonstrates the foreground/PT capability profile. A concrete
 * fixed-rate algorithm should remove the PT service section and implement a
 * typed IsrStep directly, for example through the template_observer.h
 * interface example.
 */

#include "template_driver.h"

#include "perfc_task_pt.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Validate the PT example dependency and timing contract.
 * @param ptCfg Candidate initialization configuration.
 * @return True when all required fields are usable.
 */
static bool _template_driver_IsConfigValid(
    const template_driver_cfg_t *ptCfg)
{
    bool bValid = false;

    if ((ptCfg != NULL) && (ptCfg->tHw.ptOps != NULL)
        && (ptCfg->tHw.pContext != NULL)
        && (ptCfg->tHw.ptOps->fnInit != NULL)
        && (ptCfg->tHw.ptOps->fnSample != NULL)
        && (ptCfg->tHw.ptOps->fnApply != NULL)
        && (ptCfg->tHw.ptOps->fnStop != NULL)
        && (ptCfg->wPeriodTicks != 0U)) {
        bValid = true;
    }

    return bValid;
}

/**
 * @brief Latch a detailed failure and map it to a fault category.
 * @param ptThis Driver object receiving the failure.
 * @param eError Detailed failure cause.
 * @return None.
 */
static void _template_driver_LatchError(template_driver_t *ptThis,
                                        template_driver_result_t eError)
{
    ptThis->eLastError = eError;
    ptThis->eState = TEMPLATE_DRIVER_STATE_ERROR;

    switch (eError) {
    case TEMPLATE_DRIVER_RESULT_INVALID_ARGUMENT:
        ptThis->wFaults |= TEMPLATE_DRIVER_FAULT_ARGUMENT;
        break;
    case TEMPLATE_DRIVER_RESULT_TIMEOUT:
        ptThis->wFaults |= TEMPLATE_DRIVER_FAULT_TIMEOUT;
        break;
    case TEMPLATE_DRIVER_RESULT_IO:
        ptThis->wFaults |= TEMPLATE_DRIVER_FAULT_IO;
        break;
    default:
        ptThis->wFaults |= TEMPLATE_DRIVER_FAULT_STATE;
        break;
    }
}

/**
 * @brief Check a wrap-safe explicit-tick deadline.
 * @param ptThis Driver object containing the timer fields.
 * @param wNowTick Current foreground soft-clock tick.
 * @return True when the configured period has elapsed.
 */
static bool _template_driver_IsExpired(const template_driver_t *ptThis,
                                       uint32_t wNowTick)
{
    uint32_t wElapsed = wNowTick - ptThis->wStartTick;

    return wElapsed >= ptThis->wPeriodTicks;
}

/**
 * @brief Initialize one caller-owned Driver object.
 * @param ptThis Caller-owned Driver object.
 * @param ptCfg Initialization configuration and dependency bindings.
 * @return Detailed initialization result.
 */
template_driver_result_t template_driver_Init(
    template_driver_t *ptThis,
    const template_driver_cfg_t *ptCfg)
{
    template_driver_result_t eResult = TEMPLATE_DRIVER_RESULT_OK;

    if (ptThis == NULL) {
        return TEMPLATE_DRIVER_RESULT_NULL;
    }

    (void)memset(ptThis, 0, sizeof(*ptThis));
    ptThis->eState = TEMPLATE_DRIVER_STATE_UNINITIALIZED;
    ptThis->eLastError = TEMPLATE_DRIVER_RESULT_OK;

    if (ptCfg == NULL) {
        _template_driver_LatchError(ptThis, TEMPLATE_DRIVER_RESULT_NULL);
        return TEMPLATE_DRIVER_RESULT_NULL;
    }

    if (!_template_driver_IsConfigValid(ptCfg)) {
        _template_driver_LatchError(ptThis,
                                   TEMPLATE_DRIVER_RESULT_INVALID_ARGUMENT);
        return TEMPLATE_DRIVER_RESULT_INVALID_ARGUMENT;
    }

    ptThis->tHw = ptCfg->tHw;
    ptThis->wPeriodTicks = ptCfg->wPeriodTicks;
    ptThis->wValue = ptCfg->wInitialValue;
    eResult = ptThis->tHw.ptOps->fnInit(ptThis->tHw.pContext);
    if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
        _template_driver_LatchError(ptThis, eResult);
        return eResult;
    }

    ptThis->eState = TEMPLATE_DRIVER_STATE_IDLE;
    return TEMPLATE_DRIVER_RESULT_OK;
}

/**
 * @brief Advance one non-blocking Driver PT step.
 * @param ptThis Caller-owned Driver object.
 * @param wNowTick Explicit soft-clock tick.
 * @return PT progress result.
 */
fsm_rt_t template_driver_Run(template_driver_t *ptThis, uint32_t wNowTick)
{
    template_driver_result_t eResult = TEMPLATE_DRIVER_RESULT_OK;
    uint32_t wSample = 0U;

    if (ptThis == NULL) {
        return fsm_rt_err;
    }

    if (ptThis->eState == TEMPLATE_DRIVER_STATE_ERROR) {
        return fsm_rt_err;
    }

    if (ptThis->eState == TEMPLATE_DRIVER_STATE_UNINITIALIZED) {
        _template_driver_LatchError(ptThis,
                                   TEMPLATE_DRIVER_RESULT_NOT_READY);
        return fsm_rt_err;
    }

    PERFC_PT_BEGIN(ptThis->chRunPt)

    ptThis->eState = TEMPLATE_DRIVER_STATE_RUNNING;
    while (1) {
        ptThis->wStartTick = wNowTick;
        ptThis->bTimerActive = true;
        PERFC_PT_WAIT_UNTIL(_template_driver_IsExpired(ptThis, wNowTick))

        eResult = ptThis->tHw.ptOps->fnSample(ptThis->tHw.pContext,
                                              &wSample);
        if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
            _template_driver_LatchError(ptThis, eResult);
            return fsm_rt_err;
        }

        eResult = ptThis->tHw.ptOps->fnApply(ptThis->tHw.pContext, wSample);
        if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
            _template_driver_LatchError(ptThis, eResult);
            return fsm_rt_err;
        }

        ptThis->wValue = wSample;
        ptThis->bTimerActive = false;
        PERFC_PT_YIELD(fsm_rt_cpl)
    }

    PERFC_PT_END()

    return fsm_rt_on_going;
}

/**
 * @brief Start the periodic Driver flow.
 * @param ptThis Caller-owned Driver object.
 * @return Detailed state-transition result.
 */
template_driver_result_t template_driver_Start(template_driver_t *ptThis)
{
    if (ptThis == NULL) {
        return TEMPLATE_DRIVER_RESULT_NULL;
    }

    if (ptThis->eState == TEMPLATE_DRIVER_STATE_ERROR) {
        return TEMPLATE_DRIVER_RESULT_FAULT;
    }

    if (ptThis->eState != TEMPLATE_DRIVER_STATE_IDLE) {
        return TEMPLATE_DRIVER_RESULT_BUSY;
    }

    ptThis->bTimerActive = false;
    ptThis->eState = TEMPLATE_DRIVER_STATE_RUNNING;
    return TEMPLATE_DRIVER_RESULT_OK;
}

/**
 * @brief Stop the Driver and request safe external output.
 * @param ptThis Caller-owned Driver object.
 * @return None; failures are latched in the Driver status.
 */
void template_driver_Stop(template_driver_t *ptThis)
{
    template_driver_result_t eResult = TEMPLATE_DRIVER_RESULT_OK;

    if ((ptThis == NULL) || (ptThis->tHw.ptOps == NULL)
        || (ptThis->tHw.ptOps->fnStop == NULL)) {
        return;
    }

    eResult = ptThis->tHw.ptOps->fnStop(ptThis->tHw.pContext);
    ptThis->bTimerActive = false;
    if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
        _template_driver_LatchError(ptThis, eResult);
    } else if (ptThis->eState != TEMPLATE_DRIVER_STATE_ERROR) {
        ptThis->eState = TEMPLATE_DRIVER_STATE_IDLE;
    } else {
        /* A latched fault remains visible until Reset(). */
    }
}

/**
 * @brief Execute one optional fixed-rate hard-clock service.
 * @param ptThis Caller-owned Driver object.
 * @return None; failures are latched in the Driver status.
 */
void template_driver_Clock(template_driver_t *ptThis)
{
    template_driver_result_t eResult = TEMPLATE_DRIVER_RESULT_OK;

    if ((ptThis == NULL) || (ptThis->tHw.ptOps == NULL)
        || (ptThis->tHw.ptOps->fnClock == NULL)) {
        return;
    }

    eResult = ptThis->tHw.ptOps->fnClock(ptThis->tHw.pContext);
    if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
        _template_driver_LatchError(ptThis, eResult);
    }
}

/**
 * @brief Execute one optional bounded ISR service.
 * @param ptThis Caller-owned Driver object.
 * @return None; failures are latched in the Driver status.
 */
void template_driver_IsrStep(template_driver_t *ptThis)
{
    template_driver_result_t eResult = TEMPLATE_DRIVER_RESULT_OK;

    if ((ptThis == NULL) || (ptThis->tHw.ptOps == NULL)
        || (ptThis->tHw.ptOps->fnIsrStep == NULL)) {
        return;
    }

    eResult = ptThis->tHw.ptOps->fnIsrStep(ptThis->tHw.pContext);
    if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
        _template_driver_LatchError(ptThis, eResult);
    }
}

/**
 * @brief Clear latched Driver faults after a safe stop.
 * @param ptThis Caller-owned Driver object.
 * @return Detailed reset result.
 */
template_driver_result_t template_driver_Reset(template_driver_t *ptThis)
{
    template_driver_result_t eResult = TEMPLATE_DRIVER_RESULT_OK;

    if (ptThis == NULL) {
        return TEMPLATE_DRIVER_RESULT_NULL;
    }

    if ((ptThis->tHw.ptOps == NULL) || (ptThis->tHw.pContext == NULL)
        || (ptThis->tHw.ptOps->fnStop == NULL)) {
        _template_driver_LatchError(ptThis,
                                   TEMPLATE_DRIVER_RESULT_NOT_READY);
        return TEMPLATE_DRIVER_RESULT_NOT_READY;
    }

    eResult = ptThis->tHw.ptOps->fnStop(ptThis->tHw.pContext);
    if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
        _template_driver_LatchError(ptThis, eResult);
        return eResult;
    }

    ptThis->eState = TEMPLATE_DRIVER_STATE_IDLE;
    ptThis->eLastError = TEMPLATE_DRIVER_RESULT_OK;
    ptThis->wFaults = 0U;
    ptThis->chRunPt = 0U;
    ptThis->bTimerActive = false;
    return TEMPLATE_DRIVER_RESULT_OK;
}

/**
 * @brief Copy a read-only status snapshot.
 * @param ptThis Caller-owned Driver object.
 * @param ptStatus Destination status snapshot.
 * @return Detailed query result.
 */
template_driver_result_t template_driver_GetStatus(
    const template_driver_t *ptThis,
    template_driver_status_t *ptStatus)
{
    if ((ptThis == NULL) || (ptStatus == NULL)) {
        return TEMPLATE_DRIVER_RESULT_NULL;
    }

    ptStatus->eState = ptThis->eState;
    ptStatus->eLastError = ptThis->eLastError;
    ptStatus->wFaults = ptThis->wFaults;
    ptStatus->wValue = ptThis->wValue;
    ptStatus->bTimerActive = ptThis->bTimerActive;
    return TEMPLATE_DRIVER_RESULT_OK;
}
