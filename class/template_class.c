/**
 * @file template_class.c
 * @brief MODUS parent Class and Class-owned Driver assembly template.
 * @author Modus project
 * @date 2026-09-16
 */

/*
 * Static objects in this file are intentionally limited to framework data or
 * immutable/default resources:
 *
 * - s_tTemplateClassBase and s_tTemplateClassBaseCfg are MODUS bookkeeping.
 * - s_tTemplateDriverDefaultOps is immutable default dependency behavior.
 * - s_chTemplateRxBuffer belongs to the generated single template object.
 * - s_wMockSystemTick is a replaceable demonstration soft-clock source.
 *
 * None of these objects stores Class business state or Driver process state.
 * A concrete multi-instance product should inject its ring buffer and tick,
 * and keep all mutable per-instance data in template_class_t.
 */

#include "template_class.h"

#include "perfc_task_pt.h"

#include <stddef.h>
#include <string.h>

static modus_base_t s_tTemplateClassBase;
static modus_base_cfg_t s_tTemplateClassBaseCfg = {
    .wId = TEMPLATE_CLASS,
    .wParent = 0U,
    .FcnInterface = {
        .Clock = template_class_Clock,
        .Run = template_class_Run,
    },
};

/**
 * @brief Provide a safe no-op dependency for the generated template object.
 * @param pContext Unused default dependency context.
 * @return Successful initialization result.
 */
static template_driver_result_t _template_class_DefaultInit(void *pContext)
{
    (void)pContext;
    return TEMPLATE_DRIVER_RESULT_OK;
}

/**
 * @brief Provide a safe sample for the generated template object.
 * @param pContext Unused default dependency context.
 * @param pwValue Destination sample value.
 * @return Successful sample result.
 */
static template_driver_result_t _template_class_DefaultSample(
    void *pContext,
    uint32_t *pwValue)
{
    (void)pContext;
    *pwValue = 0U;
    return TEMPLATE_DRIVER_RESULT_OK;
}

/**
 * @brief Accept a safe output for the generated template object.
 * @param pContext Unused default dependency context.
 * @param wValue Output value accepted by the no-op dependency.
 * @return Successful output result.
 */
static template_driver_result_t _template_class_DefaultApply(
    void *pContext,
    uint32_t wValue)
{
    (void)pContext;
    (void)wValue;
    return TEMPLATE_DRIVER_RESULT_OK;
}

/**
 * @brief Stop the safe no-op dependency.
 * @param pContext Unused default dependency context.
 * @return Successful stop result.
 */
static template_driver_result_t _template_class_DefaultStop(void *pContext)
{
    (void)pContext;
    return TEMPLATE_DRIVER_RESULT_OK;
}

static const template_driver_hw_ops_t s_tTemplateDriverDefaultOps = {
    .fnInit = _template_class_DefaultInit,
    .fnSample = _template_class_DefaultSample,
    .fnApply = _template_class_DefaultApply,
    .fnStop = _template_class_DefaultStop,
    .fnClock = NULL,
    .fnIsrStep = NULL,
};

/**
 * @brief Latch a child Driver failure in the parent Class.
 * @param ptThis Parent Class object.
 * @param eError Detailed child Driver error.
 * @return None.
 */
static void _template_class_SetError(template_class_t *ptThis,
                                     template_driver_result_t eError)
{
    ptThis->eDriverError = eError;
    ptThis->eState = TEMPLATE_CLASS_STATE_ERROR;
}

/**
 * @brief Initialize the Driver owned by one parent Class.
 * @param ptThis Parent Class object.
 * @param ptCfg Parent Class configuration.
 * @return MODUS_SUCCESS on success, otherwise MODUS_EFAIL.
 */
static int _template_class_InitDriver(template_class_t *ptThis,
                                      const template_class_cfg_t *ptCfg)
{
    template_driver_result_t eResult = TEMPLATE_DRIVER_RESULT_OK;

    eResult = template_driver_Init(&ptThis->tDriver, &ptCfg->tDriverCfg);
    if (eResult != TEMPLATE_DRIVER_RESULT_OK) {
        _template_class_SetError(ptThis, eResult);
        template_driver_Stop(&ptThis->tDriver);
        return MODUS_EFAIL;
    }

    return MODUS_SUCCESS;
}

int template_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    template_class_t *ptThis = (template_class_t *)wObjectAddr;
    const template_class_cfg_t *ptCfg =
        (const template_class_cfg_t *)wObjectCfgAddr;
    int nBaseResult = MODUS_EFAIL;

    if ((ptThis == NULL) || (ptCfg == NULL)) {
        return MODUS_EFAIL;
    }

    (void)memset(ptThis, 0, sizeof(*ptThis));
    ptThis->eState = TEMPLATE_CLASS_STATE_UNINITIALIZED;
    ptThis->eDriverError = TEMPLATE_DRIVER_RESULT_OK;

    if (ptCfg->pwSharedSystemTick == NULL) {
        _template_class_SetError(ptThis,
                                TEMPLATE_DRIVER_RESULT_INVALID_ARGUMENT);
        return MODUS_EFAIL;
    }

    ptThis->ptBase = &s_tTemplateClassBase;
    ptThis->pwSharedSystemTick = ptCfg->pwSharedSystemTick;
    s_tTemplateClassBaseCfg.wParent = wObjectAddr;
    s_tTemplateClassBaseCfg.pchRingBuffer = ptCfg->pchRingBuffer;
    s_tTemplateClassBaseCfg.hwRingSize = ptCfg->hwRingSize;

    if (_template_class_InitDriver(ptThis, ptCfg) != MODUS_SUCCESS) {
        return MODUS_EFAIL;
    }

    nBaseResult = mbase_Init(ptThis->ptBase, &s_tTemplateClassBaseCfg);
    if (nBaseResult != MODUS_SUCCESS) {
        template_driver_Stop(&ptThis->tDriver);
        _template_class_SetError(ptThis, TEMPLATE_DRIVER_RESULT_FAULT);
        return nBaseResult;
    }

    ptThis->eState = TEMPLATE_CLASS_STATE_IDLE;
    return MODUS_SUCCESS;
}

int template_class_Run(uintptr_t wObjectAddr)
{
    template_class_t *ptThis = (template_class_t *)wObjectAddr;
    template_driver_result_t eDriverError = TEMPLATE_DRIVER_RESULT_OK;
    template_driver_status_t tDriverStatus = {0};
    fsm_rt_t eDriverRun = fsm_rt_err;
    uint32_t wNowTick = 0U;

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }

    if (ptThis->eState == TEMPLATE_CLASS_STATE_ERROR) {
        return MODUS_EFAIL;
    }

    if (ptThis->pwSharedSystemTick == NULL) {
        _template_class_SetError(ptThis,
                                TEMPLATE_DRIVER_RESULT_INVALID_ARGUMENT);
        return MODUS_EFAIL;
    }

    wNowTick = (uint32_t)(*ptThis->pwSharedSystemTick);
    PERFC_PT_BEGIN(ptThis->chRunPt)

    while (1) {
        eDriverRun = template_driver_Run(&ptThis->tDriver, wNowTick);
        if (eDriverRun == fsm_rt_err) {
            eDriverError = template_driver_GetStatus(&ptThis->tDriver,
                                                      &tDriverStatus);
            if (eDriverError == TEMPLATE_DRIVER_RESULT_OK) {
                eDriverError = tDriverStatus.eLastError;
            } else {
                eDriverError = TEMPLATE_DRIVER_RESULT_FAULT;
            }
            _template_class_SetError(ptThis, eDriverError);
            return MODUS_EFAIL;
        }

        ptThis->eState = TEMPLATE_CLASS_STATE_RUNNING;
        PERFC_PT_YIELD((int)eDriverRun)
        wNowTick = (uint32_t)(*ptThis->pwSharedSystemTick);
    }

    PERFC_PT_END()

    return MODUS_SUCCESS;
}

/**
 * @brief Execute the optional short Class clock callback.
 * @param wObjectAddr Address of the Class object.
 * @return MODUS_SUCCESS on a valid object, otherwise MODUS_EFAIL.
 */
int template_class_Clock(uintptr_t wObjectAddr)
{
    template_class_t *ptThis = (template_class_t *)wObjectAddr;
    template_driver_status_t tDriverStatus = {0};

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }

    template_driver_Clock(&ptThis->tDriver);
    if (template_driver_GetStatus(&ptThis->tDriver, &tDriverStatus)
        != TEMPLATE_DRIVER_RESULT_OK) {
        _template_class_SetError(ptThis, TEMPLATE_DRIVER_RESULT_FAULT);
        return MODUS_EFAIL;
    }
    if (tDriverStatus.eState == TEMPLATE_DRIVER_STATE_ERROR) {
        _template_class_SetError(ptThis, tDriverStatus.eLastError);
        return MODUS_EFAIL;
    }

    return MODUS_SUCCESS;
}

static uint8_t s_chTemplateRxBuffer[128];
static volatile uint32_t s_wMockSystemTick = 0U;

MODUS_DECLARE_OBJECT(template_class, TemplateClass,
    .pchRingBuffer = s_chTemplateRxBuffer,
    .hwRingSize = sizeof(s_chTemplateRxBuffer),
    .pwSharedSystemTick = &s_wMockSystemTick,
    .tDriverCfg = {
        .tHw = {
            .ptOps = &s_tTemplateDriverDefaultOps,
            .pContext = &s_tTemplateClassBaseCfg,
        },
        .wInitialValue = 0U,
        .wTickRateHz = 1000U,
        .wUpdateRateHz = 1000U,
    }
)
