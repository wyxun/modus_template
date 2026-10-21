/**
 * @file app_init_test.c
 * @brief Application initialization order and failure-registration checks.
 * @author Codex
 * @date 2026-10-21
 */
#include "../../class/mfoc_app.c"
#include <assert.h>
#include <stdio.h>

static mfoc_result_t s_init_result;
static unsigned s_init_count;
static unsigned s_run_count;
static unsigned s_register_count;
static int s_register_result;

/** @brief Supply Driver initialization and record invocation.
 * @param driver Instance. @param cfg Configuration. @return Test status. */
mfoc_result_t mfoc_driver_Init(mfoc_driver_t *driver,
                               const mfoc_driver_cfg_t *cfg)
{
    assert(driver != NULL && cfg != NULL);
    ++s_init_count;
    return s_init_result;
}

/** @brief Supply foreground service without hardware.
 * @param driver Instance. @return OK. */
mfoc_result_t mfoc_driver_Run(mfoc_driver_t *driver)
{
    assert(driver != NULL);
    ++s_run_count;
    return MFOC_RESULT_OK;
}

/** @brief Verify registration follows successful Driver initialization.
 * @param base Base object. @param cfg Base configuration.
 * @return Selected framework status. */
int mbase_Init(modus_base_t *base, modus_base_cfg_t *cfg)
{
    assert(base != NULL && cfg != NULL);
    assert(s_init_count != 0U && s_init_result == MFOC_RESULT_OK);
    assert(cfg->FcnInterface.Clock != NULL);
    assert(cfg->FcnInterface.Run != NULL);
    assert(cfg->FcnInterface.Clock(cfg->wParent) == MODUS_SUCCESS);
    ++s_register_count;
    return s_register_result;
}

/** @brief Check success, failure and Run without deferred initialization.
 * @return Zero on success. */
int main(void)
{
    mfoc_app_t app = {0};
    mfoc_app_cfg_t cfg = {0};
    assert(mfoc_app_Init(0U, (uintptr_t)&cfg) == MODUS_EFAIL);
    assert(mfoc_app_Run((uintptr_t)&app) == MODUS_EFAIL);
    s_init_result = MFOC_RESULT_IO;
    assert(mfoc_app_Init((uintptr_t)&app, (uintptr_t)&cfg) == MODUS_EFAIL);
    assert(app.state == MFOC_APP_ERROR && s_register_count == 0U);
    s_init_result = MFOC_RESULT_OK;
    s_register_result = MODUS_SUCCESS;
    assert(mfoc_app_Init((uintptr_t)&app, (uintptr_t)&cfg) == MODUS_SUCCESS);
    assert(app.state == MFOC_APP_IDLE && s_register_count == 1U);
    unsigned count = s_init_count;
    assert(mfoc_app_Run((uintptr_t)&app) == MODUS_SUCCESS);
    assert(s_run_count == 1U && s_init_count == count);
    s_register_result = MODUS_EFAIL;
    assert(mfoc_app_Init((uintptr_t)&app, (uintptr_t)&cfg) == MODUS_EFAIL);
    assert(app.state == MFOC_APP_ERROR);
    puts("Application Init order, Clock binding and failures passed");
    return 0;
}
