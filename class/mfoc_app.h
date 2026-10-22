/**
 * @file mfoc_app.h
 * @brief Stopped MFOC initialization and foreground feedback integration.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_APP_H
#define MFOC_APP_H
#include "modus.h"
#include "mfoc_motor.h"

typedef enum {
    MFOC_APP_UNINITIALIZED = 0,
    MFOC_APP_IDLE,
    MFOC_APP_ERROR,
} mfoc_app_state_t;

/** @brief Fixed motor, Core and encoder configuration. */
typedef struct {
    mfoc_motor_cfg_t motor;
} mfoc_app_cfg_t;

/** @brief Application-owned Motor and initialization progress. */
typedef struct {
    modus_base_t *ptBase;
    mfoc_motor_t motor;
    mfoc_app_state_t state;
    mfoc_result_t result;
} mfoc_app_t;

/** @brief Initialize the owned Motor, then register the stopped application.
 * @param object Application address. @param cfg Configuration address.
 * @return MODUS initialization status. */
int mfoc_app_Init(uintptr_t object, uintptr_t cfg);

/** @brief Satisfy the mandatory MODUS clock callback; no periodic work.
 * @param object Application address. @return Success for a nonnull object. */
int mfoc_app_Clock(uintptr_t object);

/** @brief Service foreground encoder sampling after successful Init.
 * @param object Application address. @return Progress or failure.
 * @note Does not start Motor, run the current loop or publish waveforms. */
int mfoc_app_Run(uintptr_t object);
#endif
