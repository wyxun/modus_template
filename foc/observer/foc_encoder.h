/****************************************************************************
 * @file    foc_encoder.h
 * @brief   Mechanical angle and speed backend for an absolute sensor.
 * @note    A standalone build may use the legacy sensor binding below. A
 *          target defining FOC_ENCODER_STATIC_BINDING supplies
 *          FOC_ENCODER_PORT_INIT/READ and removes that runtime sensor object.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_position.h"

#if !defined(FOC_ENCODER_STATIC_BINDING)
typedef struct {
    foc_result_t (*fnInit)(void *pContext);
    foc_result_t (*fnRead)(void *pContext, uint16_t *phwRawAngle);
} foc_encoder_sensor_ops_t;

typedef struct {
    const foc_encoder_sensor_ops_t *ptOps;
    void *pContext;
} foc_encoder_sensor_if_t;
#endif

typedef struct {
    foc_scalar_t qSpeedFilterAlpha;
    float fInvalidTimeoutSeconds;
    bool bDirectionInvert;
#if !defined(FOC_ENCODER_STATIC_BINDING)
    const foc_encoder_sensor_if_t *ptSensor;
#endif
} foc_encoder_cfg_t;

typedef enum {
    FOC_ENCODER_STATE_UNINITIALIZED = 0,
    FOC_ENCODER_STATE_IDLE,
    FOC_ENCODER_STATE_ERROR,
} foc_encoder_state_e;

typedef struct {
    foc_position_t tPosition;
    uint32_t wSampleTick;
} foc_encoder_position_slot_t;

typedef struct {
#if !defined(FOC_ENCODER_STATIC_BINDING)
    foc_encoder_sensor_if_t tSensor;
#endif
    foc_scalar_t qSpeedFilterAlpha;
    bool bDirectionInvert;
    foc_encoder_position_slot_t atPosition[2];
    uint32_t wTickFrequency;
    uint32_t wInvalidTimeoutTicks;
    uint32_t wLastSampleTick;
    uint16_t hwLastRawAngle;
    /* Publication metadata is atomic on the target and orders slot access. */
    volatile uint8_t chPublishedIndex;
    volatile bool bHasSample;
    foc_encoder_state_e eState;
    foc_result_t eLastError;
    uint32_t wFaults;
} foc_encoder_t;

typedef struct {
    foc_encoder_state_e eState;
    foc_result_t eLastError;
    uint32_t wFaults;
    bool bHasSample;
} foc_encoder_status_t;

/**
 * @brief Initialize the encoder and its raw sensor provider.
 * @param ptEncoder Encoder object.
 * @param ptConfig Mechanical filter configuration and, for a dynamic build,
 *                 the sensor binding.
 * @return FOC_RESULT_OK, DISABLED, or an initialization error.
 */
foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_cfg_t *ptConfig);

/**
 * @brief Read one raw sensor sample and publish mechanical feedback.
 * @param ptEncoder Encoder object.
 * @return FOC_RESULT_OK or the sensor read error.
 */
foc_result_t foc_encoder_Run(foc_encoder_t *ptEncoder);

/**
 * @brief Stop foreground Encoder service and retain the latest cache.
 * @param ptEncoder Encoder object.
 * @return None.
 */
void foc_encoder_Stop(foc_encoder_t *ptEncoder);

/**
 * @brief Reset one Encoder Driver to its uninitialized state.
 * @param ptEncoder Encoder object.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_encoder_Reset(foc_encoder_t *ptEncoder);

/**
 * @brief Copy the Encoder Driver status.
 * @param ptEncoder Encoder object.
 * @param ptStatus Output status snapshot.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_encoder_GetStatus(const foc_encoder_t *ptEncoder,
                                   foc_encoder_status_t *ptStatus);

/**
 * @brief Read a consistent, age-checked mechanical position snapshot.
 * @param ptEncoder Encoder object.
 * @param wNowTick Low 32 bits of the current system tick.
 * @param ptPosition Output position snapshot.
 * @return FOC_RESULT_OK or a safety/argument error.
 */
foc_result_t foc_encoder_GetPosition(const void *pEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition);

/**
 * @brief Capture the current mechanical position for Motor alignment.
 * @param ptEncoder Encoder object.
 * @param wNowTick Low 32 bits of the current system tick.
 * @param ptPosition Output position snapshot.
 * @return FOC_RESULT_OK or a safety/argument error.
 */
foc_result_t foc_encoder_CaptureZero(const void *pEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition);

#endif /* FOC_ENCODER_H */
