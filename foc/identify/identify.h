#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include <stdint.h>

#include "foc_numeric.h"

/**
 * @brief Runtime state of one parameter-identification instance.
 */
typedef enum {
    IDENTIFY_STATE_UNINITIALIZED = 0,
    IDENTIFY_STATE_IDLE,
    IDENTIFY_STATE_RUNNING,
    IDENTIFY_STATE_ERROR,
} identify_state_t;

/**
 * @brief Initialization-only configuration for parameter identification.
 */
typedef struct {
    uint32_t wSamplePeriodTicks;
} identify_cfg_t;

/**
 * @brief Read-only status snapshot of an identification instance.
 */
typedef struct {
    identify_state_t eState;
    foc_result_t eLastResult;
    uint32_t wSampleCount;
} identify_status_t;

/**
 * @brief Runtime object owned by the caller.
 */
typedef struct {
    identify_cfg_t tConfig;
    identify_state_t eState;
    foc_result_t eLastResult;
    uint32_t wSampleCount;
} identify_t;

/**
 * @brief Initialize one parameter-identification object.
 * @param ptThis Caller-owned identification object.
 * @param ptConfig Initialization configuration.
 * @return FOC_RESULT_OK or an argument/configuration error.
 */
foc_result_t identify_Init(identify_t *ptThis,
                           const identify_cfg_t *ptConfig);

/**
 * @brief Start the empty identification flow.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t identify_Start(identify_t *ptThis);

/**
 * @brief Advance one non-blocking identification step.
 * @param ptThis Identification object.
 * @param wNowTick Current foreground soft-clock tick.
 * @return FOC_RESULT_DISABLED until the algorithm is implemented.
 */
foc_result_t identify_Run(identify_t *ptThis, uint32_t wNowTick);

/**
 * @brief Stop the identification flow without clearing a latched error.
 * @param ptThis Identification object.
 * @return None.
 */
void identify_Stop(identify_t *ptThis);

/**
 * @brief Reset runtime state after a stop or an error.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Reset(identify_t *ptThis);

/**
 * @brief Copy a read-only status snapshot.
 * @param ptThis Identification object.
 * @param ptStatus Destination status snapshot.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_GetStatus(const identify_t *ptThis,
                                identify_status_t *ptStatus);

#endif /* FOC_IDENTIFY_H */
