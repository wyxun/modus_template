/****************************************************************************
 * @file    foc_encoder_driver_test.c
 * @brief   Host contract test for the template-shaped Encoder Driver.
 * @author  Codex
 * @date    2026-09-16
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "foc_encoder.h"
#include "motor_position.h"

typedef struct {
    uint32_t wInitCount;
    uint32_t wReadCount;
    uint32_t wRawAngle;
    uint8_t chResolutionBits;
} test_sensor_t;

static int64_t s_lTick = 1000;
static uint32_t s_wFrequency = 1000000U;

int64_t get_system_ticks(void)
{
    return s_lTick;
}

uint32_t perfc_get_systimer_frequency(void)
{
    return s_wFrequency;
}

static foc_result_t test_SensorInit(void *pContext)
{
    test_sensor_t *ptSensor = (test_sensor_t *)pContext;

    ptSensor->wInitCount++;
    return FOC_RESULT_OK;
}

static foc_result_t test_SensorRead(void *pContext, foc_angle_t *ptAngle)
{
    test_sensor_t *ptSensor = (test_sensor_t *)pContext;

    ptSensor->wReadCount++;
    ptAngle->wBam32 = ptSensor->wRawAngle <<
                     (32U - ptSensor->chResolutionBits);
    return FOC_RESULT_OK;
}

static const foc_encoder_sensor_ops_t s_tSensorOps = {
    .fnInit = test_SensorInit,
    .fnRead = test_SensorRead,
};

/** @brief Check provider resolution, wrapping, inversion and speed precision. */
static void test_Resolution(uint8_t chBits, bool bInvert)
{
    const uint32_t wCounts = 1UL << chBits;
    test_sensor_t tSensor = {0U, 0U, wCounts - 1U, chBits};
    const foc_encoder_sensor_if_t tIf = {&s_tSensorOps, &tSensor};
    const foc_encoder_cfg_t tCfg = {
        .qSpeedFilterAlpha = FOC_ONE,
        .fInvalidTimeoutSeconds = 0.005f,
        .bDirectionInvert = bInvert,
        .ptSensor = &tIf,
    };
    foc_encoder_t tEncoder = {0};
    foc_position_t tPosition = {0};
    const float fSpeed = (bInvert ? -1.0f : 1.0f) *
                         2000.0f / (float)wCounts;

    assert(foc_encoder_Init(&tEncoder, &tCfg) == FOC_RESULT_OK);
    s_lTick = 1000;
    assert(foc_encoder_Run(&tEncoder) == FOC_RESULT_OK);
    tSensor.wRawAngle = 1U;
    s_lTick += 1000;
    assert(foc_encoder_Run(&tEncoder) == FOC_RESULT_OK);
    assert(foc_encoder_GetPosition(&tEncoder, (uint32_t)s_lTick,
                                   &tPosition) == FOC_RESULT_OK);
    assert(tPosition.tMechanicalAngle.wBam32 ==
        (bInvert ? 0U - (1UL << (32U - chBits)) :
                   (1UL << (32U - chBits))));
    assert(fabsf(foc_to_float(tPosition.qMechanicalSpeed) - fSpeed) <
           0.00004f);
    tSensor.wRawAngle = wCounts - 1U;
    s_lTick += 1000;
    assert(foc_encoder_Run(&tEncoder) == FOC_RESULT_OK);
    assert(foc_encoder_GetPosition(&tEncoder, (uint32_t)s_lTick,
                                   &tPosition) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(tPosition.qMechanicalSpeed) + fSpeed) <
           0.00004f);
}

/**
 * @brief Verify Encoder Driver ownership and typed position binding.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    test_sensor_t tSensor = {0U, 0U, 4090U, 12U};
    foc_encoder_cfg_t tConfig = {
        .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
        .fInvalidTimeoutSeconds = 0.005f,
        .bDirectionInvert = false,
        .ptSensor = &(const foc_encoder_sensor_if_t){
            .ptOps = &s_tSensorOps,
            .pContext = &tSensor,
        },
    };
    foc_encoder_t tEncoder = {0};
    foc_encoder_status_t tStatus = {0};
    foc_position_t tPosition = {0};
    motor_position_if_t tPositionProvider = {
        .fnGetPosition = foc_encoder_GetPosition,
        .pContext = &tEncoder,
    };

    s_wFrequency = 0U;
    assert(foc_encoder_Init(&tEncoder, &tConfig) ==
           FOC_RESULT_INVALID_ARGUMENT);
    assert(foc_encoder_GetPosition(&tEncoder, 1000U, &tPosition) ==
           FOC_RESULT_SAFETY);
    s_wFrequency = 1000000U;
    assert(foc_encoder_Init(&tEncoder, &tConfig) == FOC_RESULT_OK);
    assert(tSensor.wInitCount == 1U);
    assert(foc_encoder_GetStatus(&tEncoder, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == FOC_ENCODER_STATE_IDLE);

    assert(foc_encoder_GetPosition(&tEncoder, 1000U, &tPosition) ==
           FOC_RESULT_SAFETY);
    assert(tSensor.wReadCount == 0U);

    assert(foc_encoder_Run(&tEncoder) == FOC_RESULT_OK);
    assert(tSensor.wReadCount == 1U);
    tSensor.wRawAngle = 5U;
    s_lTick = 2000;
    assert(foc_encoder_Run(&tEncoder) == FOC_RESULT_OK);
    assert(tSensor.wReadCount == 2U);
    assert(foc_encoder_GetPosition(&tEncoder, 2000U, &tPosition) ==
           FOC_RESULT_OK);
    assert(tPosition.bValid);
    assert(tSensor.wReadCount == 2U);

    assert(tPositionProvider.fnGetPosition != NULL);
    assert(tPositionProvider.pContext == &tEncoder);

    foc_encoder_Stop(&tEncoder);
    assert(foc_encoder_GetStatus(&tEncoder, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == FOC_ENCODER_STATE_IDLE);
    assert(foc_encoder_Reset(&tEncoder) == FOC_RESULT_OK);
    assert(foc_encoder_GetStatus(&tEncoder, &tStatus) == FOC_RESULT_OK);
    assert(tStatus.eState == FOC_ENCODER_STATE_UNINITIALIZED);
    test_Resolution(12U, false);
    test_Resolution(14U, false);
    test_Resolution(20U, false);
    test_Resolution(20U, true);
    return 0;
}
