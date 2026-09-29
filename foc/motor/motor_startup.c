/****************************************************************************
 * @file    motor_startup.c
 * @brief   Bounded I/f frequency ramp, independent of position observers.
 ****************************************************************************/
#include "motor_startup.h"

#include <math.h>
#include <stddef.h>

/* Startup uses BAM32 angles with a Q16 fractional increment and Q31 speed. */
#define MOTOR_STARTUP_BAM32_SCALE          4294967296.0
#define MOTOR_STARTUP_Q16_SCALE            65536.0
#define MOTOR_STARTUP_Q31_SCALE            2147483648.0
#define MOTOR_STARTUP_Q48_SCALE            \
    (MOTOR_STARTUP_BAM32_SCALE * MOTOR_STARTUP_Q16_SCALE)
/* Keep the per-tick trajectory below half a turn and above one BAM count. */
#define MOTOR_STARTUP_MAX_STEP_Q16         \
    (MOTOR_STARTUP_Q48_SCALE / 2.0)
#define MOTOR_STARTUP_MIN_STEP_Q16         MOTOR_STARTUP_Q16_SCALE
#define MOTOR_STARTUP_Q31_TO_FLOAT_SCALE   4.656612873077392578125e-10f

foc_result_t motor_startup_Init(motor_startup_t *ptStartup,
                                const motor_startup_cfg_t *ptConfig)
{
    if (ptStartup == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptConfig->wControlFrequencyHz == 0U ||
        ptConfig->wRampSteps == 0U ||
        ptConfig->wRampSteps > 100000U ||
        !isfinite(ptConfig->fElectricalBaseHz) ||
        ptConfig->fElectricalBaseHz <= 0.0f ||
        ptConfig->fElectricalBaseHz >=
            (float)ptConfig->wControlFrequencyHz / 2.0f) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptStartup = (motor_startup_t){0};
    ptStartup->dStepQ16PerPu =
        (double)ptConfig->fElectricalBaseHz *
        MOTOR_STARTUP_Q48_SCALE /
        (double)ptConfig->wControlFrequencyHz;
    ptStartup->wRampSteps = ptConfig->wRampSteps;
    return FOC_RESULT_OK;
}

foc_result_t motor_startup_Start(motor_startup_t *ptStartup,
                                 foc_scalar_t qTargetSpeedPu)
{
    double dSpeedPu = 0.0;
    double dStepQ16 = 0.0;

    if (ptStartup == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptStartup->wRampSteps == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptStartup->bActive) {
        return FOC_RESULT_BUSY;
    }
    dSpeedPu = (double)foc_to_float(qTargetSpeedPu);
    if (!isfinite(dSpeedPu) || dSpeedPu == 0.0 ||
        fabs(dSpeedPu) >= 1.0) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    dStepQ16 = dSpeedPu * ptStartup->dStepQ16PerPu;
    if (fabs(dStepQ16) >= MOTOR_STARTUP_MAX_STEP_Q16 ||
        fabs(dStepQ16) < MOTOR_STARTUP_MIN_STEP_Q16) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptStartup->tAngle = (foc_angle_t){0U};
    ptStartup->lCurrentStepQ16 = 0;
    ptStartup->lTargetStepQ16 = (int64_t)llround(dStepQ16);
    ptStartup->lStepDeltaQ16 = ptStartup->lTargetStepQ16 /
                               (int64_t)ptStartup->wRampSteps;
    if (ptStartup->lStepDeltaQ16 == 0) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptStartup->lCurrentSpeedQ31 = 0;
    ptStartup->lTargetSpeedQ31 =
        (int64_t)llround(dSpeedPu * MOTOR_STARTUP_Q31_SCALE);
    ptStartup->lSpeedDeltaQ31 = ptStartup->lTargetSpeedQ31 /
                                (int64_t)ptStartup->wRampSteps;
    ptStartup->wRampCount = 0U;
    ptStartup->bActive = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Report whether startup has a validated ramp configuration.
 * @param ptStartup Startup object.
 * @return True when initialization enabled a ramp.
 */
bool motor_startup_IsConfigured(const motor_startup_t *ptStartup)
{
    return ptStartup != NULL && ptStartup->wRampSteps != 0U;
}

foc_result_t motor_startup_IsrStep(
    motor_startup_t *ptStartup,
    motor_electrical_feedback_t *ptForcedCandidate)
{
    if (ptStartup == NULL || ptForcedCandidate == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptForcedCandidate = (motor_electrical_feedback_t){0};
    if (!ptStartup->bActive) {
        return FOC_RESULT_DISABLED;
    }
    if (ptStartup->wRampCount < ptStartup->wRampSteps) {
        ptStartup->wRampCount++;
        ptStartup->lCurrentStepQ16 += ptStartup->lStepDeltaQ16;
        ptStartup->lCurrentSpeedQ31 += ptStartup->lSpeedDeltaQ31;
        if (ptStartup->wRampCount == ptStartup->wRampSteps) {
            ptStartup->lCurrentStepQ16 = ptStartup->lTargetStepQ16;
            ptStartup->lCurrentSpeedQ31 = ptStartup->lTargetSpeedQ31;
        }
    }
    ptForcedCandidate->tElectricalAngle = ptStartup->tAngle;
#if defined(FOC_NUMERIC_FIXED)
    ptForcedCandidate->qElectricalSpeedPu =
        (foc_scalar_t)(ptStartup->lCurrentSpeedQ31 /
                       (int64_t)MOTOR_STARTUP_Q16_SCALE);
#else
    ptForcedCandidate->qElectricalSpeedPu =
        (foc_scalar_t)((float)ptStartup->lCurrentSpeedQ31 *
                       MOTOR_STARTUP_Q31_TO_FLOAT_SCALE);
#endif
    ptStartup->tAngle.wBam32 +=
        (uint32_t)(ptStartup->lCurrentStepQ16 /
                   (int64_t)MOTOR_STARTUP_Q16_SCALE);
    ptForcedCandidate->bValid = true;
    return FOC_RESULT_OK;
}

void motor_startup_Stop(motor_startup_t *ptStartup)
{
    if (ptStartup != NULL) {
        ptStartup->bActive = false;
        ptStartup->wRampCount = 0U;
    }
}
