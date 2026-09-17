/****************************************************************************
 * @file    foc_identify_plant.h
 * @brief   Independent double-precision RL motor plant simulation for tests.
 * @author  Antigravity
 * @date    2026-09-17
 ****************************************************************************/

#ifndef FOC_IDENTIFY_PLANT_H
#define FOC_IDENTIFY_PLANT_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double dResistance;            /**< Phase resistance (Ohm) */
    double dInductanceD;           /**< D-axis inductance (Henry) */
    double dInductanceQ;           /**< Q-axis inductance (Henry) */
    double dDt;                    /**< Sampling interval Ts (seconds) */
    double dVbase;                 /**< Base voltage (Volts) */
    double dIbase;                 /**< Base current (Amperes) */
    double dVoltageOffsetD;        /**< Inverter voltage offset D (Volts) */
    double dVoltageOffsetQ;        /**< Inverter voltage offset Q (Volts) */
    double dAdcCurrentOffsetD;     /**< ADC current sensor offset D (Amperes) */
    double dAdcCurrentOffsetQ;     /**< ADC current sensor offset Q (Amperes) */
    double dAdcCurrentNoiseD;      /**< Peak deterministic noise D (Amperes) */
    double dAdcCurrentNoiseQ;      /**< Peak deterministic noise Q (Amperes) */
    double dCurrentSatMax;         /**< Current saturation upper bound (Amperes) */
    uint32_t wSubmitDelaySamples;  /**< Integer cycle delay from submit to apply */
    double dSubmitFractionalDelay; /**< Fractional cycle delay [0.0, 1.0) */
} foc_identify_plant_cfg_t;

#define FOC_IDENTIFY_PLANT_MAX_DELAY 8U

typedef struct {
    foc_identify_plant_cfg_t tCfg;
    double dCurrentD;              /**< Physical state: true D current (Amperes) */
    double dCurrentQ;              /**< Physical state: true Q current (Amperes) */
    double adVoltageDHistory[FOC_IDENTIFY_PLANT_MAX_DELAY];
    double adVoltageQHistory[FOC_IDENTIFY_PLANT_MAX_DELAY];
    uint32_t wHistoryIndex;
    uint32_t wStepCount;
    double dNoiseSeed;
} foc_identify_plant_t;

static inline void foc_identify_plant_Init(
    foc_identify_plant_t *ptPlant,
    const foc_identify_plant_cfg_t *ptCfg)
{
    uint32_t i = 0U;

    if (ptPlant == NULL || ptCfg == NULL) {
        return;
    }
    ptPlant->tCfg = *ptCfg;
    ptPlant->dCurrentD = 0.0;
    ptPlant->dCurrentQ = 0.0;
    for (i = 0U; i < FOC_IDENTIFY_PLANT_MAX_DELAY; i++) {
        ptPlant->adVoltageDHistory[i] = 0.0;
        ptPlant->adVoltageQHistory[i] = 0.0;
    }
    ptPlant->wHistoryIndex = 0U;
    ptPlant->wStepCount = 0U;
    ptPlant->dNoiseSeed = 0.123456789;
}

static inline void foc_identify_plant_ResetState(foc_identify_plant_t *ptPlant)
{
    uint32_t i = 0U;

    if (ptPlant == NULL) {
        return;
    }
    ptPlant->dCurrentD = 0.0;
    ptPlant->dCurrentQ = 0.0;
    for (i = 0U; i < FOC_IDENTIFY_PLANT_MAX_DELAY; i++) {
        ptPlant->adVoltageDHistory[i] = 0.0;
        ptPlant->adVoltageQHistory[i] = 0.0;
    }
    ptPlant->wHistoryIndex = 0U;
    ptPlant->wStepCount = 0U;
}

/**
 * @brief Step the plant physics by one Ts interval using exact analytical integration.
 * @param ptPlant Plant simulator.
 * @param dSubmittedVd Applied terminal D voltage (Volts).
 * @param dSubmittedVq Applied terminal Q voltage (Volts).
 * @param pdObservedId Output pointer: ADC-observed D current (Amperes).
 * @param pdObservedIq Output pointer: ADC-observed Q current (Amperes).
 */
static inline void foc_identify_plant_Step(
    foc_identify_plant_t *ptPlant,
    double dSubmittedVd,
    double dSubmittedVq,
    double *pdObservedId,
    double *pdObservedIq)
{
    uint32_t wDelay = 0U;
    uint32_t wDelayedIdx = 0U;
    double dAppliedVd = 0.0;
    double dAppliedVq = 0.0;
    double dDecayD = 0.0;
    double dDecayQ = 0.0;
    double dNextId = 0.0;
    double dNextIq = 0.0;
    double dNoiseD = 0.0;
    double dNoiseQ = 0.0;

    if (ptPlant == NULL) {
        return;
    }

    wDelay = ptPlant->tCfg.wSubmitDelaySamples;
    if (wDelay >= FOC_IDENTIFY_PLANT_MAX_DELAY) {
        wDelay = FOC_IDENTIFY_PLANT_MAX_DELAY - 1U;
    }

    /* Store submitted voltage into history buffer. */
    ptPlant->adVoltageDHistory[ptPlant->wHistoryIndex] = dSubmittedVd;
    ptPlant->adVoltageQHistory[ptPlant->wHistoryIndex] = dSubmittedVq;

    /* Retrieve delayed voltage for physics integration. */
    wDelayedIdx = (ptPlant->wHistoryIndex + FOC_IDENTIFY_PLANT_MAX_DELAY - wDelay)
                  % FOC_IDENTIFY_PLANT_MAX_DELAY;
    dAppliedVd = ptPlant->adVoltageDHistory[wDelayedIdx];
    dAppliedVq = ptPlant->adVoltageQHistory[wDelayedIdx];

    /* Handle fractional delay if configured. */
    if (ptPlant->tCfg.dSubmitFractionalDelay > 0.0 && wDelay > 0U) {
        uint32_t wPriorIdx = (wDelayedIdx + FOC_IDENTIFY_PLANT_MAX_DELAY - 1U)
                             % FOC_IDENTIFY_PLANT_MAX_DELAY;
        double dPriorVd = ptPlant->adVoltageDHistory[wPriorIdx];
        double dPriorVq = ptPlant->adVoltageQHistory[wPriorIdx];
        double dFrac = ptPlant->tCfg.dSubmitFractionalDelay;
        dAppliedVd = (1.0 - dFrac) * dAppliedVd + dFrac * dPriorVd;
        dAppliedVq = (1.0 - dFrac) * dAppliedVq + dFrac * dPriorVq;
    }

    /* Advance history circular pointer. */
    ptPlant->wHistoryIndex = (ptPlant->wHistoryIndex + 1U)
                             % FOC_IDENTIFY_PLANT_MAX_DELAY;
    ptPlant->wStepCount++;

    /* Exact analytical integration of di/dt = (v - offset - R*i) / L over dt. */
    if (ptPlant->tCfg.dInductanceD > 1e-9 && ptPlant->tCfg.dResistance > 1e-9) {
        dDecayD = exp(-ptPlant->tCfg.dResistance * ptPlant->tCfg.dDt /
                      ptPlant->tCfg.dInductanceD);
        dNextId = ptPlant->dCurrentD * dDecayD +
                  (dAppliedVd - ptPlant->tCfg.dVoltageOffsetD) /
                  ptPlant->tCfg.dResistance * (1.0 - dDecayD);
    } else {
        dNextId = ptPlant->dCurrentD;
    }

    if (ptPlant->tCfg.dInductanceQ > 1e-9 && ptPlant->tCfg.dResistance > 1e-9) {
        dDecayQ = exp(-ptPlant->tCfg.dResistance * ptPlant->tCfg.dDt /
                      ptPlant->tCfg.dInductanceQ);
        dNextIq = ptPlant->dCurrentQ * dDecayQ +
                  (dAppliedVq - ptPlant->tCfg.dVoltageOffsetQ) /
                  ptPlant->tCfg.dResistance * (1.0 - dDecayQ);
    } else {
        dNextIq = ptPlant->dCurrentQ;
    }

    /* Check physical current saturation if configured. */
    if (ptPlant->tCfg.dCurrentSatMax > 1e-6) {
        if (dNextId > ptPlant->tCfg.dCurrentSatMax) {
            dNextId = ptPlant->tCfg.dCurrentSatMax;
        } else if (dNextId < -ptPlant->tCfg.dCurrentSatMax) {
            dNextId = -ptPlant->tCfg.dCurrentSatMax;
        }
        if (dNextIq > ptPlant->tCfg.dCurrentSatMax) {
            dNextIq = ptPlant->tCfg.dCurrentSatMax;
        } else if (dNextIq < -ptPlant->tCfg.dCurrentSatMax) {
            dNextIq = -ptPlant->tCfg.dCurrentSatMax;
        }
    }

    /* Update true physical state. */
    ptPlant->dCurrentD = dNextId;
    ptPlant->dCurrentQ = dNextIq;

    /* Simple deterministic pseudo-random noise generator: sin-based hash. */
    ptPlant->dNoiseSeed += 0.314159265;
    dNoiseD = sin(ptPlant->dNoiseSeed * 17.0) * ptPlant->tCfg.dAdcCurrentNoiseD;
    dNoiseQ = cos(ptPlant->dNoiseSeed * 23.0) * ptPlant->tCfg.dAdcCurrentNoiseQ;

    /* Observed current = Physical current + ADC sensor offset + deterministic noise. */
    if (pdObservedId != NULL) {
        *pdObservedId = ptPlant->dCurrentD +
                        ptPlant->tCfg.dAdcCurrentOffsetD +
                        dNoiseD;
    }
    if (pdObservedIq != NULL) {
        *pdObservedIq = ptPlant->dCurrentQ +
                        ptPlant->tCfg.dAdcCurrentOffsetQ +
                        dNoiseQ;
    }
}

#endif /* FOC_IDENTIFY_PLANT_H */
