/**
 * @file mfoc_p10_physical.h
 * @brief G431 bench-only current response and KCL experiments.
 */
#ifndef MFOC_P10_PHYSICAL_H
#define MFOC_P10_PHYSICAL_H

#include <stdbool.h>

/** @brief Consume one completed frame when a physical experiment owns PWM.
 * @return True if the experiment took the frame, including on failure. */
bool mfoc_p10_PhysicalStep(void);

#endif
