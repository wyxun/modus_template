/****************************************************************************
 * @file    motor_position.h
 * @brief   Typed mechanical-position dependency used by Motor.
 * @author  Codex
 * @date    2026-09-16
 ****************************************************************************/

#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include <stdint.h>

#include "foc_position.h"

/**
 * @brief One position snapshot provider.
 * @note The provider returns the complete mechanical angle/speed/validity
 *       snapshot. A target-specific provider may be a direct static binding;
 *       this dynamic form is only the portable fallback.
 */
typedef struct {
    foc_result_t (*fnGetPosition)(const void *pContext,
                                  uint32_t wNowTick,
                                  foc_position_t *ptPosition);
    const void *pContext;
} motor_position_provider_t;

/* Source compatibility for applications that used the old name. The canonical
 * name is provider because this object describes a position source, not a
 * hardware device instance. */
typedef motor_position_provider_t motor_position_if_t;

/* These operations receive the Motor pointer. A target with a fixed position
 * implementation may replace these calls with a direct typed provider. The
 * fallback below remains a single function-pointer call for standalone
 * builds. Zero capture intentionally reuses the same current snapshot: Motor
 * stores the resulting electrical zero after a valid read. */
#define FOC_POSITION_DISPATCH_GET(M, T, O) \
    ((M)->tPosition.fnGetPosition((M)->tPosition.pContext, (T), (O)))
#define FOC_POSITION_DISPATCH_CAPTURE_ZERO(M, T, O) \
    FOC_POSITION_DISPATCH_GET((M), (T), (O))

#ifndef FOC_POSITION_GET
#define FOC_POSITION_GET(M, T, O) FOC_POSITION_DISPATCH_GET(M, T, O)
#endif
#ifndef FOC_POSITION_CAPTURE_ZERO
#define FOC_POSITION_CAPTURE_ZERO(M, T, O) \
    FOC_POSITION_DISPATCH_CAPTURE_ZERO(M, T, O)
#endif

#endif /* MOTOR_POSITION_H */
