/****************************************************************************
 * @file    foc.h
 * @brief   FOC algorithms and Motor public API umbrella.
 * @note    MODUS composition is application-specific; include foc_app.h
 *          separately when the application uses foc_app_t.
 ****************************************************************************/

#ifndef __FOC_H__
#define __FOC_H__

#include "foc_config.h"

#include "math/foc_math_types.h"

#include "middleware/foc_core.h"
#include "control/foc_pid.h"
#include "modulation/foc_modulation.h"
#include "observer/foc_encoder.h"
#include "observer/foc_observer.h"
#include "identify/identify.h"
#include "motor/motor.h"

#endif /* __FOC_H__ */
