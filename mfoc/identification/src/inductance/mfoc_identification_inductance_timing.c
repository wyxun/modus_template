/**
 * @file mfoc_identification_inductance_timing.c
 * @brief Optional foreground timing for standstill inductance capture.
 */
#include "mfoc_identification_inductance_internal.h"
#include <float.h>
#include <stddef.h>

/** @copydoc mfoc_identification_inductance_InitTiming */
mfoc_result_t mfoc_identification_inductance_InitTiming(
    mfoc_scalar_t *dt_pu, uint32_t sample_hz, mfoc_scalar_t electrical_base_hz)
{
    if (dt_pu == NULL) { return MFOC_RESULT_NULL; }
    if (sample_hz == 0U || !(electrical_base_hz > MFOC_ZERO)) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    const float dt = (6.28318530718f * mfoc_to_float(electrical_base_hz)) /
                     (float)sample_hz;
#if defined(MFOC_NUMERIC_FIXED)
    if (dt >= 65536.0f) { return MFOC_RESULT_OUT_OF_RANGE; }
    /* Match the compiled profile's positive Q15 rounding. */
    const mfoc_scalar_t raw = (mfoc_scalar_t)(dt * MFOC_Q_SCALE + 0.5f);
    /* At |u|, |i| <= 1 PU, bound the residual before scaling in Finish. */
    const int64_t max_residual = (int64_t)MFOC_IDENTIFICATION_L_SAMPLES *
        MFOC_Q_SCALE * (MFOC_Q_SCALE + MFOC_SCALAR(MFOC_MOTOR_RS_MAX_PU));
    const int64_t max_quotient =
        max_residual / MFOC_SCALAR(MFOC_IDENTIFICATION_L_DELTA_MIN);
    if (raw == 0 || raw > INT64_MAX / max_quotient) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    *dt_pu = raw;
#else
    if (!(dt > 0.0f && dt <= FLT_MAX)) { return MFOC_RESULT_OUT_OF_RANGE; }
    *dt_pu = dt;
#endif
    return MFOC_RESULT_OK;
}

/** @copydoc mfoc_identification_inductance_FinishPrepared */
mfoc_fsm_rt_t mfoc_identification_inductance_FinishPrepared(
    const mfoc_identification_inductance_t *state, mfoc_scalar_t rs,
    mfoc_scalar_t dt_pu, mfoc_scalar_t *inductance)
{
    return _mfoc_identification_EvaluateInductance(state, rs, dt_pu,
                                                   inductance);
}
