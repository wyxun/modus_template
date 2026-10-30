/** @file mfoc_sensorless_start.h
 * @brief Forced ramp and sensorless handover, owned by Motor. */
#ifndef MFOC_SENSORLESS_START_H
#define MFOC_SENSORLESS_START_H
#include "mfoc_core.h"
#include "mfoc_feedback.h"
#include "mfoc_openloop.h"

/** @brief Angle handover stages, independent of current/speed mode. */
typedef enum {
    MFOC_SENSORLESS_START_NONE = 0, /* Normal feedback control. */
    MFOC_SENSORLESS_START_RAMP,     /* Fixed-angle hold, then forced-speed ramp. */
    MFOC_SENSORLESS_START_ACQUIRE,  /* Observe and adjust current within the budget. */
} mfoc_sensorless_start_state_t;

/** @brief Sensorless startup coefficients and ISR-owned history. */
typedef struct {
    mfoc_sensorless_start_state_t state;
    uint32_t timeout_ticks;
    uint32_t stable_ticks;
    int32_t offset_step;      /* Positive BAM32 increment per control tick. */
    mfoc_scalar_t speed_window;
    uint32_t remaining;       /* Entire ACQUIRE budget. */
    uint32_t stable;          /* Consecutive qualified samples. */
    int32_t offset;           /* Forced angle minus observer angle, BAM32. */
    uint32_t iq_ticks;       /* Adaptation period in ISR ticks. */
    uint32_t iq_count;
    mfoc_scalar_t iq_gain;    /* Angle integral gain per update. */
    mfoc_scalar_t iq_step;    /* Maximum current change per update. */
    mfoc_scalar_t iq_bias;    /* Slow current amplitude, before damping. */
    mfoc_scalar_t iq_max;     /* Current requested at SensorlessStart. */
    bool captured;           /* Observer angle owns the current reference. */
} mfoc_sensorless_start_t;

/** @brief Per-step feedback and Motor startup context. */
typedef struct {
    const mfoc_feedback_output_t *feedback;
    mfoc_result_t feedback_rc;
    mfoc_scalar_t speed_ref;
    bool hold_active;
    bool complete;
} mfoc_sensorless_input_t;

/** @brief Precompute timing coefficients for the control sample rate. */
void mfoc_sensorless_start_Init(mfoc_sensorless_start_t *s, uint32_t fs);
/** @brief Prepare a new forced startup; Motor controls PWM and angle. */
void mfoc_sensorless_start_Start(mfoc_sensorless_start_t *s,
    mfoc_scalar_t speed, mfoc_scalar_t current);
/** @brief Advance ramp/acquisition; modifies angle and current references. */
mfoc_result_t mfoc_sensorless_start_Step(mfoc_sensorless_start_t *s,
    mfoc_openloop_t *loop, mfoc_sensorless_input_t *ctx,
    mfoc_core_input_t *input);
#endif
