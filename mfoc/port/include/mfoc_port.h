/**
 * @file mfoc_port.h
 * @brief Statically bound physical quantities, time and output operations.
 * @author wyxun
 * @date 2026-10-21
 */
#ifndef MFOC_PORT_H
#define MFOC_PORT_H

#include "mfoc_angle.h"
#include "mfoc_config.h"

/* One statically bound power stage. The target supplies the implementation
 * and owns sampling, PWM trigger and update timing. */

/** @brief Common conversion bases; copied/precomputed during Port Init.
 * Current/voltage bases are physical mA/mV, speed base is electrical Hz.
 * Sensor settings and hardware resources remain in the target adapter. */
typedef struct {
    uint32_t current_base_milliamp; /**< Ibase for phase current conversion. */
    uint32_t voltage_base_millivolt; /**< Vbase for bus conversion. */
    mfoc_scalar_t electrical_speed_base_hz; /**< Speed PU reference. */
    uint16_t pole_pairs; /**< Mechanical to electrical position conversion. */
} mfoc_port_cfg_t;

/** @brief Prepare board feedback and calibrate phase current conversion.
 * @param cfg Motor's common bases and pole-pair count; not retained.
 * @return OK, NULL, INVALID_ARGUMENT, OUT_OF_RANGE, INVALID_STATE, SAFETY or IO.
 * @note Calibration may wait for completed frames with a bounded duration.
 * Call after timebase and ADC trigger setup, before enabling the control IRQ,
 * with Motor stopped and phase outputs gated. Exclude other frame consumers.
 * Init leaves phase outputs gated. */
mfoc_result_t mfoc_port_init(const mfoc_port_cfg_t *cfg);

/** @brief Read the latest published DC-bus voltage in Vdc/Vbase.
 * @param bus Output; valid only on OK.
 * @return OK, NULL, INVALID_STATE if the ADC sample is not ready, or
 * OUT_OF_RANGE or IO.
 * @note Reads a stable adapter-published sample without waiting; an active
 * publication returns INVALID_STATE and leaves the output unchanged. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus);

#if MFOC_FEEDBACK_ENABLE_POSITION
/** @brief Service the statically bound physical position source.
 * @return Sampling/service status.
 * @note Foreground only, including while stopped. The provider owns its
 * sampling cadence and component instance; no Motor/Feedback state is used. */
mfoc_result_t mfoc_port_position_run(void);

/** @brief Obtain one coherent electrical angle/speed result.
 * @param angle Electrical BAM32. @param speed Electrical speed PU.
 * @return Position status; both outputs unchanged on failure.
 * @note Control ISR, valid output pointers. May extrapolate a published
 * sample, but must not wait for a sensor bus or initiate blocking I/O. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed);

/** @brief Establish physical position zero against a held electrical angle.
 * @param held_angle Known electrical BAM32 while the rotor is held.
 * @return Calibration/read status; failure preserves the previous zero.
 * @note Foreground with readers excluded, or during fixed-field alignment
 * where measured position does not control the applied field. */
mfoc_result_t mfoc_port_position_calibrate_zero(mfoc_angle_t held_angle);
#endif

/** @brief Read the complete raw timebase. @return Monotonic modulo ticks.
 * @note Stable in foreground/ISR; same source as tick_frequency. The target
 * handles narrow hardware counters. Do not reset it while objects use it. */
mfoc_tick_t mfoc_port_tick(void);

/** @brief Read the initialized timebase frequency. @return Ticks/second. */
uint32_t mfoc_port_tick_frequency(void);

/** @brief Read and normalize one fresh, coherent completed ADC frame.
 * @param current U/V/W phase currents in I/Ibase; valid only on OK.
 * @return OK, INVALID_STATE if unconfigured/not ready, or IO.
 * @note current must be valid. The target applies the ADC format, offsets,
 * phase polarity and scale for the configured current base. Values are
 * representable and are not clamped to Motor operating limits.
 * No conversion wait, Clarke transform or PWM gate change is performed. */
mfoc_result_t mfoc_port_current(mfoc_uvw_t *current);

/** @brief Submit trusted duties without changing phase output gates.
 * @param duty U/V/W phase duties; each must be finite and within [0,1].
 * @return OK, OUT_OF_RANGE for rejected hardware state/frame, or IO.
 * @note Control ISR; trusted data, no duplicate finite/range checks.
 * Updates never enable outputs or clear a physical fault. */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty);

/** @brief Load initial duties, then start the phase outputs.
 * @param initial_duty Trusted U/V/W duties in [0,1].
 * @return Write/enable status; a physical startup fault returns SAFETY.
 * @note Foreground startup. The target makes the initial duties effective
 * before enabling phase gates; hardware update details stay in the target. */
mfoc_result_t mfoc_port_pwm_start(const mfoc_uvw_t *initial_duty);

/** @brief Close phase outputs while preserving the ADC trigger.
 * @return OK or IO from the target shutdown.
 * @note Safe to repeat; does not clear the physical fault latch. */
mfoc_result_t mfoc_port_pwm_stop(void);

#endif
