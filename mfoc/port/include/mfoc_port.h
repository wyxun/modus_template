/**
 * @file mfoc_port.h
 * @brief MFOC phase-current, bus-voltage and PWM port contract.
 * @author wyxun
 * @date 2026-10-21
 */
#ifndef MFOC_PORT_H
#define MFOC_PORT_H

#include "mfoc_angle.h"

/* One statically bound power stage. The target supplies the implementation
 * and owns sampling, PWM trigger and update timing. */

/** @brief Prepare board feedback and calibrate phase current conversion.
 * @return OK, INVALID_ARGUMENT, OUT_OF_RANGE, INVALID_STATE, SAFETY or IO.
 * @note Calibration may wait for completed frames with a bounded duration.
 * Call after timebase and ADC trigger setup, before enabling the control IRQ,
 * with Motor stopped and phase outputs gated. Exclude other frame consumers.
 * Init leaves phase outputs gated. */
mfoc_result_t mfoc_port_init(void);

/** @brief Read the latest published DC-bus voltage in Vdc/Vbase.
 * @param bus Output; valid only on OK.
 * @return OK, NULL, INVALID_STATE if the ADC sample is not ready, or
 * OUT_OF_RANGE or IO.
 * @note Reads a stable MDI-published mean without waiting; an active
 * publication returns INVALID_STATE and leaves the output unchanged. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus);

/** @brief Read the board encoder's mechanical angle in BAM32.
 * @param angle Output, unchanged on error. @return OK, NULL, or IO.
 * @note Foreground only; the target owns sensor access and its timeout. */
mfoc_result_t mfoc_port_encoder(mfoc_angle_t *angle);

/** @brief Read the low 32 bits of the timebase. @return Wrapping raw ticks.
 * @note Same raw clock as mfoc_port_tick_frequency; callable from ISR. */
uint32_t mfoc_port_tick(void);

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

/** @brief Submit trusted duties and optionally start the phase outputs.
 * @param duty U/V/W phase duties; each must be finite and within [0,1].
 * @param start True for startup; false only updates compare values.
 * @return OK, OUT_OF_RANGE for rejected hardware state/frame, or IO.
 * A physical fault during startup returns SAFETY.
 * @note duty must be valid; no duplicate finite/range checks. Startup writes
 * duty before enabling outputs. The target must make that duty effective
 * before opening the phase gates. ISR updates use start=false;
 * they never change output gates or clear a hardware fault. */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty, bool start);

/** @brief Close phase outputs while preserving the ADC trigger.
 * @return OK or IO from the target shutdown.
 * @note Safe to repeat; does not clear the physical fault latch. */
mfoc_result_t mfoc_port_pwm_stop(void);

#endif
