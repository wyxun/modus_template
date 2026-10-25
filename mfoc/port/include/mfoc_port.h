/**
 * @file mfoc_port.h
 * @brief MFOC phase-current, bus-voltage and PWM port contract.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_PORT_H
#define MFOC_PORT_H

#include "mfoc_angle.h"
#include "mfoc_motor_config.h"

/** @brief Board bus-voltage conversion settings used by mfoc_port_bus(). */
#define MFOC_PORT_BUS_MV_PER_COUNT_NUM UINT32_C(20570)
#define MFOC_PORT_BUS_MV_PER_COUNT_DEN UINT32_C(2457)
#define MFOC_PORT_BUS_BASE_MILLIVOLT MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT
#define MFOC_PORT_BUS_OFFSET_MILLIVOLT INT32_C(0)
#define MFOC_PORT_BUS_MAX_MILLIVOLT UINT32_C(60000)
#define MFOC_PORT_ADC_SUM_COUNTS_PER_AMP UINT32_C(136)
#define MFOC_PORT_COUNTS_PER_BASE \
    ((MFOC_PORT_ADC_SUM_COUNTS_PER_AMP * \
      MFOC_MOTOR_BASE_CURRENT_MILLIAMP + 500U) / 1000U)
/* Preserve G431 legacy current feedback: (calibrated offset - ADC raw). */
#define MFOC_PORT_CURRENT_INVERTED 1U

/* One statically bound power stage. port/src/mfoc_port.c adapts MFOC to MDI;
 * the target supplies mdi/instance.h with phase_current_completed,
 * adc_bus_voltage and bridge. Runtime operations are bounded and callable
 * from the control ISR.
 * Sampling and PWM trigger/update timing belong to the target setup. */

/** @brief Verify AS5600 SF=3, measure offsets and prepare conversion.
 * @return OK, INVALID_ARGUMENT, OUT_OF_RANGE, INVALID_STATE, SAFETY or IO.
 * @note Collects ten completed current frames with bounded waits.
 * Call after perfc and ADC trigger setup, before enabling the control IRQ,
 * with Motor stopped and phase outputs gated. Exclude other frame consumers.
 * No bus read, ADC scheduling or interrupt control occurs here.
 * AS5600 CONF is read back after setting only SF; OTP is never written.
 * Polarity and current scale are
 * compile-time settings above. Init does not enable hardware. */
mfoc_result_t mfoc_port_init(void);

/** @brief Read the latest published DC-bus voltage in Vdc/Vbase.
 * @param bus Output; valid only on OK.
 * @return OK, NULL, INVALID_STATE if the ADC sample is not ready, or
 * OUT_OF_RANGE or IO.
 * @note Reads a stable MDI-published mean without waiting; an active
 * publication returns INVALID_STATE and leaves the output unchanged. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus);

/** @brief Read AS5600 mechanical BAM32 through the encoder_angle MDI resource.
 * @param angle Output, unchanged on error. @return OK, NULL, or IO.
 * @note Foreground only; bounded synchronous I2C access. Board initializes
 * the bus and binds the sensor address before calling this function. */
mfoc_result_t mfoc_port_encoder(mfoc_angle_t *angle);

/** @brief Read the low 32 bits of raw_tick. @return Wrapping raw ticks.
 * @note Same raw clock as mfoc_port_tick_frequency; callable from ISR. */
uint32_t mfoc_port_tick(void);

/** @brief Read initialized perfc tick frequency. @return Ticks/second. */
uint32_t mfoc_port_tick_frequency(void);

/** @brief Read and normalize one fresh, coherent completed ADC frame.
 * @param current U/V/W phase currents in I/Ibase; valid only on OK.
 * @return OK, INVALID_STATE if unconfigured/not ready, or IO.
 * @note current must be valid. MDI decodes the ADC format; the port applies
 * offsets, polarity and precomputed count-to-PU gains with Motor/Core Ibase.
 * The ADC
 * count range guarantees finite, representable results; values are not
 * clamped to Motor operating limits.
 * No conversion wait, Clarke transform or PWM gate change is performed. */
mfoc_result_t mfoc_port_current(mfoc_uvw_t *current);

/** @brief Submit trusted duties and optionally start the phase outputs.
 * @param duty U/V/W phase duties; each must be finite and within [0,1].
 * @param start True for startup; false only updates compare values.
 * @return OK, OUT_OF_RANGE for rejected hardware state/frame, or IO.
 * A physical fault during startup returns SAFETY.
 * @note duty must be valid; no duplicate finite/range checks. Startup writes
 * duty before enabling outputs. The target must make that duty effective
 * before its MDI Enable opens the phase gates. ISR updates use start=false;
 * they never change output gates or clear a hardware fault. */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty, bool start);

/** @brief Close phase outputs while preserving the ADC trigger.
 * @return MDI shutdown status mapped to OK or IO.
 * @note Safe to repeat; does not clear the physical fault latch. */
mfoc_result_t mfoc_port_pwm_stop(void);

#endif
