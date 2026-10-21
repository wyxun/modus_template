/**
 * @file mfoc_driver.h
 * @brief Top-level sampling, speed PI and Motor control scheduling.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_DRIVER_H
#define MFOC_DRIVER_H

#include "mfoc_motor.h"
#include "mfoc_feedback.h"
#include "mfoc_port.h"

/** @brief Initial configuration on the same bases as Motor/Core/port.
 * speed_divider = 0 selects dq current control and ignores speed parameters.
 * Otherwise speed PI runs on the first Step and every N Steps thereafter;
 * its ki_ts must contain Ki * N * current_loop_period. Its output/integral
 * use I/Ibase. Output bounds must include zero and fit Motor's i_max.
 * Speed mode produces iq_ref with id_ref = 0; no field weakening. */
typedef struct {
    mfoc_motor_cfg_t motor;
    mfoc_feedback_cfg_t feedback;
    mfoc_pi_params_t speed;
    uint16_t speed_divider;
} mfoc_driver_cfg_t;

/** @brief Caller-owned scheduler; Motor owns lifecycle and fault state.
 * One instance exclusively controls the statically bound port/power stage.
 * Samples and current Core output are local; Driver retains the prior
 * successful Core output for feedback, including its bus snapshot. Feedback
 * owns its runtime state. Do not modify runtime members. Use Motor's Trip,
 * ClearFault and GetStatus on &driver->motor; start with mfoc_motor_Start and
 * use mfoc_driver_Stop to reset outer-loop state on shutdown.
 * Commands run in the control ISR or while IDLE. Init/Stop/Run run in
 * foreground; only Run may overlap Step through backend snapshot publication.
 * Other mutating operations must not overlap Step or each other. */
typedef struct {
    mfoc_motor_t motor;
    mfoc_feedback_t feedback;
    mfoc_core_output_t previous_core_output;
    bool previous_core_output_valid;
    mfoc_pi_t speed;
    mfoc_scalar_t speed_ref;
    uint16_t speed_divider;
    uint16_t speed_count;
    volatile mfoc_result_t irq_result; /* Last IRQ Step result; diagnostics. */
} mfoc_driver_t;

/** @brief Initialize control and collect current zero offsets.
 * @param driver Instance. @param cfg Initialization configuration.
 * @return Configuration status; failure leaves driver unchanged.
 * @note ADC triggering and perfc must be initialized before Init. Call before
 * enabling any ISR reader of the sample resource. Init blocks for ten frames
 * spaced 10 ms apart, then reads the published DC bus, with phase outputs
 * gated throughout. */
mfoc_result_t mfoc_driver_Init(mfoc_driver_t *driver,
                              const mfoc_driver_cfg_t *cfg);

/** @brief Service enabled feedback algorithms from the foreground.
 * @param driver Instance. @return Algorithm sampling/state status.
 * @note Call repeatedly while idle or running, including once successfully
 * before Motor Start. Each algorithm owns its cadence. This does not update bus
 * coefficients or Motor state; expired feedback is rejected by ISR Step. */
mfoc_result_t mfoc_driver_Run(mfoc_driver_t *driver);

/** @brief Stop Motor and clear speed reference, integral and cadence.
 * @param driver Instance. @return Shutdown/reset status.
 * @note Motor faults remain latched. ISR faults use mfoc_motor_Trip. */
mfoc_result_t mfoc_driver_Stop(mfoc_driver_t *driver);

/** @brief Sample the published bus value and update Motor/Core coefficients.
 * @param driver Running or idle instance.
 * @return Port or Motor bus-update status; failure leaves bus unchanged.
 * @note Call at the desired bus update cadence from the control ISR while
 * RUNNING, or from foreground while IDLE. Serialize with Driver Step. */
mfoc_result_t mfoc_driver_UpdateBus(mfoc_driver_t *driver);

/** @brief Set dq reference when speed_divider is zero.
 * @param driver Instance. @param ref Current in I/Ibase.
 * @return Motor command status or INVALID_STATE in speed mode. */
mfoc_result_t mfoc_driver_SetCurrent(mfoc_driver_t *driver,
                                    const mfoc_dq_t *ref);

/** @brief Set speed reference when the speed PI is configured.
 * @param driver Instance. @param ref omega_e/omega_base.
 * @return OK, NULL, INVALID_STATE or OUT_OF_RANGE; rejection retains ref. */
mfoc_result_t mfoc_driver_SetSpeed(mfoc_driver_t *driver,
                                  mfoc_scalar_t ref);

/** @brief Read current and feedback, run speed PI and execute Motor.
 * @param driver Instance.
 * @return Control/port status; sampling/math failure trips Motor.
 * @note Call once per completed current frame in the control ISR. Sampling
 * does not wait or trigger ADC; Motor submits the resulting PWM. Idle/fault
 * calls consume no frame. Feedback and normalized current are not rechecked
 * for finiteness here. Speed output is held between speed PI updates. */
mfoc_result_t mfoc_driver_Step(mfoc_driver_t *driver);

#endif
