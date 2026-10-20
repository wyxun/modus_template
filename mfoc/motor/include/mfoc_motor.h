/**
 * @file mfoc_motor.h
 * @brief Motor-owned per-unit parameters and current-control state.
 * @author Codex
 * @date 2026-10-20
 */
#ifndef MFOC_MOTOR_H
#define MFOC_MOTOR_H

#include "mfoc_core.h"

/** @brief Motor constants on one fixed set of bases.
 * Use amplitude-invariant Clarke and peak phase-current base Ib.
 * Vb is the phase-voltage base; wb is electrical rad/s, not Hz.
 * rs = Rs*Ib/Vb; ld/lq = Ld/Lq*wb*Ib/Vb; flux = psi*wb/Vb.
 * With p=pole_pairs: tb=1/wb, wmb=wb/p, psib=Vb/wb,
 * Tb=1.5*p*psib*Ib, Jb=Tb*tb/wmb; inertia=J/Jb.
 * All scalars must be finite. rs/ld/lq/flux must be positive.
 * inertia >= 0; zero means unspecified, unused by current control.
 * Convert physical units once before Init; pole_pairs is an integer count. */
typedef struct {
    uint16_t pole_pairs;
    mfoc_scalar_t rs;      /* Per-phase stator resistance, R/Rb. */
    mfoc_scalar_t ld;      /* d-axis inductance, Ld/Lb. */
    mfoc_scalar_t lq;      /* q-axis inductance, Lq/Lb. */
    mfoc_scalar_t flux;    /* Permanent-magnet flux linkage, psi/psib. */
    mfoc_scalar_t inertia; /* Rotor inertia, J/Jb; zero if unspecified. */
} mfoc_motor_params_t;

/** @brief Motor operating limits; current uses the same Ib as Core. */
typedef struct {
    mfoc_scalar_t i_max; /* Positive phase-current amplitude limit, I/Ib. */
} mfoc_motor_limits_t;

/** @brief Persistent current command; angle is supplied per control step. */
typedef struct {
    mfoc_dq_t i_ref; /* dq current reference, I/Ib. */
} mfoc_motor_command_t;

/** @brief Motor lifecycle; zero-initialized storage cannot start PWM. */
typedef enum {
    MFOC_MOTOR_UNINITIALIZED = 0,
    MFOC_MOTOR_IDLE,
    MFOC_MOTOR_RUNNING,
    MFOC_MOTOR_FAULT,
} mfoc_motor_state_t;

/** @brief Fault bits accumulate until an explicit successful recovery. */
typedef enum {
    MFOC_MOTOR_FAULT_SAMPLE = 1U << 0,
    MFOC_MOTOR_FAULT_OVERCURRENT = 1U << 1,
    MFOC_MOTOR_FAULT_FEEDBACK = 1U << 2,
    MFOC_MOTOR_FAULT_MATH = 1U << 3,
    MFOC_MOTOR_FAULT_PWM = 1U << 4,
    MFOC_MOTOR_FAULT_EXTERNAL = 1U << 5,
} mfoc_motor_fault_t;

/** @brief Synchronous inputs; current uses Ib and angle is electrical.
 * Supply a fresh Sample result and angle for that sampling instant.
 * angle_valid is determined by the external angle source. */
typedef struct {
    mfoc_ab_t i;
    mfoc_angle_t angle;
    bool angle_valid;
} mfoc_motor_input_t;

/** @brief Coherent lifecycle snapshot, not a duplicate persistent cache. */
typedef struct {
    mfoc_motor_state_t state;
    uint32_t faults;
} mfoc_motor_status_t;

/** @brief Initialization input; not retained as a second runtime copy.
 * Motor and Core must share the same fixed Ib/Vb bases. */
typedef struct {
    mfoc_motor_params_t params;
    mfoc_motor_limits_t limits;
    mfoc_core_params_t core;
} mfoc_motor_cfg_t;

/** @brief Motor-owned runtime data; initialize before use.
 * No ADC/PWM handles, sample cache or observer state is stored here.
 * Do not change members directly while dependent algorithms are active.
 * One application-owned instance controls the statically bound power stage.
 * Sample/Step run in one control ISR; they must not overlap each other.
 * Foreground calls may be preempted by that ISR. Only Trip may preempt
 * Step from a higher-priority ISR. Step's final commit checks the latch. */
typedef struct {
    mfoc_motor_params_t params;
    mfoc_motor_limits_t limits;
    mfoc_core_t core;
    mfoc_motor_command_t command;
    volatile mfoc_motor_state_t state;
    volatile uint32_t faults; /* ISR latch; read together via GetStatus. */
} mfoc_motor_t;

/** @brief Validate/copy motor data and initialize current control in IDLE.
 * @param motor Instance; initialization requires inactive hardware.
 * @param cfg Motor PU data and finite Core configuration.
 * @return OK, NULL, INVALID_ARGUMENT or OUT_OF_RANGE.
 * @note Failure leaves motor unchanged. No hardware operation is performed.
 * Init must not be used to clear a live fault or reconfigure a running motor.
 * PI limits must include zero; gains and limits must be finite. */
mfoc_result_t mfoc_motor_Init(mfoc_motor_t *motor,
                             const mfoc_motor_cfg_t *cfg);

/** @brief Check prerequisites, reset PI and enable neutral PWM from IDLE.
 * @param motor Initialized instance. @return Operation status.
 * @note Retains a previously accepted current command. Failure to write
 * or enable PWM trips the motor. Readiness rejection leaves it IDLE.
 * Call in foreground; no alignment or angle acquisition is performed. */
mfoc_result_t mfoc_motor_Start(mfoc_motor_t *motor);

/** @brief Disable PWM, clear command/integrals and retain any fault latch.
 * @param motor Initialized instance. @return Shutdown/reset status.
 * @note Foreground operation; ISR faults use Trip. Idempotent in IDLE. */
mfoc_result_t mfoc_motor_Stop(mfoc_motor_t *motor);

/** @brief Accept a dq command with magnitude <= i_max.
 * @param motor IDLE/RUNNING instance. @param ref Finite current in I/Ibase.
 * @return Status; rejection leaves the previous command intact.
 * @note Foreground or control ISR only; d/q are published together. */
mfoc_result_t mfoc_motor_SetCurrent(mfoc_motor_t *motor,
                                   const mfoc_dq_t *ref);

/** @brief Update Core's bus coefficients without resetting PI.
 * @param motor IDLE/RUNNING instance. @param bus Positive Vdc/Vbase.
 * @return Status; rejected updates retain the previous coefficients.
 * @note Call in the control ISR, or in foreground only while IDLE.
 * Caller must handle invalid measured voltage through its safety policy. */
mfoc_result_t mfoc_motor_SetBus(mfoc_motor_t *motor, mfoc_scalar_t bus);

/** @brief Check phase currents and return Clarke-transformed current.
 * @param motor RUNNING instance. @param current Output valid only on OK.
 * @return Status; bad sampling/overcurrent shuts down and latches a fault.
 * @note Control ISR only. The port performs ADC calibration/normalization. */
mfoc_result_t mfoc_motor_Sample(mfoc_motor_t *motor, mfoc_ab_t *current);

/** @brief Execute current control and submit PWM for the next interval.
 * @param motor RUNNING instance. @param input Fresh current/angle.
 * @param output Core results, valid only on OK; never aliases motor/input.
 * @return Status; invalid input, math or PWM errors trip the motor.
 * @note Control ISR only. No observer, speed loop or implicit angle source. */
mfoc_result_t mfoc_motor_Step(mfoc_motor_t *motor,
                             const mfoc_motor_input_t *input,
                             mfoc_core_output_t *output);

/** @brief Disable PWM immediately and latch a nonzero set of fault bits.
 * @param motor Initialized instance. @param faults Cause bits.
 * @return Shutdown status; failure also latches PWM fault.
 * @note ISR-safe, including preemption of Step. Does not reset an in-flight
 * Core; ClearFault resets it before another start is permitted. */
mfoc_result_t mfoc_motor_Trip(mfoc_motor_t *motor, uint32_t faults);

/** @brief Confirm shutdown/readiness, reset PI and recover FAULT to IDLE.
 * @param motor Faulted instance. @return Status; failure retains fault.
 * @note Foreground only. Caller confirms external fault causes are gone.
 * Never automatically restarts; clear the physical latch in the adapter. */
mfoc_result_t mfoc_motor_ClearFault(mfoc_motor_t *motor);

/** @brief Read state and fault bits under one short critical section.
 * @param motor Instance. @param status Snapshot destination.
 * @return OK or NULL. */
mfoc_result_t mfoc_motor_GetStatus(const mfoc_motor_t *motor,
                                  mfoc_motor_status_t *status);

#endif
