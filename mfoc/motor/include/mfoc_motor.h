/**
 * @file mfoc_motor.h
 * @brief Motor-owned per-unit parameters and current-control state.
 * @author Codex
 * @date 2026-10-20
 */
#ifndef MFOC_MOTOR_H
#define MFOC_MOTOR_H

#include "mfoc_core.h"
#include "mfoc_feedback.h"
#include "mfoc_openloop.h"
#if MFOC_IDENTIFICATION_ENABLE
#include "mfoc_identification.h"
#endif

/** @brief Fixed-field hold duration, serviced by Motor Run. */
#ifndef MFOC_MOTOR_ALIGN_TIME_MS
#define MFOC_MOTOR_ALIGN_TIME_MS 1500U
#endif

/** @brief Motor operating limits in the selected per-unit bases. */
typedef struct {
    mfoc_scalar_t i_max; /* Positive phase-current amplitude limit, I/Ib. */
    mfoc_scalar_t speed_max; /* Electrical speed limit, omega_e/omega_base. */
} mfoc_motor_limits_t;

/** @brief Selected reference type; changes are accepted while stopped. */
typedef enum {
    MFOC_MOTOR_MODE_CURRENT = 0,
    MFOC_MOTOR_MODE_SPEED,
} mfoc_motor_mode_t;

/** @brief Angle handover stages, independent of current/speed mode. */
typedef enum {
    MFOC_START_NONE = 0, /* Normal feedback control. */
    MFOC_START_FORCED,   /* Existing open-loop command; no handover. */
    MFOC_START_WAIT,     /* Hold, ramp and qualify the observer. */
    MFOC_START_REDUCE,   /* Hold forced speed and reduce startup current. */
    MFOC_START_BLEND,    /* Remove residual angle offset, then speed control. */
} mfoc_start_state_t;

/** @brief Startup coefficients and ISR-owned handover history. */
typedef struct {
    uint32_t timeout_ticks;
    uint32_t stable_ticks;
    uint32_t reduce_period;
    int32_t offset_step; /* Positive BAM32 ticks per control sample. */
    mfoc_scalar_t current_step; /* Current reduction per reduce_period. */
    mfoc_scalar_t speed_window;
    uint32_t remaining; /* ISR timeout budget; paused during fixed-angle hold. */
    uint32_t stable;
    uint32_t reduce_count;
    int32_t offset; /* Control d-axis minus estimated rotor d-axis. */
} mfoc_motor_start_t;

/** @brief Persistent current command in the selected control frame. */
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

/** @brief Lifecycle values; a foreground read may straddle an ISR update. */
typedef struct {
    mfoc_motor_state_t state;
    uint32_t faults;
} mfoc_motor_status_t;

/** @brief Initialization input; not retained as a second runtime copy.
 * Motor and Core must share the same fixed Ib/Vb bases.
 * speed_divider=0 disables speed mode; N>0 enables it, with
 * ki_ts=Ki*N*current_period and output bounds inside +/-i_max.
 * speed_max is required when speed PI or OpenLoop is enabled. */
typedef struct {
    mfoc_motor_params_t params;
    mfoc_motor_limits_t limits;
    mfoc_core_params_t core;
    mfoc_feedback_cfg_t feedback;
    mfoc_pi_params_t speed;
    uint16_t speed_divider; /* 0: current mode; N: speed PI every N Steps. */
    mfoc_openloop_cfg_t openloop; /* sample_hz=0 disables forced startup. */
} mfoc_motor_cfg_t;

/** @brief Motor-owned runtime data; initialize before use.
 * Owns Feedback, speed PI, current control, and prior Core output.
 * No ADC/PWM handles are stored here.
 * Do not change members directly while dependent algorithms are active.
 * One application-owned instance controls the statically bound power stage.
 * Mode and live commands are serialized against the ISR by the application.
 * Step runs in one highest-priority, nonreentrant control ISR. There is no
 * library interrupt masking. Live command updates run in that ISR; other
 * foreground mutations are serialized by the application. Stop publishes a
 * nonrunning state before resetting Core; Start publishes RUNNING last. */
typedef struct {
    mfoc_motor_params_t params;
    mfoc_motor_limits_t limits;
    mfoc_core_t core;
    mfoc_feedback_t feedback;
#if MFOC_IDENTIFICATION_ENABLE
    mfoc_identification_t identification;
#endif
    mfoc_openloop_t openloop;
    mfoc_motor_command_t command;
    volatile mfoc_motor_mode_t mode;
    volatile mfoc_motor_state_t state;
    volatile uint32_t faults; /* ISR fault latch. */
    volatile mfoc_result_t irq_result; /* Last IRQ Step result; diagnostics. */
    mfoc_core_output_t previous_core_output;
    bool previous_core_output_valid;
#if MFOC_IDENTIFICATION_ENABLE
    mfoc_ab_t previous_voltage_ab; /* Prior requested voltage, V/Vbase. */
#endif
    mfoc_pi_t speed;
    mfoc_scalar_t speed_ref;
    uint16_t speed_divider;
    uint16_t speed_count;
    mfoc_angle_t align_angle;
    int64_t hold_deadline; /* Run deadline for fixed-angle hold only. */
    volatile bool hold_active; /* Freeze angle for align or startup hold. */
    mfoc_start_state_t start_state;
    mfoc_motor_start_t start;
} mfoc_motor_t;

/** @brief Validate/copy motor data and initialize current control in IDLE.
 * @param motor Instance; initialization requires inactive hardware.
 * @param cfg Motor PU data and finite Core configuration.
 * @return OK, NULL, INVALID_ARGUMENT or OUT_OF_RANGE.
 * @note Failure leaves motor unchanged. After validating control settings,
 * Port Init collects ten ADC offset frames spaced 10 ms apart. ADC trigger
 * and perfc must be ready, with control IRQ disabled and phase outputs off.
 * Initial bus comes from cfg.core.bus; Init does not sample bus or start PWM.
 * Init must not clear a live fault or reconfigure a running motor.
 * PI limits must include zero; gains and limits must be finite. */
mfoc_result_t mfoc_motor_Init(mfoc_motor_t *motor,
                             const mfoc_motor_cfg_t *cfg);

/** @brief Reset PI and start neutral PWM from IDLE.
 * @param motor Initialized instance. @return Operation status.
 * @note Retains a previously accepted current command. Failure to write
 * or enable PWM trips the motor. Port Init must already have succeeded.
 * Call in foreground; no bus read or angle acquisition occurs here. */
mfoc_result_t mfoc_motor_Start(mfoc_motor_t *motor);

/** @brief Disable PWM, clear command/integrals and retain any fault latch.
 * @param motor Initialized instance. @return Shutdown/reset status.
 * @note Foreground or control ISR; no waits.
 * Foreground callers serialize control changes. Idempotent in IDLE. */
mfoc_result_t mfoc_motor_Stop(mfoc_motor_t *motor);

/** @brief Select current mode and accept a dq command within i_max.
 * @param motor IDLE/RUNNING instance. @param ref Finite current in I/Ibase.
 * @return Status; rejection leaves the previous command intact.
 * @note Selects current mode while IDLE. While RUNNING, only update a
 * current-mode Motor from its control ISR; this function masks no IRQ. */
mfoc_result_t mfoc_motor_SetCurrent(mfoc_motor_t *motor,
                                   const mfoc_dq_t *ref);

/** @brief Update Core's bus coefficients without resetting PI.
 * @param motor IDLE/RUNNING instance. @param bus Positive Vdc/Vbase.
 * @return Status; rejected updates retain the previous coefficients.
 * @note Call in the control ISR, or in foreground only while IDLE.
 * Caller must handle invalid measured voltage through its safety policy. */
mfoc_result_t mfoc_motor_SetBus(mfoc_motor_t *motor, mfoc_scalar_t bus);

/** @brief Poll feedback and service a fixed-field hold.
 * @param motor Valid initialized instance. @return Service/stop status.
 * @note Foreground only; keep calling while idle, running, or aligning.
 * Feedback errors do not block alignment timeout and Stop. */
mfoc_result_t mfoc_motor_Run(mfoc_motor_t *motor);

/** @brief Start fixed electrical angle/current from IDLE.
 * @param motor Valid instance. @param id_pu Positive d-axis PU; iq=0.
 * @param angle_pu Electrical turns in [0,1). @return Startup status.
 * @note At timeout Run reads the selected encoder while holding the field,
 * calibrates electrical zero, then stops. Direction must be configured.
 * Read failure stops and latches a feedback fault; cancellation skips capture. */
mfoc_result_t mfoc_motor_AlignStart(mfoc_motor_t *motor,
    mfoc_scalar_t id_pu, mfoc_scalar_t angle_pu);

/** @brief Hold a fixed field, then ramp a forced electrical angle from IDLE.
 * @param motor Instance with OpenLoop configured.
 * @param speed Nonzero signed electrical PU within speed_max and [-1,1].
 * @param id_pu Positive forced-frame d current within i_max; iq remains zero.
 * @return Command/start status. Existing Stop cancels hold or rotation.
 * @note Encoder/SMO remain observational; no zero capture or SMO takeover.
 * The initial hold lasts MFOC_MOTOR_ALIGN_TIME_MS, serviced by Run. */
mfoc_result_t mfoc_motor_OpenLoopStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t id_pu);

/** @brief Align, force rotation, reduce current and hand over to speed FOC.
 * @param motor IDLE instance. @param speed Signed electrical speed PU.
 * @param current Positive startup current amplitude, I/Ibase.
 * @return Command/PWM status. Requires configured speed PI and OpenLoop.
 * @note Requires SMO/FLUX selected and HFI disabled. Run services alignment;
 * Step owns qualification, bounded current reduction and angle blending.
 * Thresholds are profile settings, not proof of observer lock. */
mfoc_result_t mfoc_motor_SensorlessStart(mfoc_motor_t *motor,
    mfoc_scalar_t speed, mfoc_scalar_t current);

/** @brief Reset HFI and start the Motor with zero current reference.
 * @param motor IDLE instance with HFI compiled in. @return Startup status. */
mfoc_result_t mfoc_motor_HfiStart(mfoc_motor_t *motor);

/** @brief Select speed mode and set a bounded electrical-speed reference.
 * @param motor Instance with the speed PI configured.
 * @param ref Electrical speed PU.
 * @return Command status; rejection retains the previous reference. */
mfoc_result_t mfoc_motor_SetSpeed(mfoc_motor_t *motor, mfoc_scalar_t ref);

/** @brief Run one complete current-sampling and control cycle.
 * @param motor Initialized instance. @return Sampling/control/PWM status.
 * @note Control ISR only, one call per completed current frame. A single
 * RUNNING check precedes current read, Clarke, feedback, optional speed PI,
 * Core, and PWM submission. Idle/fault calls consume no frame. Runtime
 * failures Trip Motor; successful PWM submission publishes the prior output.
 * Forced startup treats observer status as diagnostic. Feedback acquisition
 * holds fundamental current at zero and suspends speed control. Feedback
 * failures in normal control/acquisition Trip, as do sampling/PWM errors. */
mfoc_result_t mfoc_motor_Step(mfoc_motor_t *motor);

/** @brief Read the published bus sample and update Core coefficients.
 * @param motor IDLE/RUNNING instance. @return Sampling/update status.
 * @note Does not wait for an ADC conversion. Call at the desired cadence,
 * in the control ISR while RUNNING, or in foreground while IDLE. */
mfoc_result_t mfoc_motor_UpdateBus(mfoc_motor_t *motor);

/** @brief Disable PWM immediately and latch a nonzero set of fault bits.
 * @param motor Initialized instance. @param faults Cause bits.
 * @return Shutdown status; failure also latches PWM fault.
 * @note Call from the control ISR, or while stopped. Does not reset Core;
 * ClearFault resets it before another start is permitted. */
mfoc_result_t mfoc_motor_Trip(mfoc_motor_t *motor, uint32_t faults);

/** @brief Shut down, reset PI and recover software FAULT to IDLE.
 * @param motor Faulted instance. @return Status; failure retains fault.
 * @note Foreground only. Caller confirms external fault causes are gone.
 * Never automatically restarts; clear the physical latch in the adapter. */
mfoc_result_t mfoc_motor_ClearFault(mfoc_motor_t *motor);

/** @brief Read state and fault bits without interrupt masking.
 * @param motor Valid instance. @param status Valid snapshot destination.
 * @return None. @note Fields are not an atomic pair across ISR updates. */
void mfoc_motor_GetStatus(const mfoc_motor_t *motor,
                          mfoc_motor_status_t *status);

#endif
