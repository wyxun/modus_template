/**
 * @file mfoc_identification.h
 * @brief Offline identification lifecycle and synchronous control requests.
 * @author wyxun
 * @date 2026-10-24
 */
#ifndef MFOC_IDENTIFICATION_H
#define MFOC_IDENTIFICATION_H
#include "mfoc_identification_rotation.h"
#include "mfoc_pt.h"

#ifndef MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS
#define MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS 5000U
#endif

typedef enum {
    MFOC_IDENTIFICATION_IDLE = 0,
    MFOC_IDENTIFICATION_RUNNING,
    MFOC_IDENTIFICATION_DONE,
    MFOC_IDENTIFICATION_ERROR,
} mfoc_identification_state_t;

/** @brief Select an experiment without retaining an initialization cfg. */
typedef enum {
    MFOC_IDENTIFICATION_ELECTRICAL = 0, /* Alignment, Rs, Ld, Lq. */
    MFOC_IDENTIFICATION_FLUX,
    MFOC_IDENTIFICATION_INERTIA,
    MFOC_IDENTIFICATION_RESISTANCE, /* Alignment and Rs only. */
} mfoc_identification_operation_t;

/** @brief Foreground sequence, including preparation for rotating tests. */
typedef enum {
    MFOC_IDENTIFICATION_STAGE_NONE = 0,
    /* High control bits select the ISR path; low bits identify the stage. */
    MFOC_IDENTIFICATION_STAGE_ALIGN =
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE | 1U,
    MFOC_IDENTIFICATION_STAGE_RS =
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE | 2U,
    MFOC_IDENTIFICATION_STAGE_LD =
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE | 3U,
    MFOC_IDENTIFICATION_STAGE_LQ =
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE | 4U,
    MFOC_IDENTIFICATION_STAGE_SPINUP =
        MFOC_ROTATION_SPINUP,
    MFOC_IDENTIFICATION_STAGE_FLUX =
        MFOC_ROTATION_FLUX,
    MFOC_IDENTIFICATION_STAGE_INERTIA =
        MFOC_ROTATION_INERTIA,
    MFOC_IDENTIFICATION_STAGE_ROTATION_FINISH =
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE | 9U,
    MFOC_IDENTIFICATION_STAGE_COMPLETE =
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE | 8U,
} mfoc_identification_stage_t;

/** @brief Synchronous current and the preceding PWM interval's model voltage. */
typedef struct {
    const mfoc_ab_t *i_ab_k; /**< Sample at k, alpha-beta I/Ibase. */
    /** Model voltage over [k-1,k), alpha-beta V/Vbase; NULL without history. */
    const mfoc_ab_t *u_ab_previous;
} mfoc_identification_input_t;

/** @brief Mutually exclusive workspaces; unused algorithms add no state. */
typedef union {
    uint32_t align_samples;
    mfoc_identification_rs_t rs;
    mfoc_identification_inductance_t inductance;
    mfoc_identification_rotation_t rotating;
} mfoc_identification_work_t;

/** @brief Instance-owned experiment and result; no Motor or hardware pointer.
 * Start/Stop/GetResult are serialized with the ISR by the caller.
 * A completed result preserves all model fields not measured by this run. */
typedef struct {
    mfoc_motor_params_t result;
    mfoc_identification_work_t work;
    mfoc_scalar_t current_limit; /* Effective experiment limit, I/Ibase. */
    mfoc_scalar_t speed_limit; /* Electrical speed magnitude, PU. */
    mfoc_identification_operation_t operation;
    mfoc_angle_t angle; /* Last identification-owned virtual angle. */
    volatile mfoc_identification_state_t state;
    volatile mfoc_identification_stage_t stage;
    volatile bool stage_done; /* ISR publishes completion after sample writes. */
    bool timing_mismatch; /* Configured Motor uses another timing profile. */
    mfoc_pt_t run_pt;          /* Foreground continuation across polls. */
    mfoc_tick_t stage_started; /* Foreground Port timestamp. */
    mfoc_tick_t stage_ticks;   /* Stage timeout in Port ticks. */
} mfoc_identification_t;

/** @brief Initialize before ISR use.
 * @param id Instance. @return None. */
void mfoc_identification_Init(mfoc_identification_t *id);

/** @brief Arm an experiment; does not enable PWM or start Motor.
 * @param id Instance. @param model Current PU model. @param operation Test.
 * @param bus Positive Vdc/Vbase at startup. Each sample uses its own interval.
 * @return OK when armed; INVALID_STATE for active or missing model prerequisites.
 * @note Call while Motor is IDLE; the caller owns Motor startup/shutdown.
 * ELECTRICAL runs alignment, Rs and both inductance pulses with the rotor
 * held at the aligned position. FLUX and INERTIA run with a free shaft.
 * They estimate rotation from phase current and preceding PWM voltage.
 * FLUX needs known Rs/Ld/Lq; INERTIA also needs known magnet flux. */
mfoc_result_t mfoc_identification_Start(mfoc_identification_t *id,
    const mfoc_motor_params_t *model,
    mfoc_identification_operation_t operation, mfoc_scalar_t bus);

/** @brief Cancel an active experiment; preserve completed/error status.
 * @param id Instance. @return None.
 * @note Does not disable hardware. Stop Motor before cancelling in foreground. */
void mfoc_identification_Stop(mfoc_identification_t *id);

/** @brief Advance the selected experiment without blocking.
 * @param id Instance. @return Ongoing, complete/idle, or error.
 * @note Foreground only. Spinup and rotating capture share one deadline.
 * An ISR completion already published at the deadline is consumed first.
 * New stage data is prepared while stage_done is true; clearing it publishes
 * the next stage to Step. Unsupported stages report ERROR and stop. */
mfoc_fsm_rt_t mfoc_identification_Run(
    mfoc_identification_t *id);

/** @brief Read the experiment lifecycle.
 * @param id Instance. @return Current state. */
mfoc_identification_state_t mfoc_identification_GetStatus(
    const mfoc_identification_t *id);

/** @brief Copy only a successfully completed model result.
 * @param id Instance. @param model Destination; unchanged unless DONE.
 * @return OK or INVALID_STATE; read after Motor stops. */
mfoc_result_t mfoc_identification_GetResult(const mfoc_identification_t *id,
    mfoc_motor_params_t *model);

/** @brief Consume i[k] and the preceding submitted PWM interval.
 * @param id Active instance. @param input Current/prior-interval samples.
 * @param output Next experiment command.
 * @return Effective actuator route, including neutral voltage on failure.
 * @note ISR only. Alignment and Rs can start with no voltage history.
 * If |i_alpha| + |i_beta| exceeds MFOC_IDENTIFICATION_RS_CURRENT_MAX,
 * the experiment requests zero excitation and fails. Motor may lower this limit. */
mfoc_identification_control_t mfoc_identification_Step(
    mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output);
#endif
