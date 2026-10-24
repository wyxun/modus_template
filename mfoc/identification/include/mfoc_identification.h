/**
 * @file mfoc_identification.h
 * @brief Offline identification lifecycle and synchronous control requests.
 * @author Codex
 * @date 2026-10-24
 */
#ifndef MFOC_IDENTIFICATION_H
#define MFOC_IDENTIFICATION_H
#include "mfoc_identification_algorithms.h"
#include "mfoc_feedback.h"
#include "perf_counter.h"

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
} mfoc_identification_operation_t;

/** @brief Foreground sequence, including preparation for rotating tests. */
typedef enum {
    MFOC_IDENTIFICATION_STAGE_NONE = 0,
    MFOC_IDENTIFICATION_STAGE_ALIGN,
    MFOC_IDENTIFICATION_STAGE_RS,
    MFOC_IDENTIFICATION_STAGE_LD,
    MFOC_IDENTIFICATION_STAGE_LQ,
    MFOC_IDENTIFICATION_STAGE_SPINUP,
    MFOC_IDENTIFICATION_STAGE_FLUX,
    MFOC_IDENTIFICATION_STAGE_INERTIA,
    MFOC_IDENTIFICATION_STAGE_COMPLETE,
} mfoc_identification_stage_t;

/** @brief Borrowed Motor samples; feedback is supplied for rotating tests. */
typedef struct {
    const mfoc_ab_t *i_ab_k;
    const mfoc_core_output_t *previous_core_output;
    const mfoc_feedback_output_t *feedback; /* Valid iff CONTROL_FEEDBACK. */
} mfoc_identification_input_t;

/** @brief Instance-owned experiment and result; no Motor or hardware pointer.
 * Start/Stop/GetResult are serialized with the ISR by the caller.
 * A completed result preserves all model fields not measured by this run. */
typedef struct {
    mfoc_motor_params_t result;
    mfoc_identification_operation_t operation;
    mfoc_angle_t fixed_angle; /* Last submitted fixed-frame angle. */
    volatile mfoc_identification_state_t state;
    volatile mfoc_identification_stage_t stage;
    volatile bool stage_done; /* ISR publishes completion after sample writes. */
    uint8_t run_pt;           /* Foreground PT cursor. */
    int64_t stage_deadline;   /* Foreground perfc timeout slot. */
} mfoc_identification_t;

/** @brief Initialize before ISR use.
 * @param id Instance. @return None. */
void mfoc_identification_Init(mfoc_identification_t *id);

/** @brief Arm an experiment; does not enable PWM or start Motor.
 * @param id Instance. @param model Current PU model. @param operation Test.
 * @return OK when armed, INVALID_STATE when already running.
 * @note Call while Motor is IDLE, then use its existing Start API.
 * Algorithms are TODO: the first Step reports ERROR and requests stop. */
mfoc_result_t mfoc_identification_Start(mfoc_identification_t *id,
    const mfoc_motor_params_t *model,
    mfoc_identification_operation_t operation);

/** @brief Declare current control/feedback requirements before sampling.
 * @param id Instance. @return Inactive, fixed frame, or normal feedback.
 * @note Motor evaluates once per ISR; terminal stages request fixed stop. */
mfoc_identification_control_t mfoc_identification_GetControl(
    const mfoc_identification_t *id);

/** @brief Cancel an active experiment; preserve completed/error status.
 * @param id Instance. @return None.
 * @note Does not disable hardware. Stop Motor before cancelling in foreground. */
void mfoc_identification_Stop(mfoc_identification_t *id);

/** @brief Advance the selected experiment without blocking.
 * @param id Instance. @return Ongoing, complete/idle, or error.
 * @note Foreground only. Each stage waits for ISR completion with a timeout.
 * New stage data is prepared while stage_done is true; clearing it publishes
 * the next stage to Step. Algorithms remain TODO and cannot report success. */
fsm_rt_t mfoc_identification_Run(mfoc_identification_t *id);

/** @brief Read the experiment lifecycle.
 * @param id Instance. @return Current state. */
mfoc_identification_state_t mfoc_identification_GetStatus(
    const mfoc_identification_t *id);

/** @brief Copy only a successfully completed model result.
 * @param id Instance. @param model Destination; unchanged unless DONE.
 * @return OK or INVALID_STATE; read after Motor stops. */
mfoc_result_t mfoc_identification_GetResult(const mfoc_identification_t *id,
    mfoc_motor_params_t *model);

/** @brief Refresh observers, then commit a successfully identified model.
 * @param id Completed experiment. @param model Motor-owned model destination.
 * @param feedback Dependent feedback instance.
 * @return OK or rejection; failure preserves model and observer state.
 * @note Foreground with Motor stopped; caller serializes against the ISR.
 * PU bases/pole pairs stay fixed. PI tuning remains independently configured. */
mfoc_result_t mfoc_identification_ApplyResult(const mfoc_identification_t *id,
    mfoc_motor_params_t *model, mfoc_feedback_t *feedback);

/** @brief Consume i[k] and the preceding submitted PWM interval.
 * @param id Active instance. @param input Current/prior-interval samples.
 * @param output Next experiment command. @return None.
 * @note ISR only. Duty/bus describe model voltage, not measured terminal
 * voltage. Inputs are borrowed for this call; no retained sample pointers.
 * Caller gates once using GetControl; FEEDBACK requires valid feedback. */
void mfoc_identification_Step(mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output);
#endif
