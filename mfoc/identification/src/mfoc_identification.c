/**
 * @file mfoc_identification.c
 * @brief Foreground experiment sequencing and synchronous sample routing.
 * @author wyxun
 * @date 2026-10-24
 */
#include "mfoc_identification_internal.h"
#include "mfoc_port.h"
#include <stddef.h>

/** @brief Clear experiment state before use.
 * @param id Instance. @return None. */
void mfoc_identification_Init(mfoc_identification_t *id)
{
    *id = (mfoc_identification_t){0};
}

/** @brief Arm an experiment, preserving unmeasured model fields.
 * @param id Instance. @param model Existing model.
 * @return OK or INVALID_STATE if already active. */
mfoc_result_t mfoc_identification_Start(mfoc_identification_t *id,
    const mfoc_motor_params_t *model,
    mfoc_identification_operation_t operation, mfoc_scalar_t bus)
{
    if (id == NULL || model == NULL) { return MFOC_RESULT_NULL; }
    if (bus <= MFOC_ZERO) { return MFOC_RESULT_INVALID_ARGUMENT; }
    if (operation < MFOC_IDENTIFICATION_ELECTRICAL ||
        operation > MFOC_IDENTIFICATION_RESISTANCE) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    if (id->state == MFOC_IDENTIFICATION_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (operation != MFOC_IDENTIFICATION_RESISTANCE && id->timing_mismatch) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    if (operation == MFOC_IDENTIFICATION_FLUX ||
        operation == MFOC_IDENTIFICATION_INERTIA) {
        /* Low-rate builds can still use the standstill experiments. */
        if (!MFOC_ROTATION_RATE_SUPPORTED) { return MFOC_RESULT_OUT_OF_RANGE; }
        if (!(model->rs >= MFOC_SCALAR(MFOC_MOTOR_RS_MIN_PU) &&
              model->rs <= MFOC_SCALAR(MFOC_MOTOR_RS_MAX_PU) &&
              model->ld >= MFOC_SCALAR(MFOC_MOTOR_L_MIN_PU) &&
              model->ld <= MFOC_SCALAR(MFOC_MOTOR_L_MAX_PU) &&
              model->lq >= MFOC_SCALAR(MFOC_MOTOR_L_MIN_PU) &&
              model->lq <= MFOC_SCALAR(MFOC_MOTOR_L_MAX_PU))) {
            return MFOC_RESULT_INVALID_STATE;
        }
        if (operation == MFOC_IDENTIFICATION_INERTIA &&
            !(model->flux > MFOC_ZERO &&
              model->flux <= MFOC_SCALAR(MFOC_MOTOR_FLUX_MAX_PU))) {
            return MFOC_RESULT_INVALID_STATE;
        }
    }
    uint32_t tick_hz = mfoc_port_tick_frequency();
    if (tick_hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    id->current_limit = MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MAX);
    id->speed_limit = MFOC_IDENTIFICATION_SPEED_CAP_PU;
    id->result = *model;
    id->work.align_samples = 0U;
    id->operation = operation;
    id->angle = (mfoc_angle_t){0U};
    id->run_pt = 0U;
    id->stage_started = 0U;
    id->stage_ticks = ((mfoc_tick_t)tick_hz *
        MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS + 999U) / 1000U;
    id->stage_done = false;
    id->stage = MFOC_IDENTIFICATION_STAGE_ALIGN;
    id->state = MFOC_IDENTIFICATION_RUNNING;
    return MFOC_RESULT_OK;
}

/** @brief Cancel without discarding a terminal result/status.
 * @param id Instance. @return None. */
void mfoc_identification_Stop(mfoc_identification_t *id)
{
    id->stage = MFOC_IDENTIFICATION_STAGE_NONE;
    if (id->state == MFOC_IDENTIFICATION_RUNNING) {
        id->state = MFOC_IDENTIFICATION_IDLE;
    }
}

/** @brief Finish a sampled stage and publish its successor to the ISR.
 * @param id Instance with stage_done set. @return True on success. */
static bool _mfoc_identification_AdvanceStage(mfoc_identification_t *id)
{
    switch (id->stage) {
    case MFOC_IDENTIFICATION_STAGE_ALIGN:
        if (id->operation == MFOC_IDENTIFICATION_ELECTRICAL ||
            id->operation == MFOC_IDENTIFICATION_RESISTANCE) {
            id->work.rs = (mfoc_identification_rs_t){0};
            id->stage = MFOC_IDENTIFICATION_STAGE_RS;
        } else {
            mfoc_identification_rotation_Init(&id->work.rotating,
                id->current_limit, id->speed_limit);
            id->stage = MFOC_IDENTIFICATION_STAGE_SPINUP;
        }
        break;
    case MFOC_IDENTIFICATION_STAGE_RS:
        if (mfoc_identification_rs_Finish(&id->work.rs, &id->result.rs) !=
            MFOC_FSM_COMPLETE) { return false; }
        if (id->operation == MFOC_IDENTIFICATION_RESISTANCE) {
            id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        } else {
            id->work.inductance = (mfoc_identification_inductance_series_t){0};
            id->stage = MFOC_IDENTIFICATION_STAGE_LD;
        }
        break;
    case MFOC_IDENTIFICATION_STAGE_LD:
    case MFOC_IDENTIFICATION_STAGE_LQ: {
        bool d_axis = id->stage == MFOC_IDENTIFICATION_STAGE_LD;
        if (mfoc_identification_inductance_SeriesFinish(&id->work.inductance,
                id->result.rs, d_axis ? &id->result.ld : &id->result.lq) !=
            MFOC_FSM_COMPLETE) { return false; }
        if (d_axis) {
            id->work.inductance = (mfoc_identification_inductance_series_t){0};
            id->stage = MFOC_IDENTIFICATION_STAGE_LQ;
        } else {
            id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        }
        break;
    }
    case MFOC_IDENTIFICATION_STAGE_ROTATION_FINISH: {
        bool flux = id->operation == MFOC_IDENTIFICATION_FLUX;
        if (mfoc_identification_rotation_Finish(&id->work.rotating,
                flux ? MFOC_ROTATION_FLUX : MFOC_ROTATION_INERTIA,
                &id->result, flux ? &id->result.flux : &id->result.inertia) !=
            MFOC_FSM_COMPLETE) { return false; }
        id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        break;
    }
    default:
        return false;
    }
    if (id->stage != MFOC_IDENTIFICATION_STAGE_COMPLETE) {
        id->stage_done = false;
    }
    return true;
}

/** @brief Poll the foreground stage sequence without blocking.
 * @param id Instance. @return Ongoing, complete/idle, or error. */
mfoc_fsm_rt_t mfoc_identification_Run(mfoc_identification_t *id)
{
    MFOC_PT_BEGIN(id->run_pt)
    while (id->state == MFOC_IDENTIFICATION_RUNNING &&
           id->stage != MFOC_IDENTIFICATION_STAGE_COMPLETE) {
        id->stage_started = mfoc_port_tick();
        MFOC_PT_WAIT_UNTIL(id->stage_done ||
            id->state != MFOC_IDENTIFICATION_RUNNING ||
            mfoc_port_tick() - id->stage_started >= id->stage_ticks);
        if (id->state != MFOC_IDENTIFICATION_RUNNING) { break; }
        if (!id->stage_done || !_mfoc_identification_AdvanceStage(id)) {
            /* An ISR can preempt Motor before the PWM stop reaches the Port. */
            id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
            id->state = MFOC_IDENTIFICATION_ERROR;
            break;
        }
    }
    if (id->state == MFOC_IDENTIFICATION_RUNNING) {
        id->state = MFOC_IDENTIFICATION_DONE;
    }
    MFOC_PT_END();
    return id->state == MFOC_IDENTIFICATION_ERROR ?
        MFOC_FSM_FAILED : MFOC_FSM_COMPLETE;
}

/** @brief Read lifecycle without exposing algorithm history.
 * @param id Instance. @return Current state. */
mfoc_identification_state_t mfoc_identification_GetStatus(
    const mfoc_identification_t *id)
{
    return id->state;
}

/** @brief Publish the complete model only after successful identification.
 * @param id Instance. @param model Destination.
 * @return OK or INVALID_STATE without changing the destination. */
mfoc_result_t mfoc_identification_GetResult(const mfoc_identification_t *id,
    mfoc_motor_params_t *model)
{
    if (id->state != MFOC_IDENTIFICATION_DONE) {
        return MFOC_RESULT_INVALID_STATE;
    }
    *model = id->result;
    return MFOC_RESULT_OK;
}

/** @brief Check |alpha| + |beta| against the effective experiment limit.
 * Subtract |alpha| first to avoid overflow when adding FIXED magnitudes. */
static bool _mfoc_identification_CurrentAllowed(const mfoc_ab_t *current,
                                                mfoc_scalar_t limit)
{
    if (current == NULL) { return false; }
    const mfoc_scalar_t beta_limit = limit - mfoc_abs(current->qAlpha);
    return current->qBeta >= -beta_limit && current->qBeta <= beta_limit;
}

/** @brief Choose the polarity of a zero-based applied interval.
 * @param d_axis True for d. @param interval Index into the series.
 * @return True for negative voltage; d reverses the prior Rs excitation. */
static inline bool _mfoc_identification_LNegative(bool d_axis,
                                                    uint32_t interval)
{
    const uint32_t half_wave = interval / MFOC_IDENTIFICATION_L_SAMPLES;
    return d_axis != ((half_wave & 1U) != 0U);
}

/** @brief Dispatch one synchronous sample directly to the selected leaf.
 * @param id Active instance. @param stage Stage selected for this frame.
 * @param input Samples. @param output Request.
 * @return Actuator route for the next PWM interval. */
mfoc_identification_control_t mfoc_identification_SampleStage(
    mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output, mfoc_identification_stage_t stage)
{
    *output = (mfoc_identification_output_t){.angle = id->angle};
    mfoc_fsm_rt_t progress = MFOC_FSM_FAILED;
    if (!_mfoc_identification_CurrentAllowed(input->i_ab_k,
                                              id->current_limit)) {
        id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        id->state = MFOC_IDENTIFICATION_ERROR;
        return MFOC_IDENTIFICATION_CONTROL_VOLTAGE;
    }
    switch (stage) {
    case MFOC_IDENTIFICATION_STAGE_ALIGN:
        output->modulation_ab.qAlpha =
            MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW);
        ++id->work.align_samples;
        progress = id->work.align_samples >= MFOC_IDENTIFICATION_SAMPLE_HZ *
            MFOC_IDENTIFICATION_RS_ALIGN_MS / 1000U ?
            MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING;
        break;
    case MFOC_IDENTIFICATION_STAGE_RS: {
        const mfoc_ab_t *u = input->u_ab_previous;
#if MFOC_PWM_PRELOAD_DELAY
        const uint8_t excitation =
            id->work.rs.level == MFOC_IDENTIFICATION_RS_LOW ?
                MFOC_IDENTIFICATION_EXCITATION_RS_LOW :
                MFOC_IDENTIFICATION_EXCITATION_RS_HIGH;
        /* Neither the old level nor a neutral handoff is high/low dwell. */
        if (input->applied_excitation != excitation) { u = NULL; }
#endif
        progress = mfoc_identification_rs_Step(
            &id->work.rs, input->i_ab_k, u);
        /* Current belongs to the old level; apply the next one now. */
        if (progress == MFOC_FSM_ONGOING) {
            const bool low = id->work.rs.level == MFOC_IDENTIFICATION_RS_LOW;
            output->modulation_ab.qAlpha = low ?
                MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW) :
                MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH);
#if MFOC_PWM_PRELOAD_DELAY
            output->excitation = low ? MFOC_IDENTIFICATION_EXCITATION_RS_LOW :
                MFOC_IDENTIFICATION_EXCITATION_RS_HIGH;
#endif
        }
        break;
    }
    case MFOC_IDENTIFICATION_STAGE_LD:
    case MFOC_IDENTIFICATION_STAGE_LQ: {
        const bool d_axis = stage == MFOC_IDENTIFICATION_STAGE_LD;
        mfoc_identification_inductance_series_t *capture =
            &id->work.inductance;
        const uint32_t count = capture->count;
#if MFOC_PWM_PRELOAD_DELAY
        const uint8_t excitation = d_axis ? MFOC_IDENTIFICATION_EXCITATION_LD :
            MFOC_IDENTIFICATION_EXCITATION_LQ;
#endif
        mfoc_scalar_t u = MFOC_ZERO;
        bool negative = false;
        if (count != 0U) {
            /* A lost interval invalidates the volt-second integral. */
            if (input->u_ab_previous == NULL
#if MFOC_PWM_PRELOAD_DELAY
                || input->applied_excitation != excitation
#endif
            ) { break; }
            u = d_axis ? input->u_ab_previous->qAlpha :
                         input->u_ab_previous->qBeta;
            negative = _mfoc_identification_LNegative(d_axis, count - 1U);
            const bool valid = negative ?
                u < MFOC_ZERO && u >= -MFOC_ONE :
                u > MFOC_ZERO && u <= MFOC_ONE;
            if (!valid) { break; }
        }
#if MFOC_PWM_PRELOAD_DELAY
        /* i[k] is the baseline only when this axis first loads at k. */
        progress = count == 0U &&
            input->active_excitation != excitation ? MFOC_FSM_ONGOING :
#else
        progress =
#endif
            mfoc_identification_inductance_SeriesStep(capture,
                d_axis ? input->i_ab_k->qAlpha : input->i_ab_k->qBeta,
                u, negative);
        if (progress == MFOC_FSM_ONGOING
#if MFOC_PWM_PRELOAD_DELAY
            /* The last interval is already active when neutral is queued. */
            && capture->count < MFOC_IDENTIFICATION_L_SERIES_INTERVALS
#endif
        ) {
            uint32_t next = capture->count;
#if !MFOC_PWM_PRELOAD_DELAY
            /* Without preload, the command applies one interval sooner. */
            --next;
#endif
            const mfoc_scalar_t pulse =
                MFOC_SCALAR(MFOC_IDENTIFICATION_L_PULSE);
            const mfoc_scalar_t command =
                _mfoc_identification_LNegative(d_axis, next) ? -pulse : pulse;
            if (d_axis) {
                output->modulation_ab.qAlpha = command;
            } else {
                output->modulation_ab.qBeta = command;
            }
#if MFOC_PWM_PRELOAD_DELAY
            output->excitation = excitation;
#endif
        }
        break;
    }
    case MFOC_IDENTIFICATION_STAGE_SPINUP:
    case MFOC_IDENTIFICATION_STAGE_FLUX:
    case MFOC_IDENTIFICATION_STAGE_INERTIA: {
        mfoc_rotation_stage_t phase = (mfoc_rotation_stage_t)stage;
        progress = mfoc_identification_rotation_Step(&id->work.rotating,
            phase, &id->result, input->i_ab_k, input->u_ab_previous);
        if (progress == MFOC_FSM_ONGOING ||
            (progress == MFOC_FSM_COMPLETE && phase == MFOC_ROTATION_SPINUP)) {
            if (progress == MFOC_FSM_COMPLETE) {
                /* Keep the field turning through the capture boundary. */
                id->stage = id->operation == MFOC_IDENTIFICATION_FLUX ?
                    MFOC_IDENTIFICATION_STAGE_FLUX :
                    MFOC_IDENTIFICATION_STAGE_INERTIA;
            }
            id->angle = id->work.rotating.drive_angle;
            output->angle = id->angle;
            output->i_ref = id->work.rotating.i_ref;
            return MFOC_IDENTIFICATION_CONTROL_VIRTUAL;
        }
        id->stage = MFOC_IDENTIFICATION_STAGE_ROTATION_FINISH;
        break;
    }
    default:
        /* Unsupported preparation never reports completion or energizes. */
        break;
    }
    if (progress == MFOC_FSM_FAILED) {
        id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        id->state = MFOC_IDENTIFICATION_ERROR;
    } else if (progress == MFOC_FSM_COMPLETE) {
        id->stage_done = true;
    } else {
        /* Leaf retains sampling history until a terminal result. */
    }
    return MFOC_IDENTIFICATION_CONTROL_VOLTAGE;
}

/** @brief Sample the currently published stage for standalone callers.
 * @param id Instance. @param input Samples. @param output Request. */
mfoc_identification_control_t mfoc_identification_Step(mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output)
{
    const mfoc_identification_frame_t frame = mfoc_identification_BeginFrame(id);
    return mfoc_identification_StepFrame(id, frame, input, output);
}
