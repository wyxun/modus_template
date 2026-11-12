/**
 * @file mfoc_identification.c
 * @brief Foreground stages and synchronous standstill Rs identification.
 * @author wyxun
 * @date 2026-10-24
 */
#include "mfoc_identification.h"
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
    if (operation != MFOC_IDENTIFICATION_RESISTANCE) {
        return MFOC_RESULT_INVALID_STATE;
    }
    uint32_t tick_hz = mfoc_port_tick_frequency();
    if (tick_hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    id->result = *model;
    id->bus = bus;
    id->work = (mfoc_identification_work_t){0};
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
        id->stage = MFOC_IDENTIFICATION_STAGE_RS;
        break;
    case MFOC_IDENTIFICATION_STAGE_RS:
        if (mfoc_identification_rs_Finish(&id->work.rs, id->bus,
                &id->result.rs) != MFOC_FSM_COMPLETE) { return false; }
        id->stage = id->operation == MFOC_IDENTIFICATION_RESISTANCE ?
            MFOC_IDENTIFICATION_STAGE_COMPLETE : MFOC_IDENTIFICATION_STAGE_LD;
        break;
    case MFOC_IDENTIFICATION_STAGE_LD:
        id->stage = MFOC_IDENTIFICATION_STAGE_LQ;
        break;
    case MFOC_IDENTIFICATION_STAGE_SPINUP:
        id->stage = id->operation == MFOC_IDENTIFICATION_FLUX ?
            MFOC_IDENTIFICATION_STAGE_FLUX : MFOC_IDENTIFICATION_STAGE_INERTIA;
        break;
    case MFOC_IDENTIFICATION_STAGE_FLUX:
        if (mfoc_identification_flux_Finish(&id->work.flux, &id->result,
                &id->result.flux) != MFOC_FSM_COMPLETE) { return false; }
        id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        break;
    case MFOC_IDENTIFICATION_STAGE_LQ:
    case MFOC_IDENTIFICATION_STAGE_INERTIA:
        id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        break;
    default:
        return false;
    }
    if (id->stage != MFOC_IDENTIFICATION_STAGE_COMPLETE) {
        id->work = (mfoc_identification_work_t){0};
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

/** @brief Establish a bounded fixed field before electrical measurement.
 * @param id Active ALIGN stage. @param input Current sample.
 * @param output Fixed alpha modulation. @return Stage progress. */
static mfoc_fsm_rt_t _mfoc_identification_Align(
    mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output)
{
    const mfoc_ab_t *current = input->i_ab_k;
    if (current == NULL || mfoc_abs(current->qAlpha) >
            MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MAX) ||
        mfoc_abs(current->qBeta) >
            MFOC_SCALAR(MFOC_IDENTIFICATION_RS_CURRENT_MAX) -
                mfoc_abs(current->qAlpha)) {
        return MFOC_FSM_FAILED;
    }
    output->modulation_ab.qAlpha=
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW);
    id->work.align_samples++;
    return id->work.align_samples >= MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_ALIGN_MS / 1000U ?
        MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING;
}

/** @brief Dispatch one synchronous sample directly to the selected leaf.
 * @param id Active instance. @param input Samples. @param output Request.
 * @return None. */
void mfoc_identification_Step(mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output)
{
    *output = (mfoc_identification_output_t){.angle = id->angle};
    if (id->state != MFOC_IDENTIFICATION_RUNNING) { return; }
    if (id->stage_done) { return; }
    mfoc_fsm_rt_t progress = MFOC_FSM_FAILED;
    switch (id->stage) {
    case MFOC_IDENTIFICATION_STAGE_ALIGN:
        progress = _mfoc_identification_Align(id, input, output);
        break;
    case MFOC_IDENTIFICATION_STAGE_RS:
        progress = mfoc_identification_rs_Step(
            &id->work.rs, input->i_ab_k);
        /* Current belongs to the old level; apply the next one now. */
        if (progress != MFOC_FSM_COMPLETE) {
            output->modulation_ab.qAlpha=
                id->work.rs.level == MFOC_IDENTIFICATION_RS_LOW ?
                MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW) :
                MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH);
        }
        break;
    case MFOC_IDENTIFICATION_STAGE_LD:
    case MFOC_IDENTIFICATION_STAGE_LQ:
        /* TODO: schedule pulses and supply aligned axis samples. */
        break;
    case MFOC_IDENTIFICATION_STAGE_FLUX:
        /* TODO: virtual-dq flux estimator consumes i_ab/u_ab samples. */
        break;
    case MFOC_IDENTIFICATION_STAGE_INERTIA:
        /* TODO: estimate speed/acceleration within the identification module. */
        break;
    case MFOC_IDENTIFICATION_STAGE_SPINUP:
        /* TODO: create the virtual rotation command inside identification. */
        break;
    default:
        /* Unsupported preparation never reports completion or energizes. */
        break;
    }
    if (progress == MFOC_FSM_FAILED) {
        id->state = MFOC_IDENTIFICATION_ERROR;
    } else if (progress == MFOC_FSM_COMPLETE) {
        id->stage_done = true;
    } else {
        /* Leaf retains sampling history until a terminal result. */
    }
    if (progress != MFOC_FSM_FAILED) { id->angle = output->angle; }
}
