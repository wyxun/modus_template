/**
 * @file mfoc_identification.c
 * @brief Identification management; excitation and estimation remain TODO.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_identification.h"
#include "perfc_task_pt.h"

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
    mfoc_identification_operation_t operation)
{
    if (id->state == MFOC_IDENTIFICATION_RUNNING) {
        return MFOC_RESULT_INVALID_STATE;
    }
    id->result = *model;
    id->operation = operation;
    id->fixed_angle = (mfoc_angle_t){0U};
    id->run_pt = 0U;
    id->stage_deadline = 0;
    id->stage_done = false;
    id->stage = operation == MFOC_IDENTIFICATION_ELECTRICAL ?
        MFOC_IDENTIFICATION_STAGE_ALIGN : MFOC_IDENTIFICATION_STAGE_SPINUP;
    id->state = MFOC_IDENTIFICATION_RUNNING;
    return MFOC_RESULT_OK;
}

/** @brief Choose feedback preparation before the leaf receives its sample.
 * @param id Instance. @return Inactive, fixed frame, or valid feedback. */
mfoc_identification_control_t mfoc_identification_GetControl(
    const mfoc_identification_t *id)
{
    if (id->stage == MFOC_IDENTIFICATION_STAGE_NONE) {
        return MFOC_IDENTIFICATION_CONTROL_INACTIVE;
    }
    return id->state == MFOC_IDENTIFICATION_RUNNING &&
        id->operation != MFOC_IDENTIFICATION_ELECTRICAL ?
        MFOC_IDENTIFICATION_CONTROL_FEEDBACK :
        MFOC_IDENTIFICATION_CONTROL_FIXED;
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

#if defined(__GNUC__) && !defined(__clang__)
/* perfc PT intentionally falls through into its generated resume case. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#endif

/** @brief Wait for synchronous stages and publish the next experiment.
 * @param id Instance. @return Ongoing, complete, or error. */
fsm_rt_t mfoc_identification_Run(mfoc_identification_t *id)
{
    if (id->state == MFOC_IDENTIFICATION_ERROR) { return fsm_rt_err; }
    if (id->state != MFOC_IDENTIFICATION_RUNNING) { return fsm_rt_cpl; }

    PERFC_PT_BEGIN(id->run_pt)
    while (id->stage != MFOC_IDENTIFICATION_STAGE_COMPLETE) {
        id->stage_deadline = 0;
        (void)perfc_is_time_out_ms(MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS,
                                  &id->stage_deadline, false);
        PERFC_PT_WAIT_UNTIL(id->stage_done ||
            id->state != MFOC_IDENTIFICATION_RUNNING ||
            perfc_is_time_out_ms(MFOC_IDENTIFICATION_STAGE_TIMEOUT_MS,
                                &id->stage_deadline, false))
        if (id->state != MFOC_IDENTIFICATION_RUNNING) {
            return id->state == MFOC_IDENTIFICATION_ERROR ?
                fsm_rt_err : fsm_rt_cpl;
        }
        if (!id->stage_done) {
            id->state = MFOC_IDENTIFICATION_ERROR;
            return fsm_rt_err;
        }
        /* TODO: prepare the next experiment while Step is paused by
         * stage_done. Rotating preparation must establish stable speed. */
        if (id->stage == MFOC_IDENTIFICATION_STAGE_SPINUP) {
            id->stage = id->operation == MFOC_IDENTIFICATION_FLUX ?
                MFOC_IDENTIFICATION_STAGE_FLUX :
                MFOC_IDENTIFICATION_STAGE_INERTIA;
        } else if (id->stage == MFOC_IDENTIFICATION_STAGE_LQ ||
                   id->stage == MFOC_IDENTIFICATION_STAGE_FLUX ||
                   id->stage == MFOC_IDENTIFICATION_STAGE_INERTIA) {
            id->stage = MFOC_IDENTIFICATION_STAGE_COMPLETE;
        } else {
            id->stage = (mfoc_identification_stage_t)(id->stage + 1);
        }
        if (id->stage != MFOC_IDENTIFICATION_STAGE_COMPLETE) {
            id->stage_done = false;
        }
    }
    id->state = MFOC_IDENTIFICATION_DONE;
    PERFC_PT_END()
    return fsm_rt_cpl;
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

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

/** @brief Commit a completed model after dependent coefficients succeed.
 * @param id Completed experiment. @param model Motor-owned parameters.
 * @param feedback Stopped feedback. @return OK or rejected result. */
mfoc_result_t mfoc_identification_ApplyResult(const mfoc_identification_t *id,
    mfoc_motor_params_t *model, mfoc_feedback_t *feedback)
{
    if (id->state != MFOC_IDENTIFICATION_DONE) {
        return MFOC_RESULT_INVALID_STATE;
    }
    if (id->result.pole_pairs != model->pole_pairs) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_result_t rc = mfoc_feedback_SetMotorParamModel(feedback, &id->result);
    if (rc == MFOC_RESULT_OK) { *model = id->result; }
    return rc;
}

/** @brief Dispatch one synchronous sample directly to the selected leaf.
 * @param id Active instance. @param input Samples. @param output Request.
 * @return None. */
void mfoc_identification_Step(mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output)
{
    *output = (mfoc_identification_output_t){.angle = id->fixed_angle};
    if (id->state != MFOC_IDENTIFICATION_RUNNING) {
        output->stop = true;
        return;
    }
    if (id->stage_done) { return; }
    fsm_rt_t progress = fsm_rt_err;
    if (id->stage == MFOC_IDENTIFICATION_STAGE_RS ||
        id->stage == MFOC_IDENTIFICATION_STAGE_LD ||
        id->stage == MFOC_IDENTIFICATION_STAGE_LQ) {
        mfoc_identification_fixed_input_t sample = {
            .i_ab_k = input->i_ab_k,
            .previous_core_output = input->previous_core_output,
            .angle = id->fixed_angle};
        if (id->stage == MFOC_IDENTIFICATION_STAGE_RS) {
            progress = mfoc_identification_rs_Step(
                &sample, output, &id->result.rs);
        } else {
            bool d_axis = id->stage == MFOC_IDENTIFICATION_STAGE_LD;
            progress = mfoc_identification_inductance_Step(&sample,
                d_axis ? MFOC_IDENTIFICATION_AXIS_D :
                    MFOC_IDENTIFICATION_AXIS_Q, output,
                d_axis ? &id->result.ld : &id->result.lq);
        }
    } else if (id->stage == MFOC_IDENTIFICATION_STAGE_FLUX) {
        mfoc_identification_flux_input_t sample = {
            .i_ab_k = input->i_ab_k,
            .previous_core_output = input->previous_core_output,
            .angle = input->feedback->angle, .speed = input->feedback->speed};
        progress = mfoc_identification_flux_Step(
            &sample, &id->result, output, &id->result.flux);
    } else if (id->stage == MFOC_IDENTIFICATION_STAGE_INERTIA) {
        mfoc_identification_inertia_input_t sample = {
            .i_ab_k = input->i_ab_k, .angle = input->feedback->angle,
            .speed = input->feedback->speed};
        progress = mfoc_identification_inertia_Step(
            &sample, &id->result, output, &id->result.inertia);
    } else {
        /* TODO: fixed-field alignment / closed-loop spinup requests.
         * Unsupported preparation never reports completion or energizes. */
        output->stop = true;
    }
    if (progress == fsm_rt_err || output->stop) {
        output->stop = true;
        id->state = MFOC_IDENTIFICATION_ERROR;
    } else if (progress == fsm_rt_cpl) {
        id->stage_done = true;
    } else {
        /* Leaf retains sampling history until a terminal result. */
    }
    if (!output->stop && id->operation == MFOC_IDENTIFICATION_ELECTRICAL) {
        id->fixed_angle = output->angle;
    }
}
