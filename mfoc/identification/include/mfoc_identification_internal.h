/**
 * @file mfoc_identification_internal.h
 * @brief ISR dispatch shared by Motor and the standalone Step entry.
 * @author Codex
 * @date 2026-11-15
 */
#ifndef MFOC_IDENTIFICATION_INTERNAL_H
#define MFOC_IDENTIFICATION_INTERNAL_H
#include "mfoc_identification.h"

/** @brief Control for Motor and the stage for this frame. Identification
 * uses the matching stage for dispatch without a second volatile read. */
typedef struct {
    mfoc_identification_control_t control;
    mfoc_identification_stage_t stage;
} mfoc_identification_frame_t;

/** @brief Select the actuator path for the published experiment stage.
 * @param id Instance. @return Control and its sampling snapshot. */
static inline mfoc_identification_frame_t mfoc_identification_BeginFrame(
    const mfoc_identification_t *id)
{
    const mfoc_identification_stage_t stage = id->stage;
    return (mfoc_identification_frame_t){
        .control = (mfoc_identification_control_t)(stage &
            MFOC_IDENTIFICATION_CONTROL_MASK),
        .stage = stage};
}

/** @brief Ownership persists across foreground handoff until Stop.
 * @param id Instance. @return Whether identification owns the output. */
static inline bool mfoc_identification_OwnsControl(
    const mfoc_identification_t *id)
{
    return id->stage != MFOC_IDENTIFICATION_STAGE_NONE;
}

/** @brief The foreground can publish new work only between sampling ISRs.
 * @param id Instance. @return True if the current stage accepts samples. */
static inline bool mfoc_identification_IsSampling(
    const mfoc_identification_t *id)
{
    return id->state == MFOC_IDENTIFICATION_RUNNING && !id->stage_done;
}

/** @brief Dispatch a ready stage, from the published stage snapshot.
 * @param id Sampling instance. @param stage Published stage.
 * @param input Samples. @param output Request for this frame.
 * @return Effective route; terminal rotation submits neutral direct PWM. */
mfoc_identification_control_t mfoc_identification_SampleStage(
    mfoc_identification_t *id,
    const mfoc_identification_input_t *input,
    mfoc_identification_output_t *output, mfoc_identification_stage_t stage);

/** @brief Sample or hold the published command using one stage snapshot.
 * @return Route to apply, including direct neutral on terminal rotation. */
static inline mfoc_identification_control_t mfoc_identification_StepFrame(
    mfoc_identification_t *id, mfoc_identification_frame_t frame,
    const mfoc_identification_input_t *input, mfoc_identification_output_t *output)
{
    if (mfoc_identification_IsSampling(id)) {
        return mfoc_identification_SampleStage(id, input, output, frame.stage);
    }
    *output = (mfoc_identification_output_t){.angle = id->angle};
    if (frame.control == MFOC_IDENTIFICATION_CONTROL_VIRTUAL) {
        output->i_ref = id->work.rotating.i_ref;
    }
    return frame.control;
}
#endif
