/**
 * @file mfoc_identification_inertia.c
 * @brief inertia identification leaf; algorithm remains TODO.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_identification_algorithms.h"

/** @brief Reject unsupported excitation without publishing a result.
 * @param input Synchronous sample.
 * @param model Known motor PU parameters.
 * @param inertia Result, unchanged until implemented.
 * @param output Stop request. @return Error until implemented. */
fsm_rt_t mfoc_identification_inertia_Step(
    const mfoc_identification_inertia_input_t *input,
    const mfoc_motor_params_t *model, mfoc_identification_output_t *output,
    mfoc_scalar_t *inertia)
{
    (void)input;
    (void)model;
    (void)inertia;
    *output = (mfoc_identification_output_t){.stop = true};
    /* TODO: torque/acceleration estimation and load-torque handling. */
    return fsm_rt_err;
}
