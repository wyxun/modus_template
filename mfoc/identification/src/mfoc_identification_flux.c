/**
 * @file mfoc_identification_flux.c
 * @brief flux identification leaf; algorithm remains TODO.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_identification_algorithms.h"

/** @brief Reject unsupported excitation without publishing a result.
 * @param input Synchronous sample.
 * @param model Known motor PU parameters.
 * @param flux Result, unchanged until implemented.
 * @param output Stop request. @return Error until implemented. */
fsm_rt_t mfoc_identification_flux_Step(
    const mfoc_identification_flux_input_t *input,
    const mfoc_motor_params_t *model, mfoc_identification_output_t *output,
    mfoc_scalar_t *flux)
{
    (void)input;
    (void)model;
    (void)flux;
    *output = (mfoc_identification_output_t){.stop = true};
    /* TODO: voltage-model flux estimation after stable closed-loop rotation. */
    return fsm_rt_err;
}
