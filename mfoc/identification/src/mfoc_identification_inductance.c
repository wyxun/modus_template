/**
 * @file mfoc_identification_inductance.c
 * @brief inductance identification leaf; algorithm remains TODO.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_identification_algorithms.h"

/** @brief Reject unsupported excitation without publishing a result.
 * @param input Synchronous sample.
 * @param axis Selected d/q axis.
 * @param inductance Result, unchanged until implemented.
 * @param output Stop request. @return Error until implemented. */
fsm_rt_t mfoc_identification_inductance_Step(
    const mfoc_identification_fixed_input_t *input,
    mfoc_identification_axis_t axis, mfoc_identification_output_t *output,
    mfoc_scalar_t *inductance)
{
    (void)input;
    (void)axis;
    (void)inductance;
    *output = (mfoc_identification_output_t){.stop = true};
    /* TODO: axis-specific perturbation and incremental Ld/Lq estimation. */
    return fsm_rt_err;
}
