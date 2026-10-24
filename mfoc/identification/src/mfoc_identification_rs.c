/**
 * @file mfoc_identification_rs.c
 * @brief rs identification leaf; algorithm remains TODO.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_identification_algorithms.h"

/** @brief Reject unsupported excitation without publishing a result.
 * @param input Synchronous sample.
 * @param rs Result, unchanged until implemented.
 * @param output Stop request. @return Error until implemented. */
fsm_rt_t mfoc_identification_rs_Step(
    const mfoc_identification_fixed_input_t *input,
    mfoc_identification_output_t *output, mfoc_scalar_t *rs)
{
    (void)input;
    (void)rs;
    *output = (mfoc_identification_output_t){.stop = true};
    /* TODO: voltage/current accumulation and Rs estimation. */
    return fsm_rt_err;
}
