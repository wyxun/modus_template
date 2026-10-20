/**
 * @file mfoc_transform.h
 * @brief Stateless Clarke/Park transforms extracted from foc_core.
 * @author Codex
 * @date 2026-10-19
 */
#ifndef MFOC_TRANSFORM_H
#define MFOC_TRANSFORM_H
#include "mfoc_types.h"

/** @brief Balanced three-phase Clarke: alpha=u, beta=(v-w)/sqrt(3).
 * @param u Phase U. @param v Phase V. @param w Phase W.
 * @param ab Output. @return NULL or OK. Values must be finite.
 * @note Four arguments keep the existing phase inputs explicit; no zero
 * sequence removal. Fixed retains two product truncations before subtraction. */
mfoc_result_t mfoc_clarke(mfoc_scalar_t u, mfoc_scalar_t v,
                          mfoc_scalar_t w, mfoc_ab_t *ab);
/** @brief Park: d=alpha*cos+beta*sin, q=beta*cos-alpha*sin.
 * @param ab Stationary input. @param sine Cached sine in [-1,1].
 * @param cosine Cached cosine in [-1,1]. @param dq Output.
 * @return NULL or OK. Input values must be finite. */
mfoc_result_t mfoc_park_cached(const mfoc_ab_t *ab, mfoc_scalar_t sine,
                              mfoc_scalar_t cosine, mfoc_dq_t *dq);
/** @brief Inverse Park: alpha=d*cos-q*sin, beta=d*sin+q*cos.
 * @param dq Rotating input, each component in [-1,1].
 * @param sine Cached sine. @param cosine Cached cosine. @param ab Output.
 * @return NULL or OK; retains legacy PU multiplication rounding. */
mfoc_result_t mfoc_ipark_cached(const mfoc_dq_t *dq, mfoc_scalar_t sine,
                               mfoc_scalar_t cosine, mfoc_ab_t *ab);
#endif
