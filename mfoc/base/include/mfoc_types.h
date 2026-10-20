/**
 * @file mfoc_types.h
 * @brief Shared scalar, result and coordinate value types for mfoc.
 * @author Codex
 * @date 2026-10-19
 */
#ifndef MFOC_TYPES_H
#define MFOC_TYPES_H
#include <stdbool.h>
#include <stdint.h>

#if defined(MFOC_NUMERIC_FLOAT) && defined(MFOC_NUMERIC_FIXED)
#error "Select exactly one mfoc numeric backend"
#elif defined(MFOC_NUMERIC_FLOAT)
typedef float mfoc_scalar_t;
#elif defined(MFOC_NUMERIC_FIXED)
typedef int32_t mfoc_scalar_t;
#else
#error "Select MFOC_NUMERIC_FLOAT or MFOC_NUMERIC_FIXED"
#endif

/** @brief Common operation status, independent of numeric implementation. */
typedef enum {
    MFOC_RESULT_OK = 0,
    MFOC_RESULT_NULL,
    MFOC_RESULT_INVALID_ARGUMENT,
    MFOC_RESULT_OUT_OF_RANGE,
    MFOC_RESULT_DIVIDE_BY_ZERO,
    MFOC_RESULT_INVALID_STATE,
    MFOC_RESULT_IO,
    MFOC_RESULT_SAFETY,
} mfoc_result_t;

typedef struct { mfoc_scalar_t qAlpha; mfoc_scalar_t qBeta; } mfoc_ab_t;
typedef struct { mfoc_scalar_t qD; mfoc_scalar_t qQ; } mfoc_dq_t;
/** @brief Three phase values; instance name and API define quantity/units. */
typedef struct {
    mfoc_scalar_t qU;
    mfoc_scalar_t qV;
    mfoc_scalar_t qW;
} mfoc_uvw_t;
#endif
