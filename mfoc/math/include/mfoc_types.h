/**
 * @file mfoc_types.h
 * @brief Shared scalar, result and coordinate value types for mfoc.
 * @author wyxun
 * @date 2026-10-19
 */
#ifndef MFOC_TYPES_H
#define MFOC_TYPES_H
#include <stdbool.h>
#include <stdint.h>

/* Set MFOC_USE_PERFC_FSM=1 for every translation unit in a perfc build. */
#if defined(MFOC_USE_PERFC_FSM) && MFOC_USE_PERFC_FSM
#include "perf_counter.h"
typedef fsm_rt_t mfoc_fsm_rt_t;
#define MFOC_FSM_FAILED            fsm_rt_err
#define MFOC_FSM_COMPLETE          fsm_rt_cpl
#define MFOC_FSM_ONGOING           fsm_rt_on_going
#define MFOC_FSM_WAIT_FOR_OBJECT   fsm_rt_wait_for_obj
#define MFOC_FSM_ASYNC             fsm_rt_asyn
#define MFOC_FSM_WAIT_FOR_RESOURCE fsm_rt_wait_for_res
#else
/** @brief Standalone return codes for nonblocking operations. */
typedef enum {
    MFOC_FSM_FAILED = -1,  /**< Operation ended in error. */
    MFOC_FSM_COMPLETE = 0, /**< No further polling is needed. */
    MFOC_FSM_ONGOING = 1,  /**< Poll again to continue. */
    MFOC_FSM_WAIT_FOR_OBJECT = 2,   /**< Await an object's readiness. */
    MFOC_FSM_ASYNC = 3,             /**< Check the asynchronous result later. */
    MFOC_FSM_WAIT_FOR_RESOURCE = 4, /**< Retry when a resource is available. */
} mfoc_fsm_rt_t;
#endif

/** @brief Monotonic raw ticks supplied by the statically bound Port. */
typedef uint64_t mfoc_tick_t;

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
