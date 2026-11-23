/**
 * @file mfoc_trace_binding.h
 * @brief Candidate-only timing marks. The compiler removes inactive sections.
 * @author Codex
 * @date 2026-11-21
 */
#ifndef G431_MFOC_TRACE_BINDING_H
#define G431_MFOC_TRACE_BINDING_H

#include "stm32g4xx.h"
#include <stdbool.h>

typedef enum {
    P10_CURRENT, P10_FEEDBACK, P10_POSITION, P10_SMO, P10_FLUX, P10_EKF,
    P10_HFI, P10_SPEED, P10_IDENTIFICATION, P10_CORE, P10_PWM,
    P10_SECTION_COUNT
} mfoc_p10_section_t;

enum {
    P10_SPEED_PI = 1U, P10_SMO_WINDOW = 2U, P10_HFI_EIGHTH_OK = 4U,
    P10_HFI_EIGHTH_WEAK = 8U, P10_LIMITED = 16U
};

extern uint32_t g_p10_events;
#if MFOC_RUNTIME_BUS
extern bool g_p10_bus_applied;
#endif
#define MFOC_TRACE_EVENT(event) (g_p10_events |= P10_##event)

#if MFOC_P10_SEGMENTS
extern uint32_t g_p10_start[P10_SECTION_COUNT];
extern uint32_t g_p10_end[P10_SECTION_COUNT];
#define MFOC_TRACE_BEGIN(section) \
    (g_p10_start[P10_##section] = DWT->CYCCNT)
#define MFOC_TRACE_END(section) \
    (g_p10_end[P10_##section] = DWT->CYCCNT)
#else
#define MFOC_TRACE_BEGIN(section) ((void)0)
#define MFOC_TRACE_END(section)   ((void)0)
#endif
#endif
