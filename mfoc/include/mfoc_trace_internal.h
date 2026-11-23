/**
 * @file mfoc_trace_internal.h
 * @brief Optional target-supplied probes for bench firmware.
 * @author Codex
 * @date 2026-11-21
 */
#ifndef MFOC_TRACE_INTERNAL_H
#define MFOC_TRACE_INTERNAL_H

#if defined(MFOC_TRACE_ENABLE) && MFOC_TRACE_ENABLE
#include "mfoc_trace_binding.h"
#else
#define MFOC_TRACE_BEGIN(section) ((void)0)
#define MFOC_TRACE_END(section)   ((void)0)
#define MFOC_TRACE_EVENT(event)   ((void)0)
#endif

#endif
