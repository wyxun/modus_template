/**
 * @file mfoc_pt.h
 * @brief Nonblocking waits for MFOC foreground state machines.
 * @author Codex
 * @date 2026-11-12
 */
#ifndef MFOC_PT_H
#define MFOC_PT_H
#include "mfoc_types.h"

#if defined(MFOC_USE_PERFC_FSM) && MFOC_USE_PERFC_FSM
#include "perfc_task_pt.h"
typedef uint8_t mfoc_pt_t;
#define MFOC_PT_BEGIN(cursor)     PERFC_PT_BEGIN(cursor)
#if defined(__GNUC__) && !defined(__clang__)
/* perfc deliberately falls through into its generated resume case. */
#define MFOC_PT_WAIT_UNTIL(ready)                                      \
    _Pragma("GCC diagnostic push")                                     \
    _Pragma("GCC diagnostic ignored \"-Wimplicit-fallthrough\"")        \
    PERFC_PT_WAIT_UNTIL((ready), (void)0)                              \
    _Pragma("GCC diagnostic pop")
#else
#define MFOC_PT_WAIT_UNTIL(ready) PERFC_PT_WAIT_UNTIL((ready), (void)0)
#endif
#define MFOC_PT_END()             PERFC_PT_END()
#else
/* One WAIT per source line. A zero cursor starts at the beginning.
 * Values needed after a wait belong to the instance, not the stack. */
typedef uint32_t mfoc_pt_t;
#define MFOC_PT_BEGIN(cursor)                           \
    mfoc_pt_t *const mfoc_pt_cursor = &(cursor);        \
    switch (*mfoc_pt_cursor) { case 0:

/* Resume at the bottom of the loop to recheck ready immediately. */
#define MFOC_PT_WAIT_UNTIL(ready)                       \
    while (!(ready)) {                                  \
        *mfoc_pt_cursor = __LINE__;                     \
        return MFOC_FSM_ONGOING;                        \
        case __LINE__: ;                                \
    }

#define MFOC_PT_END()                                   \
        *mfoc_pt_cursor = 0U;                           \
        break;                                          \
    }
#endif
#endif
