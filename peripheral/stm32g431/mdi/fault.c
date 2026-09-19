/**
 * @file mdi_fault_g431.c
 * @brief G431 COMP/TIM1 Break fault state for static MDI.
 * @author Codex
 * @date 2026-09-18
 */
#include "fault.h"

#include "halcomp.h"
#include "haltim1.h"

static volatile bool s_bFaultLatched;

bool mdi_g431_fault_source_active(void)
{
    return (halcomp_GetOutput(HALCOMP_IDX_COMP1) != 0U) ||
           (halcomp_GetOutput(HALCOMP_IDX_COMP2) != 0U) ||
           (halcomp_GetOutput(HALCOMP_IDX_COMP4) != 0U);
}

bool mdi_g431_fault_active(void)
{
    return s_bFaultLatched || haltim1_GetBreakFault();
}

void mdi_g431_fault_clear(void)
{
    if (!mdi_g431_fault_source_active()) {
        s_bFaultLatched = false;
        (void)haltim1_ClearBreakFault();
    }
}

void mdi_g431_fault_notify_break(void)
{
    s_bFaultLatched = true;
}



