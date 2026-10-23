/**
 * @file mfoc_motor_irq.c
 * @brief Motor-owned hardware IRQ body with a target-bound vector alias.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_motor_irq.h"
#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
#include "mfoc_debug.h"
#endif
#if !defined(__NO_USE_LOG__) && !defined(FOC_ISR_TIMING_DWT_ONLY)
#include "perf_counter.h"
#endif

#include "mfoc_motor_irq_binding.h"

#if !defined(MFOC_MOTOR_IRQ_VECTOR) || !defined(MFOC_MOTOR_IRQ_INSTANCE) ||     \
    !defined(MFOC_MOTOR_IRQ_READY) || !defined(MFOC_MOTOR_IRQ_BEGIN) ||         \
    !defined(MFOC_MOTOR_IRQ_END)
#error "Provide the target Motor IRQ binding"
#endif

/** @brief Handle one pending frame and always finish the hardware cycle.
 * @return None; irq_result is a diagnostic status, not a second fault latch.
 * @note All bindings are direct calls/objects, without runtime dispatch. */
void mfoc_motor_IRQHandler(void)
{
#if !defined(__NO_USE_LOG__) && !defined(FOC_ISR_TIMING_DWT_ONLY)
    uint32_t start = (uint32_t)get_system_ticks();
#endif
    if (MFOC_MOTOR_IRQ_READY()) {
        MFOC_MOTOR_IRQ_BEGIN();
        MFOC_MOTOR_IRQ_INSTANCE.irq_result = mfoc_motor_Step(
            &MFOC_MOTOR_IRQ_INSTANCE);
        MFOC_MOTOR_IRQ_END();
    }
#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
    mfoc_debug_WaveformStep();
#endif
#if !defined(__NO_USE_LOG__) && !defined(FOC_ISR_TIMING_DWT_ONLY)
    uint32_t elapsed = (uint32_t)get_system_ticks() - start;
    if (elapsed > tMfocApp.tIsrTiming.wMaximumTicks) {
        tMfocApp.tIsrTiming.wMaximumTicks = elapsed;
    }
    if (elapsed > MFOC_ISR_BUDGET_TICKS) {
        tMfocApp.tIsrTiming.bOverrun = true;
    }
#endif
}

/* A typedef specifies the signature; alias exports the actual vector name.
 * The alias target must be defined in this translation unit. */
extern mfoc_motor_irq_handler_t MFOC_MOTOR_IRQ_VECTOR
    __attribute__((alias("mfoc_motor_IRQHandler")));
