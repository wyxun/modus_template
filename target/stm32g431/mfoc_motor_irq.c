/**
 * @file mfoc_motor_irq.c
 * @brief STM32G431 control IRQ and application diagnostics.
 * @author wyxun
 * @date 2026-10-21
 */
#include "mfoc_motor_irq.h"
#if MFOC_RUNTIME_BUS
#include "mfoc_board_bus.h"
#endif
#if MFOC_P10_TIMING
#include "mfoc_p10_timing.h"
#endif
#if MFOC_P10_PHYSICAL
#include "mfoc_p10_physical.h"
#endif
#if MFOC_P10_KCL
#include "mfoc_p10_kcl.h"
#endif
#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT)
#include "mfoc_debug.h"
#endif
#if MFOC_ISR_MEASURE
#include "perf_counter.h"
#endif

#include <mfoc_motor_irq_binding.h>

#if !defined(MFOC_MOTOR_IRQ_VECTOR) || !defined(MFOC_MOTOR_IRQ_INSTANCE) ||     \
    !defined(MFOC_MOTOR_IRQ_READY) || !defined(MFOC_MOTOR_IRQ_BEGIN) ||         \
    !defined(MFOC_MOTOR_IRQ_END)
#error "Provide MFOC_MOTOR_IRQ_BINDING"
#endif

/** @brief Handle one pending frame and always finish the hardware cycle.
 * @return None; irq_result is a diagnostic status, not a second fault latch.
 * @note All bindings are direct calls/objects, without runtime dispatch. */
void mfoc_motor_IRQHandler(void)
{
#if MFOC_P10_PHASE
    g_p10_irq_entry = TIM2->CNT;
#endif
#if MFOC_ISR_MEASURE
    uint32_t start = (uint32_t)get_system_ticks();
#endif
    if (MFOC_MOTOR_IRQ_READY()) {
#if MFOC_P10_PHYSICAL
        MFOC_MOTOR_IRQ_BEGIN();
        if (mfoc_p10_PhysicalStep()) {
            MFOC_MOTOR_IRQ_END();
            return;
        }
#endif
#if MFOC_P10_TIMING
        mfoc_p10_Begin(&MFOC_MOTOR_IRQ_INSTANCE);
#endif
        MFOC_MOTOR_IRQ_BEGIN();
#if MFOC_RUNTIME_BUS
        mfoc_motor_t *motor = &MFOC_MOTOR_IRQ_INSTANCE;
        mfoc_result_t bus = MFOC_RESULT_OK;
        if (motor->state == MFOC_MOTOR_RUNNING) {
            bus = mfoc_board_bus_Apply(&motor->core);
        }
        if (bus != MFOC_RESULT_OK) {
            mfoc_result_t stop = mfoc_motor_Trip(motor, MFOC_MOTOR_FAULT_SAMPLE);
            motor->irq_result = stop == MFOC_RESULT_OK ? bus : stop;
        } else
#endif
        {
#if MFOC_P10_LIMIT
            mfoc_core_bus_coeff_t saved = {0};
            const bool limited_frame = mfoc_p10_LimitFrame(motor);
            if (limited_frame) {
                mfoc_core_BenchHalfVoltageLimit(&motor->core, &saved);
            }
#endif
        MFOC_MOTOR_IRQ_INSTANCE.irq_result = mfoc_motor_Step(
            &MFOC_MOTOR_IRQ_INSTANCE);
#if MFOC_P10_LIMIT
            if (limited_frame) { mfoc_core_ApplyBus(&motor->core, &saved); }
#endif
        }
        MFOC_MOTOR_IRQ_END();
#if MFOC_P10_KCL
        mfoc_p10_kcl_AfterStep(&MFOC_MOTOR_IRQ_INSTANCE);
#endif
#if MFOC_P10_TIMING
        mfoc_p10_End(&MFOC_MOTOR_IRQ_INSTANCE);
#if MFOC_P10_PHASE
        *g_p10_irq_tail = TIM2->CNT;
#endif
#endif
#if MFOC_DEBUG_POSITION_WAVEFORM
        mfoc_debug_WaveformStep();
#endif
    }
#if MWAVEFORM_ENABLE && defined(MFOC_NUMERIC_FLOAT) && !MFOC_DEBUG_POSITION_WAVEFORM
    mfoc_debug_WaveformStep();
#endif
#if MFOC_ISR_MEASURE
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
