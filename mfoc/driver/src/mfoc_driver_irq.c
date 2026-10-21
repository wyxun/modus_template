/**
 * @file mfoc_driver_irq.c
 * @brief Driver-owned hardware IRQ body with a target-bound vector alias.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_driver_irq.h"

#if defined(MFOC_DRIVER_IRQ_ENABLED) && MFOC_DRIVER_IRQ_ENABLED
#include "mfoc_driver_irq_binding.h"

#if !defined(MFOC_DRIVER_IRQ_VECTOR) || !defined(MFOC_DRIVER_IRQ_INSTANCE) ||   \
    !defined(MFOC_DRIVER_IRQ_BEGIN) || !defined(MFOC_DRIVER_IRQ_END)
#error "Provide the target Driver IRQ binding"
#endif

/** @brief Handle one pending frame and always finish the hardware cycle.
 * @return None; irq_result is a diagnostic status, not a second fault latch.
 * @note All bindings are direct calls/objects, without runtime dispatch. */
void mfoc_driver_IRQHandler(void)
{
    uint32_t token = 0U;
    if (!MFOC_DRIVER_IRQ_BEGIN(&token)) { return; }
    MFOC_DRIVER_IRQ_INSTANCE.irq_result = mfoc_driver_Step(
        &MFOC_DRIVER_IRQ_INSTANCE);
    MFOC_DRIVER_IRQ_END(token);
}

/* A typedef specifies the signature; alias exports the actual vector name.
 * The alias target must be defined in this translation unit. */
extern mfoc_driver_irq_handler_t MFOC_DRIVER_IRQ_VECTOR
    __attribute__((alias("mfoc_driver_IRQHandler")));
#endif
