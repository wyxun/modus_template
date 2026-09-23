/**
 * @file   stm32g4xx_it.c
 * @brief  Interrupt handlers (STM32G431)
 *
 * SysTick_Handler → perf_counter + MODUS/MDI 1 ms clock.
 * ADC1_2 → foc_app_HighFrequencyISR (FOC 20 kHz control step, TIM1 CH4 trigger).
 * USART / FDCAN / TIM1 break handlers active; other peripherals are stubs.
 */

#include "stm32g4xx_hal.h"
#include "perf_counter.h"
#include "halfdcan.h"
#include "mdebug_cm.h"
#include "foc_app.h"
#include "haltim1.h"
#include "mdi/instance.h"
#include "mdi/service.h"
#include <math.h>

/* Exported by main.c */
extern void modus_Clock(void);

/* --------------------------------------------------------------------------
 *  Cortex-M4 Core Exceptions
 * -------------------------------------------------------------------------- */
void NMI_Handler(void)              { while(1); }
#ifndef MDEBUG_CM_FAULT_HANDLERS_ACTIVE
void HardFault_Handler(void)        { while(1); }
void MemManage_Handler(void)        { while(1); }
void BusFault_Handler(void)         { while(1); }
void UsageFault_Handler(void)       { while(1); }
#endif
void SVC_Handler(void)              {}
void DebugMon_Handler(void)         {}
void PendSV_Handler(void)           {}

void SysTick_Handler(void)
{
    HAL_IncTick();
    perfc_port_insert_to_system_timer_insert_ovf_handler();
#if MODUS_ENABLE
    modus_Clock();
#endif
}

/* --------------------------------------------------------------------------
 *  STM32G431 Peripheral Interrupts
 * -------------------------------------------------------------------------- */

/* ---- USART ---- */
void USART1_IRQHandler(void)        {}
void USART2_IRQHandler(void)        { MDI_UART_STREAM_IRQ(board_stream); }
void USART3_IRQHandler(void)        {}

/* ---- FDCAN ---- */
void FDCAN1_IT0_IRQHandler(void)    { HAL_FDCAN_IRQHandler(&hfdcan1); }
void FDCAN1_IT1_IRQHandler(void)    { HAL_FDCAN_IRQHandler(&hfdcan1); }

/* ---- TIM1 (motor PWM) ---- */
void TIM1_BRK_TIM15_IRQHandler(void)
{
    /* 先锁存软件故障，再清硬件标志，避免前台读到被清空的 BIF */
    mdi_g431_fault_notify_break();
    (void)haltim1_ClearBreakFault();
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    TIM1->SR &= ~TIM_SR_UIF;
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)    {}
void TIM1_CC_IRQHandler(void)               {}

/* ---- ADC1_2 (current sensing end-of-conversion) ---- */
void ADC1_2_IRQHandler(void)
{
    if ((ADC1->ISR & ADC_ISR_JEOS) != 0U) {
        /*
         * MDI's completed-frame provider owns the JEOS acknowledgement when
         * the FOC cycle consumes the frame.  Calling FOC before clearing the
         * flag is required: MDI_Sample_ReadCompleted() uses JEOS as the
         * completion boundary and returns MDI_BUSY after an early clear.
         * If the application is not ready yet, the cleanup below still
         * acknowledges the pending conversion.
         */
        foc_app_HighFrequencyISR();

        ADC1->ISR = ADC_ISR_JEOS;
        ADC2->ISR = ADC_ISR_JEOS;
        ADC1->ISR = ADC_ISR_OVR;
        ADC2->ISR = ADC_ISR_OVR;
        (void)ADC1->ISR;
        (void)ADC2->ISR;
    }
}

/* ---- Stubs ---- */
void WWDG_IRQHandler(void)                  {}
void PVD_PVM_IRQHandler(void)               {}
void RTC_TAMP_LSECSS_IRQHandler(void)       {}
void RTC_WKUP_IRQHandler(void)              {}
void FLASH_IRQHandler(void)                 {}
void RCC_IRQHandler(void)                   {}
void EXTI0_IRQHandler(void)                 {}
void EXTI1_IRQHandler(void)                 {}
void EXTI2_IRQHandler(void)                 {}
void EXTI3_IRQHandler(void)                 {}
void EXTI4_IRQHandler(void)                 {}
void DMA1_Channel1_IRQHandler(void)
{
    mdi_g431_adc_dma_complete();
}
void DMA1_Channel2_IRQHandler(void)         {}
void DMA1_Channel3_IRQHandler(void)         {}
void DMA1_Channel4_IRQHandler(void)         {}
void DMA1_Channel5_IRQHandler(void)         {}
void DMA1_Channel6_IRQHandler(void)         {}
void USB_HP_IRQHandler(void)                {}
void USB_LP_IRQHandler(void)                {}
void EXTI9_5_IRQHandler(void)               {}
void TIM2_IRQHandler(void)                  {}
void TIM3_IRQHandler(void)                  {}
void TIM4_IRQHandler(void)                  {}
void I2C1_EV_IRQHandler(void)               {}
void I2C1_ER_IRQHandler(void)               {}
void I2C2_EV_IRQHandler(void)               {}
void I2C2_ER_IRQHandler(void)               {}
void SPI1_IRQHandler(void)                  {}
void SPI2_IRQHandler(void)                  {}
void EXTI15_10_IRQHandler(void)             {}
void RTC_Alarm_IRQHandler(void)             {}
void USBWakeUp_IRQHandler(void)             {}
void TIM8_BRK_IRQHandler(void)              {}
void TIM8_UP_IRQHandler(void)               {}
void TIM8_TRG_COM_IRQHandler(void)          {}
void TIM8_CC_IRQHandler(void)               {}
void LPTIM1_IRQHandler(void)                {}
void SPI3_IRQHandler(void)                  {}
void UART4_IRQHandler(void)                 {}
void TIM6_DAC_IRQHandler(void)              {}
void TIM7_IRQHandler(void)                  {}
void DMA2_Channel1_IRQHandler(void)         {}
void DMA2_Channel2_IRQHandler(void)         {}
void DMA2_Channel3_IRQHandler(void)         {}
void DMA2_Channel4_IRQHandler(void)         {}
void DMA2_Channel5_IRQHandler(void)         {}
void UCPD1_IRQHandler(void)                 {}
void COMP1_2_3_IRQHandler(void)             {}
void COMP4_IRQHandler(void)                 {}
void CRS_IRQHandler(void)                   {}
void SAI1_IRQHandler(void)                  {}
void FPU_IRQHandler(void)                   {}
void RNG_IRQHandler(void)                   {}
void LPUART1_IRQHandler(void)               {}
void I2C3_EV_IRQHandler(void)               {}
void I2C3_ER_IRQHandler(void)               {}
void DMAMUX_OVR_IRQHandler(void)            {}
void DMA2_Channel6_IRQHandler(void)         {}
void CORDIC_IRQHandler(void)                {}
void FMAC_IRQHandler(void)                  {}
