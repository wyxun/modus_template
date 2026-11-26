/**
 * @file   at32f413_it.c
 * @brief  Interrupt handlers (AT32F413 Motor EVB V1)
 *
 * SysTick_Handler drives perf_counter and MODUS.
 * ADC1_2 is bound separately to the MFOC control IRQ.
 */

#ifndef CORE_DEBUG_OVERRIDE_FAULT_HANDLER
#define CORE_DEBUG_OVERRIDE_FAULT_HANDLER
#endif
#include "at32f413.h"
#include "perf_counter.h"
#include "mdebug_cm.h"
#include "mdi/instance.h"
#include "mfoc_safety.h"

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
    mfoc_at32_safety_Tick();
    perfc_port_insert_to_system_timer_insert_ovf_handler();
    modus_Clock();
}

/* --------------------------------------------------------------------------
 *  AT32F413 Peripheral Interrupts
 * -------------------------------------------------------------------------- */

/* ---- USART1 (debug shell) ---- */
void USART1_IRQHandler(void)        { MDI_UART_STREAM_IRQ(board_stream); }

/* ---- TMR1 (motor PWM) ---- */
void TMR1_BRK_TMR9_IRQHandler(void)
{
    /* Break interrupt is masked; the physical gate and flag latch. */
}

void TMR1_OVF_TMR10_IRQHandler(void)
{
    if (tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET) {
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    }
}

void TMR1_TRG_HALL_TMR11_IRQHandler(void) {}
void TMR1_CH_IRQHandler(void)              {}

/* ---- Stubs ---- */
void WWDT_IRQHandler(void)                  {}
void PVM_IRQHandler(void)                   {}
void TAMPER_IRQHandler(void)                {}
void RTC_IRQHandler(void)                   {}
void FLASH_IRQHandler(void)                 {}
void CRM_IRQHandler(void)                   {}
void EXINT0_IRQHandler(void)                {}
void EXINT1_IRQHandler(void)                {}
void EXINT2_IRQHandler(void)                {}
void EXINT3_IRQHandler(void)                {}
void EXINT4_IRQHandler(void)                {}
void DMA1_Channel1_IRQHandler(void)         {}
void DMA1_Channel2_IRQHandler(void)         {}
void DMA1_Channel3_IRQHandler(void)         {}
void DMA1_Channel4_IRQHandler(void)         {}
void DMA1_Channel5_IRQHandler(void)         {}
void DMA1_Channel6_IRQHandler(void)         {}
void DMA1_Channel7_IRQHandler(void)         {}
void USBFS_H_CAN1_TX_IRQHandler(void)       {}
void USBFS_L_CAN1_RX0_IRQHandler(void)      {}
void CAN1_RX1_IRQHandler(void)              {}
void CAN1_SE_IRQHandler(void)               {}
void EXINT9_5_IRQHandler(void)              {}
void TMR2_GLOBAL_IRQHandler(void)           {}
#if !AT32F413_BOARD_HALL
void TMR3_GLOBAL_IRQHandler(void)           {}
#endif
void TMR4_GLOBAL_IRQHandler(void)           {}
void I2C1_EVT_IRQHandler(void)              {}
void I2C1_ERR_IRQHandler(void)              {}
void I2C2_EVT_IRQHandler(void)              {}
void I2C2_ERR_IRQHandler(void)              {}
void SPI1_IRQHandler(void)                  {}
void SPI2_IRQHandler(void)                  {}
void USART2_IRQHandler(void)                {}
void USART3_IRQHandler(void)                {}
void EXINT15_10_IRQHandler(void)            {}
void RTCAlarm_IRQHandler(void)              {}
void USBFSWakeUp_IRQHandler(void)           {}
void TMR8_BRK_IRQHandler(void)              {}
void TMR8_OVF_IRQHandler(void)              {}
void TMR8_TRG_HALL_IRQHandler(void)         {}
void TMR8_CH_IRQHandler(void)               {}
void SDIO1_IRQHandler(void)                 {}
void TMR5_GLOBAL_IRQHandler(void)           {}
void UART4_IRQHandler(void)                 {}
void UART5_IRQHandler(void)                 {}
void DMA2_Channel1_IRQHandler(void)         {}
void DMA2_Channel2_IRQHandler(void)         {}
void DMA2_Channel3_IRQHandler(void)         {}
void DMA2_Channel4_5_IRQHandler(void)       {}
void CAN2_TX_IRQHandler(void)               {}
void CAN2_RX0_IRQHandler(void)              {}
void CAN2_RX1_IRQHandler(void)              {}
void CAN2_SE_IRQHandler(void)               {}
void ACC_IRQHandler(void)                   {}
void USBFS_MAPH_IRQHandler(void)            {}
void USBFS_MAPL_IRQHandler(void)            {}
void DMA2_Channel6_7_IRQHandler(void)       {}
