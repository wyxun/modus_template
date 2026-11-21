/**
 * @file state.c
 * @brief Storage for AT32F413 DMA samples and UART queues.
 * @author Codex
 * @date 2026-11-21
 */
#include "instance.h"

volatile uint16_t g_ahwAt32AdcOrdinary[AT32_ADC_ORDINARY_COUNT];
mdi_uart_stream_state_t g_tAt32Stream;
uint8_t g_achAt32StreamTx[AT32_STREAM_BUFFER_SIZE];
uint8_t g_achAt32StreamRx[AT32_STREAM_BUFFER_SIZE];
