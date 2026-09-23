/**
 * @file state.c
 * @brief G431 ADC/DMA acquisition state owned by the MDI instance.
 * @author Codex
 * @date 2026-09-23
 */

#include "instance.h"

volatile uint16_t
    g_awG431AdcDma[G431_ADC_DMA_SLOT_COUNT * G431_ADC_BLOCK_SIZE];
volatile uint32_t g_wG431AdcPublished = 0U;
volatile uint32_t g_wG431AdcConsumed = 0U;
volatile uint32_t g_wG431AdcSequence = 0U;
volatile uint32_t g_wG431AdcRateHz = G431_ADC_RATE_HZ;
volatile uint32_t g_awG431AdcMean[G431_ADC_CHANNEL_COUNT];
volatile bool g_bG431AdcMeanValid = false;
volatile mdi_status_t g_eG431AdcStatus = MDI_OK;
volatile bool g_bG431FaultLatched = false;
g431_stream_state_t g_tG431Stream;
uint8_t g_achG431StreamTx[G431_STREAM_BUFFER_SIZE];
uint8_t g_achG431StreamRx[G431_STREAM_BUFFER_SIZE];
