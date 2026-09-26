/**
 * @file state.c
 * @brief Storage owned by the generic ADC/DMA reference instance.
 */
#include "instance.h"

volatile uint16_t
    g_awPt32AdcDma[PT32_ADC_BLOCK_SIZE * PT32_ADC_DMA_SLOT_COUNT];
volatile uint32_t g_awPt32AdcMean[PT32_ADC_CHANNEL_COUNT];
volatile uint32_t g_wPt32AdcMeanSequence = 0U;
volatile bool g_bPt32AdcMeanValid = false;
volatile uint32_t g_wPt32AdcPublished = 0U;
volatile uint32_t g_wPt32AdcConsumed = 0U;
volatile mdi_tick_t g_qwPt32RawTick = 0U;
volatile mdi_status_t g_ePt32AdcStatus = MDI_OK;
mdi_uart_stream_state_t g_tPt32Stream;
uint8_t g_achPt32StreamTx[PT32_STREAM_CAPACITY];
uint8_t g_achPt32StreamRx[PT32_STREAM_CAPACITY];
