/**
 * @file state.c
 * @brief Storage owned by the generic ADC/DMA reference instance.
 */
#include "instance.h"

volatile uint16_t g_awPt32AdcDma[PT32_ADC_BLOCK_SIZE * 2U];
volatile uint32_t g_awPt32AdcMean[PT32_ADC_CHANNEL_COUNT];
volatile uint32_t g_wPt32AdcMeanSequence;
volatile bool g_bPt32AdcMeanValid;
volatile uint32_t g_wPt32AdcPublished;
volatile uint32_t g_wPt32AdcConsumed;
