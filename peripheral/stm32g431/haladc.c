/**
 * @file  haladc.c
 * @brief ADC1/ADC2 init — regular + injected, TIM1_CH4 trigger
 *
 * ADC clock from PLL_P (42.5 MHz), requires PLLP output enabled in
 * SystemClock_Config (__HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_ADCCLK)).
 *
 * Calibration and enable are inside MX_ADCx_Init (no separate Activate step).
 */

#include "haladc.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"

/* Post-calibration delay from ST example formula */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32UL)
static volatile bool s_bRegularBusy;

/*----------------------------------------------------------------------------*/
/* Helpers                                                                    */
/*----------------------------------------------------------------------------*/

static void regulator_stabilize(void)
{
    /* volatile 防止空循环被 -Os/-O2 优化掉，稳压器稳定时间必须真实等待 */
    volatile uint32_t wait_loop_index;
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US *
                       (SystemCoreClock / (100000UL * 2UL))) / 10UL);
    while (wait_loop_index != 0U) { wait_loop_index--; }
}

static void post_calib_delay(void)
{
    volatile uint32_t wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while (wait_loop_index != 0U) { wait_loop_index--; }
}

/* PB10 high turns Q48 on.  R58 is then placed in parallel with R30 and the
 * divider becomes the 48 V range.  Keep the pin low during initialization so
 * a 12 V board never starts with the high-range switch enabled. */
void haladc_SetVbusRange48V(uint32_t bEnable)
{
    if (bEnable != 0U) {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);
    } else {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);
    }
}

static void vbus_range_gpio_init(void)
{
    LL_GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Pin        = LL_GPIO_PIN_10;
    GPIO_Init.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_Init.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_Init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_Init.Pull       = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOB, &GPIO_Init);

    haladc_SetVbusRange48V(HALADC_VBUS_RANGE_48V);
}

/*----------------------------------------------------------------------------*/
/* ADC1                                                                       */
/*----------------------------------------------------------------------------*/

static void MX_ADC1_Init(void)
{
    LL_ADC_InitTypeDef      ADC_Init      = {0};
    LL_ADC_REG_InitTypeDef  ADC_REG_Init  = {0};
    LL_ADC_CommonInitTypeDef ADC_Common   = {0};
    LL_ADC_INJ_InitTypeDef  ADC_INJ_Init  = {0};
    LL_GPIO_InitTypeDef     GPIO_Init     = {0};

    /* ---- Clock source ---- */
    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    vbus_range_gpio_init();

    /* ---- Analog GPIO ---- */
    GPIO_Init.Pin  = LL_GPIO_PIN_0;         /* PA0 = ADC1_IN1 busV */
    GPIO_Init.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_Init.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_Init);

    GPIO_Init.Pin = LL_GPIO_PIN_12;         /* PB12 = ADC1_IN11 pot */
    LL_GPIO_Init(GPIOB, &GPIO_Init);

    GPIO_Init.Pin = LL_GPIO_PIN_14;         /* PB14 = ADC1_IN5 temp */
    LL_GPIO_Init(GPIOB, &GPIO_Init);

    /* PA2 (ADC1_IN3, U), PA0 (ADC1_IN1, VBUS) and PB1 (ADC1_IN12, W)
     * are configured as analog inputs by the board initialization. */

    /* ---- Core config ---- */
    ADC_Init.Resolution    = LL_ADC_RESOLUTION_12B;
    ADC_Init.DataAlignment  = LL_ADC_DATA_ALIGN_LEFT;
    ADC_Init.LowPowerMode   = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_Init);

    /* ---- Regular group ---- */
    ADC_REG_Init.TriggerSource    = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_Init.SequencerLength  = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
    ADC_REG_Init.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_Init.ContinuousMode   = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_Init.DMATransfer      = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    ADC_REG_Init.Overrun          = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_Init);

    LL_ADC_SetGainCompensation(ADC1, 0);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);

    /* ---- Common config ---- */
    ADC_Common.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
    ADC_Common.Multimode   = LL_ADC_MULTI_INDEPENDENT;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_Common);

    /* ---- Injected group ---- */
    ADC_INJ_Init.TriggerSource    = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
    ADC_INJ_Init.SequencerLength  = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
    ADC_INJ_Init.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_Init.TrigAuto         = LL_ADC_INJ_TRIG_INDEPENDENT;
    LL_ADC_INJ_Init(ADC1, &ADC_INJ_Init);
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    /* ---- Exit deep power down, enable regulator, wait ---- */
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    regulator_stabilize();

    /* ---- Channel config (AFTER regulator stable) ---- */
    /* Regular ranks */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_11);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);

    /* Injected ranks. Rank 2 repeats U to preserve the two-rank JEOS timing. */
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);

    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_3);

    /* ---- Calibration ---- */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    {
        uint32_t _to = 1000000UL;
        while ((LL_ADC_IsCalibrationOnGoing(ADC1) != 0) && --_to) {}
    }

    /* Post-calibration delay (required by STM32G4 spec) */
    post_calib_delay();

    /* ---- Enable ---- */
    LL_ADC_Enable(ADC1);
    { uint32_t _to = 1000000UL; while (!LL_ADC_IsActiveFlag_ADRDY(ADC1) && --_to) {} }

    /* ---- Arm injected group (required for external trigger) ---- */
    LL_ADC_INJ_StartConversion(ADC1);
}

/*----------------------------------------------------------------------------*/
/* ADC2                                                                       */
/*----------------------------------------------------------------------------*/

static void MX_ADC2_Init(void)
{
    LL_ADC_InitTypeDef     ADC_Init     = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_Init = {0};
    LL_ADC_INJ_InitTypeDef ADC_INJ_Init = {0};

    /* Clock already enabled by ADC1 init */

    /* ---- Core config ---- */
    ADC_Init.Resolution    = LL_ADC_RESOLUTION_12B;
    ADC_Init.DataAlignment  = LL_ADC_DATA_ALIGN_LEFT;
    ADC_Init.LowPowerMode   = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC2, &ADC_Init);

    /* ---- Regular (disabled) ---- */
    ADC_REG_Init.SequencerLength  = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_Init.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_Init.ContinuousMode   = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_Init.DMATransfer      = LL_ADC_REG_DMA_TRANSFER_NONE;
    ADC_REG_Init.Overrun          = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC2, &ADC_REG_Init);

    LL_ADC_SetGainCompensation(ADC2, 0);
    LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);

    /* ---- Injected group ---- */
    ADC_INJ_Init.TriggerSource    = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
    ADC_INJ_Init.SequencerLength  = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
    ADC_INJ_Init.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_Init.TrigAuto         = LL_ADC_INJ_TRIG_INDEPENDENT;
    LL_ADC_INJ_Init(ADC2, &ADC_INJ_Init);
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    /* ---- Exit deep power down, enable regulator, wait ---- */
    LL_ADC_DisableDeepPowerDown(ADC2);
    LL_ADC_EnableInternalRegulator(ADC2);
    regulator_stabilize();

    /* ---- Channel config (AFTER regulator stable) ---- */
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP3_ADC2);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2, LL_ADC_SINGLE_ENDED);

    /* V 相：OPAMP2 内部输出 → ADC2（LL_ADC_CHANNEL_VOPAMP2，与 W 相 VOPAMP3_ADC2 对称）；
       外部引脚 PA6 实测零偏 24632 ≠ U/W 的 41000，非正确节点。 */
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_VOPAMP2);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_VOPAMP2, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_VOPAMP2, LL_ADC_SINGLE_ENDED);

    /* ---- Calibration ---- */
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    {
        uint32_t _to = 1000000UL;
        while ((LL_ADC_IsCalibrationOnGoing(ADC2) != 0) && --_to) {}
    }

    post_calib_delay();

    /* ---- Enable ---- */
    LL_ADC_Enable(ADC2);
    {
        uint32_t _to = 1000000UL;
        while (!LL_ADC_IsActiveFlag_ADRDY(ADC2) && --_to) {}
        if (_to == 0) {
            LL_ADC_Disable(ADC2);
            return; /* ADC2 not ready — skip, MCU continues */
        }
    }

    /* ---- Arm injected group (required for external trigger) ---- */
    LL_ADC_INJ_StartConversion(ADC2);
}

static void regular_dma_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    s_bRegularBusy = false;
    LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_0,
                           LL_DMAMUX_REQ_ADC1);
    LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_NORMAL |
        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
        LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD |
        LL_DMA_PRIORITY_HIGH);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/*----------------------------------------------------------------------------*/
/* Public API                                                                 */
/*----------------------------------------------------------------------------*/

void haladc_Init(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
    LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC12);
    LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC12);

    MX_ADC1_Init();
    MX_ADC2_Init();
    regular_dma_init();

    /* 注入序列完成中断：双 ADC 同沿触发、等长等速序列，
     * 只需 ADC1 的 JEOS 作为高频环时基（同 MCSDK 做法）。 */
    LL_ADC_EnableIT_JEOS(ADC1);
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    /* NVIC 在所有底层硬件（含 TIM1）初始化完成后统一由 haladc_EnableISR 使能 */

}

void haladc_EnableISR(void)
{
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

bool haladc_StartRegular(volatile uint16_t *pwDmaBuffer,
                         uint32_t wTransferCount)
{
    if (pwDmaBuffer == NULL || wTransferCount == 0U ||
        wTransferCount > UINT16_MAX || s_bRegularBusy) {
        return false;
    }
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
        (uint32_t)(uintptr_t)&ADC1->DR,
        (uint32_t)(uintptr_t)pwDmaBuffer,
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, wTransferCount);
    s_bRegularBusy = true;
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_REG_StartConversion(ADC1);
    return true;
}

bool haladc_RegularDmaCompleteISR(void)
{
    if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0U) {
        LL_DMA_ClearFlag_TC1(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        LL_ADC_REG_StopConversion(ADC1);
        s_bRegularBusy = false;
        return true;
    }
    return false;
}

