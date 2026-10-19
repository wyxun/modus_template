/**
 * @file mdi/instance.h
 * @brief G431 compile-time MDI resource instance binding.
 * @author Codex
 * @date 2026-09-18
 * @note Clocks, pad modes, ownership, ADC triggers and PWM preload are
 *       configured by the board startup code.
 */
#ifndef STM32G431_MDI_INSTANCE_H
#define STM32G431_MDI_INSTANCE_H
#include "stm32g431xx.h"
#include "mdi/core/contract.h"
#include "mdi/core/bind.h"
#include "mdi/core/stream.h"
#include "mdi/feature/adc_mean.h"
#include "mdi/feature/i2c_reg8.h"
#include "backend.h"
#include "i2c.h"
#include "pwm.h"
#include "haladc.h"

MDI_STM32_TICK_BIND(raw_tick, get_system_ticks)
MDI_G431_UART_STREAM_BIND(board_stream, g_tG431Stream, USART2)

/* Each physical port appears once. Pin lists: logical bit, pin, invert. */
#define G431_LED_PINS(X, V, ...) X(V, 0, 6, 1)
#define G431_LED_PORTS(X, V) X(V, GPIOC->IDR, GPIOC->BSRR, G431_LED_PINS)
MDI_STM32_IO_BIND_OUTPUT_CAPS(status_led, 1, G431_LED_PORTS,
                              MDI_IO_CAP_OUTPUT)

/* Hardware I2C owns PB7/PB8 and is initialized during board startup. */
MDI_STM32_I2C_BIND(encoder_i2c, I2C1, 100000U)
MDI_I2C_REG8_BIND(encoder_angle, encoder_i2c, 0x36U)

/* One regular ADC scan group: VBUS, temperature, potentiometer. */
#define G431_ADC_RATE_HZ          100U
#define G431_ADC_SAMPLE_COUNT     10U
#define G431_ADC_CHANNEL_COUNT    HALADC_REGULAR_CHANNEL_COUNT
#define G431_ADC_DMA_SLOT_COUNT   2U
#define G431_ADC_BLOCK_SIZE \
    (G431_ADC_SAMPLE_COUNT * G431_ADC_CHANNEL_COUNT)
_Static_assert(G431_ADC_BLOCK_SIZE > 0U &&
               G431_ADC_BLOCK_SIZE <= UINT16_MAX, "invalid DMA block size");
#define G431_ADC_CHANNELS(X, ...) \
    X(__VA_ARGS__, adc_bus_voltage, 0) \
    X(__VA_ARGS__, adc_temperature, 1) \
    X(__VA_ARGS__, adc_potentiometer, 2)

extern volatile uint16_t
    g_awG431AdcDma[G431_ADC_DMA_SLOT_COUNT * G431_ADC_BLOCK_SIZE];
extern volatile uint32_t g_wG431AdcPublished;
extern volatile uint32_t g_wG431AdcConsumed;
extern volatile uint32_t g_wG431AdcSequence;
/* This board owns one regular DMA transaction; ISR releases its slot. */
typedef struct {
    volatile bool bBusy;
    volatile mdi_status_t eError;
} mdi_g431_adc_state_t;
extern mdi_g431_adc_state_t g_tG431Adc;
extern volatile uint32_t g_awG431AdcMean[G431_ADC_CHANNEL_COUNT];
extern volatile bool g_bG431AdcMeanValid;
extern volatile mdi_status_t g_eG431AdcStatus;

MDI_ADC_DMA_FLAG_BIND(adc_dma, g_wG431AdcPublished,
                      g_wG431AdcConsumed, G431_ADC_DMA_SLOT_COUNT)
MDI_ADC_MEAN_GROUP_BIND(adc_mean, adc_dma, g_awG431AdcDma,
                        G431_ADC_SAMPLE_COUNT, G431_ADC_CHANNEL_COUNT,
                        G431_ADC_CHANNELS, g_awG431AdcMean,
                        g_wG431AdcSequence, g_bG431AdcMeanValid)

/**
 * @brief Arm the next regular ADC DMA slot after mean publication.
 * @param None.
 * @return OK, BUSY while stopping/in flight, or the latched DMA error.
 * @note Foreground only; the ISR cannot release an unarmed transaction.
 */
static inline mdi_status_t mdi_g431_adc_start(void)
{
    uint32_t wSlot = 0U;
    volatile uint16_t *pwBuffer = NULL;

    if (g_tG431Adc.eError != MDI_OK) { return g_tG431Adc.eError; }
    if (g_tG431Adc.bBusy || (DMA1_Channel1->CCR & DMA_CCR_EN) != 0U ||
        (ADC1->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP)) != 0U) {
        return MDI_BUSY;
    }
    if ((ADC1->CR & ADC_CR_ADEN) == 0U) { return MDI_IO_ERROR; }
    /* A completion IRQ may advance Published before the busy checks. Once
     * idle, no armed transaction can publish while this slot is selected. */
    wSlot = MDI_ADC_DMA_Published(adc_dma) % G431_ADC_DMA_SLOT_COUNT;
    pwBuffer = &g_awG431AdcDma[wSlot * G431_ADC_BLOCK_SIZE];
    DMA1->IFCR = DMA_IFCR_CGIF1;
    ADC1->ISR = ADC_ISR_OVR;
    DMA1_Channel1->CPAR = (uint32_t)(uintptr_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)(uintptr_t)pwBuffer;
    DMA1_Channel1->CNDTR = G431_ADC_BLOCK_SIZE;
    g_tG431Adc.bBusy = true;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;
    return MDI_OK;
}

MDI_ADC_TRIGGER_FN_BIND(adc_mean, mdi_g431_adc_start)
MDI_ADC_CHANNEL_VIEW_BIND(adc_bus_voltage, g_awG431AdcMean[0],
                          0x0FFFU, 0U, g_wG431AdcSequence,
                          g_bG431AdcMeanValid)
MDI_ADC_CHANNEL_VIEW_BIND(adc_temperature, g_awG431AdcMean[1],
                          0x0FFFU, 0U, g_wG431AdcSequence,
                          g_bG431AdcMeanValid)
MDI_ADC_CHANNEL_VIEW_BIND(adc_potentiometer, g_awG431AdcMean[2],
                          0x0FFFU, 0U, g_wG431AdcSequence,
                          g_bG431AdcMeanValid)

/* Regular DR stays 12-bit. Injected JDR keeps the right-aligned 4x sum;
 * target current scaling uses 476 sum counts per 3.5 A (4 * 119). */
#define G431_CURRENT_CHANNELS(X) \
    X(u, ADC1->JDR1, 0x3FFFU, 0) \
    X(v, ADC2->JDR2, 0x3FFFU, 0) \
    X(w, ADC2->JDR1, 0x3FFFU, 0)
extern volatile bool g_bG431JeosConsumed;

/** @brief Acknowledge both injected frames after their JDR values are read. */
static inline void mdi_g431_ack_current_frame(void)
{
    ADC1->ISR = ADC_ISR_JEOS;
    ADC2->ISR = ADC_ISR_JEOS;
    g_bG431JeosConsumed = true;
}

/** @brief At IRQ exit, leave a newly completed frame pending if consumed. */
static inline void mdi_g431_ack_unconsumed_current_frame(void)
{
    uint32_t wAck = ADC_ISR_OVR;

    if (!g_bG431JeosConsumed) {
        wAck |= ADC_ISR_JEOS;
    }
    ADC1->ISR = wAck;
    ADC2->ISR = wAck;
    (void)ADC1->ISR;
    (void)ADC2->ISR;
}

#define G431_PHASE_CURRENT_READY (ADC1->ISR & ADC_ISR_JEOS)
#define G431_PHASE_CURRENT_CLEAR mdi_g431_ack_current_frame()
MDI_SAMPLE_READY_BIND(phase_current_completed, G431_CURRENT_CHANNELS,
                      G431_PHASE_CURRENT_READY, G431_PHASE_CURRENT_CLEAR)

/* Center-aligned PWM1: ticks per half cycle = ARR. One group owns TIM1. */
#define G431_BRIDGE_CHANNELS(X)                                                    \
    X(u, TIM1->CCR1, TIM1->ARR)                                                  \
    X(v, TIM1->CCR2, TIM1->ARR)                                                  \
    X(w, TIM1->CCR3, TIM1->ARR)
MDI_PWM_REG_BIND(bridge, G431_BRIDGE_CHANNELS)
MDI_STM32_PWM_TIMING_BIND(bridge, TIM1, 170000000U, 2U, G431_BRIDGE_CHANNELS)
MDI_G431_PWM_LIFECYCLE_BIND(bridge)
/* Edge/up PWM1: ARR + 1 ticks per cycle; channel one is a one-field group. */
#define G431_BUZZER_CHANNELS(X) X(value, TIM3->CCR1, TIM3->ARR + 1U)
MDI_PWM_REG_BIND(buzzer, G431_BUZZER_CHANNELS)
MDI_STM32_PWM_TIMING_BIND(buzzer, TIM3, 170000000U, 1U, G431_BUZZER_CHANNELS)

#define G431_TIM1_HALF_PERIOD_TICKS  4250U
#define G431_TIM1_FULL_PERIOD_TICKS  8500U
#define G431_TIM1_TRIG_POS_TICKS     4240U
#define G431_ISR_LATE_CYC_THRESHOLD  9350U
#define G431_ISR_VLATE_CYC_THRESHOLD 12750U

#if defined(__ARM_ARCH) || defined(__CORTEX_M) || defined(__arm__) ||           \
    defined(MDI_G431_ENABLE_HOST_PROBE)
#define MDI_G431_PROBE_ACTIVE 1
#else
#define MDI_G431_PROBE_ACTIVE 0
#endif

/**
 * @brief Board-owned 20 kHz ADC1 JEOS ISR cycle and TIM1 phase probe state.
 * @note All 24 fields share one responsibility: capturing monotonic DWT
 *       cycle intervals/splits and turnaround-safe TIM1 phase relative to
 *       the CH4 PWM2 OC4REF rising trigger at (DIR=0, CNT=4240).
 */
typedef struct {
    uint32_t wSamples;
    uint32_t wPrevEntryCyc;
    uint32_t wLastEntryCyc;
    uint32_t wLastDutyCyc;
    uint32_t wLastObsCyc;
    uint32_t wLastExecCyc;
    uint32_t wMaxExecCyc;
    uint32_t wMinIntervalCyc;
    uint32_t wMaxIntervalCyc;
    uint32_t wMaxEntryToDutyCyc;
    uint32_t wMaxDutyToObsCyc;
    uint32_t wMaxObsToExitCyc;
    uint32_t wLateEntryCount;
    uint32_t wVeryLateEntryCount;
    uint32_t wOverrunCount;
    uint16_t hwMinEntryPhase;
    uint16_t hwMaxEntryPhase;
    uint16_t hwMinDutyPhase;
    uint16_t hwMaxDutyPhase;
    uint16_t hwLastDutyPhase;
    uint16_t hwMaxExitPhase;
    uint16_t hwProbeOverheadCyc;
    uint8_t bDutyUpdated;
    uint8_t bObsUpdated;
    uint8_t bIsrActive;
} mdi_g431_isr_probe_t;

extern mdi_g431_isr_probe_t g_tG431IsrProbe;

/**
 * @brief Read TIM1 phase in [0..8499] ticks from CH4 OC4REF rising edge.
 * @param None.
 * @return Ticks since (DIR=0, CNT=4240); 10 = top (4250), 4260 = bottom (0).
 * @note Sandwiches CNT between two CR1 reads and re-reads CNT if DIR flips.
 */
static inline uint16_t mdi_g431_tim1_phase_from_trig(void)
{
#if MDI_G431_PROBE_ACTIVE
    uint32_t wCr1First = TIM1->CR1;
    uint32_t wCnt = TIM1->CNT & 0xFFFFU;
    uint32_t wCr1Second = TIM1->CR1;
    uint32_t wPos = 0U;

    if (((wCr1First ^ wCr1Second) & TIM_CR1_DIR) != 0U) {
        wCnt = TIM1->CNT & 0xFFFFU;
        wCr1First = wCr1Second;
    }
    if (wCnt > G431_TIM1_HALF_PERIOD_TICKS) {
        wCnt = G431_TIM1_HALF_PERIOD_TICKS;
    }
    wPos = ((wCr1First & TIM_CR1_DIR) != 0U) ?
           (G431_TIM1_FULL_PERIOD_TICKS - wCnt) : wCnt;
    return (uint16_t)((wPos >= G431_TIM1_TRIG_POS_TICKS) ?
        (wPos - G431_TIM1_TRIG_POS_TICKS) :
        (wPos + (G431_TIM1_FULL_PERIOD_TICKS - G431_TIM1_TRIG_POS_TICKS)));
#else
    return 0U;
#endif
}

/** @brief Reset timing probe accumulators while keeping measured overhead. */
static inline void mdi_g431_isr_probe_reset(uint16_t hwOverheadCyc)
{
    g_tG431IsrProbe = (mdi_g431_isr_probe_t){
        .wMinIntervalCyc = UINT32_MAX,
        .hwMinEntryPhase = UINT16_MAX,
        .hwMinDutyPhase = UINT16_MAX,
        .hwProbeOverheadCyc = hwOverheadCyc,
    };
}

/** @brief Record outer ADC1_2_IRQHandler entry cycle and TIM1 phase. */
static inline uint32_t mdi_g431_isr_probe_begin(void)
{
#if MDI_G431_PROBE_ACTIVE
    uint32_t wEntryCyc = DWT->CYCCNT;
    uint16_t hwPhase = mdi_g431_tim1_phase_from_trig();
    uint32_t wSamples = g_tG431IsrProbe.wSamples;

    g_tG431IsrProbe.wLastEntryCyc = wEntryCyc;
    g_tG431IsrProbe.bIsrActive = 1U;
    g_tG431IsrProbe.bDutyUpdated = 0U;
    g_tG431IsrProbe.bObsUpdated = 0U;
    if (wSamples != 0U) {
        uint32_t wInterval = wEntryCyc - g_tG431IsrProbe.wPrevEntryCyc;
        if (wInterval < g_tG431IsrProbe.wMinIntervalCyc) {
            g_tG431IsrProbe.wMinIntervalCyc = wInterval;
        }
        if (wInterval > g_tG431IsrProbe.wMaxIntervalCyc) {
            g_tG431IsrProbe.wMaxIntervalCyc = wInterval;
        }
        if (wInterval > G431_ISR_LATE_CYC_THRESHOLD) {
            g_tG431IsrProbe.wLateEntryCount++;
            if (wInterval > G431_ISR_VLATE_CYC_THRESHOLD) {
                g_tG431IsrProbe.wVeryLateEntryCount++;
            }
        }
    }
    if (hwPhase < g_tG431IsrProbe.hwMinEntryPhase) {
        g_tG431IsrProbe.hwMinEntryPhase = hwPhase;
    }
    if (hwPhase > g_tG431IsrProbe.hwMaxEntryPhase) {
        g_tG431IsrProbe.hwMaxEntryPhase = hwPhase;
    }
    g_tG431IsrProbe.wPrevEntryCyc = wEntryCyc;
    g_tG431IsrProbe.wSamples = wSamples + 1U;
    return wEntryCyc;
#else
    return 0U;
#endif
}

/** @brief Record TIM1 phase and DWT cycle right after CCR1-3 write. */
static inline void mdi_g431_isr_probe_record_duty(void)
{
#if MDI_G431_PROBE_ACTIVE
    if (g_tG431IsrProbe.bIsrActive == 0U) {
        return;
    }
    uint16_t hwPhase = mdi_g431_tim1_phase_from_trig();
    uint32_t wDutyCyc = DWT->CYCCNT;
    uint32_t wEntryToDuty = wDutyCyc - g_tG431IsrProbe.wLastEntryCyc;

    g_tG431IsrProbe.wLastDutyCyc = wDutyCyc;
    g_tG431IsrProbe.hwLastDutyPhase = hwPhase;
    g_tG431IsrProbe.bDutyUpdated = 1U;
    if (hwPhase < g_tG431IsrProbe.hwMinDutyPhase) {
        g_tG431IsrProbe.hwMinDutyPhase = hwPhase;
    }
    if (hwPhase > g_tG431IsrProbe.hwMaxDutyPhase) {
        g_tG431IsrProbe.hwMaxDutyPhase = hwPhase;
    }
    if (wEntryToDuty > g_tG431IsrProbe.wMaxEntryToDutyCyc) {
        g_tG431IsrProbe.wMaxEntryToDutyCyc = wEntryToDuty;
    }
#endif
}

/** @brief Record DWT cycle right after the observer step completes. */
static inline void mdi_g431_isr_probe_record_observer(void)
{
#if MDI_G431_PROBE_ACTIVE
    uint32_t wObsCyc = DWT->CYCCNT;

    g_tG431IsrProbe.wLastObsCyc = wObsCyc;
    g_tG431IsrProbe.bObsUpdated = 1U;
    if (g_tG431IsrProbe.bDutyUpdated != 0U) {
        uint32_t wDutyToObs = wObsCyc - g_tG431IsrProbe.wLastDutyCyc;
        if (wDutyToObs > g_tG431IsrProbe.wMaxDutyToObsCyc) {
            g_tG431IsrProbe.wMaxDutyToObsCyc = wDutyToObs;
        }
    }
#endif
}

/** @brief Record outer ADC1_2_IRQHandler exit phase and total DWT cycles. */
static inline void mdi_g431_isr_probe_end(uint32_t wEntryCyc)
{
#if MDI_G431_PROBE_ACTIVE
    uint16_t hwExitPhase = mdi_g431_tim1_phase_from_trig();
    uint32_t wExitCyc = DWT->CYCCNT;
    uint32_t wExecCyc = wExitCyc - wEntryCyc;

    g_tG431IsrProbe.wLastExecCyc = wExecCyc;
    if (wExecCyc > g_tG431IsrProbe.wMaxExecCyc) {
        g_tG431IsrProbe.wMaxExecCyc = wExecCyc;
    }
    if (wExecCyc > G431_TIM1_FULL_PERIOD_TICKS) {
        g_tG431IsrProbe.wOverrunCount++;
    }
    if (hwExitPhase > g_tG431IsrProbe.hwMaxExitPhase) {
        g_tG431IsrProbe.hwMaxExitPhase = hwExitPhase;
    }
    if (g_tG431IsrProbe.bObsUpdated != 0U) {
        uint32_t wObsToExit = wExitCyc - g_tG431IsrProbe.wLastObsCyc;
        if (wObsToExit > g_tG431IsrProbe.wMaxObsToExitCyc) {
            g_tG431IsrProbe.wMaxObsToExitCyc = wObsToExit;
        }
    }
    g_tG431IsrProbe.bIsrActive = 0U;
#else
    (void)wEntryCyc;
#endif
}

/** @brief Calibrate probe self-overhead before enabling ADC interrupts. */
static inline void mdi_g431_isr_probe_init(void)
{
#if MDI_G431_PROBE_ACTIVE
    uint32_t wStart = 0U;
    uint32_t wEntry = 0U;
    uint32_t wOverhead = 0U;

    mdi_g431_isr_probe_reset(0U);
    wStart = DWT->CYCCNT;
    wEntry = mdi_g431_isr_probe_begin();
    mdi_g431_isr_probe_record_duty();
    mdi_g431_isr_probe_record_observer();
    mdi_g431_isr_probe_end(wEntry);
    wOverhead = DWT->CYCCNT - wStart;
    mdi_g431_isr_probe_reset(
        (uint16_t)(wOverhead <= UINT16_MAX ? wOverhead : UINT16_MAX));
#else
    mdi_g431_isr_probe_reset(0U);
#endif
}
#endif
