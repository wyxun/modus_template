/**
 * @file   port_mdi.c
 * @brief  AT32F407 MDI adapter — binds peripheral hardware to MDI objects
 *
 * USART1 interrupt-driven TX/RX ringbuf (same pattern as AT32F413).
 * GPIO wrappers for status LED.
 */

#include "mdi_hw.h"
#include "at32f403a_407.h"
#include "at32f403a_407_dma.h"
#include "mdi/mdi.h"
#include <stdbool.h>

#include "port_mdi.h"

/*============================================================================
 * AT32F407 USART1 adapter (optimized DMA-driven stream pattern)
 *===========================================================================*/
#define USART2_TX_BUFFER_SIZE  2048
#define USART2_RX_BUFFER_SIZE  256

#define USART_RXFLAG_IDLE   0
#define USART_RXFLAG_BUSY   1
#define USART_RXFLAG_FINISH 2

#define USART_TXFLAG_IDLE   0
#define USART_TXFLAG_BUSY   1

#define USART_DELAYTIME     5

static uint8_t s_chUsart2TxBuf[USART2_TX_BUFFER_SIZE];
static uint8_t s_chUsart2RxBuf[USART2_RX_BUFFER_SIZE];

/* DMA TX flat buffer and state */
static uint8_t s_chUsart2DmaTxBuf[512];
static volatile bool s_bTxDmaActive = false;
static uint32_t s_wRxReadPtr = 0;

/* ---- GPIO wrappers ---- */

static int32_t at32_gpio_Set(void *pPriv, mdi_gpio_level_t eLevel)
{
    void **ap = (void **)pPriv;
    gpio_bits_write((gpio_type *)ap[0], (uint16_t)(uintptr_t)ap[1], (confirm_state)eLevel);
    return 0;
}

static int32_t at32_gpio_Toggle(void *pPriv)
{
    void **ap = (void **)pPriv;
    gpio_bits_toggle((gpio_type *)ap[0], (uint16_t)(uintptr_t)ap[1]);
    return 0;
}

static int32_t at32_gpio_Get_ActiveLow(void *pPriv)
{
    void **ap = (void **)pPriv;
    uint16_t hwPins = (uint16_t)(uintptr_t)ap[1];
    return ((gpio_type *)ap[0])->idt & hwPins ? MDI_GPIO_LOW : MDI_GPIO_HIGH;
}

/* ---- LED GPIO instance (active-low, PD13 = AT-START-F407 LED2) ---- */

static void *s_apvLedStatPriv[] = { GPIOD, (void *)(uintptr_t)GPIO_PINS_13 };
static mdi_gpio_t s_tLedStatus = {
    .pPriv = s_apvLedStatPriv, .fnSet = at32_gpio_Set,
    .fnGet = at32_gpio_Get_ActiveLow, .fnToggle = at32_gpio_Toggle,
};

/* ---- USART2 DMA & Ringbuffer logic ---- */

at32_usart_priv_t s_tUsart2Priv = { .ptUsart = NULL };

void at32_usart_dma_init(void)
{
    dma_init_type dma_init_struct;

    // Enable DMA1 clock
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    // 1. Configure DMA1 Channel 3 for USART2 RX
    dma_reset(DMA1_CHANNEL3);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = USART2_RX_BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)s_chUsart2RxBuf;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART2->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = TRUE; // Circular mode!
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

    // Enable Half Transfer and Full Transfer interrupts for RX DMA
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_HDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_FDT_INT, TRUE);

    // Configure flexible DMA channel for USART2 RX
    dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_UART2_RX);

    // Enable DMA1 Channel 3 NVIC interrupt
    nvic_irq_enable(DMA1_Channel3_IRQn, 8, 1);

    // Enable DMA1 Channel 3
    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    // 2. Configure DMA1 Channel 4 for USART2 TX
    dma_reset(DMA1_CHANNEL4);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = 0; // Will be set dynamically
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)s_chUsart2DmaTxBuf;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART2->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE; // Normal mode
    dma_init(DMA1_CHANNEL4, &dma_init_struct);

    // Enable Full Transfer interrupt for TX DMA
    dma_interrupt_enable(DMA1_CHANNEL4, DMA_FDT_INT, TRUE);

    // Configure flexible DMA channel for USART2 TX
    dma_flexible_config(DMA1, FLEX_CHANNEL4, DMA_FLEXIBLE_UART2_TX);

    // Enable DMA1 Channel 4 NVIC interrupt
    nvic_irq_enable(DMA1_Channel4_IRQn, 8, 2);

    // Keep DMA1 Channel 4 disabled until we start transmitting
    dma_channel_enable(DMA1_CHANNEL4, FALSE);
}

void at32_usart_rx_dma_poll(void)
{
    if (s_tUsart2Priv.ptUsart == NULL) return;
    
    // Disable USART2 IRQ to prevent re-entrancy during extraction
    nvic_irq_disable(USART2_IRQn);
    
    uint16_t hwRemaining = dma_data_number_get(DMA1_CHANNEL3);
    uint32_t wWritePtr = USART2_RX_BUFFER_SIZE - hwRemaining;
    
    while (s_wRxReadPtr != wWritePtr) {
        uint8_t ch = s_chUsart2RxBuf[s_wRxReadPtr];
        s_wRxReadPtr = (s_wRxReadPtr + 1) % USART2_RX_BUFFER_SIZE;
        
        mringbuf_Write(&s_tUsart2Priv.tRxQueue, ch);
    }
    
    nvic_irq_enable(USART2_IRQn, 8, 0);
}

void at32_usart_tx_dma_start(void)
{
    nvic_irq_disable(DMA1_Channel4_IRQn);
    
    if (s_bTxDmaActive) {
        nvic_irq_enable(DMA1_Channel4_IRQn, 8, 2);
        return;
    }
    
    uint16_t hwCount = (uint16_t)mringbuf_GetUsed(&s_tUsart2Priv.tTxQueue);
    if (hwCount == 0) {
        nvic_irq_enable(DMA1_Channel4_IRQn, 8, 2);
        return;
    }
    
    if (hwCount > sizeof(s_chUsart2DmaTxBuf)) {
        hwCount = sizeof(s_chUsart2DmaTxBuf);
    }
    
    for (uint16_t i = 0; i < hwCount; i++) {
        mringbuf_Read(&s_tUsart2Priv.tTxQueue, &s_chUsart2DmaTxBuf[i]);
    }
    
    s_bTxDmaActive = true;
    
    dma_channel_enable(DMA1_CHANNEL4, FALSE);
    DMA1_CHANNEL4->dtcnt = hwCount;
    dma_channel_enable(DMA1_CHANNEL4, TRUE);
    
    nvic_irq_enable(DMA1_Channel4_IRQn, 8, 2);
}

void at32_usart_tx_dma_isr(void)
{
    s_bTxDmaActive = false;
    at32_usart_tx_dma_start();
}

void at32_usart2_init(void)
{
    if (s_tUsart2Priv.ptUsart != NULL) return;
    at32_usart_init(&s_tUsart2Priv, USART2,
                    s_chUsart2TxBuf, USART2_TX_BUFFER_SIZE,
                    s_chUsart2RxBuf, USART2_RX_BUFFER_SIZE);
    at32_usart_dma_init();
}

void at32_usart_init(at32_usart_priv_t *ptPriv,
                     usart_type *ptUsart,
                     uint8_t *pchTxBuf, uint32_t wTxBufSize,
                     uint8_t *pchRxBuf, uint32_t wRxBufSize)
{
    if (NULL == ptPriv || NULL == ptUsart) return;

    ptPriv->ptUsart = ptUsart;
    mringbuf_Init(&ptPriv->tTxQueue, pchTxBuf, (uint16_t)wTxBufSize);
    mringbuf_Init(&ptPriv->tRxQueue, pchRxBuf, (uint16_t)wRxBufSize);
    ptPriv->chTXFlag = USART_TXFLAG_IDLE;
    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    ptPriv->chRXFinishTime = 0;
}

void at32_usart_timer_1ms(at32_usart_priv_t *ptPriv)
{
    (void)ptPriv;
}

static int32_t at32_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    if (NULL == ptPriv || NULL == ptPriv->ptUsart) return 0;

    uint32_t i;
    for (i = 0; i < wLen; i++) {
        if (mringbuf_Write(&ptPriv->tTxQueue, pchData[i]) == 0) {
            break;
        }
    }

    at32_usart_tx_dma_start();
    return i;
}

static int32_t at32_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i = 0;
    uint16_t hwRead;

    at32_usart_rx_dma_poll();

    do {
        hwRead = mringbuf_Read(&ptPriv->tRxQueue, &pchBuf[i]);
        if (hwRead > 0) i++;
        if (i >= wLen) break;
    } while (hwRead > 0);

    return i;
}

static int32_t at32_stream_IsBusy(void *pPriv)
{
    (void)pPriv;
    return s_bTxDmaActive ? 1 : 0;
}

static mdi_stream_t s_tStreamSerial = {
    .pPriv    = &s_tUsart2Priv,
    .fnWrite  = at32_stream_Write,
    .fnRead   = at32_stream_Read,
    .fnIsBusy = at32_stream_IsBusy,
};

void at32_usart_irq_handler(at32_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv || NULL == ptPriv->ptUsart) return;
    usart_type *ptHW = ptPriv->ptUsart;

    /* 清除可能发生的溢出与帧错误，防止接收锁死 */
    if (usart_flag_get(ptHW, USART_ROERR_FLAG) != RESET ||
        usart_flag_get(ptHW, USART_FERR_FLAG) != RESET ||
        usart_flag_get(ptHW, USART_NERR_FLAG) != RESET ||
        usart_flag_get(ptHW, USART_PERR_FLAG) != RESET) {
        usart_flag_clear(ptHW, USART_ROERR_FLAG);
        usart_flag_clear(ptHW, USART_FERR_FLAG);
        usart_flag_clear(ptHW, USART_NERR_FLAG);
        usart_flag_clear(ptHW, USART_PERR_FLAG);
        (void)usart_data_receive(ptHW);
    }

    if (ptHW->ctrl1_bit.idleien != RESET) {
        if (usart_flag_get(ptHW, USART_IDLEF_FLAG) != RESET) {
            /* Clear IDLE flag by reading STS and then DT */
            (void)ptHW->sts;
            (void)ptHW->dt;
            at32_usart_rx_dma_poll();
        }
    }
}

/*============================================================================
 * Global hardware resource pool
 *===========================================================================*/

const mdi_hardware_t HW = {
    .ptLedStatus = &s_tLedStatus,   /* PD13 — primary heartbeat LED */
    .ptSerial    = &s_tStreamSerial,
};
