/**
 * @file   port_mdi.c
 * @brief  AT32F413 MDI adapter — binds peripheral hardware to MDI objects
 *
 * Follows the AT32F421 pattern (USART ringbuf) + STM32G431-style
 * PWM/ADC/comparator wrappers for FOC.
 */

#include "mdi_hw.h"
#include "at32f413.h"
#include "mdi/legacy/mdi.h"

#include "port_mdi.h"
#include "halpwm.h"
#include "haladc.h"

/*============================================================================
 * AT32F413 USART1 adapter (same ringbuf pattern as AT32F421)
 *===========================================================================*/
#define USART1_TX_BUFFER_SIZE  256
#define USART1_RX_BUFFER_SIZE  256

#define USART_RXFLAG_IDLE   0
#define USART_RXFLAG_BUSY   1
#define USART_RXFLAG_FINISH 2

#define USART_TXFLAG_IDLE   0
#define USART_TXFLAG_BUSY   1

#define USART_DELAYTIME     5

static uint8_t s_chUsart1TxBuf[USART1_TX_BUFFER_SIZE];
static uint8_t s_chUsart1RxBuf[USART1_RX_BUFFER_SIZE];

/* ---- GPIO wrappers ---- */

static int32_t at32_gpio_Set(void *pPriv, mdi_gpio_level_t eLevel)
{
    void **ap = (void **)pPriv;
    gpio_bits_write((gpio_type *)ap[0], (uint16_t)(uintptr_t)ap[1], (confirm_state)eLevel);
    return 0;
}

static int32_t at32_gpio_Get(void *pPriv)
{
    void **ap = (void **)pPriv;
    uint16_t hwPins = (uint16_t)(uintptr_t)ap[1];
    return ((gpio_type *)ap[0])->idt & hwPins ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
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

/* ---- LED GPIO instances (active-low) ---- */

static void *s_apvLedErrorPriv[]   = { GPIOB, (void *)(uintptr_t)GPIO_PINS_9 };
static void *s_apvLedStat1Priv[]   = { GPIOC, (void *)(uintptr_t)GPIO_PINS_13 };
static void *s_apvLedStat2Priv[]   = { GPIOC, (void *)(uintptr_t)GPIO_PINS_14 };
static void *s_apvLedStat3Priv[]   = { GPIOC, (void *)(uintptr_t)GPIO_PINS_15 };

static mdi_gpio_t s_tLedError = {
    .pPriv = s_apvLedErrorPriv, .fnSet = at32_gpio_Set,
    .fnGet = at32_gpio_Get_ActiveLow, .fnToggle = at32_gpio_Toggle,
};
static mdi_gpio_t s_tLedStatus1 = {
    .pPriv = s_apvLedStat1Priv, .fnSet = at32_gpio_Set,
    .fnGet = at32_gpio_Get_ActiveLow, .fnToggle = at32_gpio_Toggle,
};
static mdi_gpio_t s_tLedStatus2 = {
    .pPriv = s_apvLedStat2Priv, .fnSet = at32_gpio_Set,
    .fnGet = at32_gpio_Get_ActiveLow, .fnToggle = at32_gpio_Toggle,
};
static mdi_gpio_t s_tLedStatus3 = {
    .pPriv = s_apvLedStat3Priv, .fnSet = at32_gpio_Set,
    .fnGet = at32_gpio_Get_ActiveLow, .fnToggle = at32_gpio_Toggle,
};

/* ---- Button GPIO instances (active-low but we read physical level) ---- */
static void *s_apvButtonStartPriv[] = { GPIOA, (void *)(uintptr_t)GPIO_PINS_12 };
static mdi_gpio_t s_tButtonStart = {
    .pPriv = s_apvButtonStartPriv, .fnSet = NULL,
    .fnGet = at32_gpio_Get, .fnToggle = NULL,
};

/* ---- Comp (brake) GPIO instance (input only) ---- */
static void *s_apvCompBrkPriv[] = { GPIOB, (void *)(uintptr_t)GPIO_PINS_12 };
static mdi_gpio_t s_tCompBrk = {
    .pPriv = s_apvCompBrkPriv, .fnSet = NULL, .fnGet = at32_gpio_Get,
};

/* ---- PWM instances (duty via TMR1 CH1-3, enable/disable via halpwm) ---- */

static int32_t pwm_setduty(void *pPriv, uint32_t wDuty)
{
    uint32_t wChannel = (uint32_t)(uintptr_t)pPriv;
    if (wChannel == 1U)      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, wDuty);
    else if (wChannel == 2U) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, wDuty);
    else if (wChannel == 3U) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, wDuty);
    return 0;
}

int32_t port_mdi_MotorPwmSetDuty3(uint32_t wDutyU,
                                  uint32_t wDutyV,
                                  uint32_t wDutyW)
{
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, wDutyU);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, wDutyV);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, wDutyW);
    return 0;
}

static int32_t pwm_enable(void *pPriv, bool bEn)
{
    (void)pPriv;
    if (bEn) halpwm_Start();
    else     halpwm_Stop();
    return 0;
}

static mdi_pwm_t s_tPwmU = { .pPriv = (void *)1U, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };
static mdi_pwm_t s_tPwmV = { .pPriv = (void *)2U, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };
static mdi_pwm_t s_tPwmW = { .pPriv = (void *)3U, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };

/* ---- ADC instances (read via haladc) ---- */

static int32_t adc_read_busv(void *pPriv)
{
    (void)pPriv;
    haladc_StartRegular();
    return (int32_t)haladc_GetOrdinary(HALADC_ORD_BUS_VOLT);
}

static int32_t adc_read_temp(void *pPriv)
{
    (void)pPriv;
    return (int32_t)haladc_GetOrdinary(HALADC_ORD_MOS_TEMP);
}

static mdi_adc_t s_tAdcCurrU = { .pPriv = NULL, .fnRead = NULL };  /* via FOC ops */
static mdi_adc_t s_tAdcCurrV = { .pPriv = NULL, .fnRead = NULL };
static mdi_adc_t s_tAdcCurrW = { .pPriv = NULL, .fnRead = NULL };
static mdi_adc_t s_tAdcBusV = { .pPriv = NULL, .fnRead = adc_read_busv };
static mdi_adc_t s_tAdcTemp = { .pPriv = NULL, .fnRead = adc_read_temp };

/* ---- USART ringbuf logic (reused from AT32F421) ---- */

at32_usart_priv_t s_tUsart1Priv = { .ptUsart = NULL };

void at32_usart1_init(void)
{
    if (s_tUsart1Priv.ptUsart != NULL) return;
    at32_usart_init(&s_tUsart1Priv, USART1,
                    s_chUsart1TxBuf, USART1_TX_BUFFER_SIZE,
                    s_chUsart1RxBuf, USART1_RX_BUFFER_SIZE);
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
    if (NULL == ptPriv) return;
    if (ptPriv->chRXFlag == USART_RXFLAG_BUSY) {
        if (ptPriv->chRXFinishTime > 0) {
            ptPriv->chRXFinishTime--;
        } else {
            ptPriv->chRXFlag = USART_RXFLAG_FINISH;
        }
    }
}

static int32_t at32_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i;

    for (i = 0; i < wLen; i++) {
        if (mringbuf_Write(&ptPriv->tTxQueue, pchData[i]) == 0) break;
    }

    if (ptPriv->chTXFlag == USART_TXFLAG_IDLE
        && mringbuf_GetUsed(&ptPriv->tTxQueue) > 0) {
        uint8_t chData;
        if (mringbuf_Read(&ptPriv->tTxQueue, &chData) == 1) {
            usart_data_transmit(ptPriv->ptUsart, chData);
            ptPriv->chTXFlag = USART_TXFLAG_BUSY;
        }
    }
    return i;
}

static int32_t at32_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i = 0;
    uint16_t hwRead;

    if (ptPriv->chRXFlag != USART_RXFLAG_FINISH) return 0;

    do {
        hwRead = mringbuf_Read(&ptPriv->tRxQueue, &pchBuf[i]);
        if (hwRead > 0) i++;
        if (i >= wLen) break;
    } while (hwRead > 0);

    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    return i;
}

static int32_t at32_stream_IsBusy(void *pPriv)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    return (ptPriv->chTXFlag == USART_TXFLAG_BUSY) ? 1 : 0;
}

static mdi_stream_t s_tStreamSerial = {
    .pPriv    = &s_tUsart1Priv,
    .fnWrite  = at32_stream_Write,
    .fnRead   = at32_stream_Read,
    .fnIsBusy = at32_stream_IsBusy,
};

void at32_usart_irq_handler(at32_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv || NULL == ptPriv->ptUsart) return;
    uint8_t chData;
    usart_type *ptHW = ptPriv->ptUsart;

    if (ptHW->ctrl1_bit.rdbfien != RESET) {
        if (usart_interrupt_flag_get(ptHW, USART_RDBF_FLAG) != RESET) {
            uint8_t ch = usart_data_receive(ptHW);
            mringbuf_Write(&ptPriv->tRxQueue, ch);
            ptPriv->chRXFinishTime = USART_DELAYTIME;
            ptPriv->chRXFlag = USART_RXFLAG_BUSY;
        }
    }

    if (ptHW->ctrl1_bit.tdcien != RESET) {
        if (usart_flag_get(ptHW, USART_TDC_FLAG) != RESET) {
            usart_flag_clear(ptHW, USART_TDC_FLAG);
            if (mringbuf_Read(&ptPriv->tTxQueue, &chData) == 1) {
                usart_data_transmit(ptHW, chData);
                ptPriv->chTXFlag = USART_TXFLAG_BUSY;
            } else {
                ptPriv->chTXFlag = USART_TXFLAG_IDLE;
            }
        }
    }
}

/*============================================================================
 * Global hardware resource pool
 *===========================================================================*/

const mdi_hardware_t HW = {
    .ptLedStatus  = &s_tLedStatus1,   /* PC13 — primary heartbeat LED */
    .ptLedError   = &s_tLedError,
    .ptLedStatus2 = &s_tLedStatus2,
    .ptLedStatus3 = &s_tLedStatus3,
    .ptCompBrk    = &s_tCompBrk,
    .ptMotorU     = &s_tPwmU,
    .ptMotorV     = &s_tPwmV,
    .ptMotorW     = &s_tPwmW,
    .ptAdcCurrU   = &s_tAdcCurrU,
    .ptAdcCurrV   = &s_tAdcCurrV,
    .ptAdcCurrW   = &s_tAdcCurrW,
    .ptAdcBusV    = &s_tAdcBusV,
    .ptAdcTemp    = &s_tAdcTemp,
    .ptSerial     = &s_tStreamSerial,
    .ptButtonStart = &s_tButtonStart,
};
