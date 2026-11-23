#ifndef TEST_AT32F413_H
#define TEST_AT32F413_H
#include <stdbool.h>
#include <stdint.h>

extern uint32_t SystemCoreClock;

#define ADC_PCCE_FLAG UINT32_C(4)
#define DMA1_FDT1_FLAG UINT32_C(2)
#define DMA1_DTERR1_FLAG UINT32_C(8)
#define TMR_BRK_FLAG UINT32_C(128)
#define TMR_OVERFLOW_SWTRIG UINT32_C(1)

typedef struct {
    volatile uint32_t sts;
    struct { volatile unsigned ocswtrg; } ctrl2_bit;
    volatile uint32_t pdt1, pdt2, pdt3;
} test_adc_t;
typedef struct {
    volatile uint32_t sts, clr;
} test_dma_t;
typedef struct {
    struct { volatile unsigned oen; } brk_bit;
    struct { volatile uint32_t cval; } cval_bit;
    volatile uint32_t c1dt, c2dt, c3dt;
    volatile uint32_t ists, swevt, iden;
} test_timer_t;
typedef test_timer_t tmr_type;

extern test_adc_t test_adc;
extern test_dma_t test_dma;
extern test_timer_t test_timer;
#define ADC1 (&test_adc)
#define DMA1 (&test_dma)
#define TMR1 (&test_timer)
#if AT32F413_BOARD_HALL
extern test_timer_t test_hall_timer;
#define TMR3 (&test_hall_timer)
#define TMR_C1_FLAG 2U
#define TMR_C1_INT 2U
#define TMR_C1_RECAPTURE_FLAG 512U
#define TMR_SELECT_CHANNEL_1 0U
uint32_t tmr_channel_value_get(tmr_type *tmr, unsigned channel);
static inline void tmr_flag_clear(tmr_type *tmr, uint32_t flags)
{
    tmr->ists &= ~flags;
}
static inline void tmr_interrupt_enable(tmr_type *tmr, uint32_t irq,
                                        bool enable)
{
    if (enable) { tmr->iden |= irq; } else { tmr->iden &= ~irq; }
}
#endif

typedef struct {
    volatile uint32_t idt, scr;
} test_gpio_t;
typedef struct {
    uint32_t gpio_pins;
    unsigned gpio_mode;
    unsigned gpio_pull;
} gpio_init_type;

extern test_gpio_t test_gpio;
#define GPIOA (&test_gpio)
#define GPIOB (&test_gpio)
#define GPIOC (&test_gpio)
#define GPIO_PINS_0 (1U << 0U)
#define GPIO_PINS_4 (1U << 4U)
#define GPIO_PINS_5 (1U << 5U)
#define GPIO_PINS_12 (1U << 12U)

#define USART_RDBF_FLAG UINT32_C(0x20)
#define USART_ROERR_FLAG UINT32_C(0x08)
#define USART_TDC_FLAG UINT32_C(0x40)

typedef struct {
    volatile uint32_t sts, dt, baudr;
    struct { volatile unsigned uen, ten, ren, tdcien, rdbfien; } ctrl1_bit;
} usart_type;

extern usart_type test_usart;
#define USART1 (&test_usart)

enum {
    CRM_IOMUX_PERIPH_CLOCK, CRM_GPIOB_PERIPH_CLOCK, SWJTAG_GMUX_010,
    GPIO_MODE_INPUT, GPIO_PULL_NONE, USART1_IRQn
};
#define TRUE true
#define FALSE false

static inline void crm_periph_clock_enable(unsigned clock, bool enabled)
{
    (void)clock;
    (void)enabled;
}

static inline void gpio_pin_remap_config(unsigned pins, bool enabled)
{
    (void)pins;
    (void)enabled;
}

static inline void gpio_default_para_init(gpio_init_type *cfg)
{
    *cfg = (gpio_init_type){0};
}

static inline void gpio_init(test_gpio_t *port, const gpio_init_type *cfg)
{
    (void)port;
    (void)cfg;
}

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
static inline void NVIC_ClearPendingIRQ(unsigned irq) { (void)irq; }
static inline void nvic_irq_enable(unsigned irq, unsigned priority,
                                   unsigned subpriority)
{
    (void)irq;
    (void)priority;
    (void)subpriority;
}

#endif
