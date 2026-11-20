#ifndef TEST_AT32F413_H
#define TEST_AT32F413_H
#include <stdbool.h>
#include <stdint.h>

extern uint32_t SystemCoreClock;

#define ADC_PCCE_FLAG UINT32_C(4)
typedef struct {
    volatile uint32_t sts;
} test_adc_t;
typedef struct {
    struct { volatile unsigned oen; } brk_bit;
} test_timer_t;

extern test_adc_t test_adc;
extern test_timer_t test_timer;
#define ADC1 (&test_adc)
#define TMR1 (&test_timer)

typedef struct {
    volatile uint32_t idt;
} test_gpio_t;
typedef struct {
    uint32_t gpio_pins;
    unsigned gpio_mode;
    unsigned gpio_pull;
} gpio_init_type;

extern test_gpio_t test_gpio;
#define GPIOB (&test_gpio)
#define GPIO_PINS_0 (1U << 0U)
#define GPIO_PINS_4 (1U << 4U)
#define GPIO_PINS_5 (1U << 5U)

enum {
    CRM_IOMUX_PERIPH_CLOCK, CRM_GPIOB_PERIPH_CLOCK, SWJTAG_GMUX_010,
    GPIO_MODE_INPUT, GPIO_PULL_NONE
};
#define TRUE true

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

#endif
