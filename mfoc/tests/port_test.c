/**
 * @file port_test.c
 * @brief G431 MDI port normalization and PWM register boundary checks.
 * @author Codex
 * @date 2026-10-21
 */
#include "stm32g431xx.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "mfoc_motor.h"
#include "motor_port_fake.h"

static TIM_TypeDef s_tim;
static ADC_TypeDef s_adc1;
static ADC_TypeDef s_adc2;
static I2C_TypeDef s_i2c;
static COMP_TypeDef s_comp1;
static COMP_TypeDef s_comp2;
static COMP_TypeDef s_comp4;
#undef TIM1
#define TIM1 (&s_tim)
#undef ADC1
#define ADC1 (&s_adc1)
#undef ADC2
#define ADC2 (&s_adc2)
#undef I2C1
#define I2C1 (&s_i2c)
#undef COMP1
#define COMP1 (&s_comp1)
#undef COMP2
#define COMP2 (&s_comp2)
#undef COMP4
#define COMP4 (&s_comp4)

static bool s_feed_current;

#include "../port/src/mfoc_port.c"

volatile bool g_bG431JeosConsumed;
volatile uint32_t g_awG431AdcMean[G431_ADC_CHANNEL_COUNT];
volatile uint32_t g_wG431AdcSequence;
volatile bool g_bG431AdcMeanValid;
volatile bool g_bG431FaultLatched;
static int64_t s_tick;

/** @brief Supply the raw timer for port linkage. @return Fake raw ticks. */
int64_t get_system_ticks(void) { return s_tick; }

/** @brief Supply the raw tick scale. @return One MHz clock. */
uint32_t perfc_get_systimer_frequency(void) { return 1000000U; }

/** @brief Advance the fake clock for the port's calibration delay.
 * @param ms Milliseconds. @return None. */
void perfc_delay_ms(uint32_t ms)
{
    s_tick += (int64_t)ms * INT64_C(1000);
    if (s_feed_current) {
        ADC1->JDR1 = 8192U;
        ADC2->JDR2 = 8200U;
        ADC2->JDR1 = 8100U;
        ADC1->ISR = ADC_ISR_JEOS;
        ADC2->ISR = ADC_ISR_JEOS;
        g_bG431JeosConsumed = false;
    }
}

/** @brief Check the real AS5600 MDI conversion and raw clock access.
 * @return None. */
static void _test_Encoder(void)
{
    mfoc_angle_t angle = {0U};
    assert(mfoc_port_encoder(NULL) == MFOC_RESULT_NULL);
    I2C1->ISR = I2C_ISR_TXIS | I2C_ISR_TC | I2C_ISR_RXNE | I2C_ISR_STOPF;
    I2C1->RXDR = 0xABU;
    assert(mfoc_port_encoder(&angle) == MFOC_RESULT_OK);
    assert(angle.wBam32 == UINT32_C(0xBAB00000));
    I2C1->ISR = I2C_ISR_NACKF;
    assert(mfoc_port_encoder(&angle) == MFOC_RESULT_IO);
    assert(angle.wBam32 == UINT32_C(0xBAB00000));
    s_tick = INT64_C(4294967299);
    assert(mfoc_port_tick() == 3U);
    assert(mfoc_port_tick_frequency() == 1000000U);
}

/** @brief Publish fake register values in the board's actual phase order.
 * @param u U count. @param v V count. @param w W count. @return None. */
static void _test_Frame(uint32_t u, uint32_t v, uint32_t w)
{
    ADC1->JDR1 = u;
    ADC2->JDR2 = v;
    ADC2->JDR1 = w;
    ADC1->ISR = ADC_ISR_JEOS;
    ADC2->ISR = ADC_ISR_JEOS;
    g_bG431JeosConsumed = false;
}

/** @brief Check backend accuracy against the ADC-to-PU formula.
 * @param actual Port value. @param expected Reference PU. @return None. */
static void _test_Near(mfoc_scalar_t actual, float expected)
{
#if defined(MFOC_NUMERIC_FIXED)
    float tolerance = 2.0f / 32768.0f + fabsf(expected) * 0.0000001f;
#else
    float tolerance = (1.0f + fabsf(expected)) * 0.000001f;
#endif
    assert(fabsf(mfoc_to_float(actual) - expected) <= tolerance);
}

/** @brief Install test conversion values without running blocking Init.
 * @param offsets Phase zero counts. @param scale ADC counts per Ibase.
 * @param inverted Current polarity. @return None. */
static void _test_SetPort(uint32_t u, uint32_t v, uint32_t w,
                          uint32_t scale, bool inverted)
{
    s_port.offset_u = u;
    s_port.offset_v = v;
    s_port.offset_w = w;
    s_port.configured = true;
#if defined(MFOC_NUMERIC_FIXED)
    s_port.gain_q30 = (int32_t)(MFOC_PORT_GAIN_SCALE / scale);
    if (inverted) { s_port.gain_q30 = -s_port.gain_q30; }
#else
    s_port.gain = 1.0f / (mfoc_scalar_t)scale;
    if (inverted) { s_port.gain = -s_port.gain; }
#endif
}

/** @brief Check missing frames, phase order, polarity and unclipped PU.
 * @return None. */
static void _test_Current(void)
{
    mfoc_uvw_t current = {MFOC_ONE, MFOC_ONE, MFOC_ONE};
    _test_SetPort(8192U, 8200U, 8100U, 476U, false);
    ADC1->ISR = 0U;
    g_bG431JeosConsumed = false;
    assert(mfoc_port_current(&current) == MFOC_RESULT_INVALID_STATE);
    assert(current.qU == MFOC_ONE && current.qW == MFOC_ONE);
    assert(!g_bG431JeosConsumed);
    _test_Frame(8668U, 7962U, 8100U);
    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
    _test_Near(current.qU, 1.0f);
    _test_Near(current.qV, -0.5f);
    _test_Near(current.qW, 0.0f);
    assert(g_bG431JeosConsumed);
    _test_SetPort(8192U, 8200U, 8100U, 476U, true);
    _test_Frame(8668U, 7962U, 8100U);
    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
    _test_Near(current.qU, -1.0f);
    _test_Near(current.qV, 0.5f);
    _test_SetPort(8192U, 8200U, 8100U, 476U, false);
    _test_Frame(9144U, 8200U, 8100U);
    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
    _test_Near(current.qU, 2.0f);
}

/** @brief Sweep native ADC range and conversion gains for both polarities.
 * @return None. */
static void _test_Range(void)
{
    const uint32_t scales[] = {1U, 119U, 476U, 16384U, 1048576U};
    mfoc_uvw_t current = {0};
    for (unsigned polarity = 0U; polarity < 2U; ++polarity) {
        for (unsigned i = 0U; i < sizeof(scales) / sizeof(scales[0]); ++i) {
            bool inverted = polarity != 0U;
            _test_SetPort(8192U, 8200U, 8100U, scales[i], inverted);
            for (uint32_t raw = 0U; raw <= 0x3FFFU; ++raw) {
                float sign = inverted ? -1.0f : 1.0f;
                float expected = sign * (float)((int32_t)raw - 8192) /
                    (float)scales[i];
                _test_Frame(raw, 8200U, 8100U);
                assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
                _test_Near(current.qU, expected);
                assert(current.qV == MFOC_ZERO && current.qW == MFOC_ZERO);
            }
        }
    }
}

/** @brief Convert trusted duties, reject bad timer state and retain gating.
 * @return None. */
static void _test_Pwm(void)
{
    mfoc_uvw_t duty = {MFOC_ZERO, MFOC_HALF, MFOC_ONE};
    TIM1->ARR = 4250U;
    TIM1->CCR1 = 101U;
    TIM1->CCR2 = 202U;
    TIM1->CCR3 = 303U;
    TIM1->CR1 = TIM_CR1_CEN;
    TIM1->CCER = TIM_CCER_CC4E;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->EGR = 0U;
    assert(TIM1->CCR1 == 101U && TIM1->CCR2 == 202U);
    assert(TIM1->CCR3 == 303U);
    TIM1->ARR = 0U;
    assert(mfoc_port_pwm(&duty, false) == MFOC_RESULT_OUT_OF_RANGE);
    assert(TIM1->CCR1 == 101U && TIM1->CCR3 == 303U);
    TIM1->ARR = 4250U;
    assert(mfoc_port_pwm(&duty, false) == MFOC_RESULT_OK);
    assert(TIM1->CCR1 == 0U && TIM1->CCR2 == 2125U);
    assert(TIM1->CCR3 == 4250U);
    duty = (mfoc_uvw_t){
        MFOC_SCALAR(0.25f), MFOC_HALF, MFOC_SCALAR(0.75f)};
    assert(mfoc_port_pwm(&duty, false) == MFOC_RESULT_OK);
    assert(TIM1->CCR1 == 1063U && TIM1->CCR2 == 2125U);
    assert(TIM1->CCR3 == 3188U);
    assert(TIM1->CCER == TIM_CCER_CC4E && TIM1->EGR == 0U);
    assert(TIM1->CR1 == TIM_CR1_CEN && TIM1->BDTR == TIM_BDTR_MOE);
}

/** @brief Reject unsafe calibration without consuming any current frame.
 * @return None. */
static void _test_InitGate(void)
{
    TIM1->CR1 = 0U;
    TIM1->BDTR = 0U;
    TIM1->CCER = TIM_CCER_CC1NE | TIM_CCER_CC4E;
    int64_t tick = s_tick;
    assert(mfoc_port_init() == MFOC_RESULT_INVALID_STATE);
    assert(s_tick == tick);
    TIM1->SR = TIM_SR_BIF;
    assert(mfoc_port_init() == MFOC_RESULT_INVALID_STATE);
    assert(s_tick == tick);
    TIM1->CCER = TIM_CCER_CC4E;
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    TIM1->SR = 0U;
}

/** @brief Verify stable bus reads and unchanged output on unavailable data.
 * @return None. */
static void _test_Bus(void)
{
    mfoc_scalar_t bus = MFOC_ONE;
    assert(mfoc_port_bus(NULL) == MFOC_RESULT_NULL);
    s_port.configured = false;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_INVALID_STATE);
    s_port.configured = true;
    g_bG431AdcMeanValid = false;
    g_wG431AdcSequence = 2U;
    g_awG431AdcMean[0] = 2457U;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_INVALID_STATE);
    assert(bus == MFOC_ONE);
    g_bG431AdcMeanValid = true;
    g_wG431AdcSequence = 3U;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_INVALID_STATE);
    assert(bus == MFOC_ONE);
    g_wG431AdcSequence = 4U;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_OK);
    _test_Near(bus, 20570.0f / 12000.0f);
    mfoc_scalar_t previous = bus;
    g_awG431AdcMean[0] = 0U;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_OUT_OF_RANGE);
    assert(bus == previous);
}

/** @brief Verify current-only calibration and bounded missing-frame failure.
 * @return None. */
static void _test_Init(void)
{
    TIM1->CCER = TIM_CCER_CC4E;
    g_bG431AdcMeanValid = false;
    s_feed_current = true;
    assert(mfoc_port_init() == MFOC_RESULT_OK);
    assert(!g_bG431AdcMeanValid);
    assert(s_port.offset_u == 8192U && s_port.offset_v == 8200U);
    assert(s_port.offset_w == 8100U);
    /* G431 legacy convention: positive U current lowers its ADC code. */
    _test_Frame(8173U, 8209U, 8110U);
    mfoc_uvw_t current = {0};
    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
    _test_Near(current.qU, 19.0f / 476.0f);
    _test_Near(current.qV, -9.0f / 476.0f);
    _test_Near(current.qW, -10.0f / 476.0f);
    s_feed_current = false;
    ADC1->ISR = 0U;
    int64_t start = s_tick;
    assert(mfoc_port_init() == MFOC_RESULT_IO);
    assert(s_tick - start == 10000);
    assert(s_port.configured && s_port.offset_u == 8192U);
    assert(TIM1->CCER == TIM_CCER_CC4E);
}

/** @brief Check polling-to-control handoff through existing bridge startup.
 * @return None. */
static void _test_ControlStartStop(void)
{
    const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
    TIM1->ARR = 4250U;
    TIM1->SR = 0U;
    COMP1->CSR = 0U;
    COMP2->CSR = 0U;
    COMP4->CSR = 0U;
    g_bG431FaultLatched = false;
    ADC1->IER = ADC_IER_OVRIE;
    assert(mfoc_port_pwm(&neutral, false) == MFOC_RESULT_OK);
    assert(ADC1->IER == ADC_IER_OVRIE);
    assert(mfoc_port_pwm(&neutral, true) == MFOC_RESULT_OK);
    assert((ADC1->IER & ADC_IER_JEOSIE) != 0U);
    assert(MDI_PWM_IsEnabled(bridge));
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(ADC1->IER == ADC_IER_OVRIE);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    assert((TIM1->CCER & TIM_CCER_CC4E) != 0U);
    COMP1->CSR = COMP_CSR_VALUE;
    assert(mfoc_port_pwm(&neutral, true) == MFOC_RESULT_SAFETY);
    assert(ADC1->IER == ADC_IER_OVRIE);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    COMP1->CSR = 0U;
}

/** @brief Exercise real lifecycle adapters against the MDI register provider.
 * @return None. */
static void _test_Lifecycle(void)
{
    const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
    g_bG431AdcMeanValid = false; /* Startup no longer reads the bus. */
    g_bG431FaultLatched = true;
    assert(mfoc_port_pwm(&neutral, true) == MFOC_RESULT_SAFETY);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    g_bG431FaultLatched = false;
    TIM1->ARR = 0U;
    assert(mfoc_port_pwm(&neutral, true) == MFOC_RESULT_OUT_OF_RANGE);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    TIM1->ARR = 4250U;
    assert(mfoc_port_pwm(&neutral, true) == MFOC_RESULT_OK);
    assert(TIM1->CCR1 == 2125U && TIM1->CCR2 == 2125U);
    assert(TIM1->CCR3 == 2125U && MDI_PWM_IsEnabled(bridge));
    assert(!g_bG431AdcMeanValid);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(TIM1->CCER == TIM_CCER_CC4E);
    assert((TIM1->CR1 & TIM_CR1_CEN) != 0U);
    assert((TIM1->BDTR & TIM_BDTR_MOE) != 0U);
    assert(mfoc_port_pwm(&neutral, false) == MFOC_RESULT_OK);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
}

/** @brief Run the real G431 MDI adapter against host register storage.
 * @return Zero when initialization, current and PWM checks pass. */
int main(void)
{
    _test_Encoder();
    _test_Current();
    _test_Range();
    _test_Pwm();
    _test_InitGate();
    _test_Bus();
    _test_Init();
    _test_ControlStartStop();
    _test_Lifecycle();
    puts("G431 current conversion and PWM passed");
    return 0;
}
