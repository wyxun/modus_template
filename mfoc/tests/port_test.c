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
#include "mfoc_driver.h"
#include "motor_port_fake.h"

static TIM_TypeDef s_tim;
static ADC_TypeDef s_adc1;
static ADC_TypeDef s_adc2;
static I2C_TypeDef s_i2c;
#undef TIM1
#define TIM1 (&s_tim)
#undef ADC1
#define ADC1 (&s_adc1)
#undef ADC2
#define ADC2 (&s_adc2)
#undef I2C1
#define I2C1 (&s_i2c)

#include "../port/src/mfoc_port.c"

volatile bool g_bG431JeosConsumed;
volatile uint32_t g_awG431AdcMean[G431_ADC_CHANNEL_COUNT];
volatile uint32_t g_wG431AdcSequence;
volatile bool g_bG431AdcMeanValid;
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
    assert(mfoc_port_pwm(NULL) == MFOC_RESULT_NULL);
    assert(TIM1->CCR1 == 101U && TIM1->CCR2 == 202U);
    assert(TIM1->CCR3 == 303U);
    TIM1->ARR = 0U;
    assert(mfoc_port_pwm(&duty) == MFOC_RESULT_OUT_OF_RANGE);
    assert(TIM1->CCR1 == 101U && TIM1->CCR3 == 303U);
    TIM1->ARR = 4250U;
    assert(mfoc_port_pwm(&duty) == MFOC_RESULT_OK);
    assert(TIM1->CCR1 == 0U && TIM1->CCR2 == 2125U);
    assert(TIM1->CCR3 == 4250U);
    duty = (mfoc_uvw_t){
        MFOC_SCALAR(0.25f), MFOC_HALF, MFOC_SCALAR(0.75f)};
    assert(mfoc_port_pwm(&duty) == MFOC_RESULT_OK);
    assert(TIM1->CCR1 == 1063U && TIM1->CCR2 == 2125U);
    assert(TIM1->CCR3 == 3188U);
    assert(TIM1->CCER == TIM_CCER_CC4E && TIM1->EGR == 0U);
    assert(TIM1->CR1 == TIM_CR1_CEN && TIM1->BDTR == TIM_BDTR_MOE);
}

/** @brief Run the real G431 MDI adapter against host register storage.
 * @return Zero when initialization, current and PWM checks pass. */
int main(void)
{
    _test_Encoder();
    _test_Current();
    _test_Range();
    _test_Pwm();
    puts("G431 current conversion and PWM passed");
    return 0;
}
