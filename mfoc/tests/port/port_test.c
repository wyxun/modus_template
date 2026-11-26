/**
 * @file port_test.c
 * @brief G431 MDI port normalization and PWM register boundary checks.
 * @author wyxun
 * @date 2026-10-21
 */
#include "stm32g431xx.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mfoc_port.h"
#include "mfoc_numeric.h"

/* Run the real board acknowledgement against host interrupt state. */
static uint32_t s_primask;
#define __get_PRIMASK() (s_primask)
#define __disable_irq() ((void)(s_primask = 1U))
#define __set_PRIMASK(mask) ((void)(s_primask = (mask)))

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
static const mfoc_port_cfg_t s_cfg = {
    .current_base_milliamp = 3500U,
    .voltage_base_millivolt = 12000U,
    .electrical_speed_base_hz = MFOC_SCALAR(400.0f),
    .pole_pairs = 7U,
};

#include "../../port/projects/g431_2804/board/mfoc_port.c"
#include "../../../peripheral/stm32g431/mdi/state.c"
#include "../../../peripheral/stm32g431/mdi/service.c"

#if MFOC_DEBUG_POSITION_WAVEFORM
#include "../../../class/mfoc_debug.c"
mfoc_app_t tMfocApp;
static bool s_accept_wave;
static unsigned s_clock_reads;
static void _test_WaveStep(void)
{
    if (s_accept_wave) { mfoc_debug_CaptureWaveform(); }
}
const mwaveform_api_t mwaveform = {.Step = _test_WaveStep};
#endif
static int64_t s_tick;

/** @brief Supply the raw timer for port linkage. @return Fake raw ticks. */
int64_t get_system_ticks(void)
{
#if MFOC_DEBUG_POSITION_WAVEFORM
    ++s_clock_reads;
#endif
    return s_tick;
}

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
    uint16_t raw = 0U;
    assert(mdi_g431_encoder_ReadRaw(NULL) == MDI_INVALID);
    I2C1->ISR = I2C_ISR_TXIS | I2C_ISR_TC | I2C_ISR_RXNE | I2C_ISR_STOPF;
    I2C1->RXDR = 0xABU;
    assert(mdi_g431_encoder_ReadRaw(&raw) == MDI_OK);
    assert(raw == UINT16_C(0xBAB));
    I2C1->ISR = I2C_ISR_NACKF;
    assert(mdi_g431_encoder_ReadRaw(&raw) == MDI_IO_ERROR);
    assert(raw == UINT16_C(0xBAB));
    s_tick = INT64_C(4294967299);
    assert(mfoc_port_tick() == UINT64_C(4294967299));
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
    s_port.voltage_base_millivolt = 12000U;
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
    const mfoc_uvw_t previous = {MFOC_ONE, MFOC_HALF, -MFOC_ONE};
    mfoc_uvw_t current = previous;
    s_port.configured = false;
    _test_Frame(8668U, 7962U, 8100U);
    assert(mfoc_port_current(&current) == MFOC_RESULT_INVALID_STATE);
    assert(memcmp(&current, &previous, sizeof(current)) == 0);
    assert(!g_bG431JeosConsumed);
    _test_SetPort(8192U, 8200U, 8100U, 476U, false);
    ADC1->ISR = 0U;
    assert(mfoc_port_current(&current) == MFOC_RESULT_INVALID_STATE);
    assert(memcmp(&current, &previous, sizeof(current)) == 0);
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

/** @brief Compare one phase bit for bit with the original 64-bit formula.
 * @param actual Port output. @param raw ADC count. @param offset Zero count.
 * @return None. */
static void _test_CurrentBits(mfoc_scalar_t actual, uint32_t raw,
                              uint32_t offset)
{
    int64_t delta = (int64_t)raw - (int64_t)offset;
#if defined(MFOC_NUMERIC_FIXED)
    mfoc_scalar_t expected = (mfoc_scalar_t)(delta * (int64_t)s_port.gain_q30 /
                                             INT64_C(32768));
#else
    mfoc_scalar_t expected = (mfoc_scalar_t)delta * s_port.gain;
#endif
    assert(memcmp(&actual, &expected, sizeof(expected)) == 0);
}

/** @brief Sweep every signed 14-bit ADC difference on all three channels.
 * @return None. */
static void _test_Range(void)
{
    const uint32_t max_count = 0x3FFFU;
    const uint32_t scales[] = {1U, 119U, 476U, 16384U, 1048576U};
    mfoc_uvw_t current = {0};
    for (unsigned polarity = 0U; polarity < 2U; ++polarity) {
        for (unsigned i = 0U; i < sizeof(scales) / sizeof(scales[0]); ++i) {
            bool inverted = polarity != 0U;
            float sign = inverted ? -1.0f : 1.0f;
            /* The two offset endpoints cover both signs on every phase. */
            for (unsigned end = 0U; end < 2U; ++end) {
                uint32_t offset = end == 0U ? 0U : max_count;
                uint32_t v_offset = max_count - offset;
                _test_SetPort(offset, v_offset, offset, scales[i], inverted);
                for (uint32_t u = 0U; u <= max_count; ++u) {
                    uint32_t v = max_count - u;
                    uint32_t w = (u + 1U) & max_count;
                    int32_t delta = (int32_t)u - (int32_t)offset;
                    _test_Frame(u, v, w);
                    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
                    _test_CurrentBits(current.qU, u, offset);
                    _test_CurrentBits(current.qV, v, v_offset);
                    _test_CurrentBits(current.qW, w, offset);
                    _test_Near(current.qU,
                               sign * (float)delta / (float)scales[i]);
                }
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

/** @brief Reject unsafe calibration without consuming any current frame.
 * @return None. */
static void _test_InitGate(void)
{
    TIM1->CR1 = 0U;
    TIM1->BDTR = 0U;
    TIM1->CCER = TIM_CCER_CC1NE | TIM_CCER_CC4E;
    int64_t tick = s_tick;
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_INVALID_STATE);
    assert(s_tick == tick);
    TIM1->SR = TIM_SR_BIF;
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_INVALID_STATE);
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
    int64_t before = s_tick;
    I2C1->ISR = I2C_ISR_NACKF;
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_IO);
    assert(s_tick == before && TIM1->CCER == TIM_CCER_CC4E);
    I2C1->ISR = I2C_ISR_TXIS | I2C_ISR_TC | I2C_ISR_RXNE | I2C_ISR_STOPF;
    /* Constant RXDR cannot echo the write: reject SF readback mismatch. */
    I2C1->RXDR = 0xA8U;
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_SAFETY);
    assert(s_tick == before && TIM1->CCER == TIM_CCER_CC4E);
    /* SF=3 with nonzero unrelated fields must also pass unchanged. */
    I2C1->RXDR = 0xABU;
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_OK);
    I2C1->RXDR = 0x03U;
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_OK);
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
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_IO);
    assert(s_tick - start == 10000);
    assert(s_port.configured && s_port.offset_u == 8192U);
    assert(TIM1->CCER == TIM_CCER_CC4E);
}

/** @brief Check explicit current and bus bases without Motor configuration.
 * @return None. */
static void _test_Bases(void)
{
    mfoc_port_cfg_t cfg = s_cfg;
    assert(mfoc_port_init(NULL) == MFOC_RESULT_NULL);
    cfg.current_base_milliamp = 0U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_INVALID_ARGUMENT);
    cfg = s_cfg;
    cfg.voltage_base_millivolt = 0U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_INVALID_ARGUMENT);
    cfg = s_cfg;
    cfg.pole_pairs = 0U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_INVALID_ARGUMENT);
    cfg = s_cfg;
    cfg.electrical_speed_base_hz = MFOC_ZERO;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_INVALID_ARGUMENT);
    cfg = s_cfg;
    cfg.current_base_milliamp = 7000U;
    cfg.voltage_base_millivolt = 24000U;
    s_feed_current = true;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_OK);
    _test_Frame(7716U, 8200U, 8100U);
    mfoc_uvw_t current = {0};
    assert(mfoc_port_current(&current) == MFOC_RESULT_OK);
    _test_Near(current.qU, 0.5f);
    g_bG431AdcMeanValid = true;
    g_wG431AdcSequence = 4U;
    g_awG431AdcMean[0] = 2457U;
    mfoc_scalar_t bus = MFOC_ZERO;
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_OK);
    _test_Near(bus, 20570.0f / 24000.0f);
    cfg.voltage_base_millivolt = 0U;
    assert(mfoc_port_init(&cfg) == MFOC_RESULT_INVALID_ARGUMENT);
    assert(mfoc_port_bus(&bus) == MFOC_RESULT_OK);
    _test_Near(bus, 20570.0f / 24000.0f);
    /* Finish Init just before the next low-32-bit wrap. */
    s_tick = INT64_C(8589834296);
    assert(mfoc_port_init(&s_cfg) == MFOC_RESULT_OK);
    s_feed_current = false;
}

/** @brief Verify foreground acquisition and ISR output share one instance.
 * @return None. */
static void _test_Position(void)
{
    mfoc_angle_t angle = {UINT32_C(0x12345678)};
    mfoc_scalar_t speed = MFOC_ONE;
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_INVALID_STATE);
    assert(angle.wBam32 == UINT32_C(0x12345678) && speed == MFOC_ONE);
    I2C1->RXDR = 0U;
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == 0U && speed == MFOC_ZERO);
    I2C1->RXDR = 1U;
    s_tick += 999;
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == 0U && speed == MFOC_ZERO);
    s_tick += 1;
    assert(mfoc_port_position_run() == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == UINT32_C(0x70700000));
    float expected = (257.0f / 4096.0f) * 1000.0f * 0.25f * 7.0f / 400.0f;
    assert(fabsf(mfoc_to_float(speed) - expected) < 0.001f);
    mfoc_scalar_t previous_speed = speed;
    s_tick += 500;
    I2C1->ISR = I2C_ISR_NACKF;
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 > UINT32_C(0x70700000));
    assert(speed == previous_speed); /* ISR performs no I2C access. */
    s_tick += 500;
    assert(mfoc_port_position_run() == MFOC_RESULT_IO);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    mfoc_angle_t previous_angle = angle;
    s_tick += INT64_C(4294967296);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_SAFETY);
    assert(angle.wBam32 == previous_angle.wBam32 && speed == previous_speed);
    I2C1->ISR = I2C_ISR_TXIS | I2C_ISR_TC | I2C_ISR_RXNE | I2C_ISR_STOPF;
    mfoc_angle_t held = {UINT32_C(0x40000000)};
    assert(mfoc_port_position_calibrate_zero(held) == MFOC_RESULT_OK);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == held.wBam32 && speed == MFOC_ZERO);
    I2C1->ISR = I2C_ISR_NACKF;
    held.wBam32 = 0U;
    assert(mfoc_port_position_calibrate_zero(held) == MFOC_RESULT_IO);
    assert(mfoc_port_position(&angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == UINT32_C(0x40000000) && speed == MFOC_ZERO);
}

#if MFOC_DEBUG_POSITION_WAVEFORM
/** @brief Compare Feedback and captured position at the expiry boundary,
 * then skip a frame and exercise the diagnostic-only read during hold. */
static void _test_PositionWave(int32_t mechanical_delta)
{
    mfoc_encoder_t *encoder = &s_port.encoder;
    mfoc_angle_t zero = {0U};
    mfoc_angle_t moved = {(uint32_t)mechanical_delta};
    assert(mfoc_encoder_CalibrateZero(encoder, zero, zero, s_tick) ==
           MFOC_RESULT_OK);
    s_tick += 1000;
    assert(mfoc_encoder_Update(encoder, moved, s_tick) == MFOC_RESULT_OK);
    s_tick += encoder->timeout_ticks;
    tMfocApp.motor.state = MFOC_MOTOR_RUNNING;
    mfoc_feedback_t *feedback = &tMfocApp.motor.feedback;
    mfoc_feedback_output_t output = {0};
    mfoc_feedback_cfg_t cfg = {0};
    mfoc_motor_params_t model = {0};
    assert(mfoc_feedback_Init(feedback, &cfg, &model) == MFOC_RESULT_OK);
    s_clock_reads = 0U;
    s_accept_wave = true;
    assert(mfoc_feedback_Step(feedback, NULL, &output) == MFOC_RESULT_OK);
    assert(output.speed * (float)mechanical_delta > 0.0f);
    ++s_tick; /* A second extrapolation here would time out. */
    mfoc_debug_WaveformStep();
    assert(s_clock_reads == 1U);
    assert(s_waveform[MFOC_DEBUG_ENCODER_ANGLE] ==
           mfoc_angle_to_turns(output.angle));

    assert(mfoc_feedback_Step(feedback, NULL, &output) == MFOC_RESULT_SAFETY);
    /* A fresh foreground publication cannot repair the frame just read. */
    assert(mfoc_encoder_Update(encoder, zero, s_tick) == MFOC_RESULT_OK);
    mfoc_debug_WaveformStep();
    assert(s_waveform[MFOC_DEBUG_ENCODER_ANGLE] == -1.0f);
    assert(s_clock_reads == 2U);

    s_accept_wave = false;
    assert(mfoc_feedback_Step(feedback, NULL, &output) == MFOC_RESULT_OK);
    mfoc_debug_WaveformStep();
    assert(s_clock_reads == 3U);
    tMfocApp.motor.hold_active = true;
    ++s_tick;
    assert(mfoc_encoder_Update(encoder, moved, s_tick) == MFOC_RESULT_OK);
    s_accept_wave = true;
    mfoc_debug_WaveformStep(); /* Hold skips Feedback. Read once on demand. */
    assert(s_clock_reads == 4U);
    assert(s_waveform[MFOC_DEBUG_ENCODER_ANGLE] !=
           mfoc_angle_to_turns(output.angle));
    assert(s_waveform[MFOC_DEBUG_OBSERVER_ANGLE] == -1.0f &&
           s_waveform[MFOC_DEBUG_ANGLE_ERROR] == -1.0f);
    tMfocApp.motor.hold_active = false;
}
#endif

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
#if MFOC_RUNTIME_BUS
    mfoc_board_bus_Reset();
    g_bG431AdcMeanValid = false;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_INVALID_STATE);
    assert(!MDI_PWM_IsEnabled(bridge));
    g_wG431AdcSequence += 2U;
    g_bG431AdcMeanValid = true;
    g_awG431AdcMean[0] = 1400U;
    g_g431MeanStarted = s_tick;
    mfoc_board_bus_sample_t sample;
    assert(mfoc_board_bus_Poll(&sample) && sample.started == (uint64_t)s_tick);
    assert(mfoc_board_bus_Publish(&sample) == MFOC_RESULT_OK);
    assert(!mfoc_board_bus_Poll(&sample));
    s_tick += 30000;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_SAFETY);
    g_eG431AdcStatus = MDI_IO_ERROR;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_IO);
    g_eG431AdcStatus = MDI_OK;
    g_wG431AdcSequence += 2U;
    g_g431MeanStarted = s_tick;
#endif
    assert(mfoc_port_pwm(&neutral) == MFOC_RESULT_OK);
    assert(ADC1->IER == ADC_IER_OVRIE);
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_OK);
    assert((ADC1->IER & ADC_IER_JEOSIE) != 0U);
    assert(MDI_PWM_IsEnabled(bridge));
#if MFOC_RUNTIME_BUS
    mfoc_core_t control = {0};
    assert(mfoc_board_bus_Apply(&control) == MFOC_RESULT_OK);
    assert(mfoc_core_GetBus(&control) > MFOC_ZERO);
#endif
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(ADC1->IER == ADC_IER_OVRIE);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    assert((TIM1->CCER & TIM_CCER_CC4E) != 0U);
#if MFOC_RUNTIME_BUS
    /* Same published DMA mean, 29.9 ms old on re-arm at 20 kHz. */
    s_tick += 29900;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_OK);
    s_tick += 50;
    assert(mfoc_board_bus_Apply(&control) == MFOC_RESULT_OK);
    s_tick += 50;
    assert(mfoc_board_bus_Apply(&control) == MFOC_RESULT_SAFETY);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_SAFETY);
    assert(!MDI_PWM_IsEnabled(bridge));
    g_wG431AdcSequence += 2U;
    g_g431MeanStarted = s_tick;
#endif
    COMP1->CSR = COMP_CSR_VALUE;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_SAFETY);
    assert(ADC1->IER == ADC_IER_OVRIE);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    COMP1->CSR = 0U;
    /* Source released with the hardware history still latched. */
    TIM1->SR = TIM_SR_BIF;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_SAFETY);
    (void)mfoc_port_pwm(&neutral);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert((TIM1->SR & TIM_SR_BIF) != 0U);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    assert(MDI_PWM_ClearFault(bridge) == MDI_OK);
    assert(s_primask == 0U);
    COMP2->CSR = COMP_CSR_VALUE; /* Fault recurs after explicit clear. */
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_SAFETY);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    COMP2->CSR = 0U;
}

/** @brief Exercise real lifecycle adapters against the MDI register provider.
 * @return None. */
static void _test_Lifecycle(void)
{
    const mfoc_uvw_t neutral = {MFOC_HALF, MFOC_HALF, MFOC_HALF};
    g_bG431AdcMeanValid = false; /* Startup no longer reads the bus. */
    g_bG431FaultLatched = true;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_SAFETY);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    g_bG431FaultLatched = false;
    TIM1->ARR = 0U;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_OUT_OF_RANGE);
    assert(!MDI_OP(bridge, _pwm_AnyOutputEnabled)());
    TIM1->ARR = 4250U;
    assert(mfoc_port_pwm_start(&neutral) == MFOC_RESULT_OK);
    assert(TIM1->CCR1 == 2125U && TIM1->CCR2 == 2125U);
    assert(TIM1->CCR3 == 2125U && MDI_PWM_IsEnabled(bridge));
    assert(!g_bG431AdcMeanValid);
    assert(mfoc_port_pwm_stop() == MFOC_RESULT_OK);
    assert(TIM1->CCER == TIM_CCER_CC4E);
    assert((TIM1->CR1 & TIM_CR1_CEN) != 0U);
    assert((TIM1->BDTR & TIM_BDTR_MOE) != 0U);
    assert(mfoc_port_pwm(&neutral) == MFOC_RESULT_OK);
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
    _test_Bases();
    _test_Position();
#if MFOC_DEBUG_POSITION_WAVEFORM
    _test_PositionWave(INT32_C(0x01000000));
    _test_PositionWave(-INT32_C(0x01000000));
#endif
    _test_ControlStartStop();
    _test_Lifecycle();
    puts("G431 bases, current, position, clock and PWM passed");
    return 0;
}
