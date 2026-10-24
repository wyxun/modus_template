/**
 * @file mfoc_hfi.c
 * @brief Four-interval saliency demodulation and opposite-pulse polarity.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_hfi.h"
#include <stddef.h>

/* BAM32 phase gain: phase increment per speed PU per ISR sample. */
#if defined(MFOC_NUMERIC_FIXED)
#define MFOC_HFI_PHASE_GAIN \
    (INT64_C(4294967296) * MFOC_HFI_SPEED_BASE_HZ / MFOC_HFI_SAMPLE_HZ)
#else
#define MFOC_HFI_PHASE_GAIN \
    (4294967296.0f * MFOC_HFI_SPEED_BASE_HZ / MFOC_HFI_SAMPLE_HZ)
#endif

#define MFOC_HFI_STARTUP_SAMPLES \
    (MFOC_HFI_SAMPLE_HZ * MFOC_HFI_STARTUP_MS / 1000U)

#if MFOC_HFI_ENABLE_POLARITY
#define MFOC_HFI_REST_SAMPLES \
    (MFOC_HFI_SAMPLE_HZ * MFOC_HFI_REST_MS / 1000U)
#define MFOC_HFI_POLARITY_MIN_VOLTAGE_SUM \
    (MFOC_HFI_POLARITY_VOLTAGE * \
     (mfoc_scalar_t)MFOC_HFI_POLARITY_SAMPLES / 2)

#if defined(MFOC_NUMERIC_FIXED)
#define MFOC_HFI_POLARITY_CURRENT_SQ_MAX \
    ((int64_t)MFOC_HFI_POLARITY_CURRENT_MAX * \
     MFOC_HFI_POLARITY_CURRENT_MAX)
#else
#define MFOC_HFI_POLARITY_CURRENT_SQ_MAX \
    (MFOC_HFI_POLARITY_CURRENT_MAX * MFOC_HFI_POLARITY_CURRENT_MAX)
#endif
#endif

#if defined(MFOC_NUMERIC_FIXED)
#define MFOC_HFI_SPEED_INTEGRAL_MAX \
    ((int64_t)MFOC_HFI_SPEED_MAX * MFOC_Q_SCALE)
#endif

/* Products stay wide until normalization: small HF signals would disappear
 * if each Q30 product were truncated to Q15 before the subtraction. With the
 * documented sample bounds, products and Q15 ratio numerators fit int64_t. */
#if defined(MFOC_NUMERIC_FIXED)
typedef int64_t mfoc_hfi_product_t;
#define MFOC_HFI_RATIO_SCALE MFOC_Q_SCALE
#else
typedef float mfoc_hfi_product_t;
#define MFOC_HFI_RATIO_SCALE 1.0f
#endif

/** @brief Clear the electrical estimate.
 * @param hfi Valid instance. @return None. */
void mfoc_hfi_Init(mfoc_hfi_t *hfi)
{
    *hfi = (mfoc_hfi_t){0};
}

/** @brief Extract the d axis modulo pi from two independent excitations.
 * @param hfi Completed carrier cycle. @param axis Estimated axis.
 * @return True when excitation and measured saliency are sufficient.
 * @note Di = Ts * inverse(L) * Du after opposite-pulse differencing.
 * Slow fundamental voltage/back-EMF approximately cancel within each pair.
 * Form Di*adj(Du); its trace and anisotropy share the same positive scale.
 * atan2(Gab+Gba, Gaa-Gbb) = 2*theta for Ld < Lq. */
static bool _mfoc_hfi_Axis(const mfoc_hfi_t *hfi, mfoc_angle_t *axis)
{
    const mfoc_ab_t *i = hfi->delta_i;
    const mfoc_ab_t *u = hfi->delta_u;
    mfoc_hfi_product_t det = (mfoc_hfi_product_t)u[0].qAlpha * u[1].qBeta -
        (mfoc_hfi_product_t)u[1].qAlpha * u[0].qBeta;
    mfoc_scalar_t response = mfoc_abs(i[0].qAlpha) + mfoc_abs(i[0].qBeta) +
        mfoc_abs(i[1].qAlpha) + mfoc_abs(i[1].qBeta);
    if (det < (mfoc_hfi_product_t)MFOC_HFI_VOLTAGE * MFOC_HFI_VOLTAGE ||
        response < MFOC_HFI_RESPONSE_MIN) { return false; }
    mfoc_hfi_product_t aa = (mfoc_hfi_product_t)i[0].qAlpha * u[1].qBeta -
        (mfoc_hfi_product_t)i[1].qAlpha * u[0].qBeta;
    mfoc_hfi_product_t bb = (mfoc_hfi_product_t)i[1].qBeta * u[0].qAlpha -
        (mfoc_hfi_product_t)i[0].qBeta * u[1].qAlpha;
    mfoc_hfi_product_t ab = (mfoc_hfi_product_t)i[1].qAlpha * u[0].qAlpha -
        (mfoc_hfi_product_t)i[0].qAlpha * u[1].qAlpha;
    mfoc_hfi_product_t ba = (mfoc_hfi_product_t)i[0].qBeta * u[1].qBeta -
        (mfoc_hfi_product_t)i[1].qBeta * u[0].qBeta;
    mfoc_hfi_product_t x = (aa - bb) * MFOC_HFI_SALIENCY_SIGN;
    mfoc_hfi_product_t y = (ab + ba) * MFOC_HFI_SALIENCY_SIGN;
    mfoc_hfi_product_t magnitude = (x < 0 ? -x : x) + (y < 0 ? -y : y);
    mfoc_hfi_product_t trace = aa + bb;
    if (aa <= 0 || bb <= 0 || magnitude <= 0 || magnitude > 2 * trace ||
        (mfoc_scalar_t)(magnitude * MFOC_HFI_RATIO_SCALE / trace) <
        MFOC_HFI_SALIENCY_MIN) {
        return false;
    }
    mfoc_scalar_t y_unit = (mfoc_scalar_t)(y * MFOC_HFI_RATIO_SCALE / magnitude);
    mfoc_scalar_t x_unit = (mfoc_scalar_t)(x * MFOC_HFI_RATIO_SCALE / magnitude);
    *axis = mfoc_angle_atan2(y_unit, x_unit);
    axis->wBam32 >>= 1U;
    return true;
}

/** @brief Correct the phase predictor and integrate electrical speed.
 * @param hfi Tracker. @param axis Measured axis modulo pi. @return None.
 * @note Gains apply once per four samples; error is electrical turns. */
static void _mfoc_hfi_Track(mfoc_hfi_t *hfi, mfoc_angle_t axis)
{
    if (!hfi->has_angle) {
        hfi->angle = axis;
        hfi->has_angle = true;
    }
    mfoc_scalar_t error = mfoc_angle_diff(axis, hfi->angle);
    if (error > MFOC_SCALAR(0.25f)) { error -= MFOC_HALF; }
    if (error < MFOC_SCALAR(-0.25f)) { error += MFOC_HALF; }
    hfi->angle = mfoc_angle_add_scalar(hfi->angle,
        mfoc_mul_wide(MFOC_HFI_PLL_KP, error));
#if defined(MFOC_NUMERIC_FIXED)
    hfi->speed_integral += (int64_t)MFOC_HFI_PLL_KI * error;
    if (hfi->speed_integral > MFOC_HFI_SPEED_INTEGRAL_MAX) {
        hfi->speed_integral = MFOC_HFI_SPEED_INTEGRAL_MAX;
    }
    if (hfi->speed_integral < -MFOC_HFI_SPEED_INTEGRAL_MAX) {
        hfi->speed_integral = -MFOC_HFI_SPEED_INTEGRAL_MAX;
    }
    hfi->speed = (mfoc_scalar_t)(hfi->speed_integral / MFOC_Q_SCALE);
#else
    hfi->speed = mfoc_sat(hfi->speed + MFOC_HFI_PLL_KI * error,
                          -MFOC_HFI_SPEED_MAX, MFOC_HFI_SPEED_MAX);
#endif
    if (mfoc_abs(error) <= MFOC_HFI_LOCK_ERROR) {
        if (hfi->lock_count < MFOC_HFI_LOCK_CYCLES) { ++hfi->lock_count; }
    } else {
        hfi->lock_count = 0U;
    }
    if (hfi->state == MFOC_HFI_ACQUIRE &&
        hfi->lock_count == MFOC_HFI_LOCK_CYCLES) {
#if MFOC_HFI_ENABLE_POLARITY
        if (mfoc_abs(hfi->speed) >= MFOC_HFI_REST_SPEED) { return; }
        mfoc_angle_sincos(hfi->angle, &hfi->polarity.axis.qBeta,
                           &hfi->polarity.axis.qAlpha);
        hfi->speed = MFOC_ZERO;
#if defined(MFOC_NUMERIC_FIXED)
        hfi->speed_integral = 0;
#endif
        hfi->lock_count = 0U;
        hfi->state = MFOC_HFI_SETTLE_POSITIVE;
#else
        hfi->state = MFOC_HFI_TRACK;
#endif
    }
}

/** @brief Pair current increments with the carrier actually issued last Step.
 * @param hfi Demodulator. @param i Present current. @param u Prior voltage.
 * @return None. One matrix extraction per four intervals. */
static void _mfoc_hfi_Demodulate(mfoc_hfi_t *hfi, const mfoc_ab_t *i,
    const mfoc_ab_t *u)
{
    mfoc_ab_t di = {i->qAlpha - hfi->previous_i.qAlpha,
                    i->qBeta - hfi->previous_i.qBeta};
    uint8_t axis = hfi->carrier / 2U;
    if ((hfi->carrier & 1U) == 0U) {
        hfi->delta_i[axis] = di;
        hfi->delta_u[axis] = *u;
    } else {
        hfi->delta_i[axis].qAlpha -= di.qAlpha;
        hfi->delta_i[axis].qBeta -= di.qBeta;
        hfi->delta_u[axis].qAlpha -= u->qAlpha;
        hfi->delta_u[axis].qBeta -= u->qBeta;
    }
    hfi->carrier = (uint8_t)((hfi->carrier + 1U) & 3U);
    if (hfi->carrier == 0U) {
        mfoc_angle_t angle = {0U};
        if (_mfoc_hfi_Axis(hfi, &angle)) {
            _mfoc_hfi_Track(hfi, angle);
        } else {
            hfi->lock_count = 0U;
            if (hfi->state == MFOC_HFI_TRACK) {
                hfi->state = MFOC_HFI_FAILED;
            }
        }
    }
}

#if MFOC_HFI_ENABLE_POLARITY
/** @brief Compare voltage-normalized pulse responses and retain agreeing votes.
 * @param hfi Pulse history. @param response Negative-pulse current excursion.
 * @return None; ambiguous/opposing evidence latches failure. */
static void _mfoc_hfi_PolarityCompare(mfoc_hfi_t *hfi,
    mfoc_scalar_t response)
{
    mfoc_hfi_polarity_t *p = &hfi->polarity;
    mfoc_hfi_product_t positive = (mfoc_hfi_product_t)p->positive_i * p->u_sum;
    mfoc_hfi_product_t negative = (mfoc_hfi_product_t)response * p->positive_u;
    mfoc_scalar_t contrast = (mfoc_scalar_t)((positive - negative) *
        MFOC_HFI_RATIO_SCALE / (positive + negative));
    /* Similar delivered volt-seconds are required: normalization cannot
     * repair a cancelled/clipped pulse or large resistive-waveform changes. */
    mfoc_scalar_t mismatch = (mfoc_scalar_t)(
        (mfoc_hfi_product_t)mfoc_abs(p->positive_u - p->u_sum) *
        MFOC_HFI_RATIO_SCALE / (p->positive_u + p->u_sum));
    if (mismatch > MFOC_SCALAR(0.10f) ||
        mfoc_abs(contrast) < MFOC_HFI_POLARITY_CONTRAST) {
        hfi->state = MFOC_HFI_FAILED;
        return;
    }
    int8_t vote = contrast * MFOC_HFI_POLARITY_SIGN > MFOC_ZERO ? 1 : -1;
    if (p->pairs != 0U && vote != p->vote) {
        hfi->state = MFOC_HFI_FAILED;
        return;
    }
    p->vote = vote;
    ++p->pairs;
    if (p->pairs < MFOC_HFI_POLARITY_PAIRS) {
        hfi->state = MFOC_HFI_SETTLE_POSITIVE;
    } else {
        if (vote < 0) { hfi->angle.wBam32 += UINT32_C(0x80000000); }
        hfi->state = MFOC_HFI_SETTLE_TRACK;
    }
}

/** @brief Advance nonblocking rest/pulse phases using ISR sample counts.
 * @param hfi Polarity state. @param i Present current. @param u Prior voltage.
 * @return None. The rotor must remain stationary throughout this sequence. */
static void _mfoc_hfi_PolarityStep(mfoc_hfi_t *hfi, const mfoc_ab_t *i,
    const mfoc_ab_t *u)
{
    mfoc_hfi_polarity_t *p = &hfi->polarity;
    mfoc_scalar_t id = mfoc_mul_wide(i->qAlpha, p->axis.qAlpha) +
        mfoc_mul_wide(i->qBeta, p->axis.qBeta);
    bool positive = hfi->state == MFOC_HFI_PULSE_POSITIVE;
    bool pulsing = positive || hfi->state == MFOC_HFI_PULSE_NEGATIVE;
    if (!pulsing) {
        if (p->count < MFOC_HFI_REST_SAMPLES) { ++p->count; }
        if (p->count < MFOC_HFI_REST_SAMPLES ||
            mfoc_abs(i->qAlpha) + mfoc_abs(i->qBeta) > MFOC_HFI_REST_CURRENT ||
            mfoc_abs(u->qAlpha) + mfoc_abs(u->qBeta) > MFOC_HFI_REST_VOLTAGE) {
            return;
        }
        p->count = 0U;
        p->i_start = id;
        p->u_sum = MFOC_ZERO;
        if (hfi->state == MFOC_HFI_SETTLE_TRACK) {
            hfi->state = MFOC_HFI_TRACK;
            hfi->carrier = 0U;
        } else {
            hfi->state = hfi->state == MFOC_HFI_SETTLE_POSITIVE ?
                MFOC_HFI_PULSE_POSITIVE : MFOC_HFI_PULSE_NEGATIVE;
        }
        return;
    }
    mfoc_scalar_t ud = mfoc_mul_wide(u->qAlpha, p->axis.qAlpha) +
        mfoc_mul_wide(u->qBeta, p->axis.qBeta);
    if (!positive) { ud = -ud; }
    mfoc_hfi_product_t current_sq = (mfoc_hfi_product_t)i->qAlpha * i->qAlpha +
        (mfoc_hfi_product_t)i->qBeta * i->qBeta;
    if (ud <= MFOC_ZERO || current_sq >
        MFOC_HFI_POLARITY_CURRENT_SQ_MAX) {
        hfi->state = MFOC_HFI_FAILED;
        return;
    }
    p->u_sum += ud;
    ++p->count;
    if (p->count < MFOC_HFI_POLARITY_SAMPLES) { return; }
    mfoc_scalar_t response = positive ? id - p->i_start : p->i_start - id;
    if (response < MFOC_HFI_RESPONSE_MIN ||
        p->u_sum < MFOC_HFI_POLARITY_MIN_VOLTAGE_SUM) {
        hfi->state = MFOC_HFI_FAILED;
        return;
    }
    p->count = 0U;
    if (positive) {
        p->positive_i = response;
        p->positive_u = p->u_sum;
        hfi->state = MFOC_HFI_SETTLE_NEGATIVE;
    } else {
        _mfoc_hfi_PolarityCompare(hfi, response);
    }
}
#endif

/** @brief Produce the next carrier or polarity pulse, zero during rest/failure.
 * @param hfi Current sequence. @return Stationary alpha-beta voltage, V/Vbase. */
static mfoc_ab_t _mfoc_hfi_Injection(const mfoc_hfi_t *hfi)
{
    mfoc_ab_t u = {0};
    if (hfi->state == MFOC_HFI_ACQUIRE || hfi->state == MFOC_HFI_TRACK) {
        mfoc_scalar_t amplitude = (hfi->carrier & 1U) == 0U ?
            MFOC_HFI_VOLTAGE : -MFOC_HFI_VOLTAGE;
        if (hfi->carrier < 2U) { u.qAlpha = amplitude; }
        else { u.qBeta = amplitude; }
#if MFOC_HFI_ENABLE_POLARITY
    } else if (hfi->state == MFOC_HFI_PULSE_POSITIVE ||
               hfi->state == MFOC_HFI_PULSE_NEGATIVE) {
        mfoc_scalar_t amplitude = hfi->state == MFOC_HFI_PULSE_POSITIVE ?
            MFOC_HFI_POLARITY_VOLTAGE : -MFOC_HFI_POLARITY_VOLTAGE;
        u.qAlpha = mfoc_mul_wide(amplitude, hfi->polarity.axis.qAlpha);
        u.qBeta = mfoc_mul_wide(amplitude, hfi->polarity.axis.qBeta);
#endif
    } else { /* Rest or failed: only this module's injection is removed. */ }
    return u;
}

/** @brief Consume the preceding interval before issuing the next injection.
 * @param hfi Instance. @param i_ab_k Present current, I/Ibase.
 * @param u_previous Prior-interval voltage, V/Vbase; NULL at startup.
 * @param output Electrical estimates and next injection. @return None. */
void mfoc_hfi_Step(mfoc_hfi_t *hfi, const mfoc_ab_t *i_ab_k,
    const mfoc_ab_t *u_previous, mfoc_hfi_output_t *output)
{
    if (u_previous == NULL) { mfoc_hfi_Init(hfi); }
    if (hfi->state != MFOC_HFI_TRACK && hfi->state != MFOC_HFI_FAILED) {
        ++hfi->startup_count;
        if (hfi->startup_count > MFOC_HFI_STARTUP_SAMPLES) {
            hfi->state = MFOC_HFI_FAILED;
        }
    }
    if (hfi->has_current && u_previous != NULL) {
        if (hfi->state == MFOC_HFI_ACQUIRE || hfi->state == MFOC_HFI_TRACK) {
            /* Integrate in BAM32, not Q15 turns, to retain low-speed motion. */
#if defined(MFOC_NUMERIC_FIXED)
            int32_t delta = (int32_t)((int64_t)hfi->speed *
                MFOC_HFI_PHASE_GAIN / MFOC_Q_SCALE);
#else
            int32_t delta = (int32_t)(hfi->speed *
                                      MFOC_HFI_PHASE_GAIN);
#endif
            hfi->angle.wBam32 += (uint32_t)delta;
            _mfoc_hfi_Demodulate(hfi, i_ab_k, u_previous);
#if MFOC_HFI_ENABLE_POLARITY
        } else if (hfi->state != MFOC_HFI_FAILED) {
            _mfoc_hfi_PolarityStep(hfi, i_ab_k, u_previous);
#endif
        } else { /* Failure is latched until the next Init. */ }
    }
    hfi->previous_i = *i_ab_k;
    hfi->has_current = true;
    *output = (mfoc_hfi_output_t){
        .angle = hfi->angle, .speed = hfi->speed,
        .u_inject_ab = _mfoc_hfi_Injection(hfi),
        .valid = hfi->state == MFOC_HFI_TRACK &&
                 hfi->lock_count == MFOC_HFI_LOCK_CYCLES};
}
