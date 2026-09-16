#include "foc_identify_minimal.h"

#include <stddef.h>

static bool identify_is_finite(float value)
{
    return value == value && value <= 3.4e38f && value >= -3.4e38f;
}

static float identify_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static bool identify_is_active(const foc_identify_t *ctx)
{
    return ctx->status != FOC_IDENTIFY_STATUS_IDLE &&
           ctx->status != FOC_IDENTIFY_STATUS_DONE &&
           ctx->status != FOC_IDENTIFY_STATUS_ERROR;
}

static void identify_set_zero_command(foc_identify_t *ctx)
{
    ctx->command.vd_pu = 0.0f;
    ctx->command.vq_pu = 0.0f;
    ctx->command.pwm_enable = false;
}

static void identify_fail(foc_identify_t *ctx, foc_identify_error_t error)
{
    ctx->status = FOC_IDENTIFY_STATUS_ERROR;
    ctx->error = error;
    ctx->result.error = error;
    ctx->result.valid = false;
    identify_set_zero_command(ctx);
}

static bool identify_config_valid(const foc_identify_config_t *config)
{
    return config != NULL &&
           identify_is_finite(config->v_rs_low_pu) &&
           identify_is_finite(config->v_rs_high_pu) &&
           identify_is_finite(config->v_ld_pu) &&
           identify_is_finite(config->v_lq_pu) &&
           identify_is_finite(config->current_limit_pu) &&
           identify_is_finite(config->zero_current_limit_pu) &&
           identify_is_finite(config->min_delta_i_pu) &&
           identify_is_finite(config->sample_time_s) &&
           config->v_rs_low_pu >= 0.0f &&
           config->v_rs_high_pu > config->v_rs_low_pu &&
           config->v_ld_pu > 0.0f &&
           config->v_lq_pu > 0.0f &&
           config->current_limit_pu > 0.0f &&
           config->zero_current_limit_pu > 0.0f &&
           config->zero_current_limit_pu < config->current_limit_pu &&
           config->min_delta_i_pu > 0.0f &&
           config->sample_time_s > 0.0f &&
           config->zero_max_ticks >= config->zero_min_ticks &&
           config->zero_confirm_samples > 0u &&
           config->average_samples > 0u &&
           config->pulse_samples > 1u &&
           config->task_sample_budget > 0u;
}

static void identify_reset_accumulators(foc_identify_t *ctx)
{
    ctx->stage_sample_count = 0u;
    ctx->sum_current = 0.0f;
    ctx->sum_voltage = 0.0f;
    ctx->inductance_sum = 0.0f;
    ctx->inductance_initial_current = 0.0f;
    ctx->inductance_previous_current = 0.0f;
    ctx->inductance_has_previous = false;
    ctx->zero_confirm_count = 0u;
}

static void identify_set_command(
    foc_identify_t *ctx,
    float vd_pu,
    float vq_pu,
    bool pwm_enable)
{
    ctx->command.vd_pu = vd_pu;
    ctx->command.vq_pu = vq_pu;
    ctx->command.pwm_enable = pwm_enable;
    ++ctx->command_sequence;
    ctx->stage_command_sequence = ctx->command_sequence;
}

static void identify_enter_stage(
    foc_identify_t *ctx,
    foc_identify_status_t status,
    uint32_t tick)
{
    ctx->status = status;
    ctx->stage_start_tick = tick;
    ctx->stage_clock_started = true;
    identify_reset_accumulators(ctx);

    switch (status) {
    case FOC_IDENTIFY_STATUS_RS_LOW:
        identify_set_command(ctx, ctx->config.v_rs_low_pu, 0.0f, true);
        break;
    case FOC_IDENTIFY_STATUS_RS_HIGH:
        identify_set_command(ctx, ctx->config.v_rs_high_pu, 0.0f, true);
        break;
    case FOC_IDENTIFY_STATUS_LD:
        identify_set_command(ctx, ctx->config.v_ld_pu, 0.0f, true);
        break;
    case FOC_IDENTIFY_STATUS_LQ:
        identify_set_command(ctx, 0.0f, ctx->config.v_lq_pu, true);
        break;
    case FOC_IDENTIFY_STATUS_ZERO:
        identify_set_command(ctx, 0.0f, 0.0f, true);
        break;
    case FOC_IDENTIFY_STATUS_DONE:
        ctx->result.valid = true;
        identify_set_zero_command(ctx);
        break;
    default:
        break;
    }
}

static void identify_enter_zero(
    foc_identify_t *ctx,
    foc_identify_status_t next_status,
    uint32_t tick)
{
    ctx->zero_next_status = next_status;
    identify_enter_stage(ctx, FOC_IDENTIFY_STATUS_ZERO, tick);
}

static bool identify_elapsed_reached(
    const foc_identify_t *ctx,
    const foc_identify_sample_t *sample,
    uint32_t ticks)
{
    return (uint32_t)(sample->tick - ctx->stage_start_tick) >= ticks;
}

static bool identify_current_valid(
    const foc_identify_t *ctx,
    const foc_identify_sample_t *sample)
{
    float current_squared;

    if (sample->fault || !identify_is_finite(sample->id_pu) ||
        !identify_is_finite(sample->iq_pu) || !identify_is_finite(sample->applied_vd_pu) ||
        !identify_is_finite(sample->applied_vq_pu)) {
        return false;
    }

    current_squared = (sample->id_pu * sample->id_pu) +
                      (sample->iq_pu * sample->iq_pu);
    return identify_is_finite(current_squared) &&
           current_squared <=
               (ctx->config.current_limit_pu * ctx->config.current_limit_pu);
}

static bool identify_pop_sample(
    foc_identify_t *ctx,
    foc_identify_sample_t *sample)
{
    uint16_t read_index = ctx->queue_read;

    if (read_index == ctx->queue_write) {
        return false;
    }

    *sample = ctx->sample_queue[read_index];
    ctx->queue_read = (uint16_t)((read_index + 1u) %
                                 FOC_IDENTIFY_SAMPLE_QUEUE_SIZE);
    return true;
}

static void identify_process_zero(
    foc_identify_t *ctx,
    const foc_identify_sample_t *sample)
{
    float current_squared = (sample->id_pu * sample->id_pu) +
                            (sample->iq_pu * sample->iq_pu);
    float zero_limit_squared =
        ctx->config.zero_current_limit_pu *
        ctx->config.zero_current_limit_pu;

    if (!identify_elapsed_reached(ctx, sample, ctx->config.zero_min_ticks)) {
        return;
    }

    if (identify_elapsed_reached(ctx, sample, ctx->config.zero_max_ticks) &&
        current_squared > zero_limit_squared) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_ZERO_TIMEOUT);
        return;
    }

    if (current_squared <= zero_limit_squared) {
        ++ctx->zero_confirm_count;
    } else {
        ctx->zero_confirm_count = 0u;
    }

    if (ctx->zero_confirm_count >= ctx->config.zero_confirm_samples) {
        if (ctx->zero_next_status == FOC_IDENTIFY_STATUS_DONE) {
            identify_enter_stage(ctx, FOC_IDENTIFY_STATUS_DONE, sample->tick);
        } else {
            identify_enter_stage(ctx, ctx->zero_next_status, sample->tick);
        }
    }
}

static void identify_process_resistance(
    foc_identify_t *ctx,
    const foc_identify_sample_t *sample)
{
    float delta_current;
    float delta_voltage;

    if (!identify_elapsed_reached(ctx, sample, ctx->config.settle_ticks)) {
        return;
    }

    ctx->sum_current += sample->id_pu;
    ctx->sum_voltage += sample->applied_vd_pu;
    ++ctx->stage_sample_count;

    if (ctx->stage_sample_count < ctx->config.average_samples) {
        return;
    }

    if (ctx->status == FOC_IDENTIFY_STATUS_RS_LOW) {
        ctx->rs_current_low =
            ctx->sum_current / (float)ctx->stage_sample_count;
        ctx->rs_voltage_low =
            ctx->sum_voltage / (float)ctx->stage_sample_count;
        identify_enter_stage(ctx, FOC_IDENTIFY_STATUS_RS_HIGH, sample->tick);
        return;
    }

    ctx->rs_current_high =
        ctx->sum_current / (float)ctx->stage_sample_count;
    ctx->rs_voltage_high =
        ctx->sum_voltage / (float)ctx->stage_sample_count;
    delta_current = ctx->rs_current_high - ctx->rs_current_low;
    delta_voltage = ctx->rs_voltage_high - ctx->rs_voltage_low;

    if (!identify_is_finite(delta_current) || !identify_is_finite(delta_voltage) ||
        identify_abs(delta_current) < ctx->config.min_delta_i_pu) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_SMALL_DELTA_I);
        return;
    }

    ctx->result.resistance_pu = delta_voltage / delta_current;
    if (!identify_is_finite(ctx->result.resistance_pu) ||
        ctx->result.resistance_pu <= 0.0f) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_NUMERIC);
        return;
    }

    identify_enter_zero(ctx, FOC_IDENTIFY_STATUS_LD, sample->tick);
}

static void identify_process_inductance(
    foc_identify_t *ctx,
    const foc_identify_sample_t *sample)
{
    float current;
    float voltage;
    float current_average;
    float delta_current;
    float inductance;

    if (!identify_elapsed_reached(ctx, sample, ctx->config.settle_ticks)) {
        return;
    }

    if (ctx->status == FOC_IDENTIFY_STATUS_LD) {
        current = sample->id_pu;
        voltage = sample->applied_vd_pu;
    } else {
        current = sample->iq_pu;
        voltage = sample->applied_vq_pu;
    }

    if (!ctx->inductance_has_previous) {
        ctx->inductance_initial_current = current;
        ctx->inductance_previous_current = current;
        ctx->inductance_has_previous = true;
        ctx->stage_sample_count = 1u;
        return;
    }

    current_average =
        (ctx->inductance_previous_current + current) * 0.5f;
    ctx->inductance_sum +=
        (voltage - ctx->result.resistance_pu * current_average) *
        ctx->config.sample_time_s;
    ctx->inductance_previous_current = current;
    ++ctx->stage_sample_count;

    if (ctx->stage_sample_count < ctx->config.pulse_samples) {
        return;
    }

    delta_current = current - ctx->inductance_initial_current;
    if (!identify_is_finite(delta_current) ||
        identify_abs(delta_current) < ctx->config.min_delta_i_pu) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_SMALL_DELTA_I);
        return;
    }

    inductance = ctx->inductance_sum / delta_current;
    if (!identify_is_finite(inductance) || inductance <= 0.0f) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_NUMERIC);
        return;
    }

    if (ctx->status == FOC_IDENTIFY_STATUS_LD) {
        ctx->result.inductance_d_pu = inductance;
        identify_enter_zero(ctx, FOC_IDENTIFY_STATUS_LQ, sample->tick);
    } else {
        ctx->result.inductance_q_pu = inductance;
        identify_enter_zero(ctx, FOC_IDENTIFY_STATUS_DONE, sample->tick);
    }
}

static void identify_process_sample(
    foc_identify_t *ctx,
    const foc_identify_sample_t *sample)
{
    if (sample->command_sequence != ctx->stage_command_sequence) {
        return;
    }

    if (!identify_current_valid(ctx, sample)) {
        if (sample->fault) {
            identify_fail(ctx, FOC_IDENTIFY_ERROR_FAULT);
        } else {
            identify_fail(ctx, FOC_IDENTIFY_ERROR_OVERCURRENT);
        }
        return;
    }

    if (!ctx->stage_clock_started) {
        ctx->stage_start_tick = sample->tick;
        ctx->stage_clock_started = true;
    }

    switch (ctx->status) {
    case FOC_IDENTIFY_STATUS_ZERO:
        identify_process_zero(ctx, sample);
        break;
    case FOC_IDENTIFY_STATUS_RS_LOW:
    case FOC_IDENTIFY_STATUS_RS_HIGH:
        identify_process_resistance(ctx, sample);
        break;
    case FOC_IDENTIFY_STATUS_LD:
    case FOC_IDENTIFY_STATUS_LQ:
        identify_process_inductance(ctx, sample);
        break;
    default:
        break;
    }
}

bool foc_identify_Init(
    foc_identify_t *ctx,
    const foc_identify_config_t *config)
{
    if (ctx == NULL || !identify_config_valid(config)) {
        return false;
    }

    {
        unsigned char *bytes = (unsigned char *)ctx;
        size_t index;
        for (index = 0u; index < sizeof(*ctx); ++index) {
            bytes[index] = 0u;
        }
    }
    ctx->config = *config;
    ctx->status = FOC_IDENTIFY_STATUS_IDLE;
    ctx->error = FOC_IDENTIFY_ERROR_NONE;
    ctx->result.error = FOC_IDENTIFY_ERROR_NONE;
    identify_set_zero_command(ctx);
    return true;
}

bool foc_identify_RequestStart(foc_identify_t *ctx)
{
    if (ctx == NULL || identify_is_active(ctx) || ctx->start_requested) {
        return false;
    }

    ctx->start_requested = true;
    return true;
}

void foc_identify_RequestAbort(foc_identify_t *ctx)
{
    if (ctx != NULL) {
        ctx->abort_requested = true;
    }
}

void foc_identify_StepISR(
    foc_identify_t *ctx,
    const foc_identify_sample_t *sample)
{
    uint16_t write_index;
    uint16_t next_write;

    if (ctx == NULL || sample == NULL || !identify_is_active(ctx)) {
        return;
    }

    if (sample->fault) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_FAULT);
        return;
    }

    if (!identify_is_finite(sample->id_pu) || !identify_is_finite(sample->iq_pu)) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_NUMERIC);
        return;
    }

    if ((sample->id_pu * sample->id_pu) +
            (sample->iq_pu * sample->iq_pu) >
        (ctx->config.current_limit_pu * ctx->config.current_limit_pu)) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_OVERCURRENT);
        return;
    }

    write_index = ctx->queue_write;
    next_write = (uint16_t)((write_index + 1u) %
                            FOC_IDENTIFY_SAMPLE_QUEUE_SIZE);
    if (next_write == ctx->queue_read) {
        identify_fail(ctx, FOC_IDENTIFY_ERROR_QUEUE_OVERRUN);
        return;
    }

    ctx->sample_queue[write_index] = *sample;
    ctx->queue_write = next_write;
}

void foc_identify_Task(foc_identify_t *ctx)
{
    foc_identify_sample_t sample;
    uint32_t processed = 0u;

    if (ctx == NULL) {
        return;
    }

    if (ctx->abort_requested) {
        ctx->abort_requested = false;
        if (identify_is_active(ctx)) {
            identify_fail(ctx, FOC_IDENTIFY_ERROR_ABORTED);
        }
    }

    if (ctx->start_requested) {
        ctx->start_requested = false;
        ctx->abort_requested = false;
        ctx->queue_read = ctx->queue_write;
        ctx->status = FOC_IDENTIFY_STATUS_RS_LOW;
        ctx->error = FOC_IDENTIFY_ERROR_NONE;
        ctx->result.resistance_pu = 0.0f;
        ctx->result.inductance_d_pu = 0.0f;
        ctx->result.inductance_q_pu = 0.0f;
        ctx->result.error = FOC_IDENTIFY_ERROR_NONE;
        ctx->result.valid = false;
        ctx->stage_clock_started = false;
        identify_reset_accumulators(ctx);
        identify_set_command(ctx, ctx->config.v_rs_low_pu, 0.0f, true);
    }

    while (processed < ctx->config.task_sample_budget &&
           identify_is_active(ctx) && identify_pop_sample(ctx, &sample)) {
        identify_process_sample(ctx, &sample);
        ++processed;
    }
}

foc_identify_status_t foc_identify_GetStatus(const foc_identify_t *ctx)
{
    return ctx == NULL ? FOC_IDENTIFY_STATUS_ERROR : ctx->status;
}

const foc_identify_result_t *foc_identify_GetResult(
    const foc_identify_t *ctx)
{
    return ctx == NULL ? NULL : &ctx->result;
}

const foc_identify_command_t *foc_identify_GetCommand(
    const foc_identify_t *ctx)
{
    return ctx == NULL ? NULL : &ctx->command;
}
