#ifndef FOC_IDENTIFY_MINIMAL_H
#define FOC_IDENTIFY_MINIMAL_H

#include <stdbool.h>
#include <stdint.h>

#define FOC_IDENTIFY_SAMPLE_QUEUE_SIZE 64u

typedef enum {
    FOC_IDENTIFY_STATUS_IDLE = 0,
    FOC_IDENTIFY_STATUS_RS_LOW,
    FOC_IDENTIFY_STATUS_RS_HIGH,
    FOC_IDENTIFY_STATUS_ZERO,
    FOC_IDENTIFY_STATUS_LD,
    FOC_IDENTIFY_STATUS_LQ,
    FOC_IDENTIFY_STATUS_DONE,
    FOC_IDENTIFY_STATUS_ERROR
} foc_identify_status_t;

typedef enum {
    FOC_IDENTIFY_ERROR_NONE = 0,
    FOC_IDENTIFY_ERROR_CONFIG,
    FOC_IDENTIFY_ERROR_QUEUE_OVERRUN,
    FOC_IDENTIFY_ERROR_ABORTED,
    FOC_IDENTIFY_ERROR_FAULT,
    FOC_IDENTIFY_ERROR_OVERCURRENT,
    FOC_IDENTIFY_ERROR_ZERO_TIMEOUT,
    FOC_IDENTIFY_ERROR_SMALL_DELTA_I,
    FOC_IDENTIFY_ERROR_NUMERIC
} foc_identify_error_t;

typedef struct {
    float v_rs_low_pu;
    float v_rs_high_pu;
    float v_ld_pu;
    float v_lq_pu;
    float current_limit_pu;
    float zero_current_limit_pu;
    float min_delta_i_pu;
    float sample_time_s;
    uint32_t settle_ticks;
    uint32_t zero_min_ticks;
    uint32_t zero_max_ticks;
    uint32_t zero_confirm_samples;
    uint32_t average_samples;
    uint32_t pulse_samples;
    uint32_t task_sample_budget;
} foc_identify_config_t;

typedef struct {
    float id_pu;
    float iq_pu;
    float applied_vd_pu;
    float applied_vq_pu;
    uint32_t tick;
    uint32_t command_sequence;
    bool fault;
} foc_identify_sample_t;

typedef struct {
    float vd_pu;
    float vq_pu;
    bool pwm_enable;
} foc_identify_command_t;

typedef struct {
    float resistance_pu;
    float inductance_d_pu;
    float inductance_q_pu;
    foc_identify_error_t error;
    bool valid;
} foc_identify_result_t;

typedef struct {
    foc_identify_config_t config;
    foc_identify_status_t status;
    foc_identify_error_t error;
    foc_identify_result_t result;
    foc_identify_command_t command;

    volatile bool start_requested;
    volatile bool abort_requested;
    volatile uint16_t queue_write;
    volatile uint16_t queue_read;
    foc_identify_sample_t sample_queue[FOC_IDENTIFY_SAMPLE_QUEUE_SIZE];

    foc_identify_status_t zero_next_status;
    uint32_t stage_start_tick;
    uint32_t command_sequence;
    uint32_t stage_command_sequence;
    bool stage_clock_started;
    uint32_t stage_sample_count;
    uint32_t zero_confirm_count;

    float sum_current;
    float sum_voltage;
    float rs_current_low;
    float rs_current_high;
    float rs_voltage_low;
    float rs_voltage_high;

    float inductance_sum;
    float inductance_initial_current;
    float inductance_previous_current;
    bool inductance_has_previous;
} foc_identify_t;

bool foc_identify_Init(
    foc_identify_t *ctx,
    const foc_identify_config_t *config);

bool foc_identify_RequestStart(foc_identify_t *ctx);
void foc_identify_RequestAbort(foc_identify_t *ctx);

void foc_identify_StepISR(
    foc_identify_t *ctx,
    const foc_identify_sample_t *sample);

void foc_identify_Task(foc_identify_t *ctx);

foc_identify_status_t foc_identify_GetStatus(const foc_identify_t *ctx);
const foc_identify_result_t *foc_identify_GetResult(
    const foc_identify_t *ctx);
const foc_identify_command_t *foc_identify_GetCommand(
    const foc_identify_t *ctx);

#endif
