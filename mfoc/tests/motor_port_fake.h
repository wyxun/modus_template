/**
 * @file motor_port_fake.h
 * @brief Test-only state of the statically bound motor port.
 * @author Codex
 * @date 2026-10-20
 */
#ifndef MOTOR_PORT_FAKE_H
#define MOTOR_PORT_FAKE_H
#include "mfoc_motor.h"
#include "mfoc_feedback.h"
typedef struct {
    mfoc_uvw_t current;
    mfoc_scalar_t bus;
    mfoc_feedback_output_t feedback;
    mfoc_uvw_t duty;
    mfoc_result_t ready;
    mfoc_result_t sample;
    mfoc_result_t bus_sample;
    mfoc_result_t write;
    mfoc_result_t enable;
    mfoc_result_t disable;
    bool enabled;
    unsigned writes;
    unsigned samples;
    unsigned bus_reads;
    unsigned feedback_reads;
    unsigned feedback_runs;
    mfoc_result_t feedback_run;
    unsigned enables;
    unsigned disables;
    uint32_t irq_mask;
    mfoc_motor_t *trip_before_commit;
    mfoc_result_t port_init;
    bool bus_configured;
    unsigned port_inits;
} motor_port_fake_t;
extern motor_port_fake_t motor_port_fake;
#endif
