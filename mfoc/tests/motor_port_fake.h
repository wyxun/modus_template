/**
 * @file motor_port_fake.h
 * @brief Test-only state of the statically bound motor port.
 * @author Codex
 * @date 2026-10-20
 */
#ifndef MOTOR_PORT_FAKE_H
#define MOTOR_PORT_FAKE_H
#include "mfoc_motor.h"
typedef struct {
    mfoc_uvw_t current;
    mfoc_scalar_t bus;
    mfoc_feedback_output_t feedback;
    mfoc_uvw_t duty;
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
    mfoc_result_t feedback_step;
    mfoc_result_t calibration;
    unsigned calibrations;
    mfoc_angle_t held_angle;
    unsigned enables;
    unsigned disables;
    mfoc_motor_t *step_during_disable;
    mfoc_result_t port_init;
    bool bus_configured;
    unsigned port_inits;
    int64_t time_ms;
} motor_port_fake_t;
extern motor_port_fake_t motor_port_fake;
#endif
