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
    mfoc_uvw_t duty;
    mfoc_result_t ready;
    mfoc_result_t sample;
    mfoc_result_t write;
    mfoc_result_t enable;
    mfoc_result_t disable;
    bool enabled;
    unsigned writes;
    unsigned enables;
    unsigned disables;
    uint32_t irq_mask;
    mfoc_motor_t *trip_on_sample;
    mfoc_motor_t *trip_before_commit;
} motor_port_fake_t;
extern motor_port_fake_t motor_port_fake;
#endif
