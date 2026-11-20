#ifndef TEST_AT32F413_PERF_COUNTER_H
#define TEST_AT32F413_PERF_COUNTER_H
#include <stdbool.h>
#include <stdint.h>

int64_t get_system_ticks(void);
uint32_t perfc_get_systimer_frequency(void);
bool perfc_is_time_out_ms(uint32_t ms, int64_t *timestamp, bool repeat);

#endif
