/** @file waveform_capture_test.c
 * @brief Exercise the real waveform scheduler and deferred preparation.
 * @author Codex
 * @date 2026-10-11 */
#include <assert.h>
#include "../../modus/src/mdebug/mwaveform.c"

static unsigned captures = 0U;
static float value = 0.0f;

/* Foreground transport is deliberately forbidden in this ISR test. */
const mwaveform_protocol_t default_waveform_protocol = {0};
int64_t get_system_ms(void) { assert(0); return 0; }
int SEGGER_RTT_ConfigUpBuffer(unsigned index, const char *name, void *buffer,
                             unsigned size, unsigned flags)
{
    (void)index; (void)name; (void)buffer; (void)size; (void)flags;
    assert(0);
    return 0;
}
unsigned SEGGER_RTT_Write(unsigned index, const void *buffer, unsigned size)
{
    (void)index; (void)buffer; (void)size;
    assert(0);
    return 0U;
}

void test_PrepareSample(void)
{
    captures++;
    value = (float)captures;
}

int main(void)
{
    s_tWave.chCount = 1U;
    s_tWave.chDecimation = 2U;
    s_tWave.apvVariables[0] = &value;
    s_tWave.achVariableTypes[0] = MWAVEFORM_VAR_FLOAT;
    s_tWave.atChannels[0].fScale = 100.0f;
    mwaveform_Step();
    assert(captures == 0U);
    s_tWave.bIsRunning = true;
    mwaveform_Step();
    assert(captures == 0U);
    mwaveform_Step();
    assert(captures == 1U && s_tWave.wBatchWCount == 1U);
    assert(s_tWave.atBatchSamples[0].ahwSamples[0] == 100);
    assert(s_tWave.atBatchSamples[0].wSampleIndex == 0U);
    for (unsigned i = 1U; i < MWAVEFORM_BATCH_DEPTH; i++) {
        mwaveform_Step();
        mwaveform_Step();
    }
    assert(captures == MWAVEFORM_BATCH_DEPTH);
    mwaveform_Step();
    mwaveform_Step();
    assert(captures == MWAVEFORM_BATCH_DEPTH && s_tWave.wDropCount == 1U);
    s_tWave.wBatchRCount++;
    mwaveform_Step();
    mwaveform_Step();
    assert(captures == MWAVEFORM_BATCH_DEPTH + 1U);
    assert(s_tWave.atBatchSamples[0].wSampleIndex ==
           MWAVEFORM_BATCH_DEPTH + 1U);
    assert(s_tWave.atBatchSamples[0].ahwSamples[0] == (int16_t)(captures * 100U));
    s_tWave.bIsRunning = false;
    mwaveform_Step();
    mwaveform_Step();
    assert(captures == MWAVEFORM_BATCH_DEPTH + 1U);
    return 0;
}
