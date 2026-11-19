/**
 * @file identification_inertia_test.c
 * @brief Independent matched-speed torque and drag fitting replay.
 * @author Codex
 * @date 2026-11-19
 */
#include "mfoc_identification_rotation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    mfoc_identification_rotation_t run = {.current_max = MFOC_SCALAR(0.12f),
        .speed_max = MFOC_IDENTIFICATION_SPEED_CAP_PU};
    const float dt = mfoc_to_float(MFOC_ROTATION_BLOCK_DT);
    const float inertia = 23.0f, drag = 0.0008f;
    float speed = 0.58f * mfoc_to_float(run.speed_max);
    float previous_torque = 0.0f;
    mfoc_scalar_t result = MFOC_ONE;
    assert(mfoc_identification_inertia_Finish(&run.inertia, &result) ==
        MFOC_FSM_FAILED && result == MFOC_ONE);
    mfoc_fsm_rt_t status = MFOC_FSM_ONGOING;
    for (unsigned k = 0; k < 400U && status == MFOC_FSM_ONGOING; ++k) {
        float torque = 0.7f * mfoc_to_float(run.i_ref.qQ);
        speed += (previous_torque - drag) * dt / inertia;
        status = mfoc_identification_inertia_Block(&run, mfoc_from_float(speed),
            mfoc_from_float(torque));
        previous_torque = torque;
    }
    assert(status == MFOC_FSM_COMPLETE);
    assert(mfoc_identification_inertia_Finish(&run.inertia, &result) ==
        MFOC_FSM_COMPLETE);
    assert(fabsf(mfoc_to_float(result) - inertia) < inertia * 0.015f);
    puts("Matched-speed impulse fit passed");
    return 0;
}
