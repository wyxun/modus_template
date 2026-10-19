# G431 20 kHz ISR timing baseline (2026-10-17)

Firmware: `build/template.hex` SHA-256
`93ADC5B236C2CFFBBBE71B8D4DE328DC1206400D5DEDCAF8A58E13FBFA68D915`.
The on-board IRQ instruction and active probe sample counter matched this
image. This build disables the prior `foc ISR avg` accumulator; four 10 kHz
waveform channels remain configured. The probe's initialization-only overhead
calibration read 274 cycles (1.61 us), which is **not** a per-cycle correction.

| Condition | Full IRQ max | Entry interval min..max | >55 us / >75 us intervals | >50 us execution |
| --- | ---: | ---: | ---: | ---: |
| IDLE, PWM off | 2651 cycles (15.59 us) | 8195..8809 cycles | 0 / 0 | 0 |
| Rotor align, then stop | 4683 cycles (27.55 us) | 8193..8809 cycles | 0 / 0 | 0 |
| `motor speed 0.4`, about 5.1 s, then stop | 8384 cycles (49.32 us) | 5728..17040 cycles | 528 / 527 | 0 |

During the 0.4 run, the independent stage maxima were 5519 cycles (32.46 us)
from IRQ entry to the duty record, 1406 cycles (8.27 us) from duty to observer
record, and 1502 cycles (8.84 us) from observer to IRQ tail. They occurred on
potentially different cycles and must not be added as one measured cycle.
The duty record is after CCR1-3 writes. The observer-to-tail stage contains
waveform capture and other application/IRQ work. There were no wave CSV captures
in this timing trial, so waveform transport completeness was not established.

The minimum observed entry phase was 688 timer ticks after the CH4 trigger.
Even this minimum plus the 8384-cycle execution peak exceeds the 8500-cycle
trigger period. The execution-only `wOverrunCount == 0` therefore does not
prove meeting the next-trigger deadline. The near-two-period entry interval
is compatible with a delayed or lost control event; this measurement alone
does not identify the cause. The IRQ still clears JEOS after the FOC function,
which could acknowledge a newly completed conversion if the cycle crosses it.

After the motor stopped, the late interval counters remained 528/527 during a
further four-second idle observation. Final motor state was IDLE, fault 0,
PWM off. Further SMO takeover testing should wait for a measured deadline
margin and a separate review of JEOS acknowledgement ownership.

Target disassembly showed the 64-bit divide helper calls inside
`motor_IsrPrepare` correspond to ADC offset calibration, not the RUN bus
voltage hot path. The only direct `__fpclassifyf` call in the linked image was
inside libc's `__math_invalidf`. Thus the prior proposed RUN optimizations of
64-bit bus division and scalar finite checks need fresh evidence before edits.
The FLOAT SMO `atan2f` call is present in the observer stage, but that stage's
maximum is only an upper bound on its cost.

## Follow-up change awaiting board verification

The next build acknowledges ADC1/ADC2 JEOS immediately after the MDI current
frame read. The IRQ tail clears JEOS only when that cycle did not consume a
frame; it still clears OVR. This removes the second, unconditional JEOS write
on normal control cycles. The FLOAT/FIXED host ADC tests, module boundary
test, application combination matrix, and G431 target build passed. These
tests check acknowledgement commands, but cannot reproduce peripheral
write-one-to-clear timing or establish that late entries will disappear.
## Board trial with early JEOS acknowledgement

The engineer flashed the new image (`build/template.hex` SHA-256
`E1E16A2BA3026A7DF3B30BF7E6DE9434C131D685ABAA1C3AD8B352BC1F2C5857`).
The on-board ISR instruction at `0x080004AA` matched the ELF. The motor was
initially idle, PWM off, fault 0. At idle the outer IRQ maximum was 2673
cycles (15.72 us), entry intervals were 8192..8806 cycles, and all three
late/overrun counters were zero.

One `motor speed 0.4` trial captured four 10 kHz channels for five wall-clock
seconds. AITrace returned 41984 contiguous samples (sample indices
669958..711941, timestamps 0..4.1983 s) and reported zero sequence drops.
This verifies the recorded slice, not the uncaptured edges of the five-second
window. The motor entered `MOTOR_FAULT_POSITION` (`fault=0x4`) with PWM off
at approximately CSV time 3.5127 s. After `motor stop`, it remained in the
latched fault state. No further motor run was attempted.

| 0.4 trial | Old acknowledgement | Early acknowledgement |
| --- | ---: | ---: |
| Outer IRQ maximum | 8384 cycles (49.32 us) | 8553 cycles (50.31 us) |
| Entry interval min..max | 5728..17040 cycles | 8191..8809 cycles |
| >55 us / >75 us entry intervals | 528 / 527 | 0 / 0 |
| Execution >50 us | 0 | 17 |

The new board trial removes the observed long entry gaps but does **not**
meet the 50 us execution budget. The runs also differed in their final state,
so they do not establish the JEOS change as the sole cause of the entry-gap
improvement. The initialization-only full probe self-calibration remained
274 cycles (1.61 us); it is not a per-cycle subtraction from the measured
maximum.

The fault occurred in feedback phase 3 (FAILED) after 4000 qualifying cycles
and 83 blend cycles. Since 83 is below the configured 400 blend steps,
`motor_position_SelectSmo()` reached its loss-of-qualification branch during
the blend, rather than the blend-timeout branch. In the CSV, SMO angle was
valid through 3.5126 s, then
became its invalid sentinel. Its final recorded one-millisecond angle change
corresponds to approximately 0.3375 speed pu. The current raw-speed
qualification range at forced speed 0.4 pu is 0.34..0.46 pu, so this dip is
a plausible loss condition. The waveform quantizes angle to 0.001 turn and
is sampled at half the ISR rate; it cannot prove which exact 20 kHz condition
failed. Other qualification inputs, including BEMF and control limiting,
were not captured. The recorded current estimation residuals alone do not
identify the failure. Keep the position fault latched and do not repeat
takeover until ISR margin and the qualification failure are independently
verified.
