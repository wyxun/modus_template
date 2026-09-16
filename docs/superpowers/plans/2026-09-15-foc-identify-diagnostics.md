# FOC Identify Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add low-overhead RS/Ld/Lq diagnostic snapshots that can be printed from
Shell without adding logging, fitting, or loops to the high-frequency ISR.

**Architecture:** Keep all mutable diagnostic state in `foc_identify_t`. The
identification step records already-computed scalar values into per-stage
snapshots; a const getter copies those snapshots for the foreground Shell
printer. Measurement constants and formulas remain unchanged in this change.

**Tech Stack:** C11, FLOAT/FIXED FOC numeric backends, PowerShell host tests,
GNU GCC, STM32G431 debug-rel build.

---

### Task 1: Add the diagnostic data contract and failing tests

**Files:**
- Modify: `foc/identify/foc_identify.h`
- Modify: `foc/tests/foc_identify_test.c`

- [ ] **Step 1: Add a test for successful stage snapshots.**

  Extend `test_full_identification_flow()` to retrieve diagnostics after the
  final Lq step and assert that RS, Ld, and Lq snapshots contain positive
  `qDeltaI`/`qSumV` values and the expected result fields.

- [ ] **Step 2: Add a test for a safety snapshot.**

  In `test_safety_and_lifecycle()`, retrieve diagnostics after the existing
  over-current failure and assert that the failure stage is recorded as the
  active RS stage and the snapshot current contains the triggering value.

- [ ] **Step 3: Run the identify test to verify the new tests fail.**

  Run:

  ```powershell
  .\foc\tests\run_identify_test.ps1
  ```

  Expected: compilation fails because the diagnostic type, getter, and fields
  do not yet exist.

### Task 2: Implement per-stage diagnostic snapshots

**Files:**
- Modify: `foc/identify/foc_identify.h`
- Modify: `foc/identify/foc_identify.c`

- [ ] **Step 1: Define the public snapshot type.**

  Add `foc_identify_diag_t` with `qIStart`, `qILast`, `qDeltaI`, `qSumV`,
  `qResistancePu`, `qResult`, `hwTicks`, and `bValid`. Add RS low/high voltage
  and current fields only if required by the test evidence; keep the first
  version limited to values requested by the diagnostic goal.

- [ ] **Step 2: Add three snapshots to `foc_identify_t`.**

  Store `tRsDiag`, `tLdDiag`, and `tLqDiag`, plus `eFailureStage`. Initialize
  them through the existing whole-object reset in `foc_identify_Init()` and
  clear them in `foc_identify_Start()`.

- [ ] **Step 3: Record values without adding ISR work beyond scalar stores.**

  In the RS stage, save the averaged low/high current, `qDeltaI`, and the
  resistance result. In the inductance stage, save `qI_start`, the final
  current, `qDeltaI`, accumulated `qSumV`, resistance used, and calculated L
  for the active axis. Record the failure stage before `identify_fail()` clears
  the result.

- [ ] **Step 4: Add `foc_identify_GetDiagnostics()`.**

  Validate both pointers, copy the three snapshots and failure stage, and
  return the existing FOC result codes. The getter must not alter controller
  state.

- [ ] **Step 5: Run the identify tests.**

  Run the same PowerShell test script and expect FLOAT and FIXED tests to pass.

### Task 3: Print diagnostics from the foreground Shell path

**Files:**
- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: Add a small foreground diagnostic printer.**

  Convert the copied snapshots with the existing `foc_to_float()` helper and
  print one compact line per stage. Do not call this function from
  `foc_app_HighFrequencyISR()`.

- [ ] **Step 2: Print before consuming COMPLETE/ERROR.**

  Call the getter and printer from `foc_app_PrintIdentify()` before
  `foc_identify_ConsumeTerminal()`, preserving the current terminal lifecycle.

- [ ] **Step 3: Build the target firmware.**

  Run:

  ```powershell
  .\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=float FOC_ENABLE_SMO=0 FOC_ENABLE_HFI=0 FOC_EXPERIMENTAL_IDENTIFY=1
  ```

  Expected: successful build with no new compiler warnings.

### Task 4: Verification and review

**Files:**
- Review: `foc/identify/foc_identify.h`
- Review: `foc/identify/foc_identify.c`
- Review: `foc/app/foc_app.c`

- [ ] **Step 1: Run all FOC host tests.**

  Run every `foc/tests/run_*.ps1` script and require exit code 0.

- [ ] **Step 2: Inspect the final diff.**

  Confirm no `MLOGF`, fitting loop, floating-point conversion, or new blocking
  work was added to the ISR path. Confirm unrelated worktree changes remain
  untouched.

- [ ] **Step 3: Report the exact diagnostic fields and verification results.**

  Do not claim hardware behavior is fixed; report only that the diagnostic
  instrumentation is built and tested until a new bench run is performed.
