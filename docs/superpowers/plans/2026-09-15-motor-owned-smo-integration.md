# Motor-Owned SMO Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Motor the sole owner and caller of the optional SMO Observer while retaining HFI as an unused compile-time reservation.

**Architecture:** `motor_t` owns an embedded `foc_observer_t`; `motor_Init()` initializes it from `motor_cfg_t`, and the Motor high-frequency path calls a simplified Observer API. The current Core feedback remains encoder-based; SMO runs only for lifecycle and numerical validation, while future HFI has only a configuration macro.

**Tech Stack:** C11, GCC host tests, FLOAT/FIXED FOC numeric backends, PowerShell test runners.

---

### Task 1: Update test contracts first

**Files:**
- Modify: `foc/tests/motor_alpha_beta_test.c`
- Modify: `foc/tests/foc_observer_contract_test.c`
- Modify: `foc/tests/foc_app_encoder_command_test.c`
- Modify: `foc/tests/run_motor_alpha_beta_test.ps1`
- Modify: `foc/tests/run_observer_contract_test.ps1`

- [x] Replace external Observer construction and `ptObserver` assignment with `motor_cfg_t.tObserverCfg`, and inspect `tMotor.tObserver` after Motor initialization.
- [x] Replace direct `fnSelectedStep` calls with `foc_observer_Step(foc_observer_t *, const foc_observer_input_t *)`.
- [x] Remove the App test assertion for the deleted `ptObserver` field.
- [x] Enable `FOC_ENABLE_SMO=1` in SMO/Motor host test compile commands.
- [x] Run the affected test runner; execution is blocked because this environment has no `gcc`.

### Task 2: Complete ownership and interface migration

**Files:**
- Modify: `foc/foc_config.h`
- Modify: `foc/app/foc_app.h`
- Modify: `foc/app/foc_app.c`
- Modify: `foc/motor/motor.h`
- Modify: `foc/motor/motor.c`
- Modify: `foc/observer/foc_observer.h`
- Modify: `foc/observer/foc_observer.c`

- [x] Add `FOC_ENABLE_HFI` defaulting to zero without adding HFI state or algorithm code.
- [x] Move Observer configuration into `motor_cfg_t` and remove duplicate App configuration.
- [x] Initialize the embedded Observer inside `motor_Init()` and reset it through Motor-owned paths.
- [x] Remove all old ownership fields, App-level `foc_observer_Init()` calls, and the SMO-specific callback.
- [x] Replace the SMO-specific callback with `foc_observer_Step(foc_observer_t *, const foc_observer_input_t *)`.
- [x] Keep encoder feedback as the Core feedback source; run SMO only as a Motor-owned validation observer.
- [x] Update ownership comments from App-owned to Motor-owned.

### Task 3: Verify both numeric backends and stale-reference cleanup

**Files:**
- Verify: all `foc` sources and tests.

- [x] Confirm no old `ptMotorConfig->ptObserver`, `ptMotor->tCfg.ptObserver`, `ptApp->tObserver`, or `fnSelectedStep` references remain.
- [ ] Run the Observer contract, SMO, Motor alpha-beta, encoder command, and relevant Motor regression tests; blocked by the missing host compiler.
- [ ] Run FLOAT and FIXED variants where the test scripts support both; blocked by the missing host compiler.
- [x] Record that HFI has only a macro reservation and that SMO is instantiated through Motor, without claiming hardware validation.
