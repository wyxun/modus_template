# FOC Inductance MShell Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect the existing Phase 1 `Ld` identification lifecycle to `foc_app` through the `identify inductance` MShell command and print its completed result.

**Architecture:** `foc_app` remains the composition boundary. It converts fixed motor-profile macros into a temporary `identify_inductance_cfg_t`, calls the existing public Start API, and consumes the existing result API from the foreground loop. Identify and Motor continue to own validation, ISR injection, safe-stop, and fault latching.

**Tech Stack:** C11, MODUS MShell, perfc-PT, FLOAT/FIXED FOC backends, PowerShell host-test scripts.

---

### Task 1: Add a failing App command test

**Files:**
- Modify: `foc/tests/foc_app_encoder_command_test.c`
- Modify: `foc/tests/run_encoder_command_test.ps1`

- [ ] **Step 1: Enable a deterministic nominal bus in the App host test**

Add these compiler definitions to `$commonArgs`:

```powershell
"-DFOC_DCBUS_SOURCE=1",
"-DFOC_DCBUS_NOMINAL_MILLIVOLT=12000",
```

- [ ] **Step 2: Make the Motor status stub configurable**

Add a test-owned status snapshot and return it from `motor_GetStatus()`:

```c
static motor_status_t s_tMotorStatus = {0};

foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    (void)ptMotor;
    *ptStatus = s_tMotorStatus;
    return FOC_RESULT_OK;
}
```

- [ ] **Step 3: Add the missing-command test**

Initialize Identify, make Motor idle and aligned, invoke the command, and assert that
the inductance operation starts with the expected prepared values:

```c
assert(identify_Init(&tFocApp.tIdentify) == FOC_RESULT_OK);
s_tMotorStatus.eState = MOTOR_STATE_IDLE;
s_tMotorStatus.bElectricalZeroValid = true;
foc_app_CmdIdentify("inductance");
assert(tFocApp.tIdentify.eOperation == IDENTIFY_OPERATION_INDUCTANCE);
assert(tFocApp.tIdentify.tInductance.wHalfPeriodCycles == 10U);
assert(tFocApp.tIdentify.tInductance.hwCaptureTarget == 4U);
assert(tFocApp.tIdentify.tInductance.hwHalfCycleTarget == 4U);
identify_Stop(&tFocApp.tIdentify, &tFocApp.tMotor);
```

Then clear the alignment flag and verify the same command is rejected with the
existing alignment diagnostic:

```c
assert(identify_Init(&tFocApp.tIdentify) == FOC_RESULT_OK);
s_tMotorStatus.bElectricalZeroValid = false;
s_chLog[0] = '\0';
foc_app_CmdIdentify("inductance");
assert(tFocApp.tIdentify.eOperation == IDENTIFY_OPERATION_NONE);
assert(strstr(s_chLog, "completed motor align") != NULL);
```

- [ ] **Step 4: Run the App test and verify RED**

Run:

```powershell
& .\foc\tests\run_encoder_command_test.ps1
```

Expected: FAIL because `foc_app_CmdIdentify()` does not recognize `inductance` and
the operation remains `IDENTIFY_OPERATION_NONE`.

### Task 2: Add the minimal production command path

**Files:**
- Modify: `foc/app/motor_config.h`
- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: Add the fixed Phase 1 profile macros**

Add the following motor-profile values:

```c
#define MOTOR_IDENTIFY_LD_FREQUENCY_HZ        1000U
#define MOTOR_IDENTIFY_LD_CAPTURE_DELAY       1U
#define MOTOR_IDENTIFY_LD_CAPTURE_SAMPLES     4U
#define MOTOR_IDENTIFY_LD_HALF_CYCLES         4U
#define MOTOR_IDENTIFY_LD_MODULATION_PU       0.10f
#define MOTOR_IDENTIFY_LD_MAX_CURRENT_PU      0.10f
#define MOTOR_IDENTIFY_LD_MIN_DELTA_PU        0.01f
#define MOTOR_IDENTIFY_LD_MAX_SPEED_PU        0.01f
#define MOTOR_IDENTIFY_LD_MOTION_CYCLES       1U
```

- [ ] **Step 2: Extend the command handler**

Treat `resistance` and `inductance` as start commands sharing the existing idle,
PWM-off, and alignment checks. For the inductance branch construct:

```c
identify_inductance_cfg_t tConfig = {
    .wInjectionFrequencyHz = MOTOR_IDENTIFY_LD_FREQUENCY_HZ,
    .hwCaptureDelayCycles = MOTOR_IDENTIFY_LD_CAPTURE_DELAY,
    .hwCaptureSampleCount = MOTOR_IDENTIFY_LD_CAPTURE_SAMPLES,
    .hwHalfCycleCount = MOTOR_IDENTIFY_LD_HALF_CYCLES,
    .qModulationAmplitude =
        FOC_SCALAR(MOTOR_IDENTIFY_LD_MODULATION_PU),
    .qMaxIdentificationCurrent =
        FOC_SCALAR(MOTOR_IDENTIFY_LD_MAX_CURRENT_PU),
    .qMinCurrentDelta =
        FOC_SCALAR(MOTOR_IDENTIFY_LD_MIN_DELTA_PU),
    .qMaxElectricalSpeedPu =
        FOC_SCALAR(MOTOR_IDENTIFY_LD_MAX_SPEED_PU),
    .hwMotionFaultCycles = MOTOR_IDENTIFY_LD_MOTION_CYCLES,
};

eResult = identify_StartInductance(&tFocApp.tIdentify, &tConfig);
```

Update the usage text and `MODUS_SHELL_CMD` description to include `inductance`.

- [ ] **Step 3: Print the completed result once**

In `foc_app_Run()`, after the resistance result check, consume
`identify_GetInductance()` and print:

```text
identify Ld=<uH> uH freq=<Hz> Hz v=<mV> mV i=<mA> mA samples=<n> halves=<n>
```

Extend the App host test by publishing a pending result, advancing one foreground
iteration, and asserting that the log contains `identify Ld=860 uH` and the
configured frequency, voltage, current, sample, and half-cycle fields.

- [ ] **Step 4: Run the App test and verify GREEN**

Run:

```powershell
& .\foc\tests\run_encoder_command_test.ps1
```

Expected: FLOAT and FIXED App tests pass.

### Task 3: Verify identification regressions and source constraints

**Files:**
- Verify: `foc/app/motor_config.h`
- Verify: `foc/app/foc_app.c`
- Verify: `foc/tests/foc_app_encoder_command_test.c`

- [ ] **Step 1: Run both identification suites**

```powershell
& .\foc\tests\run_identify_inductance_test.ps1
& .\foc\tests\run_identify_resistance_test.ps1
```

Expected: FLOAT and FIXED tests pass for both suites.

- [ ] **Step 2: Check formatting and line length**

```powershell
git diff --check
$files = @(
    'foc/app/motor_config.h',
    'foc/app/foc_app.c',
    'foc/tests/foc_app_encoder_command_test.c'
)
foreach ($file in $files) {
    $line = 0
    Get-Content $file | ForEach-Object {
        $line++
        if ($_.Length -gt 82) { "$file`:$line $($_.Length)" }
    }
}
```

Expected: no whitespace errors and no source line longer than 82 characters.
