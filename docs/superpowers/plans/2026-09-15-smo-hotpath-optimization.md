# SMO 高频路径优化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce repeated SMO high-frequency arithmetic and numeric helper call overhead while preserving the existing FLOAT/FIXED formulas and behavior.

**Architecture:** Compute the cross-axis speed coefficient once in `foc_smo_Step()` and pass that derived value into the two axis updates. Move the four small numeric primitives used by the SMO hot path into the numeric header as `static inline` implementations, removing their out-of-line duplicates while preserving their existing backend semantics.

**Tech Stack:** C11, PowerShell host tests, FLOAT and Q15 FIXED numeric backends, existing FOC math/SMO APIs.

---

### Task 1: Add a failing hot-path contract test

**Files:**
- Create: `foc/tests/run_smo_hotpath_contract_test.ps1`
- Test: `foc/observer/foc_smo.c`, `foc/math/foc_numeric.h`,
  `foc/math/foc_numeric.c`

- [ ] **Step 1: Write the failing test**

Create a PowerShell source contract that extracts `smo_AxisStep()` and
`foc_smo_Step()` and checks the intended structure:

```powershell
$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$smoPath = Join-Path $repoRoot "foc\observer\foc_smo.c"
$numericHeaderPath = Join-Path $repoRoot "foc\math\foc_numeric.h"
$numericSourcePath = Join-Path $repoRoot "foc\math\foc_numeric.c"
$smo = Get-Content -LiteralPath $smoPath -Raw
$numericHeader = Get-Content -LiteralPath $numericHeaderPath -Raw
$numericSource = Get-Content -LiteralPath $numericSourcePath -Raw

$axisMatch = [regex]::Match(
    $smo,
    '(?s)static void smo_AxisStep\(.*?\n\}\n\n/\*\*')
$stepMatch = [regex]::Match(
    $smo,
    '(?s)foc_result_t foc_smo_Step\(.*?\n\}\s*$')
if (-not $axisMatch.Success -or -not $stepMatch.Success) {
    throw "SMO hot-path functions cannot be located"
}
$axisBody = $axisMatch.Value
$stepBody = $stepMatch.Value

if ($axisBody -match 'qCrossAxisGain') {
    throw "smo_AxisStep still recomputes the cross-axis speed coefficient"
}
if ($axisBody -notmatch 'qCrossAxisSpeedGain') {
    throw "smo_AxisStep does not consume the precomputed cross-axis coefficient"
}
if ($stepBody -notmatch 'qCrossAxisSpeedGain') {
    throw "foc_smo_Step does not compute the cross-axis coefficient"
}
if ([regex]::Matches($stepBody, 'qCrossAxisGain').Count -ne 1) {
    throw "foc_smo_Step must use qCrossAxisGain exactly once"
}

foreach ($name in @('foc_mul_wide', 'foc_add_sat', 'foc_sub_sat', 'foc_abs')) {
    if ($numericHeader -notmatch "static inline foc_scalar_t $name") {
        throw "$name is not provided as a static inline numeric primitive"
    }
    if ($numericSource -match "foc_scalar_t $name\(") {
        throw "$name still has an out-of-line implementation"
    }
}
Write-Output "SMO hot-path contract passed"
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```powershell
& .\foc\tests\run_smo_hotpath_contract_test.ps1
```

Expected: FAIL because the current axis function still computes
`qCrossAxisGain * qElectricalModelSpeed`, and the numeric primitives are
out-of-line.

### Task 2: Remove the repeated cross-axis coefficient calculation

**Files:**
- Modify: `foc/observer/foc_smo.c:476-553`
- Modify: `foc/observer/foc_smo.c:627-659`

- [ ] **Step 1: Change the axis helper input**

Rename the axis helper input and use it directly:

```c
 * @param qCrossAxisSpeedGain Precomputed cross-axis speed coefficient.
```

```c
static void smo_AxisStep(foc_smo_t *ptSmo,
                         foc_smo_axis_t *ptAxis,
                         foc_scalar_t qMeasured,
                         foc_scalar_t qVoltage,
                         foc_scalar_t qCrossCurrent,
                         foc_scalar_t qCrossAxisSpeedGain)
{
    foc_scalar_t qDerivative = foc_mul_wide(
        ptSmo->qVoltageCurrentGain, qVoltage);
    foc_scalar_t qCross = foc_mul_wide(
        qCrossAxisSpeedGain, qCrossCurrent);
```

Keep all integrator, sliding, filter, saturation and state-update code
unchanged.

- [ ] **Step 2: Compute the shared coefficient once in `foc_smo_Step()`**

Add an initialized local and compute it once immediately after the electrical
model speed:

```c
    foc_scalar_t qCrossAxisSpeedGain = FOC_ZERO;
```

```c
    qElectricalModelSpeed = foc_mul_wide(
        ptSmo->qPllMechanicalSpeed, ptSmo->qPolePairs);
    qCrossAxisSpeedGain = foc_mul_wide(
        ptSmo->qCrossAxisGain, qElectricalModelSpeed);
```

Pass `qCrossAxisSpeedGain` to both `smo_AxisStep()` calls. Do not change the
saved `qPreviousAlpha` and `qPreviousBeta` ordering.

- [ ] **Step 3: Run the focused contract**

Run:

```powershell
& .\foc\tests\run_smo_hotpath_contract_test.ps1
```

Expected: the cross-axis assertions pass; the inline numeric assertions still
fail until Task 3 is complete.

### Task 3: Inline the numeric primitives without changing semantics

**Files:**
- Modify: `foc/math/foc_numeric.h:35-168`
- Modify: `foc/math/foc_numeric.c:8-105`
- Modify: `foc/math/foc_numeric.c:151-159`

- [ ] **Step 1: Add fixed-width limit definitions to the header**

Add `<limits.h>` beside the existing standard headers so the fixed inline
implementations have the same limit constants as the source implementation:

```c
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
```

- [ ] **Step 2: Replace the four declarations with `static inline` bodies**

Keep the existing Doxygen comments and use these bodies:

```c
static inline foc_scalar_t foc_add_sat(foc_scalar_t qA,
                                       foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA + qB;
#else
    if (qB > 0 && qA > INT32_MAX - qB) {
        return INT32_MAX;
    }
    if (qB < 0 && qA < INT32_MIN - qB) {
        return INT32_MIN;
    }
    return qA + qB;
#endif
}
```

```c
static inline foc_scalar_t foc_sub_sat(foc_scalar_t qA,
                                       foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA - qB;
#else
    if (qB < 0 && qA > INT32_MAX + qB) {
        return INT32_MAX;
    }
    if (qB > 0 && qA < INT32_MIN + qB) {
        return INT32_MIN;
    }
    return qA - qB;
#endif
}
```

```c
static inline foc_scalar_t foc_mul_wide(foc_scalar_t qA,
                                        foc_scalar_t qB)
{
#if defined(FOC_NUMERIC_FLOAT)
    return qA * qB;
#else
    int64_t llProduct = (int64_t)qA * (int64_t)qB;

    llProduct /= FOC_Q_SCALE;
    if (llProduct > INT32_MAX) {
        return INT32_MAX;
    }
    if (llProduct < INT32_MIN) {
        return INT32_MIN;
    }
    return (foc_scalar_t)llProduct;
#endif
}
```

```c
static inline foc_scalar_t foc_abs(foc_scalar_t qValue)
{
#if defined(FOC_NUMERIC_FIXED)
    if (qValue == INT32_MIN) {
        return INT32_MAX;
    }
#endif
    return qValue < FOC_ZERO ? -qValue : qValue;
}
```

- [ ] **Step 3: Remove the four out-of-line definitions**

Delete only the existing `foc_add_sat()`, `foc_sub_sat()`, `foc_mul_wide()` and
`foc_abs()` function bodies from `foc_numeric.c`. Leave `foc_from_float()`,
`foc_to_float()`, `foc_mul_pu()`, `foc_div_checked()`, `foc_sat()` and the gain
functions unchanged.

- [ ] **Step 4: Run the focused contract**

Run:

```powershell
& .\foc\tests\run_smo_hotpath_contract_test.ps1
```

Expected: `SMO hot-path contract passed`.

- [ ] **Step 5: Run the SMO behavior tests**

Run:

```powershell
& .\foc\tests\run_smo_test.ps1
```

Expected: FLOAT and FIXED SMO tests pass. If the host compiler is unavailable,
record that limitation and run all available static contracts instead.

### Task 4: Final review and verification

**Files:**
- Review: `foc/observer/foc_smo.c`
- Review: `foc/math/foc_numeric.h`
- Review: `foc/math/foc_numeric.c`
- Review: `foc/tests/run_smo_hotpath_contract_test.ps1`

- [ ] **Step 1: Check the diff and style constraints**

Run:

```powershell
git diff --check
```

Then inspect that the modified C functions retain Doxygen comments, initialized
locals, no function body exceeds the project limit, and no SMO high-frequency
path adds dynamic allocation, logging or runtime estimator dispatch.

- [ ] **Step 2: Check the final source properties**

Run:

```powershell
rg -n 'qCrossAxisGain|qCrossAxisSpeedGain|static inline foc_scalar_t foc_(mul_wide|add_sat|sub_sat|abs)' foc\observer\foc_smo.c foc\math\foc_numeric.h
```

Expected: the raw cross-axis gain is used once in `foc_smo_Step()`, the axis
helper consumes the precomputed value, and all four numeric primitives are
defined inline in the header.

- [ ] **Step 3: Report evidence and remaining hardware validation**

Report host test results separately from target-MCU cycle measurements. Do not
claim the 20 kHz budget is met until the target build has been measured with
the final optimization flags and trig backend.
