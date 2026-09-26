$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$source = Get-Content -LiteralPath (
    Join-Path $repoRoot "foc\app\foc_app.c") -Raw

foreach ($token in @(
        'SMO rms x1e4 e=', 'iErr=', 'err25=',
        'SMO cond x1e4 eBad=', 'eGood=', 'iBad=', 'iGood=',
        'SMO bin bad/tot [0]=',
        'SMO sec err/low s0=')) {
    if ($source -notmatch [regex]::Escape($token)) {
        throw "SMO diagnostic log is missing: $token"
    }
}
foreach ($stale in @(
        'SMO init Rs=', 'SMO coeff x1e6', 'SMO diag state=',
        'SMO input x1e4', 'SMO axis x1e4',
        'SMO lag x1e4')) {
    if ($source -match [regex]::Escape($stale)) {
        throw "Stale SMO diagnostic remains: $stale"
    }
}
if ($source -notmatch 'perfc_is_time_out_ms\(1000U') {
    throw "SMO diagnostic output must remain foreground and rate-limited"
}

Write-Output "SMO diagnostic log test passed"
