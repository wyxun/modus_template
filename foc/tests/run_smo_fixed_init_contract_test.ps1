$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourcePath = Join-Path $repoRoot "foc/observer/foc_smo.c"
$source = Get-Content $sourcePath -Raw

if ($source -notmatch "smo_StoreFixedRatio") {
    throw "FIXED SMO initialization helper is missing"
}
$initMatch = [regex]::Match(
    $source,
    "(?s)foc_result_t foc_smo_Init\(.*?\r?\n\}\r?\n\r?\n/\*\*")
if (-not $initMatch.Success) {
    throw "foc_smo_Init body cannot be located"
}
$initBody = $initMatch.Value
if ($initBody -notmatch "(?s)#if defined\(FOC_NUMERIC_FIXED\).*?smo_StoreFixed") {
    throw "foc_smo_Init has no FIXED-specific initialization branch"
}
$fixedMatch = [regex]::Match(
    $initBody,
    "(?s)#if defined\(FOC_NUMERIC_FIXED\)(?<fixed>.*?)#else")
if (-not $fixedMatch.Success -or
    $fixedMatch.Groups["fixed"].Value -match "\bfloat\b") {
    throw "FIXED foc_smo_Init branch contains floating-point code"
}
if ($source -notmatch "(?s)smo_StoreFixedRatio.*?qSpeedConversionGain") {
    throw "FIXED SMO initialization does not cover speed conversion"
}

Write-Output "SMO FIXED initialization contract passed"
