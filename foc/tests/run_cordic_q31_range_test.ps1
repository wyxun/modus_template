$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourcePath = Join-Path $repoRoot "peripheral\stm32g431\halcordic.c"
$source = Get-Content -LiteralPath $sourcePath -Raw
$scaleMatch = [regex]::Match(
    $source,
    's_fCordicFloatQ31Scale\s*=\s*([0-9.]+)f')

if (-not $scaleMatch.Success) {
    throw "Safe float-to-Q31 scale is not declared"
}
if ($source -notmatch 'qX\s*/\s*fMax\)\s*\*\s*s_fCordicFloatQ31Scale' -or
    $source -notmatch 'qY\s*/\s*fMax\)\s*\*\s*s_fCordicFloatQ31Scale') {
    throw "Atan2 inputs do not use the bounded Q31 scale"
}

$scale = [single]::Parse(
    $scaleMatch.Groups[1].Value,
    [Globalization.CultureInfo]::InvariantCulture)
if ([double]$scale -ge 2147483648.0) {
    throw "Float-to-Q31 scale reaches the signed 32-bit overflow boundary"
}

foreach ($normalized in @(-1.0, 0.0, 1.0)) {
    $converted = [double]([single]$normalized * $scale)
    if ($converted -lt [int]::MinValue -or
        $converted -gt [int]::MaxValue) {
        throw "Normalized endpoint is outside the int32 range"
    }
}

Write-Output "CORDIC float Q31 range test passed"
