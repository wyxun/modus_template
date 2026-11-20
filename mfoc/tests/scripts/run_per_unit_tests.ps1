$ErrorActionPreference = 'Stop'
$tests = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $tests
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-pu-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
Write-Output "PU test artifacts: $output"

$modules = @('math', 'control', 'core', 'motor', 'port', 'position')
$includes = @("-I$library") + @($modules | ForEach-Object {
    "-I$library/$_/include"
})
$sources = @(Get-ChildItem -File ($modules | ForEach-Object {
    "$library/$_/src/*.c"
}) | Where-Object { $_.Name -ne 'mfoc_feedback.c' } |
    ForEach-Object { $_.FullName })
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "replay-$numeric.exe"
    $ekf = [int]($numeric -eq 'FLOAT')
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        -DMFOC_FEEDBACK_BACKEND=1 -DMFOC_FEEDBACK_ENABLE_SMO=0 `
        -DMFOC_FEEDBACK_ENABLE_FLUX=0 "-DMFOC_FEEDBACK_ENABLE_EKF=$ekf" `
        @includes "$tests/motor/per_unit_invariance_test.c" `
        "$tests/motor/motor_port_fake.c" @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "PU replay build failed: $numeric" }
    Write-Output "$numeric interleaved physical replay"
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "PU replay failed: $numeric" }
}

# Ld/Lq and rotating identification use a shared build-time timing profile.
# Reuse the P09 PMSM fixture under both physical base sets in a private copy.
$isolated = Join-Path $output 'mfoc'
New-Item -ItemType Directory -Path $isolated | Out-Null
Get-ChildItem -LiteralPath $library -File -Filter '*.h' | Copy-Item -Destination $isolated
foreach ($module in $modules + 'identification') {
    $destination = Join-Path $isolated $module
    New-Item -ItemType Directory -Path $destination | Out-Null
    Copy-Item -LiteralPath "$library/$module/include" -Destination $destination -Recurse
    $sourceDir = "$library/$module/src"
    if (Test-Path -LiteralPath $sourceDir) {
        Copy-Item -LiteralPath $sourceDir -Destination $destination -Recurse
    }
}
New-Item -ItemType Directory -Path "$isolated/tests/scripts", "$isolated/tests/motor" | Out-Null
Copy-Item -LiteralPath "$tests/scripts/run_core_tests.ps1" -Destination "$isolated/tests/scripts"
Copy-Item -LiteralPath "$tests/motor/motor_port_fake.c", "$tests/motor/motor_port_fake.h", `
    "$tests/motor/motor_identification_plant_test.c" -Destination "$isolated/tests/motor"
$profileHeader = Join-Path $isolated 'mfoc_motor_config.h'
$template = Get-Content -LiteralPath $profileHeader -Raw -Encoding UTF8
$profiles = @(
    @{ CURRENT_MILLIAMP = 3500; VOLTAGE_MILLIVOLT = 12000; ELECTRICAL_HZ = 400 },
    @{ CURRENT_MILLIAMP = 4000; VOLTAGE_MILLIVOLT = 14000; ELECTRICAL_HZ = 320 }
)
$results = @()
foreach ($profile in $profiles) {
    $config = $template
    foreach ($base in $profile.Keys) {
        $macro = "MFOC_MOTOR_BASE_$base"
        $definition = "(?m)^#define $macro[ \t]+[0-9]+U(?=\r?$)"
        if ([regex]::Matches($config, $definition).Count -ne 1) {
            throw "Expected one literal definition of $macro"
        }
        $config = $config -replace $definition, "#define $macro $($profile[$base])U"
    }
    Set-Content -LiteralPath $profileHeader -Value $config -Encoding UTF8
    $log = & (Join-Path $isolated 'tests/scripts/run_core_tests.ps1') `
        -Tests motor_identification_plant -Backend 0 -Position 0 -Hfi 0 -Identify 1
    $log | Set-Content -LiteralPath (Join-Path $output "plant-$($profile.ELECTRICAL_HZ).log")
    $rows = @($log | Where-Object { $_ -match '^SI ' })
    if ($rows.Count -ne 2) { throw 'Expected FLOAT and FIXED SI results' }
    $results += ,$rows
}

# Tolerances compare equivalent experiments, with an independent truth check
# in the plant fixture for every measured parameter and each numeric backend.
$tolerances = @(0.005, 0.01, 0.01, 0.02, 0.05)
$parameters = @('Rs', 'Ld', 'Lq', 'flux', 'inertia')
$culture = [Globalization.CultureInfo]::InvariantCulture
for ($i = 0; $i -lt 2; ++$i) {
    $numeric = @('FLOAT', 'FIXED')[$i]
    $a = $results[0][$i].Split(' ')
    $b = $results[1][$i].Split(' ')
    for ($j = 0; $j -lt $tolerances.Count; ++$j) {
        $x = [double]::Parse($a[$j + 1], $culture)
        $y = [double]::Parse($b[$j + 1], $culture)
        $delta = [math]::Abs(($y - $x) / $x)
        $name = $parameters[$j]
        Write-Output ("{0} {1}: {2:G8} and {3:G8} SI ({4:P3})" -f `
            $numeric, $name, $x, $y, $delta)
        if ($delta -ge $tolerances[$j]) {
            throw "$numeric $name varies across equivalent physical runs"
        }
    }
}
