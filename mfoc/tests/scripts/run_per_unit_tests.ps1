$ErrorActionPreference = 'Stop'
$tests = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $tests
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-pu-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
Write-Output "PU test artifacts: $output"

$modules = @('math', 'control', 'core', 'motor', 'position')
$includes = @("-I$library", "-I$library/include",
    "-I$library/port/include") + @($modules | ForEach-Object {
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

# Reuse the independent P09 PMSM fixture under both compile-time profiles.
$profiles = @(
    @{ Name = '2804'; Path = ''; Bases = 'BASES 3500 12000 400' },
    @{ Name = 'alternate'; Path = (Join-Path $tests 'motor/motor_config_test.h');
        Bases = 'BASES 4000 14000 320' }
)
$results = @()
foreach ($profile in $profiles) {
    $log = & (Join-Path $PSScriptRoot 'run_core_tests.ps1') `
        -Tests motor_identification_plant -Backend 0 -Position 0 -Hfi 0 `
        -Identify 1 -MotorConfig $profile.Path
    $log | Set-Content -LiteralPath (Join-Path $output "plant-$($profile.Name).log")
    $bases = @($log | Where-Object { $_ -match '^BASES ' })
    if ($bases.Count -ne 2 -or ($bases | Where-Object { $_ -ne $profile.Bases })) {
        throw "Plant did not compile with the $($profile.Name) profile bases"
    }
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
