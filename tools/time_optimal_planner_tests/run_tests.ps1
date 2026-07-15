$ErrorActionPreference = "Stop"

$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$planner = Join-Path $root "project\code\time_optimal_planner"
$out = Join-Path $root ".tmp"
New-Item -ItemType Directory -Force $out | Out-Null

$common = @(
    (Join-Path $planner "TopGrid.c"),
    (Join-Path $planner "TopPath.c"),
    (Join-Path $planner "TopPlanner.c"),
    (Join-Path $planner "TopVerify.c")
)

function Invoke-GccTest {
    param(
        [string]$Name,
        [string[]]$ExtraSources,
        [string]$Optimization = "-O2"
    )
    $exe = Join-Path $out "$Name.exe"
    $arguments = @(
        "-std=c99", $Optimization, "-Wall", "-Wextra", "-Werror",
        "-I", $planner
    ) + $common + $ExtraSources + @("-o", $exe, "-lm")
    & gcc @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Name compilation failed with exit code $LASTEXITCODE"
    }
    & $exe
    if ($LASTEXITCODE -ne 0) {
        throw "$Name failed with exit code $LASTEXITCODE"
    }
}

Invoke-GccTest -Name "top_smoke" -ExtraSources @(
    (Join-Path $planner "TopControlV2.c"),
    (Join-Path $PSScriptRoot "test_top_planner.c")
)
Invoke-GccTest -Name "top_fuzz" -ExtraSources @(
    (Join-Path $PSScriptRoot "fuzz_top_planner.c")
) -Optimization "-O3"
Invoke-GccTest -Name "top_benchmark" -ExtraSources @(
    (Join-Path $PSScriptRoot "benchmark_top_planner.c")
) -Optimization "-O3"

Write-Host "All time-optimal planner host tests passed."
