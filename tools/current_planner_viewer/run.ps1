$ErrorActionPreference = "Stop"

$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$code = Join-Path $root "project\code"
$outDir = Join-Path $root ".tmp"
$exe = Join-Path $outDir "current_planner_viewer.exe"
$report = Join-Path $outDir "current_planner_paths.html"

New-Item -ItemType Directory -Force $outDir | Out-Null

$sources = @(
    (Join-Path $PSScriptRoot "current_planner_viewer.c"),
    (Join-Path $code "Algorithm.c"),
    (Join-Path $code "Game_logic.c"),
    (Join-Path $code "Map_Path_Data.c"),
    (Join-Path $code "path.c")
)

$gccArgs = @(
    "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
    "-Wno-missing-field-initializers", "-Wno-unused-function",
    "-I", (Join-Path $PSScriptRoot "host_include"),
    "-I", $code
) + $sources + @("-o", $exe, "-lm")

& gcc @gccArgs
if ($LASTEXITCODE -ne 0) {
    throw "current_planner_viewer compilation failed with exit code $LASTEXITCODE"
}

& $exe --output $report @args
if ($LASTEXITCODE -ne 0) {
    throw "current_planner_viewer failed with exit code $LASTEXITCODE"
}

Write-Host "Open this report in a browser: $report"
