$ErrorActionPreference = 'Stop'
$toolDirectory = $PSScriptRoot
$compiler = 'C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe'
$output = Join-Path $toolDirectory 'VehicleAutoCalibration.exe'
$sources = Get-ChildItem -LiteralPath (Join-Path $toolDirectory 'src') -Filter '*.cs' |
           Select-Object -ExpandProperty FullName

if (-not (Test-Path -LiteralPath $compiler)) {
    throw "C# compiler not found: $compiler"
}

& $compiler /nologo /optimize+ /target:exe /out:$output $sources
if ($LASTEXITCODE -ne 0) {
    throw 'VehicleAutoCalibration compilation failed.'
}

Write-Host "Built: $output" -ForegroundColor Green
