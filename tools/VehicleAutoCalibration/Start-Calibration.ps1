param(
    [ValidateSet('kinematics', 'yaw', 'position', 'guide', 'scurve', 'validation', 'lateral-validation', 'all')]
    [string]$Phase = 'kinematics',
    [string]$Port = '',
    [string]$BluetoothAddress = ''
)

$ErrorActionPreference = 'Stop'
$toolDirectory = $PSScriptRoot
$executable = Join-Path $toolDirectory 'VehicleAutoCalibration.exe'

if (-not (Test-Path -LiteralPath $executable)) {
    & (Join-Path $toolDirectory 'Build.ps1')
}

$monitor = Get-ChildItem -LiteralPath 'D:\workwork' -Recurse `
                         -Filter 'latest_pose.txt' -File -ErrorAction SilentlyContinue |
           Where-Object { $_.FullName -like '*SmartCarPoseMonitor*\logs\latest_pose.txt' } |
           Sort-Object LastWriteTime -Descending |
           Select-Object -First 1
if (-not $monitor) {
    throw 'Cannot find SmartCarPoseMonitor logs\latest_pose.txt under D:\workwork.'
}

$arguments = @('--monitor', $monitor.FullName, '--phase', $Phase)
if ($Port) {
    $arguments += @('--port', $Port)
}
if ($BluetoothAddress) {
    $arguments += @('--bt-address', $BluetoothAddress)
}

& $executable @arguments
exit $LASTEXITCODE
