param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('forward', 'backward', 'left', 'right')]
    [string]$Direction,

    [Parameter(Mandatory = $true)]
    [double]$DistanceCm,

    [Parameter(Mandatory = $true)]
    [double]$SpeedCmps,

    [Parameter(Mandatory = $true)]
    [double]$Kp,

    [double]$Ki = 0.0,

    [Parameter(Mandatory = $true)]
    [double]$Kd,

    [ValidateRange(0.10, 1.00)]
    [double]$SpeedFactor = 0.85,

    [Parameter(Mandatory = $true)]
    [string]$Label,

    [int]$TimeoutSeconds = 18,

    [string]$PortName = 'COM6',

    [string]$VisionPosePath = ''
)

$ErrorActionPreference = 'Stop'
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$workspace = Split-Path $PSScriptRoot -Parent
$recordRoot = Join-Path $workspace 'project\calibration_records\2026-07-15_position_pid_tuning'
$summaryPath = Join-Path $recordRoot 'summary_envelope.csv'
if ([string]::IsNullOrWhiteSpace($VisionPosePath)) {
    $visionRoot = Get-ChildItem -LiteralPath 'E:\' -Directory -Filter 'SmartCarPoseMonitor*' |
        Where-Object { Test-Path -LiteralPath (Join-Path $_.FullName 'logs\latest_pose.txt') } |
        Select-Object -First 1
    if ($null -ne $visionRoot) {
        $VisionPosePath = Join-Path $visionRoot.FullName 'logs\latest_pose.txt'
    }
}
$safeLabel = $Label -replace '[^A-Za-z0-9_.-]', '_'
$rawPath = Join-Path $recordRoot ($safeLabel + '.log')
$raw = [System.Text.StringBuilder]::new()
$port = [System.IO.Ports.SerialPort]::new(
    $PortName,
    115200,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One)
$port.Handshake = [System.IO.Ports.Handshake]::None
$port.DtrEnable = $false
$port.RtsEnable = $false
$port.ReadTimeout = 300
$port.WriteTimeout = 800
$motionStarted = $false
$done = $false
$timedOut = $false
$doneElapsedS = [double]::NaN
$visionStart = $null
$visionEnd = $null

function Add-RawChunk {
    param([string]$Chunk)

    if (-not [string]::IsNullOrEmpty($Chunk)) {
        [void]$script:raw.Append($Chunk)
    }
}

function Read-ForMilliseconds {
    param([int]$Milliseconds)

    $deadline = (Get-Date).AddMilliseconds($Milliseconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 40
        Add-RawChunk $script:port.ReadExisting()
    }
}

function Send-Frame {
    param(
        [string]$Frame,
        [string]$Expected,
        [int]$WaitMs = 260
    )

    $startLength = $script:raw.Length
    $script:port.Write('[' + $Frame + ']')
    Read-ForMilliseconds $WaitMs
    $reply = $script:raw.ToString().Substring($startLength)
    if (-not [string]::IsNullOrEmpty($Expected) -and
        -not $reply.Contains($Expected)) {
        throw "Command [$Frame] missing reply [$Expected]. Reply: $reply"
    }
    if ($reply.Contains('ERR ')) {
        throw "Command [$Frame] failed. Reply: $reply"
    }
    return $reply
}

function Format-Number {
    param([double]$Value)
    return $Value.ToString('0.####', $script:culture)
}

function Read-VisionPose {
    if (-not (Test-Path -LiteralPath $VisionPosePath)) {
        return $null
    }

    for ($attempt = 0; $attempt -lt 20; $attempt++) {
        try {
            $line = [System.IO.File]::ReadAllText($VisionPosePath).Trim()
            $fields = $line.Split(',')
            if ($fields.Count -lt 4) {
                throw 'Incomplete vision pose frame'
            }
            return [pscustomobject]@{
                timestamp = [datetimeoffset]::Parse($fields[0], $script:culture)
                x_grid = [double]::Parse($fields[1], $script:culture)
                y_grid = [double]::Parse($fields[2], $script:culture)
                yaw_deg = [double]::Parse($fields[3], $script:culture)
            }
        }
        catch {
            Start-Sleep -Milliseconds 20
        }
    }
    return $null
}

New-Item -ItemType Directory -Force -Path $recordRoot | Out-Null

try {
    $port.Open()
    Read-ForMilliseconds 300
    $statusReply = Send-Frame 'status' 'STATUS ' 420
    if ($statusReply -match 'yawhold=0') {
        [void](Send-Frame 'yawhold' 'OK yawhold=on' 420)
        $statusReply = Send-Frame 'status' 'yawhold=1' 420
    }
    elseif ($statusReply -notmatch 'yawhold=1') {
        throw "Cannot verify yawhold state. Reply: $statusReply"
    }

    [void](Send-Frame ('slider,pos.kp,' + (Format-Number $Kp)) 'OK pos.kp=' 260)
    [void](Send-Frame ('slider,pos.ki,' + (Format-Number $Ki)) 'OK pos.ki=' 260)
    [void](Send-Frame ('slider,pos.kd,' + (Format-Number $Kd)) 'OK pos.kd=' 260)
    [void](Send-Frame ('slider,pos.speed.factor,' + (Format-Number $SpeedFactor)) 'OK pos.speed.factor=' 260)
    [void](Send-Frame ('slider,speed,' + (Format-Number $SpeedCmps)) 'OK next_speed=' 260)
    [void](Send-Frame ('slider,position,' + (Format-Number $DistanceCm)) 'OK next_position=' 260)
    [void](Send-Frame ('button,' + $Direction) ('OK direction=' + $Direction) 260)
    $confirmReply = Send-Frame 'status' 'TUNE ' 420
    $expectedPid = 'pospid=' + $Kp.ToString('0.0000', $culture) + ',' +
                   $Ki.ToString('0.0000', $culture) + ',' +
                   $Kd.ToString('0.0000', $culture)
    if (-not $confirmReply.Contains($expectedPid)) {
        throw "Runtime PID confirmation mismatch. Expected $expectedPid. Reply: $confirmReply"
    }
    $expectedFactor = 'pos.speed.factor=' + $SpeedFactor.ToString('0.000', $culture)
    if (-not $confirmReply.Contains($expectedFactor)) {
        throw "Runtime speed-factor confirmation mismatch. Expected $expectedFactor. Reply: $confirmReply"
    }

    $visionStart = Read-VisionPose
    $runWatch = [System.Diagnostics.Stopwatch]::StartNew()
    [void](Send-Frame 'button,start' 'OK start direction=' 260)
    # Drop idle telemetry that can share the start-command reply chunk. It
    # carries the previous TPOS and would otherwise look like a huge overshoot.
    $motionRawStart = $raw.Length
    $motionStarted = $true
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        Read-ForMilliseconds 80
        $motionText = $raw.ToString().Substring($motionRawStart)
        if ($motionText -match '(?m)(^|\r?\n)(SEG_DONE|DONE)(\r?\n|$)') {
            $done = $true
            $doneElapsedS = $runWatch.Elapsed.TotalSeconds
            break
        }
    }
    if ($done) {
        Read-ForMilliseconds 120
        $visionEnd = Read-VisionPose
        $motionTextForMetrics = $raw.ToString().Substring($motionRawStart)
        [void](Send-Frame 'stop' 'OK stop' 300)
    }
    else {
        $timedOut = $true
        $visionEnd = Read-VisionPose
        $motionTextForMetrics = $raw.ToString().Substring($motionRawStart)
        [void](Send-Frame 'stop' 'OK stop' 500)
    }
}
catch {
    if ($motionStarted -and $port.IsOpen) {
        try {
            $port.Write('[stop]')
            Read-ForMilliseconds 500
        }
        catch {
        }
    }
    $failureText = $_.Exception.Message
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
    $port.Dispose()
    $raw.ToString() | Set-Content -Path $rawPath -Encoding UTF8
}

if (-not [string]::IsNullOrEmpty($failureText)) {
    throw $failureText
}

$telemetryPattern = [regex]::new(
    'TPOS=(?<tx>-?\d+(?:\.\d+)?),(?<ty>-?\d+(?:\.\d+)?) APOS=(?<ax>-?\d+(?:\.\d+)?),(?<ay>-?\d+(?:\.\d+)?) ' +
    'TVEL=(?<tv>-?\d+(?:\.\d+)?) AVEL=(?<av>-?\d+(?:\.\d+)?) VCAP=(?<cap>-?\d+(?:\.\d+)?) ' +
    'PSLIM=(?<pslim>-?\d+(?:\.\d+)?) BRK=(?<brk>\d+) POS=(?<pos>\d+) LINE=(?<line>\d+) ' +
    'TYAW=(?<tyaw>-?\d+(?:\.\d+)?) AYAW=(?<ayaw>-?\d+(?:\.\d+)?)')
$matches = $telemetryPattern.Matches($motionTextForMetrics)
if ($matches.Count -eq 0) {
    throw "No motion telemetry found. Set BLUESERIAL_PERIODIC_TELEMETRY_ENABLE=1, rebuild, and retry. Raw log: $rawPath"
}

$remainingValues = [System.Collections.Generic.List[double]]::new()
$peakActualSpeed = 0.0
$brakingSamples = 0
$positionLoopSamples = 0
$lineGuidanceSamples = 0
$maxYawError = 0.0
$restartCount = 0
$reversalCount = 0
$hasMoved = $false
$stoppedAfterMove = $false
$previousRemaining = [double]::NaN
$lastMotionSign = 0
foreach ($match in $matches) {
    $tx = [double]::Parse($match.Groups['tx'].Value, $culture)
    $ty = [double]::Parse($match.Groups['ty'].Value, $culture)
    $ax = [double]::Parse($match.Groups['ax'].Value, $culture)
    $ay = [double]::Parse($match.Groups['ay'].Value, $culture)
    $actualSpeed = [double]::Parse($match.Groups['av'].Value, $culture)
    $targetYaw = [double]::Parse($match.Groups['tyaw'].Value, $culture)
    $actualYaw = [double]::Parse($match.Groups['ayaw'].Value, $culture)
    switch ($Direction) {
        'forward'  { $remaining = ($tx - $ax) * 100.0 }
        'backward' { $remaining = ($ax - $tx) * 100.0 }
        'left'     { $remaining = ($ty - $ay) * 100.0 }
        'right'    { $remaining = ($ay - $ty) * 100.0 }
    }
    $remainingValues.Add($remaining)
    $peakActualSpeed = [Math]::Max($peakActualSpeed, $actualSpeed)
    if ([int]$match.Groups['brk'].Value -ne 0) {
        $brakingSamples++
    }
    if ([int]$match.Groups['pos'].Value -ne 0) {
        $positionLoopSamples++
    }
    if ([int]$match.Groups['line'].Value -ne 0) {
        $lineGuidanceSamples++
    }
    if (-not $hasMoved -and $actualSpeed -ge 3.0) {
        $hasMoved = $true
    }
    elseif ($hasMoved) {
        if ($actualSpeed -le 1.0) {
            $stoppedAfterMove = $true
        }
        elseif ($stoppedAfterMove -and $actualSpeed -ge 3.0) {
            $restartCount++
            $stoppedAfterMove = $false
        }
    }
    if (-not [double]::IsNaN($previousRemaining)) {
        $deltaRemaining = $remaining - $previousRemaining
        $motionSign = 0
        if ($deltaRemaining -le -0.20) {
            $motionSign = 1
        }
        elseif ($deltaRemaining -ge 0.20) {
            $motionSign = -1
        }
        if ($motionSign -ne 0) {
            if ($lastMotionSign -ne 0 -and $motionSign -ne $lastMotionSign) {
                $reversalCount++
            }
            $lastMotionSign = $motionSign
        }
    }
    $previousRemaining = $remaining
    $yawError = [Math]::Abs($targetYaw - $actualYaw)
    if ($yawError -gt 180.0) {
        $yawError = 360.0 - $yawError
    }
    $maxYawError = [Math]::Max($maxYawError, $yawError)
}

$last = $matches[$matches.Count - 1]
$finalTx = [double]::Parse($last.Groups['tx'].Value, $culture)
$finalTy = [double]::Parse($last.Groups['ty'].Value, $culture)
$finalAx = [double]::Parse($last.Groups['ax'].Value, $culture)
$finalAy = [double]::Parse($last.Groups['ay'].Value, $culture)
$finalActualSpeed = [double]::Parse($last.Groups['av'].Value, $culture)
$finalRemaining = $remainingValues[$remainingValues.Count - 1]
$minRemaining = ($remainingValues | Measure-Object -Minimum).Minimum
$overshootCm = [Math]::Max(0.0, -$minRemaining)
$radialErrorCm = [Math]::Sqrt(
    [Math]::Pow(($finalTx - $finalAx) * 100.0, 2.0) +
    [Math]::Pow(($finalTy - $finalAy) * 100.0, 2.0))
if ($Direction -eq 'forward' -or $Direction -eq 'backward') {
    $crossErrorCm = [Math]::Abs(($finalTy - $finalAy) * 100.0)
}
else {
$crossErrorCm = [Math]::Abs(($finalTx - $finalAx) * 100.0)
}

$visionStartTime = ''
$visionEndTime = ''
$visionStartX = ''
$visionStartY = ''
$visionStartYaw = ''
$visionEndX = ''
$visionEndY = ''
$visionEndYaw = ''
$visionDxCm = ''
$visionDyCm = ''
$visionTravelCm = ''
$visionYawChangeDeg = ''
if ($null -ne $visionStart -and $null -ne $visionEnd) {
    $visionStartTime = $visionStart.timestamp.ToString('o')
    $visionEndTime = $visionEnd.timestamp.ToString('o')
    $visionStartX = [Math]::Round($visionStart.x_grid, 4)
    $visionStartY = [Math]::Round($visionStart.y_grid, 4)
    $visionStartYaw = [Math]::Round($visionStart.yaw_deg, 3)
    $visionEndX = [Math]::Round($visionEnd.x_grid, 4)
    $visionEndY = [Math]::Round($visionEnd.y_grid, 4)
    $visionEndYaw = [Math]::Round($visionEnd.yaw_deg, 3)
    $visionDxCm = [Math]::Round(($visionEnd.x_grid - $visionStart.x_grid) * 20.0, 3)
    $visionDyCm = [Math]::Round(($visionEnd.y_grid - $visionStart.y_grid) * 20.0, 3)
    $visionTravelCm = [Math]::Round([Math]::Sqrt(
        [Math]::Pow($visionDxCm, 2.0) + [Math]::Pow($visionDyCm, 2.0)), 3)
    $yawChange = $visionEnd.yaw_deg - $visionStart.yaw_deg
    while ($yawChange -gt 180.0) { $yawChange -= 360.0 }
    while ($yawChange -lt -180.0) { $yawChange += 360.0 }
    $visionYawChangeDeg = [Math]::Round($yawChange, 3)
}

$result = [pscustomobject]@{
    timestamp = (Get-Date).ToString('s')
    label = $Label
    direction = $Direction
    distance_cm = [Math]::Round($DistanceCm, 3)
    speed_cmps = [Math]::Round($SpeedCmps, 3)
    kp = [Math]::Round($Kp, 4)
    ki = [Math]::Round($Ki, 4)
    kd = [Math]::Round($Kd, 4)
    speed_factor = [Math]::Round($SpeedFactor, 3)
    done = [int]$done
    timeout = [int]$timedOut
    completion_s = if ([double]::IsNaN($doneElapsedS)) { '' } else { [Math]::Round($doneElapsedS, 3) }
    final_remaining_cm = [Math]::Round($finalRemaining, 3)
    radial_error_cm = [Math]::Round($radialErrorCm, 3)
    max_overshoot_cm = [Math]::Round($overshootCm, 3)
    final_cross_error_cm = [Math]::Round($crossErrorCm, 3)
    final_actual_cmps = [Math]::Round($finalActualSpeed, 3)
    peak_actual_cmps = [Math]::Round($peakActualSpeed, 3)
    restart_count = $restartCount
    reversal_count = $reversalCount
    braking_samples = $brakingSamples
    position_loop_samples = $positionLoopSamples
    line_guidance_samples = $lineGuidanceSamples
    max_yaw_error_deg = [Math]::Round($maxYawError, 3)
    telemetry_samples = $matches.Count
    vision_start_time = $visionStartTime
    vision_end_time = $visionEndTime
    vision_start_x_grid = $visionStartX
    vision_start_y_grid = $visionStartY
    vision_start_yaw_deg = $visionStartYaw
    vision_end_x_grid = $visionEndX
    vision_end_y_grid = $visionEndY
    vision_end_yaw_deg = $visionEndYaw
    vision_dx_cm = $visionDxCm
    vision_dy_cm = $visionDyCm
    vision_travel_cm = $visionTravelCm
    vision_yaw_change_deg = $visionYawChangeDeg
    raw_log = $rawPath
}

if (-not (Test-Path $summaryPath)) {
    $result | Export-Csv -Path $summaryPath -NoTypeInformation -Encoding UTF8
}
else {
    $result | ConvertTo-Csv -NoTypeInformation |
        Select-Object -Skip 1 |
        Add-Content -Path $summaryPath -Encoding UTF8
}

$result | Format-List
