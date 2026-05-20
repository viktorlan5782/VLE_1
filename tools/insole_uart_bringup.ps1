param(
    [Parameter(Mandatory = $false)]
    [string]$Port = "COM16",

    [Parameter(Mandatory = $false)]
    [string]$BaudRates = "115200,9600",

    [Parameter(Mandatory = $false)]
    [double]$DurationS = 3.0
)

$ErrorActionPreference = "Stop"

$FrameLength = 39
$Header = 0xAA
$ValidFootIds = @(0x01, 0x02)

function Get-Checksum {
    param([byte[]]$Frame)

    $sum = 0
    for ($i = 0; $i -lt 38; $i++) {
        $sum += $Frame[$i]
    }
    return ($sum -band 0xFF)
}

function Test-InsoleStream {
    param(
        [byte[]]$Bytes
    )

    $buffer = New-Object System.Collections.Generic.List[byte]
    foreach ($b in $Bytes) {
        [void]$buffer.Add($b)
    }

    $validFrames = 0
    $badChecksums = 0
    $badFootIds = 0
    $discardedBytes = 0
    $leftFrames = 0
    $rightFrames = 0

    while ($buffer.Count -ge $FrameLength) {
        if ($buffer[0] -ne $Header) {
            $buffer.RemoveAt(0)
            $discardedBytes++
            continue
        }

        $frame = $buffer.GetRange(0, $FrameLength).ToArray()
        $buffer.RemoveRange(0, $FrameLength)

        $checksum = Get-Checksum -Frame $frame
        if ($checksum -ne $frame[38]) {
            $badChecksums++
            continue
        }

        if ($ValidFootIds -notcontains $frame[1]) {
            $badFootIds++
            continue
        }

        $validFrames++
        if ($frame[1] -eq 0x01) {
            $leftFrames++
        } elseif ($frame[1] -eq 0x02) {
            $rightFrames++
        }
    }

    return [pscustomobject]@{
        valid_frames    = $validFrames
        bad_checksums   = $badChecksums
        bad_foot_ids    = $badFootIds
        discarded_bytes = $discardedBytes
        leftover_bytes  = $buffer.Count
        left_frames     = $leftFrames
        right_frames    = $rightFrames
    }
}

Write-Host "Detected serial ports:" -ForegroundColor Cyan
[System.IO.Ports.SerialPort]::GetPortNames() |
    Sort-Object |
    ForEach-Object { Write-Host "  $_" }

$parsedBaudRates = $BaudRates.Split(",") |
    ForEach-Object { $_.Trim() } |
    Where-Object { $_ -ne "" } |
    ForEach-Object { [int]$_ }

foreach ($baud in $parsedBaudRates) {
    Write-Host ""
    Write-Host "=== $Port @ $baud 8N1 ===" -ForegroundColor Yellow

    $serial = $null
    try {
        $serial = New-Object System.IO.Ports.SerialPort $Port, $baud, "None", 8, "One"
        $serial.ReadTimeout = 100
        $serial.Open()
        $serial.DiscardInBuffer()

        $bytes = New-Object System.Collections.Generic.List[byte]
        $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()

        while ($stopwatch.Elapsed.TotalSeconds -lt $DurationS) {
            try {
                $value = $serial.ReadByte()
                if ($value -ge 0) {
                    [void]$bytes.Add([byte]$value)
                }
            } catch [System.TimeoutException] {
                continue
            }
        }

        $serial.Close()

        $raw = $bytes.ToArray()
        $stats = Test-InsoleStream -Bytes $raw
        $hexPreview = ($raw |
            Select-Object -First 120 |
            ForEach-Object { $_.ToString("X2") }) -join " "

        $observedHz = 0.0
        if ($DurationS -gt 0) {
            $observedHz = $stats.valid_frames / $DurationS
        }

        Write-Host ("bytes_received : {0}" -f $raw.Length)
        Write-Host ("valid_frames   : {0}" -f $stats.valid_frames)
        Write-Host ("left_frames    : {0}" -f $stats.left_frames)
        Write-Host ("right_frames   : {0}" -f $stats.right_frames)
        Write-Host ("bad_checksums  : {0}" -f $stats.bad_checksums)
        Write-Host ("bad_foot_ids   : {0}" -f $stats.bad_foot_ids)
        Write-Host ("discarded_bytes: {0}" -f $stats.discarded_bytes)
        Write-Host ("leftover_bytes : {0}" -f $stats.leftover_bytes)
        Write-Host ("observed_rate_hz: {0:N2}" -f $observedHz)
        Write-Host ("hex_preview    : {0}" -f $hexPreview)

        if ($raw.Length -eq 0) {
            Write-Host "Diagnosis: port opened, but no raw bytes arrived." -ForegroundColor Red
        } elseif ($stats.valid_frames -eq 0) {
            Write-Host "Diagnosis: bytes arrived, but no valid 39-byte pressure-insole packet was decoded." -ForegroundColor Red
        } else {
            Write-Host "Diagnosis: valid pressure-insole UART stream detected." -ForegroundColor Green
        }
    } catch {
        Write-Host ("Open/read failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
        if ($_.Exception.Message -match "denied|拒绝访问") {
            Write-Host "Diagnosis: COM port is already occupied by another process." -ForegroundColor Red
        }
    } finally {
        if ($serial -and $serial.IsOpen) {
            $serial.Close()
        }
    }
}
