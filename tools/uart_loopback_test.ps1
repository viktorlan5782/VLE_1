param(
    [Parameter(Mandatory = $false)]
    [string]$Port = "COM16",

    [Parameter(Mandatory = $false)]
    [int]$BaudRate = 115200,

    [Parameter(Mandatory = $false)]
    [int]$TimeoutMs = 500
)

$ErrorActionPreference = "Stop"

$Pattern = [byte[]](0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34, 0x56, 0x78)

Write-Host "UART loopback test" -ForegroundColor Cyan
Write-Host "Before running: disconnect the insole, then short USB-TTL TXD <-> RXD." -ForegroundColor Yellow
Write-Host "Port: $Port, BaudRate: $BaudRate 8N1"

$serial = $null
try {
    $serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, "None", 8, "One"
    $serial.ReadTimeout = $TimeoutMs
    $serial.WriteTimeout = $TimeoutMs
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $serial.Write($Pattern, 0, $Pattern.Length)

    $received = New-Object byte[] $Pattern.Length
    $offset = 0
    while ($offset -lt $Pattern.Length) {
        try {
            $n = $serial.Read($received, $offset, $Pattern.Length - $offset)
            if ($n -le 0) {
                break
            }
            $offset += $n
        } catch [System.TimeoutException] {
            break
        }
    }

    $sentHex = ($Pattern | ForEach-Object { $_.ToString("X2") }) -join " "
    $recvHex = ""
    if ($offset -gt 0) {
        $recvHex = ($received[0..($offset - 1)] | ForEach-Object { $_.ToString("X2") }) -join " "
    }

    Write-Host "sent    : $sentHex"
    Write-Host "received: $recvHex"

    $sameLength = ($offset -eq $Pattern.Length)
    $sameBytes = $true
    if ($sameLength) {
        for ($i = 0; $i -lt $Pattern.Length; $i++) {
            if ($received[$i] -ne $Pattern[$i]) {
                $sameBytes = $false
                break
            }
        }
    } else {
        $sameBytes = $false
    }

    if ($sameBytes) {
        Write-Host "Diagnosis: USB-TTL adapter TX/RX loopback is OK." -ForegroundColor Green
    } else {
        Write-Host "Diagnosis: loopback failed; suspect USB-TTL adapter, driver, or the physical TX/RX pins being used." -ForegroundColor Red
    }
} catch {
    Write-Host ("Loopback test failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
} finally {
    if ($serial -and $serial.IsOpen) {
        $serial.Close()
    }
}
