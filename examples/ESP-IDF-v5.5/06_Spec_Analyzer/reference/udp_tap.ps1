param(
    [int]$Port = 8765,
    [switch]$Raw
)

$udp = [System.Net.Sockets.UdpClient]::new($Port)
$udp.Client.ReceiveTimeout = 250
$ep = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)

Write-Host "Listening on UDP $Port. Press 'q' to quit." -ForegroundColor Cyan

try {
    while ($true) {
        if ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)
            if ($key.Key -eq 'Q') {
                break
            }
        }

        try {
            $buf = $udp.Receive([ref]$ep)
        }
        catch [System.Net.Sockets.SocketException] {
            if ($_.Exception.SocketErrorCode -eq [System.Net.Sockets.SocketError]::TimedOut) {
                continue
            }
            throw
        }

        $txt = [Text.Encoding]::UTF8.GetString($buf)
        if ($Raw) {
            Write-Host ("{0}:{1} {2}" -f $ep.Address, $ep.Port, $txt)
            continue
        }

        try {
            $obj = $txt | ConvertFrom-Json -ErrorAction Stop
            $points = @($obj.points).Count
            Write-Host ("{0}:{1} src={2} scan={3} speed={4} points={5} crc_fail={6}" -f  $ep.Address, $ep.Port, $obj.source, $obj.scan, $obj.speed, $points, $obj.crc_fail)
        }
        catch {
            Write-Host ("{0}:{1} RAW {2}" -f $ep.Address, $ep.Port, $txt)
        }
    }
}
finally {
    $udp.Close()
    Write-Host "UDP tap stopped."
}
