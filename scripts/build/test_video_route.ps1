# SPDX-License-Identifier: Apache-2.0
# Bounded transport diagnostic. Opens only the reserved simulation video port.
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
if (Get-NetUDPEndpoint -LocalPort 15600 -ErrorAction SilentlyContinue) { throw 'Reserved UDP port is occupied.' }
$nonce = [Guid]::NewGuid().ToString('N')
$receiver = [Net.Sockets.UdpClient]::new([Net.IPEndPoint]::new([Net.IPAddress]::Any, 15600))
$receiver.Client.ReceiveTimeout = 2000
$sender = [Net.Sockets.UdpClient]::new()
try {
    $local = [Text.Encoding]::ASCII.GetBytes('local-' + $nonce)
    [void]$sender.Send($local, $local.Length, '127.0.0.1', 15600)
    $peer = [Net.IPEndPoint]::new([Net.IPAddress]::Any, 0)
    if ([Text.Encoding]::ASCII.GetString($receiver.Receive([ref]$peer)) -ne ('local-' + $nonce)) {
        throw 'Local UDP positive control failed.'
    }
    $script = & wsl -d Ubuntu-24.04 --exec wslpath -a (Join-Path $root 'scripts/simulation/probe_udp.py')
    $sent = & wsl -d Ubuntu-24.04 --exec python3 $script $nonce
    if ($LASTEXITCODE -ne 0) { throw 'WSL route probe sender failed.' }
    $sent = $sent | ConvertFrom-Json
    $windowsGateway = [bool](Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object IPAddress -eq $sent.gateway)
    $received = 0
    for ($index = 0; $index -lt 5; $index++) {
        try {
            $data = $receiver.Receive([ref]$peer)
            if ([Text.Encoding]::ASCII.GetString($data) -eq ('NOMAD_ROUTE_' + $nonce)) { $received++ }
        } catch [Net.Sockets.SocketException] {
            if ($_.Exception.SocketErrorCode -ne [Net.Sockets.SocketError]::TimedOut) { throw }
            break
        }
    }
    $result = @{schema=1; local_positive=$true; gateway_is_windows=$windowsGateway;
        wsl_sent=$sent.sent; windows_received=$received; direct_route_passed=($windowsGateway -and $received -eq 5)}
    $directory = Join-Path $root 'build/tests/simulation'
    New-Item -ItemType Directory -Path $directory -Force | Out-Null
    $result | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $directory ('udp-route-' + $nonce + '.json'))
    $result | ConvertTo-Json
} finally { $sender.Dispose(); $receiver.Dispose() }
