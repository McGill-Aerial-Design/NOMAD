# SPDX-License-Identifier: Apache-2.0
param(
    [switch]$Check
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$base = Join-Path $root 'build/conference'
$prepared = Join-Path $base 'prepared'
if (!(Test-Path -LiteralPath (Join-Path $prepared 'manifest.json'))) {
    throw 'Run conference/setup.ps1 first.'
}
$manifest = Get-Content -LiteralPath (Join-Path $prepared 'manifest.json') -Raw | ConvertFrom-Json
if (!$manifest.files) {
    throw 'Run conference/setup.ps1 to refresh the prepared manifest.'
}
foreach ($file in $manifest.files) {
    $path = [IO.Path]::GetFullPath((Join-Path $base $file.name))
    if (!$path.StartsWith($base + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase) -or
        (Get-FileHash -LiteralPath $path).Hash -ne $file.sha256) {
        throw "Prepared file changed: $($file.name). Run setup again."
    }
}
$pluginFiles = @(Get-ChildItem -LiteralPath (Join-Path $prepared 'host/plugins') -Recurse -File)
if ($pluginFiles.Count -ne 1 -or $pluginFiles[0].Name -ne 'NOMADPlugin.dll') {
    throw 'The conference host must contain only NOMADPlugin.dll in its plugins folder.'
}
if (Get-Process MissionPlanner, NOMADHost -ErrorAction SilentlyContinue) {
    throw 'Close Mission Planner before starting the isolated conference session.'
}
foreach ($port in @(6310, 6312, 6313, 6410, 6412, 8554, 8989, 8990, 9002, 9003, 14600)) {
    if ((Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue) -or
        (Get-NetUDPEndpoint -LocalPort $port -ErrorAction SilentlyContinue)) {
        throw "Demo port $port is occupied."
    }
}
$run = Join-Path $base ('run-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' +
    [Guid]::NewGuid().ToString('N').Substring(0, 6))
New-Item -ItemType Directory -Path $run | Out-Null
Copy-Item -Path (Join-Path $prepared 'config/*') -Destination $run
Copy-Item -LiteralPath (Join-Path $prepared 'gst-registry.bin') -Destination $run
Copy-Item -LiteralPath (Join-Path $prepared 'mp-gst-registry.bin') -Destination $run
$appName = 'NOMADConference_' + [IO.Path]::GetFileName($run)
$profileDirectory = Join-Path (Join-Path $run $appName) 'plugins'
New-Item -ItemType Directory -Path $profileDirectory -Force | Out-Null
$id = [Guid]::NewGuid().ToString('N')
$config = @{
    AircraftName = 'NOMAD Inspection / SITL'
    AircraftFirmware = 'ArduCopter 4.8.0-dev b7ba8bca'
    FlightControllerModel = 'Project AirSim quadrotor'
    AudioAlerts = $false
    AltitudeCallouts = $false
    AutoStartHudVideo = $false
    GimbalArrowKeysEnabled = $false
    Inputs = @()
    RouterClientEnabled = $true
    RouterLocalPort = 14600
    RouterPreferredLink = 'radio'
    RouterLinks = @(
        @{
            Id = 'radio'
            Name = 'Telemetry radio'
            Transport = 'TCP'
            RemoteHost = '127.0.0.1'
            Port = 6410
            Priority = 0
        },
        @{
            Id = 'network'
            Name = 'Telemetry network'
            Transport = 'TCP'
            RemoteHost = '127.0.0.1'
            Port = 6412
            Priority = 1
        }
    )
    VideoInputs = @(
        @{
            Name = 'Inspection gimbal'
            Url = 'rtsp://127.0.0.1:8554/inspection'
            RtspTransport = 'TCP'
            LatencyMs = 60
            QueueMaxBuffers = 1
            DropOldFrames = $true
        },
        @{
            Name = 'Chase camera'
            Url = 'rtsp://127.0.0.1:8554/chase'
            RtspTransport = 'TCP'
            LatencyMs = 60
            QueueMaxBuffers = 1
            DropOldFrames = $true
        }
    )
    SelectedVideoInputName = 'Inspection gimbal'
    ActuatorDefinitions = @(@{
        id = 'marker'
        name = 'Inspection marker'
        behavior = 'ServoToggle'
        channel = 9
        safe_pwm = 1000
        hazardous = $true
        confirmation_count = 2
        require_neutral = $true
        primary_label = 'Release marker'
        secondary_label = 'Secure marker'
    })
}
@{
    Aircraft = @(@{
        Id = $id
        Config = $config
    })
    SelectedId = $id
} | ConvertTo-Json -Depth 20 |
    Set-Content -LiteralPath (Join-Path $profileDirectory 'nomad_aircraft_profiles.json') -Encoding utf8
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'validate_profile.ps1') -RunDirectory $run
if ($LASTEXITCODE -ne 0) {
    throw 'Demo profile validation failed before starting any simulator or host.'
}
$xml = [xml]'<?xml version="1.0"?><Config/>'
$settings = @{
    AutoConnect = '[]'
    DisabledPlugins = ''
    newuser = '11/02/2014'
    speechenable = 'False'
    beta_updates = 'False'
    mapCache = 'CacheOnly'
    CHK_resetapmonconnect = 'False'
    CHK_rtsresetesp32 = 'False'
    CONNECT_TIMEOUT_SECONDS = '15'
    logdirectory = (Join-Path $run 'native-logs')
    update_check = (Get-Date).ToShortDateString()
    APMFirmware = 'ArduCopter'
}
foreach ($key in $settings.Keys) {
    $entry = $xml.CreateElement($key)
    $entry.InnerText = $settings[$key]
    [void]$xml.DocumentElement.AppendChild($entry)
}
$xml.Save((Join-Path $run 'mp.config.xml'))
@'
logLevel: warn
rtspAddress: 127.0.0.1:8554
rtspTransports: [tcp]
rtmp: false
hls: false
webrtc: false
srt: false
moq: false
paths:
  inspection:
    source: publisher
  chase:
    source: publisher
'@ | Set-Content -LiteralPath (Join-Path $run 'mediamtx.yml') -Encoding ascii
'{"radio":true,"network":true}' | Set-Content -LiteralPath (Join-Path $run 'links.json') -Encoding ascii
$owned = @()
$encoders = @()
$cameraChildren = @()
function Register-Encoders {
    if (!$camera) {
        return
    }
    $expected = 'C:\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe'
    # Windows venv redirectors can put a second Python process between us and GStreamer.
    $parents = @($camera.Process.Id)
    for ($level = 0; $level -lt 3; $level++) {
        $next = @()
        foreach ($parent in $parents) {
            foreach ($child in Get-CimInstance Win32_Process -Filter "ParentProcessId=$parent") {
                if ($child.CreationDate -lt $camera.Process.StartTime) {
                    continue
                }
                $next += $child.ProcessId
                if ($script:cameraChildren | Where-Object { $_.Id -eq $child.ProcessId }) {
                    continue
                }
                try {
                    $process = [Diagnostics.Process]::GetProcessById($child.ProcessId)
                    [void]$process.Handle
                    $script:cameraChildren += $process
                    if ($child.ExecutablePath -eq $expected) {
                        $script:encoders += $process
                    }
                } catch [ArgumentException] {
                }
            }
        }
        $parents = $next
    }
}
function Read-CameraState {
    for ($attempt = 0; $attempt -lt 10; $attempt++) {
        try {
            return (Get-Content -LiteralPath (Join-Path $run 'camera-status.json') -Raw -ErrorAction Stop |
                ConvertFrom-Json)
        } catch {
            Start-Sleep -Milliseconds 50
        }
    }
    throw 'Camera status could not be read.'
}
function Start-Owned([string]$File, [string]$Arguments, [string]$Name, [bool]$Visible = $false) {
    $info = [Diagnostics.ProcessStartInfo]::new()
    $info.FileName = $File
    $info.Arguments = $Arguments
    $info.WorkingDirectory = $run
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    $info.WindowStyle = if ($Visible) {
        'Normal'
    } else {
        'Hidden'
    }
    $info.EnvironmentVariables['NOMAD_CONFERENCE_RUN'] = $run
    $info.EnvironmentVariables['NOMAD_CONFERENCE_CHECK'] = [string][int]$Check.IsPresent
    $info.EnvironmentVariables['GST_REGISTRY'] = (Join-Path $run $(
        if ($Name -eq 'mission-planner') {
            'mp-gst-registry.bin'
        } else {
            'gst-registry.bin'
        }
    ))
    if ($Check -and $Name -eq 'mission-planner') {
        $info.EnvironmentVariables['GST_DEBUG'] = '2'
    }
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    $process = [Diagnostics.Process]::Start($info)
    return @{
        Process = $process
        Out = $process.StandardOutput.ReadToEndAsync()
        Error = $process.StandardError.ReadToEndAsync()
        Name = $Name
    }
}
$preserve = @((Join-Path $env:USERPROFILE 'Documents/Mission Planner/config.xml'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json.bak'),
    (Join-Path ${env:ProgramFiles(x86)} 'Mission Planner/plugins/NOMADPlugin.dll'))
$before = @{}
foreach ($file in $preserve) {
    $before[$file] = if (Test-Path -LiteralPath $file) {
        (Get-FileHash -LiteralPath $file).Hash
    } else {
        ''
    }
}
$python = Join-Path $base 'python/Scripts/python.exe'
$mp = $null
$camera = $null
try {
    $renderArgs = if ($Check) {
        '-RenderOffScreen -ResX=960 -ResY=540 -nosound'
    } else {
        '-windowed -ResX=1280 -ResY=720 -nosound'
    }
    $owned += Start-Owned `
        (Join-Path $base 'simulators/neighborhood-1.0.1/AirSimNH/Binaries/Win64/AirSimNH-Win64-Shipping.exe') `
        ($renderArgs + ' -UserDir="' + (Join-Path $run 'unreal-user') +
            '" -abslog="' + (Join-Path $run 'renderer.log') + '"') 'renderer' (!$Check)
    $renderer = $owned[-1].Process
    $owned += Start-Owned (Join-Path $prepared 'sitl/ArduCopter.exe') `
        '-Mairsim-copter -I55 -O47.641468,-122.140165,122,0 --sim-address 127.0.0.1 --sim-port-in 9003 --sim-port-out 9002 --serial0 tcp:0 --serial1 tcp:2 --serial2 tcp:3 --defaults copter.parm,airsim-quadX.parm,project-airsim-quad.param,outputs.parm' 'sitl'
    $simulator = $owned[-1].Process
    $owned += Start-Owned (Join-Path $base 'simulators/mediamtx-1.21.2/mediamtx.exe') `
        ('"' + (Join-Path $run 'mediamtx.yml') + '"') 'rtsp'
    $deadline = (Get-Date).AddSeconds(25)
    while (!(Get-NetTCPConnection -LocalPort 8990 -State Listen -OwningProcess $renderer.Id `
        -ErrorAction SilentlyContinue)) {
        if ((Get-Date) -gt $deadline) {
            throw 'Renderer service did not start.'
        }
        Start-Sleep -Milliseconds 200
    }
    $owned += Start-Owned $python ('"' + (Join-Path $PSScriptRoot 'links.py') + '" "' + $run + '"') 'links'
    $camera = Start-Owned $python ('"' + (Join-Path $PSScriptRoot 'camera.py') +
        '" --config-dir "' + $run + '" --output-dir "' + $run + '" --sitl-port 6313') 'camera'
    $owned += $camera
    $deadline = (Get-Date).AddSeconds(45)
    do {
        $ready = $false
        try {
            $ready = (Get-Content -LiteralPath (Join-Path $run 'camera-status.json') -Raw -ErrorAction Stop |
                ConvertFrom-Json).passed
        } catch {
        }
        if ((Get-Date) -gt $deadline -or $camera.Process.HasExited) {
            throw 'Camera bridge did not become ready; inspect the owned run logs.'
        }
        Start-Sleep -Milliseconds 200
    } while (!$ready)
    try {
        Register-Encoders
    } catch {
        Write-Warning "Camera child inventory failed: $_"
    }
    if ($encoders.Count -ne 2) {
        throw 'The camera bridge must own exactly two encoder processes.'
    }
    foreach ($port in @(6310, 6312, 6313)) {
        if (!(Get-NetTCPConnection -State Listen -LocalPort $port -OwningProcess $simulator.Id `
            -ErrorAction SilentlyContinue)) {
            throw "Port $port does not belong to the owned simulator."
        }
    }
    $mp = Start-Owned (Join-Path $prepared 'NOMADHost.exe') `
        ('"' + (Join-Path $prepared 'host/MissionPlanner.exe') + '" "' + $run +
            '" -config "' + (Join-Path $run 'mp.config.xml') +
            '" -script "' + (Join-Path $PSScriptRoot 'mission_planner.py') + '"') 'mission-planner' (!$Check)
    $owned += $mp
    Write-Output "NOMAD conference session: $run"
    if ($Check) {
        if (!$mp.Process.WaitForExit(240000)) {
            throw 'Native conference rehearsal exceeded its deadline.'
        }
    } else {
        $mp.Process.WaitForExit()
    }
    if ($mp.Process.ExitCode -ne 0) {
        throw "Native host exited $($mp.Process.ExitCode)."
    }
    if ($Check) {
        $result = Get-Content -LiteralPath (Join-Path $run 'result.json') -Raw | ConvertFrom-Json
        if (!$result.passed) {
            throw 'Native rehearsal failed; see result.json.'
        }
        $cameraState = Read-CameraState
        if ($camera.Process.HasExited -or !$cameraState.passed -or
            ((Get-Date) - (Get-Item -LiteralPath (Join-Path $run 'camera-status.json')).LastWriteTime).TotalSeconds -gt 3) {
            throw 'The live camera bridge failed during rehearsal.'
        }
        foreach ($stream in $cameraState.streams.PSObject.Properties.Value) {
            if (!$stream.encoder_running -or $stream.frame_age_seconds -gt 3) {
                throw 'A live rendered camera stopped.'
            }
        }
        # Exercise camera failure only after authoritative landed/disarmed observations.
        $encoders[0].Kill()
        if (!$camera.Process.WaitForExit(20000)) {
            throw 'Camera bridge did not retire after encoder failure.'
        }
        $failedCamera = Read-CameraState
        if ($camera.Process.ExitCode -eq 0 -or $failedCamera.passed -or !$failedCamera.stopped) {
            throw 'Encoder failure was not reported as a stopped failed camera bridge.'
        }
        foreach ($encoder in $encoders) {
            if (!$encoder.WaitForExit(5000)) {
                throw 'Sibling encoder survived camera failure.'
            }
        }
        @{
            encoder_failure_detected = $true
            sibling_stopped = $true
            camera_exit_code = $camera.Process.ExitCode
        } |
            ConvertTo-Json | Set-Content -LiteralPath (Join-Path $run 'lifecycle.json') -Encoding utf8
        Write-Output 'PASS native flight, AUTO survey, video, gimbal, failover, payload, landing and encoder failure cleanup.'
    }
} finally {
    try {
        Register-Encoders
    } catch {
        Write-Warning "Camera child inventory failed: $_"
    }
    New-Item -ItemType File -Path (Join-Path $run 'camera.stop'), (Join-Path $run 'links.stop') -Force | Out-Null
    if ($mp -and !$mp.Process.HasExited) {
        [void]$mp.Process.CloseMainWindow()
        [void]$mp.Process.WaitForExit(8000)
    }
    if ($camera -and !$camera.Process.HasExited) {
        [void]$camera.Process.WaitForExit(25000)
    }
    [array]::Reverse($owned)
    foreach ($item in $owned) {
        if (!$item.Process.HasExited) {
            $item.Process.Kill()
            [void]$item.Process.WaitForExit(5000)
        }
    }
    foreach ($child in $cameraChildren) {
        if (!$child.HasExited) {
            $child.Kill()
            [void]$child.WaitForExit(5000)
        }
        if (!$child.HasExited) {
            throw 'An owned camera child did not stop.'
        }
    }
    # Venv children inherit these pipes; retire them before waiting for end-of-stream.
    foreach ($item in $owned) {
        foreach ($stream in @(@{Task=$item.Out;Suffix='out'}, @{Task=$item.Error;Suffix='err'})) {
            if ($stream.Task.Wait(5000)) {
                $stream.Task.Result | Set-Content -LiteralPath (Join-Path $run ($item.Name + '.' + $stream.Suffix))
            } else {
                Write-Warning "Output pipe did not close: $($item.Name).$($stream.Suffix)"
            }
        }
    }
    foreach ($file in $preserve) {
        $after = if (Test-Path -LiteralPath $file) {
            (Get-FileHash -LiteralPath $file).Hash
        } else {
            ''
        }
        if ($before[$file] -ne $after) {
            throw "Operator preservation check failed: $file"
        }
    }
    Write-Output 'Owned processes stopped; installed plugin and operator configuration preserved.'
}
