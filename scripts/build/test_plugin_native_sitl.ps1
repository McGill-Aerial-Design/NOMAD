# SPDX-License-Identifier: Apache-2.0
# Explicit manual acceptance: starts only an owned simulator and Mission Planner.
param(
    [Parameter(Mandatory = $true)][ValidatePattern('^[a-fA-F0-9]{40}$')][string]$ExpectedSource,
    [Parameter(Mandatory = $true)][ValidatePattern('^[a-fA-F0-9]{64}$')][string]$ExpectedPluginSha256,
    [ValidateSet("Copter", "Plane")][string]$Vehicle = "Copter",
    [ValidateRange(1, 100)][int]$Instance = 55
)
$ErrorActionPreference = 'Stop'
function Assert-NativeSitlProfileSafe($config) {
    foreach ($name in @('RouterClientEnabled', 'JoystickGimbalEnabled', 'JoystickPositionEnabled',
        'GimbalArrowKeysEnabled', 'DebugMode')) {
        if ($config.$name) {
            throw "Disable saved profile field $name before simulator startup."
        }
    }
    foreach ($name in @('JoystickSw1UpAction', 'JoystickSw1DownAction', 'JoystickSw2UpAction',
        'JoystickSw2DownAction', 'JoystickSw3UpAction', 'JoystickSw3DownAction')) {
        if (![string]::IsNullOrWhiteSpace($config.$name) -and $config.$name -cne 'None') {
            throw "Disable saved profile field $name before simulator startup."
        }
    }
    if ($config.AutoStartHudVideo -and $null -ne $config.SelectedVideoInput) {
        throw 'Disable configured automatic HUD video before simulator startup.'
    }
}

$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$hostRoot = 'C:\Program Files (x86)\Mission Planner'
$cache = Join-Path $env:USERPROFILE 'Documents/Mission Planner/sitl'
if (Get-Process MissionPlanner -ErrorAction SilentlyContinue) {
    throw 'An existing Mission Planner process is running.'
}
$fixtures = Join-Path $root 'mission_planner/tests/native_sitl'
$build = Join-Path $root 'build/tests/native-sitl'
New-Item -ItemType Directory -Path $build -Force | Out-Null
$hostVersion = [Diagnostics.FileVersionInfo]::GetVersionInfo((Join-Path $hostRoot 'MissionPlanner.exe')).FileVersion
$reviewedVersion = (Get-Content (Join-Path $root 'scripts/release/mission-planner-target.json') -Raw | ConvertFrom-Json).version
if ($hostVersion -ne $reviewedVersion) {
    throw 'Installed Mission Planner version mismatch'
}
$plugin = Join-Path $hostRoot 'plugins/NOMADPlugin.dll'
if ((Get-FileHash -LiteralPath $plugin).Hash -ne $ExpectedPluginSha256) {
    throw 'Installed DLL hash mismatch'
}
$binary = if ($Vehicle -eq 'Copter') { 'ArduCopter.exe' } else { 'ArduPlane.exe' }
$model = if ($Vehicle -eq 'Copter') { 'quad' } else { 'plane' }
$simulatorHash = if ($Vehicle -eq 'Copter') {
    'C5D95E95684D3AC0A6FA7315DD23D005D1BFCA81D1CB156EAC113D3C49822D75'
} else {
    '1D7E98968226D0A5C33CA845594B4C153D0B89DDBF48BF8E5EE3C1F350621D27'
}
if ((Get-FileHash -LiteralPath (Join-Path $cache $binary)).Hash -ne $simulatorHash) {
    throw 'Simulator hash mismatch'
}
$attempt = Join-Path $build ('attempt-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $attempt | Out-Null
Copy-Item -LiteralPath (Join-Path $cache $binary) -Destination $attempt
Get-ChildItem -LiteralPath $cache -Filter 'cyg*.dll' | Copy-Item -Destination $attempt
$defaults = 'outputs.parm'
$defaultsHash = $null
if ($Vehicle -eq 'Copter') {
    $defaultPath = Join-Path $cache 'default_params/copter.parm'
    $defaultsHash = (Get-FileHash -LiteralPath $defaultPath).Hash
    if ($defaultsHash -ne '5E01345B45D1C6190B28BECE5638BBDD4CF1CCE35E05BBBF480AB24D2B51AA0E') {
        throw 'Cached Copter defaults hash mismatch'
    }
    Copy-Item -LiteralPath $defaultPath -Destination $attempt
    $defaults = 'copter.parm,outputs.parm'
}
Copy-Item -LiteralPath (Join-Path $fixtures 'payloads.json') -Destination $attempt
Copy-Item -LiteralPath (Join-Path $fixtures 'outputs.parm') -Destination $attempt
Copy-Item -LiteralPath (Join-Path $fixtures 'native_sitl.py') -Destination $attempt
$preserved = @(
    (Join-Path $env:USERPROFILE 'Documents/Mission Planner/config.xml'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json.bak'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_config.json'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_config.json.bak'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/NOMAD/geofence_config.json')
)
$before = @{}
foreach ($path in $preserved) {
    $before[$path] = if (Test-Path -LiteralPath $path) { (Get-FileHash -LiteralPath $path).Hash } else { '' }
}
# Load only saved data from the exact installed assembly, before either child exists.
$resolver = [ResolveEventHandler] {
    param($sender, $eventArgs)
    if (([Reflection.AssemblyName]$eventArgs.Name).Name -ne 'Newtonsoft.Json') { return $null }
    return [Reflection.Assembly]::LoadFrom((Join-Path $hostRoot 'Newtonsoft.Json.dll'))
}
[AppDomain]::CurrentDomain.add_AssemblyResolve($resolver)
try {
    $assembly = [Reflection.Assembly]::LoadFile($plugin)
    $profilesType = $assembly.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    try {
        $profiles = $profilesType.GetMethod('Load').Invoke($null, @())
    } catch {
        throw 'Installed profiles could not be validated for simulator startup.'
    }
    Assert-NativeSitlProfileSafe $profiles.Selected.Config
} finally {
    [AppDomain]::CurrentDomain.remove_AssemblyResolve($resolver)
}

function Start-Owned($executable, $arguments, $directory, $variables) {
    $info = New-Object System.Diagnostics.ProcessStartInfo
    $info.FileName = $executable
    $info.Arguments = $arguments
    $info.WorkingDirectory = $directory
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    $info.WindowStyle = [System.Diagnostics.ProcessWindowStyle]::Hidden
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    foreach ($key in $variables.Keys) {
        $info.EnvironmentVariables[$key] = $variables[$key]
    }
    $process = [System.Diagnostics.Process]::Start($info)
    $stdout = $process.StandardOutput.ReadToEndAsync()
    $stderr = $process.StandardError.ReadToEndAsync()
    return @{ Process = $process; Out = $stdout; Error = $stderr }
}
$sim = $null
$mp = $null
$result = @{
    passed = $false
    simulator_sha256 = $simulatorHash
    simulator_banner = 'unobserved'
    vehicle = $Vehicle
    cached_defaults_sha256 = $defaultsHash
    mission_planner_version = $hostVersion
    plugin_sha256 = $ExpectedPluginSha256
}
try {
    $banner = [regex]::Match([Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes((Join-Path $attempt $binary))),
        'Ardu(?:Copter|Plane) V[0-9.]+(?:-dev)? \([a-f0-9]+\)')
    if ($banner.Success) { $result.simulator_banner = $banner.Value }
    $arguments = '-M' + $model + ' -O45,-73,100,0 -s1 --serial0 tcp:0 -I' + $Instance + ' --defaults ' + $defaults
    $sim = Start-Owned (Join-Path $attempt $binary) $arguments $attempt @{}
    $deadline = (Get-Date).AddSeconds(15)
    $port = $null
    while ((Get-Date) -lt $deadline -and !$sim.Process.HasExited) {
        $listeners = @(Get-NetTCPConnection -OwningProcess $sim.Process.Id -State Listen -ErrorAction SilentlyContinue)
        if ($listeners.LocalPort -contains (5760 + 10 * $Instance)) {
            $port = 5760 + 10 * $Instance
            break
        }
        Start-Sleep -Milliseconds 200
    }
    if (!$port) {
        throw 'Simulator listener unavailable'
    }
    $result.simulator_started = $true
    $disabled = (Get-ChildItem -LiteralPath (Join-Path $hostRoot 'plugins') -File |
        Where-Object { $_.Extension -in '.dll','.cs' -and $_.Name -ine 'NOMADPlugin.dll' } |
        ForEach-Object { [Uri]::EscapeDataString($_.Name.ToLowerInvariant()) }) -join ';'
    $xml = New-Object System.Xml.XmlDocument
    $declaration = $xml.CreateXmlDeclaration('1.0','utf-8',$null)
    [void]$xml.AppendChild($declaration)
    $config = $xml.CreateElement('Config')
    [void]$xml.AppendChild($config)
    $settings = @{
        AutoConnect = '[]'
        DisabledPlugins = $disabled
        CHK_resetapmonconnect = 'False'
        CHK_rtsresetesp32 = 'False'
        CONNECT_TIMEOUT_SECONDS = '10'
        mapCache = 'CacheOnly'
        logdirectory = (Join-Path $attempt 'native-logs')
        speechenable = 'False'
        beta_updates = 'False'
        update_check = (Get-Date).ToShortDateString()
    }
    foreach ($name in $settings.Keys) {
        $entry = $xml.CreateElement($name)
        $entry.InnerText = $settings[$name]
        [void]$config.AppendChild($entry)
    }
    $configPath = Join-Path $attempt 'mp.config.xml'
    $xml.Save($configPath)
    $script = Join-Path $attempt 'native_sitl.py'
    $variables = @{ NOMAD_SITL_PORT = [string]$port; NOMAD_SITL_SOURCE = $ExpectedSource }
    foreach ($path in $preserved) {
        $current = if (Test-Path -LiteralPath $path) { (Get-FileHash -LiteralPath $path).Hash } else { '' }
        if ($current -ne $before[$path]) {
            throw 'A preserved profile/configuration candidate changed before host launch.'
        }
    }
    if ((Get-FileHash -LiteralPath $plugin).Hash -ne $ExpectedPluginSha256) {
        throw 'Installed DLL changed before host launch.'
    }
    $mp = Start-Owned (Join-Path $hostRoot 'MissionPlanner.exe') ('-config "' + $configPath + '" -script "' + $script + '"') $hostRoot $variables
    if (!$mp.Process.WaitForExit(110000)) {
        [void]$mp.Process.CloseMainWindow()
        if (!$mp.Process.WaitForExit(10000)) {
            throw 'Owned Mission Planner did not close normally'
        }
    }
    $summaryPath = Join-Path $attempt 'summary.json'
    if (Test-Path -LiteralPath $summaryPath) {
        $result.host = Get-Content -LiteralPath $summaryPath -Raw | ConvertFrom-Json
        $result.passed = $result.host.passed
    } else {
        $result.failure_code = 'host_summary_absent'
    }
} catch {
    $result.runner_error_type = $_.Exception.GetType().Name
    $result.runner_error_line = $_.InvocationInfo.ScriptLineNumber
    if ($sim -and $sim.Process.HasExited) {
        $result.simulator_exit_code = $sim.Process.ExitCode
    }
    $result.failure_code = 'runner_failed'
} finally {
    if ($mp -and !$mp.Process.HasExited) {
        [void]$mp.Process.CloseMainWindow()
        [void]$mp.Process.WaitForExit(10000)
    }
    $result.owned_host_exited = $mp -and $mp.Process.HasExited
    $result.host_exit_code = if ($result.owned_host_exited) { $mp.Process.ExitCode } else { $null }
    if ($sim -and !$sim.Process.HasExited) {
        $sim.Process.Kill()
        $sim.Process.WaitForExit()
    }
    $result.owned_simulator_exited = $sim -and $sim.Process.HasExited
    $unchanged = $true
    foreach ($path in $preserved) {
        $after = if (Test-Path -LiteralPath $path) { (Get-FileHash -LiteralPath $path).Hash } else { '' }
        if ($after -ne $before[$path]) {
            $unchanged = $false
        }
    }
    $result.preserved_files_unchanged = $unchanged
    $result.preserved_file_set = @('mp-config', 'aircraft-profiles', 'aircraft-profiles-backup',
        'legacy-profile', 'legacy-profile-backup', 'advisory-config')
    $result.passed = $result.passed -and $unchanged -and $result.owned_host_exited -and $result.host_exit_code -eq 0 -and $result.owned_simulator_exited
    $result | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath (Join-Path $attempt 'result.json') -Encoding UTF8
    $result | ConvertTo-Json -Depth 10
}
if (!$result.passed) {
    exit 1
}
