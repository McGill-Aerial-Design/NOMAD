# SPDX-License-Identifier: Apache-2.0
# Every case uses a fresh CLR so static profile/log paths cannot leak between cases.
param([ValidateSet('', 'default', 'custom', 'invalid')][string]$Case = '', [string]$OutputDirectory = '')
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
if (!$Case) {
    $attempt = Join-Path $root ('build/tests/storage/attempt-' + [Guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $attempt -Force | Out-Null
    $windowsPowerShell = Join-Path $env:WINDIR 'System32/WindowsPowerShell/v1.0/powershell.exe'
    foreach ($name in @('default', 'custom', 'invalid')) {
        & $windowsPowerShell -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -Case $name -OutputDirectory $attempt
        if ($LASTEXITCODE -ne 0) { throw "Native storage case failed: $name" }
    }
    Write-Output 'PASS native host default/custom/invalid NOMAD storage'
    return
}
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
[AppDomain]::CurrentDomain.add_AssemblyResolve([ResolveEventHandler] {
    param($sender, $request)
    $name = ([Reflection.AssemblyName]$request.Name).Name
    $path = Join-Path $hostRoot ($name + '.dll')
    if (Test-Path -LiteralPath $path) { return [Reflection.Assembly]::LoadFrom($path) }
    return $null
})
[Reflection.Assembly]::LoadFrom((Join-Path $hostRoot 'MissionPlanner.Utilities.dll')) | Out-Null
$custom = Join-Path $OutputDirectory 'custom'
[MissionPlanner.Utilities.Settings]::CustomUserDataDirectory = switch ($Case) {
    'default' { '' }
    'custom' { $custom }
    'invalid' { 'relative-storage-must-not-be-used' }
}
$plugin = [Reflection.Assembly]::LoadFrom((Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'))
$storage = $plugin.GetType('NOMAD.MissionPlanner.NOMADStorage', $true)
$flags = [Reflection.BindingFlags]'Static,NonPublic'
$directory = $storage.GetProperty('PluginsDirectory', $flags)
if ($Case -eq 'default') {
    $expected = Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'Mission Planner/plugins'
    if ($directory.GetValue($null) -cne $expected) { throw 'Default operator storage path changed.' }
    Write-Output 'PASS default path only; no operator profile or log opened'
    return
}
if ($Case -eq 'invalid') {
    $rejected = $false
    try { $directory.GetValue($null) | Out-Null } catch {
        if ($_.Exception.ToString().Contains('must be absolute')) { $rejected = $true } else { throw }
    }
    if (!$rejected) { throw 'Relative custom root was accepted or fell back.' }
    $profilesType = $plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    $rejected = $false
    try { [Activator]::CreateInstance($profilesType) | Out-Null } catch {
        if ($_.Exception.ToString().Contains('must be absolute')) { $rejected = $true } else { throw }
    }
    if (!$rejected) { throw 'Profiles accepted invalid custom storage.' }
    $log = $plugin.GetType('NOMAD.MissionPlanner.Log', $true)
    $log.GetMethod('Info').Invoke($null, [object[]]@('Rejected storage test')) | Out-Null
    if (!$log.GetField('_fileFailed', $flags).GetValue($null) -or $null -ne $log.GetField('_logPath', $flags).GetValue($null)) {
        throw 'Logger did not stop before selecting an operator fallback.'
    }
    Write-Output 'PASS invalid explicit root rejected before operator fallback'
    return
}
$nativeData = [IO.Path]::GetFullPath([MissionPlanner.Utilities.Settings]::GetUserDataDirectory())
if (!$nativeData.StartsWith($custom + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Native user data escaped the owned root.' }
$expected = Join-Path $nativeData 'plugins'
if ($directory.GetValue($null).TrimEnd('\') -cne $expected) { throw 'Native custom storage root was not used.' }
$configType = $plugin.GetType('NOMAD.MissionPlanner.NOMADConfig', $true)
$profilesType = $plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
$config = [Activator]::CreateInstance($configType)
$config.AircraftName = 'Owned first profile'
$profiles = $profilesType.GetConstructor([type[]]@($configType)).Invoke([object[]]@($config))
$profiles.Save()
$profiles.Selected.Config.AircraftName = 'Owned second profile'
$profiles.Save()
$file = Join-Path $expected 'nomad_aircraft_profiles.json'
if (!(Test-Path -LiteralPath $file) -or !(Test-Path -LiteralPath ($file + '.bak'))) { throw 'Custom primary/backup missing.' }
$loaded = $profilesType.GetMethod('Load').Invoke($null, $null)
if ($loaded.Selected.Config.AircraftName -cne 'Owned second profile') { throw 'Custom profile load did not round trip.' }
if (!(Get-Content -LiteralPath ($file + '.bak') -Raw).Contains('Owned first profile')) { throw 'Backup did not retain preceding profile.' }
$log = $plugin.GetType('NOMAD.MissionPlanner.Log', $true)
$log.GetMethod('Info').Invoke($null, [object[]]@('Owned native storage check')) | Out-Null
$logPath = Join-Path $expected 'NOMAD/nomad.log'
if (!(Test-Path -LiteralPath $logPath) -or !(Get-Content -LiteralPath $logPath -Raw).Contains('Owned native storage check')) {
    throw 'Logger escaped or failed to use native custom storage.'
}
Write-Output 'PASS actual custom profile save/load/backup and native logger'
