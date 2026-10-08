# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Test Mission Planner's current configuration validation using the built plugin.

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$pluginPath = Join-Path $repoRoot 'mission_planner\src\bin\Release\NOMADPlugin.dll'
$missionPlannerDir = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$newtonsoftPath = Join-Path $missionPlannerDir 'Newtonsoft.Json.dll'

if (-not (Test-Path -LiteralPath $pluginPath -PathType Leaf)) {
    throw 'Build the Mission Planner plugin first with `pixi run build-plugin-only`.'
}
if (-not (Test-Path -LiteralPath $newtonsoftPath -PathType Leaf)) {
    throw "Mission Planner Newtonsoft.Json dependency not found: $newtonsoftPath"
}

$dependencyResolver = [ResolveEventHandler] {
    param($sender, $eventArgs)
    $name = ([Reflection.AssemblyName]$eventArgs.Name).Name
    $dependency = Join-Path $missionPlannerDir ($name + '.dll')
    if (Test-Path -LiteralPath $dependency -PathType Leaf) { return [Reflection.Assembly]::LoadFrom($dependency) }
    return $null
}
[AppDomain]::CurrentDomain.add_AssemblyResolve($dependencyResolver)
[Reflection.Assembly]::LoadFrom($newtonsoftPath) | Out-Null
[Reflection.Assembly]::LoadFrom((Join-Path $missionPlannerDir 'MissionPlanner.exe')) | Out-Null
$plugin = [Reflection.Assembly]::LoadFrom($pluginPath)
$configType = $plugin.GetType('NOMAD.MissionPlanner.NOMADConfig', $true)
    $loadMethod = $configType.GetMethod('LoadFromFile')
    $geofenceType = $plugin.GetType('NOMAD.MissionPlanner.GeofenceConfig', $true)
    $geofenceLoadMethod = $geofenceType.GetMethod('LoadFromJson', [Reflection.BindingFlags]'Static,NonPublic')
$temporary = Join-Path ([IO.Path]::GetTempPath()) ('nomad-config-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temporary | Out-Null

function Load-Config {
    param([string]$Json)

    $path = Join-Path $temporary ([guid]::NewGuid().ToString('N') + '.json')
    [IO.File]::WriteAllText($path, $Json)
    $invokeArguments = New-Object object[] 1
    $invokeArguments[0] = [string]$path
    try { return $loadMethod.Invoke($null, $invokeArguments) } finally {
        if ([IO.File]::ReadAllText($path) -cne $Json) { throw "Loading rewrote saved operator data" }
    }
}

function Assert-ThrowsMessage {
    param([string]$Json, [string]$Expected)

    try {
        Load-Config $Json | Out-Null
    } catch {
        if ($_.Exception.ToString().Contains($Expected)) {
            return
        }
        throw
    }
    throw "Expected configuration rejection containing: $Expected"
}

try {
    $config = Load-Config '{"CoreRuntimePort":14631,"RouterLocalPort":14620,"ManagementPort":14621,"CoreClientCredential":"test-only-credential"}'
    if ($config.CoreRuntimePort -ne 14631) { throw 'Current runtime endpoint changed.' }
    $export = Join-Path $temporary 'export.json'
    $config.ExportToFile($export)
    if ([IO.File]::ReadAllText($export).Contains('test-only-credential')) { throw 'Export leaked client credential.' }
    foreach ($field in @('Payloads','Actuators','RouterMode','RouterEnabled','JoystickCameraTiltEnabled','JoystickZedEnabled','SerialJoystickEnabled','SlamCameraFovDeg')) {
        Assert-ThrowsMessage ('{"' + $field + '":null}') $field
    }
    Assert-ThrowsMessage '{"CoreRuntimePort":0}' 'ports'
    Assert-ThrowsMessage '{"JoystickButtonIndices":[0,1,2,3,4,128]}' 'between 0 and 127'
    Assert-ThrowsMessage '{"JoystickButtonIndices":[0,1,2,3,4,5.5]}' 'integers'
    Assert-ThrowsMessage '{"JoystickTerminationButtonIndex":0,"JoystickKillSwitchEnabled":true,"JoystickSw1UpAction":"a:activate"}' 'disjoint'
    Assert-ThrowsMessage '{"CoreRuntimePort":14611,"CoreRuntimePort":14612}' 'already exists'
    $boundary = $geofenceLoadMethod.Invoke($null, [object[]]@('{"AdvisoryAltitudeDisplayThresholdMeters":100}'))
    if ($boundary.AdvisoryAltitudeDisplayThresholdMeters -ne 100) { throw 'Current advisory threshold changed.' }
    foreach ($json in @('{"MaxAltitudeAglMeters":100}', '{"HardBoundaryAction":"Return"}', '{"AdvisoryAltitudeDisplayThresholdMeters":-1}')) {
        $rejected = $false
        try { $geofenceLoadMethod.Invoke($null, [object[]]@($json)) | Out-Null } catch { $rejected = $true }
        if (-not $rejected) { throw 'Unsupported or invalid boundary data was accepted.' }
    }
    $loadPaths = $configType.GetMethod('LoadFromPaths', [Reflection.BindingFlags]'Static,NonPublic')
    $primary = Join-Path $temporary 'primary.json'
    $backup = Join-Path $temporary 'backup.json'
    $backupText = '{"CoreRuntimePort":14632}'
    [IO.File]::WriteAllText($backup, $backupText)
    foreach ($invalid in @('{"RouterMode":"Embedded"}', '{"JoystickTerminationButtonIndex":999999999999999999999}', '', '{')) {
        [IO.File]::WriteAllText($primary, $invalid)
        $rejected = $false
        try { $loadPaths.Invoke($null, [object[]]@($primary, $backup)) | Out-Null } catch { $rejected = $true }
        if (-not $rejected) { throw 'Invalid primary fell back to a backup/default configuration.' }
        if ([IO.File]::ReadAllText($primary) -cne $invalid -or [IO.File]::ReadAllText($backup) -cne $backupText) {
            throw 'Invalid primary load modified operator files.'
        }
    }
    # Exercise real settings controls without opening a window or saving host configuration.
    $formType = $plugin.GetType('NOMAD.MissionPlanner.NOMADSettingsForm', $true)
    $config.JoystickSwitchDevice = 'selected-unplugged-usb-hid'
    $formArguments = New-Object object[] 1
    $formArguments[0] = $config
    $form = $formType.GetConstructor([type[]]@($configType)).Invoke($formArguments)
    try {
        $privateFlags = [Reflection.BindingFlags]'Instance,NonPublic'
        $device = $formType.GetField('_cmbSwitchDevice', $privateFlags).GetValue($form)
        if ($device.SelectedItem -ne 'selected-unplugged-usb-hid') { throw 'Unplugged selected HID name was discarded.' }
        $device.Items.Clear()
        $device.Items.Add('(none)') | Out-Null
        $setDevice = $formType.GetMethod('SetDeviceComboValue', [Reflection.BindingFlags]'Static,NonPublic')
        $setDevice.Invoke($null, [object[]]@($device, 'selected-unplugged-usb-hid')) | Out-Null
        if ($device.SelectedItem -ne 'selected-unplugged-usb-hid') { throw 'Device refresh discarded an explicit HID name.' }

        $actorType = $plugin.GetType('NOMAD.MissionPlanner.Connectivity.NomadActuator', $true)
        $actionType = $plugin.GetType('NOMAD.MissionPlanner.Connectivity.NomadActuatorAction', $true)
        $actor = [Activator]::CreateInstance($actorType)
        $action = [Activator]::CreateInstance($actionType)
        $actorType.GetProperty('Id').SetValue($actor, 'stable-id', $null)
        $actorType.GetProperty('Name').SetValue($actor, 'Original name', $null)
        $actionType.GetProperty('Operation').SetValue($action, 'activate', $null)
        $actionType.GetProperty('Control').SetValue($action, 'button', $null)
        $actionType.GetProperty('Label').SetValue($action, 'Original label', $null)
        $actions = [Array]::CreateInstance($actionType, 1)
        $actions.SetValue($action, 0)
        $actorType.GetProperty('Actions').SetValue($actor, $actions, $null)
        $actors = [Array]::CreateInstance($actorType, 1)
        $actors.SetValue($actor, 0)
        $applyArguments = New-Object object[] 1
        $applyArguments[0] = $actors
        $apply = $formType.GetMethod('ApplyActuatorActions', $privateFlags)
        $apply.Invoke($form, $applyArguments) | Out-Null
        $combo = $formType.GetField('_cmbSw1Up', $privateFlags).GetValue($form)
        $combo.Text = 'Original name / Original label [stable-id:activate]'
        $actorType.GetProperty('Name').SetValue($actor, 'Renamed device', $null)
        $actionType.GetProperty('Label').SetValue($action, 'Renamed action', $null)
        $apply.Invoke($form, $applyArguments) | Out-Null
        if ($combo.Text -ne 'Renamed device / Renamed action [stable-id:activate]') {
            throw 'Backend label refresh did not preserve the stable semantic binding.'
        }
        $positionAction = [Activator]::CreateInstance($actionType)
        $actionType.GetProperty('Operation').SetValue($positionAction, 'position', $null)
        $actionType.GetProperty('Control').SetValue($positionAction, 'position', $null)
        $actionType.GetProperty('Label').SetValue($positionAction, 'Set position', $null)
        $actionType.GetProperty('ContinuousAxisAllowed').SetValue($positionAction, $false, $null)
        $actionType.GetProperty('ContinuousAxisBlockedReason').SetValue($positionAction, 'Use discrete UI confirmations.', $null)
        $eligibilityStore = $plugin.GetType('NOMAD.MissionPlanner.OutputController', $true).GetField('ContinuousAxisActions', [Reflection.BindingFlags]'Static,NonPublic').GetValue($null)
        $eligibilityStore['stable-id'] = $positionAction
        $positionId = $formType.GetField('_txtJoyPositionActuatorId', $privateFlags).GetValue($form)
        $positionId.Text = 'stable-id'
        $positionEnable = $formType.GetField('_chkJoyPositionEnabled', $privateFlags).GetValue($form)
        $positionReason = $formType.GetField('_lblJoyPositionEligibility', $privateFlags).GetValue($form)
        if ($positionEnable.Enabled -or $positionEnable.Checked -or $positionReason.Text -ne 'Use discrete UI confirmations.') {
            throw 'Unsupported axis binding was not disabled with the backend reason.'
        }
        $actionType.GetProperty('ContinuousAxisAllowed').SetValue($positionAction, $true, $null)
        $formType.GetMethod('UpdatePositionEligibility', $privateFlags).Invoke($form, @()) | Out-Null
        if (-not $positionEnable.Enabled) { throw 'Backend allowed axis data did not enable the input option.' }
        $positionId.Text = 'unknown-id'
        if ($positionEnable.Enabled) { throw 'Unknown axis eligibility did not fail closed.' }
    } finally { $form.Dispose() }
    Write-Host 'Mission Planner current configuration and HID validation passed.'
} finally {
    [AppDomain]::CurrentDomain.remove_AssemblyResolve($dependencyResolver)
    Remove-Item -LiteralPath $temporary -Recurse -Force
}
