# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Test Mission Planner's persisted configuration migration using the built plugin.

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
$temporary = Join-Path ([IO.Path]::GetTempPath()) ('nomad-config-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temporary | Out-Null

function Load-Config {
    param([string]$Json)

    $path = Join-Path $temporary ([guid]::NewGuid().ToString('N') + '.json')
    [IO.File]::WriteAllText($path, $Json)
    $invokeArguments = New-Object object[] 1
    $invokeArguments[0] = [string]$path
    return $loadMethod.Invoke($null, $invokeArguments)
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
    $legacy = [ordered]@{
        DualLinkEnabled = $true
        RouterEnabled = $false
        RouterMode = 'Standalone'
        RouterLinks = @()
        RouterConsumers = @()
        IntegratedFlightMode = $true
        RadioMasterConnectionType = 'COM'
        LteMavlinkPort = 14560
        RouterBindAddress = '127.0.0.1'
        ManagementBindAddress = '127.0.0.1'
        RouterLocalPort = 14620
        ManagementPort = 14621
        CoreRuntimePort = 14631
        CoreClientCredential = 'local-client-credential'
        CoreApiKey = 'retired-enable-gate'
    }
    $legacyJson = ConvertTo-Json -InputObject $legacy -Depth 8
    $config = Load-Config $legacyJson
    if (-not $config.DualLinkEnabled) { throw 'DualLinkEnabled did not win over RouterEnabled.' }
    if ($config.RouterLocalPort -ne 14620 -or $config.ManagementPort -ne 14621) {
        throw 'Mission Planner client endpoints were not preserved.'
    }
    if ($config.CoreRuntimePort -ne 14631 -or $config.CoreClientCredential -ne 'local-client-credential') {
        throw 'Runtime client endpoint or independent credential was not preserved.'
    }

    $firstPath = Join-Path $temporary 'migrated-first.json'
    $secondPath = Join-Path $temporary 'migrated-second.json'
    $config.ExportToFile($firstPath)
    $migrated = [IO.File]::ReadAllText($firstPath)
    if ($migrated.Contains('local-client-credential') -or $migrated.Contains('retired-enable-gate')) {
        throw 'Portable profile export leaked a credential or retired gate.'
    }
    foreach ($field in @(
        'IntegratedFlightMode', 'RouterLinks', 'RouterConsumers', 'RouterEnabled', 'RouterMode',
        'RadioMasterConnectionType', 'LteMavlinkPort', 'RouterBindAddress', 'ManagementBindAddress', 'CoreApiKey'
    )) {
        if ($migrated.Contains('"' + $field + '"')) { throw "Retired field remained after migration: $field" }
    }

    $reloaded = Load-Config $migrated
    $reloaded.ExportToFile($secondPath)
    if ([IO.File]::ReadAllText($firstPath) -cne [IO.File]::ReadAllText($secondPath)) {
        throw 'Repeated Mission Planner config migration changed the serialized result.'
    }

    $embedded = [ordered]@{ RouterMode = 'Embedded' } | ConvertTo-Json
    Assert-ThrowsMessage $embedded "RouterMode 'Embedded' is unsupported"
    Assert-ThrowsMessage '{"RouterBindAddress":"0.0.0.0"}' 'RouterBindAddress must be 127.0.0.1'
    Assert-ThrowsMessage '{"ManagementBindAddress":"0.0.0.0"}' 'ManagementBindAddress must be 127.0.0.1'
    Assert-ThrowsMessage '{"Payloads":[{"Channel":9}]}' 'must be migrated to the runtime'
    Assert-ThrowsMessage '{"Actuators":[{"id":"existing"}]}' 'must be migrated to the runtime'
    Assert-ThrowsMessage '{"SerialJoystickEnabled":true}' 'direct USB HID mappings'
    Assert-ThrowsMessage '{"JoystickSw1UpAction":"DropToggleP1"}' 'Migrate legacy mappings'
    Assert-ThrowsMessage '{"JoystickCameraTiltEnabled":true,"JoystickCameraTiltDevice":"direct-usb"}' 'Enabled legacy relative-rate'
    Assert-ThrowsMessage '{"JoystickZedEnabled":true}' 'Enabled legacy relative-rate'
    $disabledAxis = Load-Config '{"JoystickCameraTiltEnabled":false,"JoystickCameraTiltDevice":"direct-usb","JoystickCameraTiltAxis":"X"}'
    if ($disabledAxis.JoystickPositionEnabled -or $disabledAxis.JoystickPositionDevice -ne 'direct-usb' -or $disabledAxis.JoystickPositionAxis -ne 'X') {
        throw 'Disabled legacy axis preferences did not migrate safely.'
    }
    $explicitAxis = Load-Config '{"JoystickCameraTiltEnabled":true,"JoystickPositionEnabled":false}'
    if ($explicitAxis.JoystickPositionEnabled) { throw 'Explicit position opt-out was overwritten.' }
    Assert-ThrowsMessage '{"JoystickSw1UpAction":"a:activate","JoystickSw2UpAction":"b:activate","JoystickButtonIndices":[0,1,0,3,4,5]}' 'unique physical HID'
    Assert-ThrowsMessage '{"JoystickSw1UpAction":"a:activate","JoystickButtonIndices":[6,1,2,3,4,5]}' 'disjoint'
    Assert-ThrowsMessage '{"JoystickTerminationButtonIndex":6.5}' 'must be an integer'
    Assert-ThrowsMessage '{"JoystickKillSwitchEnabled":"true"}' 'enabled flag must be boolean'
    Assert-ThrowsMessage '{"JoystickButtonIndices":[0,1,2.5,3,4,5]}' 'must be integers'
    Assert-ThrowsMessage '{"JoystickTerminationButtonIndex":128}' 'between 0 and 127'
    $loadPaths = $configType.GetMethod('LoadFromPaths', [Reflection.BindingFlags]'Static,NonPublic')
    $oversizedPrimary = Join-Path $temporary 'oversized-primary.json'
    $validBackup = Join-Path $temporary 'valid-backup.json'
    [IO.File]::WriteAllText($validBackup, '{"CoreRuntimePort":14631}')
    foreach ($oversizedJson in @(
        '{"JoystickTerminationButtonIndex":4294967296}',
        '{"JoystickButtonIndices":[0,1,4294967296,3,4,5]}',
        '{"JoystickTerminationButtonIndex":18446744073709551616}'
    )) {
        [IO.File]::WriteAllText($oversizedPrimary, $oversizedJson)
        $rejected = $false
        $pathArguments = New-Object object[] 2
        $pathArguments[0] = [string]$oversizedPrimary
        $pathArguments[1] = [string]$validBackup
        try { $loadPaths.Invoke($null, $pathArguments) | Out-Null }
        catch {
            if (-not $_.Exception.ToString().Contains('between 0 and 127')) { throw }
            $rejected = $true
        }
        if (-not $rejected) { throw 'Oversized primary HID integer silently fell back to valid backup/defaults.' }
        if ([IO.File]::ReadAllText($oversizedPrimary) -cne $oversizedJson) { throw 'Invalid primary was overwritten.' }
    }
    $termDefaults = Load-Config '{}'
    if ($termDefaults.JoystickTerminationButtonIndex -ne 6) { throw 'Missing termination index did not retain reviewed default6.' }
    $termRemap = Load-Config '{"JoystickTerminationButtonIndex":10,"JoystickSw1UpAction":"a:activate"}'
    if ($termRemap.JoystickTerminationButtonIndex -ne 10) { throw 'Explicit valid termination index was not preserved.' }
    $inactiveAlias = Load-Config '{"JoystickSw1UpAction":"a:activate","JoystickButtonIndices":[0,0,0,0,0,0]}'
    if ($inactiveAlias.JoystickButtonIndices[1] -ne 0) { throw 'Inactive index alias was silently remapped.' }
    $empty = Load-Config '{"Payloads":[],"Actuators":[],"SerialJoystickEnabled":false}'
    $emptyPath = Join-Path $temporary 'empty-retired-export.json'
    $empty.ExportToFile($emptyPath)
    $emptyJson = [IO.File]::ReadAllText($emptyPath)
    if ($emptyJson.Contains('"Payloads"') -or $emptyJson.Contains('"Actuators"') -or $emptyJson.Contains('"SerialJoystick')) {
        throw 'Empty/disabled retired ownership or bridge fields survived export.'
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
    Write-Host 'Mission Planner config migration passed: stable cleanup, client ownership, and fail-fast legacy rejection.'
} finally {
    [AppDomain]::CurrentDomain.remove_AssemblyResolve($dependencyResolver)
    Remove-Item -LiteralPath $temporary -Recurse -Force
}
