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
    # Keep every subject and cleanup diagnostic in this fixture, including failure paths.
    $fixtureLogType = $plugin.GetType('NOMAD.MissionPlanner.Log', $true)
    $fixtureLogFlags = [Reflection.BindingFlags]'Static,NonPublic'
    $fixtureLogPathField = $fixtureLogType.GetField('_logPath', $fixtureLogFlags)
    $fixtureLogFailedField = $fixtureLogType.GetField('_fileFailed', $fixtureLogFlags)
    if ($null -eq $fixtureLogPathField -or $null -eq $fixtureLogFailedField) {
        throw 'Configuration fixture logger fields are unavailable.'
    }
    if ($null -ne $fixtureLogPathField.GetValue($null) -or $fixtureLogFailedField.GetValue($null)) {
        throw 'Configuration fixture requires a fresh logger.'
    }
    $fixtureLogPath = Join-Path $temporary 'nomad.log'
    $fixtureLogPathField.SetValue($null, [string]$fixtureLogPath)
    if ($fixtureLogPathField.GetValue($null) -cne $fixtureLogPath) {
        throw 'Configuration fixture log path was not installed.'
    }

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
    # Exercise preset selection without constructing a view that reads the operator's saved outlines.
    $boundaryViewType = $plugin.GetType('NOMAD.MissionPlanner.NOMADBoundaryView', $true)
    $presetType = $plugin.GetType('NOMAD.MissionPlanner.BoundaryPreset', $true)
    $view = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($boundaryViewType)
    # Initialize only WinForms ownership on that object, never the NOMAD constructor that reads AppData.
    [Windows.Forms.UserControl].GetConstructor([type[]]@()).Invoke(
        $view, [Reflection.BindingFlags]::Default, $null, [object[]]@(), $null) | Out-Null
    $presets = [Activator]::CreateInstance([Collections.Generic.List``1].MakeGenericType($presetType))
    $older = [Activator]::CreateInstance($presetType)
    $newer = [Activator]::CreateInstance($presetType)
    $sameTime = [Activator]::CreateInstance($presetType)
    $older.Name = 'Repeated name'
    $newer.Name = 'Repeated name'
    $sameTime.Name = 'Equal timestamp'
    $older.CreatedAt = [datetime]'2026-01-01'
    $newer.CreatedAt = [datetime]'2026-01-02'
    $sameTime.CreatedAt = $newer.CreatedAt
    $presets.Add($older)
    $presets.Add($newer)
    $presets.Add($sameTime)
    $privateFlags = [Reflection.BindingFlags]'Instance,NonPublic'
    $boundaryViewType.GetField('_presets', $privateFlags).SetValue($view, $presets)
    $presetCombo = New-Object Windows.Forms.ComboBox
    $boundaryViewType.GetField('_cmbPresets', $privateFlags).SetValue($view, $presetCombo)
    $refresh = $boundaryViewType.GetMethod('RefreshPresetCombo', $privateFlags)
    try {
        $refresh.Invoke($view, @()) | Out-Null
        if ($presetCombo.Items.Count -ne 4 -or $presetCombo.SelectedIndex -ne 0 -or
            $presetCombo.GetItemText($presetCombo.SelectedItem) -ne '-- Select Preset --') {
            throw 'Preset refresh did not reset selection to the placeholder.'
        }
        $expected = @($newer, $sameTime, $older)
        for ($index = 0; $index -lt $expected.Count; $index++) {
            $presetCombo.SelectedIndex = $index + 1
            if (-not [object]::ReferenceEquals($presetCombo.SelectedItem, $expected[$index]) -or
                $presetCombo.GetItemText($presetCombo.SelectedItem) -ne $expected[$index].Name) {
                throw 'Displayed preset selection lost its identity after sorting or duplicate names.'
            }
        }
        # Changing storage order must not change an already displayed selection.
        $presetCombo.SelectedIndex = 1
        $presets.Reverse()
        if (-not [object]::ReferenceEquals($presetCombo.SelectedItem, $newer)) {
            throw 'Preset selection changed when the underlying list order changed.'
        }
        foreach ($selection in @(-1, 0)) {
            $presetCombo.SelectedIndex = $selection
            foreach ($handler in @('LoadSelectedPreset', 'DeleteSelectedPreset')) {
                $boundaryViewType.GetMethod($handler, $privateFlags).Invoke(
                    $view, [object[]]@($null, [EventArgs]::Empty)) | Out-Null
            }
        }
        $presets.Clear()
        $refresh.Invoke($view, @()) | Out-Null
        if ($presetCombo.Items.Count -ne 1 -or $presetCombo.SelectedIndex -ne 0) {
            throw 'An empty preset list did not retain the placeholder.'
        }
    } finally { $presetCombo.Dispose() }
    # Exercise the real host's nonconsuming packet cache without constructing MainV2 or a vehicle connection.
    $nativeMavType = [MissionPlanner.MAVState]
    $nativeMav = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($nativeMavType)
    foreach ($name in @('packets', 'packetsLast')) {
        $property = $nativeMavType.GetProperty($name, $privateFlags)
        $property.SetValue($nativeMav, [Activator]::CreateInstance($property.PropertyType), $null)
    }
    $nativeMavType.GetField('packetslock', $privateFlags).SetValue($nativeMav, [object]::new())
    $payload = [Activator]::CreateInstance([MAVLink+mavlink_global_position_int_t])
    $payload.lat = 15000000
    $payload.lon = 15000000
    $parser = [MAVLink+MavlinkParse]::new()
    $bytes = $parser.GenerateMAVLinkPacket20([MAVLink+MAVLINK_MSG_ID]::GLOBAL_POSITION_INT, $payload, $false, 1, 1, 1)
    $packet = [MAVLink+MAVLinkMessage]::new($bytes, [DateTime]::UtcNow)
    $nativeMavType.GetMethod('addPacket').Invoke($nativeMav, [object[]]@($packet)) | Out-Null
    $cached = $nativeMavType.GetMethod('getPacketLast').Invoke($nativeMav, [object[]]@([uint32]33))
    $clockType = $plugin.GetType('NOMAD.MissionPlanner.RouterClock', $true)
    $clock = $clockType.GetConstructors($privateFlags)[0].Invoke([object[]]@($null, $null))
    $positionType = $plugin.GetType('NOMAD.MissionPlanner.AdvisoryPosition', $true)
    $positionReader = $positionType.GetConstructors($privateFlags)[0].Invoke([object[]]@($clock))
    $packetSlots = [Array]::CreateInstance([MAVLink+MAVLinkMessage], 4)
    $packetSlots.SetValue($cached, 0)
    $point = $positionType.GetMethod('Read', $privateFlags).Invoke(
        $positionReader, [object[]]@($packetSlots, [byte]1, [byte]1))
    $queued = $nativeMavType.GetMethod('getPacket').Invoke($nativeMav, [object[]]@([uint32]33))
    if ($point.Lat -ne 1.5 -or $point.Lon -ne 1.5 -or -not [object]::ReferenceEquals($queued, $packet)) {
        throw 'Received-position decoding changed coordinates or consumed the native host packet queue.'
    }
    # Real MP field and real float->Locationwp->int conversion, without a MainV2 connection.
    $fenceField = $nativeMavType.GetField('fencepoints')
    $fenceCache = [Activator]::CreateInstance($fenceField.FieldType)
    $fenceField.SetValue($nativeMav, $fenceCache)
    $locationType = [MissionPlanner.Utilities.Locationwp]
    $toLocation = $locationType.GetMethods() | Where-Object {
        $_.Name -eq 'op_Implicit' -and $_.ReturnType -eq $locationType -and
        $_.GetParameters()[0].ParameterType -eq [MAVLink+mavlink_mission_item_t]
    }
    $toInteger = $locationType.GetMethods() | Where-Object {
        $_.Name -eq 'op_Implicit' -and $_.ReturnType -eq [MAVLink+mavlink_mission_item_int_t]
    }
    $expectedFence = @(@(0,0), @(0,2), @(1,1), @(2,2), @(2,0))
    foreach ($index in @(4,2,0,3,1)) {
        $floating = [Activator]::CreateInstance([MAVLink+mavlink_mission_item_t])
        $floating.seq = $index; $floating.command = 5001; $floating.param1 = 5
        $floating.frame = 0; $floating.mission_type = 1
        $floating.x = $expectedFence[$index][0]; $floating.y = $expectedFence[$index][1]
        $location = $toLocation.Invoke($null, [object[]]@($floating))
        $integer = $toInteger.Invoke($null, [object[]]@($location))
        if ($integer.mission_type -ne 0 -or $integer.x -ne $floating.x * 1e7 -or
            $integer.y -ne $floating.y * 1e7 -or -not $fenceCache.TryAdd($index, $integer)) {
            throw 'Native float cache conversion/payload differs from the reviewed MP target.'
        }
    }
    $pairType = [Collections.Generic.KeyValuePair``2].MakeGenericType([int], [MAVLink+mavlink_mission_item_int_t])
    $nativeSnapshot = [Array]::CreateInstance($pairType, $fenceCache.Count)
    $index = 0
    foreach ($entry in $fenceCache.ToArray()) { $nativeSnapshot.SetValue($entry, $index++ ) }
    $fenceBefore = [Newtonsoft.Json.JsonConvert]::SerializeObject($nativeSnapshot)
    $decoder = $plugin.GetType('NOMAD.MissionPlanner.NativeFenceImport', $true)
    $decodeArgs = New-Object object[] 3
    $decodeArgs[0] = $nativeSnapshot
    if (-not $decoder.GetMethod('TryDecode', [Reflection.BindingFlags]'Static,NonPublic').Invoke($null, $decodeArgs)) {
        throw ('Real native float cache was rejected: ' + $decodeArgs[2])
    }
    $fencePoints = $decodeArgs[1]
    if ($fencePoints.Count -ne 5) { throw 'Native polygon count changed.' }
    for ($index = 0; $index -lt 5; $index++) {
        if ($fencePoints[$index].Lat -ne $expectedFence[$index][0] -or
            $fencePoints[$index].Lon -ne $expectedFence[$index][1]) { throw 'Native polygon order/E7 coordinates changed.' }
    }
    $fractionalSnapshot = [Array]::CreateInstance($pairType, $nativeSnapshot.Length)
    for ($index = 0; $index -lt $nativeSnapshot.Length; $index++) {
        $entry = $nativeSnapshot.GetValue($index)
        $item = $entry.Value; $item.x += 12345678; $item.y -= 23456789
        $fractionalSnapshot.SetValue([Activator]::CreateInstance($pairType, [object[]]@([int]$entry.Key, $item)), $index)
    }
    $fractionalArgs = New-Object object[] 3
    $fractionalArgs[0] = $fractionalSnapshot
    if (-not $decoder.GetMethod('TryDecode', [Reflection.BindingFlags]'Static,NonPublic').Invoke($null, $fractionalArgs) -or
        $fractionalArgs[1][0].Lat -ne 1.2345678 -or $fractionalArgs[1][0].Lon -ne -2.3456789 -or
        $fractionalArgs[1][4].Lat -ne 3.2345678 -or $fractionalArgs[1][4].Lon -ne -2.3456789) {
        throw 'Fractional signed typed-cache E7 coordinates changed.'
    }
    if (-not [object]::ReferenceEquals($fenceField.GetValue($nativeMav), $fenceCache) -or
        [Newtonsoft.Json.JsonConvert]::SerializeObject($fenceCache.ToArray()) -cne $fenceBefore) {
        throw 'Native import changed field identity, keys or payloads.'
    }
    # Persist the staged import only to an explicit temporary path. No operator AppData is touched.
    $applyFence = $boundaryViewType.GetMethod('ApplyOutlinePoints', $privateFlags)
    $originalJson = '{"SoftBoundary":{"Name":"Saved inner","Vertices":[{"Lat":1,"Lon":1,"Alt":11},{"Lat":1,"Lon":2,"Alt":12},{"Lat":2,"Lon":1,"Alt":13}]},"HardBoundary":{"Name":"Saved outer","Vertices":[{"Lat":3,"Lon":3,"Alt":21},{"Lat":3,"Lon":4,"Alt":22},{"Lat":4,"Lon":3,"Alt":23}]},"SoftBoundaryFromHard":true,"SoftBoundaryInsetMeters":12,"AdvisoryPreviewEnabled":true,"AdvisoryAudioAlertsEnabled":false,"AdvisoryAltitudeDisplayThresholdMeters":100}'
    foreach ($fault in @('cancel','derived-inner','blocked-temp','blocked-backup','locked-primary','success','new-file',
            'inner-success','outer-undervived')) {
        $importConfig = $geofenceLoadMethod.Invoke($null, [object[]]@($originalJson))
        if ($fault -in @('inner-success','outer-undervived')) { $importConfig.SoftBoundaryFromHard = $false }
        $boundaryViewType.GetField('_missionConfig', $privateFlags).SetValue($view, $importConfig)
        $innerBefore = $importConfig.SoftBoundary.Vertices; $outerBefore = $importConfig.HardBoundary.Vertices
        $geometryBefore = [Newtonsoft.Json.JsonConvert]::SerializeObject($importConfig)
        $initialDisk = $geometryBefore
        $importPath = Join-Path $temporary ($fault + '.json')
        if ($fault -ne 'new-file') { [IO.File]::WriteAllText($importPath, $initialDisk) }
        if ($fault -eq 'blocked-temp') { New-Item -ItemType Directory -Path ($importPath + '.tmp') | Out-Null }
        if ($fault -eq 'blocked-backup') { New-Item -ItemType Directory -Path ($importPath + '.bak') | Out-Null }
        $locked = $null
        if ($fault -eq 'locked-primary') { $locked = [IO.File]::Open($importPath, 'Open', 'Read', 'Read') }
        $choice = if ($fault -eq 'cancel') { [Windows.Forms.DialogResult]::Cancel }
            elseif ($fault -in @('derived-inner','inner-success')) { [Windows.Forms.DialogResult]::Yes }
            else { [Windows.Forms.DialogResult]::No }
        $applyArgs = [object[]]@($fencePoints, $choice, $null, $importPath, $false)
        $applyArgs[3] = [string]$importPath
        try { $saved = $applyFence.Invoke($view, $applyArgs) } finally { if ($locked) { $locked.Dispose() } }
        if ($fault -in @('success','new-file','inner-success','outer-undervived')) {
            $target = if ($fault -eq 'inner-success') { $importConfig.SoftBoundary } else { $importConfig.HardBoundary }
            if (-not $saved -or $target.Vertices.Count -ne 5 -or $importConfig.AdvisoryAudioAlertsEnabled -or
                -not $importConfig.AdvisoryPreviewEnabled -or $importConfig.SoftBoundaryInsetMeters -ne 12 -or
                $importConfig.SoftBoundary.Name -ne 'Saved inner' -or $importConfig.HardBoundary.Name -ne 'Saved outer' -or
                $importConfig.AdvisoryAltitudeDisplayThresholdMeters -ne 100) { throw 'Committed import lost geometry/settings.' }
            if ($fault -eq 'inner-success' -and (-not [object]::ReferenceEquals($outerBefore, $importConfig.HardBoundary.Vertices) -or
                $importConfig.HardBoundary.Vertices[0].Alt -ne 21)) { throw 'Inner import changed the other outline.' }
            if ($fault -eq 'outer-undervived' -and (-not [object]::ReferenceEquals($innerBefore, $importConfig.SoftBoundary.Vertices) -or
                $importConfig.SoftBoundary.Vertices[0].Alt -ne 11)) { throw 'Outer import changed the other outline.' }
            $persisted = $geofenceLoadMethod.Invoke($null, [object[]]@([IO.File]::ReadAllText($importPath)))
            if ([Newtonsoft.Json.JsonConvert]::SerializeObject($persisted) -cne
                [Newtonsoft.Json.JsonConvert]::SerializeObject($importConfig)) { throw 'Persisted import differs from published geometry.' }
            if ($fault -eq 'success' -and [IO.File]::ReadAllText($importPath + '.bak') -cne $initialDisk) {
                throw 'Atomic import lost the previous primary bytes.'
            }
        } else {
            if ($saved -or -not [object]::ReferenceEquals($innerBefore, $importConfig.SoftBoundary.Vertices) -or
                -not [object]::ReferenceEquals($outerBefore, $importConfig.HardBoundary.Vertices) -or
                [Newtonsoft.Json.JsonConvert]::SerializeObject($importConfig) -cne $geometryBefore -or
                [IO.File]::ReadAllText($importPath) -cne $initialDisk) { throw ('Failed/cancelled import mutated originals: ' + $fault) }
            if ($fault -eq 'blocked-backup' -and (Test-Path -LiteralPath ($importPath + '.tmp'))) {
                throw 'Failed staged replacement left temporary import bytes.'
            }
        }
    }
    # Actual DLL coordinate parser: explicit order and all-or-nothing input under both cultures.
    $parseCoordinates = $boundaryViewType.GetMethod('TryParseCoordinates', [Reflection.BindingFlags]'Static,NonPublic')
    $originalCulture = [Threading.Thread]::CurrentThread.CurrentCulture
    try {
        foreach ($culture in @('en-CA', 'fr-CA')) {
            [Threading.Thread]::CurrentThread.CurrentCulture = [Globalization.CultureInfo]::GetCultureInfo($culture)
            foreach ($longitudeFirst in @($false, $true)) {
                $parseArguments = [object[]]@('-20,30', $longitudeFirst, $null, $null)
                if (-not $parseCoordinates.Invoke($null, $parseArguments)) { throw 'Actual DLL rejected ordered coordinates.' }
                $parsedPoint = $parseArguments[2][0]
                $latitude = if ($longitudeFirst) { 30 } else { -20 }
                $longitude = if ($longitudeFirst) { -20 } else { 30 }
                if ($parsedPoint.Lat -ne $latitude -or $parsedPoint.Lon -ne $longitude) { throw 'Actual DLL guessed coordinate order.' }
            }
            $parseArguments = [object[]]@("`r`n1,2`r`nbad`r`n3,4", $false, $null, $null)
            if ($parseCoordinates.Invoke($null, $parseArguments) -or $null -ne $parseArguments[2] -or
                -not $parseArguments[3].StartsWith('Line 3:')) { throw 'Actual DLL partly accepted malformed input.' }
        }
    } finally { [Threading.Thread]::CurrentThread.CurrentCulture = $originalCulture }
    # Replace and append use the same actual filesystem transaction; failure cannot publish to live outlines.
    foreach ($append in @($false, $true)) {
        foreach ($fault in @('blocked-temp','blocked-backup','locked-primary','success')) {
            $pasteConfig = $geofenceLoadMethod.Invoke($null, [object[]]@($originalJson))
            $pasteConfig.SoftBoundaryFromHard = $false
            $boundaryViewType.GetField('_missionConfig', $privateFlags).SetValue($view, $pasteConfig)
            $innerBefore = $pasteConfig.SoftBoundary.Vertices; $outerBefore = $pasteConfig.HardBoundary.Vertices
            $beforePaste = [Newtonsoft.Json.JsonConvert]::SerializeObject($pasteConfig)
            $pastePath = Join-Path $temporary ('paste-' + $append + '-' + $fault + '.json')
            [IO.File]::WriteAllText($pastePath, $beforePaste)
            if ($fault -eq 'blocked-temp') { New-Item -ItemType Directory -Path ($pastePath + '.tmp') | Out-Null }
            if ($fault -eq 'blocked-backup') { New-Item -ItemType Directory -Path ($pastePath + '.bak') | Out-Null }
            $locked = $null
            if ($fault -eq 'locked-primary') { $locked = [IO.File]::Open($pastePath, 'Open', 'Read', 'Read') }
            $parseArguments = [object[]]@('0,0', $false, $null, $null)
            if (-not $parseCoordinates.Invoke($null, $parseArguments)) { throw 'Actual DLL rejected legal zero.' }
            $pasteArguments = [object[]]@($parseArguments[2], [Windows.Forms.DialogResult]::No, $null, $pastePath, $append)
            $pasteArguments[3] = [string]$pastePath
            try { $saved = $applyFence.Invoke($view, $pasteArguments) } finally { if ($locked) { $locked.Dispose() } }
            if ($fault -eq 'success') {
                $expectedCount = if ($append) { 4 } else { 1 }
                if (-not $saved -or $pasteConfig.HardBoundary.Vertices.Count -ne $expectedCount -or
                    $pasteConfig.HardBoundary.Vertices[$expectedCount - 1].Lat -ne 0 -or
                    -not [object]::ReferenceEquals($innerBefore, $pasteConfig.SoftBoundary.Vertices) -or
                    $pasteConfig.AdvisoryAltitudeDisplayThresholdMeters -ne 100 -or
                    ($append -and $pasteConfig.HardBoundary.Vertices[0].Alt -ne 21)) {
                    throw 'Actual DLL paste lost append order, prior altitude, settings or other outline.'
                }
                $persistedPaste = $geofenceLoadMethod.Invoke($null, [object[]]@([IO.File]::ReadAllText($pastePath)))
                if ([Newtonsoft.Json.JsonConvert]::SerializeObject($persistedPaste) -cne
                    [Newtonsoft.Json.JsonConvert]::SerializeObject($pasteConfig)) { throw 'Actual DLL paste persistence differs.' }
            } else {
                if ($saved -or -not [object]::ReferenceEquals($innerBefore, $pasteConfig.SoftBoundary.Vertices) -or
                    -not [object]::ReferenceEquals($outerBefore, $pasteConfig.HardBoundary.Vertices) -or
                    [Newtonsoft.Json.JsonConvert]::SerializeObject($pasteConfig) -cne $beforePaste -or
                    [IO.File]::ReadAllText($pastePath) -cne $beforePaste -or
                    -not $pasteArguments[2].Contains('Check the primary and backup')) {
                    throw ('Actual DLL failed paste changed originals or concealed persistence uncertainty: ' + $fault)
                }
            }
        }
    }
    # Inspect the real contextual controls without calling the view constructor or loading saved outlines.
    $pasteConfig = $geofenceLoadMethod.Invoke($null, [object[]]@($originalJson))
    $boundaryViewType.GetField('_missionConfig', $privateFlags).SetValue($view, $pasteConfig)
    $innerCard = $boundaryViewType.GetMethod('BuildSoftBoundaryCard', $privateFlags).Invoke($view, @())
    $outerCard = $boundaryViewType.GetMethod('BuildHardBoundaryCard', $privateFlags).Invoke($view, @())
    try {
        foreach ($entry in @(@($innerCard, $false), @($outerCard, $true))) {
            $pending = [Collections.Generic.Queue[Windows.Forms.Control]]::new()
            $pending.Enqueue($entry[0])
            $pasteButtons = @()
            while ($pending.Count -gt 0) {
                $control = $pending.Dequeue()
                if ($control -is [Windows.Forms.Button] -and $control.Text -eq 'Paste') { $pasteButtons += $control }
                foreach ($child in $control.Controls) { $pending.Enqueue($child) }
            }
            if ($pasteButtons.Count -ne 1 -or $pasteButtons[0].Enabled -ne $entry[1]) {
                throw 'Contextual Paste controls lost target or derived-inner enablement.'
            }
        }
    } finally { $innerCard.Dispose(); $outerCard.Dispose() }
    Write-Host 'Actual DLL coordinate orders, whole-input rejection, paste transactions and contextual controls passed.'
    # The same current observation must drive both the position label and advisory state, including queued old events.
    $displayConfig = $geofenceLoadMethod.Invoke($null, [object[]]@(
        '{"HardBoundary":{"Vertices":[{"Lat":1,"Lon":1},{"Lat":1,"Lon":2},{"Lat":2,"Lon":2},{"Lat":2,"Lon":1}]}}'))
    $monitorType = $plugin.GetType('NOMAD.MissionPlanner.AdvisoryBoundaryMonitor', $true)
    $monitor = [Activator]::CreateInstance($monitorType, [object[]]@($displayConfig))
    $positionLabel = [Windows.Forms.Label]::new()
    $statusLabel = [Windows.Forms.Label]::new()
    $statusPanel = [Windows.Forms.Panel]::new()
    $boundaryViewType.GetField('_monitor', $privateFlags).SetValue($view, $monitor)
    $boundaryViewType.GetField('_lblPosition', $privateFlags).SetValue($view, $positionLabel)
    $boundaryViewType.GetField('_lblStatus', $privateFlags).SetValue($view, $statusLabel)
    $boundaryViewType.GetField('_statusPanel', $privateFlags).SetValue($view, $statusPanel)
    $observationType = $plugin.GetType('NOMAD.MissionPlanner.AdvisoryBoundaryStatusEventArgs', $true)
    $insideObservation = [Activator]::CreateInstance($observationType)
    $insideObservation.Position = $point
    $insideObservation.Status = [Enum]::Parse(
        $plugin.GetType('NOMAD.MissionPlanner.AdvisoryOutlineStatus'), 'InsideOutlines')
    $applyObservation = $boundaryViewType.GetMethod('ApplyAdvisoryObservation', $privateFlags)
    $presentObservation = $boundaryViewType.GetMethod('Monitor_AdvisoryStatusChanged', $privateFlags)
    try {
        $applyObservation.Invoke($view, [object[]]@($insideObservation)) | Out-Null
        if ($positionLabel.Text -ne ('Received position: {0:F6}, {1:F6}' -f 1.5, 1.5) -or
            -not $statusLabel.Text.Contains('inside the saved outlines')) {
            throw 'The boundary label and status did not use the same received observation.'
        }
        $presentObservation.Invoke($view, [object[]]@($monitor, $insideObservation)) | Out-Null
        if ($positionLabel.Text -ne 'Received position: --' -or
            -not $statusLabel.Text.Contains('No fresh live received position')) {
            throw 'An old queued event restored inside state or mismatched position text.'
        }
    } finally { $monitor.Dispose(); $positionLabel.Dispose(); $statusLabel.Dispose(); $statusPanel.Dispose() }
    # Use isolated maps and a constructor-skipped planner; never load a real MP screen or saved outlines.
    $renderer = $plugin.GetType('NOMAD.MissionPlanner.MapOverlayManager', $true)
    $staticFlags = [Reflection.BindingFlags]'Static,Public,NonPublic'
    $plannerType = [MissionPlanner.GCSViews.FlightPlanner]
    $plannerInstance = $plannerType.GetProperty('instance', $staticFlags)
    $dataMapField = [MissionPlanner.GCSViews.FlightData].GetField('mymap', $staticFlags)
    $oldPlanner = $plannerInstance.GetValue($null, $null)
    $oldDataMap = $dataMapField.GetValue($null)
    $planner = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($plannerType)
    $planMap = [Activator]::CreateInstance($plannerType.GetField('MainMap').FieldType)
    $dataMap = [Activator]::CreateInstance($dataMapField.FieldType)
    $nativeOverlay = [GMap.NET.WindowsForms.GMapOverlay]::new('native-plan-sentinel')
    $nativePoints = [Collections.Generic.List[GMap.NET.PointLatLng]]::new()
    foreach ($point in @(@(46.0, -76.0), @(46.01, -76.0), @(46.01, -76.01))) {
        $nativePoints.Add([GMap.NET.PointLatLng]::new($point[0], $point[1]))
    }
    $nativePolygon = [GMap.NET.WindowsForms.GMapPolygon]::new($nativePoints, 'native-fence')
    $nativePoints = $nativePolygon.Points
    $otherPolygon = [GMap.NET.WindowsForms.GMapPolygon]::new(
        [Collections.Generic.List[GMap.NET.PointLatLng]]::new($nativePoints), 'native-other-shape')
    $otherPoints = $otherPolygon.Points
    $nativeMarker = [GMap.NET.WindowsForms.Markers.GMarkerGoogle]::new(
        [GMap.NET.PointLatLng]::new(46.005, -76.005),
        [GMap.NET.WindowsForms.Markers.GMarkerGoogleType]::blue_small)
    $nativeOverlay.Polygons.Add($nativePolygon)
    $nativeOverlay.Polygons.Add($otherPolygon)
    $nativeOverlay.Markers.Add($nativeMarker)
    $planMap.Overlays.Add($nativeOverlay)
    $plannerType.GetField('MainMap').SetValue($planner, $planMap)
    $plannerType.GetField('geofencepolygon').SetValue($planner, $nativePolygon)
    $plannerType.GetField('geofenceoverlay').SetValue($planner, $nativeOverlay)
    $polygonCollection = $nativeOverlay.Polygons
    $markerCollection = $nativeOverlay.Markers
    $nativePointValues = [Newtonsoft.Json.JsonConvert]::SerializeObject($nativePoints)
    $markerPosition = $nativeMarker.Position
    $outlineJson = '{"SoftBoundary":{"Vertices":[' +
        '{"Lat":44.998,"Lon":-75.003},{"Lat":45.002,"Lon":-75.003},' +
        '{"Lat":45.002,"Lon":-74.997},{"Lat":44.998,"Lon":-74.997}]},' +
        '"HardBoundary":{"Vertices":[' +
        '{"Lat":44.996,"Lon":-75.006},{"Lat":45.004,"Lon":-75.006},' +
        '{"Lat":45.004,"Lon":-74.994},{"Lat":44.996,"Lon":-74.994}]}}'
    $outlines = $geofenceLoadMethod.Invoke($null, [object[]]@($outlineJson))
    $outlineValues = [Newtonsoft.Json.JsonConvert]::SerializeObject($outlines)
    $inner = $outlines.SoftBoundary.Vertices
    $outer = $outlines.HardBoundary.Vertices
    $innerPoints = $inner.ToArray()
    $outerPoints = $outer.ToArray()
    try {
        $plannerInstance.SetValue($null, $planner, $null)
        $dataMapField.SetValue($null, $dataMap)
        foreach ($map in @($planMap, $dataMap)) {
            # No handle or tiles are opened. Center the in-memory projection for the paint callback.
            $map.Size = [Drawing.Size]::new(320, 240)
            $map.MapProvider = [GMap.NET.MapProviders.EmptyProvider]::Instance
            $map.MaxZoom = 18
            $map.Zoom = 14
            $map.Position = [GMap.NET.PointLatLng]::new(45.0, -75.0)
            $coreProperty = [GMap.NET.WindowsForms.GMapControl].GetProperty(
                'Core', [Reflection.BindingFlags]'Instance,Public,NonPublic')
            $coreProperty.PropertyType.GetMethod('GoToCurrentPosition').Invoke(
                $coreProperty.GetValue($map, $null), @()) | Out-Null
        }
        # Import while native Plan sentinels are bound; the existing checks below cover every reference/collection.
        $planImport = $geofenceLoadMethod.Invoke($null, [object[]]@($originalJson))
        $boundaryViewType.GetField('_missionConfig', $privateFlags).SetValue($view, $planImport)
        $planImportArgs = [object[]]@($fencePoints, [Windows.Forms.DialogResult]::No, $null,
            [string](Join-Path $temporary 'plan-import.json'), $false)
        if (-not $applyFence.Invoke($view, $planImportArgs)) { throw 'Native Plan isolation import failed.' }
        $renderer.GetMethod('DrawBoundaries').Invoke($null, [object[]]@($planImport)) | Out-Null
        if (-not [object]::ReferenceEquals($fenceField.GetValue($nativeMav), $fenceCache) -or
            [Newtonsoft.Json.JsonConvert]::SerializeObject($fenceCache.ToArray()) -cne $fenceBefore) {
            throw 'Import/draw changed native cache keys or payloads.'
        }
        $renderer.GetMethod('DrawBoundaries').Invoke($null, [object[]]@($outlines)) | Out-Null
        if ($renderer.GetMethod('EnsureBoundaryMaps').Invoke($null, @()) -ne 2) {
            throw 'Saved advisory outlines did not bind both Data and Plan maps.'
        }
        $paint = $renderer.GetMethod('BoundaryMap_Paint', $staticFlags)
        foreach ($map in @($planMap, $dataMap)) {
            $canvas = [Drawing.Bitmap]::new(320, 240)
            $graphics = [Drawing.Graphics]::FromImage($canvas)
            try {
                $graphics.Clear([Drawing.Color]::White)
                $paintArgs = [Windows.Forms.PaintEventArgs]::new(
                    $graphics, [Drawing.Rectangle]::new(0, 0, 320, 240))
                $paint.Invoke($null, [object[]]@($map, $paintArgs)) | Out-Null
                if ($canvas.GetPixel(0, 0).ToArgb() -eq [Drawing.Color]::White.ToArgb() -or
                    $canvas.GetPixel(160, 120).ToArgb() -ne [Drawing.Color]::White.ToArgb()) {
                    $center = $map.FromLatLngToLocal([GMap.NET.PointLatLng]::new(45.0, -75.0))
                    $paintFailure = 'Advisory painting did not preserve exterior/interior colors: ' +
                        'corner={0}, center={1}, projected center={2}, zoom={3}'
                    throw ($paintFailure -f $canvas.GetPixel(0, 0), $canvas.GetPixel(160, 120), $center, $map.Zoom)
                }
            } finally { $graphics.Dispose(); $canvas.Dispose() }
        }
        if ($renderer.GetField('_paintErrorLogged', $staticFlags).GetValue($null)) {
            throw 'Advisory rendering failed while painting the isolated maps.'
        }
        if (-not [object]::ReferenceEquals(
                $plannerType.GetField('geofencepolygon').GetValue($planner), $nativePolygon) -or
            -not [object]::ReferenceEquals(
                $plannerType.GetField('geofenceoverlay').GetValue($planner), $nativeOverlay) -or
            -not [object]::ReferenceEquals($nativePolygon.Points, $nativePoints) -or
            -not [object]::ReferenceEquals($otherPolygon.Points, $otherPoints) -or
            [Newtonsoft.Json.JsonConvert]::SerializeObject($nativePoints) -cne $nativePointValues -or
            [Newtonsoft.Json.JsonConvert]::SerializeObject($otherPolygon.Points) -cne $nativePointValues) {
            throw 'Advisory drawing replaced or changed native Plan fence geometry.'
        }
        if (-not [object]::ReferenceEquals($nativeOverlay.Polygons, $polygonCollection) -or
            $nativeOverlay.Polygons.Count -ne 2 -or
            -not [object]::ReferenceEquals($nativeOverlay.Polygons[0], $nativePolygon) -or
            -not [object]::ReferenceEquals($nativeOverlay.Polygons[1], $otherPolygon) -or
            -not [object]::ReferenceEquals($nativeOverlay.Markers, $markerCollection) -or
            $nativeOverlay.Markers.Count -ne 1 -or
            -not [object]::ReferenceEquals($nativeOverlay.Markers[0], $nativeMarker) -or
            $nativeMarker.Position -ne $markerPosition -or $planMap.Overlays.Count -ne 1 -or
            -not [object]::ReferenceEquals($planMap.Overlays[0], $nativeOverlay)) {
            throw 'Advisory drawing changed native Plan overlay or marker state.'
        }
        if (-not [object]::ReferenceEquals($outlines.SoftBoundary.Vertices, $inner) -or
            -not [object]::ReferenceEquals($outlines.HardBoundary.Vertices, $outer) -or
            [Newtonsoft.Json.JsonConvert]::SerializeObject($outlines) -cne $outlineValues) {
            throw 'Advisory drawing changed the imported or saved outline data.'
        }
        for ($index = 0; $index -lt $innerPoints.Length; $index++) {
            if (-not [object]::ReferenceEquals($inner[$index], $innerPoints[$index]) -or
                -not [object]::ReferenceEquals($outer[$index], $outerPoints[$index])) {
                throw 'Advisory drawing replaced an input outline vertex.'
            }
        }
        $card = $boundaryViewType.GetMethod('BuildImportCard', $privateFlags).Invoke($view, @())
        try {
            $controls = [Collections.Generic.Queue[Windows.Forms.Control]]::new()
            $controls.Enqueue($card)
            $texts = [Collections.Generic.List[string]]::new()
            while ($controls.Count -gt 0) {
                $control = $controls.Dequeue()
                $texts.Add($control.Text)
                foreach ($child in $control.Controls) { $controls.Enqueue($child) }
            }
            if ($texts.Contains('Export to Plan Map') -or -not $texts.Contains('KML') -or
                $texts.Contains('Coords') -or -not $texts.Contains('MP Fence')) {
                throw 'Advisory map controls still offer native Plan export or lost a read-only import.'
            }
        } finally { $card.Dispose() }
    } finally {
        $renderer.GetMethod('StopBoundaryRendering').Invoke($null, @()) | Out-Null
        $plannerInstance.SetValue($null, $oldPlanner, $null)
        $dataMapField.SetValue($null, $oldDataMap)
        $planMap.Dispose()
        $dataMap.Dispose()
    }
    if ($fixtureLogPathField.GetValue($null) -cne $fixtureLogPath -or $fixtureLogFailedField.GetValue($null) -or
        -not (Test-Path -LiteralPath $fixtureLogPath -PathType Leaf) -or
        (Get-Item -LiteralPath $fixtureLogPath).Length -eq 0) {
        throw 'Configuration fixture did not retain a working temporary log.'
    }
    Write-Host 'Temporary fixture logging verified.'
    Write-Host 'Mission Planner configuration, HID, presets and advisory map/position validation passed.'
} finally {
    [AppDomain]::CurrentDomain.remove_AssemblyResolve($dependencyResolver)
    $cleanupPath = (Resolve-Path -LiteralPath $temporary).ProviderPath
    $tempRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
    if (-not $cleanupPath.StartsWith($tempRoot, [StringComparison]::OrdinalIgnoreCase) -or
        [IO.Path]::GetFileName($cleanupPath) -notmatch '^nomad-config-test-[0-9a-f]{32}$') {
        throw 'Configuration fixture cleanup target is outside its temporary directory.'
    }
    Remove-Item -LiteralPath $cleanupPath -Recurse -Force
}
