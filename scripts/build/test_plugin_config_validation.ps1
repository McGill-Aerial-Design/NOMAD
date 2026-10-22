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
    # Exercise the replacement page without native host startup or operator-file access.
    $fenceViewType = $plugin.GetType('NOMAD.MissionPlanner.NativeFenceSetupView', $true)
    $fenceInputType = $plugin.GetType('NOMAD.MissionPlanner.FenceSetupInput', $true)
    $fenceInput = [Activator]::CreateInstance($fenceInputType, $true)
    $fenceConstructor = $fenceViewType.GetConstructor([Reflection.BindingFlags]'Instance,NonPublic',
        $null, [type[]]@([Threading.CancellationToken], $fenceInputType), $null)
    $fenceRenderDir = Join-Path $repoRoot 'build/fence-ui'
    New-Item -ItemType Directory -Path $fenceRenderDir -Force | Out-Null
    foreach ($scale in @([single]1, [single]1.5)) {
        $fenceView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::None, $fenceInput))
        try {
            $fenceView.Size = [Drawing.Size]::new(760, 680)
            $fenceView.CreateControl()
            $orderControl = $fenceViewType.GetField('_order', [Reflection.BindingFlags]'Instance,NonPublic').GetValue($fenceView)
            $null = $orderControl.Handle
            if ($orderControl.SelectedIndex -ne 0 -or $orderControl.Text -ne 'Latitude, longitude') {
                throw 'Fence coordinate order is not selected or readable.'
            }
            $published = $plugin.GetType('NOMAD.MissionPlanner.FenceCoordinates').GetField(
                'PublishedC1', [Reflection.BindingFlags]'Static,NonPublic').GetRawConstantValue()
            $fenceView.Controls.Find('FenceCoordinates', $true)[0].Text = $published
            $fenceView.Scale([Drawing.SizeF]::new($scale, $scale))
            $bitmap = [Drawing.Bitmap]::new($fenceView.Width, $fenceView.Height)
            try {
                $fenceView.DrawToBitmap($bitmap, [Drawing.Rectangle]::new(0, 0, $bitmap.Width, $bitmap.Height))
                $bitmap.Save((Join-Path $fenceRenderDir ('fence-setup-' + $scale + '.png')))
            } finally { $bitmap.Dispose() }
            $text = $fenceView.Controls.Find('FenceSetupStatus', $true)[0].Text
            if ($text -notmatch 'Staging.*does not upload') { throw 'Fence setup lost truthful initial transfer status.' }
            if ($fenceView.Controls.Find('FenceCoordinates', $true).Count -ne 1) { throw 'Fence coordinate editor missing.' }
        } finally { $fenceView.Dispose() }
    }
    $draftText = "1,1`r`n1,2`r`n2,1"
    $firstView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::None, $fenceInput))
    $firstView.Controls.Find('FenceCoordinates', $true)[0].Text = $draftText
    $firstView.Dispose()
    $secondView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::None, $fenceInput))
    try {
        if ($secondView.Controls.Find('FenceCoordinates', $true)[0].Text -cne $draftText) {
            throw 'Native page navigation/disposal lost the unsaved coordinate draft.'
        }
    } finally { $secondView.Dispose() }
    $cancelledView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::new($true), $fenceInput))
    try {
        $cancelledView.Controls.Find('FenceCoordinates', $true)[0].Text = 'stale input'
        if ($fenceInputType.GetField('Coordinates', [Reflection.BindingFlags]'Instance,NonPublic').GetValue($fenceInput) -cne $draftText) {
            throw 'A cancelled session modified the retained input.'
        }
    } finally { $cancelledView.Dispose() }
    Write-Host 'Native Fence page rendered at 100/150 percent; unsaved draft survives recreation and rejects retired input.'

    if ($fixtureLogPathField.GetValue($null) -cne $fixtureLogPath) {
        throw 'Configuration fixture log path was not installed.'
    }

    # This isolated process has no live Mission Planner connection or selected MAV.
    $outputType = $plugin.GetType('NOMAD.MissionPlanner.OutputController', $true)
    $connectionField = $outputType.GetField('CurrentConnection', [Reflection.BindingFlags]'Static,NonPublic')
    $connectionField.SetValue($null, [System.Management.Automation.LanguagePrimitives]::ConvertTo(
        [scriptblock] { return $null }, $connectionField.FieldType))
    # Exercise real MP event add/remove accessors without constructing a stream, reading settings or opening devices.
    $nativeType = $connectionField.FieldType.GetGenericArguments()[0]
    $nativeConnection = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($nativeType)
    [GC]::SuppressFinalize($nativeConnection)
    $nativeFlags = [Reflection.BindingFlags]'Instance,NonPublic'
    $initializeNative = $outputType.GetMethod('Initialize', [Reflection.BindingFlags]'Static,NonPublic')
    $shutdownNative = $outputType.GetMethod('Shutdown', [Reflection.BindingFlags]'Static,NonPublic')
    $generationField = $outputType.GetField('_generation', [Reflection.BindingFlags]'Static,NonPublic')
    $connectionField.SetValue($null, [System.Management.Automation.LanguagePrimitives]::ConvertTo(
        [scriptblock] { return $nativeConnection }, $connectionField.FieldType))
    try {
        $initializeNative.Invoke($null, [object[]]@([Activator]::CreateInstance($configType))) | Out-Null
        foreach ($name in @('_MavChanged','_CommsClose')) {
            $handlers = $nativeType.GetField($name, $nativeFlags).GetValue($nativeConnection)
            if ($null -eq $handlers -or $handlers.GetInvocationList().Count -ne 1 -or $null -eq $handlers.Target) {
                throw 'Native MP connection events require one instance-target subscription.'
            }
        }
        $beforeGeneration = $generationField.GetValue($null)
        $nativeConnection.sysidcurrent = 42
        if ($generationField.GetValue($null) -le $beforeGeneration) { throw 'Native selection event did not retire old input.' }
        $shutdownNative.Invoke($null, @()) | Out-Null
        foreach ($name in @('_MavChanged','_CommsClose')) {
            if ($null -ne $nativeType.GetField($name, $nativeFlags).GetValue($nativeConnection)) {
                throw 'Shutdown retained a native event subscription.'
            }
        }
    } finally {
        $connectionField.SetValue($null, [System.Management.Automation.LanguagePrimitives]::ConvertTo(
            [scriptblock] { return $null }, $connectionField.FieldType))
    }
    Write-Host 'Actual Mission Planner event accessors accept, invalidate and remove the native subscriptions.'

    . (Join-Path $PSScriptRoot 'test_health_observation.ps1')
    Invoke-HealthObservationTests $plugin

    . (Join-Path $PSScriptRoot 'test_aircraft_profiles.ps1')
    Invoke-AircraftProfileTests $plugin $configType $temporary
    Invoke-AircraftProfileUiTests $plugin $configType
    . (Join-Path $PSScriptRoot 'test_setup_editors.ps1')
    Invoke-SetupEditorTests $plugin $configType

    # Legacy source migration is in-memory only; every current save has one named source of truth.
    $legacyVideo = Load-Config '{"VideoUrl":"rtsp://synthetic:credential@video.invalid/live"}'
    if ($legacyVideo.SelectedVideoInputName -ne 'Default' -or $legacyVideo.VideoInputs.Count -ne 1 -or
        $legacyVideo.SelectedVideoInput.Url -ne 'rtsp://synthetic:credential@video.invalid/live') {
        throw 'Legacy video URL was lost during named-input migration.'
    }
    $videoExport = Join-Path $temporary 'video-export.json'
    $legacyVideo.ExportToFile($videoExport)
    if ([IO.File]::ReadAllText($videoExport).Contains('"VideoUrl"')) { throw 'Video export retained two sources of truth.' }
    $videoCopy = $loadMethod.Invoke($null, [object[]]@([string]$videoExport)).Clone()
    if ($videoCopy.SelectedVideoInput.Url -ne $legacyVideo.SelectedVideoInput.Url) { throw 'Video save/load/clone lost the source.' }
    foreach ($json in @('{}', '{"VideoUrl":""}', '{"VideoInputs":[],"SelectedVideoInputName":""}')) {
        $empty = Load-Config $json
        if ($null -ne $empty.SelectedVideoInput -or $empty.VideoInputs.Count -ne 0) { throw 'Empty video acquired a source.' }
    }
    $videoJson = '{"VideoInputs":[{"Name":"Camera","Url":"rtsp://video.invalid/live","RtspTransport":"UDP",' +
        '"LatencyMs":5000,"QueueMaxBuffers":120,"DropOldFrames":false}],"SelectedVideoInputName":"Camera"}'
    $video = Load-Config $videoJson
    if ($video.SelectedVideoInput.LatencyMs -ne 5000 -or $video.SelectedVideoInput.QueueMaxBuffers -ne 120 -or
        $video.SelectedVideoInput.DropOldFrames -or $video.SelectedVideoInput.RtspTransport -ne 'UDP') {
        throw 'Named receiver settings did not retain their boundary values.'
    }
    foreach ($json in @('{"VideoUrl":null}', '{"VideoUrl":1}', '{"VideoUrl":"","VideoInputs":[]}',
        '{"VideoInputs":null}', '{"VideoInputs":{}}', '{"VideoInputs":[null]}',
        '{"VideoInputs":[{}]}', '{"SelectedVideoInputName":1}', '{"SelectedVideoInput":null}',
        '{"VideoInputs":[],"SelectedVideoInputName":"missing"}',
        $videoJson.Replace('"Camera"}', '"missing"}'),
        $videoJson.Replace('"LatencyMs":5000', '"LatencyMs":"100"'),
        $videoJson.Replace('"LatencyMs":5000', '"LatencyMs":-1'),
        $videoJson.Replace('"LatencyMs":5000', '"LatencyMs":5001'),
        $videoJson.Replace('"QueueMaxBuffers":120', '"QueueMaxBuffers":0'),
        $videoJson.Replace('"QueueMaxBuffers":120', '"QueueMaxBuffers":121'),
        $videoJson.Replace('"QueueMaxBuffers":120', '"QueueMaxBuffers":1.5'),
        $videoJson.Replace('"DropOldFrames":false', '"DropOldFrames":0'),
        $videoJson.Replace('"RtspTransport":"UDP"', '"RtspTransport":"AUTO"'),
        $videoJson.Replace('"Url":', '"url":'),
        $videoJson.Replace('"Name":"Camera"', '"Name":" Camera"'),
        $videoJson.Replace('"Name":"Camera"', '"Name":""'),
        $videoJson.Replace('"Url":"rtsp://video.invalid/live"', '"Url":null'),
        $videoJson.Replace('"Url":"rtsp://video.invalid/live"', '"Url":"http://video.invalid/live"'),
        '{"VideoInputs":[{"Name":"A","Url":"udp://5600"},{"Name":"a","Url":"udp://5601"}],"SelectedVideoInputName":"A"}')) {
        $rejected = $false
        try { Load-Config $json | Out-Null } catch { $rejected = $true }
        if (!$rejected) { throw 'Malformed named video input was accepted.' }
    }
    $profilesType = $plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    $profileDeserialize = $profilesType.GetMethod('Deserialize', [Reflection.BindingFlags]'Static,NonPublic')
    $profilePrefix = '{"Aircraft":[{"Id":"00000000000000000000000000000001","Config":'
    $profileSuffix = '}],"SelectedId":"00000000000000000000000000000001"}'
    $nestedVideo = $profileDeserialize.Invoke($null, [object[]]@($profilePrefix +
        '{"AircraftName":"Synthetic","VideoUrl":"udp://5600"}' + $profileSuffix))
    if ($nestedVideo.Selected.Config.SelectedVideoInput.Url -ne 'udp://5600') { throw 'Aircraft profile missed legacy video migration.' }
    $rejected = $false
    try {
        $profileDeserialize.Invoke($null, [object[]]@($profilePrefix +
            '{"AircraftName":"Synthetic","VideoInputs":[{"Name":"A","Url":false}]}' + $profileSuffix)) | Out-Null
    } catch { $rejected = $true }
    if (!$rejected) { throw 'Aircraft profile bypassed nested video token validation.' }
    Write-Host 'Named video migration, save/load/clone, strict nested validation and limits passed.'

    $config = Load-Config '{"CoreRuntimePort":14631,"RouterLocalPort":14620,"ManagementPort":14621,"CoreClientCredential":"test-only-credential"}'
    if ($config.RouterLocalPort -ne 14620) { throw 'Saved Mission Planner router endpoint changed.' }
    $export = Join-Path $temporary 'export.json'
    $config.ExportToFile($export)
    if ([IO.File]::ReadAllText($export).Contains('test-only-credential')) { throw 'Export leaked client credential.' }
    foreach ($field in @('Payloads','Actuators','RouterMode','RouterEnabled','JoystickCameraTiltEnabled','JoystickZedEnabled','SerialJoystickEnabled','SlamCameraFovDeg')) {
        Assert-ThrowsMessage ('{"' + $field + '":null}') $field
    }
    Assert-ThrowsMessage '{"RouterLocalPort":0}' 'ports'
    Assert-ThrowsMessage '{"JoystickButtonIndices":[0,1,2,3,4,128]}' 'between 0 and 127'
    Assert-ThrowsMessage '{"JoystickButtonIndices":[0,1,2,3,4,5.5]}' 'integers'
    $freedButton = Load-Config '{"JoystickButtonIndices":[6,1,2,3,4,5],"JoystickSw1UpAction":"a:activate"}'
    if ($freedButton.JoystickButtonIndices[0] -ne 6) { throw 'The removed monitor still reserved its former default button.' }
    foreach ($button in @(0, 127)) {
        $retiredPath = Join-Path $temporary ('retired-monitor-' + $button + '.json')
        $retiredJson = '{"JoystickTerminationButtonIndex":' + $button +
            ',"JoystickKillSwitchEnabled":true,"JoystickButtonIndices":[' + $button +
            ',1,2,3,4,5],"JoystickSw1UpAction":"a:activate"}'
        [IO.File]::WriteAllText($retiredPath, $retiredJson)
        $retired = $loadMethod.Invoke($null, [object[]]@([string]$retiredPath))
        if ($retired.JoystickButtonIndices[0] -ne $button -or $retired.JoystickSw1UpAction -ne 'a:activate' -or
            [IO.File]::ReadAllText($retiredPath) -cne $retiredJson) {
            throw 'Retired termination monitoring reserved a payload button or changed its input file.'
        }
        $retiredExport = Join-Path $temporary ('retired-monitor-export-' + $button + '.json')
        $retired.ExportToFile($retiredExport)
        $saved = [IO.File]::ReadAllText($retiredExport)
        if ($saved.Contains('JoystickTerminationButtonIndex') -or $saved.Contains('JoystickKillSwitchEnabled')) {
            throw 'Current configuration export persisted unused termination monitor settings.'
        }
    }
    foreach ($invalid in @('{"JoystickKillSwitchEnabled":"true"}', '{"JoystickKillSwitchEnabled":1}',
        '{"JoystickKillSwitchEnabled":null}', '{"JoystickTerminationButtonIndex":"0"}',
        '{"JoystickTerminationButtonIndex":0.5}', '{"JoystickTerminationButtonIndex":null}')) {
        Assert-ThrowsMessage $invalid 'must be'
    }
    foreach ($invalid in @('{"JoystickTerminationButtonIndex":-1}', '{"JoystickTerminationButtonIndex":128}',
        '{"JoystickTerminationButtonIndex":999999999999999999999}')) {
        Assert-ThrowsMessage $invalid 'between 0 and 127'
    }
    foreach ($field in @('joystickKillSwitchEnabled', 'joystickTerminationButtonIndex', 'JoystickTerminationEnabled')) {
        Assert-ThrowsMessage ('{"' + $field + '":true}') $field
    }
    Assert-ThrowsMessage '{"JoystickKillSwitchEnabled":true,"JoystickKillSwitchEnabled":false}' 'already exists'
    Assert-ThrowsMessage '{"JoystickTerminationButtonIndex":0,"JoystickTerminationButtonIndex":1}' 'already exists'
    Assert-ThrowsMessage '{"JoystickButtonIndices":[0,0,2,3,4,5],"JoystickSw1UpAction":"a:activate","JoystickSw1DownAction":"b:activate"}' 'unique'
    Assert-ThrowsMessage '{"CoreRuntimePort":14611,"CoreRuntimePort":14612}' 'already exists'
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
    foreach ($json in @('{"RouterLinks":null}', '{"RouterLinks":[{"Id":"x","Port":1.5}]}',
        '{"RouterLinks":[{"Id":"x","Enabled":"true"}]}', '{"RouterLinks":[{"Id":"x","port":14750}]}',
        '{"RouterLinks":[{"Id":"x","Port":14750,"Unexpected":1}]}', '{"RouterPreferredLink":5}')) {
        $rejected = $false
        try { Load-Config $json | Out-Null } catch { $rejected = $true }
        if (-not $rejected) { throw "Invalid router document was accepted: $json" }
    }
    $router = Load-Config '{"RouterLocalPort":14700,"RouterPreferredLink":"elrs","RouterLinks":[{"Id":"elrs","Port":14750,"Priority":100},{"Id":"lte","Port":14760,"Priority":50}]}'
    $routerExport = Join-Path $temporary 'router.json'
    $router.ExportToFile($routerExport)
    $routerCopy = $configType.GetMethod('LoadFromFile').Invoke($null, [object[]]@([string]$routerExport))
    if ($routerCopy.RouterLinks.Count -ne 2 -or $routerCopy.RouterPreferredLink -ne 'elrs') { throw 'Router profile roundtrip lost links.' }
    # Exercise real settings controls without opening a window or saving host configuration.
    $formType = $plugin.GetType('NOMAD.MissionPlanner.NOMADSettingsForm', $true)
    $config.JoystickSwitchDevice = 'selected-unplugged-usb-hid'
    $formArguments = New-Object object[] 1
    $formArguments[0] = $config
    $profilesType = $plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    $profiles = $profilesType.GetConstructor([type[]]@($configType)).Invoke($formArguments)
    $formArguments[0] = $profiles
    $outputType = $plugin.GetType('NOMAD.MissionPlanner.OutputController', $true)
    $initializeOutput = $outputType.GetMethod('Initialize', [Reflection.BindingFlags]'Static,NonPublic')
    $initializeOutput.Invoke($null, [object[]]@($config)) | Out-Null
    $form = $formType.GetConstructor([type[]]@($profilesType)).Invoke($formArguments)
    try {
        $privateFlags = [Reflection.BindingFlags]'Instance,NonPublic'
        $device = $formType.GetField('_cmbSwitchDevice', $privateFlags).GetValue($form)
        if ($device.SelectedItem -ne 'selected-unplugged-usb-hid') { throw 'Unplugged selected HID name was discarded.' }
        $device.Items.Clear()
        $device.Items.Add('(none)') | Out-Null
        $setDevice = $formType.GetMethod('SetDeviceComboValue', [Reflection.BindingFlags]'Static,NonPublic')
        $setDevice.Invoke($null, [object[]]@($device, 'selected-unplugged-usb-hid')) | Out-Null
        if ($device.SelectedItem -ne 'selected-unplugged-usb-hid') { throw 'Device refresh discarded an explicit HID name.' }

        $routerDraft = Load-Config '{"RouterPreferredLink":"elrs","RouterLinks":[{"Id":"elrs","Name":"Primary","Transport":"UDP","Port":14750,"Priority":100},{"Id":"lte","Name":"Backup","Transport":"UDP","Port":14760,"Priority":50}]}'
        $formType.GetMethod('ApplyRouterDraft', $privateFlags).Invoke($form, [object[]]@($routerDraft)) | Out-Null
        $formType.GetMethod('SaveSettings', $privateFlags).Invoke($form, @()) | Out-Null
        if ($form.Config.RouterLinks.Count -ne 2 -or $form.Config.RouterPreferredLink -ne 'elrs') {
            throw 'Saving router settings lost physical links or preferred path.'
        }

    } finally { $form.Dispose() }
    if ($fixtureLogPathField.GetValue($null) -cne $fixtureLogPath -or $fixtureLogFailedField.GetValue($null) -or
        -not (Test-Path -LiteralPath $fixtureLogPath -PathType Leaf) -or
        (Get-Item -LiteralPath $fixtureLogPath).Length -eq 0) {
        throw 'Configuration fixture did not retain a working temporary log.'
    }
    Write-Host 'Temporary fixture logging verified.'
    Write-Host 'Mission Planner configuration, HID and router validation passed.'
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
