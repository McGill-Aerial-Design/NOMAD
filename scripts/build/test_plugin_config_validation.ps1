# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Test Mission Planner's current configuration validation using the built plugin.

$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'test_plugin_storage.ps1')
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
        $null, [type[]]@([Threading.CancellationToken], $fenceInputType, $configType), $null)
    $fenceRenderDir = Join-Path $repoRoot 'build/fence-ui'
    New-Item -ItemType Directory -Path $fenceRenderDir -Force | Out-Null
    foreach ($scale in @([single]1, [single]1.5)) {
        $fenceView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::None, $fenceInput, [Activator]::CreateInstance($configType)))
        $fenceForm = [Windows.Forms.Form]::new()
        try {
            $fenceView.Size = [Drawing.Size]::new(760, 680)
            $fenceForm.ClientSize = $fenceView.Size
            $fenceForm.StartPosition = 'Manual'
            $fenceForm.Location = [Drawing.Point]::new(-20000, -20000)
            $fenceForm.ShowInTaskbar = $false
            $fenceForm.Controls.Add($fenceView)
            $fenceForm.Show()
            [Windows.Forms.Application]::DoEvents()
            $orderControl = $fenceViewType.GetField('_order', [Reflection.BindingFlags]'Instance,NonPublic').GetValue($fenceView)
            $null = $orderControl.Handle
            if ($orderControl.SelectedIndex -ne 0 -or $orderControl.Text -ne 'Latitude, longitude') {
                throw 'Fence coordinate order is not selected or readable.'
            }
            $coordinates = "1,1`r`n1,2`r`n2,1"
            $fenceView.Controls.Find('FenceCoordinates', $true)[0].Text = $coordinates
            $fenceView.Scale([Drawing.SizeF]::new($scale, $scale))
            $fenceForm.ClientSize = $fenceView.Size
            $fenceForm.PerformLayout()
            [Windows.Forms.Application]::DoEvents()
            $bitmap = [Drawing.Bitmap]::new($fenceView.Width, $fenceView.Height)
            try {
                $fenceView.DrawToBitmap($bitmap, [Drawing.Rectangle]::new(0, 0, $bitmap.Width, $bitmap.Height))
                $bitmap.Save((Join-Path $fenceRenderDir ('fence-setup-' + $scale + '.png')))
            } finally { $bitmap.Dispose() }
            $text = $fenceView.Controls.Find('FenceSetupStatus', $true)[0].Text
            if ($text -ne '') { throw 'Idle Fence setup displayed a stale action status.' }
            if ($fenceView.Controls.Find('FenceCoordinates', $true).Count -ne 1) { throw 'Fence coordinate editor missing.' }
        } finally { $fenceForm.Dispose() }
    }
    $draftText = "1,1`r`n1,2`r`n2,1"
    $firstView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::None, $fenceInput, [Activator]::CreateInstance($configType)))
    $firstView.Controls.Find('FenceCoordinates', $true)[0].Text = $draftText
    $firstView.Dispose()
    $secondView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::None, $fenceInput, [Activator]::CreateInstance($configType)))
    try {
        if ($secondView.Controls.Find('FenceCoordinates', $true)[0].Text -cne $draftText) {
            throw 'Native page navigation/disposal lost the unsaved coordinate draft.'
        }
    } finally { $secondView.Dispose() }
    $cancelledView = $fenceConstructor.Invoke([object[]]@([Threading.CancellationToken]::new($true), $fenceInput, [Activator]::CreateInstance($configType)))
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

    foreach ($json in @('{}', '{"VideoInputs":[],"SelectedVideoInputName":""}')) {
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
    foreach ($json in @(
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
    $rejected = $false
    try {
        $profileDeserialize.Invoke($null, [object[]]@($profilePrefix +
            '{"AircraftName":"Synthetic","VideoInputs":[{"Name":"A","Url":false}]}' + $profileSuffix)) | Out-Null
    } catch { $rejected = $true }
    if (!$rejected) { throw 'Aircraft profile bypassed nested video token validation.' }
    Write-Host 'Named video strict nested validation and limits passed.'

    $config = Load-Config '{"RouterLocalPort":14620}'
    if ($config.RouterLocalPort -ne 14620) { throw 'Saved Mission Planner router endpoint changed.' }
    foreach ($field in @('UnknownSetting', 'UnknownVideoSetting')) {
        Assert-ThrowsMessage ('{"' + $field + '":null}') $field
    }
    $export = Join-Path $temporary 'export.json'
    $config.ExportToFile($export)
    $configCopy = $loadMethod.Invoke($null, [object[]]@([string]$export)).Clone()
    if ($configCopy.RouterLocalPort -ne 14620) { throw 'Configuration export/load/clone lost the router port.' }
    $videoExport = Join-Path $temporary 'video-export.json'
    $video.ExportToFile($videoExport)
    $videoCopy = $loadMethod.Invoke($null, [object[]]@([string]$videoExport)).Clone()
    if ($videoCopy.SelectedVideoInput.Url -ne $video.SelectedVideoInput.Url) { throw 'Video export/load/clone lost the source.' }
    Assert-ThrowsMessage '{"RouterLocalPort":0}' 'ports'
    $input = '{"Id":"one","Name":"Control","Device":"test","Source":"Button","Action":"a:activate","Enabled":true,"Button":0}'
    foreach ($button in @(0, 127)) {
        $mapped = Load-Config ('{"Inputs":[' + $input.Replace('"Button":0', '"Button":' + $button) + ']}')
        if ($mapped.Inputs[0]['Button'].Value -ne $button) { throw 'HID button boundary was changed.' }
    }
    foreach ($invalid in @($input.Replace('"Button":0', '"Button":128'),
        $input.Replace('"Button":0', '"Button":0.5'), $input.Replace('"Enabled":true', '"Enabled":"true"'),
        $input.Replace('"Source":"Button"', '"Source":"POV"'), $input.Replace('"Action":"a:activate"', '"Action":"a:arm"'),
        $input.Replace('"Button":0', '"button":0'), $input.Replace('"Button":0', '"Deadzone":0.51'),
        $input.Replace('"Action":"a:activate"', '"Action":"gimbal:pitch"'))) {
        $rejected = $false
        try { Load-Config ('{"Inputs":[' + $invalid + ']}') | Out-Null } catch { $rejected = $true }
        if (!$rejected) { throw 'Invalid nested input was accepted.' }
    }
    Assert-ThrowsMessage ('{"Inputs":[' + $input + ',' + $input.Replace('"Id":"one"','"Id":"two"') + ']}') 'unique physical'
    Assert-ThrowsMessage ('{"Inputs":[' + $input + ',' + $input + ']}') 'identities'
    Assert-ThrowsMessage '{"Inputs":null}' 'array'
    Assert-ThrowsMessage '{"Inputs":{}}' 'array'
    $optional = Load-Config ('{"Inputs":[' + $input.Replace('"Source":"Button"', '"Source":"ButtonPair"') + ']}')
    if ($optional.Inputs.Count -ne 1) { throw 'Single mapped switch endpoint was rejected.' }
    Assert-ThrowsMessage '{"RouterLocalPort":14611,"RouterLocalPort":14612}' 'already exists'
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
    $config.Inputs = [Newtonsoft.Json.Linq.JArray]::Parse('[' + $input.Replace('"Device":"test"', '"Device":"selected-unplugged-usb-hid"') + ']')
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
        $editorType = $plugin.GetType('NOMAD.MissionPlanner.InputBindingEditorForm', $true)
        $editor = $editorType.GetConstructors([Reflection.BindingFlags]'Instance,NonPublic')[0].Invoke([object[]]@($form.Config, [int]0))
        try {
            $device = $editorType.GetField('_device', $privateFlags).GetValue($editor)
            if ($device.SelectedItem.Key -ne 'selected-unplugged-usb-hid') { throw 'Unplugged selected HID name was discarded.' }
            $editorType.GetMethod('RefreshDevices', $privateFlags).Invoke($editor, @()) | Out-Null
            if ($device.SelectedItem.Key -ne 'selected-unplugged-usb-hid') { throw 'Device refresh discarded an explicit HID name.' }
        } finally { $editor.Dispose() }

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
