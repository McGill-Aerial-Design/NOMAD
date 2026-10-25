# SPDX-License-Identifier: Apache-2.0
# Render real plugin controls with example data; never load the operator profile or start the plugin.
param([string]$OutputDirectory = 'docs/images')
$ErrorActionPreference = 'Stop'
Import-Module (Join-Path $PSHOME 'Modules/Microsoft.PowerShell.Utility/Microsoft.PowerShell.Utility.psd1') -ErrorAction Stop
if ([Threading.Thread]::CurrentThread.ApartmentState -ne 'STA') {
    throw 'Run with Windows PowerShell: powershell -STA -NoProfile -File scripts/docs/capture_screenshots.ps1'
}
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$output = [IO.Path]::GetFullPath((Join-Path $root $OutputDirectory))
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$pluginPath = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'
if (-not (Test-Path -LiteralPath $pluginPath)) { throw 'Build the plugin first: pixi run build-plugin-only' }
$attempt = Join-Path $root ('build/screenshots/' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $output, $attempt -Force | Out-Null
[AppDomain]::CurrentDomain.add_AssemblyResolve([ResolveEventHandler] {
    param($sender, $request)
    $name = ([Reflection.AssemblyName]$request.Name).Name
    $path = Join-Path $hostRoot ($name + '.dll')
    if (Test-Path -LiteralPath $path) { return [Reflection.Assembly]::LoadFrom($path) }
    return $null
})
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
. (Join-Path $PSScriptRoot 'render_controls.ps1')
[Windows.Forms.Application]::EnableVisualStyles()
[Reflection.Assembly]::LoadFrom((Join-Path $hostRoot 'MissionPlanner.exe')) | Out-Null
[Reflection.Assembly]::LoadFrom((Join-Path $hostRoot 'MissionPlanner.Utilities.dll')) | Out-Null
$plugin = [Reflection.Assembly]::LoadFrom($pluginPath)
[MissionPlanner.Utilities.Settings]::CustomUserDataDirectory = $attempt
[MissionPlanner.Utilities.Settings]::AppConfigName = 'NOMAD-Documentation'
[MissionPlanner.Utilities.Settings]::FileName = Join-Path $attempt 'host.xml'
$private = [Reflection.BindingFlags]'Instance,NonPublic'
$plugin.GetType('NOMAD.MissionPlanner.Log').GetField('_logPath',
    [Reflection.BindingFlags]'Static,NonPublic').SetValue($null, (Join-Path $attempt 'nomad.log'))

function New-PluginObject([string]$name) {
    return [Activator]::CreateInstance($plugin.GetType('NOMAD.MissionPlanner.' + $name, $true))
}
$capturedPaths = [Collections.Generic.List[string]]::new()
function Capture([Windows.Forms.Control]$control, [string]$filename) {
    $imagePath = Join-Path $output $filename
    Save-ControlImage $control $imagePath
    $capturedPaths.Add($imagePath)
}

# An unstarted shared service prevents dashboard construction from starting monitoring.
$notifications = New-PluginObject 'NotificationService'
$outputType = $plugin.GetType('NOMAD.MissionPlanner.OutputController', $true)
$payloadType = $plugin.GetType('NOMAD.MissionPlanner.PayloadController', $true)
$static = [Reflection.BindingFlags]'Static,NonPublic'
$connectionField = $outputType.GetField('CurrentConnection', $static)
$connectionField.SetValue($null, [Management.Automation.LanguagePrimitives]::ConvertTo(
    { return $null }, $connectionField.FieldType))
$notificationType = $notifications.GetType()
$previousNotifications = $notificationType.GetProperty('Shared').GetValue($null)
$notificationType.GetProperty('Shared').SetValue($null, $notifications)
try {
    foreach ($view in @(@('Dashboard', 'dashboard.png'), @('Boundaries', 'fence-setup.png'))) {
        $config = New-PluginObject 'NOMADConfig'
        $config.AircraftName = 'Example Copter'
        $config.AutoStartHudVideo = $config.AudioAlerts = $config.AltitudeCallouts = $false
        $screenType = $plugin.GetType('NOMAD.MissionPlanner.NOMADMainScreen')
        if ($view[0] -eq 'Boundaries') {
            $input = New-PluginObject 'FenceSetupInput'
            $coordinates = "45.00, -73.00`r`n45.01, -73.00`r`n45.01, -73.02`r`n45.00, -73.02"
            $input.GetType().GetField('Coordinates', $private).SetValue($input, $coordinates)
            $screenType.GetMethod('SetFenceSetupInput', [Reflection.BindingFlags]'Static,NonPublic').Invoke(
                $null, [object[]]@($input)) | Out-Null
        }
        $screen = $screenType.GetConstructors() | Where-Object { $_.GetParameters().Count -eq 2 } |
            ForEach-Object { $_.Invoke([object[]]@($config, $null)) }
        $screenType.GetMethod('ShowView', $private).Invoke($screen, [object[]]@($view[0])) | Out-Null
        Capture $screen $view[1]
    }
    $config = New-PluginObject 'NOMADConfig'
    $config.AircraftName = 'Example Copter'
    $config.AutoStartHudVideo = $config.AudioAlerts = $config.AltitudeCallouts = $false
    foreach ($camera in @(@('Edge T3 receiver (example)', 'rtsp://camera.example.invalid:8554/live'),
        @('Jetson / Tailscale (example)', 'rtsp://jetson.example.invalid:8554/camera'))) {
        $input = New-PluginObject 'VideoInputProfile'
        $input.Name = $camera[0]; $input.Url = $camera[1]
        $config.VideoInputs.Add($input)
    }
    $config.SelectedVideoInputName = $config.VideoInputs[0].Name
    $link = New-PluginObject 'LinkConfig'
    $link.Id = 'example-elrs'; $link.Name = 'ELRS telemetry'; $link.Port = 14550
    $link.BindAddress = '127.0.0.1'
    $config.RouterLinks.Add($link)
    $link = New-PluginObject 'LinkConfig'
    $link.Id = 'example-lte'; $link.Name = 'LTE telemetry'; $link.Transport = 'TCP'
    $link.RemoteHost = 'relay.example.invalid'; $link.Port = 5760; $link.Priority = 1
    $config.RouterLinks.Add($link)
    $config.RouterPreferredLink = 'example-elrs'
    $config.ActuatorDefinitions = [Newtonsoft.Json.Linq.JArray]::Parse('[{"id":"light","name":"Inspection light","behavior":"RelayToggle","channel":0,"safe_pwm":1000,"hazardous":false,"confirmation_count":0}]')
    foreach ($definition in @(@('light-switch', 'Inspection light switch', 'AxisSwitch3', 'Rz', 'light:activate', 'light:safe'),
        @('gimbal-pitch', 'Gimbal pitch stick', 'Axis', 'Y', 'gimbal:pitch', ''))) {
        $binding = New-PluginObject 'InputBinding'
        $binding.Id = $definition[0]; $binding.Name = $definition[1]; $binding.Device = 'Example USB controller'
        $binding.Source = $definition[2]; $binding.Axis = $definition[3]
        $binding.Action = $definition[4]; $binding.OtherAction = $definition[5]
        $config.Inputs.Add([Newtonsoft.Json.Linq.JObject]::FromObject($binding))
    }
    foreach ($tab in @(@('Aircraft', 'aircraft-profiles.png'), @('Links', 'router-settings.png'),
        @('Video', 'video-profiles.png'), @('Inputs', 'inputs-settings.png'))) {
        $profilesType = $plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles')
        $profiles = $profilesType.GetConstructors() | Where-Object { $_.GetParameters().Count -eq 1 } |
            ForEach-Object { $_.Invoke([object[]]@($config)) }
        $settingsType = $plugin.GetType('NOMAD.MissionPlanner.NOMADSettingsForm')
        $settings = $settingsType.GetConstructors()[0].Invoke([object[]]@($profiles))
        $tabs = $settingsType.GetField('_tabControl', $private).GetValue($settings)
        $page = @($tabs.TabPages | Where-Object { $_.Text -eq $tab[0] })
        if ($page.Count -ne 1) { throw ('Expected one settings page: ' + $tab[0]) }
        $tabs.SelectedTab = $page[0]
        Capture $settings $tab[1]
    }
    $editorType = $plugin.GetType('NOMAD.MissionPlanner.InputBindingEditorForm', $true)
    $editor = [Activator]::CreateInstance($editorType, [Reflection.BindingFlags]'Instance,NonPublic',
        $null, [object[]]@($config, [int]0), $null)
    Capture $editor 'input-editor.png'
    $outputType.GetMethod('Initialize', $static).Invoke($null, [object[]]@($config)) | Out-Null
    $payloadType.GetMethod('ApplyProfile', $static).Invoke($null, [object[]]@($config)) | Out-Null
    $selectMethod = $screenType.GetMethod('SetVideoInputSelection', $static)
    $selectCallback = [Management.Automation.LanguagePrimitives]::ConvertTo(
        {
            param($draft, $name)
            $draft.SelectedVideoInputName = $name
            return $true
        }, $selectMethod.GetParameters()[0].ParameterType)
    $selectMethod.Invoke($null, [object[]]@($selectCallback)) | Out-Null
    $videoType = $plugin.GetType('NOMAD.MissionPlanner.NOMADVideoView', $true)
    $video = [Activator]::CreateInstance($videoType, [Reflection.BindingFlags]'Instance,NonPublic',
        $null, [object[]]@($config, [Threading.CancellationToken]::None), $null)
    Capture $video 'video-page.png'
} finally {
    $payloadType.GetMethod('Shutdown', $static).Invoke($null, $null) | Out-Null
    $outputType.GetMethod('Shutdown', $static).Invoke($null, $null) | Out-Null
    $screenType.GetMethod('SetVideoInputSelection', $static).Invoke($null, [object[]]@($null)) | Out-Null
    $screenType.GetMethod('SetFenceSetupInput', [Reflection.BindingFlags]'Static,NonPublic').Invoke(
        $null, [object[]]@($null)) | Out-Null
    $notificationType.GetProperty('Shared').SetValue($null, $previousNotifications)
    $notifications.Dispose()
}
$capturedPaths | ForEach-Object {
    $image = [Drawing.Image]::FromFile($_)
    try {
        if ($image.Width -lt 600 -or $image.Height -lt 500) { throw ('Incomplete capture: ' + $_) }
    } finally { $image.Dispose() }
}
[ordered]@{ plugin_sha256 = (Get-FileHash $pluginPath).Hash; example_data = $true;
    aircraft_connected = $false; capture = 'WinForms DrawToBitmap without activation'; images = @($capturedPaths |
        ForEach-Object { [ordered]@{ name = [IO.Path]::GetFileName($_); sha256 = (Get-FileHash $_).Hash } }) } |
    ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $attempt 'captures.json')
Write-Output ('Captured NOMAD screens to ' + $output)
