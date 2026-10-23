# SPDX-License-Identifier: Apache-2.0
# Render real plugin controls with example data; never load the operator profile or start the plugin.
param([string]$OutputDirectory = 'docs/images')
$ErrorActionPreference = 'Stop'
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
function Capture([Windows.Forms.Control]$control, [string]$filename) {
    $form = if ($control -is [Windows.Forms.Form]) { $control } else {
        $hostForm = [Windows.Forms.Form]::new()
        $hostForm.ClientSize = [Drawing.Size]::new(1180, 760)
        $hostForm.Controls.Add($control)
        $hostForm
    }
    try {
        $form.StartPosition = 'Manual'
        $form.Location = [Drawing.Point]::new(-20000, -20000)
        $form.ShowInTaskbar = $false
        $form.Show()
        $form.PerformLayout()
        [Windows.Forms.Application]::DoEvents()
        $bitmap = [Drawing.Bitmap]::new($form.ClientSize.Width, $form.ClientSize.Height)
        try {
            # Capture only the client area; no desktop pixels, unrelated windows or synthetic UI artwork.
            $full = [Drawing.Bitmap]::new($form.Width, $form.Height)
            try {
                $form.DrawToBitmap($full, [Drawing.Rectangle]::new(0, 0, $form.Width, $form.Height))
                $client = $form.PointToScreen([Drawing.Point]::Empty)
                $graphics = [Drawing.Graphics]::FromImage($bitmap)
                try { $graphics.DrawImage($full, $form.Left - $client.X, $form.Top - $client.Y) }
                finally { $graphics.Dispose() }
            } finally { $full.Dispose() }
            $bitmap.Save((Join-Path $output $filename), [Drawing.Imaging.ImageFormat]::Png)
        } finally { $bitmap.Dispose() }
    } finally { $form.Dispose() }
}

# An unstarted shared service prevents dashboard construction from starting monitoring.
$notifications = New-PluginObject 'NotificationService'
$notificationType = $notifications.GetType()
$previousNotifications = $notificationType.GetProperty('Shared').GetValue($null)
$notificationType.GetProperty('Shared').SetValue($null, $notifications)
try {
    foreach ($view in @(@('Dashboard', 'dashboard.png'), @('Boundaries', 'fence-setup.png'))) {
        $config = New-PluginObject 'NOMADConfig'
        $config.AircraftName = 'Demo Copter'
        $config.AutoStartHudVideo = $config.AudioAlerts = $config.AltitudeCallouts = $false
        $screenType = $plugin.GetType('NOMAD.MissionPlanner.NOMADMainScreen')
        if ($view[0] -eq 'Boundaries') {
            $input = New-PluginObject 'FenceSetupInput'
            $coordinates = $plugin.GetType('NOMAD.MissionPlanner.FenceCoordinates').GetField('PublishedC1',
                [Reflection.BindingFlags]'Static,NonPublic,Public').GetValue($null)
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
    $config.AircraftName = 'Demo Copter'
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
    foreach ($tab in @(@(0, 'aircraft-profiles.png'), @(1, 'router-settings.png'), @(2, 'video-profiles.png'))) {
        $profilesType = $plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles')
        $profiles = $profilesType.GetConstructors() | Where-Object { $_.GetParameters().Count -eq 1 } |
            ForEach-Object { $_.Invoke([object[]]@($config)) }
        $settingsType = $plugin.GetType('NOMAD.MissionPlanner.NOMADSettingsForm')
        $settings = $settingsType.GetConstructors()[0].Invoke([object[]]@($profiles))
        $settingsType.GetField('_tabControl', $private).GetValue($settings).SelectedIndex = $tab[0]
        Capture $settings $tab[1]
    }
} finally {
    $screenType.GetMethod('SetFenceSetupInput', [Reflection.BindingFlags]'Static,NonPublic').Invoke(
        $null, [object[]]@($null)) | Out-Null
    $notificationType.GetProperty('Shared').SetValue($null, $previousNotifications)
    $notifications.Dispose()
}
Get-ChildItem -LiteralPath $output -Filter '*.png' | ForEach-Object {
    $image = [Drawing.Image]::FromFile($_.FullName)
    try {
        if ($image.Width -lt 700 -or $image.Height -lt 600) { throw ('Incomplete capture: ' + $_.Name) }
    } finally { $image.Dispose() }
}
[ordered]@{ plugin_sha256 = (Get-FileHash $pluginPath).Hash; example_data = $true;
    aircraft_connected = $false; capture = 'WinForms DrawToBitmap'; images = @(Get-ChildItem $output -Filter '*.png' |
        ForEach-Object { [ordered]@{ name = $_.Name; sha256 = (Get-FileHash $_.FullName).Hash } }) } |
    ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $attempt 'captures.json')
Write-Output ('Captured five NOMAD screens to ' + $output)
