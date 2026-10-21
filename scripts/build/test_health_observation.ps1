# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Called by the configuration harness; no timers, host constructors or devices are started.
function Invoke-HealthObservationTests {
    param([Reflection.Assembly]$Plugin)
    $private = [Reflection.BindingFlags]'Instance,NonPublic'
    $serviceType = $Plugin.GetType('NOMAD.MissionPlanner.NotificationService', $true)
    $service = [Activator]::CreateInstance($serviceType, [object[]]@($null))
    try {
        $ekf = $serviceType.GetMethod('CheckEKFPositionValidity', $private)
        $ekf.Invoke($service, [object[]]@([int]0x10)) | Out-Null
        $ekf.Invoke($service, [object[]]@([int]0x110)) | Out-Null
        if ($service.Notifications.Count -ne 0) { throw 'Initial/unrelated EKF flags produced a false transition.' }
        $ekf.Invoke($service, [object[]]@([int]0x08)) | Out-Null
        $ekf.Invoke($service, [object[]]@([int]0)) | Out-Null
        $loss = $service.Notifications | Where-Object { $_.Severity.ToString() -eq 'Critical' }
        if ($service.Notifications.Count -ne 2 -or $null -eq $loss -or
            ($service.Notifications.Message -join ' ') -match 'GPS|VIO|OptFlow') {
            throw 'EKF validity used sensor provenance or suppressed critical loss behind warning cooldown.'
        }
    } finally { $service.Dispose() }

    $service = [Activator]::CreateInstance($serviceType, [object[]]@($null))
    try {
        $context = $serviceType.GetMethod('ObserveHealthContext', $private)
        $first = New-Object object[] 3
        $second = New-Object object[] 3
        for ($index = 0; $index -lt 3; $index++) {
            $type = $context.GetParameters()[$index].ParameterType
            $first[$index] = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($type)
            $second[$index] = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($type)
            [GC]::SuppressFinalize($first[$index]); [GC]::SuppressFinalize($second[$index])
        }
        $selected = [object[]]$first.Clone()
        $context.Invoke($service, $selected) | Out-Null
        $severity = [Enum]::Parse($Plugin.GetType('NOMAD.MissionPlanner.NotificationSeverity'), 'Warning')
        $category = [Enum]::Parse($Plugin.GetType('NOMAD.MissionPlanner.NotificationCategory'), 'GPS')
        for ($index = 0; $index -lt 3; $index++) {
            foreach ($name in @('_lastGpsFix','_lastEkfPositionValidity','_altBand')) {
                $serviceType.GetField($name, $private).SetValue($service, [int]3)
            }
            $serviceType.GetField('_lastBatterySeverity', $private).GetValue($service).Add([int]1, [int]1)
            $serviceType.GetMethod('CanSpeakBattery', $private).Invoke($service, [object[]]@([int]1)) | Out-Null
            $service.AddNotification($severity, $category, 'Synthetic health warning', 'Fixture observation')
            $before = $service.Notifications.Count
            $context.Invoke($service, $selected) | Out-Null
            if ($serviceType.GetField('_altBand', $private).GetValue($service) -ne 3) {
                throw 'Unchanged health context reset a live altitude band.'
            }
            $selected[$index] = $second[$index]
            $context.Invoke($service, $selected) | Out-Null
            foreach ($name in @('_lastGpsFix','_lastEkfPositionValidity','_altBand')) {
                if ($serviceType.GetField($name, $private).GetValue($service) -ne -1) {
                    throw 'Changed native connection/MAV/state retained another aircraft baseline.'
                }
            }
            foreach ($name in @('_lastBatterySeverity','_lastBatterySpeechUtc','_lastNotificationTime')) {
                if ($serviceType.GetField($name, $private).GetValue($service).Count -ne 0) {
                    throw 'Changed native context retained battery state or notification cooldown.'
                }
            }
            $service.AddNotification($severity, $category, 'Synthetic health warning', 'New fixture observation')
            if ($service.Notifications.Count -ne $before + 1) {
                throw 'Context reset erased history or suppressed the next aircraft warning.'
            }
        }
        $context.Invoke($service, [object[]]@($null, $null, $null)) | Out-Null
        if ($serviceType.GetField('_lastNotificationTime', $private).GetValue($service).Count -ne 0) {
            throw 'Disconnected health context retained notification cooldown.'
        }
    } finally { $service.Dispose() }

    $units = $Plugin.GetType('NOMAD.MissionPlanner.AltitudeUnits', $true).GetMethod(
        'ToMeters', [Reflection.BindingFlags]'Static,NonPublic')
    $rangeCheck = $serviceType.GetMethod('CheckOpticalFlowHealth', $private)
    $stateType = $rangeCheck.GetParameters()[0].ParameterType
    $state = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($stateType)
    [GC]::SuppressFinalize($state)
    $scale = $stateType.GetField('multiplieralt')
    $originalScale = $scale.GetValue($null)
    $configType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADConfig', $true)
    $boundaryType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADBoundaryView', $true)
    $boundary = [Runtime.Serialization.FormatterServices]::GetUninitializedObject($boundaryType)
    $altitudeLabel = New-Object System.Windows.Forms.Label
    $boundaryType.GetField('_lblAltitude', $private).SetValue($boundary, $altitudeLabel)
    $outlines = [Activator]::CreateInstance($Plugin.GetType('NOMAD.MissionPlanner.GeofenceConfig', $true))
    $outlines.AdvisoryAltitudeDisplayThresholdMeters = 100
    $boundaryType.GetField('_missionConfig', $private).SetValue($boundary, $outlines)
    try {
        foreach ($multiplier in @([single]1, [single]3.28084)) {
            $scale.SetValue($null, $multiplier)
            foreach ($physicalMeters in @([single]50, [single]100)) {
                $state.alt = $physicalMeters
                $meters = $units.Invoke($null, [object[]]@([single]$state.alt, [single]$multiplier))
                if ($meters -ne $physicalMeters) { throw 'Native float display units moved an exact meter boundary.' }
                $outlines.AdvisoryAltitudeDisplayThresholdMeters = $physicalMeters
                $boundaryType.GetMethod('UpdateAdvisoryAltitude', $private).Invoke(
                    $boundary, [object[]]@($meters)) | Out-Null
                if ($altitudeLabel.Text -notmatch ('{0}[.,]0m / {0}m' -f [int]$physicalMeters) -or
                    $altitudeLabel.ForeColor -ne [Drawing.Color]::White) {
                    throw 'Host display units changed an exact advisory meter boundary or threshold verdict.'
                }
            }
            $service = [Activator]::CreateInstance($serviceType, [object[]]@($null))
            try {
                $state.sonarrange = [single]50
                $rangeCheck.Invoke($service, [object[]]@($state)) | Out-Null
                if ($service.Notifications.Count -ne 0) { throw 'Display feet caused a false rangefinder warning.' }
                $state.sonarrange = [single]100
                $rangeCheck.Invoke($service, [object[]]@($state)) | Out-Null
                if ($service.Notifications.Count -ne 1 -or $service.Notifications[0].Message -notmatch '100[.,]0m') {
                    throw 'Rangefinder threshold or label used display units instead of meters.'
                }
            } finally { $service.Dispose() }
        }
    } finally { $scale.SetValue($null, $originalScale); $altitudeLabel.Dispose() }

    $dashboardType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADDashboardView', $true)
    $shared = $serviceType.GetProperty('Shared')
    $previousShared = $shared.GetValue($null)
    $service = [Activator]::CreateInstance($serviceType, [object[]]@($null))
    $shared.SetValue($null, $service)
    $view = $null
    try {
        $view = $dashboardType.GetConstructor([type[]]@($configType,
            $Plugin.GetType('NOMAD.MissionPlanner.MAVLinkConnectionManager'))).Invoke(
                [object[]]@([Activator]::CreateInstance($configType), $null))
        $dashboardType.GetMethod('UpdateBatteryCard', $private).Invoke($view, [object[]]@($null)) | Out-Null
        $label = $dashboardType.GetField('_lblBattery', $private).GetValue($view)
        $neutral = $Plugin.GetType('NOMAD.MissionPlanner.NOMADTheme').GetField('TEXT_SECONDARY').GetValue($null)
        if ($label.Text -ne 'Battery unavailable' -or $label.ForeColor -ne $neutral) {
            throw 'Unavailable battery telemetry displayed a healthy numeric reading or green verdict.'
        }
    } finally { if ($null -ne $view) { $view.Dispose() }; $shared.SetValue($null, $previousShared); $service.Dispose() }
    Write-Host 'Native health validity, context/cooldown isolation, meter units and unavailable battery checks passed.'
}
