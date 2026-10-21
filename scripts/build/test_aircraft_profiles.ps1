# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Uses the configuration fixture's isolated directory, loaded plugin and temporary logger.
function Invoke-AircraftProfileTests {
    param($Plugin, $ConfigType, [string]$Temporary)

    $profilesType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    $loadProfiles = $profilesType.GetMethod('LoadFromPaths', [Reflection.BindingFlags]'Static,NonPublic')
    $directory = Join-Path $Temporary 'aircraft-profiles'
    New-Item -ItemType Directory -Path $directory | Out-Null
    $primary = Join-Path $directory 'profiles.json'
    $backup = $primary + '.bak'
    $legacy = Join-Path $directory 'legacy.json'
    $legacyText = '{"CoreRuntimePort":14641,"CoreClientCredential":"fixture-only-alpha-key"}'
    [IO.File]::WriteAllText($legacy, $legacyText)
    function Load-Profiles {
        $loadArguments = New-Object object[] 4
        $loadArguments[0] = [string]$primary
        $loadArguments[1] = [string]$backup
        $loadArguments[2] = [string]$legacy
        $loadArguments[3] = [string]($legacy + '.bak')
        return $loadProfiles.Invoke($null, $loadArguments)
    }
    $profiles = Load-Profiles
    $alpha = $profiles.Selected
    if ($null -ne $alpha.Config.RcArmingChannel -or $alpha.Config.FlightControllerModel -ne '') {
        throw 'Existing configurations acquired invented hardware/RC defaults.'
    }
    $alpha.Config.AircraftName = 'Profile Alpha'
    $alpha.Config.FlightControllerModel = 'Controller Alpha'
    $alpha.Config.ReceiverType = 'Receiver Alpha'
    $alpha.Config.RcArmingChannel = 5
    $alpha.Config.RcFlightModeChannel = 6
    $alpha.Config.JoystickGimbalMaxRateDegSec = 20
    $alpha.Config.GimbalMountMode = 2
    $alpha.Config.ActuatorDefinitions = [Newtonsoft.Json.Linq.JArray]::Parse(
        '[{"id":"alpha_output","name":"Alpha Output","behavior":"RelayToggle","channel":0,"safe_pwm":1000}]')
    $beta = Load-Config ('{"AircraftName":"Profile Beta","FlightControllerModel":"Controller Beta",' +
        '"ReceiverType":"Receiver Beta","AircraftFirmware":"fixture-version","Airframe":"fixture-airframe",' +
        '"RcArmingChannel":7,"RcFlightModeChannel":8,"CoreRuntimePort":14642,' +
        '"RouterLocalPort":14643,"ManagementPort":14644,"CoreClientCredential":"fixture-only-beta-key",' +
        '"VideoUrl":"rtsp://profile-beta.invalid/stream","AutoStartHudVideo":false,' +
        '"JoystickGimbalMaxRateDegSec":80,"GimbalMountMode":3,"JoystickGimbalPitchInvert":false,' +
        '"ActuatorDefinitions":[{"id":"beta_output","name":"Beta Output","behavior":"RelayToggle","channel":1,"safe_pwm":1000}]}')
    $added = $profiles.Add($beta)
    $profiles.Save()
    $reopened = Load-Profiles
    $restored = $reopened.Selected.Config
    if ($reopened.Aircraft.Count -ne 2 -or $reopened.SelectedId -ne $added.Id -or
        $restored.AircraftName -ne 'Profile Beta' -or $restored.FlightControllerModel -ne 'Controller Beta' -or
        $restored.ReceiverType -ne 'Receiver Beta' -or $restored.AircraftFirmware -ne 'fixture-version' -or
        $restored.Airframe -ne 'fixture-airframe' -or $restored.RcArmingChannel -ne 7 -or
        $restored.RcFlightModeChannel -ne 8) {
        throw 'Restart did not restore the selected aircraft and its hardware/RC values.'
    }
    if ($restored.RouterLocalPort -ne 14643 -or $restored.SelectedVideoInput.Url -ne 'rtsp://profile-beta.invalid/stream' -or
        $restored.AutoStartHudVideo -or $restored.JoystickGimbalMaxRateDegSec -ne 80 -or
        $restored.GimbalMountMode -ne 3 -or $restored.JoystickGimbalPitchInvert -or
        $restored.ActuatorDefinitions.Count -ne 1 -or
        $restored.ActuatorDefinitions[0]['id'].ToString() -ne 'beta_output') {
        throw 'Restart mixed another aircraft''s connections, gimbal or actuator definitions into the selected profile.'
    }
    if ([IO.File]::ReadAllText($legacy) -cne $legacyText) {
        throw 'Loading legacy configuration rewrote operator files.'
    }
    $clone = $reopened.Clone()
    $clone.Selected.Config.ActuatorDefinitions[0]['name'] = [Newtonsoft.Json.Linq.JValue]::new('Edited draft')
    $clone.Selected.Config.RouterLocalPort = 14652
    if ($reopened.Selected.Config.RouterLocalPort -ne 14643 -or
        $reopened.Selected.Config.ActuatorDefinitions[0]['name'].ToString() -ne 'Beta Output') {
        throw 'Editing a draft profile mutated the live profile collection.'
    }
    $portable = Join-Path $directory 'portable.json'
    $restored.ExportToFile($portable)
    $portableText = [IO.File]::ReadAllText($portable)
    if ($portableText.Contains('CoreClientCredential') -or $portableText.Contains('fixture-only-') -or
        $portableText.Contains('Profile Alpha')) {
        throw 'Single-aircraft export leaked credentials or another aircraft configuration.'
    }
    $imported = $ConfigType.GetMethod('LoadFromFile').Invoke($null, [object[]]@([string]$portable))
    if ($imported.AircraftName -ne 'Profile Beta' -or
        $imported.GimbalMountMode -ne 3 -or $imported.RcFlightModeChannel -ne 8 -or
        $imported.ActuatorDefinitions[0]['id'].ToString() -ne 'beta_output') {
        throw 'Portable aircraft configuration did not restore independently of local credentials.'
    }
    $profiles.SelectedId = $alpha.Id.ToUpperInvariant()
    $profiles.Save()
    $selectedAlpha = Load-Profiles
    if ($selectedAlpha.Selected.Config.AircraftName -ne 'Profile Alpha' -or
        $selectedAlpha.Selected.Config.RcArmingChannel -ne 5 -or
        $selectedAlpha.Selected.Config.JoystickGimbalMaxRateDegSec -ne 20 -or
        $selectedAlpha.Selected.Config.ActuatorDefinitions[0]['id'].ToString() -ne 'alpha_output') {
        throw 'Changing persisted selection did not restore the other aircraft independently.'
    }
    $beforeGuardPrimary = [IO.File]::ReadAllText($primary)
    $beforeGuardBackup = [IO.File]::ReadAllText($backup)
    $saveForConnection = $profilesType.GetMethod('SaveForConnection', [Reflection.BindingFlags]'Instance,NonPublic')
    $draft = $selectedAlpha.Clone()
    $draft.Selected.Config.AudioAlerts = -not $draft.Selected.Config.AudioAlerts
    $changed = $saveForConnection.Invoke($draft, [object[]]@($selectedAlpha, $true))
    if ($changed) { throw 'Audio-only Save requested a router restart.' }
    $beforeRefusedSave = [IO.File]::ReadAllText($primary)
    $changedDraft = $draft.Clone()
    $changedDraft.Selected.Config.RouterLocalPort = 14670
    $rejected = $false
    try { $saveForConnection.Invoke($changedDraft, [object[]]@($draft, $true)) | Out-Null } catch { $rejected = $true }
    if (-not $rejected -or [IO.File]::ReadAllText($primary) -cne $beforeRefusedSave) {
        throw 'Connected router change persisted a refused aircraft mapping.'
    }
    foreach ($setting in @('definition', 'input')) {
        $outputDraft = $draft.Clone()
        if ($setting -eq 'definition') {
            $outputDraft.Selected.Config.ActuatorDefinitions[0]['channel'] = [Newtonsoft.Json.Linq.JValue]::new(2)
        } else {
            $outputDraft.Selected.Config.JoystickGimbalDeadzone = 0.2
        }
        $rejected = $false
        try { $saveForConnection.Invoke($outputDraft, [object[]]@($draft, $true)) | Out-Null } catch { $rejected = $true }
        if (-not $rejected -or [IO.File]::ReadAllText($primary) -cne $beforeRefusedSave) {
            throw 'Connected output/input calibration change persisted before disconnect.'
        }
    }
    $payloadType = $Plugin.GetType('NOMAD.MissionPlanner.PayloadController', $true)
    $payloadType.GetMethod('ApplyProfile', [Reflection.BindingFlags]'Static,NonPublic').Invoke($null, [object[]]@($draft.Selected.Config)) | Out-Null
    $payload = $payloadType.GetProperty('Current', [Reflection.BindingFlags]'Static,NonPublic').GetValue($null)
    $busyField = $payloadType.GetField('_busy', [Reflection.BindingFlags]'Instance,NonPublic')
    $busyField.SetValue($payload, $true)
    try {
        $rejected = $false
        try { $saveForConnection.Invoke($draft.Clone(), [object[]]@($draft, $true)) | Out-Null } catch { $rejected = $true }
        if (-not $rejected -or [IO.File]::ReadAllText($primary) -cne $beforeRefusedSave) {
            throw 'Settings persisted while a native payload operation could lose its captured Safe target.'
        }
    } finally {
        $busyField.SetValue($payload, $false)
        $payloadType.GetMethod('Shutdown', [Reflection.BindingFlags]'Static,NonPublic').Invoke($null, @()) | Out-Null
    }
    $selectedDraft = $draft.Clone()
    $selectedDraft.SelectedId = $added.Id
    $rejected = $false
    try { $saveForConnection.Invoke($selectedDraft, [object[]]@($draft, $true)) | Out-Null } catch { $rejected = $true }
    if (-not $rejected -or [IO.File]::ReadAllText($primary) -cne $beforeRefusedSave) {
        throw 'Connected aircraft selection persisted before disconnect.'
    }
    if (-not $saveForConnection.Invoke($changedDraft, [object[]]@($draft, $false))) {
        throw 'Disconnected router change did not request replacement.'
    }
    [IO.File]::WriteAllText($primary, $beforeGuardPrimary)
    [IO.File]::WriteAllText($backup, $beforeGuardBackup)
    $validPrimary = [IO.File]::ReadAllText($primary)
    $validBackup = [IO.File]::ReadAllText($backup)
    [IO.File]::Move($primary, $primary + '.held')
    try {
        if ((Load-Profiles).SelectedId -ne $added.Id -or [IO.File]::ReadAllText($backup) -cne $validBackup) {
            throw 'A missing profile primary did not use the retained backup without changing it.'
        }
    } finally { [IO.File]::Move($primary + '.held', $primary) }
    $invalidCases = @('', '{', ($validPrimary + '{}'), $validPrimary.Replace($added.Id, $alpha.Id),
        $validPrimary.Replace('Profile Beta', 'PROFILE ALPHA'),
        $validPrimary.Replace('"Aircraft":', '"Unknown":0,"Aircraft":'),
        '{"Aircraft":[],"SelectedId":"00000000000000000000000000000000"}',
        $validPrimary.Replace('"SelectedId": "' + $alpha.Id.ToUpperInvariant() + '"',
            '"SelectedId": "00000000000000000000000000000000"'))
    foreach ($invalid in $invalidCases) {
        if ($invalid -ceq $validPrimary) { throw 'Invalid-profile fixture did not change its subject.' }
        [IO.File]::WriteAllText($primary, $invalid)
        $rejected = $false
        try { Load-Profiles | Out-Null } catch { $rejected = $true }
        if (-not $rejected -or [IO.File]::ReadAllText($primary) -cne $invalid -or
            [IO.File]::ReadAllText($backup) -cne $validBackup -or [IO.File]::ReadAllText($legacy) -cne $legacyText) {
            throw 'Invalid profile storage fell back or changed existing configuration files.'
        }
    }
    [IO.File]::WriteAllText($primary, $validPrimary)
    $failedSave = Load-Profiles
    $failedSave.SelectedId = $added.Id
    New-Item -ItemType Directory -Path ($primary + '.tmp') | Out-Null
    $rejected = $false
    try { $failedSave.Save() } catch { $rejected = $true }
    if (-not $rejected -or [IO.File]::ReadAllText($primary) -cne $validPrimary -or
        [IO.File]::ReadAllText($backup) -cne $validBackup) {
        throw 'A failed profile save replaced the previous persisted selection or backup.'
    }
    [IO.Directory]::Delete($primary + '.tmp')
    foreach ($invalid in @('{"RcArmingChannel":0}', '{"RcFlightModeChannel":19}',
        '{"RcArmingChannel":"5"}', '{"rcArmingChannel":"5"}',
        '{"RcArmingChannel":null,"rcArmingChannel":"5"}',
        '{"RcFlightModeChannel":5.5}', '{"GimbalMountMode":4}',
        '{"ActuatorDefinitions":{}}', '{"ActuatorDefinitions":[1]}', '{"FlightControllerModel":3}',
        '{"JoystickGimbalPitchAxis":"unknown"}', '{"JoystickPositionAxis":null}',
        '{"JoystickGimbalMaxRateDegSec":"NaN"}', '{"JoystickPositionDeadzone":0.51}')) {
        $rejected = $false
        try { Load-Config $invalid | Out-Null } catch { $rejected = $true }
        if (-not $rejected) { throw 'Invalid aircraft hardware/configuration data was accepted.' }
    }
    $dateName = Load-Config '{"AircraftName":"2026-10-18T00:00:00Z","RcArmingChannel":null}'
    if ($dateName.AircraftName -cne '2026-10-18T00:00:00Z' -or $null -ne $dateName.RcArmingChannel) {
        throw 'A textual aircraft name or explicitly unconfigured RC channel changed during parsing.'
    }
    $edgeChannels = Load-Config '{"RcArmingChannel":1,"RcFlightModeChannel":18}'
    if ($edgeChannels.RcArmingChannel -ne 1 -or $edgeChannels.RcFlightModeChannel -ne 18) {
        throw 'The supported first or last RC input channel was rejected or changed.'
    }
    $filterType = $Plugin.GetType('NOMAD.MissionPlanner.GimbalArrowKeyFilter', $true)
    $filter = $filterType.GetConstructor([type[]]@($profilesType)).Invoke([object[]]@($selectedAlpha))
    try {
        $filterType.GetMethod('SetEnabled').Invoke($null, [object[]]@($true)) | Out-Null
        $afterToggle = Load-Profiles
        if (-not $afterToggle.Selected.Config.GimbalArrowKeysEnabled -or
            $afterToggle.Aircraft[1].Config.GimbalArrowKeysEnabled) {
            throw 'The live keyboard toggle did not persist only the active aircraft profile.'
        }
        New-Item -ItemType Directory -Path ($primary + '.tmp') | Out-Null
        $rejected = $false
        try { $filterType.GetMethod('SetEnabled').Invoke($null, [object[]]@($false)) | Out-Null }
        catch { $rejected = $true }
        if (-not $rejected -or -not $filterType.GetProperty('Enabled').GetValue($null, $null) -or
            -not (Load-Profiles).Selected.Config.GimbalArrowKeysEnabled) {
            throw 'A failed keyboard-toggle save changed live or persisted aircraft settings.'
        }
        [IO.Directory]::Delete($primary + '.tmp')
    } finally { $filter.Dispose() }
    Write-Host 'Aircraft profile persistence, isolation, export and storage fault checks passed.'
}

function Invoke-AircraftProfileUiTests {
    param($Plugin, $ConfigType)

    $instance = [Reflection.BindingFlags]'Instance,NonPublic'
    $static = [Reflection.BindingFlags]'Static,NonPublic'
    $profilesType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    $formType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADSettingsForm', $true)
    $outputType = $Plugin.GetType('NOMAD.MissionPlanner.OutputController', $true)
    $gimbalType = $Plugin.GetType('NOMAD.MissionPlanner.GimbalController', $true)
    $alpha = Load-Config ('{"AircraftName":"Alpha UI","CoreRuntimePort":14651,' +
        '"JoystickGimbalMaxRateDegSec":30,"FlightControllerModel":"Alpha controller"}')
    $arguments = New-Object object[] 1
    $arguments[0] = $alpha
    $profiles = $profilesType.GetConstructor([type[]]@($ConfigType)).Invoke($arguments)
    $alphaId = $profiles.SelectedId
    $beta = Load-Config ('{"AircraftName":"Beta UI","CoreRuntimePort":14652,"RouterLocalPort":14653,' +
        '"ManagementPort":14654,"VideoUrl":"rtsp://beta-ui.invalid/stream",' +
        '"FlightControllerModel":"Beta controller","ReceiverType":"Beta receiver",' +
        '"RcArmingChannel":7,"RcFlightModeChannel":8,"GimbalMountMode":3,"JoystickGimbalMaxRateDegSec":80,' +
        '"JoystickGimbalPitchAxis":"Rz","JoystickPositionEnabled":true,"JoystickPositionActuatorId":"beta",' +
        '"ActuatorDefinitions":[{"id":"beta","name":"Beta","behavior":"RelayToggle","channel":0,"safe_pwm":1000}]}')
    $betaId = $profiles.Add($beta).Id
    $profiles.SelectedId = $alphaId
    $outputType.GetMethod('Initialize', $static).Invoke($null, [object[]]@($alpha)) | Out-Null
    $initialGeneration = $outputType.GetProperty('Generation', $static).GetValue($null)
    $gimbalType.GetMethod('ApplyProfile', $static).Invoke($null, [object[]]@($alpha)) | Out-Null
    $gimbalType.GetMethod('SetTargetAngles').Invoke($null, [object[]]@([single]12, [single]18)) | Out-Null
    $arguments[0] = $profiles
    $form = $formType.GetConstructor([type[]]@($profilesType)).Invoke($arguments)
    function Profile-Control([string]$Name) {
        return $formType.GetField($Name, $instance).GetValue($form)
    }
    try {
        $selector = Profile-Control '_cmbAircraft'
        (Profile-Control '_txtFlightController').Text = 'Alpha draft controller'
        $selector.SelectedItem = $form.Profiles.Aircraft[1]
        if ($form.Profiles.SelectedId -ne $betaId -or
            (Profile-Control '_txtFlightController').Text -ne 'Beta controller' -or
            (Profile-Control '_txtReceiver').Text -ne 'Beta receiver' -or
            (Profile-Control '_cmbArmingChannel').SelectedIndex -ne 7 -or
            (Profile-Control '_cmbFlightModeChannel').SelectedIndex -ne 8 -or
            (Profile-Control '_numRouterLocalPort').Value -ne 14653 -or
            (Profile-Control '_txtVideoUrl').Text -ne 'rtsp://beta-ui.invalid/stream') {
            throw 'Aircraft selection did not restore the selected hardware, RC and connection controls.'
        }
        if ((Profile-Control '_cmbGimbalMountMode').SelectedIndex -ne 3 -or
            (Profile-Control '_numJoyGimbalMaxRate').Value -ne 80 -or
            (Profile-Control '_cmbJoyGimbalPitchAxis').SelectedItem -ne 'Rz' -or
            -not (Profile-Control '_chkJoyPositionEnabled').Checked -or
            (Profile-Control '_actuatorJson').Text -notmatch '"beta"') {
            throw 'Aircraft selection lost gimbal, actuator or unavailable-backend input settings.'
        }
        (Profile-Control '_numJoyGimbalMaxRate').Value = 90
        $selector.SelectedItem = $form.Profiles.Aircraft[0]
        if ((Profile-Control '_txtFlightController').Text -ne 'Alpha draft controller' -or
            (Profile-Control '_numJoyGimbalMaxRate').Value -ne 30) {
            throw 'Returning to an aircraft did not restore its independent draft.'
        }
        $formType.GetMethod('AddAircraft', $instance).Invoke($form, [object[]]@($true)) | Out-Null
        if ($form.Profiles.Aircraft.Count -ne 3 -or $form.Profiles.SelectedId -eq $alphaId -or
            $form.Config.FlightControllerModel -ne 'Alpha draft controller') {
            throw 'Copy did not create an independent aircraft identity and configuration.'
        }
        $formType.GetMethod('RemoveAircraft', $instance).Invoke($form, @()) | Out-Null
        $formType.GetMethod('AddAircraft', $instance).Invoke($form, [object[]]@($false)) | Out-Null
        if ($form.Config.FlightControllerModel -ne '' -or $null -ne $form.Config.RcArmingChannel -or
            $form.Config.ActuatorDefinitions.Count -ne 0) {
            throw 'New aircraft inherited another aircraft''s hardware, RC roles or actuator mappings.'
        }
        $formType.GetMethod('RemoveAircraft', $instance).Invoke($form, @()) | Out-Null
        $imported = Load-Config '{"AircraftName":"Imported UI","RcArmingChannel":9,"CoreRuntimePort":14659}'
        $formType.GetMethod('LoadAircraftConfiguration', $instance).Invoke($form, [object[]]@($imported)) | Out-Null
        $formType.GetMethod('LoadSettings', $instance).Invoke($form, @()) | Out-Null
        if ($form.Profiles.SelectedId -ne $alphaId -or (Profile-Control '_cmbArmingChannel').SelectedIndex -ne 9 -or
            $form.Profiles.Aircraft[1].Config.JoystickGimbalMaxRateDegSec -ne 90) {
            throw 'Import replaced another profile or lost independent draft edits.'
        }
        if ($profiles.Aircraft.Count -ne 2 -or $profiles.SelectedId -ne $alphaId -or
            $profiles.Selected.Config.FlightControllerModel -ne 'Alpha controller' -or
            $gimbalType.GetProperty('MaxRateDegSec').GetValue($null, $null) -ne 30 -or
            $gimbalType.GetProperty('TargetPitchDeg').GetValue($null, $null) -ne 12 -or
            $gimbalType.GetProperty('TargetRollDeg').GetValue($null, $null) -ne 18 -or
            $outputType.GetProperty('Generation', $static).GetValue($null) -ne $initialGeneration) {
            throw 'Draft profile changes affected the live collection, gimbal or native output generation before Save.'
        }
    } finally { $form.Dispose() }
    Write-Host 'Real aircraft selector, draft isolation, copy/new/remove, import and native output isolation checks passed.'
}
