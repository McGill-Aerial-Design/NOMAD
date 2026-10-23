# SPDX-License-Identifier: Apache-2.0
# Actual settings dialogs and draft models only: no plugin Init, transports or devices.
function Invoke-SetupEditorTests($Plugin, $ConfigType) {
    $private = [Reflection.BindingFlags]'Instance,NonPublic'
    $constructors = [Reflection.BindingFlags]'Instance,NonPublic,Public'
    $payloadType = $Plugin.GetType('NOMAD.MissionPlanner.PayloadEditorForm', $true)
    $linkType = $Plugin.GetType('NOMAD.MissionPlanner.RouterLinkEditorForm', $true)
    function Require([bool]$Condition, [string]$Message) { if (!$Condition) { throw $Message } }
    function Field($Form, [string]$Name) { return $Form.GetType().GetField($Name, $private).GetValue($Form) }
    function New-Editor($Type, $Source, [int]$Index, [string]$Id) {
        $arguments = New-Object object[] 3
        $arguments[0] = $Source; $arguments[1] = $Index; $arguments[2] = $Id
        return $Type.GetConstructors($constructors)[0].Invoke($arguments)
    }
    function Candidate($Form) { return ,$Form.GetType().GetMethod('BuildCandidate', $private).Invoke($Form, @()) }
    function Reject-Editor($Form) {
        $failed = $false
        try { Candidate $Form | Out-Null } catch { $failed = $true }
        Require $failed 'Invalid editor candidate bypassed model validation.'
    }
    $types = @('ServoToggle','ServoPosition','ServoBidirectional','RelayToggle','RelayPulse')
    foreach ($behavior in $types) {
        $safe = if ($behavior -eq 'ServoToggle') { 2000 } elseif ($behavior -eq 'ServoBidirectional') { 1500 } else { 1300 }
        $json = '{"id":"stable","name":"Friendly output","behavior":"' + $behavior + '","channel":4,' +
            '"pwm_min":1000,"pwm_max":2000,"pwm_neutral":1500,"safe_pwm":' + $safe + ',"reversed":true,' +
            '"pulse_ms":650,"hold_ms":850,"primary_label":"Open","secondary_label":"Close",' +
            '"negative_label":"Lower","neutral_label":"Hold","positive_label":"Raise",' +
            '"hazardous":true,"confirmation_count":3,"confirmation_window_ms":2500,"require_neutral":true}'
        $source = [Newtonsoft.Json.Linq.JArray]::Parse('[' + $json + ']')
        $before = $source.ToString()
        $form = New-Editor $payloadType $source 0 ''
        try {
            $value = Candidate $form
            Require ([Newtonsoft.Json.Linq.JToken]::DeepEquals($source, $value)) ('Unchanged edit lost inactive payload fields: ' + $behavior)
            (Field $form '_name').Text = 'Renamed output'
            $renamed = Candidate $form
            Require ($renamed[0]['id'].ToString() -eq 'stable' -and $renamed[0]['name'].ToString() -eq 'Renamed output') 'Payload rename changed its stable ID.'
            (Field $form '_count').Value = 1; Reject-Editor $form
            (Field $form '_count').Value = 3; (Field $form '_requireNeutral').Checked = $false; Reject-Editor $form
            (Field $form '_requireNeutral').Checked = $true
            (Field $form '_minimum').Value = 2100; Reject-Editor $form
            Require ($source.ToString() -ceq $before) 'Invalid or cancelled payload edit changed its source.'
        } finally { $form.Dispose() }
        $empty = [Newtonsoft.Json.Linq.JArray]::new()
        $form = New-Editor $payloadType $empty -1 'new_payload'
        try {
            (Field $form '_behavior').SelectedIndex = [array]::IndexOf($types,$behavior)
            $created = Candidate $form
            Require ($created.Count -eq 1 -and $created[0]['behavior'].ToString() -eq $behavior -and
                $created[0]['confirmation_count'].ToString() -eq '3' -and $created[0]['require_neutral'].ToString() -eq 'True') 'New payload lost type or conservative interlock defaults.'
            Require ($empty.Count -eq 0) 'Creating a payload changed the original array before acceptance.'
        } finally { $form.Dispose() }
    }
    $duplicate = [Newtonsoft.Json.Linq.JArray]::Parse('[{"id":"a","name":"First","behavior":"ServoToggle","channel":1,"safe_pwm":1000}]')
    $form = New-Editor $payloadType $duplicate -1 'b'
    try { Reject-Editor $form; Require ($duplicate.Count -eq 1) 'Duplicate output changed payload source.' } finally { $form.Dispose() }
    $form = New-Editor $payloadType $duplicate -1 'a'
    try { (Field $form '_channel').Value = 2; Reject-Editor $form } finally { $form.Dispose() }

    foreach ($transport in @('UDP','TCP','COM')) {
        $draft = Load-Config ('{"RouterLinks":[{"Id":"stable_link","Name":"Friendly link","Transport":"' + $transport +
            '","Port":14750,"RemoteHost":"server.invalid","RemotePort":14790,"Device":"COM7","BaudRate":230400,' +
            '"Priority":123,"ReconnectSeconds":1.234567}],"RouterPreferredLink":"stable_link"}')
        foreach ($interval in @(0.0001, 7200.0, 1.234567)) {
            $draft.RouterLinks[0].ReconnectSeconds = $interval
            $before = [Newtonsoft.Json.JsonConvert]::SerializeObject($draft)
            $form = New-Editor $linkType $draft 0 ''
            try {
                $candidate = Candidate $form
                Require ([Newtonsoft.Json.JsonConvert]::SerializeObject($candidate) -ceq $before) ('Unchanged link edit lost fields: ' + $transport)
                (Field $form '_name').Text = 'Renamed link'
                $candidate = Candidate $form
                Require ($candidate.RouterLinks[0].Id -eq 'stable_link' -and !$candidate.RouterClientEnabled) 'Link rename changed stable ID or enabled the router.'
                (Field $form '_reconnect').Text = 'NaN'; Reject-Editor $form
                Require ([Newtonsoft.Json.JsonConvert]::SerializeObject($draft) -ceq $before) 'Invalid or cancelled link edit changed the original draft.'
            } finally { $form.Dispose() }
        }
        $empty = Load-Config '{}'
        $form = New-Editor $linkType $empty -1 'new_link'
        try {
            (Field $form '_transport').SelectedItem = $transport
            (Field $form '_host').Text = 'server.invalid'; (Field $form '_device').Text = 'COM8'
            (Field $form '_remotePort').Value = 14790
            $candidate = Candidate $form
            Require ($candidate.RouterLinks.Count -eq 1 -and $candidate.RouterLinks[0].Transport -eq $transport -and
                !$candidate.RouterClientEnabled -and $empty.RouterLinks.Count -eq 0) 'New physical link changed the source or enabled the router.'
        } finally { $form.Dispose() }
    }
    $draft = Load-Config '{"RouterLinks":[{"Id":"one","Port":14750},{"Id":"two","Port":14760}]}'
    $form = New-Editor $linkType $draft 1 ''
    try {
        (Field $form '_port').Value = 14750; Reject-Editor $form
        (Field $form '_port').Value = 14600; Reject-Editor $form
        (Field $form '_port').Value = 0; Reject-Editor $form
        Require ($draft.RouterLinks[1].Port -eq 14760) 'Port validation changed the previous link draft.'
    } finally { $form.Dispose() }

    $draft.RouterPreferredLink = 'one'
    $form = New-Editor $linkType $draft 0 ''
    try {
        (Field $form '_enabled').Checked = $false; Reject-Editor $form
        (Field $form '_enabled').Checked = $true; (Field $form '_bind').Text = 'not-an-ip'; Reject-Editor $form
        (Field $form '_bind').Text = '0.0.0.0'; (Field $form '_host').Text = 'bad host'; Reject-Editor $form
    } finally { $form.Dispose() }

    $config = Load-Config ('{"AircraftName":"Draft","Inputs":[' +
        '{"Id":"button","Name":"Button","Device":"missing","Source":"Button","Action":"removed:activate"},' +
        '{"Id":"axis","Name":"Axis","Device":"missing","Source":"Axis","Action":"toggle:position"}],' +
        '"ActuatorDefinitions":[{"id":"position","name":"Friendly position","behavior":"ServoPosition","channel":1,"safe_pwm":1500,' +
        '"hazardous":false,"confirmation_count":0},{"id":"toggle","name":"Friendly toggle","behavior":"ServoToggle","channel":2,"safe_pwm":1000}]}')
    $profilesType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    $profiles = $profilesType.GetConstructor([type[]]@($ConfigType)).Invoke([object[]]@($config))
    $formType = $Plugin.GetType('NOMAD.MissionPlanner.NOMADSettingsForm', $true)
    $inputType = $Plugin.GetType('NOMAD.MissionPlanner.InputBindingEditorForm', $true)
    function Input-Editor($Config, [int]$Index) {
        return $inputType.GetConstructors($constructors)[0].Invoke([object[]]@($Config, $Index))
    }
    $form = $formType.GetConstructor([type[]]@($profilesType)).Invoke([object[]]@($profiles))
    try {
        $form.Config.Inputs.Add([Newtonsoft.Json.Linq.JObject]::Parse('{"Id":"switch","Name":"Switch","Device":"missing","Source":"AxisSwitch3","Action":"toggle:safe"}'))
        $formType.GetMethod('LoadInputList', $private).Invoke($form, @()) | Out-Null
        $inputRow = (Field $form '_inputList').Rows[2]
        Require ($inputRow.Cells[1].Value.Contains('3-position switch') -and $inputRow.Cells[1].Value.Contains('Unavailable: missing') -and
            $form.Config.Inputs[2]['Device'].Value -eq 'missing' -and $form.Config.Inputs[2]['Source'].Value -eq 'AxisSwitch3') 'Input list lost friendly/missing labels or changed raw source identity.'
        $form.Config.Inputs.RemoveAt(2)
        $editor = Input-Editor $form.Config 0
        try {
            $actions = Field $editor '_action'
            Require (@($actions.Items | Where-Object { $_.Key -eq 'position:position' }).Count -eq 0) 'Button offered position without a value.'
            Require (@($actions.Items | Where-Object { $_.Key -eq 'position:safe' }).Count -eq 1) 'Explicit Safe was missing.'
            Require ($actions.SelectedItem.Key -eq 'removed:activate' -and $actions.SelectedItem.Value.StartsWith('Unavailable:')) 'Missing saved action was replaced.'
            $candidate = Candidate $editor
            Require ($candidate[0]['Action'].Value -eq 'removed:activate') 'Saving changed unresolved input action.'
        } finally { $editor.Dispose() }
        $editor = Input-Editor $form.Config 1
        try {
            $actions = Field $editor '_action'
            Require ($actions.SelectedItem.Key -eq 'toggle:position' -and $actions.SelectedItem.Value.StartsWith('Unavailable:') -and
                @($actions.Items | Where-Object { $_.Key -eq 'position:position' }).Count -eq 1) 'Continuous eligibility/unavailable action was incorrect.'
            Require (@($actions.Items | Where-Object { $_.Key -eq 'gimbal:pitch' }).Count -eq 1) 'Gimbal movement was missing from continuous inputs.'
        } finally { $editor.Dispose() }
        $edited = [Newtonsoft.Json.Linq.JArray]$form.Config.ActuatorDefinitions.DeepClone()
        $edited[1]['name'] = [Newtonsoft.Json.Linq.JValue]::new('Renamed friendly toggle')
        $applyArguments = New-Object object[] 1; $applyArguments[0] = $edited
        $formType.GetMethod('ApplyPayloadDefinitions', $private).Invoke($form, $applyArguments) | Out-Null
        $form.Config.Inputs[0]['Action'] = [Newtonsoft.Json.Linq.JValue]::new('toggle:activate')
        $editor = Input-Editor $form.Config 0
        try {
            Require ((Field $editor '_action').SelectedItem.Value.Contains('Renamed friendly toggle')) 'Payload edits lost friendly input action names.'
            $candidate = Candidate $editor
            Require ($candidate[0]['Id'].Value -eq 'button') 'Editing changed stable input identity.'
        } finally { $editor.Dispose() }
        Require ($config.ActuatorDefinitions[1]['name'].Value -eq 'Friendly toggle' -and $config.Inputs[0]['Action'].Value -eq 'removed:activate') 'Canceled draft changed active source.'
        $form.Config.Inputs[0]['Action'] = [Newtonsoft.Json.Linq.JValue]::new('position:safe')
        $form.Config.Inputs[1]['Action'] = [Newtonsoft.Json.Linq.JValue]::new('position:position')
        $remaining = [Newtonsoft.Json.Linq.JArray]$form.Config.ActuatorDefinitions.DeepClone(); $remaining.RemoveAt(0)
        $applyArguments[0] = $remaining
        $formType.GetMethod('ApplyPayloadDefinitions', $private).Invoke($form, $applyArguments) | Out-Null
        $newEditor = $formType.GetMethod('CreatePayloadEditor', $private).Invoke($form, [object[]]@([int]-1))
        try {
            (Field $newEditor '_channel').Value = 1
            $added = Candidate $newEditor
            $newId = $added[1]['id'].Value
            Require ($newId -ne 'position' -and $newId.StartsWith('payload_') -and $newId.Length -eq 40) 'New output reused old identity.'
            $applyArguments[0] = $added
            $formType.GetMethod('ApplyPayloadDefinitions', $private).Invoke($form, $applyArguments) | Out-Null
            foreach ($index in 0..1) {
                $editor = Input-Editor $form.Config $index
                try { Require ((Field $editor '_action').SelectedItem.Value.StartsWith('Unavailable:')) 'Adding a payload resurrected removed input target.' }
                finally { $editor.Dispose() }
            }
        } finally { $newEditor.Dispose() }
        # Failed top-level validation must leave router drafts controlled.
        $validInputs = $form.Config.Inputs.DeepClone()
        $duplicate = $form.Config.Inputs[0].DeepClone(); $form.Config.Inputs.Add($duplicate)
        $failed = $false
        try { $formType.GetMethod('SaveSettings', $private).Invoke($form, @()) | Out-Null } catch { $failed = $true }
        Require $failed 'Duplicate input did not establish failed save.'
        $linksBefore = [Newtonsoft.Json.JsonConvert]::SerializeObject($form.Config.RouterLinks)
        $formType.GetMethod('EditRouterLink', $private).Invoke($form, [object[]]@([int]-1)) | Out-Null
        Require ((Field $form '_linkError').Text.Length -gt 0 -and
            [Newtonsoft.Json.JsonConvert]::SerializeObject($form.Config.RouterLinks) -ceq $linksBefore) 'Router action after failed Save escaped validation or changed links.'
        $form.Config.Inputs = $validInputs
        $formType.GetMethod('LoadSettings', $private).Invoke($form, @()) | Out-Null
        Require ((Field $form '_linkError').Text -eq '') 'Reload retained old link validation error.'
    } finally { $form.Dispose() }
    Write-Host 'Payload/link/input editors passed current source choices, interlocks, unavailable bindings and independent drafts.'
}
