# SPDX-License-Identifier: Apache-2.0
# Test the runner's actual prelaunch guard without executing its startup body.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$runner = Join-Path $PSScriptRoot 'test_plugin_native_sitl.ps1'
$tokens = $null
$errors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile($runner, [ref]$tokens, [ref]$errors)
if ($errors) { throw 'Native runner parse failed.' }
$guard = $ast.Find({ param($node)
    $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Assert-NativeSitlProfileSafe'
}, $false)
if (!$guard) { throw 'Native runner prelaunch guard missing.' }
. ([scriptblock]::Create($guard.Extent.Text))
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$resolver = [ResolveEventHandler] {
    param($sender, $eventArgs)
    if (([Reflection.AssemblyName]$eventArgs.Name).Name -ne 'Newtonsoft.Json') { return $null }
    return [Reflection.Assembly]::LoadFrom((Join-Path $hostRoot 'Newtonsoft.Json.dll'))
}
[AppDomain]::CurrentDomain.add_AssemblyResolve($resolver)
try {
    $assembly = [Reflection.Assembly]::LoadFile((Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'))
    $configType = $assembly.GetType('NOMAD.MissionPlanner.NOMADConfig', $true)
    $profilesType = $assembly.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true)
    function New-SafeConfig {
        $config = [Activator]::CreateInstance($configType)
        $config.AircraftName = 'Synthetic preflight'
        return $config
    }
    function Assert-Rejected($config) {
        $rejected = $false
        try { Assert-NativeSitlProfileSafe $config } catch { $rejected = $true }
        if (!$rejected) { throw 'Unsafe preflight configuration was accepted.' }
    }
    Assert-NativeSitlProfileSafe (New-SafeConfig)
    foreach ($name in @('RouterClientEnabled', 'JoystickGimbalEnabled', 'JoystickPositionEnabled',
        'GimbalArrowKeysEnabled', 'DebugMode')) {
        $config = New-SafeConfig
        $config.$name = $true
        Assert-Rejected $config
    }
    foreach ($name in @('JoystickSw1UpAction', 'JoystickSw1DownAction', 'JoystickSw2UpAction',
        'JoystickSw2DownAction', 'JoystickSw3UpAction', 'JoystickSw3DownAction')) {
        $config = New-SafeConfig
        $config.$name = 'synthetic:activate'
        Assert-Rejected $config
        $config.$name = 'none'
        Assert-Rejected $config
        $config.$name = ' '
        Assert-NativeSitlProfileSafe $config
        $config.$name = 'None'
        Assert-NativeSitlProfileSafe $config
    }
    $config = New-SafeConfig
    $inputProfile = [Activator]::CreateInstance($assembly.GetType('NOMAD.MissionPlanner.VideoInputProfile', $true))
    $inputProfile.Name = 'Synthetic'
    $inputProfile.Url = 'rtsp://127.0.0.1/synthetic'
    $config.VideoInputs.Add($inputProfile)
    $config.SelectedVideoInputName = $inputProfile.Name
    Assert-Rejected $config
    $config.AutoStartHudVideo = $false
    Assert-NativeSitlProfileSafe $config

    # Exercise real profile selection and fallback parsing in build-only files.
    $directory = Join-Path $root ('build/tests/native-preflight-' + [guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $directory | Out-Null
    $paths = [object[]]@('profiles.json', 'profiles.json.bak', 'legacy.json', 'legacy.json.bak')
    for ($index = 0; $index -lt $paths.Count; $index++) { $paths[$index] = [string](Join-Path $directory $paths[$index]) }
    $load = $profilesType.GetMethod('LoadFromPaths', [Reflection.BindingFlags]'Static,NonPublic')
    $config = New-SafeConfig
    $config.RouterClientEnabled = $true
    $profiles = $profilesType.GetConstructor([type[]]@($configType)).Invoke([object[]]@($config))
    foreach ($index in 0..3) {
        $value = if ($index -lt 2) { $profiles } else { $config }
        $json = [Newtonsoft.Json.JsonConvert]::SerializeObject($value)
        [IO.File]::WriteAllText($paths[$index], $json)
        $loaded = $load.Invoke($null, $paths)
        Assert-Rejected $loaded.Selected.Config
        if ([IO.File]::ReadAllText($paths[$index]) -cne $json) { throw 'Preflight loading rewrote a fixture.' }
        Remove-Item -LiteralPath $paths[$index]
    }
    Write-Host 'Native preflight passed: disabled settings, every startup owner, six bindings and four profile candidates.'
    Write-Host 'No plugin Init/Start, child process or operator file writes were performed.'
} finally {
    [AppDomain]::CurrentDomain.remove_AssemblyResolve($resolver)
}
