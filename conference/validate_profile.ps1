# SPDX-License-Identifier: Apache-2.0
param(
    [Parameter(Mandatory = $true)][string]$RunDirectory
)

$ErrorActionPreference = 'Stop'
$hostRoot = Join-Path $PSScriptRoot '../build/conference/prepared/host'
[AppDomain]::CurrentDomain.add_AssemblyResolve([ResolveEventHandler] {
    param($sender, $request)
    $name = ([Reflection.AssemblyName]$request.Name).Name
    $file = Join-Path $hostRoot $(
        if ($name -eq 'MissionPlanner') {
            'MissionPlanner.exe'
        } else {
            $name + '.dll'
        }
    )
    if (Test-Path -LiteralPath $file) {
        return [Reflection.Assembly]::LoadFrom($file)
    }
    return $null
})
[Reflection.Assembly]::LoadFrom((Join-Path $hostRoot 'MissionPlanner.Utilities.dll')) | Out-Null
[MissionPlanner.Utilities.Settings]::CustomUserDataDirectory = [IO.Path]::GetFullPath($RunDirectory)
[MissionPlanner.Utilities.Settings]::AppConfigName = 'NOMADConference_' + [IO.Path]::GetFileName($RunDirectory)
$plugin = [Reflection.Assembly]::LoadFrom((Join-Path $hostRoot 'plugins/NOMADPlugin.dll'))
$profiles = $plugin.GetType('NOMAD.MissionPlanner.NOMADProfiles', $true).GetMethod('Load').Invoke($null, $null)
if ($profiles.Aircraft.Count -ne 1 -or $profiles.Selected.Config.Inputs.Count -ne 0) {
    throw 'Expected one isolated aircraft without physical input bindings.'
}
Write-Output 'PASS actual NOMAD demo profile validation'
