# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Test Mission Planner's persisted configuration migration using the built plugin.

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

[Reflection.Assembly]::LoadFrom($newtonsoftPath) | Out-Null
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
    return $loadMethod.Invoke($null, $invokeArguments)
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
    $legacy = [ordered]@{
        DualLinkEnabled = $true
        RouterEnabled = $false
        RouterMode = 'Standalone'
        RouterLinks = @()
        RouterConsumers = @()
        IntegratedFlightMode = $true
        RadioMasterConnectionType = 'COM'
        LteMavlinkPort = 14560
        RouterBindAddress = '127.0.0.1'
        ManagementBindAddress = '127.0.0.1'
        RouterLocalPort = 14620
        ManagementPort = 14621
        CoreRuntimePort = 14631
        CoreClientCredential = 'local-client-credential'
        CoreApiKey = 'retired-enable-gate'
    }
    $legacyJson = ConvertTo-Json -InputObject $legacy -Depth 8
    $config = Load-Config $legacyJson
    if (-not $config.DualLinkEnabled) { throw 'DualLinkEnabled did not win over RouterEnabled.' }
    if ($config.RouterLocalPort -ne 14620 -or $config.ManagementPort -ne 14621) {
        throw 'Mission Planner client endpoints were not preserved.'
    }
    if ($config.CoreRuntimePort -ne 14631 -or $config.CoreClientCredential -ne 'local-client-credential') {
        throw 'Runtime client endpoint or independent credential was not preserved.'
    }

    $firstPath = Join-Path $temporary 'migrated-first.json'
    $secondPath = Join-Path $temporary 'migrated-second.json'
    $config.ExportToFile($firstPath)
    $migrated = [IO.File]::ReadAllText($firstPath)
    if ($migrated.Contains('local-client-credential') -or $migrated.Contains('retired-enable-gate')) {
        throw 'Portable profile export leaked a credential or retired gate.'
    }
    foreach ($field in @(
        'IntegratedFlightMode', 'RouterLinks', 'RouterConsumers', 'RouterEnabled', 'RouterMode',
        'RadioMasterConnectionType', 'LteMavlinkPort', 'RouterBindAddress', 'ManagementBindAddress', 'CoreApiKey'
    )) {
        if ($migrated.Contains('"' + $field + '"')) { throw "Retired field remained after migration: $field" }
    }

    $reloaded = Load-Config $migrated
    $reloaded.ExportToFile($secondPath)
    if ([IO.File]::ReadAllText($firstPath) -cne [IO.File]::ReadAllText($secondPath)) {
        throw 'Repeated Mission Planner config migration changed the serialized result.'
    }

    $embedded = [ordered]@{ RouterMode = 'Embedded' } | ConvertTo-Json
    Assert-ThrowsMessage $embedded "RouterMode 'Embedded' is unsupported"
    Assert-ThrowsMessage '{"RouterBindAddress":"0.0.0.0"}' 'RouterBindAddress must be 127.0.0.1'
    Assert-ThrowsMessage '{"ManagementBindAddress":"0.0.0.0"}' 'ManagementBindAddress must be 127.0.0.1'
    Write-Host 'Mission Planner config migration passed: stable cleanup, client ownership, and fail-fast legacy rejection.'
} finally {
    Remove-Item -LiteralPath $temporary -Recurse -Force
}
