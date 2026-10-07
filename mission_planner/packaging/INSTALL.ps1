# SPDX-License-Identifier: Apache-2.0
# Versioned deployment wrapper. Never copy an unverified loose DLL.
[CmdletBinding()]
param(
    [Parameter(Mandatory)][ValidateSet('verify', 'stage', 'adopt', 'activate', 'status', 'rollback', 'recover', 'cleanup')]
    [string]$Action,
    [Parameter(Mandatory)][string]$Root,
    [string]$Manifest,
    [string]$Package,
    [string]$Release,
    [string]$MissionPlanner = "${env:ProgramFiles(x86)}\Mission Planner",
    [string]$DeploymentTool = (Join-Path $PSScriptRoot 'tools\deploy.py'),
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $DeploymentTool -PathType Leaf)) {
    throw 'Verified release deployment tools are required; specify -DeploymentTool to their deploy.py.'
}
$toolArguments = @($DeploymentTool, $Action, '--root', $Root, '--component', 'plugin',
    '--platform', 'windows', '--architecture', 'any', '--adapter', 'plugin', '--mission-planner', $MissionPlanner)
foreach ($entry in @(@('--manifest', $Manifest), @('--package', $Package), @('--release', $Release))) {
    if ($entry[1]) {
        $toolArguments += $entry
    }
}
& $Python @toolArguments
if ($LASTEXITCODE -ne 0) {
    throw "NOMAD plugin $Action failed with exit code $LASTEXITCODE."
}
