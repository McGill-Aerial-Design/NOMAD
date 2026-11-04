# SPDX-License-Identifier: Apache-2.0
param([Parameter(Mandatory=$true)][string]$MSBuild, [Parameter(Mandatory=$true)][string]$RepoRoot)
$ErrorActionPreference = 'Stop'
$project = Join-Path $RepoRoot 'mission_planner/media_worker/NOMADMediaWorker.csproj'
& $MSBuild $project /t:Build /p:Configuration=Release /v:minimal /nologo
if ($LASTEXITCODE -ne 0) { throw 'Media worker compilation failed.' }
$output = Join-Path $RepoRoot 'build/media_worker'
New-Item -ItemType Directory -Force -Path $output | Out-Null
Copy-Item -LiteralPath (Join-Path $RepoRoot 'mission_planner/media_worker/bin/Release/NOMADMediaWorker.exe') -Destination $output -Force
$hostDirectory = 'C:\Program Files (x86)\Mission Planner'
$support = @('Newtonsoft.Json', 'System.Memory', 'System.Buffers', 'System.Numerics.Vectors',
    'System.Runtime.CompilerServices.Unsafe')
foreach ($name in $support) { Copy-Item -LiteralPath (Join-Path $hostDirectory "$name.dll") -Destination $output -Force }
$redirects = $support | Where-Object { $_ -ne 'Newtonsoft.Json' } | ForEach-Object {
    $identity = [Reflection.AssemblyName]::GetAssemblyName((Join-Path $output "$_.dll"))
    $key = ([BitConverter]::ToString($identity.GetPublicKeyToken())).Replace('-', '').ToLowerInvariant()
    '<dependentAssembly><assemblyIdentity name="' + $_ + '" publicKeyToken="' + $key +
        '" culture="neutral"/><bindingRedirect oldVersion="0.0.0.0-' + $identity.Version +
        '" newVersion="' + $identity.Version + '"/></dependentAssembly>'
}
[IO.File]::WriteAllText((Join-Path $output 'NOMADMediaWorker.exe.config'),
    '<configuration><startup><supportedRuntime version="v4.0" sku=".NETFramework,Version=v4.8"/></startup>' +
    '<runtime><assemblyBinding xmlns="urn:schemas-microsoft-com:asm.v1">' + ($redirects -join '') +
    '</assemblyBinding></runtime></configuration>')
$bundle = [ordered]@{}
foreach ($name in @('NOMADMediaWorker.exe', 'NOMADMediaWorker.exe.config') + @($support | ForEach-Object { "$_.dll" })) {
    $bundle[$name] = (Get-FileHash -LiteralPath (Join-Path $output $name)).Hash.ToLowerInvariant()
}
$bundle | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $output 'bundle.json') -Encoding UTF8
