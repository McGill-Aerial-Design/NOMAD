param(
    [string]$ScriptDir = $PSScriptRoot
)

$srcManaged = Resolve-Path (Join-Path $ScriptDir '..\third_party\libvlc') -ErrorAction SilentlyContinue
if (-not $srcManaged) {
    Write-Host "No managed LibVLC assemblies found in third_party (skipping)" -ForegroundColor Gray
    return 0
}

$buildDir = Resolve-Path (Join-Path $ScriptDir '..\src\bin\Release')
if (-not $buildDir) {
    Write-Host "Build output not found - run build first" -ForegroundColor Yellow
    return 1
}

Write-Host "Copying LibVLC managed assemblies from $($srcManaged) to $($buildDir.Path)" -ForegroundColor Cyan
Copy-Item "$($srcManaged)\*" $buildDir -Filter "LibVLCSharp.*.dll" -Force -ErrorAction SilentlyContinue
Write-Host "Copied managed assemblies" -ForegroundColor Green

return 0
