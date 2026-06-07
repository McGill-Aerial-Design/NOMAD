param(
    [string]$ScriptDir = $PSScriptRoot
)

# Packaging helper: copy libVLC redistributables into the build output
$packageDir = Resolve-Path (Join-Path $ScriptDir "libvlc-windows") -ErrorAction SilentlyContinue
if (-not $packageDir) {
    Write-Host "No libVLC packaging directory found at: $(Join-Path $ScriptDir 'libvlc-windows')" -ForegroundColor Gray
    return 0
}

$projectDir = Resolve-Path (Join-Path $ScriptDir '..\src')
$buildDir = Join-Path $projectDir.Path "bin\Release"

if (-not (Test-Path $buildDir)) {
    Write-Host "Build output not found at $buildDir - run build first" -ForegroundColor Yellow
    return 1
}

Write-Host "Copying libVLC redistributables from $($packageDir) to $buildDir" -ForegroundColor Cyan
Copy-Item "$($packageDir)\*" $buildDir -Recurse -Force
Write-Host "Copied libVLC files into build output" -ForegroundColor Green

return 0
