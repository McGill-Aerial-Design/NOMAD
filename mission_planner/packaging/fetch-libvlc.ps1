param(
    [string]$Arch = "win-x64",
    [string]$TempDir = "$env:TEMP\nomad_libvlc",
    [switch]$Verbose
)

function Write-Log { param($m) ; Write-Host $m -ForegroundColor Cyan }

# Prepare directories
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$packDir = Join-Path $scriptDir "libvlc-windows"
$thirdParty = Join-Path $scriptDir "..\third_party\libvlc"

if (!(Test-Path $TempDir)) { New-Item -ItemType Directory -Path $TempDir | Out-Null }
if (!(Test-Path $packDir)) { New-Item -ItemType Directory -Path $packDir | Out-Null }
if (!(Test-Path $thirdParty)) { New-Item -ItemType Directory -Path $thirdParty | Out-Null }

# Helper: get latest version from NuGet registration index
function Get-LatestVersion($packageId) {
    $url = "https://api.nuget.org/v3/registration5-gz-semver2/$($packageId.ToLower())/index.json"
    if ($Verbose) { Write-Log "Querying $url" }
    $json = Invoke-RestMethod -Uri $url -UseBasicParsing
    $versions = $json.items | ForEach-Object { $_.items } | ForEach-Object { $_ } | Select-Object -ExpandProperty catalogEntry
    $sorted = $versions | Sort-Object -Property version -Descending
    return $sorted[0].version
}

# Download and extract a NuGet package
function Download-Nupkg($packageId, $version, $outDir) {
    $url = "https://www.nuget.org/api/v2/package/$packageId/$version"
    $nupkg = Join-Path $TempDir "$packageId.$version.nupkg"
    Write-Log "Downloading $packageId $version..."
    Invoke-WebRequest -Uri $url -OutFile $nupkg -UseBasicParsing
    $nupkgZip = "$nupkg.zip"
    Copy-Item $nupkg $nupkgZip -Force
    Write-Log "Extracting $nupkgZip to $outDir"
    Expand-Archive -Path $nupkgZip -DestinationPath $outDir -Force
    return $outDir
}

# 1) VideoLAN.LibVLC.Windows (native runtimes)
$vlcPkg = 'VideoLAN.LibVLC.Windows'
$vlcVersion = Get-LatestVersion $vlcPkg
Write-Log ("Latest $($vlcPkg): $vlcVersion")
$vlcExtract = Download-Nupkg $vlcPkg $vlcVersion $TempDir\$vlcPkg

# Copy native files for requested arch
$archShort = if ($Arch -match 'x64') { 'x64' } elseif ($Arch -match 'x86') { 'x86' } else { $Arch }
$buildNative = Join-Path $vlcExtract "build\$archShort"
if (Test-Path $buildNative) {
    Write-Log "Copying native runtime files from build\$archShort to packaging/libvlc-windows"
    Copy-Item (Join-Path $buildNative "*") $packDir -Recurse -Force
} else {
    # Fallback: try runtimes layout
    $nativeDir = Join-Path $vlcExtract "runtimes\$Arch\native"
    if (!(Test-Path $nativeDir)) {
        Write-Log "ERROR: native runtime folder not found: $buildNative or $nativeDir"; exit 1
    }
    Write-Log "Copying native runtime files to packaging/libvlc-windows"
    Copy-Item (Join-Path $nativeDir "*") $packDir -Recurse -Force
}

# 2) LibVLCSharp.WinForms (managed assemblies)
$libPkg = 'LibVLCSharp.WinForms'
$libVersion = Get-LatestVersion $libPkg
Write-Log ("Latest $($libPkg): $libVersion")
$libExtract = Download-Nupkg $libPkg $libVersion $TempDir\$libPkg

# Find dlls under lib/
$libPath = Join-Path $libExtract 'lib'
$dlls = Get-ChildItem -Path $libPath -Recurse -Include 'LibVLCSharp.WinForms.dll','LibVLCSharp.Shared.dll' -ErrorAction SilentlyContinue
if ($dlls.Count -eq 0) { Write-Log "ERROR: managed DLLs not found in package"; exit 1 }

foreach ($dll in $dlls) {
    Write-Log "Copying $($dll.Name) to third_party/libvlc"
    Copy-Item $dll.FullName $thirdParty -Force
}

# 3) LibVLCSharp core package (contains LibVLCSharp.Shared.dll)
$corePkg = 'LibVLCSharp'
$coreVersion = Get-LatestVersion $corePkg
Write-Log ("Latest $($corePkg): $coreVersion")
$coreExtract = Download-Nupkg $corePkg $coreVersion $TempDir\$corePkg
$coreLibPath = Join-Path $coreExtract 'lib'
# Copy any LibVLCSharp managed assemblies (LibVLCSharp.dll, LibVLCSharp.Shared.dll, etc.)
$coreDlls = Get-ChildItem -Path $coreLibPath -Recurse -Include 'LibVLCSharp*.dll' -ErrorAction SilentlyContinue
foreach ($dll in $coreDlls) {
    Write-Log "Copying $($dll.Name) to third_party/libvlc"
    Copy-Item $dll.FullName $thirdParty -Force
}

Write-Log "Fetch and extraction complete. Packaging files are in: $packDir" 
Write-Log "Managed dlls are in: $thirdParty"

# Cleanup temp (optional)
# Remove-Item -Recurse -Force $TempDir

return 0
