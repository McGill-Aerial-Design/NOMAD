# SPDX-License-Identifier: Apache-2.0
# Downloads pinned build inputs only. The installed plugin never downloads models or runtimes.
param([string]$CacheDirectory = (Join-Path $PSScriptRoot '..\..\build\vision'))
$ErrorActionPreference = 'Stop'
# Pixi can inherit PowerShell 7's module path while launching Windows PowerShell.
Import-Module (Join-Path $PSHOME 'Modules/Microsoft.PowerShell.Utility/Microsoft.PowerShell.Utility.psd1') -Force
Add-Type -AssemblyName System.IO.Compression.FileSystem
$cache = [IO.Path]::GetFullPath($CacheDirectory)
$runtime = Join-Path $cache 'runtime'
New-Item -ItemType Directory -Force -Path $cache, $runtime | Out-Null

function Get-PinnedFile([string]$Name, [string]$Url, [string]$Hash) {
    $path = Join-Path $cache $Name
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $temporary = "$path.$([guid]::NewGuid().ToString('N')).tmp"
        try {
            Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile $temporary
            if ((Get-FileHash -LiteralPath $temporary -Algorithm SHA256).Hash -ne $Hash) {
                throw "Vision input checksum mismatch: $Name"
            }
            Move-Item -LiteralPath $temporary -Destination $path
        } finally {
            if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary }
        }
    }
    if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $Hash) {
        throw "Vision cache checksum mismatch: $Name"
    }
    return $path
}

function Copy-PinnedEntry([string]$Archive, [string]$Entry, [string]$Name, [string]$Hash) {
    $target = Join-Path $runtime $Name
    if ((Test-Path -LiteralPath $target -PathType Leaf) -and
        (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -eq $Hash) { return }
    $zip = [IO.Compression.ZipFile]::OpenRead($Archive)
    try {
        $found = @($zip.Entries | Where-Object { $_.FullName -ceq $Entry })
        if ($found.Count -ne 1) { throw "Pinned vision archive entry is missing or duplicated: $Entry" }
        $inputStream = $found[0].Open()
        try {
            $outputStream = [IO.File]::Create($target)
            try { $inputStream.CopyTo($outputStream) } finally { $outputStream.Dispose() }
        } finally { $inputStream.Dispose() }
    } finally { $zip.Dispose() }
    if ((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $Hash) {
        throw "Vision archive entry checksum mismatch: $Name"
    }
}

# ONNX Runtime embeds an isolation manifest (RT_MANIFEST ID 2), so Windows resolves its C++ runtime imports in that
# context and binds them to the 2017 runtime Mission Planner's GDAL has already loaded (error 1114). Renaming the ID
# to 3, which Windows ignores for DLLs, lets VisionRuntime's activation context supply a current runtime. No code or
# manifest bytes change; ONNX Runtime is MIT-licensed.
function Copy-NonIsolatedEntry([string]$Archive, [string]$Entry, [string]$Name, [string]$Hash, [string]$PatchedHash) {
    $target = Join-Path $runtime $Name
    if ((Test-Path -LiteralPath $target -PathType Leaf) -and
        (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -eq $PatchedHash) { return }
    Copy-PinnedEntry $Archive $Entry $Name $Hash
    $bytes = [IO.File]::ReadAllBytes($target)
    $pe = [BitConverter]::ToInt32($bytes, 0x3C)
    $optional = $pe + 24
    if ([BitConverter]::ToUInt32($bytes, $pe) -ne 0x4550 -or [BitConverter]::ToUInt16($bytes, $optional) -ne 0x20B) {
        throw "Expected a PE32+ image: $Name"
    }
    $resourceRva = [BitConverter]::ToUInt32($bytes, $optional + 112 + 16)
    $sections = $optional + [BitConverter]::ToUInt16($bytes, $pe + 20)
    $base = -1
    for ($i = 0; $i -lt [BitConverter]::ToUInt16($bytes, $pe + 6); $i++) {
        $section = $sections + 40 * $i
        $va = [BitConverter]::ToUInt32($bytes, $section + 12)
        if ($resourceRva -ge $va -and $resourceRva -lt $va + [BitConverter]::ToUInt32($bytes, $section + 16)) {
            $base = [BitConverter]::ToUInt32($bytes, $section + 20) + ($resourceRva - $va)
        }
    }
    if ($base -lt 0) { throw "PE resource section not found: $Name" }
    function Get-Entries([int]$Directory) {
        $count = [BitConverter]::ToUInt16($bytes, $Directory + 12) + [BitConverter]::ToUInt16($bytes, $Directory + 14)
        for ($e = 0; $e -lt $count; $e++) { $Directory + 16 + 8 * $e }
    }
    $changed = 0
    foreach ($type in Get-Entries $base) {
        if ([BitConverter]::ToUInt32($bytes, $type) -ne 24) { continue }
        foreach ($id in Get-Entries ($base + ([BitConverter]::ToUInt32($bytes, $type + 4) -band 0x7FFFFFFF))) {
            if ([BitConverter]::ToUInt32($bytes, $id) -eq 2) { [BitConverter]::GetBytes([uint32]3).CopyTo($bytes, $id); $changed++ }
        }
    }
    if ($changed -ne 1) { throw "Expected exactly one isolation manifest in $Name; found $changed." }
    [IO.File]::WriteAllBytes($target, $bytes)
    if ((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $PatchedHash) {
        throw "Non-isolated vision runtime checksum mismatch: $Name"
    }
}


# The DirectML build includes the CPU provider used by the detector; DirectML serves only the opt-in GPU tracker.
$native = Get-PinnedFile 'microsoft.ml.onnxruntime.directml.1.24.4.zip' `
    'https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.directml/1.24.4/microsoft.ml.onnxruntime.directml.1.24.4.nupkg' `
    '57e9f11b73437bef7a309496135d4c1f96b1a8e9ddba60013fa27bfc1d788681'
$managed = Get-PinnedFile 'microsoft.ml.onnxruntime.managed.1.24.4.zip' `
    'https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.managed/1.24.4/microsoft.ml.onnxruntime.managed.1.24.4.nupkg' `
    '95cc5d366e876bcc9c39e87af277278aa3df56108fb572c884bf21f6b9e22182'
$directMl = Get-PinnedFile 'microsoft.ai.directml.1.15.4.zip' `
    'https://api.nuget.org/v3-flatcontainer/microsoft.ai.directml/1.15.4/microsoft.ai.directml.1.15.4.nupkg' `
    '4e7cb7ddce8cf837a7a75dc029209b520ca0101470fcdf275c1f49736a3615b9'
$model = Get-PinnedFile 'yolox_nano.onnx' `
    'https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.onnx' `
    'c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d'
$license = Get-PinnedFile 'YOLOX-LICENSE.txt' `
    'https://raw.githubusercontent.com/Megvii-BaseDetection/YOLOX/e1052df/LICENSE' `
    '577c03d505ec80f667ebf96ebd0cc4f6825c817ca1088ee348c48aaabd51bd92'
Copy-PinnedEntry $managed 'lib/netstandard2.0/Microsoft.ML.OnnxRuntime.dll' 'Microsoft.ML.OnnxRuntime.dll' `
    '5c3c531af36a6cb4baa01db20dcf94a0464ec36f8090aa42f35ff565b90a1ea6'
Copy-NonIsolatedEntry $native 'runtimes/win-x64/native/onnxruntime.dll' 'onnxruntime.dll' `
    'e7eedec6a6f26dc39dc948276a75ef6d2bee3fff944d874ceed0bbd3b97bff40' `
    '35d9ffb1351e4151e1b147a06750c4a66ac17ebd6d16c11be60a05d5ed8d6c9d'
Copy-NonIsolatedEntry $native 'runtimes/win-x64/native/onnxruntime_providers_shared.dll' 'onnxruntime_providers_shared.dll' `
    '265c8daf29637cb259cac8be9f08f2cd45f3883f0f0e4949cbfddd5b4cbec3b6' `
    'fc74b2954f44e618c60194ba788d918681fe7571132efcaa7ea39b32791d7e89'
Copy-PinnedEntry $native 'LICENSE' 'ONNX-RUNTIME-LICENSE.txt' `
    'c250d6278f0b47a6439fb7592b08b58a55eb9f535aa49a1db63211c3f982b674'
Copy-PinnedEntry $native 'ThirdPartyNotices.txt' 'ONNX-RUNTIME-NOTICES.txt' `
    'fb0af774b4d7cffc5b9d046f2aaeade2f37df2f80abf8033c95dfffcc77a8866'
Copy-PinnedEntry $directMl 'bin/x64-win/DirectML.dll' 'DirectML.dll' `
    '9c9e6d822561c6c41b90e6994b3e8857cf1d66dbfb1e0c4c799c7c89b4e92da1'
Copy-PinnedEntry $directMl 'LICENSE.txt' 'DirectML-LICENSE.txt' `
    'a05138e3a085ff60a44881eedfa58dccb03ecc1d7b1f6ae888418e8c2fec4b8d'
Copy-PinnedEntry $directMl 'ThirdPartyNotices.txt' 'DirectML-NOTICES.txt' `
    '2c95795c13ff48a58b6ed916f37901c23d964b5d9d601af422f17ad2172e7950'
Copy-Item -LiteralPath $model, $license -Destination $runtime -Force
$openCvManaged = Get-PinnedFile 'opencvsharp4.4.10.0.20241108.zip' `
    'https://api.nuget.org/v3-flatcontainer/opencvsharp4/4.10.0.20241108/opencvsharp4.4.10.0.20241108.nupkg' `
    '56351848e5c9210797102de05307d52cee2a122ccd9695ff5fb5e1d904e101c3'
$openCvNative = Get-PinnedFile 'opencvsharp4.runtime.win.4.10.0.20241108.zip' `
    'https://api.nuget.org/v3-flatcontainer/opencvsharp4.runtime.win/4.10.0.20241108/opencvsharp4.runtime.win.4.10.0.20241108.nupkg' `
    '4573c2c5bfb45553894d3bb08554fe533cf4397c8ce7d8ede4d639ef57d0d738'
Copy-PinnedEntry $openCvManaged 'lib/netstandard2.0/OpenCvSharp.dll' 'OpenCvSharp.dll' `
    '7e591871b2eaaa045c22b7d720bde3adceab2e19d64047135ad39ade03f8c045'
Copy-PinnedEntry $openCvNative 'runtimes/win-x64/native/OpenCvSharpExtern.dll' 'OpenCvSharpExtern.dll' `
    'c812d49897b29c20be941c0c2aff7768e34d959611829971e6fe8021d6bace69'
$noticesDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\mission_planner\third_party'))
foreach ($notice in @(
    @('OpenCvSharp-NOTICES.txt', 'd4285e08e9c5102812e64cac179f30549eec7436c5816e0c085330c205906a53'),
    @('OPENCV-IPP-EULA.rtf', '37f11c79f36fb5e1b00aca31c1234a7435c2929d83e181ee802a349ca8571749')
)) {
    $source = Join-Path $noticesDirectory $notice[0]
    if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -ne $notice[1]) {
        throw 'OpenCvSharp redistribution notice checksum mismatch.'
    }
    Copy-Item -LiteralPath $source -Destination $runtime -Force
}
# Re-exporting EfficientTAM does not reproduce the tested graph bytes, so the tested export is a pinned asset.
$tracker = Get-PinnedFile 'nomad-efficienttam-ti512-1.zip' `
    'https://github.com/McGill-Aerial-Design/NOMAD/releases/download/tracker-model-efficienttam-ti512-1/nomad-efficienttam-ti512-1.zip' `
    'cbd768719db5857a04b6319dd7ae98eea63e49321a131f23ac72bec9011be9c6'
foreach ($entry in @(
    @('encoder.onnx', '9f1322591497f2cbf53eea72bbcc14fc1e0aef9803a3e65b5e02f7015910e901'),
    @('memory_attention.onnx', 'eddb65f3c0e8d4e1d3bbadc64748e4bbda37d97848cbd5d86837c6810a86a4db'),
    @('decoder.onnx', '36ccefadc3922c66188e14d6ee91656e6699016f26d30b4d5ed4c08674f68980'),
    @('memory_encoder.onnx', 'a2a2969f2437457ee009c2ede7f5a1e07708c4918f98973cc939a7020e1401a5'),
    @('constants.bin', '44dbb6dadb89d28a6467776c5c2fb596b22e8da21289ff7df6e4dee5d1631565'),
    @('EfficientTAM-LICENSE.txt', 'c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4')
)) { Copy-PinnedEntry $tracker $entry[0] $entry[0] $entry[1] }
Write-Host 'Pinned ONNX Runtime 1.24.4 DirectML build, DirectML 1.15.4 and YOLOX Nano build inputs verified.'
Write-Host 'Pinned OpenCvSharp 4.10.0.20241108 tracker inputs and notices verified.'
Write-Host 'Pinned EfficientTAM Ti512 tracker model asset 1 verified.'
