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

$native = Get-PinnedFile 'microsoft.ml.onnxruntime.1.31.0.zip' `
    'https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/1.31.0/microsoft.ml.onnxruntime.1.31.0.nupkg' `
    '7e35fdb4f5adb62f01e60798e2e5ed05108f5fbbe1cca03fbcc37903cf7c5e3b'
$managed = Get-PinnedFile 'microsoft.ml.onnxruntime.managed.1.31.0.zip' `
    'https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.managed/1.31.0/microsoft.ml.onnxruntime.managed.1.31.0.nupkg' `
    'c632dc49e68cab65407b75f9680fa22a648eafabba9a71be697ede71599be626'
$model = Get-PinnedFile 'yolox_nano.onnx' `
    'https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.onnx' `
    'c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d'
$license = Get-PinnedFile 'YOLOX-LICENSE.txt' `
    'https://raw.githubusercontent.com/Megvii-BaseDetection/YOLOX/e1052df/LICENSE' `
    '577c03d505ec80f667ebf96ebd0cc4f6825c817ca1088ee348c48aaabd51bd92'
Copy-PinnedEntry $managed 'lib/netstandard2.0/Microsoft.ML.OnnxRuntime.dll' 'Microsoft.ML.OnnxRuntime.dll' `
    'f113c8148a3fd93852c9e532cce3f8d0f15c28f1ff3cc073338084b5a1db3351'
Copy-PinnedEntry $native 'runtimes/win-x64/native/onnxruntime.dll' 'onnxruntime.dll' `
    '0c42dc4e278f0968fdf9ebf1b1e54a74c657a9942e70803e453c91993451cf4c'
Copy-PinnedEntry $native 'runtimes/win-x64/native/onnxruntime_providers_shared.dll' 'onnxruntime_providers_shared.dll' `
    'e45d471ea2a842b268c51291b96d53568b0dae10807f43d1fb7dc0eb84e83723'
Copy-PinnedEntry $native 'LICENSE' 'ONNX-RUNTIME-LICENSE.txt' `
    'c250d6278f0b47a6439fb7592b08b58a55eb9f535aa49a1db63211c3f982b674'
Copy-PinnedEntry $native 'ThirdPartyNotices.txt' 'ONNX-RUNTIME-NOTICES.txt' `
    'c53a76501ef60db6f865f20599f220761201ac4057683acefdab37d861b86622'
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
Write-Host 'Pinned ONNX Runtime 1.31.0 and YOLOX Nano build inputs verified.'
Write-Host 'Pinned OpenCvSharp 4.10.0.20241108 tracker inputs and notices verified.'
