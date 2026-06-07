Third-party LibVLC managed assemblies

This folder is intended to contain the managed assemblies for LibVLCSharp when you cannot use NuGet restore in this project format.

Files to place here:
- LibVLCSharp.WinForms.dll
- LibVLCSharp.Shared.dll

How to get them:
- Option A (recommended during development): Use Visual Studio NuGet package manager to install `LibVLCSharp.WinForms` into a temporary project, then copy the resulting DLLs from the package cache (`%UserProfile%\.nuget\packages\libvlcsharp.winforms\...\lib\`) into this folder.
- Option B: Download the compiled DLLs from a trusted build or include them from the `LibVLCSharp` GitHub releases.

When present, the project `.csproj` will link to these DLLs using conditional references so they are deployed next to `NOMADPlugin.dll` during build.
