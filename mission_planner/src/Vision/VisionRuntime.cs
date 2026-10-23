// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal static class VisionRuntime
    {
        private const string ManagedHash = "f113c8148a3fd93852c9e532cce3f8d0f15c28f1ff3cc073338084b5a1db3351";
        private const string NativeHash = "0c42dc4e278f0968fdf9ebf1b1e54a74c657a9942e70803e453c91993451cf4c";
        private static readonly object Gate = new object();
        private static Assembly _managed;
        private static IntPtr _native;
        private static Assembly _openCvManaged;
        private static IntPtr _openCvNative;
        internal static string BuiltinModelPath { get; private set; }

        static VisionRuntime() { AppDomain.CurrentDomain.AssemblyResolve += ResolveManaged; }

        internal static void EnsureLoaded(CancellationToken token)
        {
            lock (Gate)
            {
                token.ThrowIfCancellationRequested();
                if (_managed != null && _native != IntPtr.Zero) { return; }
                if (Environment.OSVersion.Platform != PlatformID.Win32NT || !Environment.Is64BitProcess)
                { throw new PlatformNotSupportedException("Vision requires Windows x64."); }
                string cache = Path.Combine(Path.GetDirectoryName(NOMADStorage.PluginsDirectory),
                    "vision", "onnxruntime-1.31.0-" + NativeHash.Substring(0, 12));
                EnsureDirectory(cache);
                string managed = Extract(cache, "Microsoft.ML.OnnxRuntime.dll", ManagedHash, token);
                string native = Extract(cache, "onnxruntime.dll", NativeHash, token);
                Extract(cache, "onnxruntime_providers_shared.dll",
                    "e45d471ea2a842b268c51291b96d53568b0dae10807f43d1fb7dc0eb84e83723", token);
                BuiltinModelPath = Extract(cache, "yolox_nano.onnx", VisionModelProfile.BuiltinHash, token);
                Extract(cache, "ONNX-RUNTIME-LICENSE.txt",
                    "c250d6278f0b47a6439fb7592b08b58a55eb9f535aa49a1db63211c3f982b674", token);
                Extract(cache, "ONNX-RUNTIME-NOTICES.txt",
                    "c53a76501ef60db6f865f20599f220761201ac4057683acefdab37d861b86622", token);
                Extract(cache, "YOLOX-LICENSE.txt",
                    "577c03d505ec80f667ebf96ebd0cc4f6825c817ca1088ee348c48aaabd51bd92", token);
                _native = LoadNative(native, "onnxruntime.dll", NativeHash);
                var existing = AppDomain.CurrentDomain.GetAssemblies()
                    .FirstOrDefault(assembly => assembly.GetName().Name == "Microsoft.ML.OnnxRuntime");
                if (existing != null && (string.IsNullOrEmpty(existing.Location) ||
                    VisionModelProfile.Hash(File.ReadAllBytes(existing.Location)) != ManagedHash))
                { throw new InvalidOperationException("A different ONNX Runtime is already loaded."); }
                _managed = existing ?? Assembly.Load(File.ReadAllBytes(managed));
            }
        }

        internal static void EnsureOpenCvLoaded(CancellationToken token)
        {
            const string managedHash = "7e591871b2eaaa045c22b7d720bde3adceab2e19d64047135ad39ade03f8c045";
            const string nativeHash = "c812d49897b29c20be941c0c2aff7768e34d959611829971e6fe8021d6bace69";
            lock (Gate)
            {
                token.ThrowIfCancellationRequested();
                if (_openCvManaged != null && _openCvNative != IntPtr.Zero) { return; }
                if (Environment.OSVersion.Platform != PlatformID.Win32NT || !Environment.Is64BitProcess)
                { throw new PlatformNotSupportedException("Visual tracking requires Windows x64."); }
                string cache = Path.Combine(Path.GetDirectoryName(NOMADStorage.PluginsDirectory),
                    "vision", "opencv-4.10.0-" + nativeHash.Substring(0, 12));
                EnsureDirectory(cache);
                string managed = Extract(cache, "OpenCvSharp.dll", managedHash, token);
                string native = Extract(cache, "OpenCvSharpExtern.dll", nativeHash, token);
                Extract(cache, "OpenCvSharp-NOTICES.txt",
                    "d4285e08e9c5102812e64cac179f30549eec7436c5816e0c085330c205906a53", token);
                Extract(cache, "OPENCV-IPP-EULA.rtf",
                    "37f11c79f36fb5e1b00aca31c1234a7435c2929d83e181ee802a349ca8571749", token);
                _openCvNative = LoadNative(native, "OpenCvSharpExtern.dll", nativeHash);
                var existing = AppDomain.CurrentDomain.GetAssemblies()
                    .FirstOrDefault(assembly => assembly.GetName().Name == "OpenCvSharp");
                if (existing != null && (string.IsNullOrEmpty(existing.Location) ||
                    VisionModelProfile.Hash(File.ReadAllBytes(existing.Location)) != managedHash))
                { throw new InvalidOperationException("A different OpenCvSharp is already loaded."); }
                _openCvManaged = existing ?? Assembly.Load(File.ReadAllBytes(managed));
            }
        }

        private static Assembly ResolveManaged(object sender, ResolveEventArgs args)
        {
            if (args.Name == _managed?.FullName) { return _managed; }
            return args.Name == _openCvManaged?.FullName ? _openCvManaged : null;
        }

        private static IntPtr LoadNative(string path, string moduleName, string expectedHash)
        {
            IntPtr existing = GetModuleHandle(moduleName);
            if (existing != IntPtr.Zero)
            {
                var name = new System.Text.StringBuilder(32768);
                if (GetModuleFileName(existing, name, name.Capacity) == 0 ||
                    VisionModelProfile.Hash(File.ReadAllBytes(name.ToString())) != expectedHash)
                { throw new InvalidOperationException("A different native vision runtime is already loaded."); }
                return existing;
            }
            // LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS avoids current-directory lookup.
            IntPtr native = LoadLibraryEx(path, IntPtr.Zero, 0x100 | 0x1000);
            if (native == IntPtr.Zero) { throw new InvalidOperationException("Native vision runtime could not load."); }
            // The process owns this library; unloading can invalidate other managed sessions.
            return native;
        }

        private static string Extract(string directory, string name, string expectedHash, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            string path = Path.Combine(directory, name);
            GuardReparse(path);
            if (File.Exists(path))
            {
                if (VisionModelProfile.Hash(File.ReadAllBytes(path)) != expectedHash)
                { throw new IOException("Vision cache checksum mismatch; remove the damaged cache and retry."); }
                return path;
            }
            byte[] bytes;
            using (var resource = typeof(VisionRuntime).Assembly.GetManifestResourceStream("NOMAD.Vision." + name))
            using (var buffer = new MemoryStream())
            {
                if (resource == null) { throw new IOException("Embedded vision asset is missing."); }
                resource.CopyTo(buffer);
                bytes = buffer.ToArray();
            }
            if (VisionModelProfile.Hash(bytes) != expectedHash)
            { throw new IOException("Embedded vision asset checksum mismatch."); }
            token.ThrowIfCancellationRequested();
            string temporary = Path.Combine(directory, Guid.NewGuid().ToString("N") + ".tmp");
            try
            {
                using (var output = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                { output.Write(bytes, 0, bytes.Length); output.Flush(true); }
                GuardReparse(path);
                try { File.Move(temporary, path); }
                catch (IOException) when (File.Exists(path))
                {
                    GuardReparse(path);
                    if (VisionModelProfile.Hash(File.ReadAllBytes(path)) != expectedHash) { throw; }
                }
            }
            finally { if (File.Exists(temporary)) { File.Delete(temporary); } }
            return path;
        }

        private static void EnsureDirectory(string path)
        {
            GuardReparse(path);
            Directory.CreateDirectory(path);
            GuardReparse(path);
        }

        private static void GuardReparse(string path)
        {
            string current = Path.GetFullPath(path);
            while (!string.IsNullOrEmpty(current))
            {
                if ((File.Exists(current) || Directory.Exists(current)) &&
                    (File.GetAttributes(current) & FileAttributes.ReparsePoint) != 0)
                { throw new IOException("Vision cache cannot use a reparse point."); }
                current = Path.GetDirectoryName(current);
            }
        }

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr LoadLibraryEx(string path, IntPtr file, uint flags);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode)]
        private static extern IntPtr GetModuleHandle(string name);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode)]
        private static extern uint GetModuleFileName(IntPtr module, System.Text.StringBuilder name, int length);
    }
}
