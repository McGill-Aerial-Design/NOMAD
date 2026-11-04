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
        // ONNX Runtime 1.24.4 DirectML build: its CPU provider runs the detector; DirectML serves the opt-in GPU tracker.
        private const string ManagedHash = "5c3c531af36a6cb4baa01db20dcf94a0464ec36f8090aa42f35ff565b90a1ea6";
        // Native ORT files have their isolation manifest disabled (see prepare_vision.ps1 and CRuntimeContext).
        private const string NativeHash = "35d9ffb1351e4151e1b147a06750c4a66ac17ebd6d16c11be60a05d5ed8d6c9d";
        private const string ProvidersHash = "fc74b2954f44e618c60194ba788d918681fe7571132efcaa7ea39b32791d7e89";
        private const string DirectMlHash = "9c9e6d822561c6c41b90e6994b3e8857cf1d66dbfb1e0c4c799c7c89b4e92da1";
        private static readonly object Gate = new object();
        private static Assembly _managed;
        private static IntPtr _native, _directMl;
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
                    "vision", "onnxruntime-1.24.4-dml-" + NativeHash.Substring(0, 12));
                EnsureDirectory(cache);
                string managed = Extract(cache, "Microsoft.ML.OnnxRuntime.dll", ManagedHash, token);
                string native = Extract(cache, "onnxruntime.dll", NativeHash, token);
                string providers = Extract(cache, "onnxruntime_providers_shared.dll", ProvidersHash, token);
                BuiltinModelPath = Extract(cache, "yolox_nano.onnx", VisionModelProfile.BuiltinHash, token);
                Extract(cache, "ONNX-RUNTIME-LICENSE.txt",
                    "c250d6278f0b47a6439fb7592b08b58a55eb9f535aa49a1db63211c3f982b674", token);
                Extract(cache, "ONNX-RUNTIME-NOTICES.txt",
                    "fb0af774b4d7cffc5b9d046f2aaeade2f37df2f80abf8033c95dfffcc77a8866", token);
                Extract(cache, "YOLOX-LICENSE.txt",
                    "577c03d505ec80f667ebf96ebd0cc4f6825c817ca1088ee348c48aaabd51bd92", token);
                IntPtr context = CRuntimeContext(cache);
                if (!ActivateActCtx(context, out IntPtr cookie))
                { throw new InvalidOperationException("Vision runtime context could not activate."); }
                try
                {
                    _native = LoadNative(native, "onnxruntime.dll", NativeHash);
                    LoadNative(providers, "onnxruntime_providers_shared.dll", ProvidersHash);
                }
                finally { DeactivateActCtx(0, cookie); }
                var existing = AppDomain.CurrentDomain.GetAssemblies()
                    .FirstOrDefault(assembly => assembly.GetName().Name == "Microsoft.ML.OnnxRuntime");
                if (existing != null && (string.IsNullOrEmpty(existing.Location) ||
                    VisionModelProfile.Hash(File.ReadAllBytes(existing.Location)) != ManagedHash))
                { throw new InvalidOperationException("A different ONNX Runtime is already loaded."); }
                _managed = existing ?? Assembly.Load(File.ReadAllBytes(managed));
            }
        }

        // Loads the pinned DirectML redistributable by path before any DirectML session, so the provider never binds
        // the operating system's copy. Only the GPU tracker calls this; a conflict cannot disable the CPU detector.
        internal static void EnsureDirectMlLoaded(CancellationToken token)
        {
            EnsureLoaded(token);
            lock (Gate)
            {
                token.ThrowIfCancellationRequested();
                if (_directMl != IntPtr.Zero) { return; }
                string cache = Path.Combine(Path.GetDirectoryName(NOMADStorage.PluginsDirectory),
                    "vision", "directml-1.15.4-" + DirectMlHash.Substring(0, 12));
                EnsureDirectory(cache);
                string native = Extract(cache, "DirectML.dll", DirectMlHash, token);
                Extract(cache, "DirectML-LICENSE.txt",
                    "a05138e3a085ff60a44881eedfa58dccb03ecc1d7b1f6ae888418e8c2fec4b8d", token);
                Extract(cache, "DirectML-NOTICES.txt",
                    "2c95795c13ff48a58b6ed916f37901c23d964b5d9d601af422f17ad2172e7950", token);
                _directMl = LoadNative(native, "DirectML.dll", DirectMlHash);
            }
        }

        // Extracts the embedded, tested EfficientTAM export (and its licence) to the hash-checked vision cache.
        internal static string EnsureTrackerModel(CancellationToken token)
        {
            lock (Gate)
            {
                string cache = Path.Combine(Path.GetDirectoryName(NOMADStorage.PluginsDirectory), "vision",
                    "efficienttam-ti512-" + EfficientTamCore.Hashes["encoder.onnx"].Substring(0, 12));
                EnsureDirectory(cache);
                foreach (var file in EfficientTamCore.Hashes) { Extract(cache, file.Key, file.Value, token); }
                Extract(cache, "EfficientTAM-LICENSE.txt",
                    "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4", token);
                return cache;
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

        private static readonly string[] CRuntime = { "vcruntime140.dll", "vcruntime140_1.dll", "msvcp140.dll", "msvcp140_1.dll" };

        // Mission Planner's GDAL loads a 2017 Visual C++ runtime before plugins start, and Windows reuses a loaded DLL by
        // name, so ONNX Runtime (built with toolset 14.44) failed with error 1114. ORT therefore loads inside this
        // activation context, which redirects its C++ runtime to a private copy of the installed runtime. GDAL keeps
        // its own; ORT's C API shares no C++ runtime objects with either side.
        private static IntPtr CRuntimeContext(string cache)
        {
            string assembly = Path.Combine(cache, "NOMAD.VisionCrt");
            EnsureDirectory(assembly);
            string system = Environment.GetFolderPath(Environment.SpecialFolder.System);
            foreach (string name in CRuntime)
            {
                string source = Path.Combine(system, name), target = Path.Combine(assembly, name);
                var version = File.Exists(source) ? System.Diagnostics.FileVersionInfo.GetVersionInfo(source) : null;
                if (version == null || version.FileMajorPart != 14 || version.FileMinorPart < 44)
                {
                    throw new InvalidOperationException("Vision needs the Microsoft Visual C++ 2015-2022 " +
                        "Redistributable (x64) 14.44 or later. Install the latest version from Microsoft.");
                }
                GuardReparse(target);
                if (!File.Exists(target) || new FileInfo(target).Length != new FileInfo(source).Length ||
                    System.Diagnostics.FileVersionInfo.GetVersionInfo(target).FileVersion != version.FileVersion)
                {
                    string temporary = target + "." + Guid.NewGuid().ToString("N") + ".tmp";
                    try { File.Copy(source, temporary); File.Copy(temporary, target, true); }
                    finally { if (File.Exists(temporary)) { File.Delete(temporary); } }
                }
            }
            const string head = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>" +
                "<assembly xmlns=\"urn:schemas-microsoft-com:asm.v1\" manifestVersion=\"1.0\">";
            const string identity = "<assemblyIdentity type=\"win32\" name=\"NOMAD.VisionCrt\" version=\"1.0.0.0\" " +
                "processorArchitecture=\"amd64\"/>";
            WriteText(Path.Combine(assembly, "NOMAD.VisionCrt.manifest"), head + identity +
                string.Concat(CRuntime.Select(name => "<file name=\"" + name + "\"/>")) + "</assembly>");
            string root = Path.Combine(cache, "vision.manifest");
            WriteText(root, head + "<assemblyIdentity type=\"win32\" name=\"NOMAD.Vision\" version=\"1.0.0.0\" " +
                "processorArchitecture=\"amd64\"/><dependency><dependentAssembly>" + identity +
                "</dependentAssembly></dependency></assembly>");
            var request = new ActivationContextRequest
            {
                Size = Marshal.SizeOf(typeof(ActivationContextRequest)), Flags = 0x4, // ACTCTX_FLAG_ASSEMBLY_DIRECTORY_VALID
                Source = root, AssemblyDirectory = cache,
            };
            IntPtr context = CreateActCtx(ref request);
            if (context == new IntPtr(-1))
            {
                throw new InvalidOperationException("Vision runtime context could not be created (Windows error " +
                    Marshal.GetLastWin32Error() + ").");
            }
            return context; // Kept for the process, like the native runtime it served.
        }

        private static void WriteText(string path, string text)
        {
            GuardReparse(path);
            if (!File.Exists(path) || File.ReadAllText(path) != text) { File.WriteAllText(path, text); }
        }

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        private struct ActivationContextRequest
        {
            internal int Size;
            internal uint Flags;
            internal string Source;
            internal ushort Architecture, Language;
            internal string AssemblyDirectory, ResourceName, ApplicationName;
            internal IntPtr Module;
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
            if (native == IntPtr.Zero)
            {
                int error = Marshal.GetLastWin32Error();
                throw new InvalidOperationException("Native vision runtime could not load (Windows error " + error + ").");
            }
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
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr CreateActCtx(ref ActivationContextRequest request);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool ActivateActCtx(IntPtr context, out IntPtr cookie);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool DeactivateActCtx(uint flags, IntPtr cookie);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode)]
        private static extern uint GetModuleFileName(IntPtr module, System.Text.StringBuilder name, int length);
    }
}
