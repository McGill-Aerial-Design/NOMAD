// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    internal sealed class MediaWorkerException : IOException
    { internal MediaWorkerException() : base("NOMAD media worker stopped; tracking was retired.") { } }

    // Only background media owners call Open. UI retirement never waits for a channel or native call.
    internal static class MediaWorkerHost
    {
        private static readonly SemaphoreSlim Launch = new SemaphoreSlim(1, 1);
        private static readonly object Lifecycle = new object();
        private static bool _startRequested;
        private static Task _shutdownCompletion = Task.CompletedTask;
        private static volatile Worker _current;
        private static int _failures, _shutdown;
        private static long _nextEpoch;
        internal static bool FixtureMode;
        internal static bool FixtureHoldResourceHello;
        internal static string FixtureDirectory;
        internal static long WorkerEpoch => _current?.Epoch ?? 0;
        internal static Task ShutdownCompletion => Volatile.Read(ref _shutdownCompletion);
        internal static string DiagnosticStatus
        {
            get
            {
                var current = _current;
                if (current == null || current.IsFailed || current.Control == null) { return "Starting worker"; }
                try { return current.Control.Call(new MediaMessage { Operation = "ping" }, CancellationToken.None).Text; }
                catch (IOException) { return "Worker restarting"; }
            }
        }
        internal static MediaMessage FixtureStats()
        {
            if (!FixtureMode || _current == null) { throw new InvalidOperationException(); }
            return _current.Control.Call(new MediaMessage { Operation = "stats" }, CancellationToken.None);
        }
        internal static event Action Failed;
        internal static bool CanRecover => Volatile.Read(ref _failures) == 1 && Volatile.Read(ref _shutdown) == 0;
        internal static void RequestStart()
        {
            lock (Lifecycle)
            {
                _startRequested = true; Interlocked.Exchange(ref _failures, 0);
                if (ShutdownCompletion.IsCompleted) { Volatile.Write(ref _shutdown, 0); }
            }
        }

        internal static MediaChannel Open(CancellationToken token)
        {
            while (true)
            {
                token.ThrowIfCancellationRequested();
                var shutdown = ShutdownCompletion;
                try { shutdown.Wait(token); }
                catch (AggregateException) when (shutdown.IsFaulted) { }
                Launch.Wait(token);
                try
                {
                    lock (Lifecycle)
                    {
                        if (!ReferenceEquals(shutdown, ShutdownCompletion)) { continue; }
                        if (Volatile.Read(ref _shutdown) != 0 && _startRequested) { Volatile.Write(ref _shutdown, 0); }
                        if (Volatile.Read(ref _shutdown) != 0 || Volatile.Read(ref _failures) > 1)
                        { throw new MediaWorkerException(); }
                    }
                    var current = _current;
                    if (current == null || current.IsFailed)
                    {
                        if (current != null)
                        {
                            if (!current.WaitForExit(2000)) { throw new MediaWorkerException(); }
                            current.Dispose();
                        }
                        _current = current = StartWorker();
                    }
                    MediaPixels.EnablePooling();
                    var channel = new MediaChannel(current, false, token);
                    lock (Lifecycle)
                    {
                        if (ReferenceEquals(shutdown, ShutdownCompletion) && Volatile.Read(ref _shutdown) == 0)
                        { return channel; }
                    }
                    channel.Dispose();
                }
                finally { Launch.Release(); }
            }
        }

        private static Worker StartWorker()
        {
            string bundle = ExtractBundle();
            string secret;
            using (var random = RandomNumberGenerator.Create())
            { var bytes = new byte[24]; random.GetBytes(bytes); secret = BitConverter.ToString(bytes).Replace("-", ""); }
            string pipe = "NOMAD.Media." + secret;
            string data = FixtureDirectory ?? Path.GetDirectoryName(NOMADStorage.PluginsDirectory);
            string gst = FixtureMode ? "" : global::MissionPlanner.Utilities.Settings.Instance["gstlaunchexe"] ?? "";
            var start = new ProcessStartInfo(Path.Combine(bundle, "NOMADMediaWorker.exe"))
            {
                UseShellExecute = false, CreateNoWindow = true, WorkingDirectory = bundle,
                WindowStyle = ProcessWindowStyle.Hidden
            };
            long epoch = Interlocked.Increment(ref _nextEpoch);
            start.Arguments = string.Join(" ", new[] { pipe, secret, Process.GetCurrentProcess().Id.ToString(),
                epoch.ToString(), data, gst, FixtureMode ? "fixture" : "production" }.Select(Quote));
            var job = new NOMAD.Simulation.OwnedProcessJob();
            Process process = null;
            Worker worker = null;
            try
            {
                process = Process.Start(start); job.Attach(process);
                process.PriorityClass = ProcessPriorityClass.BelowNormal;
                worker = new Worker(process, job, pipe, secret, epoch);
                worker.Control = new MediaChannel(worker, true, CancellationToken.None);
                // Native calls must not occupy the pool needed to keep their worker alive.
                Task.Factory.StartNew(worker.Supervise, CancellationToken.None,
                    TaskCreationOptions.LongRunning, TaskScheduler.Default);
                return worker;
            }
            catch (Exception error)
            {
                if (worker == null) { job.Dispose(); process?.Dispose(); }
                else
                {
                    worker.Stop();
                    bool exited = worker.WaitForExit(2000);
                    worker.Dispose();
                    if (!exited) { throw new IOException("Media worker startup cleanup did not complete.", error); }
                }
                throw;
            }
        }

        private static string Quote(string value)
        {
            if (value.IndexOf('"') >= 0 || value.IndexOf('\0') >= 0) { throw new InvalidDataException("Invalid media path."); }
            return "\"" + value.TrimEnd('\\') + "\"";
        }

        internal static void Shutdown()
        {
            lock (Lifecycle)
            {
                _startRequested = false; Volatile.Write(ref _shutdown, 1);
                var prior = ShutdownCompletion;
                _shutdownCompletion = Task.Run(() =>
                {
                    try { prior.GetAwaiter().GetResult(); }
                    catch (Exception) { Log.Warn("Retrying incomplete owned media cleanup."); }
                    Launch.Wait();
                    try
                    {
                        var current = _current;
                        if (current == null) { return; }
                        current.Stop();
                        if (!current.WaitForExit(2000)) { throw new InvalidOperationException("Owned media worker did not exit."); }
                        current.Dispose();
                        _current = null;
                    }
                    finally { MediaPixels.ReleasePool(); Launch.Release(); }
                });
            }
        }

        internal static void CrashFixture()
        {
            if (!FixtureMode || _current == null) { throw new InvalidOperationException(); }
            try { _current.Control.Call(new MediaMessage { Operation = "crash" }, CancellationToken.None); }
            catch (IOException) { }
        }

        internal sealed class Worker : IDisposable
        {
            internal readonly Process Process;
            internal readonly int Id;
            private readonly Task _exited;
            private readonly TaskCompletionSource<bool> _retired =
                new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            internal readonly string Pipe, Secret, Prefix;
            internal readonly long Epoch;
            internal readonly ConcurrentDictionary<long, MediaChannel> Channels = new ConcurrentDictionary<long, MediaChannel>();
            internal readonly ConcurrentDictionary<MediaChannel, long> Handshakes =
                new ConcurrentDictionary<MediaChannel, long>();
            internal readonly ConcurrentQueue<long> Cancellations = new ConcurrentQueue<long>();
            internal MediaChannel Control;
            internal long ControlStarted;
            private readonly NOMAD.Simulation.OwnedProcessJob _job;
            private int _failed, _disposed, _jobClosed;
            internal bool IsFailed => Volatile.Read(ref _failed) == 2;
            internal Worker(Process process, NOMAD.Simulation.OwnedProcessJob job, string pipe, string secret, long epoch)
            {
                Process = process; Id = process.Id; _job = job; Pipe = pipe; Secret = secret; Epoch = epoch;
                Prefix = "Local\\NOMAD.Media." + secret + ".";
                _exited = Task.Factory.StartNew(MonitorExit, CancellationToken.None,
                    TaskCreationOptions.LongRunning, TaskScheduler.Default);
            }
            internal bool WaitForExit(int milliseconds) => _exited.Wait(milliseconds);
            private void CloseJob() { if (Interlocked.Exchange(ref _jobClosed, 1) == 0) { _job.Dispose(); } }

            private void MonitorExit()
            {
                while (!Process.WaitForExit(50))
                {
                    long started = Interlocked.Read(ref ControlStarted);
                    long now = Stopwatch.GetTimestamp();
                    if (started != 0 && (now - started) / (double)Stopwatch.Frequency > 2 &&
                        Interlocked.Read(ref ControlStarted) == started)
                    { Fail(); }
                    foreach (var hello in Handshakes)
                    {
                        if ((now - hello.Value) / (double)Stopwatch.Frequency > 2 && Handshakes.TryRemove(hello.Key, out _))
                        { Fail(); hello.Key.CloseTransport(); }
                    }
                }
            }

            internal void Supervise()
            {
                try
                {
                    while (!IsFailed && Volatile.Read(ref _shutdown) == 0)
                    {
                        while (Cancellations.TryDequeue(out long resource))
                        { Control.Call(new MediaMessage { Operation = "cancel", Ticks = resource }, CancellationToken.None); }
                        Control.Call(new MediaMessage { Operation = "ping" }, CancellationToken.None);
                        Thread.Sleep(250);
                    }
                }
                catch { if (Volatile.Read(ref _shutdown) == 0) { Fail(); } }
            }

            internal void Fail()
            {
                if (Interlocked.CompareExchange(ref _failed, 1, 0) != 0)
                {
                    if (Volatile.Read(ref _failed) == 1) { _retired.Task.Wait(2000); }
                    return;
                }
                Action handlers = null;
                lock (Lifecycle)
                {
                    if (Volatile.Read(ref _shutdown) == 0 && Volatile.Read(ref _failed) == 1)
                    { Interlocked.Increment(ref _failures); handlers = Failed; }
                }
                if (handlers != null)
                { foreach (Action handler in handlers.GetInvocationList()) { try { handler(); } catch { } } }
                Volatile.Write(ref _failed, 2); _retired.TrySetResult(true);
                CloseJob();
                foreach (var channel in Channels.Values) { channel.CloseTransport(); }
                foreach (var channel in Handshakes.Keys) { channel.CloseTransport(); }
                Control?.CloseTransport();
            }

            internal void Stop() { Interlocked.Exchange(ref _failed, 2); _retired.TrySetResult(true); CloseJob();
                foreach (var channel in Channels.Values) { channel.CloseTransport(); }
                foreach (var channel in Handshakes.Keys) { channel.CloseTransport(); }
                Control?.CloseTransport(); }
            public void Dispose()
            {
                if (Interlocked.Exchange(ref _disposed, 1) != 0) { return; }
                Stop();
                _exited.ContinueWith(_ => Process.Dispose(), CancellationToken.None,
                    TaskContinuationOptions.ExecuteSynchronously, TaskScheduler.Default);
            }
        }

        private static string ExtractBundle()
        {
            var assembly = typeof(MediaWorkerHost).Assembly;
            string text;
            using (var stream = assembly.GetManifestResourceStream("NOMAD.Media.bundle.json"))
            {
                if (stream == null || stream.Length > 65536) { throw new IOException("Media bundle is missing."); }
                using (var reader = new StreamReader(stream)) { text = reader.ReadToEnd(); }
            }
            var files = JsonConvert.DeserializeObject<Dictionary<string, string>>(text);
            if (files == null || files.Count != 7 || !files.ContainsKey("NOMADMediaWorker.exe"))
            { throw new IOException("Media bundle is invalid."); }
            var allowed = new[] { "NOMADMediaWorker.exe", "NOMADMediaWorker.exe.config", "Newtonsoft.Json.dll",
                "System.Memory.dll", "System.Buffers.dll", "System.Numerics.Vectors.dll", "System.Runtime.CompilerServices.Unsafe.dll" };
            if (files.Keys.Any(name => !allowed.Contains(name)) || files.Values.Any(hash => hash == null ||
                hash.Length != 64 || hash.Any(c => !Uri.IsHexDigit(c)))) { throw new IOException("Media bundle is invalid."); }
            string root = FixtureDirectory ?? Path.GetDirectoryName(NOMADStorage.PluginsDirectory);
            string identity;
            using (var sha = SHA256.Create())
            { identity = BitConverter.ToString(sha.ComputeHash(System.Text.Encoding.UTF8.GetBytes(text))).Replace("-", ""); }
            string directory = Path.Combine(root, "media", identity);
            GuardPath(directory); Directory.CreateDirectory(directory); GuardPath(directory);
            foreach (var entry in files)
            {
                if (Path.GetFileName(entry.Key) != entry.Key || entry.Value.Length != 64 ||
                    entry.Value.Any(c => !Uri.IsHexDigit(c))) { throw new IOException("Media bundle is invalid."); }
                string path = Path.Combine(directory, entry.Key); GuardPath(path);
                if (File.Exists(path))
                { if (!SameHash(path, entry.Value)) { throw new IOException("Media bundle cache is damaged."); } continue; }
                string temporary = Path.Combine(directory, Guid.NewGuid().ToString("N") + ".tmp");
                try
                {
                    using (var resource = assembly.GetManifestResourceStream("NOMAD.Media." + entry.Key))
                    using (var output = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                    { if (resource == null) { throw new IOException("Media bundle asset is missing."); }
                      resource.CopyTo(output); output.Flush(true); }
                    if (!SameHash(temporary, entry.Value)) { throw new IOException("Media bundle checksum mismatch."); }
                    GuardPath(path);
                    try { File.Move(temporary, path); }
                    catch (IOException) when (File.Exists(path) && SameHash(path, entry.Value)) { }
                }
                finally { if (File.Exists(temporary)) { File.Delete(temporary); } }
            }
            return directory;
        }

        private static bool SameHash(string path, string expected)
        {
            GuardPath(path);
            using (var input = File.OpenRead(path))
            using (var sha = SHA256.Create())
            { return BitConverter.ToString(sha.ComputeHash(input)).Replace("-", "").Equals(expected, StringComparison.OrdinalIgnoreCase); }
        }

        private static void GuardPath(string path)
        {
            for (string current = Path.GetFullPath(path); current != null; current = Path.GetDirectoryName(current))
            { if ((File.Exists(current) || Directory.Exists(current)) &&
                (File.GetAttributes(current) & FileAttributes.ReparsePoint) != 0)
              { throw new IOException("Media cache cannot use a reparse point."); } }
        }
    }

    internal sealed class MediaChannel : IDisposable
    {
        private readonly object _callGate = new object();
        private readonly object _bufferGate = new object();
        private readonly MediaWorkerHost.Worker _owner;
        private readonly NamedPipeClientStream _pipe;
        private readonly bool _control;
        private System.IO.MemoryMappedFiles.MemoryMappedFile _buffer;
        private string _mapping;
        private int _capacity, _disposed;
        private long _request, _resource;
        internal string Prefix => _owner.Prefix;
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool GetNamedPipeServerProcessId(Microsoft.Win32.SafeHandles.SafePipeHandle pipe, out uint id);

        internal MediaChannel(MediaWorkerHost.Worker owner, bool control, CancellationToken token)
        {
            _owner = owner;
            _control = control;
            _pipe = new NamedPipeClientStream(".", owner.Pipe, PipeDirection.InOut, PipeOptions.Asynchronous);
            try
            {
                var connecting = Stopwatch.StartNew();
                // Framework ConnectAsync queues pool work even when its caller already owns a dedicated thread.
                while (true)
                {
                    token.ThrowIfCancellationRequested();
                    try
                    {
                        _pipe.Connect(Math.Max(1, Math.Min(50, 2000 - (int)connecting.ElapsedMilliseconds)));
                        break;
                    }
                    catch (TimeoutException) when (connecting.ElapsedMilliseconds < 2000) { }
                }
                if (!GetNamedPipeServerProcessId(_pipe.SafePipeHandle, out uint server) || server != owner.Id)
                { throw new IOException("Unexpected media process."); }
                owner.Handshakes.TryAdd(this, Stopwatch.GetTimestamp());
                try
                {
                    var hello = Call(new MediaMessage { Operation = "hello", Secret = owner.Secret,
                        Text = control ? "control" : MediaWorkerHost.FixtureMode && MediaWorkerHost.FixtureHoldResourceHello
                            ? "fixture:held-hello" : "resource" }, token);
                    if (hello.Text != NomadRelease.SourceSha || (control ? hello.Resource != 0 : hello.Resource <= 0))
                    { throw new IOException("Media worker version mismatch."); }
                    _resource = hello.Resource;
                    if (!control && !owner.Channels.TryAdd(_resource, this))
                    { throw new IOException("Duplicate media resource."); }
                    token.ThrowIfCancellationRequested();
                    if (owner.IsFailed || Volatile.Read(ref _disposed) != 0) { throw new MediaWorkerException(); }
                }
                finally { owner.Handshakes.TryRemove(this, out _); }
            }
            catch (TimeoutException) { CloseTransport(); owner.Fail(); throw new MediaWorkerException(); }
            catch { CloseTransport(); throw; }
        }

        internal MediaMessage Call(MediaMessage request, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            lock (_callGate)
            {
                if (Volatile.Read(ref _disposed) != 0 || _owner.IsFailed) { throw new MediaWorkerException(); }
                request.Request = ++_request; request.Resource = _resource; request.Epoch = _owner.Epoch;
                using (token.Register(Dispose))
                {
                    try
                    {
                        if (_control) { Interlocked.Exchange(ref _owner.ControlStarted, Stopwatch.GetTimestamp()); }
                        using (var deadline = new Timer(_ => _owner.Fail(), null, 2000, Timeout.Infinite))
                        { MediaWire.Write(_pipe, request); deadline.Change(Timeout.Infinite, Timeout.Infinite); }
                        var reply = MediaWire.Read(_pipe);
                        if (reply.Request != request.Request || reply.Epoch != _owner.Epoch ||
                            (request.Operation != "hello" && reply.Resource != _resource))
                        { throw new InvalidDataException("Stale media reply."); }
                        token.ThrowIfCancellationRequested();
                        if (reply.Error != null)
                        { if (reply.Error == "canceled") { throw new OperationCanceledException(token); }
                          throw new InvalidOperationException(MediaWire.OperatorError(reply.Error)); }
                        return reply;
                    }
                    catch (IOException)
                    {
                        token.ThrowIfCancellationRequested();
                        if (Volatile.Read(ref _disposed) == 0) { _owner.Fail(); }
                        throw new MediaWorkerException();
                    }
                    catch (ObjectDisposedException)
                    { token.ThrowIfCancellationRequested(); throw new MediaWorkerException(); }
                    finally { if (_control) { Interlocked.Exchange(ref _owner.ControlStarted, 0); } }
                }
            }
        }

        internal void Buffer(MediaMessage message, int width, int height)
        {
            lock (_bufferGate)
            {
                if (_owner.IsFailed || Volatile.Read(ref _disposed) != 0) { throw new MediaWorkerException(); }
                int bytes = MediaWire.PixelBytes(width, height);
                if (_capacity < bytes)
                {
                    _buffer?.Dispose(); _mapping = Prefix + Guid.NewGuid().ToString("N");
                    _buffer = MediaPixels.Create(_mapping, bytes); _capacity = bytes;
                }
                message.Mapping = _mapping; message.Capacity = _capacity; message.Width = width; message.Height = height;
            }
        }

        internal MediaMessage ImageCall(string operation, System.Drawing.Bitmap image, CancellationToken token,
            System.Drawing.RectangleF? bounds = null, VisualFrameContext context = default)
        {
            var message = new MediaMessage { Operation = operation, Bounds = bounds, Hint = context.SearchHint,
                Discontinuous = context.Discontinuous, Ticks = MediaPixels.ReceivedTicks(image) };
            if (_owner.IsFailed || Volatile.Read(ref _disposed) != 0) { throw new MediaWorkerException(); }
            try
            {
                Buffer(message, image.Width, image.Height); MediaPixels.Write(image, message, Prefix);
                return Call(message, token);
            }
            catch (IOException)
            { token.ThrowIfCancellationRequested(); if (!_owner.IsFailed) { _owner.Fail(); } throw new MediaWorkerException(); }
        }

        internal System.Drawing.Bitmap ReadFrame(CancellationToken token, out long ticks, out DateTime utc)
        {
            var request = new MediaMessage { Operation = "read", Mapping = _mapping, Capacity = _capacity };
            var reply = Call(request, token);
            if (reply.End) { ticks = 0; utc = default; return null; }
            if (reply.NeedBuffer)
            { request = new MediaMessage { Operation = "fetch" }; Buffer(request, reply.Width, reply.Height); reply = Call(request, token); }
            if (reply.NeedBuffer || reply.Ticks <= 0 || reply.Ticks > Stopwatch.GetTimestamp() || reply.UtcTicks < 0 ||
                reply.UtcTicks > DateTime.MaxValue.Ticks) { _owner.Fail(); throw new MediaWorkerException(); }
            reply.Mapping = _mapping; reply.Capacity = _capacity;
            var result = MediaPixels.Read(reply, Prefix); ticks = reply.Ticks; utc = new DateTime(reply.UtcTicks, DateTimeKind.Utc);
            return result;
        }

        internal void StopAndWait()
        {
            Dispose();
            if (_resource == 0) { return; }
            var deadline = Stopwatch.StartNew();
            while (!_owner.IsFailed && deadline.ElapsedMilliseconds < 2500)
            {
                try
                {
                    var reply = _owner.Control.Call(new MediaMessage { Operation = "closed", Ticks = _resource },
                        CancellationToken.None);
                    if (reply.End) { return; }
                }
                catch (IOException) { break; }
                Thread.Sleep(20);
            }
            if (!_owner.IsFailed) { _owner.Fail(); }
            if (!_owner.WaitForExit(2000)) { throw new IOException("Media cleanup did not complete."); }
        }

        internal void CloseTransport()
        {
            if (Interlocked.Exchange(ref _disposed, 1) != 0) { return; }
            try { _pipe.Dispose(); } catch { }
            lock (_bufferGate) { _buffer?.Dispose(); _buffer = null; }
            var registration = new KeyValuePair<long, MediaChannel>(_resource, this);
            ((ICollection<KeyValuePair<long, MediaChannel>>)_owner.Channels).Remove(registration);
        }
        public void Dispose()
        {
            if (Volatile.Read(ref _disposed) != 0) { return; }
            if (_resource != 0 && !_owner.IsFailed) { _owner.Cancellations.Enqueue(_resource); }
            CloseTransport();
        }
    }
}
