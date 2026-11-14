// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.AccessControl;
using System.Security.Principal;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    internal static class NOMADStorage
    { internal static string PluginsDirectory => Path.Combine(MediaServer.CacheRoot, "plugins"); }

    internal static class NativeProgress
    {
        [ThreadStatic] internal static Action Current;
        internal static void Touch() { Current?.Invoke(); }
    }

    internal static class MediaServer
    {
        internal static string CacheRoot;
        private static string _pipe, _secret, _prefix, _gst;
        private static int _parent;
        private static long _epoch, _next, _heartbeat;
        private static bool _fixture;
        private static readonly ConcurrentDictionary<long, Resource> Resources = new ConcurrentDictionary<long, Resource>();
        private static readonly object GpuGate = new object();

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool GetNamedPipeClientProcessId(Microsoft.Win32.SafeHandles.SafePipeHandle pipe, out uint id);

        private static int Main(string[] args)
        {
            if (args.Length != 7 || !int.TryParse(args[2], out _parent) || !long.TryParse(args[3], out _epoch)) { return 64; }
            _pipe = args[0]; _secret = args[1]; CacheRoot = Path.GetFullPath(args[4]); _gst = args[5];
            _fixture = args[6] == "fixture"; _prefix = "Local\\NOMAD.Media." + _secret + ".";
            _heartbeat = Stopwatch.GetTimestamp();
            Task.Factory.StartNew(Watch, TaskCreationOptions.LongRunning);
            try
            {
                while (true)
                {
                    var security = new PipeSecurity();
                    security.SetAccessRuleProtection(true, false);
                    security.AddAccessRule(new PipeAccessRule(WindowsIdentity.GetCurrent().User,
                        PipeAccessRights.FullControl, AccessControlType.Allow));
                    var stream = new NamedPipeServerStream(_pipe, PipeDirection.InOut, 64, PipeTransmissionMode.Byte,
                        PipeOptions.Asynchronous, 65536, 65536, security);
                    stream.WaitForConnection();
                    if (!GetNamedPipeClientProcessId(stream.SafePipeHandle, out uint client) || client != _parent ||
                        Resources.Count >= 60)
                    { stream.Dispose(); continue; }
                    Task.Factory.StartNew(() => Serve(stream), TaskCreationOptions.LongRunning);
                }
            }
            catch { return 70; }
        }

        private static void Watch()
        {
            while (true)
            {
                long now = Stopwatch.GetTimestamp();
                if (Seconds(now - Interlocked.Read(ref _heartbeat)) > 2) { Environment.Exit(71); }
                foreach (var resource in Resources.Values)
                {
                    long closing = Interlocked.Read(ref resource.Closing);
                    if (closing != 0 && Seconds(now - closing) > 2) { Environment.Exit(72); }
                    long active = Interlocked.Read(ref resource.Active);
                    if (active != 0 && Seconds(now - active) * 1000 > Volatile.Read(ref resource.Budget))
                    { Environment.Exit(73); }
                }
                Thread.Sleep(50);
            }
        }

        private static double Seconds(long ticks) => ticks / (double)Stopwatch.Frequency;

        private static void Serve(NamedPipeServerStream stream)
        {
            Resource resource = null;
            try
            {
                var hello = MediaWire.Read(stream);
                if (hello.Operation != "hello" || hello.Secret != _secret || hello.Epoch != _epoch) { return; }
                bool control = hello.Text == "control";
                if (control) { Interlocked.Exchange(ref _heartbeat, Stopwatch.GetTimestamp()); }
                if (!control)
                {
                    resource = new Resource(Interlocked.Increment(ref _next));
                    if (!Resources.TryAdd(resource.Id, resource)) { return; }
                    if (_fixture && hello.Text == "fixture:held-hello")
                    { resource.LastOperation = "held hello"; Thread.Sleep(Timeout.Infinite); }
                }
                MediaWire.Write(stream, new MediaMessage
                { Request = hello.Request, Resource = resource?.Id ?? 0, Epoch = _epoch, Text = NomadRelease.SourceSha });
                while (stream.IsConnected)
                {
                    var request = MediaWire.Read(stream);
                    if (request.Epoch != _epoch || request.Resource != (resource?.Id ?? 0)) { return; }
                    var reply = new MediaMessage { Request = request.Request, Resource = request.Resource, Epoch = _epoch };
                    if (control)
                    {
                        if (request.Operation == "ping")
                        {
                            Interlocked.Exchange(ref _heartbeat, Stopwatch.GetTimestamp());
                            reply.Text = string.Join(",", Resources.Values.Select(value => value.LastOperation + ":" +
                                (value.Active == 0 ? "idle" : Seconds(Stopwatch.GetTimestamp() - value.Active).ToString("F1"))));
                        }
                        else if (request.Operation == "closed") { reply.End = !Resources.ContainsKey(request.Ticks); }
                        else if (_fixture && request.Operation == "stats")
                        {
                            reply.Width = Resources.Values.Count(value => value.Kind == "pipeline");
                            reply.Height = Resources.Values.Count(value => value.Kind == "tracker");
                            reply.Mode = Resources.Values.Count(value => value.Kind == "detector");
                            reply.Ticks = MediaPixels.AllocatedBytes;
                            reply.Text = string.Join(",", Resources.Values.Where(value => Interlocked.Read(ref value.Active) != 0)
                                .Select(value => value.LastOperation));
                        }
                        else if (request.Operation == "cancel")
                        {
                            if (Resources.TryGetValue(request.Ticks, out var canceled)) { canceled.Cancel(); }
                        }
                        else if (request.Operation == "stop") { Environment.Exit(0); }
                        else if (_fixture && request.Operation == "crash") { Environment.Exit(74); }
                        else { throw new InvalidDataException(); }
                    }
                    else
                    {
                        try { resource.Execute(request, reply); }
                        catch (Exception error)
                        {
                            reply.Error = error is OperationCanceledException ? "canceled" :
                                MediaWire.OperatorError(error.GetBaseException().Message);
                        }
                        finally { Interlocked.Exchange(ref resource.Active, 0); NativeProgress.Current = null; }
                    }
                    MediaWire.Write(stream, reply);
                    if (resource?.Cancellation.IsCancellationRequested == true) { break; }
                }
            }
            catch (IOException) { }
            catch (Exception) { }
            finally
            {
                stream.Dispose();
                if (resource != null)
                {
                    resource.Begin(2000);
                    try { resource.Dispose(); }
                    catch { }
                    Resources.TryRemove(resource.Id, out _);
                }
            }
        }

        private sealed class Resource : IDisposable
        {
            internal readonly long Id;
            internal readonly CancellationTokenSource Cancellation = new CancellationTokenSource();
            internal long Active, Closing;
            internal int Budget;
            private IVideoPipeline _pipeline;
            private IVideoDetector _detector;
            private IVisualTracker _tracker;
            private bool _gpu, _fixtureDetector, _fixtureTracker;
            private string _fixtureModel, _kind;
            private int _fixturePeriod = 33;
            internal string LastOperation;
            private Bitmap _pending;
            private long _ticks;
            private DateTime _utc;

            internal string Kind => _kind;
            internal Resource(long id) { Id = id; }
            internal void Begin(int budget)
            { Volatile.Write(ref Budget, budget); Interlocked.Exchange(ref Active, Stopwatch.GetTimestamp()); }
            internal void Cancel()
            {
                if (Interlocked.CompareExchange(ref Closing, Stopwatch.GetTimestamp(), 0) != 0) { return; }
                Task.Run(() => { try { Cancellation.Cancel(); } catch { } });
            }

            private void HoldFixture()
            {
                LastOperation = "held " + LastOperation;
                Thread.Sleep(Timeout.Infinite);
            }

            internal void Execute(MediaMessage request, MediaMessage reply)
            {
                var token = Cancellation.Token; token.ThrowIfCancellationRequested();
                LastOperation = request.Operation;
                Begin(2000);
                NativeProgress.Current = () => Interlocked.Exchange(ref Active, Stopwatch.GetTimestamp());
                if (request.Operation == "pipeline" || request.Operation == "detector" || request.Operation == "tracker")
                { if (_kind != null) { throw new InvalidDataException(); } _kind = request.Operation; }
                switch (request.Operation)
                {
                    case "pipeline":
                        Begin(120000);
                        if (!_fixture) { GStreamerNative.Configure(_gst); _pipeline = new GStreamerVideoPipeline(); }
                        break;
                    case "start":
                        if (_fixture)
                        {
                            _fixtureModel = request.Text;
                            _fixturePeriod = request.Text == "fixture:120" ? 8 : request.Text == "fixture:60" ? 16 : 33;
                            if (_fixtureModel == "fixture:hang-start") { HoldFixture(); }
                        }
                        else { _pipeline.Start(request.Text, token); }
                        break;
                    case "read":
                        if (_pending != null) { throw new InvalidDataException(); }
                        if (_fixture)
                        {
                            if (_fixtureModel == "fixture:hang-read") { HoldFixture(); }
                            Thread.Sleep(_fixturePeriod); token.ThrowIfCancellationRequested();
                            _pending = new Bitmap(64, 48); _ticks = Stopwatch.GetTimestamp(); _utc = DateTime.UtcNow;
                        }
                        else { _pending = _pipeline.ReadFrame(token, out _ticks, out _utc); }
                        if (_pending == null) { reply.End = true; break; }
                        Deliver(request, reply);
                        break;
                    case "fetch":
                        if (_pending == null) { throw new InvalidDataException(); }
                        Deliver(request, reply);
                        break;
                    case "detector":
                        Begin(120000); _fixtureModel = request.Model;
                        if (_fixture && request.Model == "fixture:no-nvidia")
                        { throw new InvalidOperationException("Tracker load failed", new NotSupportedException(MediaWire.NvidiaUnavailable)); }
                        if (_fixture && request.Model == "fixture:vc-runtime")
                        { throw new InvalidOperationException(MediaWire.VisualCppRequired); }
                        if (_fixture && request.Model == "fixture:private-error")
                        { throw new IOException("private model path and camera credentials"); }
                        if (_fixture && request.Model == "fixture:hang-load") { HoldFixture(); }
                        if (_fixture && request.Model.StartsWith("fixture:", StringComparison.Ordinal))
                        { _fixtureDetector = true; reply.Text = "fixture"; }
                        else { _detector = VisionDetectorFactory.Create(request.Model, token); reply.Text = _detector.Identity; }
                        break;
                    case "detect":
                        var frame = MediaPixels.Read(request, _prefix);
                        try
                        {
                            if (_fixtureDetector)
                            {
                                if (_fixtureModel == "fixture:hang") { HoldFixture(); }
                                reply.Detections = Array.Empty<MediaDetection>();
                            }
                            else { reply.Detections = _detector.Detect(frame, token).Select(MediaDetection.From).ToArray(); }
                        }
                        finally { MediaPixels.Dispose(frame); }
                        break;
                    case "tracker":
                        Begin(120000); _gpu = request.Gpu; _fixtureTracker = _fixture;
                        if (_fixture && _gpu) { HoldFixture(); }
                        if (!_fixtureTracker) { _tracker = VisualTrackerFactory.Create(token, (VisualTrackingMode)request.Mode, _gpu); }
                        reply.Text = (_tracker as EfficientTamVisualTracker)?.Provider ?? "CPU";
                        break;
                    case "initialize":
                    case "track":
                        var trackingFrame = MediaPixels.Read(request, _prefix);
                        try
                        {
                            Action run = () =>
                            {
                                Begin(request.Operation == "initialize" ? 5000 : 2000);
                                token.ThrowIfCancellationRequested();
                                var context = new VisualFrameContext(request.Hint, request.Discontinuous);
                                if (_fixtureTracker && request.Operation == "initialize")
                                {
                                    _fixtureModel = request.Bounds?.X == 14 ? "fixture:hang-track" : "";
                                    if (request.Bounds?.X == 13) { HoldFixture(); }
                                }
                                if (_fixtureTracker && request.Operation == "track" && _fixtureModel == "fixture:hang-track")
                                { HoldFixture(); }
                                if (_fixtureTracker)
                                { reply.State = (int)VisualTrackingState.Verified; reply.Bounds = new RectangleF(8, 8, 16, 16);
                                  reply.Aim = new PointF(16, 16); reply.Text = "Verified fixture"; }
                                else if (request.Operation == "initialize")
                                { _tracker.Initialize(trackingFrame, request.Bounds.Value, token, context); }
                                else
                                {
                                    var result = _tracker.Update(trackingFrame, token, context);
                                    reply.State = (int)result.State; reply.Bounds = result.Bounds;
                                    reply.Aim = result.AimPoint; reply.Hint = result.MotionSearchHint; reply.Text = result.Status;
                                }
                            };
                            if (_gpu) { Begin(120000); lock (GpuGate) { run(); } } else { run(); }
                        }
                        finally { MediaPixels.Dispose(trackingFrame); }
                        break;
                    case "builtin":
                        Begin(120000); VisionRuntime.EnsureLoaded(token); reply.Text = VisionRuntime.BuiltinModelPath;
                        break;
                    case "validate":
                        Begin(120000);
                        var profile = Newtonsoft.Json.JsonConvert.DeserializeObject<VisionModelProfile>(request.Text);
                        VisionDetectorFactory.ValidateModel(profile, token);
                        break;
                    default: throw new InvalidDataException("Unknown media operation.");
                }
            }

            private void Deliver(MediaMessage request, MediaMessage reply)
            {
                reply.Width = _pending.Width; reply.Height = _pending.Height; reply.Ticks = _ticks;
                reply.UtcTicks = _utc.Ticks;
                int bytes = MediaWire.PixelBytes(reply.Width, reply.Height);
                if (request.Mapping == null || request.Capacity < bytes) { reply.NeedBuffer = true; return; }
                request.Width = reply.Width; request.Height = reply.Height;
                MediaPixels.Write(_pending, request, _prefix);
                MediaPixels.Dispose(_pending); _pending = null;
            }

            public void Dispose()
            {
                LastOperation = "dispose";
                Cancel();
                if (_fixture && _fixtureModel == "fixture:hang-dispose") { HoldFixture(); }
                try { _tracker?.Dispose(); _detector?.Dispose(); _pipeline?.Dispose(); }
                finally { MediaPixels.Dispose(_pending); _pending = null; Cancellation.Dispose(); }
            }
        }
    }
}
