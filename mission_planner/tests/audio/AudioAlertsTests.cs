// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Concurrent;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using NOMAD.MissionPlanner;

// Compile-time device doubles keep this production-source test silent and deterministic.
namespace System.Media
{
    public sealed class SoundPlayer : IDisposable
    {
        public static readonly ConcurrentQueue<SoundPlayer> Players = new ConcurrentQueue<SoundPlayer>();
        public Stream Stream { get; }
        public bool Stopped, Disposed;
        public SoundPlayer(Stream stream) { Stream = stream; Players.Enqueue(this); }
        public void PlayLooping() { }
        public void PlaySync() { }
        public void Stop() { Stopped = true; }
        public void Dispose() { Disposed = true; }
    }
}
namespace System.Speech.Synthesis
{
    public sealed class SpeechSynthesizer : IDisposable
    {
        public static readonly ConcurrentQueue<string> Spoken = new ConcurrentQueue<string>();
        public static readonly ManualResetEventSlim Preparing = new ManualResetEventSlim();
        public static readonly ManualResetEventSlim ReleasePreparation = new ManualResetEventSlim(true);
        public static readonly ManualResetEventSlim Speaking = new ManualResetEventSlim();
        public static readonly ManualResetEventSlim ReleaseSpeech = new ManualResetEventSlim(true);
        public static int Disposals;
        public static bool FailPreparation;
        public int Rate { get; set; }
        public int Volume { get; set; }
        public void SetOutputToDefaultAudioDevice()
        {
            Preparing.Set();
            if (!ReleasePreparation.Wait(3000)) { throw new Exception("Speech preparation was not released"); }
            if (FailPreparation) { throw new InvalidOperationException("Injected missing audio device"); }
        }
        public void Speak(string text)
        {
            Spoken.Enqueue(text); Speaking.Set();
            if (!ReleaseSpeech.Wait(3000)) { throw new Exception("Speech was not released"); }
        }
        public void Dispose() { Interlocked.Increment(ref Disposals); }
    }
}
namespace NOMAD.MissionPlanner
{
    public sealed class NOMADConfig { public bool AudioAlerts = true, AltitudeCallouts = true; }
    public static class Log
    {
        public static readonly ConcurrentQueue<string> Errors = new ConcurrentQueue<string>();
        public static void Error(string message) { Errors.Enqueue(message); }
    }
}
internal static class AudioAlertsTests
{
    private static void Require(bool condition, string diagnostic)
    { if (!condition) { throw new Exception(diagnostic); } }
    private static void Wait(Func<bool> condition, string diagnostic) =>
        Require(SpinWait.SpinUntil(condition, 2000), diagnostic);
    private static void Stop()
    {
        AudioAlerts.Shutdown();
        var worker = typeof(AudioAlerts).GetField("_speechWorker", BindingFlags.Static | BindingFlags.NonPublic);
        Wait(() => worker.GetValue(null) == null, "Muted speech worker did not retire");
    }
    private static int Main()
    {
        AudioAlerts.ApplyConfig(new NOMADConfig());
        System.Speech.Synthesis.SpeechSynthesizer.ReleaseSpeech.Reset();
        AudioAlerts.Speak("already dispatched", ignoreRateLimit: true);
        Require(System.Speech.Synthesis.SpeechSynthesizer.Speaking.Wait(2000), "First speech did not start");
        AudioAlerts.Speak("queued before mute", ignoreRateLimit: true);
        AudioAlerts.ApplyConfig(new NOMADConfig { AudioAlerts = false });
        Require(System.Media.SoundPlayer.Players.All(player => player.Stopped && player.Disposed && !player.Stream.CanRead),
            "Mute retained the keep-warm player or owned WAV stream");
        System.Speech.Synthesis.SpeechSynthesizer.ReleaseSpeech.Set();
        Stop();
        Require(System.Speech.Synthesis.SpeechSynthesizer.Spoken.SequenceEqual(new[] { "already dispatched" }) &&
            System.Speech.Synthesis.SpeechSynthesizer.Disposals == 1,
            "Mute dispatched queued speech or retained its synthesizer");

        System.Speech.Synthesis.SpeechSynthesizer.Preparing.Reset();
        System.Speech.Synthesis.SpeechSynthesizer.ReleasePreparation.Reset();
        AudioAlerts.ApplyConfig(new NOMADConfig());
        AudioAlerts.Speak("stale during initialization", ignoreRateLimit: true);
        Require(System.Speech.Synthesis.SpeechSynthesizer.Preparing.Wait(2000), "Speech preparation did not start");
        AudioAlerts.Shutdown();
        AudioAlerts.ApplyConfig(new NOMADConfig());
        AudioAlerts.Speak("fresh after reenable", ignoreRateLimit: true);
        System.Speech.Synthesis.SpeechSynthesizer.ReleasePreparation.Set();
        Wait(() => System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("fresh after reenable"),
            "Rapid reenable stranded fresh speech");
        Stop();
        Require(!System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("stale during initialization"),
            "An old speech generation survived mute and reenable");

        AudioAlerts.ApplyConfig(new NOMADConfig());
        System.Speech.Synthesis.SpeechSynthesizer.Speaking.Reset();
        System.Speech.Synthesis.SpeechSynthesizer.ReleaseSpeech.Reset();
        AudioAlerts.Speak("held while changing altitude preference", ignoreRateLimit: true);
        Require(System.Speech.Synthesis.SpeechSynthesizer.Speaking.Wait(2000), "Held preference speech did not start");
        AudioAlerts.Speak("pending altitude", component: "altitude", ignoreRateLimit: true);
        AudioAlerts.Speak("pending battery", component: "battery.1", ignoreRateLimit: true);
        AudioAlerts.ApplyConfig(new NOMADConfig { AltitudeCallouts = false });
        AudioAlerts.Speak("disabled altitude", component: "altitude", ignoreRateLimit: true);
        System.Speech.Synthesis.SpeechSynthesizer.ReleaseSpeech.Set();
        Wait(() => System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("pending battery"),
            "Disabling altitude callouts discarded an already queued battery warning");
        Stop(); Stop();
        Require(!System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("disabled altitude") &&
            !System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("pending altitude") &&
            !AudioAlerts.Enabled && !AudioAlerts.SpeechEnabled && !AudioAlerts.AltitudeCalloutsEnabled,
            "Disabled altitude or shutdown preferences admitted speech");

        AudioAlerts.ApplyConfig(new NOMADConfig());
        var rateLock = typeof(AudioAlerts).GetField("_lock", BindingFlags.Static | BindingFlags.NonPublic).GetValue(null);
        var delayed = new Thread(() => AudioAlerts.Speak("stale before admission", ignoreRateLimit: true))
            { IsBackground = true };
        lock (rateLock)
        {
            delayed.Start();
            Wait(() => (delayed.ThreadState & ThreadState.WaitSleepJoin) != 0, "Speech did not wait for admission");
            AudioAlerts.Shutdown(); AudioAlerts.ApplyConfig(new NOMADConfig());
        }
        Require(delayed.Join(2000), "Retired speech caller did not return");
        AudioAlerts.Speak("new admission", ignoreRateLimit: true);
        Wait(() => System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("new admission"), "Fresh speech was lost");
        Stop();
        Require(!System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("stale before admission"),
            "A rate-limit waiter survived mute and reenable");

        int disposals = System.Speech.Synthesis.SpeechSynthesizer.Disposals;
        System.Speech.Synthesis.SpeechSynthesizer.FailPreparation = true;
        AudioAlerts.ApplyConfig(new NOMADConfig());
        AudioAlerts.Speak("missing device", ignoreRateLimit: true);
        Wait(() => System.Speech.Synthesis.SpeechSynthesizer.Disposals == disposals + 1,
            "Failed speech initialization leaked its synthesizer");
        System.Speech.Synthesis.SpeechSynthesizer.FailPreparation = false;
        AudioAlerts.Speak("device recovered", ignoreRateLimit: true);
        Wait(() => System.Speech.Synthesis.SpeechSynthesizer.Spoken.Contains("device recovered"),
            "Speech did not recover after failed initialization");
        Stop();
        Require(Log.Errors.Count == 1 && Log.Errors.Single().Contains("Injected missing audio device"),
            "Audio lifecycle reported unexpected errors");

        long retired = (long)typeof(AudioAlerts).GetField("_speechRevision", BindingFlags.Static | BindingFlags.NonPublic)
            .GetValue(null);
        AudioAlerts.Shutdown(); AudioAlerts.ApplyConfig(new NOMADConfig());
        int players = System.Media.SoundPlayer.Players.Count;
        typeof(AudioAlerts).GetMethod("PlayPattern", BindingFlags.Static | BindingFlags.NonPublic)
            .Invoke(null, new object[] { AlertKind.BatteryWarning, retired });
        Require(System.Media.SoundPlayer.Players.Count == players, "A retired beep pattern opened an audio device");
        Stop();
        Console.WriteLine("PASS audio mute, stale initialization, reenable, altitude and resource retirement");
        return 0;
    }
}
