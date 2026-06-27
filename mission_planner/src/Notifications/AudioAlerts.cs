// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Audio Alerts
// ============================================================
// Distinct, recognizable beep patterns for in-flight warnings.
// Uses Console.Beep on a background thread so the UI never blocks.
// All alerts respect a global Enabled flag and de-duplicate so the
// same alert won't retrigger faster than its minimum interval.
// ============================================================

using System;
using System.Collections.Generic;
using System.IO;
using System.Media;
using System.Speech.Synthesis;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    public enum AlertKind
    {
        BoundarySoft,
        BoundaryHard,
        BatteryWarning,
        BatteryCritical,
    }

    public static class AudioAlerts
    {
        public static bool Enabled { get; set; } = true;

        // Minimum interval (seconds) before the same alert can re-play.
        private static readonly Dictionary<AlertKind, TimeSpan> MinInterval = new Dictionary<AlertKind, TimeSpan>
        {
            { AlertKind.BoundarySoft,    TimeSpan.FromSeconds(5) },
            { AlertKind.BoundaryHard,    TimeSpan.FromSeconds(2) },
            { AlertKind.BatteryWarning,  TimeSpan.FromSeconds(30) },
            { AlertKind.BatteryCritical, TimeSpan.FromSeconds(10) },
        };

        private static readonly Dictionary<AlertKind, DateTime> _lastPlayed = new Dictionary<AlertKind, DateTime>();
        private static readonly Dictionary<string, DateTime> _lastSpoken = new Dictionary<string, DateTime>();
        private static readonly object _lock = new object();
        private static SpeechSynthesizer _tts;
        private static readonly object _ttsLock = new object();

        // ---- Speech queue ----------------------------------------------------
        // Single worker plays one phrase at a time so callers from different
        // components never speak over each other. When a new phrase arrives
        // with the same Component as a still-queued phrase, the queued one is
        // replaced — so a component never speaks outdated info while a fresher
        // update is waiting behind it. The currently-playing phrase is never
        // interrupted (cutting off mid-sentence sounds worse than a small delay).
        private sealed class SpeechItem
        {
            public string Text;
            public string Component; // null => no dedup
        }
        private static readonly LinkedList<SpeechItem> _speechQueue = new LinkedList<SpeechItem>();
        private static readonly object _queueLock = new object();
        private static readonly AutoResetEvent _queueSignal = new AutoResetEvent(false);
        private static Thread _speechWorker;

        /// <summary>Enable spoken (text-to-speech) descriptions of alerts.</summary>
        public static bool SpeechEnabled { get; set; } = true;

        /// <summary>Minimum seconds between identical spoken phrases.</summary>
        public static int SpeechMinIntervalSec { get; set; } = 8;

        /// <summary>Enable Airbus-style spoken altitude callouts (see AltitudeCallout).</summary>
        public static bool AltitudeCalloutsEnabled { get; set; } = true;

        /// <summary>
        /// Apply the user's saved audio preferences to the engine. Called at plugin
        /// startup and whenever settings are saved so the toggles take effect live.
        /// The master "audio alerts" switch gates beeps, speech, and altitude callouts;
        /// altitude callouts additionally have their own opt-out.
        /// </summary>
        public static void ApplyConfig(NOMADConfig cfg)
        {
            if (cfg == null) return;
            Enabled = cfg.AudioAlerts;
            SpeechEnabled = cfg.AudioAlerts;
            AltitudeCalloutsEnabled = cfg.AudioAlerts && cfg.AltitudeCallouts;
            if (Enabled) StartKeepWarm(); else StopKeepWarm();
        }

        /// <summary>
        /// Play an alert if enabled and not rate-limited. Non-blocking.
        /// </summary>
        public static void Play(AlertKind kind, bool ignoreRateLimit = false)
        {
            if (!Enabled) return;

            lock (_lock)
            {
                if (!ignoreRateLimit
                    && _lastPlayed.TryGetValue(kind, out var last)
                    && DateTime.UtcNow - last < MinInterval[kind])
                {
                    return;
                }
                _lastPlayed[kind] = DateTime.UtcNow;
            }

            Task.Run(() =>
            {
                try { PlayPattern(kind); }
                catch (Exception ex) { Log.Error($"AudioAlerts error - {ex.Message}"); }
            });
        }

        /// <summary>
        /// Human-readable description of what the pattern sounds like.
        /// </summary>
        public static string DescribePattern(AlertKind kind)
        {
            switch (kind)
            {
                case AlertKind.BoundarySoft:    return "Two rising tones (800Hz → 1100Hz) — boundary warning";
                case AlertKind.BoundaryHard:    return "Three urgent high beeps (1500Hz x3) — hard boundary violation";
                case AlertKind.BatteryWarning:  return "Two descending tones (900Hz → 600Hz) — low battery";
                case AlertKind.BatteryCritical: return "Five rapid alarm beeps (1800Hz) — critical battery";
                default: return "Unknown alert";
            }
        }

        private static void PlayPattern(AlertKind kind)
        {
            // Wake the audio device with 300 ms of silence first — output
            // devices (especially Bluetooth/USB) power up with latency, which
            // clipped the start of beep sequences.
            PlaySine(0, 300);

            switch (kind)
            {
                case AlertKind.BoundarySoft:
                    Beep(800, 180); Sleep(60); Beep(1100, 220);
                    break;
                case AlertKind.BoundaryHard:
                    for (int i = 0; i < 3; i++) { Beep(1500, 150); Sleep(80); }
                    break;
                case AlertKind.BatteryWarning:
                    Beep(900, 220); Sleep(80); Beep(600, 300);
                    break;
                case AlertKind.BatteryCritical:
                    for (int i = 0; i < 5; i++) { Beep(1800, 130); Sleep(70); }
                    break;
            }
        }

        private static void Beep(int hz, int ms)
        {
            try { Console.Beep(hz, ms); }
            catch
            {
                // Console.Beep can fail on some systems (no PC speaker / headless);
                // fall back to system asterisk so the user still hears something.
                try { System.Media.SystemSounds.Asterisk.Play(); } catch { }
            }
        }

        private static void Sleep(int ms) { try { Thread.Sleep(ms); } catch { } }

        /// <summary>
        /// Synthesize and play a sine wave at the given frequency / duration. Uses an
        /// in-memory PCM WAV so it works at any audible frequency (Console.Beep
        /// degrades to clicks below ~100 Hz on modern Windows). Blocks the caller
        /// until playback finishes so sequences can be chained without overlap.
        /// </summary>
        private static void PlaySine(int hz, int ms)
        {
            try
            {
                using (var stream = BuildWav(hz, ms))
                using (var player = new SoundPlayer(stream))
                {
                    player.PlaySync();
                }
            }
            catch (Exception ex)
            {
                Log.Error($"PlaySine({hz},{ms}) error - {ex.Message}");
            }
        }

        /// <summary>
        /// Build an in-memory 16-bit mono PCM WAV of a sine wave (hz 0 = silence).
        /// Returns a stream positioned at 0; the caller owns it. The BinaryWriter is
        /// intentionally not disposed so the underlying stream stays open.
        /// </summary>
        private static MemoryStream BuildWav(int hz, int ms)
        {
            const int sampleRate = 44100;
            const short bitsPerSample = 16;
            const short channels = 1;
            int samples = (int)((long)sampleRate * ms / 1000);
            int byteCount = samples * 2;

            var stream = new MemoryStream(44 + byteCount);
            var bw = new BinaryWriter(stream);
            // RIFF header
            bw.Write(new[] { 'R', 'I', 'F', 'F' });
            bw.Write(36 + byteCount);
            bw.Write(new[] { 'W', 'A', 'V', 'E' });
            // fmt chunk
            bw.Write(new[] { 'f', 'm', 't', ' ' });
            bw.Write(16);                  // PCM chunk size
            bw.Write((short)1);            // PCM format
            bw.Write(channels);
            bw.Write(sampleRate);
            bw.Write(sampleRate * channels * bitsPerSample / 8); // byte rate
            bw.Write((short)(channels * bitsPerSample / 8));     // block align
            bw.Write(bitsPerSample);
            // data chunk
            bw.Write(new[] { 'd', 'a', 't', 'a' });
            bw.Write(byteCount);

            double twoPiF = 2.0 * Math.PI * hz;
            // Short fade in/out (5 ms) avoids the click at start/end.
            int fade = Math.Min(samples / 10, sampleRate / 200);
            for (int i = 0; i < samples; i++)
            {
                double env = 1.0;
                if (i < fade) env = (double)i / fade;
                else if (i > samples - fade) env = (double)(samples - i) / fade;
                double sample = Math.Sin(twoPiF * i / sampleRate) * env * 0.6;
                bw.Write((short)(sample * short.MaxValue));
            }
            bw.Flush();
            stream.Position = 0;
            return stream;
        }

        // ---- Audio device keep-warm ------------------------------------------
        // Output codecs (USB / Bluetooth especially) power down when idle and take
        // ~1 s to spin back up, which clips the first syllable of speech. Instead of
        // paying that as a pre-roll on every phrase, hold the default device awake
        // with a continuous silent loop so speech and beeps fire instantly.
        // ponytail: held open the whole time audio is enabled. If keeping the device
        // open is ever a problem (battery, an always-on power LED), gate Start/Stop
        // to armed-only via the monitor instead.
        private static SoundPlayer _keepWarm;
        private static readonly object _keepWarmLock = new object();

        /// <summary>Start the silent keep-warm loop on the default device. Idempotent.</summary>
        public static void StartKeepWarm()
        {
            lock (_keepWarmLock)
            {
                if (_keepWarm != null) return;
                try
                {
                    // 0.5 s silent buffer, looped seamlessly by the OS (SND_LOOP).
                    var player = new SoundPlayer(BuildWav(0, 500));
                    player.PlayLooping();
                    _keepWarm = player; // holds the stream alive for the life of the loop
                }
                catch (Exception ex)
                {
                    Log.Error($"keep-warm start failed - {ex.Message}");
                }
            }
        }

        /// <summary>Stop the keep-warm loop (lets the device idle again).</summary>
        public static void StopKeepWarm()
        {
            lock (_keepWarmLock)
            {
                try { _keepWarm?.Stop(); } catch { }
                _keepWarm = null;
            }
        }

        private static bool _welcomePlayed;

        /// <summary>
        /// Plays the one-time NOMAD startup chime: a ~6 second three-step ascending
        /// tone sequence followed by "Welcome to NOMAD." Safe to call repeatedly —
        /// only fires on the first call per process.
        /// </summary>
        public static void PlayWelcomeOnce()
        {
            lock (_lock)
            {
                if (_welcomePlayed) return;
                _welcomePlayed = true;
            }
            if (!Enabled) return;

            Speak("Welcome to NOMAD.", ignoreRateLimit: true);
        }

        /// <summary>
        /// Speak a message via Windows TTS. Phrases queue and play sequentially on a
        /// single worker thread so callers never speak over each other.
        ///
        /// <paramref name="component"/> is a dedup key: if a phrase from the same
        /// component is still waiting in the queue when a newer one arrives, the
        /// waiting phrase is replaced so we never speak stale info. Pass null for
        /// one-off announcements that should never be replaced.
        ///
        /// <paramref name="ignoreRateLimit"/> skips the identical-phrase rate limit
        /// (SpeechMinIntervalSec); it does NOT bypass the queue.
        /// </summary>
        public static void Speak(string text, string component = null, bool ignoreRateLimit = false)
        {
            if (!SpeechEnabled || string.IsNullOrWhiteSpace(text)) return;

            lock (_lock)
            {
                if (!ignoreRateLimit
                    && _lastSpoken.TryGetValue(text, out var last)
                    && (DateTime.UtcNow - last).TotalSeconds < SpeechMinIntervalSec)
                {
                    return;
                }
                _lastSpoken[text] = DateTime.UtcNow;
            }

            EnqueueSpeech(new SpeechItem { Text = text, Component = component });
        }

        // Back-compat overload for existing 2-arg callers: Speak(text, ignoreRateLimit: true)
        public static void Speak(string text, bool ignoreRateLimit) =>
            Speak(text, component: null, ignoreRateLimit: ignoreRateLimit);

        /// <summary>Convenience: play the beep pattern and speak a description.</summary>
        public static void PlayAndSpeak(AlertKind kind, string spokenText, string component = null, bool ignoreRateLimit = false)
        {
            Play(kind, ignoreRateLimit);
            Speak(spokenText, component, ignoreRateLimit);
        }

        public static void PlayAndSpeak(AlertKind kind, string spokenText, bool ignoreRateLimit) =>
            PlayAndSpeak(kind, spokenText, component: null, ignoreRateLimit: ignoreRateLimit);

        private static void EnqueueSpeech(SpeechItem item)
        {
            lock (_queueLock)
            {
                if (!string.IsNullOrEmpty(item.Component))
                {
                    // Drop any still-queued phrases from the same component — the new
                    // one supersedes them. The currently-playing phrase (already
                    // popped from the queue) is left alone.
                    var node = _speechQueue.First;
                    while (node != null)
                    {
                        var next = node.Next;
                        if (node.Value.Component == item.Component) _speechQueue.Remove(node);
                        node = next;
                    }
                }
                _speechQueue.AddLast(item);
                EnsureSpeechWorker();
            }
            _queueSignal.Set();
        }

        private static void EnsureSpeechWorker()
        {
            if (_speechWorker != null) return;
            _speechWorker = new Thread(SpeechWorkerLoop)
            {
                IsBackground = true,
                Name = "NOMAD-AudioAlerts-Speech",
            };
            _speechWorker.Start();
        }

        private static void SpeechWorkerLoop()
        {
            while (true)
            {
                SpeechItem item = null;
                lock (_queueLock)
                {
                    if (_speechQueue.Count > 0)
                    {
                        item = _speechQueue.First.Value;
                        _speechQueue.RemoveFirst();
                    }
                }

                if (item == null)
                {
                    _queueSignal.WaitOne();
                    continue;
                }

                try
                {
                    var synth = GetSynth();
                    if (synth == null) continue;
                    // No pre-roll: the device is held awake by the keep-warm loop
                    // (StartKeepWarm), so the first syllable fires immediately.
                    // Synchronous Speak — blocks the worker until the phrase finishes,
                    // which is exactly what serializes everything.
                    synth.Speak(item.Text);
                }
                catch (Exception ex)
                {
                    Log.Error($"TTS error - {ex.Message}");
                }
            }
        }

        private static SpeechSynthesizer GetSynth()
        {
            if (_tts != null) return _tts;
            lock (_ttsLock)
            {
                if (_tts != null) return _tts;
                try
                {
                    var s = new SpeechSynthesizer();
                    s.SetOutputToDefaultAudioDevice();
                    s.Rate = 1;       // slightly fast — important info in noisy environments
                    s.Volume = 100;
                    _tts = s;
                }
                catch (Exception ex)
                {
                    Log.Error($"TTS init failed - {ex.Message}");
                    return null;
                }
            }
            return _tts;
        }
    }
}
