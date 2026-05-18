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

        /// <summary>Enable spoken (text-to-speech) descriptions of alerts.</summary>
        public static bool SpeechEnabled { get; set; } = true;

        /// <summary>Minimum seconds between identical spoken phrases.</summary>
        public static int SpeechMinIntervalSec { get; set; } = 8;

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
                catch (Exception ex) { Console.WriteLine($"NOMAD: AudioAlerts error - {ex.Message}"); }
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
                const int sampleRate = 44100;
                const short bitsPerSample = 16;
                const short channels = 1;
                int samples = (int)((long)sampleRate * ms / 1000);
                int byteCount = samples * 2;

                using (var ms_stream = new MemoryStream(44 + byteCount))
                using (var bw = new BinaryWriter(ms_stream))
                {
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
                    ms_stream.Position = 0;

                    using (var player = new SoundPlayer(ms_stream))
                    {
                        player.PlaySync();
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: PlaySine({hz},{ms}) error - {ex.Message}");
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
        /// Speak a message via Windows TTS on a background thread. De-duplicates so the
        /// same phrase won't repeat faster than SpeechMinIntervalSec.
        /// </summary>
        public static void Speak(string text, bool ignoreRateLimit = false)
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

            Task.Run(() =>
            {
                try
                {
                    var synth = GetSynth();
                    if (synth == null) return;
                    // SpeakAsyncCancelAll prevents queue buildup if alerts pile up.
                    lock (_ttsLock)
                    {
                        synth.SpeakAsyncCancelAll();
                        synth.SpeakAsync(text);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"NOMAD: TTS error - {ex.Message}");
                }
            });
        }

        /// <summary>Convenience: play the beep pattern and speak a description.</summary>
        public static void PlayAndSpeak(AlertKind kind, string spokenText, bool ignoreRateLimit = false)
        {
            Play(kind, ignoreRateLimit);
            Speak(spokenText, ignoreRateLimit);
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
                    Console.WriteLine($"NOMAD: TTS init failed - {ex.Message}");
                    return null;
                }
            }
            return _tts;
        }
    }
}
