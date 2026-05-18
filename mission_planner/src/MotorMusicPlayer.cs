// ============================================================
// Motor Music Player - DroneCAN ESC tone sequencer (ground-side)
// ============================================================
// Plays melodies on the drone motors by sending DO_SET_SERVO MAVLink
// commands from Mission Planner to the Cube Orange, which forwards
// PWM to the DroneCAN ESCs. The audible "tone" is motor commutation
// whine, not a true speaker — pitch tracks throttle, so the mapping
// from musical frequency to PWM is empirical/coarse.
//
// SAFETY: Remove propellers before use. Playback is blocked while the
// vehicle is armed unless the user explicitly enables the override.
// ============================================================

using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public sealed class MotorMusicPlayer
    {
        public sealed class Note
        {
            public string Name;
            public int DurationMs;
            public Note(string name, int durationMs) { Name = name; DurationMs = durationMs; }
        }

        public sealed class Song
        {
            public string Id;
            public string Title;
            public List<Note> Notes;
            public int TotalMs
            {
                get { int t = 0; foreach (var n in Notes) t += n.DurationMs; return t; }
            }
        }

        // ----- pitch table (semitone offsets from A within an octave) -----
        private static readonly Dictionary<string, int> ChromaticOffset = new Dictionary<string, int>(StringComparer.Ordinal)
        {
            { "C", -9 }, { "C#", -8 }, { "Db", -8 },
            { "D", -7 }, { "D#", -6 }, { "Eb", -6 },
            { "E", -5 }, { "Fb", -5 },
            { "F", -4 }, { "F#", -3 }, { "Gb", -3 },
            { "G", -2 }, { "G#", -1 }, { "Ab", -1 },
            { "A",  0 }, { "A#",  1 }, { "Bb",  1 },
            { "B",  2 }, { "Cb",  2 },
        };

        private static int Midi(string note)
        {
            if (string.IsNullOrEmpty(note)) return -1;
            if (note.Equals("R", StringComparison.OrdinalIgnoreCase)) return -1;
            foreach (var kv in ChromaticOffset)
            {
                if (note.StartsWith(kv.Key, StringComparison.Ordinal) && note.Length > kv.Key.Length)
                {
                    int octave;
                    if (int.TryParse(note.Substring(kv.Key.Length), out octave))
                        return 12 * (octave + 1) + kv.Value;
                }
            }
            return -1;
        }

        private static double Frequency(string note)
        {
            int m = Midi(note);
            if (m < 0) return 0.0;
            return 440.0 * Math.Pow(2.0, (m - 69) / 12.0);
        }

        // "C4-500 R-125 Eb4-375" → list of Notes
        private static List<Note> Parse(string score)
        {
            var notes = new List<Note>();
            foreach (var tok in score.Split(new[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries))
            {
                int dash = tok.LastIndexOf('-');
                if (dash <= 0) continue;
                int dur;
                if (!int.TryParse(tok.Substring(dash + 1), out dur)) continue;
                notes.Add(new Note(tok.Substring(0, dash), dur));
            }
            return notes;
        }

        // ------------------------ song library ------------------------
        public static readonly Dictionary<string, Song> Library = BuildLibrary();

        private static Dictionary<string, Song> BuildLibrary()
        {
            var lib = new Dictionary<string, Song>(StringComparer.OrdinalIgnoreCase);
            void Add(string id, string title, string score) =>
                lib[id] = new Song { Id = id, Title = title, Notes = Parse(score) };

            Add("arming_chime", "Arming Chime",
                "C4-300 E4-300 G4-300 C5-500");

            Add("imperial_march", "Imperial March",
                "G4-500 G4-500 G4-500 Eb4-375 R-125 Bb4-125 " +
                "G4-500 Eb4-375 R-125 Bb4-125 G4-700 " +
                "D5-500 D5-500 D5-500 Eb5-375 R-125 Bb4-125 " +
                "Gb4-500 Eb4-375 R-125 Bb4-125 G4-700");

            Add("mario", "Super Mario",
                "E5-125 E5-125 R-125 E5-125 R-125 C5-125 E5-250 " +
                "G5-250 R-250 G4-250 R-250 " +
                "C5-250 R-125 G4-250 R-125 E4-250 " +
                "A4-250 B4-250 Bb4-125 A4-250 G4-167 E5-167 G5-167");

            Add("star_wars", "Star Wars Theme",
                "D4-167 D4-167 D4-167 G4-500 D5-500 " +
                "C5-167 B4-167 A4-167 G5-500 D5-250 " +
                "C5-167 B4-167 A4-167 G5-500 D5-250");

            Add("nokia", "Nokia Ringtone",
                "E5-150 D5-150 F#4-300 G#4-300 " +
                "C#5-150 B4-150 D4-300 E4-300 " +
                "B4-150 A4-150 C#4-300 E4-300 A4-600");

            Add("zelda", "Zelda Lullaby",
                "E4-500 G4-250 D4-750 C4-250 D4-250 " +
                "E4-500 G4-250 D4-750 C4-250 D4-250 " +
                "E4-250 G4-250 B4-500 A4-250 G4-250");

            Add("scale", "C Major Scale (test)",
                "C4-300 D4-300 E4-300 F4-300 G4-300 A4-300 B4-300 C5-600");

            return lib;
        }

        // -------------------- runtime config / state --------------------
        public int PwmMin = 1100;
        public int PwmMax = 1900;
        public double FreqLow = 110.0;   // Hz → PwmMin
        public double FreqHigh = 1760.0; // Hz → PwmMax
        public int InterNoteGapMs = 25;  // brief silence between notes
        public int MaxSongMs = 60_000;
        public bool AllowArmedOverride = false;

        private Thread _thread;
        private CancellationTokenSource _cts;
        private readonly object _lock = new object();
        private volatile Song _currentSong;
        private volatile float _progress;

        public bool IsPlaying { get { var t = _thread; return t != null && t.IsAlive; } }
        public Song CurrentSong { get { return _currentSong; } }
        public float Progress { get { return _progress; } }

        public event Action<string> StatusChanged;   // free-form text
        public event Action<bool> PlayingChanged;    // true=started, false=stopped

        // ------------------------- public API --------------------------
        public string Play(string songId, IEnumerable<int> channels, double tempo)
        {
            Song song;
            if (!Library.TryGetValue(songId, out song))
                return "Unknown song: " + songId;

            var chans = new List<int>();
            foreach (var c in channels) if (c > 0) chans.Add(c);
            if (chans.Count == 0) return "Select at least one motor channel.";

            if (song.TotalMs > MaxSongMs)
                return string.Format("Song length {0}ms exceeds {1}ms safety cap.", song.TotalMs, MaxSongMs);

            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                return "MAVLink link is not open.";

            if (!AllowArmedOverride)
            {
                try
                {
                    if (MainV2.comPort.MAV?.cs != null && MainV2.comPort.MAV.cs.armed)
                        return "Refusing to play: vehicle is ARMED. Disarm first or enable override.";
                }
                catch { }
            }

            if (tempo <= 0.05) tempo = 0.05;
            if (tempo > 4.0) tempo = 4.0;

            lock (_lock)
            {
                StopInternal();
                _cts = new CancellationTokenSource();
                _currentSong = song;
                _progress = 0f;

                var token = _cts.Token;
                _thread = new Thread(() => PlayLoop(song, chans.ToArray(), tempo, token))
                {
                    IsBackground = true,
                    Name = "NOMAD-MotorMusic",
                };
                _thread.Start();
            }

            Raise(PlayingChanged, true);
            RaiseStatus("Playing " + song.Title);
            return null;
        }

        public void Stop()
        {
            lock (_lock) { StopInternal(); }
            RaiseStatus("Stopped.");
            Raise(PlayingChanged, false);
        }

        private void StopInternal()
        {
            try { _cts?.Cancel(); } catch { }
            var t = _thread;
            if (t != null && t.IsAlive)
            {
                try { t.Join(500); } catch { }
            }
            _thread = null;
        }

        // ----------------------- playback loop -------------------------
        private void PlayLoop(Song song, int[] channels, double tempo, CancellationToken ct)
        {
            int totalMs = song.TotalMs;
            int elapsedMs = 0;
            try
            {
                foreach (var note in song.Notes)
                {
                    if (ct.IsCancellationRequested) break;

                    int durMs = Math.Max(20, (int)(note.DurationMs / tempo));
                    int toneMs = Math.Max(10, durMs - InterNoteGapMs);

                    SendNote(note, channels);
                    SleepFor(toneMs, ct);

                    SilenceMotors(channels);
                    SleepFor(InterNoteGapMs, ct);

                    elapsedMs += durMs;
                    _progress = totalMs > 0 ? (float)elapsedMs / totalMs : 1f;
                }
            }
            catch { /* swallow; we always idle below */ }
            finally
            {
                SilenceMotors(channels);
                _currentSong = null;
                _progress = 0f;
                Raise(PlayingChanged, false);
                RaiseStatus("Finished.");
            }
        }

        private static void SleepFor(int ms, CancellationToken ct)
        {
            if (ms <= 0) return;
            try { ct.WaitHandle.WaitOne(ms); } catch { }
        }

        private void SendNote(Note note, int[] channels)
        {
            if (note.Name.Equals("R", StringComparison.OrdinalIgnoreCase))
            {
                SilenceMotors(channels);
                return;
            }
            int pwm = FreqToPwm(Frequency(note.Name));
            foreach (var ch in channels)
                CubeOutputController.TrySendServoMavlink(ch, pwm, tryOnly: false);
        }

        private void SilenceMotors(int[] channels)
        {
            // 1000 µs ≈ disarmed throttle for most ESCs; motors stop spinning.
            foreach (var ch in channels)
                CubeOutputController.TrySendServoMavlink(ch, 1000, tryOnly: false);
        }

        private int FreqToPwm(double freq)
        {
            if (freq <= 0) return PwmMin;
            // Log-scaled interpolation — musical pitch is logarithmic.
            double lo = Math.Log(FreqLow);
            double hi = Math.Log(FreqHigh);
            double t = (Math.Log(freq) - lo) / (hi - lo);
            if (t < 0) t = 0; else if (t > 1) t = 1;
            return (int)Math.Round(PwmMin + t * (PwmMax - PwmMin));
        }

        private void RaiseStatus(string msg)
        {
            var h = StatusChanged;
            if (h != null) { try { h(msg); } catch { } }
        }

        private static void Raise<T>(Action<T> handler, T arg)
        {
            if (handler != null) { try { handler(arg); } catch { } }
        }

        // singleton
        private static readonly Lazy<MotorMusicPlayer> _instance = new Lazy<MotorMusicPlayer>(() => new MotorMusicPlayer());
        public static MotorMusicPlayer Instance { get { return _instance.Value; } }
    }
}
