// ============================================================
// Motor Music Player - DroneCAN ESC beep sequencer (ground-side)
// ============================================================
// Plays melodies by broadcasting uavcan.equipment.indication.BeepCommand
// frames through ArduPilot's MAVLink CAN bridge. This uses the ESC's
// own motor-beep path instead of DO_MOTOR_TEST throttle commands, so it
// does not intentionally spin the motors.
//
// SAFETY: Remove propellers before use. Playback is blocked while the
// vehicle is armed unless the user explicitly enables the override.
// ============================================================

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using System.Threading;
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
            public string Source;
            public int TrackCount = 1;
            public List<Note> Notes;
            public int TotalMs
            {
                get { int t = 0; foreach (var n in Notes) t += n.DurationMs; return t; }
            }
        }

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

        private const int EscMusicDurationScale = 6;
        private static readonly Regex EscSectionRegex = new Regex(
            @"\[MUSIC#(?<track>\d+)\]\s*NOTES=(?<notes>[^\r\n]+)\s*NOTE_LENGTH=(?<length>\d+)\s*NOTE_INTERVAL=(?<interval>\d+)",
            RegexOptions.Compiled | RegexOptions.IgnoreCase);

        public static readonly Dictionary<string, Song> Library = BuildLibrary();

        private static Dictionary<string, Song> BuildLibrary()
        {
            var lib = new Dictionary<string, Song>(StringComparer.OrdinalIgnoreCase);
            void Add(string id, string title, string score) =>
                lib[id] = new Song { Id = id, Title = title, Source = "Built-in", Notes = ParseInlineScore(score) };

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

            Add("rickroll", "Rickroll",
                "R-250 F#4-180 G#4-180 B4-180 G#4-180 D#5-380 D#5-380 C#5-760 " +
                "R-180 F#4-180 G#4-180 B4-180 G#4-180 C#5-380 C#5-380 B4-380 A#4-180 G#4-560 " +
                "R-180 F#4-180 G#4-180 B4-180 G#4-180 B4-380 C#5-180 A#4-380 G#4-180 F#4-380 " +
                "F#4-180 C#5-380 B4-760");

            foreach (var song in LoadEscMusicSongs())
            {
                if (song.Notes.Count == 0) continue;
                var id = song.Id;
                int suffix = 2;
                while (lib.ContainsKey(id)) id = song.Id + "_" + suffix++;
                song.Id = id;
                lib[id] = song;
            }

            return lib;
        }

        private static IEnumerable<Song> LoadEscMusicSongs()
        {
            var loaded = new List<Song>();
            var assembly = Assembly.GetExecutingAssembly();
            foreach (var name in assembly.GetManifestResourceNames().Where(n => n.IndexOf(".esc_music.", StringComparison.OrdinalIgnoreCase) >= 0 && n.EndsWith(".txt", StringComparison.OrdinalIgnoreCase)))
            {
                using (var stream = assembly.GetManifestResourceStream(name))
                using (var reader = stream == null ? null : new StreamReader(stream))
                {
                    if (reader == null) continue;
                    var title = TitleFromResourceName(name);
                    var song = ParseEscMusicText(title, reader.ReadToEnd(), "ESC Music");
                    if (song != null) loaded.Add(song);
                }
            }

            var baseDir = AppDomain.CurrentDomain.BaseDirectory;
            var fileDir = Path.Combine(baseDir, "esc_music");
            if (Directory.Exists(fileDir))
            {
                foreach (var file in Directory.GetFiles(fileDir, "*.txt", SearchOption.AllDirectories))
                {
                    var title = Path.GetFileNameWithoutExtension(file);
                    var song = ParseEscMusicText(title, File.ReadAllText(file), "ESC Music");
                    if (song != null && loaded.All(s => !s.Title.Equals(song.Title, StringComparison.OrdinalIgnoreCase)))
                        loaded.Add(song);
                }
            }

            return loaded;
        }

        private static Song ParseEscMusicText(string title, string text, string source)
        {
            var matches = EscSectionRegex.Matches(text ?? "");
            if (matches.Count == 0) return null;

            var firstTrack = matches.Cast<Match>().OrderBy(m => int.Parse(m.Groups["track"].Value)).First();
            var notes = ParseEscTrack(
                firstTrack.Groups["notes"].Value,
                int.Parse(firstTrack.Groups["length"].Value),
                int.Parse(firstTrack.Groups["interval"].Value));

            return new Song
            {
                Id = Slug(title),
                Title = title,
                Source = source,
                TrackCount = matches.Count,
                Notes = notes,
            };
        }

        private static List<Note> ParseEscTrack(string compactNotes, int noteLength, int noteInterval)
        {
            var notes = new List<Note>();
            int i = 0;
            int intervalMs = Math.Max(0, noteInterval * noteLength * EscMusicDurationScale);
            while (i < compactNotes.Length)
            {
                if (compactNotes[i] == 'P')
                {
                    if (i + 1 >= compactNotes.Length || !char.IsDigit(compactNotes[i + 1])) break;
                    int units = compactNotes[i + 1] - '0';
                    notes.Add(new Note("R", Math.Max(20, units * noteLength * EscMusicDurationScale + intervalMs)));
                    i += 2;
                    continue;
                }

                if ("ABCDEFG".IndexOf(compactNotes[i]) < 0) break;
                string pitch = compactNotes[i].ToString();
                i++;
                if (i < compactNotes.Length && compactNotes[i] == '#')
                {
                    pitch += "#";
                    i++;
                }
                if (i + 1 >= compactNotes.Length || !char.IsDigit(compactNotes[i]) || !char.IsDigit(compactNotes[i + 1])) break;
                string noteName = pitch + compactNotes[i];
                int durationUnits = compactNotes[i + 1] - '0';
                notes.Add(new Note(noteName, Math.Max(20, durationUnits * noteLength * EscMusicDurationScale + intervalMs)));
                i += 2;
            }
            return notes;
        }

        private static List<Note> ParseInlineScore(string score)
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

        private static string TitleFromResourceName(string resourceName)
        {
            var parts = resourceName.Split('.');
            var title = parts.Length >= 2 ? parts[parts.Length - 2] : resourceName;
            return title.Replace('_', '\'');
        }

        private static string Slug(string title)
        {
            var chars = new List<char>();
            foreach (char c in (title ?? "song").ToLowerInvariant())
            {
                if (char.IsLetterOrDigit(c)) chars.Add(c);
                else if (chars.Count == 0 || chars[chars.Count - 1] != '_') chars.Add('_');
            }
            return new string(chars.ToArray()).Trim('_');
        }

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

        public int DroneCanSourceNodeId = 126;
        public int InterNoteGapMs = 20;
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

        public event Action<string> StatusChanged;
        public event Action<bool> PlayingChanged;

        public string Play(string songId, IEnumerable<int> canBuses, double tempo)
        {
            Song song;
            if (!Library.TryGetValue(songId, out song))
                return "Unknown song: " + songId;

            var buses = new List<int>();
            foreach (var c in canBuses) if (c >= 1 && c <= 3 && !buses.Contains(c)) buses.Add(c);
            if (buses.Count == 0) return "Select at least one DroneCAN bus.";

            if (song.TotalMs > MaxSongMs)
                return string.Format("Song length {0}ms exceeds {1}ms safety cap.", song.TotalMs, MaxSongMs);

            if (DroneCanSourceNodeId < 1 || DroneCanSourceNodeId > 127)
                return "DroneCAN source node must be 1..127.";

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
                _thread = new Thread(() => PlayLoop(song, buses.ToArray(), tempo, token))
                {
                    IsBackground = true,
                    Name = "NOMAD-MotorMusic",
                };
                _thread.Start();
            }

            Raise(PlayingChanged, true);
            RaiseStatus("Playing " + song.Title + " via DroneCAN beep commands.");
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

        private void PlayLoop(Song song, int[] canBuses, double tempo, CancellationToken ct)
        {
            int totalMs = song.Notes.Sum(n => Math.Max(20, (int)(n.DurationMs / tempo)));
            int elapsedMs = 0;
            try
            {
                foreach (int bus in canBuses)
                {
                    try { CubeOutputController.EnableCanForward(bus, true); } catch { }
                }

                foreach (var note in song.Notes)
                {
                    if (ct.IsCancellationRequested) break;

                    int durMs = Math.Max(20, (int)(note.DurationMs / tempo));
                    int toneMs = Math.Max(10, durMs - InterNoteGapMs);

                    SendNote(note, canBuses, toneMs);
                    SleepFor(toneMs, ct);
                    SleepFor(InterNoteGapMs, ct);

                    elapsedMs += durMs;
                    _progress = totalMs > 0 ? Math.Min(1f, (float)elapsedMs / totalMs) : 1f;
                }
            }
            catch { }
            finally
            {
                foreach (int bus in canBuses)
                {
                    try { CubeOutputController.EnableCanForward(bus, false); } catch { }
                }
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

        private void SendNote(Note note, int[] canBuses, int toneMs)
        {
            if (note.Name.Equals("R", StringComparison.OrdinalIgnoreCase)) return;

            double freq = Frequency(note.Name);
            if (freq <= 0.0) return;

            double durationSeconds = Math.Max(0.02, Math.Min(5.0, toneMs / 1000.0));
            foreach (var bus in canBuses)
            {
                try
                {
                    CubeOutputController.SendDroneCanBeep(bus, DroneCanSourceNodeId, freq, durationSeconds);
                }
                catch { }
            }
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

        private static readonly Lazy<MotorMusicPlayer> _instance = new Lazy<MotorMusicPlayer>(() => new MotorMusicPlayer());
        public static MotorMusicPlayer Instance { get { return _instance.Value; } }
    }
}
