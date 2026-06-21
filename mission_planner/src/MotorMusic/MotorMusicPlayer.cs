// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    internal sealed class MotorMusicPlaybackProgress
    {
        public double PositionMs { get; set; }
        public double LengthMs { get; set; }
        public int SentNotes { get; set; }
        public int DroppedNotes { get; set; }
    }

    internal sealed class MotorMusicPlayer : IDisposable
    {
        private CancellationTokenSource _cts;
        private Task _playTask;

        public event Action<MotorMusicPlaybackProgress> Progress;
        public event Action<string> Completed;

        public bool IsPlaying => _playTask != null && !_playTask.IsCompleted;

        public void Play(MotorMusicMidiFile midi, MotorMusicCommandOptions options)
        {
            Stop();
            var notes = MotorMusicMidiParser.PrepareNotes(midi, options);
            _cts = new CancellationTokenSource();
            _playTask = Task.Run(() => PlaybackLoop(notes, options, midi?.LengthMs ?? 0, _cts.Token));
        }

        public async void Stop()
        {
            try
            {
                _cts?.Cancel();
                await MotorMusicCommand.SendStopAsync().ConfigureAwait(false);
            }
            catch { }
        }

        private async Task PlaybackLoop(
            List<MotorMusicNote> notes,
            MotorMusicCommandOptions options,
            double originalLengthMs,
            CancellationToken token)
        {
            int sent = 0;
            int dropped = 0;
            int motorCount = Math.Max(1, Math.Min(12, options.MotorCount));
            int minOutputPwm = Math.Max(1000, Math.Min(2000, options.MinOutputPwm));
            int maxOutputPwm = Math.Max(minOutputPwm, Math.Min(2000, options.MaxOutputPwm));
            var voiceMap = BuildVoiceMap(notes, motorCount);
            var stopwatch = Stopwatch.StartNew();

            try
            {
                for (int i = 0; i < notes.Count; i++)
                {
                    token.ThrowIfCancellationRequested();
                    var note = notes[i];
                    int waitMs = (int)Math.Max(0, note.StartMs - stopwatch.Elapsed.TotalMilliseconds);
                    if (waitMs > 1)
                        await Task.Delay(Math.Min(waitMs, 50), token).ConfigureAwait(false);

                    int motor = PickMotor(note, voiceMap, motorCount, i);
                    bool ok = await MotorMusicCommand.SendNoteAsync(
                        motor,
                        note.Note,
                        note.Velocity,
                        (int)Math.Round(note.DurationMs),
                        maxOutputPwm,
                        minOutputPwm).ConfigureAwait(false);
                    if (ok) sent++;
                    else dropped++;

                    if ((sent + dropped) % 12 == 0)
                    {
                        RaiseProgress(
                            stopwatch.Elapsed.TotalMilliseconds,
                            originalLengthMs / options.TempoScale,
                            sent,
                            dropped);
                    }
                }

                await Task.Delay(250, token).ConfigureAwait(false);
                Completed?.Invoke($"Playback complete - {sent} notes sent, {dropped} dropped.");
            }
            catch (OperationCanceledException)
            {
                Completed?.Invoke($"Playback stopped - {sent} notes sent, {dropped} dropped.");
            }
            catch (Exception ex)
            {
                Completed?.Invoke("Playback failed: " + ex.Message);
            }
            finally
            {
                await MotorMusicCommand.SendStopAsync().ConfigureAwait(false);
                RaiseProgress(
                    stopwatch.Elapsed.TotalMilliseconds,
                    originalLengthMs / options.TempoScale,
                    sent,
                    dropped);
            }
        }

        private static Dictionary<int, int> BuildVoiceMap(IEnumerable<MotorMusicNote> notes, int motorCount)
        {
            var voices = notes
                .Select(n => (n.Track << 8) | n.Channel)
                .Distinct()
                .OrderBy(v => v)
                .ToList();
            var map = new Dictionary<int, int>();
            for (int i = 0; i < voices.Count; i++)
                map[voices[i]] = (i % motorCount) + 1;
            return map;
        }

        private static int PickMotor(
            MotorMusicNote note,
            Dictionary<int, int> voiceMap,
            int motorCount,
            int noteIndex)
        {
            int key = (note.Track << 8) | note.Channel;
            if (voiceMap.TryGetValue(key, out int motor))
                return motor;
            return (noteIndex % motorCount) + 1;
        }

        private void RaiseProgress(double positionMs, double lengthMs, int sent, int dropped)
        {
            Progress?.Invoke(new MotorMusicPlaybackProgress
            {
                PositionMs = positionMs,
                LengthMs = lengthMs,
                SentNotes = sent,
                DroppedNotes = dropped,
            });
        }

        public void Dispose()
        {
            Stop();
            _cts?.Dispose();
        }
    }
}
