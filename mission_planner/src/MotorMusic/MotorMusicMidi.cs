// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    internal sealed class MotorMusicNote
    {
        public int Track { get; set; }
        public int Channel { get; set; }
        public int Note { get; set; }
        public int Velocity { get; set; }
        public double StartMs { get; set; }
        public double DurationMs { get; set; }
    }

    internal sealed class MotorMusicMidiFile
    {
        public string FileName { get; set; }
        public int Format { get; set; }
        public int Tracks { get; set; }
        public int Division { get; set; }
        public double LengthMs { get; set; }
        public List<MotorMusicNote> Notes { get; } = new List<MotorMusicNote>();

        public string Summary
        {
            get
            {
                var channels = Notes.Select(n => n.Channel).Distinct().Count();
                return $"{Path.GetFileName(FileName)} - {Notes.Count} notes, {Tracks} tracks, " +
                       $"{channels} channels, {LengthMs / 1000.0:0.0}s";
            }
        }
    }

    internal static class MotorMusicMidiParser
    {
        private sealed class ActiveNote
        {
            public int Track;
            public int Channel;
            public int Note;
            public int Velocity;
            public double StartMs;
        }

        public static MotorMusicMidiFile Load(string path)
        {
            var bytes = File.ReadAllBytes(path);
            int index = 0;
            if (ReadAscii(bytes, ref index, 4) != "MThd")
                throw new InvalidDataException("Not a MIDI file.");

            int headerLength = ReadInt32(bytes, ref index);
            int headerEnd = index + headerLength;
            int format = ReadInt16(bytes, ref index);
            int tracks = ReadInt16(bytes, ref index);
            int division = ReadInt16(bytes, ref index);
            index = headerEnd;

            if ((division & 0x8000) != 0)
                throw new InvalidDataException("SMPTE-time MIDI files are not supported.");

            var midi = new MotorMusicMidiFile
            {
                FileName = path,
                Format = format,
                Tracks = tracks,
                Division = division,
            };

            for (int track = 0; track < tracks && index < bytes.Length; track++)
            {
                if (ReadAscii(bytes, ref index, 4) != "MTrk")
                    throw new InvalidDataException("MIDI track header is missing.");
                int length = ReadInt32(bytes, ref index);
                ParseTrack(bytes, index, index + length, track, division, midi);
                index += length;
            }

            midi.Notes.Sort((a, b) => a.StartMs.CompareTo(b.StartMs));
            if (midi.Notes.Count > 0)
                midi.LengthMs = midi.Notes.Max(n => n.StartMs + n.DurationMs);
            return midi;
        }

        public static List<MotorMusicNote> PrepareNotes(
            MotorMusicMidiFile midi,
            MotorMusicCommandOptions options,
            int maxNotes = 18000)
        {
            var notes = new List<MotorMusicNote>();
            if (midi == null) return notes;

            double tempoScale = options?.TempoScale ?? 1.0;
            if (tempoScale <= 0.1) tempoScale = 1.0;
            int transpose = options?.Transpose ?? 0;

            foreach (var note in midi.Notes.Take(maxNotes))
            {
                var mapped = new MotorMusicNote
                {
                    Track = note.Track,
                    Channel = note.Channel,
                    Note = Math.Max(24, Math.Min(96, note.Note + transpose)),
                    Velocity = Math.Max(1, Math.Min(127, note.Velocity)),
                    StartMs = note.StartMs / tempoScale,
                    DurationMs = Math.Max(35, Math.Min(2500, note.DurationMs / tempoScale)),
                };
                notes.Add(mapped);
            }

            notes.Sort((a, b) => a.StartMs.CompareTo(b.StartMs));
            return notes;
        }

        private static void ParseTrack(
            byte[] bytes,
            int index,
            int end,
            int track,
            int division,
            MotorMusicMidiFile midi)
        {
            var active = new Dictionary<int, Queue<ActiveNote>>();
            int runningStatus = 0;
            long tick = 0;
            double currentMs = 0;
            long tempoStartTick = 0;
            double tempoStartMs = 0;
            int tempoUsPerQuarter = 500000;

            while (index < end)
            {
                long delta = ReadVariableLength(bytes, ref index);
                tick += delta;
                currentMs = tempoStartMs + ((tick - tempoStartTick) * tempoUsPerQuarter / 1000.0 / division);
                if (index >= end) break;

                int status = bytes[index++];
                if (status < 0x80)
                {
                    if (runningStatus == 0)
                        throw new InvalidDataException("MIDI running status without a previous status byte.");
                    index--;
                    status = runningStatus;
                }
                else if (status < 0xF0)
                {
                    runningStatus = status;
                }

                if (status == 0xFF)
                {
                    int metaType = bytes[index++];
                    int length = (int)ReadVariableLength(bytes, ref index);
                    if (metaType == 0x2F)
                    {
                        index += length;
                        break;
                    }
                    if (metaType == 0x51 && length == 3)
                    {
                        tempoStartMs = currentMs;
                        tempoStartTick = tick;
                        tempoUsPerQuarter = (bytes[index] << 16) | (bytes[index + 1] << 8) | bytes[index + 2];
                    }
                    index += length;
                    continue;
                }

                if (status == 0xF0 || status == 0xF7)
                {
                    int length = (int)ReadVariableLength(bytes, ref index);
                    index += length;
                    continue;
                }

                int command = status & 0xF0;
                int channel = status & 0x0F;
                int data1 = bytes[index++];
                int data2 = NeedsSecondDataByte(command) ? bytes[index++] : 0;

                if (command == 0x90 && data2 > 0)
                {
                    var key = NoteKey(channel, data1);
                    if (!active.TryGetValue(key, out var queue))
                    {
                        queue = new Queue<ActiveNote>();
                        active[key] = queue;
                    }
                    queue.Enqueue(new ActiveNote
                    {
                        Track = track,
                        Channel = channel,
                        Note = data1,
                        Velocity = data2,
                        StartMs = currentMs,
                    });
                }
                else if (command == 0x80 || command == 0x90)
                {
                    var key = NoteKey(channel, data1);
                    if (!active.TryGetValue(key, out var queue) || queue.Count == 0)
                        continue;

                    var started = queue.Dequeue();
                    var duration = Math.Max(10, currentMs - started.StartMs);
                    midi.Notes.Add(new MotorMusicNote
                    {
                        Track = started.Track,
                        Channel = started.Channel,
                        Note = started.Note,
                        Velocity = started.Velocity,
                        StartMs = started.StartMs,
                        DurationMs = duration,
                    });
                }
            }
        }

        private static bool NeedsSecondDataByte(int command)
        {
            return command != 0xC0 && command != 0xD0;
        }

        private static int NoteKey(int channel, int note)
        {
            return (channel << 8) | note;
        }

        private static string ReadAscii(byte[] bytes, ref int index, int count)
        {
            var text = System.Text.Encoding.ASCII.GetString(bytes, index, count);
            index += count;
            return text;
        }

        private static int ReadInt16(byte[] bytes, ref int index)
        {
            int value = (bytes[index] << 8) | bytes[index + 1];
            index += 2;
            return value;
        }

        private static int ReadInt32(byte[] bytes, ref int index)
        {
            uint value = ((uint)bytes[index] << 24)
                       | ((uint)bytes[index + 1] << 16)
                       | ((uint)bytes[index + 2] << 8)
                       | bytes[index + 3];
            index += 4;
            return (int)value;
        }

        private static long ReadVariableLength(byte[] bytes, ref int index)
        {
            long value = 0;
            for (int i = 0; i < 4; i++)
            {
                int b = bytes[index++];
                value = (value << 7) | (long)(b & 0x7F);
                if ((b & 0x80) == 0) break;
            }
            return value;
        }
    }
}
