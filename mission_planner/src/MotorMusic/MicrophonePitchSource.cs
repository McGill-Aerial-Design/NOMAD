// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Runtime.InteropServices;

namespace NOMAD.MissionPlanner
{
    internal sealed class PitchDetectedEventArgs : EventArgs
    {
        public double FrequencyHz { get; set; }
        public double Level { get; set; }
    }

    internal sealed class MicrophonePitchSource : IDisposable
    {
        private const int SampleRate = 16000;
        private const int BufferSamples = 2048;
        private const int BufferBytes = BufferSamples * 2;
        private const int BufferCount = 4;

        private IntPtr _handle;
        private readonly WaveInProc _callback;
        private readonly byte[][] _buffers = new byte[BufferCount][];
        private readonly GCHandle[] _bufferHandles = new GCHandle[BufferCount];
        private readonly WAVEHDR[] _headers = new WAVEHDR[BufferCount];
        private readonly GCHandle[] _headerHandles = new GCHandle[BufferCount];
        private bool _running;

        public event EventHandler<PitchDetectedEventArgs> PitchDetected;

        public MicrophonePitchSource()
        {
            _callback = WaveInCallback;
        }

        public void Start()
        {
            if (_running) return;

            var format = new WAVEFORMATEX
            {
                wFormatTag = 1,
                nChannels = 1,
                nSamplesPerSec = SampleRate,
                wBitsPerSample = 16,
            };
            format.nBlockAlign = (ushort)(format.nChannels * format.wBitsPerSample / 8);
            format.nAvgBytesPerSec = format.nSamplesPerSec * format.nBlockAlign;
            format.cbSize = 0;

            int result = waveInOpen(out _handle, UIntPtr.Zero, ref format, _callback, IntPtr.Zero, 0x00030000);
            if (result != 0)
                throw new InvalidOperationException("Unable to open microphone input.");

            for (int i = 0; i < BufferCount; i++)
            {
                _buffers[i] = new byte[BufferBytes];
                _bufferHandles[i] = GCHandle.Alloc(_buffers[i], GCHandleType.Pinned);
                _headers[i] = new WAVEHDR
                {
                    lpData = _bufferHandles[i].AddrOfPinnedObject(),
                    dwBufferLength = BufferBytes,
                };
                _headerHandles[i] = GCHandle.Alloc(_headers[i], GCHandleType.Pinned);
                waveInPrepareHeader(_handle, _headerHandles[i].AddrOfPinnedObject(), Marshal.SizeOf(typeof(WAVEHDR)));
                waveInAddBuffer(_handle, _headerHandles[i].AddrOfPinnedObject(), Marshal.SizeOf(typeof(WAVEHDR)));
            }

            result = waveInStart(_handle);
            if (result != 0)
                throw new InvalidOperationException("Unable to start microphone input.");
            _running = true;
        }

        public void Stop()
        {
            if (!_running && _handle == IntPtr.Zero) return;
            _running = false;
            try { waveInStop(_handle); } catch { }
            try { waveInReset(_handle); } catch { }

            for (int i = 0; i < BufferCount; i++)
            {
                if (_headerHandles[i].IsAllocated)
                {
                    try
                    {
                        waveInUnprepareHeader(
                            _handle,
                            _headerHandles[i].AddrOfPinnedObject(),
                            Marshal.SizeOf(typeof(WAVEHDR)));
                    }
                    catch { }
                    _headerHandles[i].Free();
                }
                if (_bufferHandles[i].IsAllocated)
                    _bufferHandles[i].Free();
            }

            if (_handle != IntPtr.Zero)
            {
                try { waveInClose(_handle); } catch { }
                _handle = IntPtr.Zero;
            }
        }

        private void WaveInCallback(IntPtr hwi, uint msg, IntPtr instance, IntPtr wavehdr, IntPtr reserved)
        {
            const uint WIM_DATA = 0x3C0;
            if (msg != WIM_DATA || !_running) return;

            try
            {
                var header = Marshal.PtrToStructure<WAVEHDR>(wavehdr);
                var samples = new short[header.dwBytesRecorded / 2];
                Marshal.Copy(header.lpData, samples, 0, samples.Length);
                Analyze(samples);
                waveInAddBuffer(_handle, wavehdr, Marshal.SizeOf(typeof(WAVEHDR)));
            }
            catch { }
        }

        private void Analyze(short[] samples)
        {
            if (samples == null || samples.Length < 256) return;

            double sumSq = 0;
            for (int i = 0; i < samples.Length; i++)
            {
                double v = samples[i] / 32768.0;
                sumSq += v * v;
            }
            double rms = Math.Sqrt(sumSq / samples.Length);
            if (rms < 0.02) return;

            double freq = EstimatePitch(samples, SampleRate, 80, 900);
            if (freq <= 0) return;
            PitchDetected?.Invoke(this, new PitchDetectedEventArgs { FrequencyHz = freq, Level = rms });
        }

        private static double EstimatePitch(short[] samples, int sampleRate, int minHz, int maxHz)
        {
            int minLag = sampleRate / maxHz;
            int maxLag = sampleRate / minHz;
            double best = 0;
            int bestLag = 0;

            for (int lag = minLag; lag <= maxLag; lag++)
            {
                double corr = 0;
                double energy = 0;
                for (int i = 0; i < samples.Length - lag; i++)
                {
                    double a = samples[i];
                    double b = samples[i + lag];
                    corr += a * b;
                    energy += a * a + b * b;
                }
                if (energy <= 0) continue;
                double score = 2.0 * corr / energy;
                if (score > best)
                {
                    best = score;
                    bestLag = lag;
                }
            }

            if (best < 0.35 || bestLag == 0) return 0;
            return sampleRate / (double)bestLag;
        }

        public void Dispose()
        {
            Stop();
        }

        private delegate void WaveInProc(IntPtr hwi, uint msg, IntPtr instance, IntPtr wavehdr, IntPtr reserved);

        [StructLayout(LayoutKind.Sequential)]
        private struct WAVEFORMATEX
        {
            public ushort wFormatTag;
            public ushort nChannels;
            public int nSamplesPerSec;
            public int nAvgBytesPerSec;
            public ushort nBlockAlign;
            public ushort wBitsPerSample;
            public ushort cbSize;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct WAVEHDR
        {
            public IntPtr lpData;
            public int dwBufferLength;
            public int dwBytesRecorded;
            public IntPtr dwUser;
            public int dwFlags;
            public int dwLoops;
            public IntPtr lpNext;
            public IntPtr reserved;
        }

        [DllImport("winmm.dll")]
        private static extern int waveInOpen(
            out IntPtr hWaveIn,
            UIntPtr deviceId,
            ref WAVEFORMATEX format,
            WaveInProc callback,
            IntPtr instance,
            uint flags);

        [DllImport("winmm.dll")]
        private static extern int waveInPrepareHeader(IntPtr hWaveIn, IntPtr header, int size);

        [DllImport("winmm.dll")]
        private static extern int waveInUnprepareHeader(IntPtr hWaveIn, IntPtr header, int size);

        [DllImport("winmm.dll")]
        private static extern int waveInAddBuffer(IntPtr hWaveIn, IntPtr header, int size);

        [DllImport("winmm.dll")]
        private static extern int waveInStart(IntPtr hWaveIn);

        [DllImport("winmm.dll")]
        private static extern int waveInStop(IntPtr hWaveIn);

        [DllImport("winmm.dll")]
        private static extern int waveInReset(IntPtr hWaveIn);

        [DllImport("winmm.dll")]
        private static extern int waveInClose(IntPtr hWaveIn);
    }
}
