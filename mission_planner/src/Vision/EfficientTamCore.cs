// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace NOMAD.MissionPlanner
{
    // EfficientTAM Ti512 video memory on four exported graphs. A prediction never changes memory; only Commit does,
    // so callers can reject an occluder or a lookalike before it is learned (upstream commits before yielding).
    internal sealed class EfficientTamCore : IDisposable
    {
        internal const int Side = 512, Grid = 32, Tokens = Grid * Grid, Channels = 256, MemoryChannels = 64, Low = 128;
        private const int SpatialMemories = 6, Pointers = 15;
        private const float NoObjectScore = -1024f;
        private static readonly float[] Mean = { .485f, .456f, .406f }, Deviation = { .229f, .224f, .225f };
        internal static readonly IReadOnlyDictionary<string, string> Hashes = new Dictionary<string, string>
        {
            ["encoder.onnx"] = "9f1322591497f2cbf53eea72bbcc14fc1e0aef9803a3e65b5e02f7015910e901",
            ["memory_attention.onnx"] = "eddb65f3c0e8d4e1d3bbadc64748e4bbda37d97848cbd5d86837c6810a86a4db",
            ["decoder.onnx"] = "36ccefadc3922c66188e14d6ee91656e6699016f26d30b4d5ed4c08674f68980",
            ["memory_encoder.onnx"] = "a2a2969f2437457ee009c2ede7f5a1e07708c4918f98973cc939a7020e1401a5",
            ["constants.bin"] = "44dbb6dadb89d28a6467776c5c2fb596b22e8da21289ff7df6e4dee5d1631565",
        };

        internal sealed class Prediction
        {
            internal float[] LowRes;   // selected 128x128 logits, already forced to NoObjectScore when absent
            internal float[] Feature;  // raw encoder feature [256,32,32] for the memory encoder
            internal float[] Pointer;  // object pointer [256]
            internal float ObjectScore, Iou;
            internal bool FromPrompt;
            internal bool Present => ObjectScore > 0;
        }

        private sealed class Memory { internal float[] Feature, Position; }

        // Loaded sessions are shared per verified folder and kept for the process: building four DirectML sessions
        // takes seconds, so reselection must not rebuild them. Each core owns only its own memory bank.
        // ponytail: process-lifetime cache (~0.3 GB VRAM once opted in); add eviction if several models are offered.
        private sealed class Model
        {
            internal InferenceSession Encoder, Attention, Decoder, MemoryEncoder;
            internal readonly float[] Temporal = new float[7 * MemoryChannels], NoMemory = new float[Channels];
            internal string Provider;
        }

        private static readonly object CacheGate = new object();
        private static readonly Dictionary<string, Model> Cache = new Dictionary<string, Model>(StringComparer.OrdinalIgnoreCase);
        private readonly Model _model;
        private Memory _conditioning;
        private float[] _conditioningPointer;
        private readonly LinkedList<Memory> _recent = new LinkedList<Memory>();
        private readonly LinkedList<float[]> _recentPointers = new LinkedList<float[]>();
        internal int CommittedFrames { get; private set; }
        internal string Provider => _model.Provider;
        // Milliseconds of the most recent call per stage: encoder, attention, decoder, memory encoder.
        internal readonly double[] StageMilliseconds = new double[4];

        internal EfficientTamCore(string directory, bool directMl)
        {
            string key = Path.GetFullPath(directory) + (directMl ? "|dml" : "|cpu");
            lock (CacheGate)
            {
                if (!Cache.TryGetValue(key, out _model)) { _model = Load(directory, directMl); Cache.Add(key, _model); }
            }
        }

        private static Model Load(string directory, bool directMl)
        {
            foreach (var file in Hashes)
            {
                string path = Path.Combine(directory, file.Key);
                if (Hash(path) != file.Value)
                { throw new InvalidDataException("EfficientTAM model file checksum mismatch: " + file.Key); }
            }
            var model = new Model();
            byte[] constants = File.ReadAllBytes(Path.Combine(directory, "constants.bin"));
            if (constants.Length != (model.Temporal.Length + model.NoMemory.Length) * 4)
            { throw new InvalidDataException("EfficientTAM constants have an unexpected size."); }
            Buffer.BlockCopy(constants, 0, model.Temporal, 0, model.Temporal.Length * 4);
            Buffer.BlockCopy(constants, model.Temporal.Length * 4, model.NoMemory, 0, model.NoMemory.Length * 4);
            GpuAdapter.Selection adapter = null;
            if (directMl)
            {
                // Only the discrete NVIDIA GPU is qualified; never fall back to an integrated or software adapter.
                adapter = GpuAdapter.FindNvidia() ?? throw new NotSupportedException("No NVIDIA DirectX 12 adapter found.");
            }
            model.Provider = directMl ? "DirectML " + adapter.Name + " (adapter " + adapter.Index + ")" : "CPU";
            try
            {
                model.Encoder = Open(directory, "encoder.onnx", directMl, adapter);
                model.Attention = Open(directory, "memory_attention.onnx", directMl, adapter);
                // The prompt decoder is small: on the CPU it is faster than a GPU round trip and leaves the GPU to
                // the renderer/encoder it shares (measured 9 ms CPU versus 18-25 ms DirectML).
                model.Decoder = Open(directory, "decoder.onnx", false, null);
                model.MemoryEncoder = Open(directory, "memory_encoder.onnx", directMl, adapter);
            }
            catch
            {
                model.Encoder?.Dispose(); model.Attention?.Dispose(); model.Decoder?.Dispose(); model.MemoryEncoder?.Dispose();
                throw;
            }
            return model;
        }

        private static InferenceSession Open(string directory, string name, bool directMl, GpuAdapter.Selection adapter)
        {
            using (var options = new SessionOptions { GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL })
            {
                if (directMl)
                {
                    options.EnableMemoryPattern = false;
                    options.ExecutionMode = ExecutionMode.ORT_SEQUENTIAL;
                    options.AppendExecutionProvider_DML(adapter.Index);
                }
                else { options.IntraOpNumThreads = 2; options.InterOpNumThreads = 1; }
                return new InferenceSession(Path.Combine(directory, name), options);
            }
        }

        // RGB bytes (row-major, 3 per pixel) of one Side x Side crop into normalized CHW floats.
        internal static float[] Normalize(byte[] rgb)
        {
            if (rgb == null || rgb.Length != Side * Side * 3) { throw new ArgumentException("Expected a 512x512 RGB crop."); }
            var image = new float[3 * Side * Side];
            for (int i = 0; i < Side * Side; i++)
            for (int c = 0; c < 3; c++)
            { image[c * Side * Side + i] = ((float)(rgb[i * 3 + c] / 255.0) - Mean[c]) / Deviation[c]; }
            return image;
        }

        // Conditioning frame: box corners in crop pixels; no memory (directly adds the no-memory embedding).
        internal Prediction Prompt(float[] image, float x1, float y1, float x2, float y2)
        {
            Reset();
            float[] feature = Encode(image, out _);
            var pix = (float[])feature.Clone();
            for (int c = 0; c < Channels; c++)
            for (int i = 0; i < Tokens; i++) { pix[c * Tokens + i] += _model.NoMemory[c]; }
            var outputs = Decode(pix, new[] { x1, y1, x2, y2 }, new[] { 2, 3 });
            // Two prompt points exceed the multimask range: the single-mask output and its token, as upstream
            // with apply_postprocessing off (no stability fallback, no binarization, no hole filling).
            return Select(outputs, 0, 0, feature, true);
        }

        internal Prediction Track(float[] image)
        {
            if (_conditioning == null) { throw new InvalidOperationException("Prompt and commit a conditioning frame first."); }
            float[] feature = Encode(image, out float[] position);
            float[] pix = Attend(feature, position);
            var outputs = Decode(pix, new[] { 0f, 0f }, new[] { -1 });
            int choice = 1 + ArgMax(outputs.Ious, 1);
            return Select(outputs, choice, choice, feature, false);
        }

        // Writes memory for an admitted prediction; rejected predictions are simply dropped.
        internal void Commit(Prediction prediction)
        {
            if (prediction == null || prediction.LowRes == null) { throw new ArgumentNullException(nameof(prediction)); }
            float[] high = Upsample(prediction.LowRes);
            var feeds = new[]
            {
                NamedOnnxValue.CreateFromTensor("pix_feat", new DenseTensor<float>(prediction.Feature, new[] { 1, Channels, Grid, Grid })),
                NamedOnnxValue.CreateFromTensor("high_res_mask", new DenseTensor<float>(high, new[] { 1, 1, Side, Side })),
                NamedOnnxValue.CreateFromTensor("binarize", new DenseTensor<float>(new[] { 0f }, new[] { 1 })),
            };
            Memory memory;
            using (var results = Timed(3, () => _model.MemoryEncoder.Run(feeds)))
            {
                // Upstream stores spatial memory as bfloat16; round identically so recurrence matches it.
                memory = new Memory { Feature = Bfloat16(Output(results, "features")), Position = Output(results, "pos") };
            }
            if (prediction.FromPrompt) { _conditioning = memory; _conditioningPointer = prediction.Pointer; }
            else
            {
                _recent.AddLast(memory); _recentPointers.AddLast(prediction.Pointer);
                while (_recent.Count > SpatialMemories) { _recent.RemoveFirst(); }
                while (_recentPointers.Count > Pointers) { _recentPointers.RemoveFirst(); }
            }
            CommittedFrames++;
        }

        internal void Reset()
        {
            _conditioning = null; _conditioningPointer = null; _recent.Clear(); _recentPointers.Clear(); CommittedFrames = 0;
        }

        private float[] Encode(float[] image, out float[] position)
        {
            if (image == null || image.Length != 3 * Side * Side) { throw new ArgumentException("Expected a normalized 512x512 image."); }
            var feeds = new[] { NamedOnnxValue.CreateFromTensor("image", new DenseTensor<float>(image, new[] { 1, 3, Side, Side })) };
            using (var results = Timed(0, () => _model.Encoder.Run(feeds)))
            {
                position = Output(results, "pos");
                return Output(results, "feat");
            }
        }

        // Upstream order: conditioning memory first (temporal slot 6), then oldest to newest recent memories,
        // newest using slot 0; pointers are conditioning then newest to oldest, four 64-wide tokens each.
        private float[] Attend(float[] feature, float[] position)
        {
            var spatial = new List<Memory> { _conditioning };
            var slots = new List<int> { 6 };
            int age = _recent.Count;
            foreach (var memory in _recent) { spatial.Add(memory); slots.Add(age - 1); age--; }
            var pointers = new List<float[]> { _conditioningPointer };
            for (var node = _recentPointers.Last; node != null; node = node.Previous) { pointers.Add(node.Value); }
            var memoryFeatures = new float[spatial.Count * Tokens * MemoryChannels];
            var memoryPositions = new float[memoryFeatures.Length];
            for (int m = 0; m < spatial.Count; m++)
            for (int i = 0; i < Tokens; i++)
            for (int c = 0; c < MemoryChannels; c++)
            {
                int at = (m * Tokens + i) * MemoryChannels + c, source = c * Tokens + i;
                memoryFeatures[at] = spatial[m].Feature[source];
                memoryPositions[at] = spatial[m].Position[source] + _model.Temporal[slots[m] * MemoryChannels + c];
            }
            var pointerTokens = new float[pointers.Count * Channels];
            for (int p = 0; p < pointers.Count; p++) { Array.Copy(pointers[p], 0, pointerTokens, p * Channels, Channels); }
            var feeds = new[]
            {
                NamedOnnxValue.CreateFromTensor("curr", new DenseTensor<float>(TokensFirst(feature, Channels), new[] { Tokens, 1, Channels })),
                NamedOnnxValue.CreateFromTensor("curr_pos", new DenseTensor<float>(TokensFirst(position, Channels), new[] { Tokens, 1, Channels })),
                NamedOnnxValue.CreateFromTensor("spatial", new DenseTensor<float>(memoryFeatures, new[] { spatial.Count * Tokens, 1, MemoryChannels })),
                NamedOnnxValue.CreateFromTensor("spatial_pos", new DenseTensor<float>(memoryPositions, new[] { spatial.Count * Tokens, 1, MemoryChannels })),
                NamedOnnxValue.CreateFromTensor("ptr", new DenseTensor<float>(pointerTokens, new[] { pointers.Count * 4, 1, MemoryChannels })),
                NamedOnnxValue.CreateFromTensor("ptr_pos", new DenseTensor<float>(new float[pointers.Count * Channels], new[] { pointers.Count * 4, 1, MemoryChannels })),
            };
            float[] conditioned;
            using (var results = Timed(1, () => _model.Attention.Run(feeds))) { conditioned = Output(results, "conditioned"); }
            var pix = new float[Channels * Tokens];
            for (int i = 0; i < Tokens; i++)
            for (int c = 0; c < Channels; c++) { pix[c * Tokens + i] = conditioned[i * Channels + c]; }
            return pix;
        }

        [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
        private struct Bits
        {
            [System.Runtime.InteropServices.FieldOffset(0)] internal float Value;
            [System.Runtime.InteropServices.FieldOffset(0)] internal uint Raw;
        }

        // float32 -> bfloat16 -> float32 with round-to-nearest-even (finite inputs).
        internal static float[] Bfloat16(float[] values)
        {
            var bits = new Bits();
            for (int i = 0; i < values.Length; i++)
            {
                bits.Value = values[i];
                uint raw = bits.Raw;
                bits.Raw = (raw + 0x7FFFu + ((raw >> 16) & 1u)) & 0xFFFF0000u;
                values[i] = bits.Value;
            }
            return values;
        }

        private static float[] TokensFirst(float[] chw, int channels)
        {
            var result = new float[chw.Length];
            for (int c = 0; c < channels; c++)
            for (int i = 0; i < Tokens; i++) { result[i * channels + c] = chw[c * Tokens + i]; }
            return result;
        }

        private sealed class Decoded { internal float[] Masks, Ious, Pointers; internal float Score; }

        private Decoded Decode(float[] pix, float[] points, int[] labels)
        {
            var feeds = new[]
            {
                NamedOnnxValue.CreateFromTensor("pix_feat", new DenseTensor<float>(pix, new[] { 1, Channels, Grid, Grid })),
                NamedOnnxValue.CreateFromTensor("points", new DenseTensor<float>(points, new[] { 1, labels.Length, 2 })),
                NamedOnnxValue.CreateFromTensor("labels", new DenseTensor<int>(labels, new[] { 1, labels.Length })),
            };
            using (var results = Timed(2, () => _model.Decoder.Run(feeds)))
            {
                return new Decoded { Masks = Output(results, "low_res_masks"), Ious = Output(results, "ious"),
                    Pointers = Output(results, "obj_ptrs"), Score = Output(results, "object_score")[0] };
            }
        }

        private static Prediction Select(Decoded outputs, int mask, int pointer, float[] feature, bool prompt)
        {
            var low = new float[Low * Low];
            if (outputs.Score > 0) { Array.Copy(outputs.Masks, mask * Low * Low, low, 0, low.Length); }
            else { for (int i = 0; i < low.Length; i++) { low[i] = NoObjectScore; } }
            var ptr = new float[Channels];
            Array.Copy(outputs.Pointers, pointer * Channels, ptr, 0, Channels);
            return new Prediction { LowRes = low, Feature = feature, Pointer = ptr, ObjectScore = outputs.Score,
                Iou = outputs.Ious[mask], FromPrompt = prompt };
        }

        private static int ArgMax(float[] values, int start)
        {
            int best = start;
            for (int i = start + 1; i < values.Length; i++) { if (values[i] > values[best]) { best = i; } }
            return best - start;
        }

        // torch.nn.functional.interpolate(bilinear, align_corners=False) from 128 to 512.
        internal static float[] Upsample(float[] low)
        {
            var result = new float[Side * Side];
            var index = new int[Side * 2];
            var weight = new float[Side];
            for (int d = 0; d < Side; d++)
            {
                float source = Math.Max(0f, (d + .5f) * (Low / (float)Side) - .5f);
                int i0 = (int)source;
                index[d * 2] = i0; index[d * 2 + 1] = Math.Min(i0 + 1, Low - 1); weight[d] = source - i0;
            }
            for (int y = 0; y < Side; y++)
            {
                int r0 = index[y * 2] * Low, r1 = index[y * 2 + 1] * Low;
                float wy = weight[y];
                for (int x = 0; x < Side; x++)
                {
                    int c0 = index[x * 2], c1 = index[x * 2 + 1];
                    float wx = weight[x];
                    float top = low[r0 + c0] + (low[r0 + c1] - low[r0 + c0]) * wx;
                    float bottom = low[r1 + c0] + (low[r1 + c1] - low[r1 + c0]) * wx;
                    result[y * Side + x] = top + (bottom - top) * wy;
                }
            }
            return result;
        }

        private IDisposableReadOnlyCollection<DisposableNamedOnnxValue> Timed(int stage,
            Func<IDisposableReadOnlyCollection<DisposableNamedOnnxValue>> run)
        {
            var clock = System.Diagnostics.Stopwatch.StartNew();
            var results = run();
            StageMilliseconds[stage] = clock.Elapsed.TotalMilliseconds;
            return results;
        }

        private static float[] Output(IDisposableReadOnlyCollection<DisposableNamedOnnxValue> results, string name) =>
            results.First(value => value.Name == name).AsTensor<float>().ToArray();

        private static string Hash(string path)
        {
            using (var sha = SHA256.Create())
            using (var stream = File.OpenRead(path))
            { return BitConverter.ToString(sha.ComputeHash(stream)).Replace("-", "").ToLowerInvariant(); }
        }

        // Releases this track's memory only; the shared sessions stay loaded for the next selection.
        public void Dispose() => Reset();
    }
}
