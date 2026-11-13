// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Text;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    // Private, version-matched protocol. Pixel bytes never enter the control stream.
    internal sealed class MediaMessage
    {
        [JsonProperty(Required = Required.Always)] public int Version = 1;
        public long Request { get; set; }
        public long Resource { get; set; }
        public long Epoch { get; set; }
        public long Ticks { get; set; }
        public long UtcTicks { get; set; }
        public string Operation { get; set; }
        public string Secret { get; set; }
        public string Mapping { get; set; }
        public string Text { get; set; }
        public string Model { get; set; }
        public string Error { get; set; }
        public int Width { get; set; }
        public int Height { get; set; }
        public int Capacity { get; set; }
        public int Mode { get; set; }
        public int State { get; set; }
        public bool Gpu { get; set; }
        public bool Discontinuous { get; set; }
        public bool End { get; set; }
        public bool NeedBuffer { get; set; }
        public RectangleF? Bounds;
        public PointF? Aim, Hint;
        public MediaDetection[] Detections;
    }

    internal sealed class MediaDetection
    {
        public int Class;
        public string Label;
        public float Confidence;
        public RectangleF Bounds;
        internal VideoDetection ToDetection() => new VideoDetection(Class, Label, Confidence, Bounds);
        internal static MediaDetection From(VideoDetection value) => new MediaDetection
        { Class = value.ClassId, Label = value.Label, Confidence = value.Confidence, Bounds = value.Bounds };
    }

    internal static class MediaWire
    {
        internal const int Version = 1, MaximumMessage = 1024 * 1024;
        internal const int MaximumPixels = 64 * 1024 * 1024;
        internal const string NvidiaUnavailable = "No NVIDIA DirectX 12 adapter found.";
        internal const string VisualCppRequired = "Vision needs the Microsoft Visual C++ 2015-2022 " +
            "Redistributable (x64) 14.44 or later. Install the latest version from Microsoft.";
        internal const string OperationUnavailable = "Media operation unavailable; review the model or source.";
        internal static string OperatorError(string message) =>
            message == NvidiaUnavailable || message == VisualCppRequired ? message : OperationUnavailable;
        private static readonly UTF8Encoding Utf8 = new UTF8Encoding(false, true);
        private static readonly JsonSerializerSettings Settings = new JsonSerializerSettings
        { TypeNameHandling = TypeNameHandling.None, MissingMemberHandling = MissingMemberHandling.Error, MaxDepth = 16 };

        internal static int PixelBytes(int width, int height)
        {
            if (width < 1 || height < 1 || width > 16384 || height > 16384 ||
                (long)width * height > MaximumPixels)
            { throw new InvalidDataException("Media frame dimensions exceed the supported limit."); }
            return checked(width * height * 4);
        }

        internal static void Write(Stream stream, MediaMessage message)
        {
            byte[] bytes = Utf8.GetBytes(JsonConvert.SerializeObject(message, Settings));
            if (bytes.Length < 1 || bytes.Length > MaximumMessage) { throw new InvalidDataException("Media message too large."); }
            var writer = new BinaryWriter(stream, Utf8, true);
            writer.Write(bytes.Length); writer.Write(bytes); writer.Flush();
        }

        internal static MediaMessage Read(Stream stream)
        {
            var reader = new BinaryReader(stream, Utf8, true);
            int count = reader.ReadInt32();
            if (count < 1 || count > MaximumMessage) { throw new InvalidDataException("Invalid media message length."); }
            byte[] bytes = reader.ReadBytes(count);
            if (bytes.Length != count) { throw new EndOfStreamException(); }
            MediaMessage result;
            try { result = JsonConvert.DeserializeObject<MediaMessage>(Utf8.GetString(bytes), Settings); }
            catch (Exception error) when (error is JsonException || error is DecoderFallbackException)
            { throw new InvalidDataException("Invalid media message."); }
            if (result == null || result.Version != Version || result.Request < 0 ||
                (result.Detections != null && result.Detections.Length > 4096))
            { throw new InvalidDataException("Invalid media message."); }
            return result;
        }
    }

    internal interface IVideoPipeline : IDisposable
    {
        void Start(string pipeline, System.Threading.CancellationToken cancellation);
        Bitmap ReadFrame(System.Threading.CancellationToken cancellation, out long receivedTicks, out DateTime receivedUtc);
    }
}
