// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal class MediaWorkerPipeline : IVideoPipeline
    {
        private MediaChannel _channel;
        internal static IVideoPipeline Create() => new MediaWorkerPipeline();
        public void Start(string pipeline, CancellationToken token)
        {
            _channel = MediaWorkerHost.Open(token);
            _channel.Call(new MediaMessage { Operation = "pipeline" }, token);
            _channel.Call(new MediaMessage { Operation = "start", Text = pipeline }, token);
        }
        public Bitmap ReadFrame(CancellationToken token, out long ticks, out DateTime utc) =>
            _channel.ReadFrame(token, out ticks, out utc);
        public void Dispose() { _channel?.StopAndWait(); }
    }

    internal sealed class MediaWorkerDetector : IVideoDetector
    {
        private readonly MediaChannel _channel;
        public string Identity { get; }
        internal MediaWorkerDetector(string path, CancellationToken token)
        {
            _channel = MediaWorkerHost.Open(token);
            try { Identity = _channel.Call(new MediaMessage { Operation = "detector", Model = path ?? "" }, token).Text; }
            catch { _channel.StopAndWait(); throw; }
        }
        public IReadOnlyList<VideoDetection> Detect(Bitmap image, CancellationToken token)
        {
            var reply = _channel.ImageCall("detect", image, token);
            var detections = (reply.Detections ?? Array.Empty<MediaDetection>()).Select(value => value.ToDetection()).ToArray();
            if (detections.Any(value => value.Bounds.Right > image.Width || value.Bounds.Bottom > image.Height))
            { throw new System.IO.InvalidDataException("Media detection exceeds its source image."); }
            return detections;
        }
        public void Dispose() { _channel.StopAndWait(); }
    }

    internal sealed class MediaWorkerTracker : IVisualTracker
    {
        private readonly MediaChannel _channel;
        internal string Provider { get; }
        internal MediaWorkerTracker(CancellationToken token, VisualTrackingMode mode, bool gpu)
        {
            _channel = MediaWorkerHost.Open(token);
            try { Provider = _channel.Call(new MediaMessage { Operation = "tracker", Mode = (int)mode, Gpu = gpu }, token).Text; }
            catch { _channel.StopAndWait(); throw; }
        }
        public void Initialize(Bitmap image, RectangleF bounds, CancellationToken token, VisualFrameContext context = default)
        { _channel.ImageCall("initialize", image, token, bounds, context); }
        public VisualTrackingResult Update(Bitmap image, CancellationToken token, VisualFrameContext context = default)
        {
            var reply = _channel.ImageCall("track", image, token, context: context);
            var result = new VisualTrackingResult((VisualTrackingState)reply.State, reply.Text, reply.Bounds, reply.Aim, reply.Hint);
            if ((result.Bounds.HasValue && (result.Bounds.Value.Right > image.Width || result.Bounds.Value.Bottom > image.Height)) ||
                (result.AimPoint.HasValue && (result.AimPoint.Value.X < 0 || result.AimPoint.Value.Y < 0 ||
                    result.AimPoint.Value.X >= image.Width || result.AimPoint.Value.Y >= image.Height)))
            { throw new System.IO.InvalidDataException("Media tracking exceeds its source image."); }
            return result;
        }
        public void Dispose() { _channel.StopAndWait(); }
    }
}
