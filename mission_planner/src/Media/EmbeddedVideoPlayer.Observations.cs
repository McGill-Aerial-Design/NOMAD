// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        internal string SaveObservation()
        {
            ValidateUiThread();
            var frame = _displayedVideoFrame;
            if (frame?.Fresh != true) { SetTrackingStatus("No fresh frame available to save"); return null; }
            string png = null, json = null;
            bool createdPng = false, createdJson = false;
            try
            {
                string directory = ObservationDirectory ?? Path.Combine(NOMADStorage.PluginsDirectory,
                    "NOMAD", "observations");
                Directory.CreateDirectory(directory);
                string name = "NOMAD_observation_" + DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff") + "_" +
                    Guid.NewGuid().ToString("N").Substring(0, 8);
                png = Path.Combine(directory, name + ".png");
                json = Path.Combine(directory, name + ".json");
                object[] transitions;
                RectangleF? selected;
                string label, selectionDetector;
                long regionRevision;
                float pitch, yaw;
                long commandGeneration, commandVideoGeneration;
                lock (_visionGate)
                {
                    transitions = _trackingEvents.ToArray(); selected = _selectedRegion;
                    label = _selectedLabel; regionRevision = _regionRevision;
                    selectionDetector = _selectionDetectorIdentity;
                    pitch = _commandedPitch; yaw = _commandedYaw;
                    commandGeneration = _commandGeneration; commandVideoGeneration = _commandVideoGeneration;
                }
                bool currentCommand = commandGeneration == OutputController.Generation &&
                    commandVideoGeneration == frame.Generation;
                var metadata = new
                {
                    SchemaVersion = 2, SavedUtc = DateTime.UtcNow, InputName = _input.Name,
                    frame.Sequence, frame.ReceivedUtc, LocalFrameAgeSeconds = frame.AgeSeconds,
                    Width = frame.Image.Width, Height = frame.Image.Height,
                    DetectorIdentity = CurrentDetections(frame) ? frame.DetectorIdentity : "",
                    Detections = (CurrentDetections(frame) ? frame.Detections : Array.Empty<VideoDetection>())
                        .Select(DetectionMetadata).ToArray(),
                    SelectedTarget = selected.HasValue && frame.Region?.Revision == regionRevision
                        ? new { OptionalDetectorLabel = label, SelectionDetectorIdentity = selectionDetector,
                            Mode = frame.Region.Mode.ToString(), State = frame.Region.State.ToString(),
                            Tracker = frame.Region.Mode == VisualTrackingMode.Area ? "OpenCV ORB planar reference" : "OpenCV CSRT",
                            Found = frame.Region.Found, ReferenceRetained = _session.HasRegion(regionRevision),
                            Box = frame.Region.Found ? frame.Region.Bounds : null,
                            CanonicalAimPoint = frame.Region.Found ? frame.Region.AimPoint : null,
                            EstimatedGroundSearchHint = frame.Motion?.IsCurrent == true ? frame.Region.GroundSearchHint : null }
                        : null,
                    AircraftPosition = ObservationAircraftPosition(),
                    TargetPosition = new { Available = false, Reason = "No target range or georeferencing is available." },
                    LastSubmittedTrackingRequest = new { PitchDeg = currentCommand ? (float?)pitch : null,
                        YawDeg = currentCommand ? (float?)yaw : null, RollDeg = currentCommand ? (int?)0 : null,
                        Available = currentCommand, ObservedPhysicalAngles = false },
                    TrackingStatus, LifetimeSubmittedTrackingCommands = SubmittedTrackingCommands, Transitions = transitions,
                };
                using (var image = (Bitmap)frame.Image.Clone())
                using (var output = new FileStream(png, FileMode.CreateNew, FileAccess.Write))
                { createdPng = true; image.Save(output, System.Drawing.Imaging.ImageFormat.Png); }
                using (var output = new StreamWriter(new FileStream(json, FileMode.CreateNew, FileAccess.Write)))
                { createdJson = true; output.Write(JsonConvert.SerializeObject(metadata, Formatting.Indented)); }
                LastObservationPath = png;
                SetTrackingStatus("Observation saved: " + name);
                return png;
            }
            catch (Exception)
            {
                try
                {
                    if (createdPng && File.Exists(png)) { File.Delete(png); }
                    if (createdJson && File.Exists(json)) { File.Delete(json); }
                }
                catch (IOException) { }
                catch (UnauthorizedAccessException) { }
                SetTrackingStatus("Observation could not be saved; check the output folder");
                return null;
            }
        }

        private static object DetectionMetadata(VideoDetection detection) => new
        {
            detection.ClassId, detection.Label, detection.Confidence,
            Box = new { detection.Bounds.X, detection.Bounds.Y, detection.Bounds.Width, detection.Bounds.Height },
        };

        private object ObservationAircraftPosition()
        {
            var target = _aircraft == null ? null : OutputController.CaptureTarget(_aircraft);
            var mav = target?.Connection.MAV;
            if (!FreshAircraft(target, mav)) { return new { Available = false }; }
            var packet = mav.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT);
            if (packet == null || packet.sysid != target.System || packet.compid != target.Component ||
                packet.rxtime.Kind != DateTimeKind.Utc) { return new { Available = false }; }
            double age = (DateTime.UtcNow - packet.rxtime).TotalSeconds;
            if (age < 0 || age > 3.5 || mav.cs.gpsstatus < 3) { return new { Available = false }; }
            var position = packet.ToStructure<MAVLink.mavlink_global_position_int_t>();
            if (Math.Abs(position.lat / 1e7) > 90 || Math.Abs(position.lon / 1e7) > 180)
            { return new { Available = false }; }
            return new { Available = true, LatitudeDeg = position.lat / 1e7, LongitudeDeg = position.lon / 1e7,
                AltitudeMslMeters = position.alt / 1000.0, PacketReceivedUtc = packet.rxtime,
                Source = "Selected aircraft GLOBAL_POSITION_INT" };
        }
    }
}
