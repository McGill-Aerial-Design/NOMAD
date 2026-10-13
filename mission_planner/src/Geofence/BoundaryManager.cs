// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Mission Planner advisory outline preview
// ============================================================
// Live host-received positions against local outlines; no vehicle decisions or command path.
// ============================================================

using System;
using System.Timers;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public class AdvisoryBoundaryStatusEventArgs : EventArgs
    {
        public AdvisoryOutlineStatus Status { get; set; }
        public string OutlineName { get; set; }
        public GpsPoint Position { get; set; }
    }

    public sealed class AdvisoryBoundaryMonitor : IDisposable
    {
        private readonly GeofenceConfig _config;
        private readonly AdvisoryPosition _position;
        private readonly object _gate = new object();
        private Timer _timer;
        private bool _disposed, _monitoring, _sampling;
        private int _generation;
        private PositionContext _context;
        private AdvisoryBoundaryStatusEventArgs _current = new AdvisoryBoundaryStatusEventArgs
        {
            Status = AdvisoryOutlineStatus.NoPosition, OutlineName = ""
        };

        private sealed class PositionContext
        {
            internal MAVLinkInterface Port;
            internal MAVState Mav;
            internal object Stream;
            internal byte SystemId, ComponentId;
            internal bool Live;
            internal MAVLink.MAVLinkMessage[] Packets;
        }

        public event EventHandler<AdvisoryBoundaryStatusEventArgs> StatusChanged;
        public AdvisoryOutlineStatus CurrentStatus => CurrentObservation.Status;
        public bool IsMonitoring { get { lock (_gate) { return _monitoring; } } }
        public AdvisoryBoundaryStatusEventArgs CurrentObservation => ReadCurrentObservation();

        public AdvisoryBoundaryMonitor(GeofenceConfig config) : this(config, new RouterClock()) { }

        internal AdvisoryBoundaryMonitor(GeofenceConfig config, RouterClock clock)
        {
            _config = config ?? new GeofenceConfig();
            _position = new AdvisoryPosition(clock);
        }

        public void StartMonitoring(int intervalMs = 1000)
        {
            lock (_gate)
            {
                if (_disposed || _monitoring)
                {
                    return;
                }
                _timer = new Timer(intervalMs);
                _timer.Elapsed += OnTimerElapsed;
                _timer.AutoReset = true;
                _context = null;
                _generation++;
                _monitoring = true;
                _timer.Start();
            }
            Log.Debug("Local advisory outline preview started");
        }

        public void StopMonitoring()
        {
            Stop(false);
        }

        private void Stop(bool disposing)
        {
            AdvisoryBoundaryStatusEventArgs change;
            int generation;
            lock (_gate)
            {
                _disposed |= disposing;
                _monitoring = false;
                _timer?.Stop();
                _timer?.Dispose();
                _timer = null;
                _context = null;
                _position.Reject();
                generation = ++_generation;
                change = SetObservation(null);
            }
            Publish(generation, change);
        }

        private void OnTimerElapsed(object sender, ElapsedEventArgs e)
        {
            SamplePosition(sender);
        }

        internal void SamplePosition()
        {
            SamplePosition(_timer);
        }

        private void SamplePosition(object timer)
        {
            int generation;
            lock (_gate)
            {
                if (_disposed || !_monitoring || _sampling || !ReferenceEquals(timer, _timer))
                {
                    return;
                }
                _sampling = true;
                generation = _generation;
            }
            try
            {
                var context = CaptureContext();
                if (!ContextStillCurrent(context))
                {
                    context = null;
                }
                Publish(generation, Commit(generation, context));
            }
            catch (Exception ex)
            {
                // Clear the prior observation before notifying or logging a sampling failure.
                try { Publish(generation, Commit(generation, null)); }
                catch { }
                Log.Error($"Advisory outline preview failed — {ex.Message}");
            }
            finally
            {
                lock (_gate) { _sampling = false; }
            }
        }

        private static PositionContext CaptureContext()
        {
            var port = MainV2.comPort;
            var mav = port?.MAV;
            var stream = port?.BaseStream;
            return new PositionContext
            {
                Port = port, Mav = mav, Stream = stream,
                SystemId = mav?.sysid ?? 0, ComponentId = mav?.compid ?? 0,
                Live = port != null && mav != null && stream != null && stream.IsOpen && !port.logreadmode,
                Packets = new[]
                {
                    mav?.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT),
                    mav?.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY),
                    mav?.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY2),
                    mav?.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT)
                }
            };
        }

        private static bool ContextStillCurrent(PositionContext context)
        {
            var port = MainV2.comPort;
            var mav = port?.MAV;
            var stream = port?.BaseStream;
            return ReferenceEquals(port, context.Port) && ReferenceEquals(mav, context.Mav) &&
                ReferenceEquals(stream, context.Stream) && (mav?.sysid ?? 0) == context.SystemId &&
                (mav?.compid ?? 0) == context.ComponentId &&
                (port != null && mav != null && stream != null && stream.IsOpen && !port.logreadmode) == context.Live;
        }

        private AdvisoryBoundaryStatusEventArgs Commit(int generation, PositionContext context)
        {
            lock (_gate)
            {
                if (generation != _generation || !_monitoring || _disposed)
                {
                    return null;
                }
                if (context == null)
                {
                    _context = null;
                    _position.Reject();
                    return SetObservation(null);
                }
                bool changed = _context == null || !ReferenceEquals(_context.Port, context.Port) ||
                    !ReferenceEquals(_context.Mav, context.Mav) || !ReferenceEquals(_context.Stream, context.Stream) ||
                    _context.SystemId != context.SystemId || _context.ComponentId != context.ComponentId ||
                    _context.Live != context.Live;
                _context = context;
                if (changed || !context.Live)
                {
                    // A context transition blocks its existing cache; only a subsequent receipt can restore a point.
                    _position.Block(context.Packets);
                    return SetObservation(null);
                }
                return SetObservation(_position.Read(context.Packets, context.SystemId, context.ComponentId));
            }
        }

        private AdvisoryBoundaryStatusEventArgs SetObservation(GpsPoint position)
        {
            var status = GeoMath.GetAdvisoryOutlineStatus(
                _config.SoftBoundary?.Vertices, _config.HardBoundary?.Vertices, position);
            bool changed = status != _current.Status;
            _current = new AdvisoryBoundaryStatusEventArgs
            {
                Status = status, Position = position,
                OutlineName = status == AdvisoryOutlineStatus.OutsideInnerOutline ? "Inner advisory outline" :
                    status == AdvisoryOutlineStatus.OutsideOuterOutline ? "Outer advisory outline" : ""
            };
            return changed ? CopyCurrent() : null;
        }

        private AdvisoryBoundaryStatusEventArgs CopyCurrent() => new AdvisoryBoundaryStatusEventArgs
        {
            Status = _current.Status, OutlineName = _current.OutlineName,
            Position = _current.Position == null ? null : new GpsPoint(_current.Position.Lat, _current.Position.Lon)
        };

        private bool CanPublish(int generation, AdvisoryOutlineStatus status)
        {
            ReadCurrentObservation();
            lock (_gate) { return generation == _generation && status == _current.Status; }
        }

        private AdvisoryBoundaryStatusEventArgs ReadCurrentObservation()
        {
            while (true)
            {
                AdvisoryBoundaryStatusEventArgs observed;
                PositionContext context;
                int generation;
                lock (_gate)
                {
                    observed = _current;
                    if (observed.Position == null) { return CopyCurrent(); }
                    context = _context;
                    generation = _generation;
                }
                bool live = false;
                Exception failure = null;
                try { live = context != null && ContextStillCurrent(context); }
                catch (Exception ex) { failure = ex; }
                AdvisoryBoundaryStatusEventArgs change;
                lock (_gate)
                {
                    if (!ReferenceEquals(observed, _current)) { continue; }
                    try
                    {
                        if (live && _position.HasFreshPosition) { return CopyCurrent(); }
                    }
                    catch (Exception ex) { failure = ex; }
                    if (!live || failure != null) { _context = null; }
                    _position.Reject();
                    change = SetObservation(null);
                }
                // Validate the already selected receipt. Snapshot/publication checks never fetch or reselect packets.
                Publish(generation, change);
                if (failure != null) { Log.Error($"Advisory observation failed — {failure.Message}"); }
            }
        }

        private void Publish(int generation, AdvisoryBoundaryStatusEventArgs change)
        {
            if (change == null)
            {
                return;
            }
            var handlers = StatusChanged;
            if (handlers != null)
            {
                foreach (EventHandler<AdvisoryBoundaryStatusEventArgs> handler in handlers.GetInvocationList())
                {
                    if (!CanPublish(generation, change.Status))
                    {
                        return;
                    }
                    handler(this, change);
                }
            }
            if (CanPublish(generation, change.Status) && _config.AdvisoryAudioAlertsEnabled &&
                (change.Status == AdvisoryOutlineStatus.OutsideInnerOutline ||
                 change.Status == AdvisoryOutlineStatus.OutsideOuterOutline))
            {
                AudioAlerts.Speak(
                    "Position is outside the local advisory outline. This outline is not enforced by NOMAD runtime.",
                    component: "boundary");
            }
        }

        public void Dispose()
        {
            Stop(true);
        }
    }
}
