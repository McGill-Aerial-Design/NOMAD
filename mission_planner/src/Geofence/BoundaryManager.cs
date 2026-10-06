// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Mission Planner advisory outline preview
// ============================================================
// Reads Mission Planner telemetry to show the current position relative to
// locally saved outlines. This class makes no vehicle or safety decisions.
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
    }

    public sealed class AdvisoryBoundaryMonitor : IDisposable
    {
        private readonly GeofenceConfig _config;
        private Timer _timer;
        private bool _disposed;

        public event EventHandler<AdvisoryBoundaryStatusEventArgs> StatusChanged;

        public AdvisoryOutlineStatus CurrentStatus { get; private set; } = AdvisoryOutlineStatus.NoPosition;

        public bool IsMonitoring { get; private set; }

        public AdvisoryBoundaryMonitor(GeofenceConfig config)
        {
            _config = config ?? new GeofenceConfig();
        }

        public void StartMonitoring(int intervalMs = 1000)
        {
            if (_disposed || IsMonitoring)
            {
                return;
            }

            _timer = new Timer(intervalMs);
            _timer.Elapsed += OnTimerElapsed;
            _timer.AutoReset = true;
            _timer.Start();
            IsMonitoring = true;
            Log.Debug("Local advisory outline preview started");
        }

        public void StopMonitoring()
        {
            if (!IsMonitoring)
            {
                return;
            }

            _timer?.Stop();
            _timer?.Dispose();
            _timer = null;
            IsMonitoring = false;
            Log.Debug("Local advisory outline preview stopped");
        }

        private void OnTimerElapsed(object sender, ElapsedEventArgs e)
        {
            try
            {
                ReadCurrentPosition();
            }
            catch (Exception ex)
            {
                Log.Error($"Advisory outline preview failed — {ex.Message}");
            }
        }

        private void ReadCurrentPosition()
        {
            var state = MainV2.comPort?.MAV?.cs;
            if (state == null || (state.lat == 0 && state.lng == 0))
            {
                SetStatus(AdvisoryOutlineStatus.NoPosition);
                return;
            }

            var position = new GpsPoint(state.lat, state.lng);
            var status = GeoMath.GetAdvisoryOutlineStatus(
                _config.SoftBoundary?.Vertices,
                _config.HardBoundary?.Vertices,
                position);
            SetStatus(status);
        }

        private void SetStatus(AdvisoryOutlineStatus status)
        {
            if (status == CurrentStatus)
            {
                return;
            }

            CurrentStatus = status;
            StatusChanged?.Invoke(this, new AdvisoryBoundaryStatusEventArgs
            {
                Status = status,
                OutlineName = GetOutlineName(status),
            });

            if (IsOutsideOutline(status) && _config.AdvisoryAudioAlertsEnabled)
            {
                AudioAlerts.Speak(
                    "Position is outside the local advisory outline. This outline is not enforced by NOMAD runtime.",
                    component: "boundary");
            }
        }

        private string GetOutlineName(AdvisoryOutlineStatus status)
        {
            if (status == AdvisoryOutlineStatus.OutsideInnerOutline)
            {
                return "Inner advisory outline";
            }

            if (status == AdvisoryOutlineStatus.OutsideOuterOutline)
            {
                return "Outer advisory outline";
            }

            return string.Empty;
        }

        private static bool IsOutsideOutline(AdvisoryOutlineStatus status)
        {
            return status == AdvisoryOutlineStatus.OutsideInnerOutline ||
                status == AdvisoryOutlineStatus.OutsideOuterOutline;
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }

            _disposed = true;
            StopMonitoring();
        }
    }
}
