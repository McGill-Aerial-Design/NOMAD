// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Flight Boundary Manager
// ============================================================
// Monitors the drone against the configured geofence.
// Supports soft (warning) and hard (kill required) boundaries.
// Provides real-time boundary checking and warnings.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Media;
using System.Timers;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Boundary violation event arguments.
    /// </summary>
    public class BoundaryViolationEventArgs : EventArgs
    {
        public string BoundaryType { get; set; } // "soft" or "hard"
        public string BoundaryName { get; set; }
        public GpsPoint DronePosition { get; set; }
        public double? AltitudeAgl { get; set; }
        public string RequiredAction { get; set; }
        public DateTime Timestamp { get; set; }
    }

    /// <summary>
    /// Boundary status changed event arguments.
    /// </summary>
    public class BoundaryStatusEventArgs : EventArgs
    {
        public string Status { get; set; } // "inside", "soft_violation", "hard_violation"
        public double? DistanceToBoundaryMeters { get; set; }
        public string NearestBoundaryName { get; set; }
    }

    /// <summary>
    /// Manages flight boundaries and provides violation monitoring.
    /// </summary>
    public class BoundaryMonitor : IDisposable
    {
        private readonly GeofenceConfig _geofence;
        private readonly NOMADConfig _config;
        private System.Timers.Timer _monitorTimer;
        private string _lastStatus = "inside";
        private DateTime? _hardViolationStart;
        private bool _terminationReported;   // one request report per violation episode
        private bool _returnCommanded;        // one return-to-boundary goto per episode
        private bool _isDisposed;

        /// <summary>
        /// Fired when a boundary violation is detected.
        /// </summary>
        public event EventHandler<BoundaryViolationEventArgs> BoundaryViolation;

        /// <summary>
        /// Fired when boundary status changes.
        /// </summary>
        public event EventHandler<BoundaryStatusEventArgs> BoundaryStatusChanged;

        /// <summary>
        /// Current boundary status.
        /// </summary>
        public string CurrentStatus { get; private set; } = "inside";

        /// <summary>
        /// Is boundary monitoring active.
        /// </summary>
        public bool IsMonitoring { get; private set; }

        public BoundaryMonitor(GeofenceConfig geofenceConfig, NOMADConfig config)
        {
            _geofence = geofenceConfig;
            _config = config;
        }

        /// <summary>
        /// Start boundary monitoring.
        /// </summary>
        public void StartMonitoring(int intervalMs = 500)
        {
            if (IsMonitoring) return;

            _monitorTimer = new System.Timers.Timer(intervalMs);
            _monitorTimer.Elapsed += MonitorTimer_Elapsed;
            _monitorTimer.AutoReset = true;
            _monitorTimer.Start();
            IsMonitoring = true;

            Log.Debug("Boundary monitoring started");
        }

        /// <summary>
        /// Stop boundary monitoring.
        /// </summary>
        public void StopMonitoring()
        {
            if (!IsMonitoring) return;

            _monitorTimer?.Stop();
            _monitorTimer?.Dispose();
            _monitorTimer = null;
            IsMonitoring = false;
            _hardViolationStart = null;

            Log.Debug("Boundary monitoring stopped");
        }

        private void MonitorTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            try
            {
                CheckBoundaries();
            }
            catch (Exception ex)
            {
                Log.Error($"Boundary check error — {ex.Message}");
            }
        }

        /// <summary>
        /// Check boundaries for current drone position.
        /// </summary>
        public void CheckBoundaries()
        {
            // Get current position from Mission Planner
            var lat = MainV2.comPort?.MAV?.cs?.lat ?? 0;
            var lon = MainV2.comPort?.MAV?.cs?.lng ?? 0;
            var altAgl = MainV2.comPort?.MAV?.cs?.alt ?? 0; // Alt above home

            // If no valid position, update status to indicate waiting for GPS
            if (lat == 0 && lon == 0)
            {
                if (CurrentStatus != "no_position")
                {
                    CurrentStatus = "no_position";
                    BoundaryStatusChanged?.Invoke(this, new BoundaryStatusEventArgs
                    {
                        Status = "no_position",
                        NearestBoundaryName = "",
                    });
                }
                return;
            }

            var position = new GpsPoint(lat, lon);
            var status = _geofence.CheckBoundaryStatus(position, altAgl);

            // Handle status change
            if (status != _lastStatus)
            {
                HandleStatusChange(status, position, altAgl);
                _lastStatus = status;
            }

            // Report the unavailable hard-boundary request once at the configured time.
            // Termination is unavailable;
            // the request reports failure and never dispatches a substitute mode.
            if (status == "hard_violation")
            {
                if (_hardViolationStart.HasValue)
                {
                    var elapsed = (DateTime.Now - _hardViolationStart.Value).TotalSeconds;
                    var delayElapsed = elapsed >= _geofence.Failsafe.HardBoundaryKillDelaySec;

                    var hardAction = (_geofence.Failsafe.HardBoundaryAction ?? "warn_and_kill").ToLower();
                    if (!_terminationReported && hardAction != "warn_only"
                        && (hardAction == "auto_kill" || delayElapsed))
                    {
                        _terminationReported = true;
                        RequestTermination();
                    }
                }
            }
            else
            {
                _hardViolationStart = null;
                _terminationReported = false;
            }
            if (status == "inside") _returnCommanded = false;

            CurrentStatus = status;
        }

        private void HandleStatusChange(string newStatus, GpsPoint position, double altAgl)
        {
            CurrentStatus = newStatus;

            // Fire status changed event
            BoundaryStatusChanged?.Invoke(this, new BoundaryStatusEventArgs
            {
                Status = newStatus,
                NearestBoundaryName = newStatus == "soft_violation"
                    ? _geofence.SoftBoundary.Name
                    : _geofence.HardBoundary.Name,
            });

            if (newStatus == "soft_violation")
            {
                HandleSoftViolation(position, altAgl);
            }
            else if (newStatus == "hard_violation")
            {
                HandleHardViolation(position, altAgl);
            }
            else
            {
                // Back inside boundaries
                _hardViolationStart = null;
            }
        }

        private void HandleSoftViolation(GpsPoint position, double altAgl)
        {
            var args = new BoundaryViolationEventArgs
            {
                BoundaryType = "soft",
                BoundaryName = _geofence.SoftBoundary.Name,
                DronePosition = position,
                AltitudeAgl = altAgl,
                RequiredAction = "Turn around - approaching boundary",
                Timestamp = DateTime.Now,
            };

            // Log violation
            _geofence.BoundaryViolations.Add(new BoundaryViolation
            {
                Timestamp = args.Timestamp,
                BoundaryName = args.BoundaryName,
                BoundaryType = args.BoundaryType,
                DronePosition = position,
                Action = args.RequiredAction,
            });
            _geofence.Save();

            // Fire event
            BoundaryViolation?.Invoke(this, args);

            var softAction = (_geofence.Failsafe.SoftBoundaryAction ?? "warn_both").ToLower();

            // Audio warning ("warn_visual" stays silent; everything else beeps)
            if (_geofence.Failsafe.EnableAudioWarnings && softAction != "warn_visual")
            {
                PlayWarningSound(false);
                AudioAlerts.Speak("Approaching boundary. Turn around.", component: "boundary");
            }

            // "return_to_boundary": command GUIDED to the closest point just
            // inside the violated boundary (not all the way to the return
            // point) at the current altitude. Once per violation episode so
            // repeated checks don't spam goto commands.
            if (softAction == "return_to_boundary" && !_returnCommanded)
            {
                _returnCommanded = true;
                var boundary = _geofence.SoftBoundary?.Vertices?.Count >= 3
                    ? _geofence.SoftBoundary : _geofence.HardBoundary;
                var target = GeoMath.NearestPointInside(boundary?.Vertices, position, ReturnInsideMarginMeters);
                if (target == null)
                {
                    Log.Warn("return_to_boundary: no boundary polygon available");
                    return;
                }

                double alt = Math.Max(5.0, altAgl); // never command a goto into the ground
                bool ok = FlightModeController.GuidedGoto(target.Lat, target.Lon, alt);
                Log.Warn($"Soft boundary action — GUIDED to nearest inside point {target.Lat:F6}, {target.Lon:F6} @ {alt:F0}m: {(ok ? "dispatched" : "FAILED")}");
                if (ok) AudioAlerts.Speak("Returning inside boundary.", component: "boundary");
            }
        }

        /// <summary>
        /// How far past the boundary edge the return goto aims, so the vehicle
        /// settles inside rather than hovering on the line.
        /// (Geometry lives in GeoMath.NearestPointInside.)
        /// </summary>
        private const double ReturnInsideMarginMeters = 1.0;

        private void RequestTermination()
        {
            Log.Warn("Hard boundary violation requires aircraft termination.");
            if (!FlightModeController.RequestTermination())
            {
                AudioAlerts.Speak("Termination unavailable. Take manual control.", component: "boundary");
            }
        }

        private void HandleHardViolation(GpsPoint position, double altAgl)
        {
            if (!_hardViolationStart.HasValue)
            {
                _hardViolationStart = DateTime.Now;
            }

            var args = new BoundaryViolationEventArgs
            {
                BoundaryType = "hard",
                BoundaryName = _geofence.HardBoundary.Name,
                DronePosition = position,
                AltitudeAgl = altAgl,
                RequiredAction = "TERMINATION REQUIRED; plugin activation unavailable!",
                Timestamp = DateTime.Now,
            };

            // Log violation
            _geofence.BoundaryViolations.Add(new BoundaryViolation
            {
                Timestamp = args.Timestamp,
                BoundaryName = args.BoundaryName,
                BoundaryType = args.BoundaryType,
                DronePosition = position,
                Action = args.RequiredAction,
            });
            _geofence.Save();

            // Fire event
            BoundaryViolation?.Invoke(this, args);

            // Urgent audio warning
            if (_geofence.Failsafe.EnableAudioWarnings)
            {
                PlayWarningSound(true);
                AudioAlerts.Speak("Hard boundary violation. Termination required. Plugin termination unavailable.",
                    component: "boundary");
            }
        }

        private void PlayWarningSound(bool urgent)
        {
            AudioAlerts.Play(urgent ? AlertKind.BoundaryHard : AlertKind.BoundarySoft);
        }

        public void Dispose()
        {
            if (_isDisposed) return;
            _isDisposed = true;
            StopMonitoring();
        }
    }

    // BoundaryConfigPanel has been consolidated into NOMADBoundaryView in NOMADViews.cs.
    // Import functionality (KML, Google Maps, MP Fence) and violation action
    // configuration are now integrated directly into the boundary view.
}
