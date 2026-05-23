// ============================================================
// NOMAD Flight Boundary Manager
// ============================================================
// Manages flight boundaries for AEAC 2026 competition.
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
        private readonly MissionConfig _missionConfig;
        private readonly NOMADConfig _config;
        private System.Timers.Timer _monitorTimer;
        private string _lastStatus = "inside";
        private DateTime? _hardViolationStart;
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
        /// Time remaining before auto-kill (seconds), null if not in hard violation.
        /// </summary>
        public int? KillCountdown { get; private set; }

        /// <summary>
        /// Is boundary monitoring active.
        /// </summary>
        public bool IsMonitoring { get; private set; }

        public BoundaryMonitor(MissionConfig missionConfig, NOMADConfig config)
        {
            _missionConfig = missionConfig;
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

            Console.WriteLine("NOMAD: Boundary monitoring started");
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
            KillCountdown = null;

            Console.WriteLine("NOMAD: Boundary monitoring stopped");
        }

        private void MonitorTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            try
            {
                CheckBoundaries();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Boundary check error - {ex.Message}");
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
            var status = _missionConfig.CheckBoundaryStatus(position, altAgl);

            // Handle status change
            if (status != _lastStatus)
            {
                HandleStatusChange(status, position, altAgl);
                _lastStatus = status;
            }

            // Update kill countdown for hard violations
            if (status == "hard_violation")
            {
                if (_hardViolationStart.HasValue)
                {
                    var elapsed = (DateTime.Now - _hardViolationStart.Value).TotalSeconds;
                    var remaining = _missionConfig.Failsafe.HardBoundaryKillDelaySec - (int)elapsed;
                    KillCountdown = Math.Max(0, remaining);
                }
            }
            else
            {
                _hardViolationStart = null;
                KillCountdown = null;
            }

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
                    ? _missionConfig.SoftBoundary.Name 
                    : _missionConfig.HardBoundary.Name,
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
                KillCountdown = null;
            }
        }

        private void HandleSoftViolation(GpsPoint position, double altAgl)
        {
            var args = new BoundaryViolationEventArgs
            {
                BoundaryType = "soft",
                BoundaryName = _missionConfig.SoftBoundary.Name,
                DronePosition = position,
                AltitudeAgl = altAgl,
                RequiredAction = "Turn around - approaching boundary",
                Timestamp = DateTime.Now,
            };

            // Log violation
            _missionConfig.BoundaryViolations.Add(new BoundaryViolation
            {
                Timestamp = args.Timestamp,
                BoundaryName = args.BoundaryName,
                BoundaryType = args.BoundaryType,
                DronePosition = position,
                Action = args.RequiredAction,
            });
            _missionConfig.Save();

            // Fire event
            BoundaryViolation?.Invoke(this, args);

            // Audio warning
            if (_missionConfig.Failsafe.EnableAudioWarnings)
            {
                PlayWarningSound(false);
                AudioAlerts.Speak("Approaching boundary. Turn around.", component: "boundary");
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
                BoundaryName = _missionConfig.HardBoundary.Name,
                DronePosition = position,
                AltitudeAgl = altAgl,
                RequiredAction = $"KILL REQUIRED within {_missionConfig.Failsafe.HardBoundaryKillDelaySec} seconds!",
                Timestamp = DateTime.Now,
            };

            // Log violation
            _missionConfig.BoundaryViolations.Add(new BoundaryViolation
            {
                Timestamp = args.Timestamp,
                BoundaryName = args.BoundaryName,
                BoundaryType = args.BoundaryType,
                DronePosition = position,
                Action = args.RequiredAction,
            });
            _missionConfig.Save();

            // Fire event
            BoundaryViolation?.Invoke(this, args);

            // Urgent audio warning
            if (_missionConfig.Failsafe.EnableAudioWarnings)
            {
                PlayWarningSound(true);
                AudioAlerts.Speak($"Hard boundary violation. Kill required in {_missionConfig.Failsafe.HardBoundaryKillDelaySec} seconds.",
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

