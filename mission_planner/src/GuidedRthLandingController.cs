using System;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    internal sealed class GuidedRthLandingController : IDisposable
    {
        private const double ClimbToleranceM = 1.0;
        private const double HomeRadiusM = 2.0;
        private const int DescentPeriodMs = 500;
        private const ushort PositionOnlyTypeMask = 0x0FF8;
        private const byte MavFrameGlobalRelativeAltInt = 6;
        private const double RangefinderValidMinM = 0.03;
        private const double RangefinderFinalMaxM = 0.80;
        private const double RangefinderTouchdownM = 0.18;
        private const double MinFinalTargetAglM = -1.00;
        private const double NoRangefinderFinalStartM = 0.50;
        private const double NoRangefinderStableEpsilonM = 0.04;
        private const double NoRangefinderStableSeconds = 3.0;

        private readonly object _lock = new object();
        private CancellationTokenSource _descentCts;
        private DateTime? _noRangefinderStableSinceUtc;
        private double? _noRangefinderStableReferenceAltM;

        public sealed class Status
        {
            public bool Active { get; set; }
            public string Phase { get; set; } = "idle";
            public string NextPhase { get; set; } = "climb";
            public bool AwaitingApproval { get; set; }
            public bool Paused { get; set; }
            public string Message { get; set; } = "RTH landing idle";
            public string ErrorMessage { get; set; }
            public double ClimbAltM { get; set; } = 30.0;
            public double DescentRateMps { get; set; } = 0.5;
            public double DescentTargetAglM { get; set; } = 0.3;
            public double? HomeLat { get; set; }
            public double? HomeLon { get; set; }
            public double? TargetLat { get; set; }
            public double? TargetLon { get; set; }
            public double? TargetAltAglM { get; set; }
            public double? DistanceToHomeM { get; set; }
            public double? AltitudeErrorM { get; set; }
            public double? CurrentAltAglM { get; set; }
            public double? RangefinderM { get; set; }
            public string LandingReference { get; set; } = "baro";
            public bool NoRangefinderFallbackActive { get; set; }
            public double? NoRangefinderStableSeconds { get; set; }
        }

        private Status _status = new Status();

        public Status CurrentStatus
        {
            get
            {
                lock (_lock)
                {
                    RefreshProgressLocked();
                    return Clone(_status);
                }
            }
        }

        public Status Start(double climbAltM, double descentRateMps, double descentTargetAglM)
        {
            if (!IsMavlinkOpen()) return FailStatus("MAVLink is not connected");
            if (!TryGetCurrent(out _, out _, out double currentAlt))
                return FailStatus("Current vehicle position/altitude is unavailable");
            if (!TryGetHome(out double homeLat, out double homeLon))
                return FailStatus("Mission Planner home position is unavailable");

            climbAltM = Clamp(climbAltM, 1.0, 150.0);
            descentRateMps = Clamp(descentRateMps, 0.1, 1.0);
            descentTargetAglM = Clamp(descentTargetAglM, 0.1, 5.0);
            if (climbAltM < descentTargetAglM + 2.0)
                return FailStatus($"RTH altitude {climbAltM:F1}m above home is too low");

            StopDescentLoop();
            ResetNoRangefinderFallback();
            lock (_lock)
            {
                _status = new Status
                {
                    Active = true,
                    Phase = "idle",
                    NextPhase = "climb",
                    AwaitingApproval = true,
                    Paused = false,
                    Message = "Awaiting approval: climb/hold to RTH altitude",
                    ClimbAltM = climbAltM,
                    DescentRateMps = descentRateMps,
                    DescentTargetAglM = descentTargetAglM,
                    HomeLat = homeLat,
                    HomeLon = homeLon,
                    CurrentAltAglM = currentAlt,
                };
                RefreshProgressLocked();
                return Clone(_status);
            }
        }

        public Status ApproveNextPhase()
        {
            string next;
            lock (_lock)
            {
                RefreshProgressLocked();
                if (!_status.Active) return FailStatus("No active RTH landing plan");
                if (_status.Paused) return FailStatus("RTH landing is paused");
                if (!_status.AwaitingApproval) return FailStatus($"Phase {_status.Phase} is already active");
                next = _status.NextPhase;
            }

            if (next == "climb") return CommandClimb(false);
            if (next == "go_home") return CommandGoHome(false);
            if (next == "descend") return CommandDescend(false);
            return FailStatus($"Unknown RTH phase: {next}");
        }

        public Status Pause()
        {
            lock (_lock)
            {
                if (!_status.Active) return FailStatus("No active RTH landing plan");
                _status.Paused = true;
                _status.AwaitingApproval = false;
                _status.Message = "Paused: holding current guided position";
            }
            HoldCurrentPosition();
            return CurrentStatus;
        }

        public Status Resume()
        {
            string phase;
            lock (_lock)
            {
                if (!_status.Active) return FailStatus("No active RTH landing plan");
                if (!_status.Paused) return Clone(_status);
                _status.Paused = false;
                phase = _status.Phase;
            }

            if (phase == "climb") return CommandClimb(true);
            if (phase == "go_home") return CommandGoHome(true);
            if (phase == "descend") return CommandDescend(true);
            lock (_lock)
            {
                _status.AwaitingApproval = true;
                _status.Message = $"Awaiting approval: {_status.NextPhase}";
                return Clone(_status);
            }
        }

        public Status Edit(double climbAltM, double descentRateMps, double descentTargetAglM)
        {
            string phase;
            bool paused;
            lock (_lock)
            {
                if (!_status.Active) return FailStatus("No active RTH landing plan");
                _status.ClimbAltM = Clamp(climbAltM, 1.0, 150.0);
                _status.DescentRateMps = Clamp(descentRateMps, 0.1, 1.0);
                _status.DescentTargetAglM = Clamp(descentTargetAglM, 0.1, 5.0);
                _status.Message = "RTH landing plan edited";
                phase = _status.Phase;
                paused = _status.Paused;
            }

            if (!paused && phase == "climb") return CommandClimb(true);
            if (!paused && phase == "go_home") return CommandGoHome(true);
            return CurrentStatus;
        }

        public Status Abort()
        {
            StopDescentLoop();
            HoldCurrentPosition();
            lock (_lock)
            {
                _status.Active = false;
                _status.Phase = "aborted";
                _status.NextPhase = "";
                _status.AwaitingApproval = false;
                _status.Paused = false;
                _status.Message = "RTH landing aborted; holding current position";
                return Clone(_status);
            }
        }

        private Status CommandClimb(bool resume)
        {
            if (!EnsureGuided()) return FailStatus("Could not switch to GUIDED");
            if (!TryGetCurrent(out double lat, out double lon, out _))
                return FailStatus("Current vehicle position is unavailable");

            double targetAlt;
            lock (_lock) targetAlt = _status.ClimbAltM;
            if (!SendGlobalRelativeTarget(lat, lon, targetAlt))
                return FailStatus("Failed to send guided climb target");

            lock (_lock)
            {
                _status.Phase = "climb";
                _status.NextPhase = "go_home";
                _status.AwaitingApproval = false;
                _status.Paused = false;
                _status.ErrorMessage = null;
                _status.Message = resume ? "Resumed climb/hold to RTH altitude" : "Commanded climb/hold to RTH altitude";
                _status.TargetLat = lat;
                _status.TargetLon = lon;
                _status.TargetAltAglM = targetAlt;
                RefreshProgressLocked();
                return Clone(_status);
            }
        }

        private Status CommandGoHome(bool resume)
        {
            if (!EnsureGuided()) return FailStatus("Could not switch to GUIDED");
            double homeLat, homeLon, targetAlt;
            lock (_lock)
            {
                if (!_status.HomeLat.HasValue || !_status.HomeLon.HasValue)
                    return FailStatus("Home position is unavailable");
                homeLat = _status.HomeLat.Value;
                homeLon = _status.HomeLon.Value;
                targetAlt = _status.ClimbAltM;
            }

            if (!SendGlobalRelativeTarget(homeLat, homeLon, targetAlt))
                return FailStatus("Failed to send guided home target");

            lock (_lock)
            {
                _status.Phase = "go_home";
                _status.NextPhase = "descend";
                _status.AwaitingApproval = false;
                _status.Paused = false;
                _status.ErrorMessage = null;
                _status.Message = resume ? "Resumed return to home point" : "Commanded return to home point";
                _status.TargetLat = homeLat;
                _status.TargetLon = homeLon;
                _status.TargetAltAglM = targetAlt;
                RefreshProgressLocked();
                return Clone(_status);
            }
        }

        private Status CommandDescend(bool resume)
        {
            if (!EnsureGuided()) return FailStatus("Could not switch to GUIDED");
            lock (_lock)
            {
                if (!_status.HomeLat.HasValue || !_status.HomeLon.HasValue)
                    return FailStatus("Home position is unavailable");
                _status.Phase = "descend";
                _status.NextPhase = "";
                _status.AwaitingApproval = false;
                _status.Paused = false;
                _status.ErrorMessage = null;
                _status.Message = resume ? "Resumed controlled descent over home" : "Commanded controlled descent over home";
                _status.TargetLat = _status.HomeLat;
                _status.TargetLon = _status.HomeLon;
                _status.TargetAltAglM = _status.DescentTargetAglM;
            }
            StartDescentLoop();
            return CurrentStatus;
        }

        private void StartDescentLoop()
        {
            StopDescentLoop();
            ResetNoRangefinderFallback();
            _descentCts = new CancellationTokenSource();
            var token = _descentCts.Token;
            Task.Run(async () =>
            {
                DateTime last = DateTime.UtcNow;
                while (!token.IsCancellationRequested)
                {
                    bool paused;
                    double homeLat, homeLon, rate, targetAgl;
                    lock (_lock)
                    {
                        if (!_status.Active || _status.Phase != "descend") return;
                        paused = _status.Paused;
                        if (!_status.HomeLat.HasValue || !_status.HomeLon.HasValue) return;
                        homeLat = _status.HomeLat.Value;
                        homeLon = _status.HomeLon.Value;
                        rate = _status.DescentRateMps;
                        targetAgl = _status.DescentTargetAglM;
                    }

                    if (paused)
                    {
                        await Task.Delay(DescentPeriodMs, token).ConfigureAwait(false);
                        continue;
                    }

                    if (!TryGetCurrent(out _, out _, out double currentAlt))
                    {
                        MarkFailed("Current altitude unavailable during guided descent");
                        return;
                    }

                    bool rangefinderValid = TryGetRangefinder(out double rangefinderM);
                    bool rangefinderFinal = rangefinderValid && rangefinderM <= RangefinderFinalMaxM;
                    if (rangefinderFinal && rangefinderM <= RangefinderTouchdownM)
                    {
                        SendGlobalRelativeTarget(homeLat, homeLon, targetAgl);
                        lock (_lock)
                        {
                            _status.Active = false;
                            _status.Phase = "complete";
                            _status.NextPhase = "";
                            _status.AwaitingApproval = false;
                            _status.Message = "Rangefinder touchdown threshold reached; disarm manually when stable";
                            _status.LandingReference = "rangefinder";
                            RefreshProgressLocked();
                        }
                        return;
                    }

                    var now = DateTime.UtcNow;
                    var dt = Math.Max(DescentPeriodMs / 1000.0, (now - last).TotalSeconds);
                    last = now;
                    double nextAlt;
                    if (rangefinderFinal)
                    {
                        // If baro/home altitude has drifted, relative-altitude guided targets
                        // can clamp at 0 m while the aircraft is still above the surface.
                        // Once RNGFND1 is in its accurate near-ground band, keep stepping the
                        // commanded target below home until the rangefinder touchdown threshold
                        // is reached.
                        double step = Math.Max(rate * dt, 0.10);
                        nextAlt = Math.Max(MinFinalTargetAglM, currentAlt - step);
                    }
                    else if (currentAlt <= Math.Max(targetAgl + 0.2, NoRangefinderFinalStartM))
                    {
                        if (UpdateNoRangefinderTouchdownFallback(currentAlt, now))
                        {
                            SendGlobalRelativeTarget(homeLat, homeLon, Math.Min(currentAlt, targetAgl));
                            lock (_lock)
                            {
                                _status.Active = false;
                                _status.Phase = "complete";
                                _status.NextPhase = "";
                                _status.AwaitingApproval = false;
                                _status.Message = "No-rangefinder touchdown fallback: altitude stopped decreasing; disarm manually when stable";
                                _status.LandingReference = "no_rangefinder_stable";
                                RefreshProgressLocked();
                            }
                            return;
                        }

                        double step = Math.Max(rate * dt, 0.10);
                        nextAlt = Math.Max(MinFinalTargetAglM, currentAlt - step);
                    }
                    else
                    {
                        ResetNoRangefinderFallback();
                        nextAlt = Math.Max(targetAgl, currentAlt - rate * dt);
                    }
                    if (!SendGlobalRelativeTarget(homeLat, homeLon, nextAlt))
                    {
                        MarkFailed("Failed to send guided descent target");
                        return;
                    }

                    lock (_lock)
                    {
                        _status.TargetAltAglM = nextAlt;
                        _status.RangefinderM = rangefinderValid ? rangefinderM : (double?)null;
                        _status.LandingReference = rangefinderFinal
                            ? "rangefinder"
                            : _noRangefinderStableSinceUtc.HasValue ? "no_rangefinder_stable" : "baro";
                        _status.Message = rangefinderFinal
                            ? $"Rangefinder final descent: {rangefinderM:F2}m AGL"
                            : _noRangefinderStableSinceUtc.HasValue
                                ? "No rangefinder: commanding below home until altitude stops decreasing"
                            : "Controlled descent over home";
                        RefreshProgressLocked();
                    }
                    await Task.Delay(DescentPeriodMs, token).ConfigureAwait(false);
                }
            }, token);
        }

        private void StopDescentLoop()
        {
            try { _descentCts?.Cancel(); }
            catch { }
            _descentCts?.Dispose();
            _descentCts = null;
        }

        private bool UpdateNoRangefinderTouchdownFallback(double currentAltM, DateTime nowUtc)
        {
            if (!_noRangefinderStableReferenceAltM.HasValue
                || currentAltM < _noRangefinderStableReferenceAltM.Value - NoRangefinderStableEpsilonM
                || currentAltM > _noRangefinderStableReferenceAltM.Value + NoRangefinderStableEpsilonM)
            {
                _noRangefinderStableReferenceAltM = currentAltM;
                _noRangefinderStableSinceUtc = nowUtc;
                return false;
            }

            if (!_noRangefinderStableSinceUtc.HasValue)
                _noRangefinderStableSinceUtc = nowUtc;

            return (nowUtc - _noRangefinderStableSinceUtc.Value).TotalSeconds >= NoRangefinderStableSeconds;
        }

        private void ResetNoRangefinderFallback()
        {
            _noRangefinderStableSinceUtc = null;
            _noRangefinderStableReferenceAltM = null;
        }

        private bool HoldCurrentPosition()
        {
            return TryGetCurrent(out double lat, out double lon, out double alt) &&
                SendGlobalRelativeTarget(lat, lon, alt);
        }

        private static bool EnsureGuided()
        {
            return FlightModeController.SetGuidedMode();
        }

        private static bool IsMavlinkOpen()
        {
            return MainV2.comPort != null && MainV2.comPort.BaseStream != null && MainV2.comPort.BaseStream.IsOpen;
        }

        private static bool TryGetCurrent(out double lat, out double lon, out double altAgl)
        {
            lat = lon = altAgl = 0;
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return false;
                lat = Convert.ToDouble(cs.lat);
                lon = Convert.ToDouble(cs.lng);
                altAgl = Convert.ToDouble(cs.alt);
                return Math.Abs(lat) > 0.000001 || Math.Abs(lon) > 0.000001;
            }
            catch { return false; }
        }

        private static bool TryGetRangefinder(out double rangeM)
        {
            rangeM = 0;
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return false;
                rangeM = Convert.ToDouble(cs.sonarrange);
                return rangeM >= RangefinderValidMinM && rangeM <= RangefinderFinalMaxM;
            }
            catch { return false; }
        }

        private static bool TryGetHome(out double lat, out double lon)
        {
            lat = lon = 0;
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                object home = GetMember(cs, "HomeLocation") ?? GetMember(cs, "home") ?? GetMember(MainV2.comPort?.MAV, "HomeLocation");
                if (home == null) return false;
                if (!TryReadDouble(home, "Lat", out lat) && !TryReadDouble(home, "lat", out lat)) return false;
                if (!TryReadDouble(home, "Lng", out lon) && !TryReadDouble(home, "lng", out lon) && !TryReadDouble(home, "Lon", out lon)) return false;
                return Math.Abs(lat) > 0.000001 || Math.Abs(lon) > 0.000001;
            }
            catch { return false; }
        }

        private static object GetMember(object obj, string name)
        {
            if (obj == null) return null;
            var t = obj.GetType();
            var p = t.GetProperty(name, BindingFlags.Public | BindingFlags.Instance | BindingFlags.IgnoreCase);
            if (p != null) return p.GetValue(obj, null);
            var f = t.GetField(name, BindingFlags.Public | BindingFlags.Instance | BindingFlags.IgnoreCase);
            return f?.GetValue(obj);
        }

        private static bool TryReadDouble(object obj, string name, out double value)
        {
            value = 0;
            try
            {
                var raw = GetMember(obj, name);
                if (raw == null) return false;
                value = Convert.ToDouble(raw);
                return true;
            }
            catch { return false; }
        }

        private static bool SendGlobalRelativeTarget(double lat, double lon, double altAgl)
        {
            if (!IsMavlinkOpen()) return false;
            bool acquired = false;
            try
            {
                acquired = CubeOutputController.MavlinkLock.Wait(1500);
                if (!acquired) return false;

                var msg = new MAVLink.mavlink_set_position_target_global_int_t
                {
                    time_boot_ms = (uint)Environment.TickCount,
                    target_system = MainV2.comPort.MAV.sysid,
                    target_component = MainV2.comPort.MAV.compid,
                    coordinate_frame = MavFrameGlobalRelativeAltInt,
                    type_mask = PositionOnlyTypeMask,
                    lat_int = (int)Math.Round(lat * 1e7),
                    lon_int = (int)Math.Round(lon * 1e7),
                    alt = (float)altAgl,
                    vx = 0,
                    vy = 0,
                    vz = 0,
                    afx = 0,
                    afy = 0,
                    afz = 0,
                    yaw = 0,
                    yaw_rate = 0,
                };
                MainV2.comPort.sendPacket(msg, MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid);
                return true;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Guided RTH target error: {ex.Message}");
                return false;
            }
            finally
            {
                if (acquired) CubeOutputController.MavlinkLock.Release();
            }
        }

        private void RefreshProgressLocked()
        {
            if (TryGetCurrent(out double lat, out double lon, out double alt))
            {
                _status.CurrentAltAglM = alt;
                if (_status.HomeLat.HasValue && _status.HomeLon.HasValue)
                    _status.DistanceToHomeM = HaversineM(lat, lon, _status.HomeLat.Value, _status.HomeLon.Value);
            }
            _status.RangefinderM = TryGetRangefinder(out double rf) ? rf : (double?)null;
            _status.NoRangefinderFallbackActive = _status.LandingReference == "no_rangefinder_stable";
            _status.NoRangefinderStableSeconds = _noRangefinderStableSinceUtc.HasValue
                ? Math.Max(0.0, (DateTime.UtcNow - _noRangefinderStableSinceUtc.Value).TotalSeconds)
                : (double?)null;

            _status.AltitudeErrorM = _status.TargetAltAglM.HasValue && _status.CurrentAltAglM.HasValue
                ? _status.TargetAltAglM.Value - _status.CurrentAltAglM.Value
                : (double?)null;

            if (!_status.Active || _status.Paused || _status.AwaitingApproval) return;
            if (_status.Phase == "climb"
                && _status.CurrentAltAglM.HasValue
                && Math.Abs(_status.ClimbAltM - _status.CurrentAltAglM.Value) <= ClimbToleranceM)
            {
                _status.AwaitingApproval = true;
                _status.NextPhase = "go_home";
                _status.Message = "At RTH altitude. Awaiting approval: go home";
            }
            else if (_status.Phase == "go_home"
                && _status.DistanceToHomeM.HasValue
                && _status.DistanceToHomeM.Value <= HomeRadiusM)
            {
                _status.AwaitingApproval = true;
                _status.NextPhase = "descend";
                _status.Message = "At home point. Awaiting approval: controlled descent";
            }
        }

        private Status FailStatus(string message)
        {
            lock (_lock)
            {
                _status.ErrorMessage = message;
                _status.Message = message;
                return Clone(_status);
            }
        }

        private void MarkFailed(string message)
        {
            lock (_lock)
            {
                _status.Active = false;
                _status.Phase = "failed";
                _status.NextPhase = "";
                _status.AwaitingApproval = false;
                _status.ErrorMessage = message;
                _status.Message = message;
            }
        }

        private static Status Clone(Status s)
        {
            return new Status
            {
                Active = s.Active,
                Phase = s.Phase,
                NextPhase = s.NextPhase,
                AwaitingApproval = s.AwaitingApproval,
                Paused = s.Paused,
                Message = s.Message,
                ErrorMessage = s.ErrorMessage,
                ClimbAltM = s.ClimbAltM,
                DescentRateMps = s.DescentRateMps,
                DescentTargetAglM = s.DescentTargetAglM,
                HomeLat = s.HomeLat,
                HomeLon = s.HomeLon,
                TargetLat = s.TargetLat,
                TargetLon = s.TargetLon,
                TargetAltAglM = s.TargetAltAglM,
                DistanceToHomeM = s.DistanceToHomeM,
                AltitudeErrorM = s.AltitudeErrorM,
                CurrentAltAglM = s.CurrentAltAglM,
                RangefinderM = s.RangefinderM,
                LandingReference = s.LandingReference,
                NoRangefinderFallbackActive = s.NoRangefinderFallbackActive,
                NoRangefinderStableSeconds = s.NoRangefinderStableSeconds,
            };
        }

        private static double Clamp(double value, double min, double max)
        {
            if (double.IsNaN(value) || double.IsInfinity(value)) return min;
            return Math.Max(min, Math.Min(max, value));
        }

        private static double HaversineM(double lat1, double lon1, double lat2, double lon2)
        {
            const double r = 6371000.0;
            double p1 = lat1 * Math.PI / 180.0;
            double p2 = lat2 * Math.PI / 180.0;
            double dp = (lat2 - lat1) * Math.PI / 180.0;
            double dl = (lon2 - lon1) * Math.PI / 180.0;
            double a = Math.Sin(dp / 2) * Math.Sin(dp / 2)
                + Math.Cos(p1) * Math.Cos(p2) * Math.Sin(dl / 2) * Math.Sin(dl / 2);
            return r * 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
        }

        public void Dispose()
        {
            StopDescentLoop();
        }
    }
}
