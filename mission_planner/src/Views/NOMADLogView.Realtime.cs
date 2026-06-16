// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADLogView
    {
        private static readonly string[] LiveGroups =
        {
            "Vibration",
            "Attitude",
            "Flight",
            "Speed",
            "Power",
            "GPS / EKF",
            "RC In / Out",
        };

        private readonly Dictionary<string, Label> _liveMetricValues =
            new Dictionary<string, Label>(StringComparer.OrdinalIgnoreCase);
        private TimeSeriesPlot _livePlot;
        private ComboBox _liveGroup;
        private Label _liveStatus;
        private CheckBox _recordSession;
        private bool _updatingRecordToggle;

        private void InitializeRealtimeUI()
        {
            _liveLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                Padding = new Padding(10),
                BackColor = NOMADTheme.BG_DARK,
            };
            _liveLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            _liveLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 100));
            _liveLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            _liveToolbar = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = true,
                BackColor = NOMADTheme.BG_DARK,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };
            _liveToolbar.Controls.Add(new Label
            {
                Text = "Panel:",
                AutoSize = true,
                ForeColor = TEXT_SECONDARY,
                Font = NOMADTheme.Font(),
                Margin = new Padding(4, 13, 4, 0),
            });
            _liveGroup = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 145,
                Margin = new Padding(0, 8, 12, 0),
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                Font = NOMADTheme.Font(),
            };
            _liveGroup.Items.AddRange(LiveGroups);
            _liveGroup.SelectedIndex = 0;
            _liveGroup.SelectedIndexChanged += (s, e) => ShowLiveGroup();
            _recordSession = new CheckBox
            {
                Text = "Record session to CSV",
                AutoSize = true,
                ForeColor = TEXT_SECONDARY,
                Font = NOMADTheme.Font(),
                Margin = new Padding(0, 12, 12, 0),
            };
            _recordSession.CheckedChanged += RecordSession_CheckedChanged;
            _liveStatus = new Label
            {
                Text = "Waiting for Mission Planner connection.",
                AutoSize = true,
                ForeColor = WARNING_COLOR,
                Font = NOMADTheme.Font(),
                Margin = new Padding(8, 13, 0, 0),
            };
            _liveToolbar.Controls.Add(_liveGroup);
            _liveToolbar.Controls.Add(_recordSession);
            _liveToolbar.Controls.Add(_liveStatus);

            _liveMetrics = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                WrapContents = true,
                AutoScroll = true,
                BackColor = NOMADTheme.BG_DARK,
            };
            foreach (string name in new[] { "Vibration", "Tracking", "Power", "Endurance", "GPS", "EKF" })
                _liveMetrics.Controls.Add(CreateLiveMetricCard(name));

            _livePlot = new TimeSeriesPlot
            {
                Dock = DockStyle.Fill,
                MaxPointsPerSeries = Math.Max(60, _config.LogLiveBufferPoints),
            };
            _liveLayout.Controls.Add(_liveToolbar, 0, 0);
            _liveLayout.Controls.Add(_liveMetrics, 0, 1);
            _liveLayout.Controls.Add(_livePlot, 0, 2);
            _liveTab.Controls.Add(_liveLayout);
            _liveStartedUtc = DateTime.UtcNow;
            ShowLiveGroup();
        }

        private Control CreateLiveMetricCard(string name)
        {
            var card = new TableLayoutPanel
            {
                Width = 185,
                Height = 82,
                ColumnCount = 1,
                RowCount = 2,
                BackColor = NOMADTheme.CARD_BG,
                Margin = new Padding(4),
                Padding = new Padding(12, 8, 12, 8),
            };
            card.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            card.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            card.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            card.Controls.Add(new Label
            {
                Text = name.ToUpperInvariant(),
                AutoSize = true,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Margin = new Padding(0),
            }, 0, 0);
            var value = new Label
            {
                Text = "--",
                Dock = DockStyle.Fill,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_TITLE, FontStyle.Bold),
                ForeColor = INFO_COLOR,
                TextAlign = ContentAlignment.MiddleLeft,
                AutoEllipsis = true,
                Margin = new Padding(0),
            };
            card.Controls.Add(value, 0, 1);
            AddNomadBorder(card);
            _liveMetricCards.Add(card);
            _liveMetricValues[name] = value;
            return card;
        }

        private void UpdateRealtimeData()
        {
            if (MainV2.comPort?.BaseStream?.IsOpen != true || MainV2.comPort.MAV?.cs == null)
            {
                _liveStatus.Text = "Disconnected - connect Mission Planner to stream live tuning data.";
                _liveStatus.ForeColor = WARNING_COLOR;
                return;
            }

            object cs = MainV2.comPort.MAV.cs;
            double time = (DateTime.UtcNow - _liveStartedUtc).TotalSeconds;
            var values = ReadLiveValues(cs);
            foreach (var item in values)
                AppendLive(item.Key, item.Value.Label, item.Value.Unit, time, item.Value.Value);
            _livePlot.Invalidate();

            double vibration = RecentPeak(
                time,
                5,
                "vibe.x",
                "vibe.y",
                "vibe.z");
            double rollError = Math.Abs(values["att.roll"].Value - values["att.roll_target"].Value);
            double pitchError = Math.Abs(values["att.pitch"].Value - values["att.pitch_target"].Value);
            double trackingError = Math.Max(rollError, pitchError);
            double ekf = new[] { "ekf.hpos", "ekf.vpos", "ekf.vel", "ekf.compass" }
                .Max(key => values[key].Value);
            double capacity = BatteryHealth.GetBattParam(MainV2.comPort.MAV, 1, "CAPACITY");
            double current = values["power.current"].Value;
            double used = values["power.usedmah"].Value;
            double endurance = current > 0.1 && capacity > used ? (capacity - used) / (current * 1000) * 60 : 0;

            SetLiveMetric("Vibration", $"{vibration:F1} m/s2 (5s peak)",
                vibration >= _config.LogVibrationCritical ? LogVerdict.Critical
                : vibration >= _config.LogVibrationWarning ? LogVerdict.Warning : LogVerdict.Good);
            SetLiveMetric("Tracking", $"{trackingError:F1} deg",
                trackingError >= _config.LogTuneRmsCritical ? LogVerdict.Critical
                : trackingError >= _config.LogTuneRmsWarning ? LogVerdict.Warning : LogVerdict.Good);
            SetLiveMetric("Power", $"{values["power.watts"].Value:F0} W", LogVerdict.Info);
            SetLiveMetric("Endurance", endurance > 0 ? $"~{endurance:F1} min" : "--", LogVerdict.Info);
            SetLiveMetric("GPS", $"{values["gps.sats"].Value:F0} sats / {values["gps.hdop"].Value:F1} HDOP",
                values["gps.hdop"].Value >= _config.LogHdopWarning ? LogVerdict.Warning : LogVerdict.Good);
            SetLiveMetric("EKF", $"{ekf:F2} peak",
                ekf >= _config.LogEkfVarianceCritical ? LogVerdict.Critical
                : ekf >= _config.LogEkfVarianceWarning ? LogVerdict.Warning : LogVerdict.Good);

            var snapshot = new IssueSnapshot
            {
                TimeSeconds = time,
                VibrationX = vibration,
                VibrationY = 0,
                VibrationZ = 0,
                ClipCount = values["vibe.clips"].Value,
                Hdop = values["gps.hdop"].Value,
                Satellites = (int)values["gps.sats"].Value,
                AttitudeError = trackingError,
                EkfVariance = ekf,
            };
            PublishLiveIssues(IssueRules.Evaluate(snapshot, CreateThresholds()));
            WriteRecordingRow(time, values);
            _liveStatus.Text = $"Live - {DateTime.Now:HH:mm:ss}";
            _liveStatus.ForeColor = SUCCESS_COLOR;
        }

        private Dictionary<string, (string Label, string Unit, double Value)> ReadLiveValues(object cs)
        {
            var values = new Dictionary<string, (string, string, double)>(StringComparer.OrdinalIgnoreCase)
            {
                ["vibe.x"] = ("Vibe X", "m/s2", Read(cs, "vibex")),
                ["vibe.y"] = ("Vibe Y", "m/s2", Read(cs, "vibey")),
                ["vibe.z"] = ("Vibe Z", "m/s2", Read(cs, "vibez")),
                ["vibe.clips"] = ("IMU clips", "",
                    Read(cs, "vibeclip0") + Read(cs, "vibeclip1") + Read(cs, "vibeclip2")),
                ["att.roll"] = ("Roll", "deg", Read(cs, "roll")),
                ["att.roll_target"] = ("Target roll", "deg", Read(cs, "nav_roll")),
                ["att.pitch"] = ("Pitch", "deg", Read(cs, "pitch")),
                ["att.pitch_target"] = ("Target pitch", "deg", Read(cs, "nav_pitch")),
                ["flight.climb"] = ("Climb rate", "m/s", Read(cs, "climbrate")),
                ["flight.vertical"] = ("Vertical speed", "m/s", Read(cs, "verticalspeed")),
                ["flight.throttle"] = ("Throttle out", "%", NormalizeRc(Read(cs, "ch3out"))),
                ["speed.ground"] = ("Groundspeed", "m/s", Read(cs, "groundspeed")),
                ["speed.air"] = ("Airspeed", "m/s", Read(cs, "airspeed")),
                ["power.voltage"] = ("Voltage", "V", Read(cs, "battery_voltage")),
                ["power.current"] = ("Current", "A", Read(cs, "current")),
                ["power.watts"] = ("Power", "W", Read(cs, "watts")),
                ["power.usedmah"] = ("Used", "mAh", Read(cs, "battery_usedmah")),
                ["gps.sats"] = ("Satellites", "", Read(cs, "satcount")),
                ["gps.hdop"] = ("HDOP", "", NormalizeHdop(Read(cs, "hdop"))),
                ["ekf.hpos"] = ("EKF H pos", "", Read(cs, "ekfposhor")),
                ["ekf.vpos"] = ("EKF V pos", "", Read(cs, "ekfposvert")),
                ["ekf.vel"] = ("EKF velocity", "", Read(cs, "ekfvelv")),
                ["ekf.compass"] = ("EKF compass", "", Read(cs, "ekfcompv")),
            };
            for (int channel = 1; channel <= 8; channel++)
            {
                values[$"rc.ch{channel}in"] = ($"CH{channel} in", "us", Read(cs, $"ch{channel}in"));
                values[$"rc.ch{channel}out"] = ($"CH{channel} out", "us", Read(cs, $"ch{channel}out"));
            }
            return values;
        }

        private void AppendLive(string key, string label, string unit, double time, double value)
        {
            if (!_liveBuffers.TryGetValue(key, out TimeSeriesData data))
            {
                data = new TimeSeriesData
                {
                    Key = key,
                    Label = label,
                    Unit = unit,
                    Color = LiveColor(key),
                };
                _liveBuffers[key] = data;
                if (KeyInSelectedGroup(key)) _livePlot.SetSeries(data);
            }
            data.Points.Add(new TimeSeriesPoint { Time = time, Value = value });
            while (data.Points.Count > Math.Max(60, _config.LogLiveBufferPoints))
                data.Points.RemoveAt(0);
        }

        private double RecentPeak(double now, double windowSeconds, params string[] keys)
        {
            double start = now - Math.Max(0, windowSeconds);
            double peak = 0;
            foreach (string key in keys)
            {
                if (!_liveBuffers.TryGetValue(key, out TimeSeriesData data))
                    continue;
                foreach (TimeSeriesPoint point in data.Points)
                {
                    if (point.Time >= start)
                        peak = Math.Max(peak, Math.Abs(point.Value));
                }
            }
            return peak;
        }

        private void ShowLiveGroup()
        {
            if (_livePlot == null) return;
            _livePlot.ClearSeries();
            foreach (TimeSeriesData data in _liveBuffers.Values.Where(data => KeyInSelectedGroup(data.Key)))
                _livePlot.SetSeries(data);
            if (string.Equals(_liveGroup?.SelectedItem?.ToString(), "Vibration", StringComparison.Ordinal))
            {
                _livePlot.SetThresholds(
                    (_config.LogVibrationWarning, WARNING_COLOR, "warning"),
                    (_config.LogVibrationCritical, ERROR_COLOR, "critical"));
            }
        }

        private bool KeyInSelectedGroup(string key)
        {
            string selected = _liveGroup?.SelectedItem?.ToString() ?? "Vibration";
            switch (selected)
            {
                case "Attitude": return key.StartsWith("att.", StringComparison.OrdinalIgnoreCase);
                case "Flight": return key.StartsWith("flight.", StringComparison.OrdinalIgnoreCase);
                case "Speed": return key.StartsWith("speed.", StringComparison.OrdinalIgnoreCase);
                case "Power": return key.StartsWith("power.", StringComparison.OrdinalIgnoreCase);
                case "GPS / EKF":
                    return key.StartsWith("gps.", StringComparison.OrdinalIgnoreCase)
                        || key.StartsWith("ekf.", StringComparison.OrdinalIgnoreCase);
                case "RC In / Out": return key.StartsWith("rc.", StringComparison.OrdinalIgnoreCase);
                default: return key.StartsWith("vibe.", StringComparison.OrdinalIgnoreCase) && key != "vibe.clips";
            }
        }

        private void PublishLiveIssues(IReadOnlyCollection<LogAnomaly> issues)
        {
            var current = new HashSet<string>(issues.Select(issue => issue.Id), StringComparer.OrdinalIgnoreCase);
            foreach (LogAnomaly issue in issues.Where(issue => !_activeLiveIssues.Contains(issue.Id)))
            {
                NotificationService.Shared?.AddNotification(
                    issue.Verdict == LogVerdict.Critical ? NotificationSeverity.Critical : NotificationSeverity.Warning,
                    NotificationCategory.System,
                    "Live tuning: " + issue.Title,
                    issue.Detail);
                if (_config.LogInjectAlertsToHud)
                {
                    _telemetryInjector.InjectStatusText(
                        issue.Title + ": " + issue.Detail,
                        issue.Verdict == LogVerdict.Critical
                            ? MAVLink.MAV_SEVERITY.CRITICAL
                            : MAVLink.MAV_SEVERITY.WARNING);
                }
            }
            _activeLiveIssues.Clear();
            foreach (string id in current) _activeLiveIssues.Add(id);
        }

        private void SetLiveMetric(string name, string value, LogVerdict verdict)
        {
            if (!_liveMetricValues.TryGetValue(name, out Label label)) return;
            label.Text = value;
            label.ForeColor = VerdictColor(verdict);
        }

        private void RecordSession_CheckedChanged(object sender, EventArgs e)
        {
            if (_updatingRecordToggle) return;
            if (!_recordSession.Checked)
            {
                StopRecording();
                return;
            }

            using (var dialog = new SaveFileDialog
            {
                Title = "Record live tuning session",
                Filter = "CSV file (*.csv)|*.csv",
                FileName = $"nomad-live-{DateTime.Now:yyyyMMdd-HHmmss}.csv",
                InitialDirectory = ExistingDirectory(_config.DefaultLogDirectory),
            })
            {
                if (dialog.ShowDialog(FindForm()) != DialogResult.OK)
                {
                    SetRecordToggle(false);
                    return;
                }
                try
                {
                    _recordWriter = new StreamWriter(dialog.FileName, false, System.Text.Encoding.UTF8);
                    _recordWriter.WriteLine("time_s,key,value,unit");
                    _recordWriter.Flush();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(FindForm(), ex.Message, "Live Recording",
                        MessageBoxButtons.OK, MessageBoxIcon.Error);
                    SetRecordToggle(false);
                }
            }
        }

        private void WriteRecordingRow(
            double time,
            IReadOnlyDictionary<string, (string Label, string Unit, double Value)> values)
        {
            if (_recordWriter == null) return;
            foreach (var item in values)
            {
                _recordWriter.WriteLine(string.Join(",",
                    time.ToString("F3", CultureInfo.InvariantCulture),
                    Csv(item.Key),
                    item.Value.Value.ToString("G17", CultureInfo.InvariantCulture),
                    Csv(item.Value.Unit)));
            }
            _recordWriter.Flush();
        }

        private void StopRecording()
        {
            try { _recordWriter?.Dispose(); } catch { }
            _recordWriter = null;
            SetRecordToggle(false);
        }

        private void SetRecordToggle(bool value)
        {
            if (_recordSession == null || _recordSession.Checked == value) return;
            _updatingRecordToggle = true;
            _recordSession.Checked = value;
            _updatingRecordToggle = false;
        }

        private static string Csv(string value) => "\"" + (value ?? "").Replace("\"", "\"\"") + "\"";
        private static double Read(object cs, string name) => BatteryHealth.GetCsDouble(cs, name);
        private static double NormalizeRc(double value) => value > 1000 ? (value - 1000) / 10 : value;
        private static double NormalizeHdop(double value) => value > 100 ? value / 100 : value;

        private static Color LiveColor(string key)
        {
            int hash = key.Aggregate(17, (value, character) => unchecked(value * 31 + character));
            Color[] colors =
            {
                NOMADTheme.ACCENT,
                Color.FromArgb(255, 82, 92),
                Color.FromArgb(178, 36, 44),
                Color.FromArgb(255, 150, 156),
                NOMADTheme.TEXT_PRIMARY,
                NOMADTheme.TEXT_SECONDARY,
            };
            return colors[(hash & int.MaxValue) % colors.Length];
        }
    }
}
