// ============================================================
// NOMAD Task 1 Checklist Panel
// ============================================================
// Persistent pre-flight and flight-line checklist for Task 1.
// State is keyed by stable item IDs and stored under Documents\NOMAD\Task1
// so progress survives Mission Planner restarts.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public class Task1ChecklistPanel : UserControl
    {
        private class ChecklistItem
        {
            public string Id { get; }
            public string Phase { get; }
            public string Text { get; }

            public ChecklistItem(string id, string phase, string text)
            {
                Id = id;
                Phase = phase;
                Text = text;
            }
        }

        private static readonly string StoragePath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "NOMAD", "Task1", "task1_checklist.json");

        private static readonly ChecklistItem[] PreFlightItems =
        {
            new ChecklistItem("pf_airframe_fasteners", "Airframe", "Inspect frame, arms, landing gear, prop guards, and payload mounts for cracks or loose fasteners."),
            new ChecklistItem("pf_props", "Airframe", "Props installed, undamaged, correct orientation, and tightened."),
            new ChecklistItem("pf_battery_main", "Power", "Main flight battery charged, balanced, strapped, connector secure, voltage recorded."),
            new ChecklistItem("pf_battery_gcs", "Power", "Ground station laptop, controller, radio, hotspot/phone, and spare batteries charged."),
            new ChecklistItem("pf_accel_cal", "Calibration", "Accelerometer calibration complete and level horizon verified in Mission Planner."),
            new ChecklistItem("pf_mag_cal", "Calibration", "Compass / magnetometer calibration complete; compass offsets reasonable; no magnetic interference nearby."),
            new ChecklistItem("pf_radio_cal", "Calibration", "RC calibration checked: sticks, flight mode switch, CH5 kill/disarm, RC12 LAND, payload switches."),
            new ChecklistItem("pf_ekf_params", "Autopilot", "Task 1 parameter profile loaded; GPS/barometer/compass EKF source active; failsafe actions confirmed."),
            new ChecklistItem("pf_gps_lock", "Autopilot", "GPS lock verified outdoors; HDOP and satellite count acceptable before arming."),
            new ChecklistItem("pf_geofence", "Autopilot", "Boundary/geofence loaded and visually checked against the competition area."),
            new ChecklistItem("pf_tailscale", "Networking", "Tailscale connected to Jetson; Edge Core /health responds from GCS."),
            new ChecklistItem("pf_video", "Networking", "Task 1 video stream visible; overlay mode set to Task 1 detector; crosshair visible."),
            new ChecklistItem("pf_mavlink", "Networking", "MAVLink primary/backup links healthy; Mission Planner HUD telemetry updates continuously."),
            new ChecklistItem("pf_edge_services", "Jetson", "Edge Core, MediaMTX, MAVLink router, ZED wrapper, video bridge, and target localizer running."),
            new ChecklistItem("pf_storage", "Jetson", "Jetson disk space sufficient; old capture folders cleared or archived."),
            new ChecklistItem("pf_building_model", "Task 1", "Building preset/corners loaded; height and wall lengths verified; ground altitude reference set if needed."),
            new ChecklistItem("pf_lap_course", "Task 1", "Lap course loaded into Mission Planner, overlaid on map, altitude/lap count checked."),
            new ChecklistItem("pf_payload_servos", "Payload", "Drop servos tested: Drop P1/P2/P3 move and retract correctly."),
            new ChecklistItem("pf_reels", "Payload", "Reel in/out tested on both reels; full reel buttons understood and cancel verified."),
            new ChecklistItem("pf_budget", "Mission", "Current budget reviewed; expected lap current and remaining task window acceptable."),
            new ChecklistItem("pf_brief", "Mission", "Pilot, payload operator, visual observer, and GCS operator briefed on abort words and roles.")
        };

        private static readonly ChecklistItem[] FlightLineItems =
        {
            new ChecklistItem("fl_area_clear", "Line Setup", "Flight area clear; props-off checks complete before final arming sequence."),
            new ChecklistItem("fl_airframe_final", "Line Setup", "Final airframe walkaround: props, battery strap, payload straps, reel lines, and camera tilt clear."),
            new ChecklistItem("fl_power_on", "Power-Up", "Power drone, Jetson, GCS, and controller in planned order; wait for services to settle."),
            new ChecklistItem("fl_time_sync", "Power-Up", "Jetson time synchronized; logs/captures will have correct timestamps."),
            new ChecklistItem("fl_health", "Systems", "Edge Core detailed health acceptable: CPU/GPU temp, memory, disk, and service status green."),
            new ChecklistItem("fl_video_confirm", "Systems", "Live video and Task 1 detector overlay visible on the GCS."),
            new ChecklistItem("fl_crosshair_backup", "Systems", "Crosshair capture backup button identified; operator knows when to use it."),
            new ChecklistItem("fl_gps_ready", "Autopilot", "GPS lock stable at the line; home position set; heading matches real orientation."),
            new ChecklistItem("fl_ekf_ready", "Autopilot", "EKF healthy, no critical pre-arm messages, vibration and battery warnings clear."),
            new ChecklistItem("fl_modes", "Autopilot", "Flight modes checked: guided/auto/manual fallback, LAND, and kill/disarm switch."),
            new ChecklistItem("fl_boundary_loaded", "Autopilot", "Map boundary and lap route visible to pilot/GCS; no stale mission loaded."),
            new ChecklistItem("fl_payload_safe", "Payload", "Payload controls page open; payloads armed mechanically but no accidental release path."),
            new ChecklistItem("fl_reel_safe", "Payload", "Reels neutral; no line snagged; full reel state not active before takeoff."),
            new ChecklistItem("fl_ground_alt", "Task 1", "Ground altitude/reference and building model still valid after moving to the line."),
            new ChecklistItem("fl_capture_test", "Task 1", "Optional last capture test completed or intentionally skipped to avoid stale target entries."),
            new ChecklistItem("fl_rtm_radio", "Comms", "RTM callsign ready; radio check completed or queued as required by judge/ATC."),
            new ChecklistItem("fl_observer", "Crew", "Visual observer has eyes on aircraft, traffic, people, and abort authority."),
            new ChecklistItem("fl_start_recording", "Crew", "Start screen/log recording if required; note battery/time at takeoff."),
            new ChecklistItem("fl_takeoff_clearance", "Takeoff", "Takeoff clearance received; pilot announces arming and takeoff."),
            new ChecklistItem("fl_window_timer", "Takeoff", "Competition flight-window timer started at the correct event.")
        };

        private readonly Dictionary<string, CheckBox> _checkboxes = new Dictionary<string, CheckBox>();
        private readonly Dictionary<string, Label> _progressLabels = new Dictionary<string, Label>();
        private bool _loading;

        public Task1ChecklistPanel()
        {
            Dock = DockStyle.Fill;
            BackColor = NOMADTheme.BG_DARK;
            Padding = new Padding(10);
            BuildUi();
            LoadState();
            UpdateAllProgress();
        }

        private void BuildUi()
        {
            var tabs = new TabControl
            {
                Dock = DockStyle.Fill,
                Appearance = TabAppearance.Normal,
            };

            tabs.TabPages.Add(CreateChecklistPage("Before Flight", "preflight", PreFlightItems));
            tabs.TabPages.Add(CreateChecklistPage("Flight Line", "flightline", FlightLineItems));
            Controls.Add(tabs);
        }

        private TabPage CreateChecklistPage(string title, string key, ChecklistItem[] items)
        {
            var page = new TabPage(title)
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(0),
            };

            var listHost = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(8),
            };

            int y = 4;
            string lastPhase = null;
            foreach (var item in items)
            {
                if (item.Phase != lastPhase)
                {
                    listHost.Controls.Add(new Label
                    {
                        Text = item.Phase.ToUpperInvariant(),
                        Font = new Font("Segoe UI", 9, FontStyle.Bold),
                        ForeColor = NOMADTheme.ACCENT,
                        Location = new Point(0, y),
                        AutoSize = true,
                    });
                    y += 24;
                    lastPhase = item.Phase;
                }

                var cb = new CheckBox
                {
                    Text = item.Text,
                    Font = new Font("Segoe UI", 9),
                    ForeColor = NOMADTheme.TEXT_PRIMARY,
                    BackColor = NOMADTheme.BG_DARK,
                    Location = new Point(14, y),
                    AutoSize = true,
                    MaximumSize = new Size(760, 0),
                };
                cb.CheckedChanged += (s, e) =>
                {
                    if (_loading) return;
                    SaveState();
                    UpdateProgress(key, items);
                };
                _checkboxes[item.Id] = cb;
                listHost.Controls.Add(cb);
                y += 24;
            }

            var bottomBar = new Panel
            {
                Dock = DockStyle.Bottom,
                Height = 42,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(8, 6, 8, 6),
            };

            var progress = new Label
            {
                Text = "Progress: 0 / 0",
                Font = new Font("Consolas", 11, FontStyle.Bold),
                ForeColor = NOMADTheme.WARNING,
                Dock = DockStyle.Left,
                Width = 220,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            _progressLabels[key] = progress;
            bottomBar.Controls.Add(progress);

            var btnReset = new Button
            {
                Text = "Reset section",
                BackColor = NOMADTheme.ERROR,
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Width = 130,
                Height = 28,
                Dock = DockStyle.Right,
            };
            btnReset.FlatAppearance.BorderSize = 0;
            btnReset.Click += (s, e) =>
            {
                _loading = true;
                foreach (var item in items)
                {
                    if (_checkboxes.TryGetValue(item.Id, out var cb))
                        cb.Checked = false;
                }
                _loading = false;
                SaveState();
                UpdateProgress(key, items);
            };
            bottomBar.Controls.Add(btnReset);

            page.Controls.Add(listHost);
            page.Controls.Add(bottomBar);
            return page;
        }

        private void UpdateAllProgress()
        {
            UpdateProgress("preflight", PreFlightItems);
            UpdateProgress("flightline", FlightLineItems);
        }

        private void UpdateProgress(string key, ChecklistItem[] items)
        {
            if (!_progressLabels.TryGetValue(key, out var label)) return;
            int done = items.Count(i => _checkboxes.TryGetValue(i.Id, out var cb) && cb.Checked);
            int total = items.Length;
            label.Text = $"Progress: {done} / {total}";
            label.ForeColor = done == total ? NOMADTheme.SUCCESS
                : done == 0 ? NOMADTheme.ERROR : NOMADTheme.WARNING;
        }

        private void LoadState()
        {
            try
            {
                if (!File.Exists(StoragePath)) return;
                var json = File.ReadAllText(StoragePath);
                var saved = JsonConvert.DeserializeObject<Dictionary<string, bool>>(json);
                if (saved == null) return;

                _loading = true;
                foreach (var kv in saved)
                {
                    if (_checkboxes.TryGetValue(kv.Key, out var cb))
                        cb.Checked = kv.Value;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to load Task 1 checklist - {ex.Message}");
            }
            finally
            {
                _loading = false;
            }
        }

        private void SaveState()
        {
            try
            {
                var dir = Path.GetDirectoryName(StoragePath);
                if (!Directory.Exists(dir))
                    Directory.CreateDirectory(dir);

                var state = _checkboxes.ToDictionary(kv => kv.Key, kv => kv.Value.Checked);
                File.WriteAllText(StoragePath, JsonConvert.SerializeObject(state, Formatting.Indented));
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to save Task 1 checklist - {ex.Message}");
            }
        }
    }
}
