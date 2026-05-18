// ============================================================
// RTM SOP Checklist Panel (CONOPS Appendix F)
// ============================================================
// Reusable pilot-aid panel: every row is one radio call the
// team needs to make to ATC in the right order. Operator ticks
// the box as each call is completed.
//
// Check state is persisted per storage key so progress survives
// restarts and is independent across tasks.
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
    public class RtmChecklistPanel : Panel
    {
        // Task 2 RTM SOP calls (CONOPS Appendix F, scored 15 pts).
        public static readonly (string phase, string text)[] TASK2_ITEMS =
        {
            ("Setup", "Radio check: \"<Callsign> to base, radio check\" — receive signal rating"),
            ("Dispatch", "Dispatch briefing: respond \"<Callsign> to dispatch, go ahead\""),
            ("Dispatch", "After briefing complete: transmit \"<Callsign> Wilco\""),
            ("Takeoff", "Request takeoff: \"<Callsign> to base, request takeoff\""),
            ("Takeoff", "Acknowledge clearance: \"Cleared to takeoff, <Callsign>\""),
            ("Takeoff", "After airborne: \"<Callsign> takeoff complete\""),
            ("Corridor", "Climb to 20–35 m UAM corridor (hold ≥30 s)"),
            ("Corridor", "Entering corridor: \"<Callsign> entering corridor\""),
            ("Corridor", "Leaving corridor (approach building): \"<Callsign> has left the corridor\""),
            ("Building", "Crossing search-volume boundary: \"<Callsign> operating near the building\""),
            ("Engagement", "Auto Spray (1×) — claim autonomy gate"),
            ("Engagement", "Manual sprays for remaining targets"),
            ("Return", "Climb back into UAM corridor: \"<Callsign> entering corridor\""),
            ("Return", "Leaving corridor at vertiport: \"<Callsign> has left the corridor\""),
            ("Landing", "Request landing: \"<Callsign> to base, request landing\""),
            ("Landing", "Acknowledge clearance: \"Cleared to land, <Callsign>\""),
            ("Landing", "After touchdown: \"<Callsign> landed\""),
        };

        // Task 1 calls. Same SOP framework, but no emergency dispatch
        // briefing (Task 1 building coords are given in advance), and the
        // engagement phase is laps + equipment drop + visual target ID
        // rather than spray.
        public static readonly (string phase, string text)[] TASK1_ITEMS =
        {
            ("Setup", "Radio check: \"<Callsign> to base, radio check\" — receive signal rating"),
            ("Setup", "Confirm callsign registered with Big City"),
            ("Takeoff", "Request takeoff: \"<Callsign> to base, request takeoff\""),
            ("Takeoff", "Acknowledge clearance: \"Cleared to takeoff, <Callsign>\""),
            ("Takeoff", "After airborne: \"<Callsign> takeoff complete\""),
            ("Corridor", "Climb to 20–35 m UAM corridor (hold ≥30 s)"),
            ("Corridor", "Entering corridor: \"<Callsign> entering corridor\""),
            ("Laps", "Fly the lap course (count complete laps only on the way to the scene)"),
            ("Scene", "Leaving corridor (approach scene): \"<Callsign> has left the corridor\""),
            ("Scene", "Crossing search-volume boundary: \"<Callsign> operating near the building\""),
            ("Scene", "Deliver equipment to staging pads (land or hover-low, no airdrop)"),
            ("Scene", "Capture targets; build relative-location descriptions"),
            ("Return", "Climb back into UAM corridor: \"<Callsign> entering corridor\""),
            ("Return", "Leaving corridor at vertiport: \"<Callsign> has left the corridor\""),
            ("Landing", "Request landing: \"<Callsign> to base, request landing\""),
            ("Landing", "Acknowledge clearance: \"Cleared to land, <Callsign>\""),
            ("Landing", "After touchdown: \"<Callsign> landed\""),
            ("Submit", "Upload Task_1_<team>_targets.txt before flight window ends"),
        };

        private static readonly string StorageDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner", "plugins", "NOMAD");

        private readonly string _storagePath;
        private readonly (string phase, string text)[] _items;
        private readonly List<CheckBox> _checkboxes = new List<CheckBox>();
        private Label _lblProgress;
        private bool _loading;

        public RtmChecklistPanel(string storageKey, (string phase, string text)[] items)
        {
            _items = items;
            _storagePath = Path.Combine(StorageDir, $"rtm_checklist_{storageKey}.json");
            Dock = DockStyle.Fill;
            BackColor = NOMADTheme.BG_DARK;
            Padding = new Padding(10);
            BuildUi();
            LoadState();
            UpdateProgress();
        }

        private void BuildUi()
        {
            var listHost = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                BackColor = NOMADTheme.BG_DARK,
            };

            int y = 4;
            string lastPhase = null;
            foreach (var (phase, text) in _items)
            {
                if (phase != lastPhase)
                {
                    listHost.Controls.Add(new Label
                    {
                        Text = $"── {phase} ──",
                        Font = new Font("Consolas", 10, FontStyle.Bold),
                        ForeColor = NOMADTheme.ACCENT,
                        Location = new Point(0, y),
                        AutoSize = true,
                    });
                    y += 24;
                    lastPhase = phase;
                }

                var cb = new CheckBox
                {
                    Text = text,
                    Font = new Font("Segoe UI", 9),
                    ForeColor = NOMADTheme.TEXT_PRIMARY,
                    Location = new Point(12, y),
                    AutoSize = true,
                    AutoCheck = true,
                    BackColor = NOMADTheme.BG_DARK,
                };
                cb.CheckedChanged += (s, e) =>
                {
                    if (_loading) return;
                    SaveState();
                    UpdateProgress();
                };
                _checkboxes.Add(cb);
                listHost.Controls.Add(cb);
                y += 22;
            }

            var bottomBar = new Panel
            {
                Dock = DockStyle.Bottom,
                Height = 40,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(8, 6, 8, 6),
            };

            _lblProgress = new Label
            {
                Text = $"Progress: 0 / {_items.Length}",
                Font = new Font("Consolas", 11, FontStyle.Bold),
                ForeColor = NOMADTheme.WARNING,
                Dock = DockStyle.Left,
                AutoSize = false,
                Width = 220,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            bottomBar.Controls.Add(_lblProgress);

            var btnReset = new Button
            {
                Text = "Reset checklist",
                BackColor = NOMADTheme.ERROR,
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Width = 140,
                Height = 28,
                Dock = DockStyle.Right,
            };
            btnReset.FlatAppearance.BorderSize = 0;
            btnReset.Click += (s, e) =>
            {
                _loading = true;
                foreach (var cb in _checkboxes) cb.Checked = false;
                _loading = false;
                SaveState();
                UpdateProgress();
            };
            bottomBar.Controls.Add(btnReset);

            var header = new Label
            {
                Text = "Big City RTM SOPs — tick each call as it is made.\n"
                     + "ICAO phonetic alphabet · 3-digit headings clockwise from magnetic north · full callsign at start of each exchange.",
                Font = new Font("Segoe UI", 9),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Dock = DockStyle.Top,
                Height = 44,
                Padding = new Padding(0, 0, 0, 8),
            };

            Controls.Add(listHost);
            Controls.Add(bottomBar);
            Controls.Add(header);
        }

        private void UpdateProgress()
        {
            int done = _checkboxes.Count(c => c.Checked);
            int total = _checkboxes.Count;
            if (_lblProgress == null) return;
            _lblProgress.Text = $"Progress: {done} / {total}";
            _lblProgress.ForeColor = done == total ? NOMADTheme.SUCCESS
                : done >= total / 2 ? NOMADTheme.WARNING : NOMADTheme.ERROR;
        }

        private void LoadState()
        {
            try
            {
                if (!File.Exists(_storagePath)) return;
                var json = File.ReadAllText(_storagePath);
                var saved = JsonConvert.DeserializeObject<bool[]>(json);
                if (saved == null) return;
                _loading = true;
                for (int i = 0; i < _checkboxes.Count && i < saved.Length; i++)
                    _checkboxes[i].Checked = saved[i];
                _loading = false;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to load RTM checklist - {ex.Message}");
                _loading = false;
            }
        }

        private void SaveState()
        {
            try
            {
                if (!Directory.Exists(StorageDir))
                    Directory.CreateDirectory(StorageDir);
                var state = _checkboxes.Select(c => c.Checked).ToArray();
                File.WriteAllText(_storagePath, JsonConvert.SerializeObject(state));
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to save RTM checklist - {ex.Message}");
            }
        }
    }
}
