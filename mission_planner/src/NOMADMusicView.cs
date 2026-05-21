// ============================================================
// NOMAD Music View - UI for motor-tone music playback
// ============================================================
// Pick a song, choose playback mode (motor test or DroneCAN
// beep), configure output, and hit Play. Props OFF only.
// ============================================================

using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public class NOMADMusicView : NOMADViewBase
    {
        private readonly MotorMusicPlayer _player = MotorMusicPlayer.Instance;

        private ListBox _lstSongs;
        private CheckedListBox _lstCanBuses;
        private NumericUpDown _numTempo;
        private NumericUpDown _numSourceNode;
        private NumericUpDown _numMotorCount;
        private NumericUpDown _numMinPwm;
        private NumericUpDown _numMaxPwm;
        private ComboBox _cmbMode;
        private CheckBox _chkArmedOverride;
        private Button _btnPlay;
        private Button _btnStop;
        private Label _lblStatus;
        private ProgressBar _progress;
        private System.Windows.Forms.Timer _uiTimer;

        private Panel _motorTestPanel;
        private Panel _droneCanPanel;

        public NOMADMusicView()
        {
            InitializeUI();
            _player.StatusChanged += OnStatusChanged;
            _player.PlayingChanged += OnPlayingChanged;

            _uiTimer = new System.Windows.Forms.Timer { Interval = 100 };
            _uiTimer.Tick += (s, e) =>
            {
                var p = (int)(_player.Progress * 100);
                _progress.Value = Math.Max(0, Math.Min(100, p));
            };
            _uiTimer.Start();
        }

        private void InitializeUI()
        {
            var root = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                BackColor = NOMADTheme.BG_DARK,
            };
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55f));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 45f));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100f));
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 120f));

            var songCard = CreateSectionCard("Song Library");
            songCard.Dock = DockStyle.Fill;
            _lstSongs = new ListBox
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 10),
            };
            foreach (var s in MotorMusicPlayer.Library.Values.OrderBy(x => x.Title))
            {
                _lstSongs.Items.Add(new SongItem(s));
            }
            if (_lstSongs.Items.Count > 0) _lstSongs.SelectedIndex = 0;
            songCard.Controls.Add(_lstSongs);
            root.Controls.Add(songCard, 0, 0);

            var settingsCard = CreateSectionCard("Output / Safety");
            settingsCard.Dock = DockStyle.Fill;
            var settings = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                AutoSize = true,
                BackColor = settingsCard.BackColor,
            };
            settings.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 130f));
            settings.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));

            int row = 0;

            settings.Controls.Add(MakeLabel("Mode:"), 0, row);
            _cmbMode = new ComboBox
            {
                Dock = DockStyle.Top,
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9),
            };
            _cmbMode.Items.Add("Motor Test (PWM)");
            _cmbMode.Items.Add("DroneCAN Beep");
            _cmbMode.SelectedIndex = 0;
            _cmbMode.SelectedIndexChanged += (s, e) => UpdateModeVisibility();
            settings.Controls.Add(_cmbMode, 1, row);
            row++;

            settings.Controls.Add(MakeLabel("Tempo (x speed):"), 0, row);
            _numTempo = MakeNumeric(0.25m, 4.0m, 1.0m, 2, 0.1m);
            settings.Controls.Add(_numTempo, 1, row);
            row++;

            _chkArmedOverride = new CheckBox
            {
                Text = "Allow while ARMED (DANGEROUS - props off!)",
                ForeColor = NOMADTheme.WARNING,
                BackColor = settingsCard.BackColor,
                AutoSize = true,
                Margin = new Padding(3, 8, 3, 3),
            };
            settings.Controls.Add(_chkArmedOverride, 1, row);
            row++;

            _motorTestPanel = new Panel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                BackColor = settingsCard.BackColor,
                Padding = Padding.Empty,
            };
            var mtLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                ColumnCount = 2,
                AutoSize = true,
                BackColor = settingsCard.BackColor,
            };
            mtLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 130f));
            mtLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));

            mtLayout.Controls.Add(MakeLabel("Motor count:"), 0, 0);
            _numMotorCount = MakeNumeric(1, 8, 4, 0, 1);
            mtLayout.Controls.Add(_numMotorCount, 1, 0);

            mtLayout.Controls.Add(MakeLabel("Min PWM:"), 0, 1);
            _numMinPwm = MakeNumeric(1000, 1300, 1050, 0, 10);
            mtLayout.Controls.Add(_numMinPwm, 1, 1);

            mtLayout.Controls.Add(MakeLabel("Max PWM:"), 0, 2);
            _numMaxPwm = MakeNumeric(1300, 2200, 1800, 0, 10);
            mtLayout.Controls.Add(_numMaxPwm, 1, 2);

            _motorTestPanel.Controls.Add(mtLayout);
            settings.Controls.Add(_motorTestPanel, 0, row);
            settings.SetColumnSpan(_motorTestPanel, 2);
            row++;

            _droneCanPanel = new Panel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                BackColor = settingsCard.BackColor,
                Padding = Padding.Empty,
            };
            var dcLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                ColumnCount = 2,
                AutoSize = true,
                BackColor = settingsCard.BackColor,
            };
            dcLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 130f));
            dcLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));

            dcLayout.Controls.Add(MakeLabel("DroneCAN bus:"), 0, 0);
            _lstCanBuses = new CheckedListBox
            {
                Height = 84,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
                CheckOnClick = true,
            };
            _lstCanBuses.Items.Add("CAN 1", true);
            _lstCanBuses.Items.Add("CAN 2", false);
            _lstCanBuses.Items.Add("CAN 3", false);
            dcLayout.Controls.Add(_lstCanBuses, 1, 0);

            dcLayout.Controls.Add(MakeLabel("Source node:"), 0, 1);
            _numSourceNode = MakeNumeric(1, 127, 126, 0, 1);
            dcLayout.Controls.Add(_numSourceNode, 1, 1);

            _droneCanPanel.Controls.Add(dcLayout);
            settings.Controls.Add(_droneCanPanel, 0, row);
            settings.SetColumnSpan(_droneCanPanel, 2);
            row++;

            settingsCard.Controls.Add(settings);
            root.Controls.Add(settingsCard, 1, 0);

            var transport = CreateSectionCard("Playback");
            transport.Dock = DockStyle.Fill;

            _btnPlay = new Button
            {
                Text = "\u25B6 Play",
                BackColor = NOMADTheme.BTN_START,
                ForeColor = TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                Size = new Size(110, 38),
                Location = new Point(15, 35),
            };
            _btnPlay.FlatAppearance.BorderSize = 0;
            _btnPlay.Click += (s, e) => OnPlayClick();

            _btnStop = new Button
            {
                Text = "\u25A0 Stop",
                BackColor = NOMADTheme.BTN_STOP,
                ForeColor = TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                Size = new Size(110, 38),
                Location = new Point(135, 35),
            };
            _btnStop.FlatAppearance.BorderSize = 0;
            _btnStop.Click += (s, e) => _player.Stop();

            _progress = new ProgressBar
            {
                Location = new Point(255, 42),
                Size = new Size(420, 22),
                Minimum = 0,
                Maximum = 100,
            };

            _lblStatus = new Label
            {
                Text = "Idle. Remove propellers before playing.",
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                Location = new Point(15, 85),
                Font = new Font("Segoe UI", 9),
            };

            transport.Controls.Add(_btnPlay);
            transport.Controls.Add(_btnStop);
            transport.Controls.Add(_progress);
            transport.Controls.Add(_lblStatus);

            root.Controls.Add(transport, 0, 1);
            root.SetColumnSpan(transport, 2);

            this.Controls.Add(root);

            UpdateModeVisibility();
        }

        private void UpdateModeVisibility()
        {
            bool motorTest = _cmbMode.SelectedIndex == 0;
            _motorTestPanel.Visible = motorTest;
            _droneCanPanel.Visible = !motorTest;
        }

        private Panel CreateSectionCard(string title)
        {
            var card = new Panel
            {
                BackColor = NOMADTheme.CARD_BG,
                Margin = new Padding(8),
                Padding = new Padding(12, 28, 12, 12),
            };
            var lbl = new Label
            {
                Text = title,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Location = new Point(12, 6),
            };
            card.Controls.Add(lbl);
            return card;
        }

        private static Label MakeLabel(string text)
        {
            return new Label
            {
                Text = text,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
                AutoSize = false,
                Margin = new Padding(0, 6, 0, 0),
            };
        }

        private static NumericUpDown MakeNumeric(decimal min, decimal max, decimal val, int decimals, decimal increment)
        {
            return new NumericUpDown
            {
                Minimum = min,
                Maximum = max,
                Value = val,
                DecimalPlaces = decimals,
                Increment = increment,
                Dock = DockStyle.Top,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
                Margin = new Padding(0, 3, 0, 3),
            };
        }

        private void OnPlayClick()
        {
            var item = _lstSongs.SelectedItem as SongItem;
            if (item == null) { SetStatus("Pick a song first.", true); return; }

            _player.Mode = _cmbMode.SelectedIndex == 0
                ? MotorMusicPlayer.PlayMode.MotorTest
                : MotorMusicPlayer.PlayMode.DroneCanBeep;
            _player.AllowArmedOverride = _chkArmedOverride.Checked;

            if (_player.Mode == MotorMusicPlayer.PlayMode.MotorTest)
            {
                _player.MotorCount = (int)_numMotorCount.Value;
                _player.MinPwm = (int)_numMinPwm.Value;
                _player.MaxPwm = (int)_numMaxPwm.Value;
                if (_player.MinPwm >= _player.MaxPwm)
                {
                    SetStatus("Min PWM must be less than Max PWM.", true);
                    return;
                }
            }
            else
            {
                _player.DroneCanSourceNodeId = (int)_numSourceNode.Value;
            }

            var buses = _lstCanBuses.CheckedIndices.Cast<int>().Select(i => i + 1).ToList();

            string err = _player.Play(item.Song.Id, buses, (double)_numTempo.Value);
            if (err != null) SetStatus(err, true);
        }

        private void OnStatusChanged(string msg)
        {
            InvokeOnUi(() => SetStatus(msg, false));
        }

        private void OnPlayingChanged(bool playing)
        {
            InvokeOnUi(() =>
            {
                _btnPlay.Enabled = !playing;
                if (!playing) _progress.Value = 0;
            });
        }

        private void InvokeOnUi(Action a)
        {
            if (this.IsDisposed || !this.IsHandleCreated) return;
            try
            {
                if (this.InvokeRequired) this.BeginInvoke(a);
                else a();
            }
            catch { }
        }

        private void SetStatus(string msg, bool isError)
        {
            _lblStatus.Text = msg;
            _lblStatus.ForeColor = isError ? NOMADTheme.ERROR : NOMADTheme.TEXT_SECONDARY;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _uiTimer?.Stop();
                _uiTimer?.Dispose();
                try { _player.StatusChanged -= OnStatusChanged; } catch { }
                try { _player.PlayingChanged -= OnPlayingChanged; } catch { }
                try { _player.Stop(); } catch { }
            }
            base.Dispose(disposing);
        }

        private sealed class SongItem
        {
            public readonly MotorMusicPlayer.Song Song;
            public SongItem(MotorMusicPlayer.Song s) { Song = s; }
            public override string ToString()
            {
                return string.Format("{0} ({1:0.0}s, {2} notes)",
                    Song.TrackCount > 1 ? Song.Title + " [track 1/" + Song.TrackCount + "]" : Song.Title,
                    Song.TotalMs / 1000.0, Song.Notes.Count);
            }
        }
    }
}
