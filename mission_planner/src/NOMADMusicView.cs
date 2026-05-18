// ============================================================
// NOMAD Music View - UI for motor-tone music playback
// ============================================================
// Pick a song, pick which motor channels to drive, set a tempo,
// and hit Play. Commands go out as DO_SET_SERVO via the local
// MAVLink link. Props OFF only.
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
        private CheckedListBox _lstChannels;
        private NumericUpDown _numTempo;
        private NumericUpDown _numPwmMin;
        private NumericUpDown _numPwmMax;
        private CheckBox _chkArmedOverride;
        private Button _btnPlay;
        private Button _btnStop;
        private Label _lblStatus;
        private ProgressBar _progress;
        private System.Windows.Forms.Timer _uiTimer;

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

            // ---- Songs card ----
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

            // ---- Settings card ----
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

            settings.Controls.Add(MakeLabel("Motor channels:"), 0, 0);
            _lstChannels = new CheckedListBox
            {
                Dock = DockStyle.Top,
                Height = 100,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
                CheckOnClick = true,
            };
            for (int ch = 1; ch <= 8; ch++)
                _lstChannels.Items.Add("Channel " + ch, ch >= 1 && ch <= 4);
            settings.Controls.Add(_lstChannels, 1, 0);

            settings.Controls.Add(MakeLabel("Tempo (× speed):"), 0, 1);
            _numTempo = MakeNumeric(0.25m, 4.0m, 1.0m, 2, 0.1m);
            settings.Controls.Add(_numTempo, 1, 1);

            settings.Controls.Add(MakeLabel("PWM min (µs):"), 0, 2);
            _numPwmMin = MakeNumeric(1000, 1500, 1100, 0, 10);
            settings.Controls.Add(_numPwmMin, 1, 2);

            settings.Controls.Add(MakeLabel("PWM max (µs):"), 0, 3);
            _numPwmMax = MakeNumeric(1200, 2000, 1900, 0, 10);
            settings.Controls.Add(_numPwmMax, 1, 3);

            _chkArmedOverride = new CheckBox
            {
                Text = "Allow playback while ARMED (DANGEROUS — props off!)",
                ForeColor = NOMADTheme.WARNING,
                BackColor = settingsCard.BackColor,
                AutoSize = true,
                Margin = new Padding(3, 8, 3, 3),
            };
            settings.Controls.Add(_chkArmedOverride, 1, 4);

            settingsCard.Controls.Add(settings);
            root.Controls.Add(settingsCard, 1, 0);

            // ---- Transport / status card spanning bottom ----
            var transport = CreateSectionCard("Playback");
            transport.Dock = DockStyle.Fill;

            _btnPlay = new Button
            {
                Text = "▶ Play",
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
                Text = "■ Stop",
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

            var channels = _lstChannels.CheckedIndices.Cast<int>().Select(i => i + 1).ToList();
            if (channels.Count == 0) { SetStatus("Pick at least one motor channel.", true); return; }

            int pwmMin = (int)_numPwmMin.Value;
            int pwmMax = (int)_numPwmMax.Value;
            if (pwmMin >= pwmMax) { SetStatus("PWM min must be < PWM max.", true); return; }

            _player.PwmMin = pwmMin;
            _player.PwmMax = pwmMax;
            _player.AllowArmedOverride = _chkArmedOverride.Checked;

            string err = _player.Play(item.Song.Id, channels, (double)_numTempo.Value);
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
                return string.Format("{0}   ({1:0.0}s, {2} notes)",
                    Song.Title, Song.TotalMs / 1000.0, Song.Notes.Count);
            }
        }
    }
}
