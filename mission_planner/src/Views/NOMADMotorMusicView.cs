// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Motor Music View
// ============================================================

using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public class NOMADMotorMusicView : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;
        private readonly MotorMusicPlayer _player = new MotorMusicPlayer();

        private MotorMusicMidiFile _midi;
        private MicrophonePitchSource _microphone;
        private readonly Stopwatch _speakerClock = new Stopwatch();
        private int _speakerInFlight;
        private int _speakerMotorSlot = 1;
        private int _lastSpeakerNote = -1;

        private Label _lblConnection;
        private Label _lblSummary;
        private Label _lblPlayback;
        private Label _lblSpeaker;
        private ProgressBar _progress;
        private CheckBox _chkSafety;
        private NumericUpDown _numMotors;
        private NumericUpDown _numMinOutput;
        private NumericUpDown _numMaxOutput;
        private NumericUpDown _numTranspose;
        private NumericUpDown _numTempo;
        private Button _btnPlay;
        private Button _btnStop;
        private Button _btnSpeaker;

        public NOMADMotorMusicView(NOMADConfig config)
        {
            _config = config ?? NOMADConfig.Load();
            _player.Progress += OnPlaybackProgress;
            _player.Completed += OnPlaybackCompleted;
            InitializeUI();
        }

        private void InitializeUI()
        {
            AutoScroll = false;

            var root = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                BackColor = NOMADTheme.BG_DARK,
            };
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100f));

            root.Controls.Add(BuildSetupCard(), 0, 0);
            root.Controls.Add(BuildControlsCard(), 0, 1);
            root.Controls.Add(BuildTabs(), 0, 2);

            Controls.Add(root);
        }

        private Control BuildSetupCard()
        {
            var card = ControlFactory.Card("Motor Music Setup", out var body);
            body.Controls.Add(MakeMutedLabel("Install the Lua bridge, reboot or restart scripting, then play only " +
                                             "with props removed and the vehicle disarmed."));
            body.Controls.Add(MakeMutedLabel("Uses ArduPilot Motor1..MotorN speed outputs only; no ESC buzzer, " +
                                             "DShot tune, DroneCAN, or vendor-specific setup is required."));

            _lblConnection = MakeMutedLabel("Vehicle link: checking...");

            var install = MakeButton(
                "Install Lua Script",
                async (s, e) => await InstallScriptAsync(),
                NOMADTheme.ACCENT,
                150);
            var ping = MakeButton("Ping Script", async (s, e) => await PingScriptAsync(), NOMADTheme.BUTTON_BG, 110);
            var stop = MakeButton("Stop All", async (s, e) => await StopAllAsync(), NOMADTheme.BTN_STOP, 100);

            body.Controls.Add(ControlFactory.ButtonRow(install, ping, stop));
            body.Controls.Add(_lblConnection);
            return card;
        }

        private Control BuildControlsCard()
        {
            var card = ControlFactory.Card("Spin Output Range", out var body);
            var grid = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                ColumnCount = 2,
                RowCount = 3,
                BackColor = Color.Transparent,
            };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50f));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50f));

            _numMotors = ControlFactory.Numeric(1, 12, _config.MotorMusicMotorCount, 1, 0, 80);
            _numMinOutput = ControlFactory.Numeric(1000, 2000, _config.MotorMusicMinOutputPwm, 1, 0, 80);
            _numMaxOutput = ControlFactory.Numeric(1000, 2000, _config.MotorMusicMaxOutputPwm, 1, 0, 80);
            _numTranspose = ControlFactory.Numeric(-48, 12, _config.MotorMusicTranspose, 1, 0, 80);
            _numTempo = ControlFactory.Numeric(0.25m, 2.00m, (decimal)_config.MotorMusicTempoScale, 0.05m, 2, 80);

            grid.Controls.Add(ControlFactory.LabeledRow("Motors:", _numMotors), 0, 0);
            grid.Controls.Add(ControlFactory.LabeledRow("Min spin:", _numMinOutput, "us equiv"), 1, 0);
            grid.Controls.Add(ControlFactory.LabeledRow("Max spin:", _numMaxOutput, "us equiv"), 0, 1);
            grid.Controls.Add(ControlFactory.LabeledRow("Transpose:", _numTranspose, "semitones"), 1, 1);
            grid.Controls.Add(ControlFactory.LabeledRow("Tempo:", _numTempo, "x"), 0, 2);

            _chkSafety = ControlFactory.CheckBox("Props removed; motors may spin up to max output");
            _chkSafety.ForeColor = NOMADTheme.WARNING;
            _chkSafety.Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold);

            body.Controls.Add(grid);
            body.Controls.Add(_chkSafety);
            return card;
        }

        private Control BuildTabs()
        {
            var tabs = ControlFactory.TabControl();
            tabs.TabPages.Add(BuildMidiTab());
            tabs.TabPages.Add(BuildSpeakerTab());
            return tabs;
        }

        private TabPage BuildMidiTab()
        {
            var tab = new TabPage("MIDI File")
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(NOMADTheme.PAD),
            };

            var panel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                BackColor = Color.Transparent,
            };
            panel.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            panel.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            panel.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100f));

            var browse = MakeButton("Load .mid", (s, e) => LoadMidi(), NOMADTheme.BUTTON_BG, 110);
            _btnPlay = MakeButton("Play MIDI", (s, e) => PlayMidi(), NOMADTheme.BTN_START, 110);
            _btnStop = MakeButton("Stop", (s, e) => _player.Stop(), NOMADTheme.BTN_STOP, 90);
            _btnPlay.Enabled = false;

            _lblSummary = MakeMutedLabel("No MIDI file loaded.");
            _lblPlayback = MakeMutedLabel("Idle");
            _progress = new ProgressBar
            {
                Dock = DockStyle.Top,
                Height = 18,
                Maximum = 1000,
                Style = ProgressBarStyle.Continuous,
            };

            panel.Controls.Add(ControlFactory.ButtonRow(browse, _btnPlay, _btnStop), 0, 0);
            panel.Controls.Add(_lblSummary, 0, 1);
            panel.Controls.Add(_progress, 0, 2);
            panel.Controls.Add(_lblPlayback, 0, 3);

            tab.Controls.Add(panel);
            return tab;
        }

        private TabPage BuildSpeakerTab()
        {
            var tab = new TabPage("Speaker Mode")
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(NOMADTheme.PAD),
            };

            var panel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                BackColor = Color.Transparent,
            };
            panel.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            panel.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100f));

            _btnSpeaker = MakeButton("Start Speaker", (s, e) => ToggleSpeaker(), NOMADTheme.BTN_START, 130);
            _lblSpeaker = MakeMutedLabel(
                "Microphone idle. Live mode follows the dominant pitch and holds motor speed briefly.");

            panel.Controls.Add(ControlFactory.ButtonRow(_btnSpeaker), 0, 0);
            panel.Controls.Add(_lblSpeaker, 0, 1);
            tab.Controls.Add(panel);
            return tab;
        }

        private async Task InstallScriptAsync()
        {
            SetStatus("Installing Lua script over MAVFTP...", NOMADTheme.WARNING);
            var text = await MotorMusicCommand.InstallScriptAsync(CancellationToken.None);
            SetStatus(text, text.StartsWith("Install failed") ? NOMADTheme.ERROR : NOMADTheme.SUCCESS);
        }

        private async Task PingScriptAsync()
        {
            bool ok = await MotorMusicCommand.SendPingAsync();
            SetStatus(
                ok ? "Ping sent. Check Messages for the script response." : "Could not send ping.",
                ok ? NOMADTheme.SUCCESS : NOMADTheme.ERROR);
        }

        private async Task StopAllAsync()
        {
            _player.Stop();
            StopSpeaker();
            bool ok = await MotorMusicCommand.SendStopAsync();
            SetStatus(
                ok ? "Stop command sent." : "Could not send stop command.",
                ok ? NOMADTheme.SUCCESS : NOMADTheme.ERROR);
        }

        private void LoadMidi()
        {
            using (var dlg = new OpenFileDialog())
            {
                dlg.Filter = "MIDI files (*.mid;*.midi)|*.mid;*.midi|All files (*.*)|*.*";
                dlg.Title = "Load MIDI file";
                if (dlg.ShowDialog(FindForm()) != DialogResult.OK) return;

                try
                {
                    _midi = MotorMusicMidiParser.Load(dlg.FileName);
                    _lblSummary.Text = _midi.Summary;
                    _btnPlay.Enabled = true;
                    _progress.Value = 0;
                }
                catch (Exception ex)
                {
                    _btnPlay.Enabled = false;
                    _lblSummary.Text = "Could not load MIDI: " + ex.Message;
                }
            }
        }

        private void PlayMidi()
        {
            if (!CanPlay()) return;
            SaveOptions();
            _player.Play(_midi, BuildOptions());
            _lblPlayback.Text = "Playing...";
        }

        private void ToggleSpeaker()
        {
            if (_microphone == null) StartSpeaker();
            else StopSpeaker();
        }

        private void StartSpeaker()
        {
            if (!CanPlay()) return;
            SaveOptions();

            try
            {
                _microphone = new MicrophonePitchSource();
                _microphone.PitchDetected += OnPitchDetected;
                _speakerClock.Restart();
                _lastSpeakerNote = -1;
                _speakerMotorSlot = 1;
                _microphone.Start();
                _btnSpeaker.Text = "Stop Speaker";
                _btnSpeaker.BackColor = NOMADTheme.BTN_STOP;
                _lblSpeaker.Text = "Listening...";
            }
            catch (Exception ex)
            {
                _microphone?.Dispose();
                _microphone = null;
                _lblSpeaker.Text = "Microphone unavailable: " + ex.Message;
            }
        }

        private void StopSpeaker()
        {
            var mic = _microphone;
            _microphone = null;
            if (mic != null)
            {
                mic.PitchDetected -= OnPitchDetected;
                mic.Dispose();
            }
            if (_btnSpeaker != null)
            {
                _btnSpeaker.Text = "Start Speaker";
                _btnSpeaker.BackColor = NOMADTheme.BTN_START;
            }
            if (_lblSpeaker != null) _lblSpeaker.Text = "Microphone idle.";
        }

        private void OnPitchDetected(object sender, PitchDetectedEventArgs e)
        {
            if (_speakerClock.ElapsedMilliseconds < 90) return;
            _speakerClock.Restart();
            if (System.Threading.Interlocked.Exchange(ref _speakerInFlight, 1) == 1) return;

            var options = BuildOptions();
            int note = FrequencyToMidi(e.FrequencyHz) + options.Transpose;
            note = Math.Max(24, Math.Min(96, note));
            int velocity = Math.Max(25, Math.Min(127, (int)(e.Level * 550)));
            int motor = _speakerMotorSlot;
            _speakerMotorSlot++;
            if (_speakerMotorSlot > options.MotorCount) _speakerMotorSlot = 1;

            Task.Run(async () =>
            {
                try
                {
                    await MotorMusicCommand.SendNoteAsync(
                        motor, note, velocity, 180, options.MaxOutputPwm, options.MinOutputPwm).ConfigureAwait(false);
                    UpdateSpeakerLabel(note, e.FrequencyHz, velocity);
                    _lastSpeakerNote = note;
                }
                finally
                {
                    System.Threading.Interlocked.Exchange(ref _speakerInFlight, 0);
                }
            });
        }

        private bool CanPlay()
        {
            if (!_chkSafety.Checked)
            {
                MessageBox.Show(
                    "Confirm props are removed, the vehicle is disarmed, and the area is clear before " +
                    "spinning motor outputs.",
                    "Motor Music Safety",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning);
                return false;
            }
            if (!MotorMusicCommand.IsConnected)
            {
                SetStatus("Vehicle link is not open.", NOMADTheme.ERROR);
                return false;
            }
            return true;
        }

        private MotorMusicCommandOptions BuildOptions()
        {
            int minOutput = (int)_numMinOutput.Value;
            int maxOutput = Math.Max(minOutput, (int)_numMaxOutput.Value);
            return new MotorMusicCommandOptions
            {
                MotorCount = (int)_numMotors.Value,
                MinOutputPwm = minOutput,
                MaxOutputPwm = maxOutput,
                Transpose = (int)_numTranspose.Value,
                TempoScale = (double)_numTempo.Value,
            };
        }

        private void SaveOptions()
        {
            int minOutput = (int)_numMinOutput.Value;
            int maxOutput = Math.Max(minOutput, (int)_numMaxOutput.Value);
            _numMaxOutput.Value = maxOutput;

            _config.MotorMusicMotorCount = (int)_numMotors.Value;
            _config.MotorMusicMinOutputPwm = minOutput;
            _config.MotorMusicMaxOutputPwm = maxOutput;
            _config.MotorMusicTranspose = (int)_numTranspose.Value;
            _config.MotorMusicTempoScale = (double)_numTempo.Value;
            _config.Save();
        }

        private void OnPlaybackProgress(MotorMusicPlaybackProgress progress)
        {
            if (!IsHandleCreated || IsDisposed) return;
            BeginInvoke((MethodInvoker)(() =>
            {
                int value = 0;
                if (progress.LengthMs > 0)
                    value = Math.Max(0, Math.Min(1000, (int)(progress.PositionMs * 1000.0 / progress.LengthMs)));
                _progress.Value = value;
                _lblPlayback.Text =
                    $"Position {progress.PositionMs / 1000.0:0.0}s - sent {progress.SentNotes}, " +
                    $"dropped {progress.DroppedNotes}";
            }));
        }

        private void OnPlaybackCompleted(string message)
        {
            if (!IsHandleCreated || IsDisposed) return;
            BeginInvoke((MethodInvoker)(() => _lblPlayback.Text = message));
        }

        private void UpdateSpeakerLabel(int note, double freq, int velocity)
        {
            if (!IsHandleCreated || IsDisposed) return;
            if (note == _lastSpeakerNote && _lblSpeaker.Text.Contains("Note")) return;
            BeginInvoke((MethodInvoker)(() =>
                _lblSpeaker.Text = $"Note {note} from {freq:0} Hz - velocity {velocity}"));
        }

        private void SetStatus(string text, Color color)
        {
            if (_lblConnection == null) return;
            _lblConnection.Text = text;
            _lblConnection.ForeColor = color;
        }

        public void UpdateData()
        {
            if (_lblConnection == null) return;
            var status = MotorMusicCommand.IsConnected ? "Vehicle link: open" : "Vehicle link: disconnected";
            _lblConnection.Text = status;
            _lblConnection.ForeColor = MotorMusicCommand.IsConnected ? NOMADTheme.SUCCESS : NOMADTheme.ERROR;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                StopSpeaker();
                _player.Dispose();
            }
            base.Dispose(disposing);
        }

        private static int FrequencyToMidi(double frequency)
        {
            if (frequency <= 0) return 60;
            return (int)Math.Round(69 + 12 * Math.Log(frequency / 440.0, 2.0));
        }

        private static Label MakeMutedLabel(string text)
        {
            return new Label
            {
                Text = text,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY),
                AutoSize = true,
                Dock = DockStyle.Top,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };
        }

        private static Button MakeButton(string text, EventHandler click, Color color, int width)
        {
            var btn = ControlFactory.Button(text, click, color, width, 30);
            btn.Margin = new Padding(0, 0, NOMADTheme.GAP, NOMADTheme.GAP);
            return btn;
        }
    }
}
