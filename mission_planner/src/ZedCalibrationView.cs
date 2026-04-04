// ============================================================
// NOMAD ZED Calibration View
// ============================================================
// Calibration workflow is performed using the official
// ZED Sensor Viewer over remote desktop (noVNC).
// ============================================================

using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public class ZedCalibrationView : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;

        private TabControl _tabControl;
        private Button _btnLaunchSensorViewer;
        private Button _btnLaunchZedCalibration;
        private Button _btnOpenRemoteDesktop;
        private Label _lblStatus;

        public ZedCalibrationView(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }

        private void InitializeUI()
        {
            this.AutoScroll = false;

            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
                Font = new Font("Segoe UI", 10),
                DrawMode = TabDrawMode.OwnerDrawFixed,
            };
            _tabControl.DrawItem += TabControl_DrawItem;

            var tab = new TabPage("ZED Sensor Viewer")
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(15),
            };

            BuildRemoteCalibrationTab(tab);
            _tabControl.TabPages.Add(tab);

            this.Controls.Add(_tabControl);
        }

        private void TabControl_DrawItem(object sender, DrawItemEventArgs e)
        {
            var tab = _tabControl.TabPages[e.Index];
            var bounds = _tabControl.GetTabRect(e.Index);
            bool selected = (_tabControl.SelectedIndex == e.Index);

            using (var bgBrush = new SolidBrush(selected ? NOMADTheme.CARD_BG : NOMADTheme.BG_DARK))
            {
                e.Graphics.FillRectangle(bgBrush, bounds);
            }

            var textColor = selected ? NOMADTheme.ACCENT : NOMADTheme.TEXT_SECONDARY;
            TextRenderer.DrawText(
                e.Graphics,
                tab.Text,
                _tabControl.Font,
                bounds,
                textColor,
                TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter
            );
        }

        private void BuildRemoteCalibrationTab(TabPage tab)
        {
            var panel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                Padding = new Padding(20),
            };
            panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));
            panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 150));
            panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));
            panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            var title = new Label
            {
                Text = "ZED CALIBRATION (OFFICIAL WORKFLOW)",
                Font = new Font("Segoe UI", 13, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            panel.Controls.Add(title, 0, 0);

            var instructions = new Label
            {
                Text =
                    "Custom in-app calibration pages were removed.\n" +
                    "Use the official ZED Sensor Viewer in remote desktop:\n\n" +
                    "1. Click 'Launch Sensor Viewer'\n" +
                    "2. (Optional) Click 'Launch ZED Calibration'\n" +
                    "3. noVNC opens in your default browser\n" +
                    "4. Run calibration in the selected ZED tool",
                Font = new Font("Segoe UI", 10),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Dock = DockStyle.Fill,
            };
            panel.Controls.Add(instructions, 0, 1);

            var buttonRow = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = false,
            };

            _btnLaunchSensorViewer = new Button
            {
                Text = "Launch Sensor Viewer",
                Size = new Size(200, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Margin = new Padding(0, 10, 12, 0),
            };
            _btnLaunchSensorViewer.FlatAppearance.BorderSize = 0;
            _btnLaunchSensorViewer.Click += async (s, e) => await LaunchSensorViewerAsync();
            buttonRow.Controls.Add(_btnLaunchSensorViewer);

            _btnLaunchZedCalibration = new Button
            {
                Text = "Launch ZED Calibration",
                Size = new Size(220, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 120, 95),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Margin = new Padding(0, 10, 12, 0),
            };
            _btnLaunchZedCalibration.FlatAppearance.BorderSize = 0;
            _btnLaunchZedCalibration.Click += async (s, e) => await LaunchZedCalibrationAsync();
            buttonRow.Controls.Add(_btnLaunchZedCalibration);

            _btnOpenRemoteDesktop = new Button
            {
                Text = "Open Remote Desktop",
                Size = new Size(200, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 90, 140),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Margin = new Padding(0, 10, 0, 0),
            };
            _btnOpenRemoteDesktop.FlatAppearance.BorderSize = 0;
            _btnOpenRemoteDesktop.Click += (s, e) => OpenNoVncInBrowser(BuildNoVncUrl());
            buttonRow.Controls.Add(_btnOpenRemoteDesktop);

            panel.Controls.Add(buttonRow, 0, 2);

            _lblStatus = new Label
            {
                Text = "Ready",
                Font = new Font("Segoe UI", 10),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Dock = DockStyle.Top,
                Height = 30,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            panel.Controls.Add(_lblStatus, 0, 3);

            tab.Controls.Add(panel);
        }

        private string BuildNoVncUrl()
        {
            string host = _config.UseTailscale ? _config.TailscaleIP : _config.JetsonIP;
            if (string.IsNullOrWhiteSpace(host))
                host = "100.85.121.98";

            return $"http://{host}:6080/vnc.html?autoconnect=0&reconnect=0&resize=scale";
        }

        private void OpenNoVncInBrowser(string url)
        {
            try
            {
                var psi = new System.Diagnostics.ProcessStartInfo
                {
                    FileName = url,
                    UseShellExecute = true,
                };
                System.Diagnostics.Process.Start(psi);
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    $"Unable to open remote desktop:\n\n{ex.Message}",
                    "Remote Desktop Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
        }

        private async Task LaunchSensorViewerAsync()
        {
            _btnLaunchSensorViewer.Enabled = false;
            _btnLaunchZedCalibration.Enabled = false;
            _lblStatus.Text = "Launching Sensor Viewer...";
            _lblStatus.ForeColor = NOMADTheme.WARNING;

            try
            {
                var resp = await JetsonApiService.PostAsync("/api/calibration/zed/sensor-viewer/start");
                var body = await resp.Content.ReadAsStringAsync();

                if (!resp.IsSuccessStatusCode)
                {
                    _lblStatus.Text = "Launch failed";
                    _lblStatus.ForeColor = NOMADTheme.ERROR;
                    MessageBox.Show(
                        $"Failed to launch ZED Sensor Viewer:\n\n{body}",
                        "Calibration Tool Error",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error
                    );
                    return;
                }

                string message = "ZED Sensor Viewer launched";
                string noVncUrl = BuildNoVncUrl();
                try
                {
                    var data = JObject.Parse(body);
                    var apiMessage = data["message"]?.Value<string>();
                    var apiNoVnc = data["novnc_url"]?.Value<string>();
                    if (!string.IsNullOrWhiteSpace(apiMessage))
                        message = apiMessage;
                    if (!string.IsNullOrWhiteSpace(apiNoVnc))
                        noVncUrl = apiNoVnc;
                }
                catch
                {
                    // Keep fallback values.
                }

                OpenNoVncInBrowser(noVncUrl);
                _lblStatus.Text = message;
                _lblStatus.ForeColor = NOMADTheme.SUCCESS;
            }
            catch (Exception ex)
            {
                _lblStatus.Text = "Launch failed";
                _lblStatus.ForeColor = NOMADTheme.ERROR;
                MessageBox.Show(
                    $"Unable to launch Sensor Viewer:\n\n{ex.Message}",
                    "Calibration Tool Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnLaunchSensorViewer.Enabled = true;
                _btnLaunchZedCalibration.Enabled = true;
            }
        }

        private async Task LaunchZedCalibrationAsync()
        {
            _btnLaunchSensorViewer.Enabled = false;
            _btnLaunchZedCalibration.Enabled = false;
            _lblStatus.Text = "Launching ZED Calibration...";
            _lblStatus.ForeColor = NOMADTheme.WARNING;

            try
            {
                var resp = await JetsonApiService.PostAsync("/api/calibration/zed/calibration/start");
                var body = await resp.Content.ReadAsStringAsync();

                if (!resp.IsSuccessStatusCode)
                {
                    _lblStatus.Text = "Launch failed";
                    _lblStatus.ForeColor = NOMADTheme.ERROR;
                    MessageBox.Show(
                        $"Failed to launch ZED Calibration:\n\n{body}",
                        "Calibration Tool Error",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error
                    );
                    return;
                }

                string message = "ZED Calibration launched";
                string noVncUrl = BuildNoVncUrl();
                try
                {
                    var data = JObject.Parse(body);
                    var apiMessage = data["message"]?.Value<string>();
                    var apiNoVnc = data["novnc_url"]?.Value<string>();
                    if (!string.IsNullOrWhiteSpace(apiMessage))
                        message = apiMessage;
                    if (!string.IsNullOrWhiteSpace(apiNoVnc))
                        noVncUrl = apiNoVnc;
                }
                catch
                {
                    // Keep fallback values.
                }

                OpenNoVncInBrowser(noVncUrl);
                _lblStatus.Text = message;
                _lblStatus.ForeColor = NOMADTheme.SUCCESS;
            }
            catch (Exception ex)
            {
                _lblStatus.Text = "Launch failed";
                _lblStatus.ForeColor = NOMADTheme.ERROR;
                MessageBox.Show(
                    $"Unable to launch ZED Calibration:\n\n{ex.Message}",
                    "Calibration Tool Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnLaunchSensorViewer.Enabled = true;
                _btnLaunchZedCalibration.Enabled = true;
            }
        }

        public void UpdateData()
        {
            // No periodic updates required for this view.
        }
    }
}
