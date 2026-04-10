// ============================================================
// NOMAD RViz2 Remote View
// ============================================================
// Launches RViz2 on the Jetson and opens noVNC in the browser.
// ============================================================

using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public class Rviz2View : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;

        private TabControl _tabControl;
        private Button _btnLaunchRviz2;
        private Button _btnOpenRemoteDesktop;
        private Label _lblStatus;

        public Rviz2View(NOMADConfig config)
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

            var tab = new TabPage("RViz2")
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(15),
            };

            BuildRviz2Tab(tab);
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

        private void BuildRviz2Tab(TabPage tab)
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
                Text = "RVIZ2 REMOTE VIEWER",
                Font = new Font("Segoe UI", 13, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            panel.Controls.Add(title, 0, 0);

            var instructions = new Label
            {
                Text =
                    "Run RViz2 on the Jetson desktop and view it through noVNC:\n\n" +
                    "1. Click 'Launch RViz2'\n" +
                    "2. noVNC opens in your default browser\n" +
                    "3. Use the remote desktop session to work in RViz2",
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

            _btnLaunchRviz2 = new Button
            {
                Text = "Launch RViz2",
                Size = new Size(200, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Margin = new Padding(0, 10, 12, 0),
            };
            _btnLaunchRviz2.FlatAppearance.BorderSize = 0;
            _btnLaunchRviz2.Click += async (s, e) => await LaunchRviz2Async();
            buttonRow.Controls.Add(_btnLaunchRviz2);

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

        private async Task LaunchRviz2Async()
        {
            _btnLaunchRviz2.Enabled = false;
            _lblStatus.Text = "Launching RViz2...";
            _lblStatus.ForeColor = NOMADTheme.WARNING;

            try
            {
                var resp = await JetsonApiService.PostLongRunAsync("/api/tools/rviz2/start");
                var body = await resp.Content.ReadAsStringAsync();

                if (!resp.IsSuccessStatusCode)
                {
                    _lblStatus.Text = "Launch failed";
                    _lblStatus.ForeColor = NOMADTheme.ERROR;
                    MessageBox.Show(
                        $"Failed to launch RViz2:\n\n{body}",
                        "RViz2 Tool Error",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error
                    );
                    return;
                }

                string message = "RViz2 launched";
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
                    $"Unable to launch RViz2:\n\n{ex.Message}",
                    "RViz2 Tool Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnLaunchRviz2.Enabled = true;
            }
        }

        public void UpdateData()
        {
            // No periodic updates required for this view.
        }
    }
}
