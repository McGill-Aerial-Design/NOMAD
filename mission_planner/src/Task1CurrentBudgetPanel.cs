// ============================================================
// NOMAD Task 1 Current Budget Panel
// ============================================================
// Shows the target average current we can afford to draw during
// the lap phase and the actual battery current right now, so the
// pilot can slow down or speed up to stay on budget.
//
//     I_target = (capacity_Amin * safety - hover_A * other_time) / lap_time
//
// where other_time = total_mission - setup - laps (drop + target work).
// All inputs live in NOMAD Settings > Budget; this panel is a
// readout only.
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using Timer = System.Windows.Forms.Timer;

namespace NOMAD.MissionPlanner
{
    public class Task1CurrentBudgetPanel : UserControl
    {
        private static readonly Color CARD_BG        = NOMADTheme.CARD_BG;
        private static readonly Color TEXT_PRIMARY   = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color SUCCESS_COLOR  = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR  = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR    = NOMADTheme.ERROR;
        private static readonly Color ACCENT_COLOR   = NOMADTheme.ACCENT;

        private readonly NOMADConfig _config;

        private Label _lblTarget;
        private Label _lblActual;
        private Label _lblDelta;
        private Label _lblInputs;
        private Label _lblRemainder;
        private Label _lblFormula;

        private Timer _refreshTimer;
        private double _emaCurrent;
        private bool _haveCurrentSample;
        private double _currentTargetA = double.NaN;

        public Task1CurrentBudgetPanel(NOMADConfig config)
        {
            _config = config ?? new NOMADConfig();
            BackColor = CARD_BG;
            Dock = DockStyle.Fill;
            Padding = new Padding(8, 4, 8, 4);

            BuildUI();
            RecomputeTarget();

            _refreshTimer = new Timer { Interval = 500 };
            _refreshTimer.Tick += (s, e) =>
            {
                RecomputeTarget();
                RefreshActual();
            };
            this.HandleCreated += (s, e) => _refreshTimer.Start();
            this.Disposed += (s, e) =>
            {
                _refreshTimer?.Stop();
                _refreshTimer?.Dispose();
                _refreshTimer = null;
            };
        }

        private void BuildUI()
        {
            var title = new Label
            {
                Text = "CURRENT BUDGET (LAP PHASE)",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(8, 4),
                AutoSize = true,
            };
            Controls.Add(title);

            int y = 26;
            _lblTarget = new Label
            {
                Text = "Target: -- A",
                Font = new Font("Segoe UI", 13, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblTarget);

            _lblActual = new Label
            {
                Text = "Actual: -- A",
                Font = new Font("Segoe UI", 13, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(165, y),
                AutoSize = true,
            };
            Controls.Add(_lblActual);

            _lblDelta = new Label
            {
                Text = "Δ: --",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(330, y + 2),
                AutoSize = true,
            };
            Controls.Add(_lblDelta);

            y += 28;
            _lblInputs = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblInputs);

            y += 16;
            _lblRemainder = new Label
            {
                Text = "Drop+target window: -- min",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblRemainder);

            y += 16;
            _lblFormula = new Label
            {
                Text = "I = (cap·s − hover·(total−setup−laps)) / laps   ·   edit in Settings > Budget",
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblFormula);

            MinimumSize = new Size(420, y + 24);
        }

        private void RecomputeTarget()
        {
            double cap     = _config.Task1BudgetCapacityAmpMin;
            double safety  = _config.Task1BudgetSafetyFactor;
            double hoverA  = _config.Task1BudgetHoverCurrentA;
            double mission = _config.Task1BudgetMissionMin;
            double setup   = _config.Task1BudgetSetupMin;
            double lap     = _config.Task1BudgetLapMin;

            double other = mission - setup - lap;
            _lblInputs.Text = $"cap {cap:F0} A·min · s {safety:F2} · hover {hoverA:F1} A";
            _lblRemainder.Text = $"Drop+target window: {other:F1} min  (mission {mission:F1} − setup {setup:F1} − laps {lap:F1})";

            if (lap <= 0)
            {
                _currentTargetA = double.NaN;
                _lblTarget.Text = "Target: -- A";
                _lblTarget.ForeColor = ERROR_COLOR;
                UpdateDelta();
                return;
            }

            double target = (cap * safety - hoverA * other) / lap;
            _currentTargetA = target;
            _lblTarget.Text = $"Target: {target:F1} A";
            _lblTarget.ForeColor = other < 0 || target < hoverA ? WARNING_COLOR : ACCENT_COLOR;

            UpdateDelta();
        }

        private void RefreshActual()
        {
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null)
                {
                    _lblActual.Text = "Actual: link down";
                    _lblActual.ForeColor = TEXT_SECONDARY;
                    _haveCurrentSample = false;
                    UpdateDelta();
                    return;
                }

                double amps = ReadDoubleProperty(cs, "current");
                if (amps < 0) amps = 0;

                if (!_haveCurrentSample)
                {
                    _emaCurrent = amps;
                    _haveCurrentSample = true;
                }
                else
                {
                    _emaCurrent = 0.3 * amps + 0.7 * _emaCurrent;
                }

                _lblActual.Text = $"Actual: {_emaCurrent:F1} A";
                _lblActual.ForeColor = ColorForActual(_emaCurrent, _currentTargetA);
                UpdateDelta();
            }
            catch
            {
                _lblActual.Text = "Actual: --";
                _lblActual.ForeColor = TEXT_SECONDARY;
            }
        }

        private void UpdateDelta()
        {
            if (!_haveCurrentSample || double.IsNaN(_currentTargetA))
            {
                _lblDelta.Text = "Δ: --";
                _lblDelta.ForeColor = TEXT_SECONDARY;
                return;
            }
            double delta = _emaCurrent - _currentTargetA;
            string sign = delta >= 0 ? "+" : "";
            _lblDelta.Text = $"Δ: {sign}{delta:F1} A";
            _lblDelta.ForeColor = ColorForActual(_emaCurrent, _currentTargetA);
        }

        private static Color ColorForActual(double actual, double target)
        {
            if (double.IsNaN(target) || target <= 0) return TEXT_PRIMARY;
            double ratio = actual / target;
            if (ratio <= 0.95) return SUCCESS_COLOR;
            if (ratio <= 1.10) return WARNING_COLOR;
            return ERROR_COLOR;
        }

        private static double ReadDoubleProperty(object obj, string name)
        {
            try
            {
                var prop = obj.GetType().GetProperty(name);
                if (prop == null) return 0.0;
                var v = prop.GetValue(obj, null);
                return v == null ? 0.0 : Convert.ToDouble(v);
            }
            catch { return 0.0; }
        }
    }
}
