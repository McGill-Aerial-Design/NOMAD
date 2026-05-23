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
// ============================================================

using System;
using System.Drawing;
using System.Globalization;
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

        // Defaults tuned for our 23Ah pack (= 1380 amp-minutes) and the
        // measured ~20A hover. Operator can override on the fly.
        private const double DefaultCapacityAmpMin = 1380.0;
        private const double DefaultSafetyFactor   = 0.80;
        private const double DefaultHoverCurrentA  = 20.0;
        private const double DefaultMissionMin     = 30.0;
        private const double DefaultSetupMin       = 8.0;
        private const double DefaultLapMin         = 12.0;

        private NumericUpDown _numCapacity;
        private NumericUpDown _numSafety;
        private NumericUpDown _numHover;
        private NumericUpDown _numMission;
        private NumericUpDown _numSetup;
        private NumericUpDown _numLap;

        private Label _lblTarget;
        private Label _lblActual;
        private Label _lblDelta;
        private Label _lblRemainder;
        private Label _lblFormula;

        private Timer _refreshTimer;
        private double _emaCurrent;          // smoothed actual amps
        private bool _haveCurrentSample;

        public Task1CurrentBudgetPanel()
        {
            BackColor = CARD_BG;
            Dock = DockStyle.Fill;
            Padding = new Padding(8, 4, 8, 4);

            BuildUI();
            RecomputeTarget();

            _refreshTimer = new Timer { Interval = 500 };
            _refreshTimer.Tick += (s, e) => RefreshActual();
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

            // ----- Inputs row 1: capacity, safety, hover -----
            int y = 26;
            _numCapacity = AddNumeric("Capacity (A·min)", 8,   y, (decimal)DefaultCapacityAmpMin, 0,   5000, 10m);
            _numSafety   = AddNumeric("Safety",            175, y, (decimal)DefaultSafetyFactor,  0.1m, 1.0m, 0.05m, decimals: 2);
            _numHover    = AddNumeric("Hover (A)",         300, y, (decimal)DefaultHoverCurrentA, 1,   80,   0.5m, decimals: 1);

            // ----- Inputs row 2: timing -----
            y += 42;
            _numMission = AddNumeric("Total (min)", 8,   y, (decimal)DefaultMissionMin, 5, 90, 0.5m, decimals: 1);
            _numSetup   = AddNumeric("Setup (min)", 175, y, (decimal)DefaultSetupMin,   0, 60, 0.5m, decimals: 1);
            _numLap     = AddNumeric("Laps (min)",  300, y, (decimal)DefaultLapMin,     1, 60, 0.5m, decimals: 1);

            foreach (var nud in new[] { _numCapacity, _numSafety, _numHover, _numMission, _numSetup, _numLap })
                nud.ValueChanged += (s, e) => RecomputeTarget();

            // ----- Big readout row -----
            y += 46;
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

            // ----- Detail row -----
            y += 28;
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
                Text = "I = (cap·s − hover·(total−setup−laps)) / laps",
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblFormula);

            MinimumSize = new Size(420, y + 24);
        }

        private NumericUpDown AddNumeric(string label, int x, int y, decimal value,
            decimal min, decimal max, decimal step, int decimals = 0)
        {
            Controls.Add(new Label
            {
                Text = label,
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y),
                AutoSize = true,
            });
            var nud = new NumericUpDown
            {
                Location = new Point(x, y + 14),
                Size = new Size(115, 22),
                Minimum = min,
                Maximum = max,
                Increment = step,
                DecimalPlaces = decimals,
                Value = Math.Max(min, Math.Min(max, value)),
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
            };
            Controls.Add(nud);
            return nud;
        }

        private void RecomputeTarget()
        {
            double cap     = (double)_numCapacity.Value;
            double safety  = (double)_numSafety.Value;
            double hoverA  = (double)_numHover.Value;
            double mission = (double)_numMission.Value;
            double setup   = (double)_numSetup.Value;
            double lap     = (double)_numLap.Value;

            double other = mission - setup - lap;
            _lblRemainder.Text = $"Drop+target window: {other:F1} min  (mission {mission:F1} − setup {setup:F1} − laps {lap:F1})";

            if (lap <= 0)
            {
                _lblTarget.Text = "Target: -- A";
                _lblTarget.ForeColor = ERROR_COLOR;
                return;
            }

            double target = (cap * safety - hoverA * other) / lap;
            _lblTarget.Text = $"Target: {target:F1} A";
            _lblTarget.ForeColor = other < 0 || target < hoverA ? WARNING_COLOR : ACCENT_COLOR;

            UpdateDelta(target);
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
                    UpdateDelta((double)_numHover.Value); // no Δ until data
                    return;
                }

                double amps = ReadDoubleProperty(cs, "current");
                // Cube reports negative when ESCs regen; clamp to 0 for display.
                if (amps < 0) amps = 0;

                // 0.3 EMA — about 2 s settling at 500 ms tick. Smooths the
                // jitter without hiding a real acceleration spike.
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

                double target = ParseTargetAmps();
                _lblActual.ForeColor = ColorForActual(_emaCurrent, target);
                UpdateDelta(target);
            }
            catch
            {
                _lblActual.Text = "Actual: --";
                _lblActual.ForeColor = TEXT_SECONDARY;
            }
        }

        private double ParseTargetAmps()
        {
            // _lblTarget.Text format: "Target: 25.3 A"
            var t = _lblTarget.Text;
            int colon = t.IndexOf(':');
            if (colon < 0) return double.NaN;
            var s = t.Substring(colon + 1).Trim().TrimEnd('A').Trim();
            if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
                return v;
            return double.NaN;
        }

        private void UpdateDelta(double target)
        {
            if (!_haveCurrentSample || double.IsNaN(target))
            {
                _lblDelta.Text = "Δ: --";
                _lblDelta.ForeColor = TEXT_SECONDARY;
                return;
            }
            double delta = _emaCurrent - target;
            string sign = delta >= 0 ? "+" : "";
            _lblDelta.Text = $"Δ: {sign}{delta:F1} A";
            _lblDelta.ForeColor = ColorForActual(_emaCurrent, target);
        }

        private static Color ColorForActual(double actual, double target)
        {
            if (double.IsNaN(target) || target <= 0) return TEXT_PRIMARY;
            double ratio = actual / target;
            if (ratio <= 0.95) return SUCCESS_COLOR;          // under budget — can go faster
            if (ratio <= 1.10) return WARNING_COLOR;          // close to budget
            return ERROR_COLOR;                                // over budget — slow down
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
