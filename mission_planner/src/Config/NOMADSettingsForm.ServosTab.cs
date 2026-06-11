// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Linq;
using System.Net.Http;
using System.Text;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateServosTab()
        {
            var tab = CreateTabPage("Servos");
            int y = 10;

            AddLabel(tab, "Channel = ArduPilot servo output number (e.g. 9 = AUX1).", 10, y, Color.Gray);
            y += 18;

            void AddServoRow(string label, ref int row,
                out NumericUpDown ch, out NumericUpDown min, out NumericUpDown max,
                int defCh, int defMin, int defMax)
            {
                AddLabel(tab, label, 10, row);
                AddLabel(tab, "Ch", 155, row, Color.Gray);
                ch  = AddNumericUpDown(tab, 175, row, 45, 1, 99, defCh);
                AddLabel(tab, "Min", 228, row, Color.Gray);
                min = AddNumericUpDown(tab, 248, row, 55, 500, 2500, defMin);
                AddLabel(tab, "Max", 310, row, Color.Gray);
                max = AddNumericUpDown(tab, 330, row, 55, 500, 2500, defMax);
                row += 28;
            }

            AddLabel(tab, "Drop / slider / relay payloads are configured in the Payloads tab.", 10, y, Color.Gray);
            y += 22;

            AddSectionLabel(tab, "Strap Reel (Payload 1)", ref y);
            AddLabel(tab, "Channel:", 10, y);
            _numReelCh = AddNumericUpDown(tab, 100, y, 50, 1, 99, 12);
            y += 28;
            AddLabel(tab, "PWM In (>2000):", 10, y);
            _numReelPwmIn  = AddNumericUpDown(tab, 145, y, 60, 2001, 2500, 2100);
            y += 28;
            AddLabel(tab, "PWM Out (<1000):", 10, y);
            _numReelPwmOut = AddNumericUpDown(tab, 145, y, 60, 500, 999, 900);
            y += 32;

            AddSectionLabel(tab, "Strap Reel (Payload 2)", ref y);
            AddLabel(tab, "Channel:", 10, y);
            _numReel2Ch = AddNumericUpDown(tab, 100, y, 50, 1, 99, 13);
            y += 28;
            AddLabel(tab, "PWM In (>2000):", 10, y);
            _numReel2PwmIn  = AddNumericUpDown(tab, 145, y, 60, 2001, 2500, 2100);
            y += 28;
            AddLabel(tab, "PWM Out (<1000):", 10, y);
            _numReel2PwmOut = AddNumericUpDown(tab, 145, y, 60, 500, 999, 900);
            y += 32;

            AddSectionLabel(tab, "Camera Tilt (MAVLink primary, API fallback)", ref y);
            AddServoRow("Camera Tilt:", ref y, out _numTiltCh, out _numTiltPwmMin, out _numTiltPwmMax, 14, 700, 1450);
            AddLabel(tab, "Neutral PWM:", 10, y);
            _numTiltPwmNeutral = AddNumericUpDown(tab, 120, y, 60, 500, 2500, 1250);
            AddLabel(tab, "Range (deg):", 190, y, Color.Gray);
            _numTiltAngleRange = AddNumericUpDown(tab, 258, y, 50, 1, 90, 45);
            y += 28;

            return tab;
        }

        private TabPage CreateSprayCalibrationTab()
        {
            var tab = CreateTabPage("Spray");
            int y = 10;

            AddSectionLabel(tab, "Fixed Firing Geometry", ref y);
            AddLabel(tab, "Camera range (m):", 10, y);
            _numSprayRange = AddNumericUpDown(tab, 145, y, 70, 0.5m, 8.0m, 3.8m, 2);
            AddLabel(tab, "Tolerance (m):", 230, y, Color.Gray);
            _numSprayRangeTol = AddNumericUpDown(tab, 330, y, 60, 0.05m, 1.0m, 0.25m, 2);
            y += 28;

            AddLabel(tab, "Start max range (m):", 10, y);
            _numSprayTriggerMax = AddNumericUpDown(tab, 145, y, 70, 1.0m, 8.0m, 5.5m, 1);
            AddLabel(tab, "Operator can trigger before final approach.", 230, y, Color.Gray);
            y += 28;

            AddLabel(tab, "Aim pixel X:", 10, y);
            _numSprayAimX = AddNumericUpDown(tab, 145, y, 70, 0, 4000, 640);
            AddLabel(tab, "Y:", 230, y, Color.Gray);
            _numSprayAimY = AddNumericUpDown(tab, 260, y, 70, 0, 3000, 390);
            AddLabel(tab, "Tol px:", 345, y, Color.Gray);
            _numSprayAimTol = AddNumericUpDown(tab, 405, y, 55, 2, 250, 25);
            y += 28;

            AddLabel(tab, "Nozzle fire angle:", 10, y);
            _numSprayServoAngle = AddNumericUpDown(tab, 145, y, 70, 0, 180, 82, 1);
            AddLabel(tab, "Pump duration uses Servos tab value.", 230, y, Color.Gray);
            y += 36;

            AddSectionLabel(tab, "Alignment Gains", ref y);
            _chkSprayUseYaw = AddCheckBox(tab, "Use yaw for horizontal pixel alignment", 10, y);
            y += 28;
            AddLabel(tab, "Forward:", 10, y);
            _numSprayForwardGain = AddNumericUpDown(tab, 80, y, 70, 0, 2, 0.45m, 3);
            AddLabel(tab, "Yaw:", 170, y, Color.Gray);
            _numSprayYawGain = AddNumericUpDown(tab, 215, y, 75, -0.02m, 0.02m, 0.0025m, 4);
            y += 28;
            AddLabel(tab, "Lateral:", 10, y);
            _numSprayLateralGain = AddNumericUpDown(tab, 80, y, 75, -0.02m, 0.02m, 0.001m, 4);
            AddLabel(tab, "Altitude:", 170, y, Color.Gray);
            _numSprayAltitudeGain = AddNumericUpDown(tab, 240, y, 75, -0.02m, 0.02m, 0.001m, 4);
            y += 36;

            AddSectionLabel(tab, "Limits and Lock", ref y);
            AddLabel(tab, "Max fwd:", 10, y);
            _numSprayMaxForward = AddNumericUpDown(tab, 80, y, 60, 0.05m, 2, 0.45m, 2);
            AddLabel(tab, "lat:", 155, y, Color.Gray);
            _numSprayMaxLateral = AddNumericUpDown(tab, 190, y, 60, 0.05m, 1, 0.25m, 2);
            AddLabel(tab, "alt:", 265, y, Color.Gray);
            _numSprayMaxAltitude = AddNumericUpDown(tab, 300, y, 60, 0.05m, 1, 0.20m, 2);
            y += 28;
            AddLabel(tab, "Max yaw:", 10, y);
            _numSprayMaxYaw = AddNumericUpDown(tab, 80, y, 60, 0.05m, 2, 0.35m, 2);
            AddLabel(tab, "Lock ms:", 155, y, Color.Gray);
            _numSprayLockMs = AddNumericUpDown(tab, 220, y, 70, 100, 5000, 700);
            AddLabel(tab, "Timeout s:", 305, y, Color.Gray);
            _numSprayTimeout = AddNumericUpDown(tab, 385, y, 55, 2, 60, 20, 1);
            y += 40;

            _btnPushSprayCalibration = new Button
            {
                Text = "Push to Jetson",
                Location = new Point(10, y),
                Size = new Size(130, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
            };
            _btnPushSprayCalibration.Click += async (s, e) => await PushSprayCalibration();
            tab.Controls.Add(_btnPushSprayCalibration);

            _lblSprayCalibrationStatus = new Label
            {
                Text = "",
                Location = new Point(150, y + 6),
                AutoSize = true,
                ForeColor = Color.Gray,
            };
            tab.Controls.Add(_lblSprayCalibrationStatus);

            return tab;
        }

        // RC pass-through option codes for ArduPilot relay numbers (RELAY1..4).
        internal static int RelayRcOptionCode(int relayNumber)
        {
            switch (relayNumber)
            {
                case 0: return 28;
                case 1: return 34;
                case 2: return 35;
                case 3: return 36;
                default: return 0;
            }
        }

        private static bool TrySetParamReflect(object comPort, string name, double value)
        {
            try
            {
                var methods = comPort.GetType()
                    .GetMethods(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance)
                    .Where(m => m.Name == "setParam").ToList();
                foreach (var m in methods)
                {
                    var p = m.GetParameters();
                    if (p.Length >= 2 && p[0].ParameterType == typeof(string))
                    {
                        var args = new object[p.Length];
                        args[0] = name;
                        args[1] = Convert.ChangeType(value, p[1].ParameterType);
                        for (int i = 2; i < p.Length; i++)
                        {
                            if (p[i].HasDefaultValue) args[i] = p[i].DefaultValue;
                            else if (p[i].ParameterType == typeof(bool)) args[i] = true;
                            else args[i] = p[i].ParameterType.IsValueType
                                ? Activator.CreateInstance(p[i].ParameterType) : null;
                        }
                        try { m.Invoke(comPort, args); return true; }
                        catch { }
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error($"setParam({name}) error - {ex.Message}");
            }
            return false;
        }

        // The server consumes only the relay number; every other spray
        // setting is GCS-local. The API rejects unknown fields (extra=forbid)
        // so this payload cannot drift silently.
        private JObject BuildSprayCalibrationPayload()
        {
            return new JObject
            {
                ["water_pump_relay_number"] = Config.WaterPump()?.Channel ?? 0,
            };
        }

        private async System.Threading.Tasks.Task PushSprayCalibration()
        {
            try
            {
                _btnPushSprayCalibration.Enabled = false;
                _lblSprayCalibrationStatus.Text = "Pushing...";
                _lblSprayCalibrationStatus.ForeColor = Color.Yellow;

                SaveSettings();
                Config.Save();
                JetsonApiService.Reconfigure(Config);

                var json = BuildSprayCalibrationPayload().ToString(Newtonsoft.Json.Formatting.None);
                var content = new StringContent(json, Encoding.UTF8, "application/json");
                var response = await JetsonApiService.PostAsync("/api/spray/calibration", content);

                if (response.IsSuccessStatusCode)
                {
                    _lblSprayCalibrationStatus.Text = "Spray calibration pushed";
                    _lblSprayCalibrationStatus.ForeColor = Color.LimeGreen;
                }
                else
                {
                    var body = await response.Content.ReadAsStringAsync();
                    string detail = body;
                    try
                    {
                        var err = JObject.Parse(body);
                        detail = err["detail"]?.ToString() ?? body;
                    }
                    catch { }
                    _lblSprayCalibrationStatus.Text = $"Failed: {detail}";
                    _lblSprayCalibrationStatus.ForeColor = Color.Red;
                }
            }
            catch (Exception ex)
            {
                _lblSprayCalibrationStatus.Text = $"Error: {ex.Message}";
                _lblSprayCalibrationStatus.ForeColor = Color.Red;
            }
            finally
            {
                _btnPushSprayCalibration.Enabled = true;
            }
        }
    }
}
