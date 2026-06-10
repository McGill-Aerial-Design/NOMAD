// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Payload Control Panel - Dynamic payload rows
// ============================================================
// Renders the configurable NOMADConfig.Payloads list:
//   Drop   - three-click-armed drop / retract servo button.
//   Slider - live PWM slider for an aiming / nozzle servo.
//   Relay  - GPIO / relay output: momentary "Fire" pulse or latching toggle.
// Drop state is shared across panel instances (and read by the joystick service).
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using Timer = System.Windows.Forms.Timer;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    public partial class PayloadControlPanel
    {
        // Drop button base color and armed-state colors (1 and 2 clicks in)
        private static readonly Color DROP_COLOR_IDLE    = Color.FromArgb(120, 50, 15);
        private static readonly Color DROP_COLOR_ARM1    = Color.FromArgb(200, 110, 0);
        private static readonly Color DROP_COLOR_ARM2    = Color.FromArgb(220, 50,  0);
        private static readonly Color DROP_COLOR_DROPPED = Color.FromArgb(50, 90, 130);
        private static readonly Color RELAY_COLOR        = Color.FromArgb(30, 100, 180);
        private static readonly Color RELAY_ON_COLOR     = Color.FromArgb(60, 150, 60);

        private const int DROP_CLICKS_REQUIRED = 3;
        private const int DROP_RESET_MS        = 3000;

        // Per drop-index (0-based, in panel order) UI + arming state.
        private readonly Dictionary<int, Button>          _dropButtons     = new Dictionary<int, Button>();
        private readonly Dictionary<int, PayloadControl>  _dropPayloads    = new Dictionary<int, PayloadControl>();
        private readonly Dictionary<int, int>             _dropClickCount  = new Dictionary<int, int>();
        private readonly Dictionary<int, Timer>           _dropResetTimers = new Dictionary<int, Timer>();
        private readonly Dictionary<int, bool>            _dropDropped     = new Dictionary<int, bool>();

        // Per slider-index settle timers (final send after the slider stops moving).
        private readonly Dictionary<int, Timer> _sliderSettleTimers = new Dictionary<int, Timer>();

        // Latching-relay on/off state, keyed by relay number.
        private readonly Dictionary<int, bool> _relayOn = new Dictionary<int, bool>();

        // Momentary-relay fire arming (SR-PAY-03): first click arms (with a
        // timeout reset, like drops), second click fires. Keyed by channel.
        private readonly Dictionary<int, bool>  _relayFireArmed       = new Dictionary<int, bool>();
        private readonly Dictionary<int, Timer> _relayFireResetTimers = new Dictionary<int, Timer>();

        // ============================================================
        // Cross-panel drop-state sync (also read by NomadJoystickService)
        // ============================================================

        public static event Action<int, bool> PayloadDroppedStateChanged;

        private static readonly bool[] s_payloadDropped = new bool[NOMADConfig.MaxPayloads];

        /// <summary>True if drop payload <paramref name="dropIdx0"/> (0-based) is currently dropped.</summary>
        public static bool IsPayloadDropped(int dropIdx0)
            => dropIdx0 >= 0 && dropIdx0 < s_payloadDropped.Length && s_payloadDropped[dropIdx0];

        public static void RaisePayloadDroppedState(int dropIdx0, bool isDropped)
        {
            if (dropIdx0 < 0 || dropIdx0 >= s_payloadDropped.Length) return;
            s_payloadDropped[dropIdx0] = isDropped;
            PayloadDroppedStateChanged?.Invoke(dropIdx0, isDropped);
        }

        // ============================================================
        // Dynamic row construction
        // ============================================================

        private void BuildPayloadRows(ref int y)
        {
            var payloads = _config.EnabledPayloads();
            if (payloads.Count == 0)
            {
                Controls.Add(new Label
                {
                    Text = "No payloads configured — add some in Settings → Payloads.",
                    Font = new Font("Segoe UI", 8, FontStyle.Italic),
                    ForeColor = TEXT_SECONDARY,
                    Location = new Point(10, y + 2),
                    AutoSize = true,
                });
                y += ROW_H + ROW_GAP;
                return;
            }

            int dropIndex = 0, sliderIndex = 0;
            foreach (var p in payloads)
            {
                switch (p.Kind)
                {
                    case PayloadKind.Drop:   BuildDropRow(p, dropIndex++, ref y);   break;
                    case PayloadKind.Slider: BuildSliderRow(p, sliderIndex++, ref y); break;
                    case PayloadKind.Relay:  BuildRelayRow(p, ref y);               break;
                }
            }
        }

        private Label RowLabel(string text, int y) => new Label
        {
            Text = text,
            Font = new Font("Segoe UI", 9),
            ForeColor = TEXT_SECONDARY,
            Location = new Point(10, y + 4),
            AutoSize = true,
        };

        // ---- Drop ----

        private void BuildDropRow(PayloadControl p, int dropIdx, ref int y)
        {
            var btn = MakeButton($"Drop {p.Name}", DROP_COLOR_IDLE, 160, ROW_H);
            btn.Location = new Point(10, y);
            btn.Click += (s, e) => OnDropClick(dropIdx);
            Controls.Add(btn);

            _dropButtons[dropIdx]    = btn;
            _dropPayloads[dropIdx]   = p;
            _dropClickCount[dropIdx] = 0;
            _dropDropped[dropIdx]    = IsPayloadDropped(dropIdx);

            y += ROW_H + ROW_GAP;
        }

        private void OnDropClick(int dropIdx)
        {
            if (_dropDropped.TryGetValue(dropIdx, out bool dropped) && dropped)
            {
                ExecuteRetract(dropIdx);
                return;
            }

            int count = _dropClickCount.TryGetValue(dropIdx, out int c) ? c + 1 : 1;
            _dropClickCount[dropIdx] = count;

            RestartDropResetTimer(dropIdx);

            if (count < DROP_CLICKS_REQUIRED)
            {
                int remaining = DROP_CLICKS_REQUIRED - count;
                if (_dropButtons.TryGetValue(dropIdx, out var b))
                    b.BackColor = count == 1 ? DROP_COLOR_ARM1 : DROP_COLOR_ARM2;
                SetStatus($"{DropName(dropIdx)}: {remaining} more click{(remaining == 1 ? "" : "s")} to drop!", WARNING_COLOR);
                return;
            }

            ClearDropResetTimer(dropIdx);
            _dropClickCount[dropIdx] = 0;
            if (_dropButtons.TryGetValue(dropIdx, out var btn)) btn.BackColor = DROP_COLOR_IDLE;

            ExecuteDrop(dropIdx);
        }

        private void RestartDropResetTimer(int dropIdx)
        {
            ClearDropResetTimer(dropIdx);
            var t = new Timer { Interval = DROP_RESET_MS };
            t.Tick += (s, e) =>
            {
                ClearDropResetTimer(dropIdx);
                _dropClickCount[dropIdx] = 0;
                if (!IsDisposed && _dropButtons.TryGetValue(dropIdx, out var b) && b != null)
                    b.BackColor = DROP_COLOR_IDLE;
                SetStatus($"{DropName(dropIdx)} drop cancelled (timeout)", TEXT_SECONDARY);
            };
            _dropResetTimers[dropIdx] = t;
            t.Start();
        }

        private void ClearDropResetTimer(int dropIdx)
        {
            if (_dropResetTimers.TryGetValue(dropIdx, out var t) && t != null)
            {
                t.Stop();
                t.Dispose();
            }
            _dropResetTimers.Remove(dropIdx);
        }

        private async void ExecuteDrop(int dropIdx)
        {
            if (!_dropPayloads.TryGetValue(dropIdx, out var p) || p == null) return;
            if (p.Channel <= 0)
            {
                SetStatus($"{p.Name} channel not configured (Settings → Payloads)", WARNING_COLOR);
                return;
            }

            int pwmDrop = p.Reversed ? p.PwmMin : p.PwmMax;
            if (await CubeOutputController.SendServoPwmAsync(p.Channel, pwmDrop))
            {
                SetStatus($"{p.Name} dropped  (Cube ch{p.Channel} {pwmDrop}us)", SUCCESS_COLOR);
                RaisePayloadDroppedState(dropIdx, true);
            }
            else
            {
                SetStatus("Drop failed: Cube output command unavailable", ERROR_COLOR);
            }
        }

        private async void ExecuteRetract(int dropIdx)
        {
            if (!_dropPayloads.TryGetValue(dropIdx, out var p) || p == null) return;
            if (p.Channel <= 0)
            {
                SetStatus($"{p.Name} channel not configured (Settings → Payloads)", WARNING_COLOR);
                return;
            }

            RaisePayloadDroppedState(dropIdx, false);

            int pwmRetract = p.Reversed ? p.PwmMax : p.PwmMin;
            if (await CubeOutputController.SendServoPwmAsync(p.Channel, pwmRetract))
                SetStatus($"{p.Name} retracted  (Cube ch{p.Channel} {pwmRetract}us)", SUCCESS_COLOR);
            else
                SetStatus("Retract failed: Cube output command unavailable", ERROR_COLOR);
        }

        private void OnPayloadDroppedStateChanged(int dropIdx0, bool isDropped)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () => ApplyDropVisual(dropIdx0, isDropped), "OnPayloadDroppedStateChanged");
        }

        private void ApplyDropVisual(int dropIdx, bool isDropped)
        {
            if (!_dropButtons.TryGetValue(dropIdx, out var btn) || btn == null) return;

            _dropDropped[dropIdx] = isDropped;
            ClearDropResetTimer(dropIdx);
            _dropClickCount[dropIdx] = 0;

            string name = DropName(dropIdx);
            btn.Text = isDropped ? $"Retract {name}" : $"Drop {name}";
            btn.BackColor = isDropped ? DROP_COLOR_DROPPED : DROP_COLOR_IDLE;
        }

        private void SeedDropVisuals()
        {
            foreach (var dropIdx in _dropButtons.Keys)
                ApplyDropVisual(dropIdx, IsPayloadDropped(dropIdx));
        }

        private string DropName(int dropIdx)
            => _dropPayloads.TryGetValue(dropIdx, out var p) && p != null ? p.Name : $"P{dropIdx + 1}";

        // ---- Slider ----

        private void BuildSliderRow(PayloadControl p, int sliderIdx, ref int y)
        {
            Controls.Add(RowLabel($"{p.Name}:", y));
            int x = 100;

            int min = Math.Min(p.PwmMin, p.PwmMax);
            int max = Math.Max(p.PwmMin, p.PwmMax);
            int initial = Math.Max(min, Math.Min(max, p.PwmNeutral));

            var slider = new TrackBar
            {
                Location    = new Point(x, y - 2),
                Size        = new Size(180, ROW_H),
                AutoSize    = false,
                TickStyle   = TickStyle.None,
                Minimum     = min,
                Maximum     = max,
                Value       = initial,
                SmallChange = 10,
                LargeChange = 50,
                BackColor   = CARD_BG,
            };
            x += 185;

            var lblValue = new Label
            {
                Text = $"{initial} us",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            };

            int channel = p.Channel;
            slider.ValueChanged += (s, e) =>
            {
                if (lblValue != null) lblValue.Text = $"{slider.Value} us";
                OnSliderChanged(sliderIdx, channel, slider);
            };

            Controls.Add(slider);
            Controls.Add(lblValue);

            y += ROW_H + ROW_GAP;
        }

        private void OnSliderChanged(int sliderIdx, int channel, TrackBar slider)
        {
            if (channel <= 0) return;

            // Stream immediately (drop in-flight duplicates), then settle-send the final value.
            SendServoPwmFireAndForget(channel, slider.Value, tryOnly: true);

            if (_sliderSettleTimers.TryGetValue(sliderIdx, out var existing) && existing != null)
            {
                existing.Stop();
                existing.Dispose();
            }

            var t = new Timer { Interval = TILT_SETTLE_MS };
            t.Tick += (s, e) =>
            {
                t.Stop();
                t.Dispose();
                _sliderSettleTimers.Remove(sliderIdx);
                if (!slider.IsDisposed)
                    SendServoPwmFireAndForget(channel, slider.Value, tryOnly: false);
            };
            _sliderSettleTimers[sliderIdx] = t;
            t.Start();
        }

        private async void SendServoPwmFireAndForget(int channel, int pwmUs, bool tryOnly)
        {
            if (channel <= 0) return;
            await CubeOutputController.SendServoPwmAsync(channel, pwmUs, tryOnly);
        }

        // ---- Relay / GPIO ----

        private void BuildRelayRow(PayloadControl p, ref int y)
        {
            Controls.Add(RowLabel($"{p.Name}:", y));

            if (p.PulseMs > 0)
            {
                var btn = MakeButton($"Fire {p.Name}", RELAY_COLOR, 120, ROW_H);
                btn.Location = new Point(100, y);
                btn.Click += (s, e) => OnFireRelayClick(p, btn);
                Controls.Add(btn);
            }
            else
            {
                var btn = MakeButton($"{p.Name}: OFF", Color.FromArgb(70, 70, 78), 120, ROW_H);
                btn.Location = new Point(100, y);
                btn.Click += (s, e) => ToggleRelay(p, btn);
                Controls.Add(btn);
            }

            y += ROW_H + ROW_GAP;
        }

        private void OnFireRelayClick(PayloadControl p, Button btn)
        {
            bool armed = _relayFireArmed.TryGetValue(p.Channel, out bool a) && a;
            if (!armed)
            {
                _relayFireArmed[p.Channel] = true;
                btn.Text = $"Confirm {p.Name}";
                btn.BackColor = DROP_COLOR_ARM2;
                SetStatus($"{p.Name} armed — click again to fire", WARNING_COLOR);
                RestartRelayFireResetTimer(p, btn);
                return;
            }

            ClearRelayFireResetTimer(p.Channel);
            ResetFireRelayButton(p, btn);
            FireRelay(p);
        }

        private void RestartRelayFireResetTimer(PayloadControl p, Button btn)
        {
            ClearRelayFireResetTimer(p.Channel);
            var t = new Timer { Interval = DROP_RESET_MS };
            t.Tick += (s, e) =>
            {
                ClearRelayFireResetTimer(p.Channel);
                if (!IsDisposed) ResetFireRelayButton(p, btn);
                SetStatus($"{p.Name} fire cancelled (timeout)", TEXT_SECONDARY);
            };
            _relayFireResetTimers[p.Channel] = t;
            t.Start();
        }

        private void ClearRelayFireResetTimer(int channel)
        {
            if (_relayFireResetTimers.TryGetValue(channel, out var t) && t != null)
            {
                t.Stop();
                t.Dispose();
            }
            _relayFireResetTimers.Remove(channel);
        }

        private void ResetFireRelayButton(PayloadControl p, Button btn)
        {
            _relayFireArmed[p.Channel] = false;
            if (btn != null && !btn.IsDisposed)
            {
                btn.Text = $"Fire {p.Name}";
                btn.BackColor = RELAY_COLOR;
            }
        }

        private async void FireRelay(PayloadControl p)
        {
            SetStatus($"{p.Name} firing  ({p.PulseMs}ms)...", SUCCESS_COLOR);
            bool success = await CubeOutputController.FireRelayAsync(p.Channel, p.PulseMs);
            SetStatus(
                success ? $"{p.Name} done" : $"{p.Name} failed: Cube relay command unavailable",
                success ? SUCCESS_COLOR : ERROR_COLOR);
        }

        private void ToggleRelay(PayloadControl p, Button btn)
        {
            bool current = _relayOn.TryGetValue(p.Channel, out bool on) && on;
            bool next = !current;
            _relayOn[p.Channel] = next;

            bool sent = CubeOutputController.TrySetRelayMavlink(p.Channel, next);
            btn.Text = $"{p.Name}: {(next ? "ON" : "OFF")}";
            btn.BackColor = next ? RELAY_ON_COLOR : Color.FromArgb(70, 70, 78);
            SetStatus(
                sent ? $"{p.Name} relay {(next ? "ON" : "OFF")}" : $"{p.Name}: Cube relay command unavailable",
                sent ? SUCCESS_COLOR : ERROR_COLOR);
        }
    }
}
