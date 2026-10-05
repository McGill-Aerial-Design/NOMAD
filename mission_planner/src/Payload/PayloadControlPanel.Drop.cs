// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Payload Control Panel - Dynamic payload rows
// ============================================================
// Renders the configurable NOMADConfig.Payloads list:
//   Drop   - three-click-armed drop / retract servo button.
//   Slider - live PWM slider for an aiming / nozzle servo.
//   Relay  - GPIO / relay output: momentary "Fire" pulse or latching toggle.
// Commanded release state is shared across panel instances (and read by the joystick service).
//
// The arm/confirm decision for the release paths (drop + momentary relay fire)
// lives in the pure, unit-tested PayloadReleaseInterlock (tier SC); this file is
// chrome — it renders the returned outcome and drives the visual revert timer.
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

        private const int DROP_CLICKS_REQUIRED = PayloadReleaseInterlock.DropConfirmations;
        private const int DROP_RESET_MS = PayloadReleaseInterlock.ConfirmationWindowMs;

        // Monotonic clock for the release interlocks (never wraps or runs backwards),
        // so the arm-window decision is independent of wall-clock changes.
        private static long NowMs() => PayloadReleaseInterlock.NowMs;

        // Per drop-index (0-based, in panel order) UI + arming state. The arm/confirm
        // decision is delegated to a PayloadReleaseInterlock; the button + reset timer
        // are the chrome around it.
        private readonly Dictionary<int, Button>                   _dropButtons     = new Dictionary<int, Button>();
        private readonly Dictionary<int, PayloadControl> _dropPayloads = new Dictionary<int, PayloadControl>();
        private readonly Dictionary<int, PayloadReleaseInterlock> _dropInterlocks =
            new Dictionary<int, PayloadReleaseInterlock>();
        private readonly Dictionary<int, Timer>                    _dropResetTimers = new Dictionary<int, Timer>();
        private readonly Dictionary<int, bool>                     _dropReleaseCommanded = new Dictionary<int, bool>();

        // Per slider-index settle timers (final send after the slider stops moving).
        private readonly Dictionary<int, Timer> _sliderSettleTimers = new Dictionary<int, Timer>();

        // ============================================================
        // Release interlocks (pure SC decision, lazily created per actuator)
        // ============================================================

        private PayloadReleaseInterlock DropInterlock(int dropIdx)
        {
            if (!_dropInterlocks.TryGetValue(dropIdx, out var il) || il == null)
            {
                il = new PayloadReleaseInterlock(DROP_CLICKS_REQUIRED, DROP_RESET_MS);
                _dropInterlocks[dropIdx] = il;
            }
            return il;
        }

        // ============================================================
        // Cross-panel commanded-release sync (also read by NomadJoystickService)
        // ============================================================

        public static event Action<int, bool> PayloadReleaseCommandedStateChanged;

        private static readonly bool[] s_payloadReleaseCommanded = new bool[NOMADConfig.MaxPayloads];

        /// <summary>
        /// True after a successful release command. Physical payload state is unobserved.
        /// </summary>
        public static bool IsPayloadReleaseCommanded(int dropIdx0)
            => dropIdx0 >= 0 && dropIdx0 < s_payloadReleaseCommanded.Length && s_payloadReleaseCommanded[dropIdx0];

        public static void RaisePayloadReleaseCommandedState(int dropIdx0, bool releaseCommanded)
        {
            if (dropIdx0 < 0 || dropIdx0 >= s_payloadReleaseCommanded.Length) return;
            s_payloadReleaseCommanded[dropIdx0] = releaseCommanded;
            PayloadReleaseCommandedStateChanged?.Invoke(dropIdx0, releaseCommanded);
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
            DropInterlock(dropIdx).Reset();
            _dropReleaseCommanded[dropIdx]    = IsPayloadReleaseCommanded(dropIdx);

            y += ROW_H + ROW_GAP;
        }

        private void OnDropClick(int dropIdx)
        {
            bool recovery = _dropPayloads.TryGetValue(dropIdx, out var payload) &&
                PayloadActions.RequiresSafeRecovery(payload.Channel);
            if (recovery || (_dropReleaseCommanded.TryGetValue(dropIdx, out bool releaseCommanded) && releaseCommanded))
            {
                DropInterlock(dropIdx).Reset();
                _ = ExecuteRetract(dropIdx);
                return;
            }

            var result = DropInterlock(dropIdx).RegisterClick(NowMs());
            RestartDropResetTimer(dropIdx);

            if (result.Outcome == ReleaseInterlockOutcome.Arming)
            {
                int remaining = result.ClicksRemaining;
                if (_dropButtons.TryGetValue(dropIdx, out var b))
                    b.BackColor = result.ClickCount == 1 ? DROP_COLOR_ARM1 : DROP_COLOR_ARM2;
                SetStatus($"{DropName(dropIdx)}: {remaining} more click{(remaining == 1 ? "" : "s")} to drop!",
                    WARNING_COLOR);
                return;
            }

            ClearDropResetTimer(dropIdx);
            if (_dropButtons.TryGetValue(dropIdx, out var btn)) btn.BackColor = DROP_COLOR_IDLE;

            _ = ExecuteDrop(dropIdx, result.Authorization);
        }

        private void RestartDropResetTimer(int dropIdx)
        {
            ClearDropResetTimer(dropIdx);
            var t = new Timer { Interval = DROP_RESET_MS };
            t.Tick += (s, e) =>
            {
                ClearDropResetTimer(dropIdx);
                DropInterlock(dropIdx).Reset();
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

        private async System.Threading.Tasks.Task ExecuteDrop(int dropIdx, PayloadReleaseAuthorization authorization)
        {
            if (!_dropPayloads.TryGetValue(dropIdx, out var p) || p == null) return;
            if (p.Channel <= 0)
            {
                SetStatus($"{p.Name} channel not configured (Settings → Payloads)", WARNING_COLOR);
                return;
            }

            var result = await PayloadActions.Drop(_config, dropIdx + 1, authorization);
            if (IsDisposed) return;
            if (result.Succeeded)
            {
                SetStatus($"{p.Name}: release command accepted; physical release unverified", SUCCESS_COLOR);
            }
            else
            {
                SetStatus(OutputController.DescribeFailure("Release command", result), ERROR_COLOR);
            }
        }

        private async System.Threading.Tasks.Task ExecuteRetract(int dropIdx)
        {
            if (!_dropPayloads.TryGetValue(dropIdx, out var p) || p == null) return;
            if (p.Channel <= 0)
            {
                SetStatus($"{p.Name} channel not configured (Settings → Payloads)", WARNING_COLOR);
                return;
            }

            var result = await PayloadActions.Retract(_config, dropIdx + 1);
            if (IsDisposed) return;
            if (result.Succeeded)
            {
                SetStatus($"{p.Name}: retract command accepted; physical retraction unverified", SUCCESS_COLOR);
            }
            else
            {
                SetStatus(OutputController.DescribeFailure("Retract command", result), ERROR_COLOR);
            }
        }

        private void OnPayloadReleaseCommandedStateChanged(int dropIdx0, bool releaseCommanded)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () => ApplyDropVisual(dropIdx0, releaseCommanded),
                "OnPayloadReleaseCommandedStateChanged");
        }

        private void ApplyDropVisual(int dropIdx, bool releaseCommanded)
        {
            if (!_dropButtons.TryGetValue(dropIdx, out var btn) || btn == null) return;

            _dropReleaseCommanded[dropIdx] = releaseCommanded;
            ClearDropResetTimer(dropIdx);
            DropInterlock(dropIdx).Reset();

            string name = DropName(dropIdx);
            btn.Text = releaseCommanded ? $"Retract {name}" : $"Drop {name}";
            btn.BackColor = releaseCommanded ? DROP_COLOR_DROPPED : DROP_COLOR_IDLE;
        }

        private void SeedDropVisuals()
        {
            foreach (var dropIdx in _dropButtons.Keys)
                ApplyDropVisual(dropIdx, IsPayloadReleaseCommanded(dropIdx));
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
            SendServoPwmFireAndForget(channel, slider.Value);

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
                    SendServoPwmFireAndForget(channel, slider.Value);
            };
            _sliderSettleTimers[sliderIdx] = t;
            t.Start();
        }

        private async void SendServoPwmFireAndForget(int channel, int pwmUs)
        {
            if (channel <= 0) return;
            await OutputController.SendServoPwmAsync(channel, pwmUs);
        }

    }
}
