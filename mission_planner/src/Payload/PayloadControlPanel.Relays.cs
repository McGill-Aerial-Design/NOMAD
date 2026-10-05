// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// Relay row rendering and command handlers; authorization remains in PayloadActions.
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using Timer = System.Windows.Forms.Timer;

namespace NOMAD.MissionPlanner
{
    public partial class PayloadControlPanel
    {
        private static readonly Color RELAY_COLOR        = Color.FromArgb(30, 100, 180);
        private static readonly Color RELAY_ON_COLOR     = Color.FromArgb(60, 150, 60);

        private const int RELAY_CLICKS_REQUIRED = PayloadReleaseInterlock.RelayConfirmations;

        // Latching-relay on/off state, keyed by relay number.
        private readonly Dictionary<int, bool> _relayOn = new Dictionary<int, bool>();

        // Momentary-relay fire arming (SR-PAY-03): arm → confirm via the same
        // interlock as drops (two clicks within the window). Keyed by channel.
        private readonly Dictionary<int, PayloadReleaseInterlock> _relayFireInterlocks =
            new Dictionary<int, PayloadReleaseInterlock>();
        private readonly Dictionary<int, Timer>                   _relayFireResetTimers = new Dictionary<int, Timer>();

        private PayloadReleaseInterlock RelayFireInterlock(int channel)
        {
            if (!_relayFireInterlocks.TryGetValue(channel, out var il) || il == null)
            {
                il = new PayloadReleaseInterlock(RELAY_CLICKS_REQUIRED, DROP_RESET_MS);
                _relayFireInterlocks[channel] = il;
            }
            return il;
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
                var btn = MakeButton($"{p.Name}: no confirmed command", Color.FromArgb(70, 70, 78), 120, ROW_H);
                btn.Location = new Point(100, y);
                btn.Click += (s, e) => _ = ToggleRelay(p, btn);
                Controls.Add(btn);
            }

            y += ROW_H + ROW_GAP;
        }

        private void OnFireRelayClick(PayloadControl p, Button btn)
        {
            if (PayloadActions.RequiresSafeRecovery(p.Channel, relay: true))
            {
                ResetFireRelayButton(p, btn);
                _ = StopRelay(p, btn);
                return;
            }
            var result = RelayFireInterlock(p.Channel).RegisterClick(NowMs());
            if (result.Outcome == ReleaseInterlockOutcome.Arming)
            {
                btn.Text = $"Confirm {p.Name}";
                btn.BackColor = DROP_COLOR_ARM2;
                SetStatus($"{p.Name} armed — click again to fire", WARNING_COLOR);
                RestartRelayFireResetTimer(p, btn);
                return;
            }

            ClearRelayFireResetTimer(p.Channel);
            ResetFireRelayButton(p, btn, resetAuthorization: false);
            FireRelay(p, result.Authorization, btn);
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

        private void ResetFireRelayButton(PayloadControl p, Button btn, bool resetAuthorization = true)
        {
            if (resetAuthorization)
            {
                RelayFireInterlock(p.Channel).Reset();
            }
            if (btn != null && !btn.IsDisposed)
            {
                btn.Text = p.PulseMs > 0 ? $"Fire {p.Name}" : $"{p.Name}: no confirmed command";
                btn.BackColor = RELAY_COLOR;
            }
        }

        private async void FireRelay(PayloadControl p, PayloadReleaseAuthorization authorization, Button btn)
        {
            SetStatus($"{p.Name} firing  ({p.PulseMs}ms)...", SUCCESS_COLOR);
            var result = await PayloadActions.FireRelay(p, authorization);
            if (IsDisposed) return;
            if (PayloadActions.RequiresSafeRecovery(p.Channel, relay: true) && !btn.IsDisposed)
            {
                btn.Text = $"Stop {p.Name}";
            }
            SetStatus(
                result.Succeeded ? $"{p.Name}: pulse commands accepted; physical effect unverified"
                    : OutputController.DescribeFailure("Relay pulse", result),
                result.Succeeded ? SUCCESS_COLOR : ERROR_COLOR);
        }

        private async System.Threading.Tasks.Task ToggleRelay(PayloadControl p, Button btn)
        {
            bool current = _relayOn.TryGetValue(p.Channel, out bool on) && on;
            bool next = !current && !PayloadActions.RequiresSafeRecovery(p.Channel, relay: true);
            PayloadReleaseAuthorization authorization = null;
            if (next)
            {
                var confirmation = RelayFireInterlock(p.Channel).RegisterClick(NowMs());
                if (confirmation.Outcome != ReleaseInterlockOutcome.Fire)
                {
                    btn.Text = $"Confirm {p.Name} ON";
                    RestartRelayFireResetTimer(p, btn);
                    return;
                }
                ClearRelayFireResetTimer(p.Channel);
                authorization = confirmation.Authorization;
            }
            else
            {
                RelayFireInterlock(p.Channel).Reset();
            }
            var result = await PayloadActions.SetRelay(p, next, authorization);
            if (IsDisposed || btn.IsDisposed)
            {
                return;
            }
            if (!result.Succeeded)
            {
                btn.Text = PayloadActions.RequiresSafeRecovery(p.Channel, relay: true)
                    ? $"Stop {p.Name}" : $"{p.Name}: no confirmed command";
                SetStatus(OutputController.DescribeFailure("Relay command", result), ERROR_COLOR);
                return;
            }
            _relayOn[p.Channel] = next;
            btn.Text = $"{p.Name}: commanded {(next ? "ON" : "OFF")}";
            btn.BackColor = next ? RELAY_ON_COLOR : Color.FromArgb(70, 70, 78);
            SetStatus($"{p.Name}: relay command accepted; physical effect unverified", SUCCESS_COLOR);
        }

        private async System.Threading.Tasks.Task StopRelay(PayloadControl p, Button btn)
        {
            var result = await PayloadActions.SetRelay(p, false);
            if (IsDisposed || btn.IsDisposed) return;
            if (result.Succeeded)
            {
                _relayOn[p.Channel] = false;
                ResetFireRelayButton(p, btn);
            }
            SetStatus(result.Succeeded ? $"{p.Name}: OFF command accepted; physical state unverified"
                : OutputController.DescribeFailure("Relay OFF", result),
                result.Succeeded ? SUCCESS_COLOR : ERROR_COLOR);
        }
    }
}
