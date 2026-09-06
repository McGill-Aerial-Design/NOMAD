// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// PayloadControlPanel.Reels.cs - Strap reel command logic
// ============================================================
// Hold-to-reel with a per-reel safety cut-off, plus the
// three-click-armed full-spool buttons with countdown/cancel.
// Each reel is a PayloadKind.Reel entry in NOMADConfig.Payloads:
//   PwmMax = reel in, PwmMin = reel out, PwmNeutral = stop,
//   HoldSafetyS = hold cut-off, FullDurationS = full-spool run.
// Layout and the camera-tilt slider live in the other partials.
// ============================================================

using System;
using System.Drawing;
using Timer = System.Windows.Forms.Timer;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class PayloadControlPanel
    {
        // ============================================================
        // Strap reel  —  hold-to-reel, per-reel safety cut-off
        // ============================================================

        private int ReelChannel(int reelIdx) => ReelPayload(reelIdx)?.Channel ?? 0;
        private int ReelStopPwm(int reelIdx) => ReelPayload(reelIdx)?.PwmNeutral ?? 1500;
        private int ReelSafetyMs(int reelIdx) => Math.Max(1, ReelPayload(reelIdx)?.HoldSafetyS ?? 10) * 1000;
        private int ReelFullDurationMs(int reelIdx) => Math.Max(1, ReelPayload(reelIdx)?.FullDurationS ?? 80) * 1000;

        private void StartReel(int reelIdx, int pwmUs)
        {
            if (reelIdx < 0 || reelIdx >= _reelActive.Length) return;

            StopFullReel(reelIdx * 2, true);
            StopFullReel(reelIdx * 2 + 1, true);

            int channel = ReelChannel(reelIdx);
            _reelActive[reelIdx] = true;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            var t = new Timer { Interval = ReelSafetyMs(reelIdx) };
            int safetyS = ReelSafetyMs(reelIdx) / 1000;
            t.Tick += (s, e) =>
            {
                t.Stop();
                t.Dispose();
                _reelSafetyTimers[reelIdx] = null;
                _reelActive[reelIdx] = false;
                SendServoNow(ReelChannel(reelIdx), ReelStopPwm(reelIdx));
                SetStatus($"{ReelName(reelIdx)} stopped  ({safetyS}s safety limit)", WARNING_COLOR);
            };
            _reelSafetyTimers[reelIdx] = t;
            t.Start();

            SendServoNow(channel, pwmUs);
            SetStatus($"{ReelName(reelIdx)} ({pwmUs}µs) — hold button...", SUCCESS_COLOR);
        }

        private void StopReel(int reelIdx)
        {
            if (reelIdx < 0 || reelIdx >= _reelActive.Length) return;
            if (!_reelActive[reelIdx]) return;
            _reelActive[reelIdx] = false;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            _reelSafetyTimers[reelIdx] = null;

            SendServoNow(ReelChannel(reelIdx), ReelStopPwm(reelIdx));
            SetStatus($"{ReelName(reelIdx)} stopped", TEXT_SECONDARY);
        }

        private void CreateFullReelButton(int slot, int x, int y)
        {
            if (slot < 0 || slot >= _fullReelButtons.Length) return;

            var btn = MakeButton(FullReelFullLabel(slot), Color.FromArgb(85, 65, 35), 76, ROW_H);
            btn.Location = new Point(x, y);
            btn.Click += (s, e) => OnFullReelClick(slot);
            Controls.Add(btn);
            _fullReelButtons[slot] = btn;
        }

        private void OnFullReelClick(int slot)
        {
            if (slot < 0 || slot >= _fullReelButtons.Length) return;

            if (_fullReelActive[slot])
            {
                StopFullReel(slot, true);
                return;
            }

            _fullReelClickCount[slot]++;
            if (_fullReelClickCount[slot] < FULL_REEL_CLICKS_REQUIRED)
            {
                StartFullReelClickReset(slot);
                UpdateFullReelButton(slot);
                SetStatus(
                    $"{FullReelFullLabel(slot)} {ReelName(FullReelIndex(slot))} armed: {_fullReelClickCount[slot]}/{FULL_REEL_CLICKS_REQUIRED}",
                    WARNING_COLOR);
                return;
            }

            StartFullReel(slot);
        }

        private void StartFullReelClickReset(int slot)
        {
            _fullReelClickReset[slot]?.Stop();
            _fullReelClickReset[slot]?.Dispose();

            var resetTimer = new Timer { Interval = FULL_REEL_CLICK_RESET_MS };
            resetTimer.Tick += (s, e) =>
            {
                resetTimer.Stop();
                resetTimer.Dispose();
                _fullReelClickReset[slot] = null;
                _fullReelClickCount[slot] = 0;
                UpdateFullReelButton(slot);
            };
            _fullReelClickReset[slot] = resetTimer;
            resetTimer.Start();
        }

        private void StartFullReel(int slot)
        {
            int reelIdx = FullReelIndex(slot);
            int oppositeSlot = FullReelOppositeSlot(slot);
            int channel = ReelChannel(reelIdx);

            if (channel <= 0)
            {
                _fullReelClickReset[slot]?.Stop();
                _fullReelClickReset[slot]?.Dispose();
                _fullReelClickReset[slot] = null;
                _fullReelClickCount[slot] = 0;
                UpdateFullReelButton(slot);
                SetStatus($"{ReelName(reelIdx)} channel not configured (see Settings > Payloads)", WARNING_COLOR);
                return;
            }

            if (_fullReelActive[oppositeSlot])
                StopFullReel(oppositeSlot, false);
            if (_reelActive[reelIdx])
                StopReel(reelIdx);

            _fullReelClickReset[slot]?.Stop();
            _fullReelClickReset[slot]?.Dispose();
            _fullReelClickReset[slot] = null;
            _fullReelClickCount[slot] = 0;

            int durationMs = ReelFullDurationMs(reelIdx);
            _fullReelActive[slot] = true;
            _fullReelRemainingMs[slot] = durationMs;
            UpdateFullReelButton(slot);

            var reel = ReelPayload(reelIdx);
            int pwmUs = FullReelIsIn(slot) ? (reel?.PwmMax ?? 2100) : (reel?.PwmMin ?? 900);

            SendServoNow(channel, pwmUs);
            SetStatus($"{FullReelFullLabel(slot)} {ReelName(reelIdx)} running for {FormatDuration(durationMs)}  (click to cancel)", SUCCESS_COLOR);

            _fullReelCountdown[slot]?.Stop();
            _fullReelCountdown[slot]?.Dispose();
            var countdown = new Timer { Interval = 1000 };
            countdown.Tick += (s, e) =>
            {
                _fullReelRemainingMs[slot] -= 1000;
                if (_fullReelRemainingMs[slot] <= 0)
                {
                    StopFullReel(slot, false);
                    return;
                }
                UpdateFullReelButton(slot);
            };
            _fullReelCountdown[slot] = countdown;
            countdown.Start();
        }

        private void StopFullReel(int slot, bool cancelled)
        {
            if (slot < 0 || slot >= _fullReelButtons.Length) return;

            bool wasActive = _fullReelActive[slot];
            int reelIdx = FullReelIndex(slot);

            _fullReelActive[slot] = false;
            _fullReelRemainingMs[slot] = 0;
            _fullReelClickCount[slot] = 0;

            _fullReelClickReset[slot]?.Stop();
            _fullReelClickReset[slot]?.Dispose();
            _fullReelClickReset[slot] = null;

            _fullReelCountdown[slot]?.Stop();
            _fullReelCountdown[slot]?.Dispose();
            _fullReelCountdown[slot] = null;

            UpdateFullReelButton(slot);

            if (!wasActive) return;

            SendServoNow(ReelChannel(reelIdx), ReelStopPwm(reelIdx));
            SetStatus(
                cancelled ? $"{FullReelFullLabel(slot)} {ReelName(reelIdx)} cancelled" : $"{FullReelFullLabel(slot)} {ReelName(reelIdx)} complete",
                cancelled ? WARNING_COLOR : SUCCESS_COLOR);
        }

        private void UpdateFullReelButton(int slot)
        {
            var btn = _fullReelButtons[slot];
            if (btn == null || btn.IsDisposed) return;

            if (_fullReelActive[slot])
            {
                int secondsRemaining = Math.Max(0, _fullReelRemainingMs[slot] / 1000);
                btn.Text = $"{FullReelShortLabel(slot)} {secondsRemaining}s";
                btn.BackColor = WARNING_COLOR;
                return;
            }

            btn.Text = _fullReelClickCount[slot] > 0
                ? $"{FullReelShortLabel(slot)} {_fullReelClickCount[slot]}/{FULL_REEL_CLICKS_REQUIRED}"
                : FullReelFullLabel(slot);
            btn.BackColor = _fullReelClickCount[slot] > 0 ? Color.FromArgb(180, 95, 25) : Color.FromArgb(85, 65, 35);
        }

        private static int FullReelIndex(int slot) => slot / 2;
        private static int FullReelOppositeSlot(int slot) => FullReelIndex(slot) * 2 + (FullReelIsIn(slot) ? 1 : 0);
        private static bool FullReelIsIn(int slot) => slot % 2 == 0;
        private static string FullReelShortLabel(int slot) => FullReelIsIn(slot) ? "In" : "Out";
        private static string FullReelFullLabel(int slot) => FullReelIsIn(slot) ? "In Full" : "Out Full";

        private static string FormatDuration(int ms)
        {
            int totalSeconds = Math.Max(0, ms / 1000);
            int minutes = totalSeconds / 60;
            int seconds = totalSeconds % 60;
            return minutes > 0 ? $"{minutes}:{seconds:00}" : $"{seconds}s";
        }

        /// <summary>
        /// Fire-and-forget servo command for time-critical paths (reel MouseDown/Up).
        /// </summary>
        private async void SendServoNow(int channel, int pwmUs)
        {
            if (channel <= 0) return;
            await OutputController.SendServoPwmAsync(channel, pwmUs);
        }
    }
}
