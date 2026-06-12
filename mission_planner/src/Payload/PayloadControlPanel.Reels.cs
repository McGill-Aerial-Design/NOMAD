// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// PayloadControlPanel.Reels.cs - Strap reel command logic
// ============================================================
// Hold-to-reel with a 10-second safety cut-off, plus the
// three-click-armed full-spool buttons with countdown/cancel.
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
        // Strap reel  —  hold-to-reel, 10-second safety cut-off
        // ============================================================

        private void StartReel(int reelIdx, int pwmUs)
        {
            StopFullReel(reelIdx * 2, true);
            StopFullReel(reelIdx * 2 + 1, true);

            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);
            _reelActive[reelIdx] = true;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            var t = new Timer { Interval = REEL_SAFETY_MS };
            t.Tick += (s, e) =>
            {
                t.Stop();
                t.Dispose();
                _reelSafetyTimers[reelIdx] = null;
                _reelActive[reelIdx] = false;
                SendServoNow(reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0), 1500);
                SetStatus($"Reel P{reelIdx + 1} stopped  (10s safety limit)", WARNING_COLOR);
            };
            _reelSafetyTimers[reelIdx] = t;
            t.Start();

            SendServoNow(channel, pwmUs);
            SetStatus($"Reel P{reelIdx + 1} ({pwmUs}µs) — hold button...", SUCCESS_COLOR);
        }

        private void StopReel(int reelIdx)
        {
            if (!_reelActive[reelIdx]) return;
            _reelActive[reelIdx] = false;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            _reelSafetyTimers[reelIdx] = null;

            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);
            SendServoNow(channel, 1500);
            SetStatus($"Reel P{reelIdx + 1} stopped", TEXT_SECONDARY);
        }

        private void CreateFullReelButton(int slot, int x, int y)
        {
            if (slot < 0 || slot >= _fullReelButtons.Length) return;

            var btn = MakeButton(_fullReelLabels[slot], Color.FromArgb(85, 65, 35), 76, ROW_H);
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
                    $"{_fullReelLabels[slot]} P{FullReelNumber(slot)} armed: {_fullReelClickCount[slot]}/{FULL_REEL_CLICKS_REQUIRED}",
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
            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);

            if (channel <= 0)
            {
                _fullReelClickReset[slot]?.Stop();
                _fullReelClickReset[slot]?.Dispose();
                _fullReelClickReset[slot] = null;
                _fullReelClickCount[slot] = 0;
                UpdateFullReelButton(slot);
                SetStatus($"Reel P{reelIdx + 1} channel not configured (see Settings > Servos)", WARNING_COLOR);
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

            _fullReelActive[slot] = true;
            _fullReelRemainingMs[slot] = FULL_REEL_DURATION_MS;
            UpdateFullReelButton(slot);

            int pwmUs = FullReelIsIn(slot)
                ? (reelIdx == 0 ? (_config?.ReelPwmIn ?? 2100) : (_config?.Reel2PwmIn ?? 2100))
                : (reelIdx == 0 ? (_config?.ReelPwmOut ?? 900) : (_config?.Reel2PwmOut ?? 900));

            SendServoNow(channel, pwmUs);
            SetStatus($"{_fullReelLabels[slot]} P{reelIdx + 1} running for 1:20  (click to cancel)", SUCCESS_COLOR);

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

            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);
            SendServoNow(channel, 1500);
            SetStatus(
                cancelled ? $"{_fullReelLabels[slot]} P{reelIdx + 1} cancelled" : $"{_fullReelLabels[slot]} P{reelIdx + 1} complete",
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
                : _fullReelLabels[slot];
            btn.BackColor = _fullReelClickCount[slot] > 0 ? Color.FromArgb(180, 95, 25) : Color.FromArgb(85, 65, 35);
        }

        private static int FullReelIndex(int slot) => slot / 2;
        private static int FullReelNumber(int slot) => FullReelIndex(slot) + 1;
        private static int FullReelOppositeSlot(int slot) => FullReelIndex(slot) * 2 + (FullReelIsIn(slot) ? 1 : 0);
        private static bool FullReelIsIn(int slot) => slot % 2 == 0;
        private static string FullReelShortLabel(int slot) => FullReelIsIn(slot) ? "In" : "Out";

        /// <summary>
        /// Fire-and-forget Cube servo command for time-critical paths (reel MouseDown/Up).
        /// </summary>
        private async void SendServoNow(int channel, int pwmUs)
        {
            if (channel <= 0) return;
            await CubeOutputController.SendServoPwmAsync(channel, pwmUs);
        }
    }
}
