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
using System.Threading.Tasks;
using NOMAD.MissionPlanner.Connectivity;
using Timer = System.Windows.Forms.Timer;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class PayloadControlPanel
    {
        // ============================================================
        // Strap reel  —  hold-to-reel, per-reel safety cut-off
        // ============================================================

        private readonly Task<NomadCoreRequestResult>[] _reelPending =
            new Task<NomadCoreRequestResult>[NOMADConfig.MaxPayloads];
        private readonly bool[] _reelStopping = new bool[NOMADConfig.MaxPayloads];
        private readonly bool[] _reelStarting = new bool[NOMADConfig.MaxPayloads];
        private readonly bool[] _reelHoldRequested = new bool[NOMADConfig.MaxPayloads];

        private int ReelChannel(int reelIdx) => ReelPayload(reelIdx)?.Channel ?? 0;
        private int ReelStopPwm(int reelIdx) => ReelPayload(reelIdx)?.PwmNeutral ?? 1500;
        private int ReelSafetyMs(int reelIdx) => Math.Max(1, ReelPayload(reelIdx)?.HoldSafetyS ?? 10) * 1000;
        private int ReelFullDurationMs(int reelIdx) => Math.Max(1, ReelPayload(reelIdx)?.FullDurationS ?? 80) * 1000;

        private Task StartReel(int reelIdx, int pwmUs)
            => RunReelStartAsync(reelIdx, () => StartReelCoreAsync(reelIdx, pwmUs));

        private async Task StartReelCoreAsync(int reelIdx, int pwmUs)
        {
            if (IsDisposed || reelIdx < 0 || reelIdx >= _reelActive.Length) return;
            if (_reelPending[reelIdx]?.IsCompleted == false || _reelStopping[reelIdx]) return;

            _reelHoldRequested[reelIdx] = true;
            await StopFullReel(reelIdx * 2, true);
            if (IsDisposed || !_reelHoldRequested[reelIdx]) return;
            await StopFullReel(reelIdx * 2 + 1, true);
            if (IsDisposed || !_reelHoldRequested[reelIdx]) return;

            int channel = ReelChannel(reelIdx);
            _reelActive[reelIdx] = true;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            var t = new Timer { Interval = ReelSafetyMs(reelIdx) };
            int safetyS = ReelSafetyMs(reelIdx) / 1000;
            t.Tick += async (s, e) =>
            {
                t.Stop();
                t.Dispose();
                _reelSafetyTimers[reelIdx] = null;
                _reelActive[reelIdx] = false;
                SetReelCommandStatus(await SendReelStopAsync(reelIdx),
                    $"{ReelName(reelIdx)}: stop command accepted ({safetyS}s safety limit); physical stop unverified");
            };
            _reelSafetyTimers[reelIdx] = t;
            t.Start();

            _reelPending[reelIdx] = SendServoNowAsync(channel, pwmUs);
            var result = await _reelPending[reelIdx];
            if (IsDisposed || !_reelHoldRequested[reelIdx]) return;
            SetReelCommandStatus(result,
                $"{ReelName(reelIdx)}: command accepted ({pwmUs}µs); physical movement unverified");
        }

        private async Task StopReel(int reelIdx)
        {
            if (IsDisposed || reelIdx < 0 || reelIdx >= _reelActive.Length) return;
            _reelHoldRequested[reelIdx] = false;
            if (!_reelActive[reelIdx] || _reelStopping[reelIdx]) return;
            _reelActive[reelIdx] = false;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            _reelSafetyTimers[reelIdx] = null;

            SetReelCommandStatus(await SendReelStopAsync(reelIdx),
                $"{ReelName(reelIdx)}: stop command accepted; physical stop unverified");
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
                _ = StopFullReel(slot, true);
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

            _ = StartFullReel(slot);
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

        private Task StartFullReel(int slot)
            => RunReelStartAsync(FullReelIndex(slot), () => StartFullReelCoreAsync(slot));

        private async Task StartFullReelCoreAsync(int slot)
        {
            if (IsDisposed || slot < 0 || slot >= _fullReelButtons.Length) return;
            int reelIdx = FullReelIndex(slot);
            if (_reelPending[reelIdx]?.IsCompleted == false || _reelStopping[reelIdx]) return;
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
            {
                await StopFullReel(oppositeSlot, false);
                if (IsDisposed) return;
            }
            if (_reelActive[reelIdx])
            {
                await StopReel(reelIdx);
                if (IsDisposed) return;
            }
            if (IsDisposed) return;

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

            _reelPending[reelIdx] = SendServoNowAsync(channel, pwmUs);
            var result = await _reelPending[reelIdx];
            if (IsDisposed) return;
            SetReelCommandStatus(result,
                $"{ReelName(reelIdx)}: command accepted; timer {FormatDuration(durationMs)}, "
                    + "physical movement unverified");

            if (!_fullReelActive[slot]) return;
            _fullReelCountdown[slot]?.Stop();
            _fullReelCountdown[slot]?.Dispose();
            var countdown = new Timer { Interval = 1000 };
            countdown.Tick += async (s, e) =>
            {
                _fullReelRemainingMs[slot] -= 1000;
                if (_fullReelRemainingMs[slot] <= 0)
                {
                    await StopFullReel(slot, false);
                    return;
                }
                UpdateFullReelButton(slot);
            };
            _fullReelCountdown[slot] = countdown;
            countdown.Start();
        }

        private async Task StopFullReel(int slot, bool cancelled)
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

            var reason = cancelled ? "timer cancelled" : "timer elapsed";
            SetReelCommandStatus(await SendReelStopAsync(reelIdx),
                $"{ReelName(reelIdx)}: {reason}, stop command accepted; physical stop unverified");
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
        /// Send one reel command and preserve its result for operator feedback.
        /// </summary>
        private async Task RunReelStartAsync(int reelIdx, Func<Task> start)
        {
            // Panel handlers and continuations share the UI thread; reject reentrant starts before any await.
            if (IsDisposed || reelIdx < 0 || reelIdx >= _reelStarting.Length || _reelStarting[reelIdx]) return;
            _reelStarting[reelIdx] = true;
            try
            {
                await start();
            }
            finally
            {
                _reelStarting[reelIdx] = false;
            }
        }
        private async Task<NomadCoreRequestResult> SendReelStopAsync(int reelIdx)
        {
            if (_reelStopping[reelIdx])
            {
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted,
                    "request_in_progress", "The explicit reel stop is already in progress.");
            }
            _reelStopping[reelIdx] = true;
            try
            {
                return await OutputController.SendServoStopAsync(ReelChannel(reelIdx), ReelStopPwm(reelIdx));
            }
            finally
            {
                _reelStopping[reelIdx] = false;
            }
        }
        private Task<NomadCoreRequestResult> SendServoNowAsync(int channel, int pwmUs)
        {
            return OutputController.SendServoPwmAsync(channel, pwmUs);
        }

        private void SetReelCommandStatus(NomadCoreRequestResult result, string acceptedMessage)
        {
            if (IsDisposed) return;
            SetStatus(result.Succeeded ? acceptedMessage : OutputController.DescribeFailure("Reel command", result),
                result.Succeeded ? SUCCESS_COLOR : ERROR_COLOR);
        }
    }
}
