// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NomadJoystickService.Switches.cs - Payload switch buttons
// ============================================================
// Maps the virtual Xbox 360 buttons produced by joystick.py
// (RadioMaster 3-position switches + kill button) to payload
// actions. Axis polling and device handling live in
// NomadJoystickService.cs.
// ============================================================

using System;
using System.Threading.Tasks;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;

namespace NOMAD.MissionPlanner
{
    public sealed partial class NomadJoystickService
    {
        // ============================================================
        // Payload switch buttons
        // ============================================================
        // joystick.py encodes each 3-position RadioMaster switch as a pair of
        // virtual Xbox 360 buttons — UP = one button, DOWN = another, middle
        // releases both. Six slots total, each user-configurable in Settings →
        // Joystick to fire any of the actions in SwitchAction below.
        //
        //   button 0 (A)  = sw1 UP    -> Config.JoystickSw1UpAction
        //   button 1 (B)  = sw1 DOWN  -> Config.JoystickSw1DownAction
        //   button 2 (X)  = sw2 UP    -> Config.JoystickSw2UpAction
        //   button 3 (Y)  = sw2 DOWN  -> Config.JoystickSw2DownAction
        //   button 4 (LB) = sw3 UP    -> Config.JoystickSw3UpAction
        //   button 5 (RB) = sw3 DOWN  -> Config.JoystickSw3DownAction
        //
        // Drop/fire confirmations require neutral between edges (three for drop,
        // two for fire within the shared window). Reel actions are level-triggered — run while the switch is
        // held off-centre, neutral PWM when it returns to middle.

        private const int SLOT_COUNT = 6;
        // joystick.py maps the radio kill pushbutton to XInput BACK, which lands
        // on DirectInput button index 6. Read separately so the user-configurable
        // 6 switch slots don't have to know about it.
        private const int KILL_BUTTON_IDX = 6;

        private bool[] _prevButtons = new bool[SLOT_COUNT];
        private bool _prevKillButton;
        private readonly bool[] _neutralObserved = new bool[3];
        private readonly PayloadReleaseInterlock[] _releaseInterlocks = new PayloadReleaseInterlock[SLOT_COUNT];
        private readonly bool[] _knownReleaseCommanded = new bool[3];
        // Drop toggle state lives in PayloadControlPanel.s_payloadReleaseCommanded so the
        // joystick and UI agree about what the next switch flip should do —
        // dropping from the GUI then flipping the switch retracts, and vice versa.
        private readonly int[] _reelDir = new int[2]; // last commanded direction per reel: 0=stop, +1=in, -1=out

        public enum SwitchAction
        {
            None,
            DropToggleP1,
            DropToggleP2,
            DropToggleP3,
            ReelInP1,
            ReelOutP1,
            ReelInP2,
            ReelOutP2,
            FireWaterPump,
        }

        private static SwitchAction ParseAction(string id)
        {
            if (string.IsNullOrWhiteSpace(id)) return SwitchAction.None;
            return Enum.TryParse(id, true, out SwitchAction a) ? a : SwitchAction.None;
        }

        private SwitchAction GetSlotAction(int slot)
        {
            switch (slot)
            {
                case 0: return ParseAction(_config.JoystickSw1UpAction);
                case 1: return ParseAction(_config.JoystickSw1DownAction);
                case 2: return ParseAction(_config.JoystickSw2UpAction);
                case 3: return ParseAction(_config.JoystickSw2DownAction);
                case 4: return ParseAction(_config.JoystickSw3UpAction);
                case 5: return ParseAction(_config.JoystickSw3DownAction);
                default: return SwitchAction.None;
            }
        }

        private void DrivePayloadButtons(IMyJoystickState st)
        {
            DrivePayloadButtons(st, PayloadReleaseInterlock.NowMs);
        }

        private void DrivePayloadButtons(IMyJoystickState st, long nowMs)
        {
            bool[] buttons;
            try { buttons = st.GetButtons(); }
            catch
            {
                ResetPayloadInput();
                return;
            }
            if (buttons == null || buttons.Length < SLOT_COUNT)
            {
                ResetPayloadInput();
                return;
            }
            ResetChangedPayloadState();

            // Snapshot per-slot pressed state for this tick.
            var pressed = new bool[SLOT_COUNT];
            for (int i = 0; i < SLOT_COUNT; i++)
                pressed[i] = i < buttons.Length && buttons[i];
            for (int pair = 0; pair < 3; pair++)
            {
                int first = pair * 2;
                if (!pressed[first] && !pressed[first + 1])
                {
                    _neutralObserved[pair] = true;
                }
                if (pressed[first] && pressed[first + 1])
                {
                    _neutralObserved[pair] = false;
                    // Contradictory positions cannot command a reel direction.
                    pressed[first] = false;
                    pressed[first + 1] = false;
                    _releaseInterlocks[first]?.Reset();
                    _releaseInterlocks[first + 1]?.Reset();
                }
            }

            // Kill switch — dedicated edge-triggered pushbutton (XInput BACK).
            // Sits outside the configurable slot table because it has fixed
            // semantics and we never want a user to accidentally map it away.
            if (_config.JoystickKillSwitchEnabled)
            {
                bool killNow = KILL_BUTTON_IDX < buttons.Length && buttons[KILL_BUTTON_IDX];
                if (killNow && !_prevKillButton)
                {
                    Log.Warn("Termination button pressed.");
                    if (!FlightModeController.RequestTermination())
                    {
                        AudioAlerts.Speak("Termination unavailable. Take manual control.", component: "joystick");
                    }
                }
                _prevKillButton = killNow;
            }
            else
            {
                _prevKillButton = false;
            }

            // Pass 1: edge-triggered actions (drop toggles, water pump)
            for (int slot = 0; slot < SLOT_COUNT; slot++)
            {
                bool now = pressed[slot];
                bool prev = _prevButtons[slot];
                if (!(now && !prev)) continue; // rising edge only
                if (!_neutralObserved[slot / 2])
                {
                    continue;
                }
                _neutralObserved[slot / 2] = false;
                FireEdgeAction(GetSlotAction(slot), slot, nowMs);
            }

            // Pass 2: level-triggered reels. A reel can be driven from either
            // direction by any slot; resolve at most one owner per reel per tick
            // (first slot wins) so two switches don't fight over the same PWM.
            var reelTarget = new int[] { 0, 0 }; // 0 = stop, +1 = reel in, -1 = reel out
            for (int slot = 0; slot < SLOT_COUNT; slot++)
            {
                if (!pressed[slot]) continue;
                var action = GetSlotAction(slot);
                int reelIdx, dir;
                switch (action)
                {
                    case SwitchAction.ReelInP1:  reelIdx = 0; dir = +1; break;
                    case SwitchAction.ReelOutP1: reelIdx = 0; dir = -1; break;
                    case SwitchAction.ReelInP2:  reelIdx = 1; dir = +1; break;
                    case SwitchAction.ReelOutP2: reelIdx = 1; dir = -1; break;
                    default: continue;
                }
                if (reelTarget[reelIdx] == 0) reelTarget[reelIdx] = dir; // first slot wins
            }
            ApplyReel(0, reelTarget[0]);
            ApplyReel(1, reelTarget[1]);

            _prevButtons = pressed;
        }

        private void FireEdgeAction(SwitchAction action, int slot, long nowMs)
        {
            switch (action)
            {
                case SwitchAction.DropToggleP1: ToggleDrop(0, slot, nowMs); break;
                case SwitchAction.DropToggleP2: ToggleDrop(1, slot, nowMs); break;
                case SwitchAction.DropToggleP3: ToggleDrop(2, slot, nowMs); break;
                case SwitchAction.FireWaterPump:
                    ConfirmWater(slot, nowMs);
                    break;
                // Reel actions are handled level-triggered in pass 2 — ignore on edge.
                default: break;
            }
        }

        private void ToggleDrop(int payloadIdx, int slot, long nowMs)
        {
            // PayloadActions.Drop / Retract raise PayloadReleaseCommandedStateChanged on
            // success, which updates the shared state we read here next time.
            var drops = _config.DropPayloads();
            if (payloadIdx >= drops.Count)
            {
                return;
            }
            if (PayloadControlPanel.IsPayloadReleaseCommanded(payloadIdx) ||
                PayloadActions.RequiresSafeRecovery(drops[payloadIdx].Channel))
            {
                _releaseInterlocks[slot]?.Reset();
                RunPayloadAction(() => PayloadActions.Retract(_config, payloadIdx + 1), $"retract payload {payloadIdx + 1}");
            }
            else
            {
                var authorization = ConfirmRelease(slot, PayloadReleaseInterlock.DropConfirmations, nowMs);
                if (authorization == null)
                {
                    return;
                }
                RunPayloadAction(() => PayloadActions.Drop(_config, payloadIdx + 1, authorization),
                    $"drop payload {payloadIdx + 1}");
            }
        }

        private PayloadReleaseAuthorization ConfirmRelease(int slot, int confirmations, long nowMs)
        {
            if (_releaseInterlocks[slot] == null)
            {
                _releaseInterlocks[slot] = new PayloadReleaseInterlock(confirmations,
                    PayloadReleaseInterlock.ConfirmationWindowMs);
            }
            var result = _releaseInterlocks[slot].RegisterClick(nowMs);
            if (result.Outcome == ReleaseInterlockOutcome.Arming)
            {
                Log.Warn($"Payload switch: return to neutral; {result.ClicksRemaining} more confirmation(s) required.");
            }
            return result.Authorization;
        }

        private void ConfirmWater(int slot, long nowMs)
        {
            var pump = _config.WaterPump();
            if (pump == null)
            {
                return;
            }
            if (PayloadActions.RequiresSafeRecovery(pump.Channel, relay: true))
            {
                _releaseInterlocks[slot]?.Reset();
                RunPayloadAction(() => PayloadActions.SetRelay(pump, false), "stop water pump");
                return;
            }
            var authorization = ConfirmRelease(slot, PayloadReleaseInterlock.RelayConfirmations, nowMs);
            if (authorization == null)
            {
                return;
            }
            RunPayloadAction(() => PayloadActions.FireWater(_config, authorization), "fire water pump");
        }

        private void ResetChangedPayloadState()
        {
            for (int payload = 0; payload < _knownReleaseCommanded.Length; payload++)
            {
                bool commanded = PayloadControlPanel.IsPayloadReleaseCommanded(payload);
                if (commanded == _knownReleaseCommanded[payload])
                {
                    continue;
                }
                _knownReleaseCommanded[payload] = commanded;
                for (int slot = 0; slot < SLOT_COUNT; slot++)
                {
                    if (GetSlotAction(slot) == (SwitchAction)((int)SwitchAction.DropToggleP1 + payload))
                    {
                        _releaseInterlocks[slot]?.Reset();
                    }
                }
            }
        }

        private void ResetPayloadInput()
        {
            ApplyReel(0, 0);
            ApplyReel(1, 0);
            for (int slot = 0; slot < SLOT_COUNT; slot++)
            {
                _releaseInterlocks[slot]?.Reset();
                _releaseInterlocks[slot] = null;
                _prevButtons[slot] = false;
            }
            for (int pair = 0; pair < _neutralObserved.Length; pair++)
            {
                _neutralObserved[pair] = false;
            }
            _prevKillButton = false;
        }

        private void ApplyReel(int reelIdx, int target)
        {
            // Only re-send when the commanded direction actually changes. The
            // servo holds the last PWM, so an idle reel costs zero MAVLink traffic.
            if (target == _reelDir[reelIdx]) return;
            _reelDir[reelIdx] = target;
            if      (target > 0) RunPayloadAction(() => PayloadActions.ReelStart(_config, reelIdx), $"reel {reelIdx + 1} in");
            else if (target < 0) RunPayloadAction(() => PayloadActions.ReelStartOut(_config, reelIdx), $"reel {reelIdx + 1} out");
            else                 RunPayloadAction(() => PayloadActions.ReelStop(_config, reelIdx), $"reel {reelIdx + 1} stop");
        }

        private static void RunPayloadAction(Func<Task> action, string name)
        {
            _ = RunPayloadActionAsync(action, name);
        }

        private static async Task RunPayloadActionAsync(Func<Task> action, string name)
        {
            try
            {
                await action().ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Log.Error($"Joystick payload action failed ({name}) — {ex.Message}");
            }
        }
    }
}
