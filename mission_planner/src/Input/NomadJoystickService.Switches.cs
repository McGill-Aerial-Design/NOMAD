// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;

namespace NOMAD.MissionPlanner
{
    public sealed partial class NomadJoystickService
    {
        private readonly bool[] _previousButtons = new bool[6];
        private readonly bool[] _previousNeutral = new bool[3];
        private long _inputGeneration;
        private readonly HashSet<string> _previousConflictingIds = new HashSet<string>();
        private readonly long[] _pairGeneration = new long[3];
        private bool _haveValidButtons;
        private bool _previousKill;
        private string _mappingError = "";
        private readonly Dictionary<string, int> _lastValidInputTargets = new Dictionary<string, int>();

        private string GetSlotAction(int slot)
        {
            var actions = new[] { _config.JoystickSw1UpAction, _config.JoystickSw1DownAction,
                _config.JoystickSw2UpAction, _config.JoystickSw2DownAction,
                _config.JoystickSw3UpAction, _config.JoystickSw3DownAction };
            return slot >= 0 && slot < actions.Length ? actions[slot] : "None";
        }

        private static bool TryBinding(string text, out string id, out string operation)
        {
            id = "";
            operation = "";
            if (string.IsNullOrWhiteSpace(text) || text == "None")
            {
                return false;
            }
            int separator = text.LastIndexOf(':');
            if (separator <= 0 || separator == text.Length - 1)
            {
                return false;
            }
            id = text.Substring(0, separator);
            operation = text.Substring(separator + 1);
            return true;
        }

        private Task DriveActuatorButtons(IMyJoystickState state)
        {
            string mappingError = _config.GetInputMappingError();
            if (mappingError != null)
            {
                ResetActuatorInput();
                if (_mappingError != mappingError) { Log.Warn("USB HID mapping rejected: " + mappingError); }
                _mappingError = mappingError;
                return Task.CompletedTask;
            }
            _mappingError = "";
            bool[] buttons;
            try
            {
                buttons = state.GetButtons();
            }
            catch
            {
                ResetActuatorInput(); return Task.CompletedTask;
            }
            if (buttons == null || _config.JoystickButtonIndices == null || _config.JoystickButtonIndices.Length != 6)
            { ResetActuatorInput(); return Task.CompletedTask; }
            var activeIndices = new HashSet<int>();
            for (int slot = 0; slot < 6; slot++)
            { if (TryBinding(GetSlotAction(slot), out _, out _)) { activeIndices.Add(_config.JoystickButtonIndices[slot]); } }
            var pressed = new bool[6];
            for (int slot = 0; slot < pressed.Length; slot++)
            {
                int index = _config.JoystickButtonIndices[slot];
                bool active = TryBinding(GetSlotAction(slot), out _, out _);
                if (!active && (activeIndices.Contains(index) ||
                    (_config.JoystickKillSwitchEnabled && index == _config.JoystickTerminationButtonIndex))) { continue; }
                if (index < 0 || index >= buttons.Length)
                {
                    if (TryBinding(GetSlotAction(slot), out _, out _))
                    {
                        ResetActuatorInput(); return Task.CompletedTask;
                    }
                    continue;
                }
                pressed[slot] = buttons[index];
            }
            _lastValidInputTargets.Clear();
            for (int slot = 0; slot < 6; slot++)
            { if (TryBinding(GetSlotAction(slot), out var id, out _)) { _lastValidInputTargets[id] = slot / 2; } }
            var dispatch = TranslateButtons(pressed);
            if (_config.JoystickKillSwitchEnabled && buttons.Length > _config.JoystickTerminationButtonIndex)
            {
                bool now = buttons[_config.JoystickTerminationButtonIndex];
                if (now && !_previousKill)
                {
                    Log.Warn("Termination button pressed.");
                    if (!FlightModeController.RequestTermination())
                    { AudioAlerts.Speak("Termination unavailable. Take manual control.", component: "joystick"); }
                }
                _previousKill = now;
            }
            return dispatch;
        }

        private Task TranslateButtons(bool[] pressed)
        {
            var conflicting = new HashSet<string>();
            var held = new HashSet<string>();
            for (int slot = 0; slot < 6; slot++)
            {
                if (pressed[slot] && TryBinding(GetSlotAction(slot), out var id, out _) && !held.Add(id))
                { conflicting.Add(id); }
            }
            for (int pair = 0; pair < 3; pair++)
            {
                int first = pair * 2;
                if (!pressed[first] || !pressed[first + 1]) { continue; }
                for (int slot = first; slot < first + 2; slot++)
                { if (TryBinding(GetSlotAction(slot), out var id, out _)) { conflicting.Add(id); } }
            }
            if (conflicting.Count > 0) { Interlocked.Increment(ref _inputGeneration); }
            var newlyConflicting = new HashSet<string>(conflicting);
            newlyConflicting.ExceptWith(_previousConflictingIds);
            var batches = new List<Task>();
            for (int pair = 0; pair < 3; pair++)
            {
                batches.Add(TranslatePair(pressed, pair, conflicting, newlyConflicting));
            }
            _previousConflictingIds.Clear();
            _previousConflictingIds.UnionWith(conflicting);
            Array.Copy(pressed, _previousButtons, pressed.Length);
            _haveValidButtons = true;
            return Task.WhenAll(batches);
        }

        private Task TranslatePair(bool[] pressed, int pair, HashSet<string> conflicting, HashSet<string> newlyConflicting)
        {
            int first = pair * 2;
            if (!_haveValidButtons || pressed[first] != _previousButtons[first] || pressed[first + 1] != _previousButtons[first + 1])
            { Interlocked.Increment(ref _pairGeneration[pair]); }
            bool contradiction = pressed[first] && pressed[first + 1];
            bool neutral = !pressed[first] && !pressed[first + 1];
            var events = new List<(string Id, string Operation, int Slot, bool Release)>();
            var neutralTargets = new HashSet<string>();
            var safeTargets = new HashSet<string>();
            for (int slot = first; slot < first + 2; slot++)
            {
                if (!TryBinding(GetSlotAction(slot), out var id, out var operation))
                {
                    continue;
                }
                string release = OutputController.GetReleaseOperation(GetSlotAction(slot));
                if (_haveValidButtons && !pressed[slot] && _previousButtons[slot] && release != "")
                { events.Add((id, release, pair, true)); }
                if (contradiction || conflicting.Contains(id))
                {
                    if (newlyConflicting.Remove(id) && safeTargets.Add(id))
                    {
                        events.Add((id, "safe", pair, true));
                    }
                }
                else if (neutral && (!_haveValidButtons || !_previousNeutral[pair]) && neutralTargets.Add(id))
                { events.Add((id, "neutral", pair, false)); }
                else if (_haveValidButtons && pressed[slot] && !_previousButtons[slot])
                { events.Add((id, operation, pair, operation == "safe" || operation == "stop")); }
            }
            _previousNeutral[pair] = neutral && !contradiction;
            return SendHidEventsAsync(events, Interlocked.Read(ref _inputGeneration), Interlocked.Read(ref _pairGeneration[pair]), pair);
        }

        private async Task SendHidEventsAsync(List<(string Id, string Operation, int Slot, bool Release)> events, long generation, long pairGeneration, int pair)
        {
            foreach (var input in events)
            {
                Func<bool> stillCurrent = input.Release ? (Func<bool>)null : () =>
                    generation == Interlocked.Read(ref _inputGeneration) && pairGeneration == Interlocked.Read(ref _pairGeneration[pair]);
                if (stillCurrent != null && !stillCurrent())
                {
                    continue;
                }
                await ReportHidResultAsync(input.Id, input.Operation, input.Slot, stillCurrent).ConfigureAwait(false);
            }
        }

        private void ResetActuatorInput()
        {
            Interlocked.Increment(ref _inputGeneration);
            if (_haveValidButtons)
            {
                foreach (var target in _lastValidInputTargets) { SendHid(target.Key, "safe", target.Value); }
            }
            _lastValidInputTargets.Clear();
            _previousConflictingIds.Clear();
            Array.Clear(_previousButtons, 0, _previousButtons.Length);
            Array.Clear(_previousNeutral, 0, _previousNeutral.Length);
            _haveValidButtons = false;
            _previousKill = false;
        }

        private static void SendHid(string id, string operation, int slot) =>
            _ = ReportHidResultAsync(id, operation, slot);
        private static async Task ReportHidResultAsync(string id, string operation, int slot, Func<bool> inputStillCurrent = null)
        {
            try
            {
                await OutputController.ActuatorActionAsync(id, operation, "hid", slot, inputStillCurrent: inputStillCurrent).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Log.Error("USB HID request failed: " + ex.Message);
            }
        }
    }
}
