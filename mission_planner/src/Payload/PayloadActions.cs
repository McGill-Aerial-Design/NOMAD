// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Headless payload-action helpers
// ============================================================
// Sends the same ArduPilot servo / relay commands as PayloadControlPanel
// without any UI, so input sources without a panel (e.g.
// NomadJoystickService driven by a transmitter switch) can trigger
// drops, reels and the water pump through the shared authorization boundary.
//
// Drops, strap reels and the water pump are all driven from the modular
// NOMADConfig.Payloads list. Indices are 1-based for drops (payload 1 == first
// enabled drop payload) and 0-based for reels (reel 0 == first enabled reel
// payload) to match the joystick mapping.
// ============================================================

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    public static class PayloadActions
    {
        private static readonly object UncertaintyGate = new object();
        private static readonly HashSet<int> UncertainServos = new HashSet<int>();
        private static readonly HashSet<int> UncertainRelays = new HashSet<int>();
        private static readonly HashSet<int> PendingServos = new HashSet<int>();
        private static readonly HashSet<int> PendingRelays = new HashSet<int>();

        private static bool TryBeginOutput(int channel, bool relay, bool safe)
        {
            lock (UncertaintyGate)
            {
                var uncertain = relay ? UncertainRelays : UncertainServos;
                var pending = relay ? PendingRelays : PendingServos;
                if (pending.Contains(channel) || (!safe && uncertain.Contains(channel)))
                {
                    return false;
                }
                pending.Add(channel);
                return true;
            }
        }

        public static bool RequiresSafeRecovery(int channel, bool relay = false)
        {
            lock (UncertaintyGate)
            {
                return (relay ? UncertainRelays : UncertainServos).Contains(channel);
            }
        }

        private static void RecordResult(int channel, bool relay, NomadCoreRequestResult result, bool safe = false)
        {
            lock (UncertaintyGate)
            {
                var channels = relay ? UncertainRelays : UncertainServos;
                if (result.Outcome == NomadCoreRequestOutcome.Interrupted ||
                    result.Outcome == NomadCoreRequestOutcome.UnknownOutcome)
                {
                    channels.Add(channel);
                }
                else if (safe && result.Succeeded)
                {
                    channels.Remove(channel);
                }
                (relay ? PendingRelays : PendingServos).Remove(channel);
            }
        }

        private static NomadCoreRequestResult NotAuthorized() => new NomadCoreRequestResult(
            NomadCoreRequestOutcome.NotAttempted, "payload_not_authorized",
            "Fresh confirmation is required; pending outputs are busy "
                + "and uncertain outputs require retract or OFF first.");

        private static bool Consume(PayloadReleaseAuthorization authorization, int confirmations)
        {
            return authorization != null && authorization.TryConsume(confirmations, PayloadReleaseInterlock.NowMs);
        }

        public static async Task<NomadCoreRequestResult> Drop(NOMADConfig cfg, int payload,
            PayloadReleaseAuthorization authorization)
        {
            var p = DropAt(cfg, payload);
            if (!Consume(authorization, PayloadReleaseInterlock.DropConfirmations) || p == null ||
                p.Channel <= 0 || RequiresSafeRecovery(p.Channel))
            {
                return NotAuthorized();
            }
            var result = await SendOutput(p.Channel, false,
                () => OutputController.SendServoPwmAsync(p.Channel, DropPwm(p))).ConfigureAwait(false);
            if (result.Succeeded)
            {
                PayloadControlPanel.RaisePayloadReleaseCommandedState(payload - 1, true);
            }
            return result;
        }

        public static async Task<NomadCoreRequestResult> Retract(NOMADConfig cfg, int payload)
        {
            var p = DropAt(cfg, payload);
            if (p == null || p.Channel <= 0)
            {
                return NotAuthorized();
            }
            var result = await SendOutput(p.Channel, false,
                () => OutputController.SendServoStopAsync(p.Channel, RetractPwm(p)), safe: true).ConfigureAwait(false);
            if (result.Succeeded)
            {
                PayloadControlPanel.RaisePayloadReleaseCommandedState(payload - 1, false);
            }
            return result;
        }

        private static async Task<NomadCoreRequestResult> SendOutput(int channel, bool relay,
            Func<Task<NomadCoreRequestResult>> send, bool safe = false)
        {
            if (!TryBeginOutput(channel, relay, safe))
            {
                return NotAuthorized();
            }
            NomadCoreRequestResult result;
            try
            {
                result = await send().ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                result = new NomadCoreRequestResult(NomadCoreRequestOutcome.UnknownOutcome,
                    "payload_output_exception", "Output state is unknown. Do not retry blindly.");
                Log.Error($"Payload output failed: {ex.Message}");
            }
            RecordResult(channel, relay, result, safe);
            return result;
        }

        public static async Task ReelStart(NOMADConfig cfg, int reelIdx)
        {
            try
            {
                var reel = cfg?.ReelAt(reelIdx);
                if (reel == null || reel.Channel <= 0) return;
                await OutputController.SendServoPwmAsync(reel.Channel, reel.PwmMax).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Log.Error($"Reel start failed — {ex.Message}");
            }
        }

        public static async Task ReelStartOut(NOMADConfig cfg, int reelIdx)
        {
            try
            {
                var reel = cfg?.ReelAt(reelIdx);
                if (reel == null || reel.Channel <= 0) return;
                await OutputController.SendServoPwmAsync(reel.Channel, reel.PwmMin).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Log.Error($"Reel out failed — {ex.Message}");
            }
        }

        public static async Task ReelStop(NOMADConfig cfg, int reelIdx)
        {
            try
            {
                var reel = cfg?.ReelAt(reelIdx);
                if (reel == null || reel.Channel <= 0) return;
                await OutputController.SendServoStopAsync(reel.Channel, reel.PwmNeutral).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Log.Error($"Reel stop failed — {ex.Message}");
            }
        }

        public static Task<NomadCoreRequestResult> FireWater(NOMADConfig cfg, PayloadReleaseAuthorization authorization)
        {
            var pump = cfg?.WaterPump();
            return pump == null ? Task.FromResult(NotAuthorized()) : FireRelay(pump, authorization);
        }

        public static async Task<NomadCoreRequestResult> FireRelay(PayloadControl relay,
            PayloadReleaseAuthorization authorization)
        {
            if (!Consume(authorization, PayloadReleaseInterlock.RelayConfirmations) || relay == null ||
                RequiresSafeRecovery(relay.Channel, relay: true))
            {
                return NotAuthorized();
            }
            return await SendOutput(relay.Channel, true,
                () => OutputController.FireRelayAsync(relay.Channel, relay.PulseMs > 0 ? relay.PulseMs : 500))
                .ConfigureAwait(false);
        }

        public static async Task<NomadCoreRequestResult> SetRelay(PayloadControl relay, bool on,
            PayloadReleaseAuthorization authorization = null)
        {
            if (relay == null || (on && (!Consume(authorization, PayloadReleaseInterlock.RelayConfirmations) ||
                RequiresSafeRecovery(relay.Channel, relay: true))))
            {
                return NotAuthorized();
            }
            return await SendOutput(relay.Channel, true,
                () => OutputController.SetRelayAsync(relay.Channel, on), safe: !on).ConfigureAwait(false);
        }

        // The 1-based n-th enabled drop payload, or null.
        private static PayloadControl DropAt(NOMADConfig cfg, int payload)
        {
            List<PayloadControl> drops = cfg?.DropPayloads();
            if (drops == null)
            {
                return null;
            }
            int idx = payload - 1;
            return idx >= 0 && idx < drops.Count ? drops[idx] : null;
        }

        // Reversed servos drop at PwmMin and retract at PwmMax; non-reversed do the opposite.
        private static int DropPwm(PayloadControl p) => p.Reversed ? p.PwmMin : p.PwmMax;
        private static int RetractPwm(PayloadControl p) => p.Reversed ? p.PwmMax : p.PwmMin;
    }
}
