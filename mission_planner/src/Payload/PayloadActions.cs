// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Headless payload-action helpers
// ============================================================
// Sends the same Cube servo / relay commands as PayloadControlPanel
// without any UI, so input sources without a panel (e.g.
// NomadJoystickService driven by a transmitter switch) can trigger
// drops, reels and the water pump directly.
//
// Drops and the water pump are driven from the modular NOMADConfig.Payloads
// list; strap reels keep their dedicated config. Indices are 1-based for drops
// (payload 1 == first enabled drop payload) to match the joystick mapping.
// ============================================================

using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    public static class PayloadActions
    {
        public static async Task Drop(NOMADConfig cfg, int payload)
        {
            try
            {
                var p = DropAt(cfg, payload);
                if (p == null || p.Channel <= 0) return;
                if (await CubeOutputController.SendServoPwmAsync(p.Channel, DropPwm(p)).ConfigureAwait(false))
                {
                    PayloadControlPanel.RaisePayloadDroppedState(payload - 1, true);
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Payload drop failed — {ex.Message}");
            }
        }

        public static async Task Retract(NOMADConfig cfg, int payload)
        {
            try
            {
                var p = DropAt(cfg, payload);
                if (p == null || p.Channel <= 0) return;
                if (await CubeOutputController.SendServoPwmAsync(p.Channel, RetractPwm(p)).ConfigureAwait(false))
                {
                    PayloadControlPanel.RaisePayloadDroppedState(payload - 1, false);
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Payload retract failed — {ex.Message}");
            }
        }

        public static async Task ReelStart(NOMADConfig cfg, int reelIdx)
        {
            try
            {
                if (cfg == null) return;
                int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
                int pwm = reelIdx == 0 ? cfg.ReelPwmIn : cfg.Reel2PwmIn;
                if (ch <= 0) return;
                await CubeOutputController.SendServoPwmAsync(ch, pwm).ConfigureAwait(false);
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
                if (cfg == null) return;
                int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
                int pwm = reelIdx == 0 ? cfg.ReelPwmOut : cfg.Reel2PwmOut;
                if (ch <= 0) return;
                await CubeOutputController.SendServoPwmAsync(ch, pwm).ConfigureAwait(false);
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
                if (cfg == null) return;
                int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
                if (ch <= 0) return;
                await CubeOutputController.SendServoPwmAsync(ch, 1500).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Log.Error($"Reel stop failed — {ex.Message}");
            }
        }

        public static async Task FireWater(NOMADConfig cfg)
        {
            try
            {
                var pump = cfg?.WaterPump();
                if (pump == null) return;
                await CubeOutputController.FireRelayAsync(pump.Channel, pump.PulseMs > 0 ? pump.PulseMs : 500)
                    .ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Log.Error($"Water pump fire failed — {ex.Message}");
            }
        }

        // The 1-based n-th enabled drop payload, or null.
        private static PayloadControl DropAt(NOMADConfig cfg, int payload)
        {
            List<PayloadControl> drops = cfg.DropPayloads();
            int idx = payload - 1;
            return idx >= 0 && idx < drops.Count ? drops[idx] : null;
        }

        // Reversed servos drop at PwmMin and retract at PwmMax; non-reversed do the opposite.
        private static int DropPwm(PayloadControl p) => p.Reversed ? p.PwmMin : p.PwmMax;
        private static int RetractPwm(PayloadControl p) => p.Reversed ? p.PwmMax : p.PwmMin;
    }
}
