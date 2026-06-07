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

using System.Collections.Generic;

namespace NOMAD.MissionPlanner
{
    public static class PayloadActions
    {
        public static async void Drop(NOMADConfig cfg, int payload)
        {
            var p = DropAt(cfg, payload);
            if (p == null || p.Channel <= 0) return;
            await CubeOutputController.SendServoPwmAsync(p.Channel, DropPwm(p));
            PayloadControlPanel.RaisePayloadDroppedState(payload - 1, true);
        }

        public static async void Retract(NOMADConfig cfg, int payload)
        {
            var p = DropAt(cfg, payload);
            if (p == null || p.Channel <= 0) return;
            await CubeOutputController.SendServoPwmAsync(p.Channel, RetractPwm(p));
            PayloadControlPanel.RaisePayloadDroppedState(payload - 1, false);
        }

        public static async void ReelStart(NOMADConfig cfg, int reelIdx)
        {
            if (cfg == null) return;
            int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
            int pwm = reelIdx == 0 ? cfg.ReelPwmIn : cfg.Reel2PwmIn;
            if (ch <= 0) return;
            await CubeOutputController.SendServoPwmAsync(ch, pwm);
        }

        public static async void ReelStartOut(NOMADConfig cfg, int reelIdx)
        {
            if (cfg == null) return;
            int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
            int pwm = reelIdx == 0 ? cfg.ReelPwmOut : cfg.Reel2PwmOut;
            if (ch <= 0) return;
            await CubeOutputController.SendServoPwmAsync(ch, pwm);
        }

        public static async void ReelStop(NOMADConfig cfg, int reelIdx)
        {
            if (cfg == null) return;
            int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
            if (ch <= 0) return;
            await CubeOutputController.SendServoPwmAsync(ch, 1500);
        }

        public static async void FireWater(NOMADConfig cfg)
        {
            var pump = cfg?.WaterPump();
            if (pump == null) return;
            await CubeOutputController.FireRelayAsync(pump.Channel, pump.PulseMs > 0 ? pump.PulseMs : 500);
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
