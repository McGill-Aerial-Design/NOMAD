// ============================================================
// Headless payload-action helpers
// ============================================================
// Sends the same Cube servo / relay commands as PayloadControlPanel
// without any UI side effects, so input sources without a panel
// (e.g. NomadJoystickService driven by a transmitter switch) can
// trigger drops, reels and the water pump directly.
// ============================================================

using System;

namespace NOMAD.MissionPlanner
{
    public static class PayloadActions
    {
        public static async void Drop(NOMADConfig cfg, int payload)
        {
            if (cfg == null) return;
            int ch, pwm, ch2 = 0, pwm2 = 0;
            switch (payload)
            {
                case 1:
                    (pwm,  _) = ServoPwm(cfg.Servo1PwmMin,  cfg.Servo1PwmMax,  cfg.Servo1Reversed);
                    (pwm2, _) = ServoPwm(cfg.Servo1bPwmMin, cfg.Servo1bPwmMax, cfg.Servo1bReversed);
                    ch  = cfg.Servo1Channel; ch2 = cfg.Servo1bChannel; break;
                case 2:
                    (pwm, _) = ServoPwm(cfg.Servo2PwmMin, cfg.Servo2PwmMax, cfg.Servo2Reversed);
                    ch = cfg.Servo2Channel; break;
                case 3:
                    (pwm, _) = ServoPwm(cfg.Servo3PwmMin, cfg.Servo3PwmMax, cfg.Servo3Reversed);
                    ch = cfg.Servo3Channel; break;
                default: return;
            }
            if (ch <= 0) return;
            await CubeOutputController.SendServoPwmAsync(ch, pwm);
            if (ch2 > 0) await CubeOutputController.SendServoPwmAsync(ch2, pwm2);
        }

        public static async void Retract(NOMADConfig cfg, int payload)
        {
            if (cfg == null) return;
            int ch, pwm, ch2 = 0, pwm2 = 0;
            switch (payload)
            {
                case 1:
                    (_, pwm)  = ServoPwm(cfg.Servo1PwmMin,  cfg.Servo1PwmMax,  cfg.Servo1Reversed);
                    (_, pwm2) = ServoPwm(cfg.Servo1bPwmMin, cfg.Servo1bPwmMax, cfg.Servo1bReversed);
                    ch  = cfg.Servo1Channel; ch2 = cfg.Servo1bChannel; break;
                case 2:
                    (_, pwm) = ServoPwm(cfg.Servo2PwmMin, cfg.Servo2PwmMax, cfg.Servo2Reversed);
                    ch = cfg.Servo2Channel; break;
                case 3:
                    (_, pwm) = ServoPwm(cfg.Servo3PwmMin, cfg.Servo3PwmMax, cfg.Servo3Reversed);
                    ch = cfg.Servo3Channel; break;
                default: return;
            }
            if (ch <= 0) return;
            await CubeOutputController.SendServoPwmAsync(ch, pwm);
            if (ch2 > 0) await CubeOutputController.SendServoPwmAsync(ch2, pwm2);
        }

        public static async void ReelStart(NOMADConfig cfg, int reelIdx)
        {
            if (cfg == null) return;
            int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
            int pwm = reelIdx == 0 ? cfg.ReelPwmIn       : cfg.Reel2PwmIn;
            if (ch <= 0) return;
            await CubeOutputController.SendServoPwmAsync(ch, pwm);
        }

        public static async void ReelStartOut(NOMADConfig cfg, int reelIdx)
        {
            if (cfg == null) return;
            int ch = reelIdx == 0 ? cfg.ReelServoChannel : cfg.Reel2ServoChannel;
            int pwm = reelIdx == 0 ? cfg.ReelPwmOut      : cfg.Reel2PwmOut;
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
            if (cfg == null) return;
            await CubeOutputController.FireRelayAsync(cfg.WaterPumpRelayNumber, cfg.WaterPumpDurationMs);
        }

        private static (int drop, int retract) ServoPwm(int pwmMin, int pwmMax, bool reversed)
            => reversed ? (pwmMin, pwmMax) : (pwmMax, pwmMin);
    }
}
