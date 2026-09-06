// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Payload Configuration Models
// ============================================================
// A payload is one configurable ArduPilot output channel. The control panel
// and the settings editor are both data-driven from NOMADConfig.Payloads, so
// operators can add / remove / reconfigure up to NOMADConfig.MaxPayloads of
// them without a code change. Outputs are standard ArduPilot servo/relay
// channels that work on any ArduPilot flight controller. Five
// kinds are supported:
//
//   Drop    - a servo with two endpoints; three-click "Drop", click again to retract.
//   Slider  - a servo exposed as a live PWM slider (e.g. an aiming / nozzle servo).
//   Relay   - a GPIO / relay output (e.g. the water pump); momentary pulse or toggle.
//   Reel    - a strap-reel servo: hold-to-reel with a safety cut-off, plus
//             three-click-armed full-spool in/out with a configurable duration.
//   CamTilt - the camera tilt servo: live slider with down/level/up presets,
//             also driven by the ZED-tilt joystick channel.
// ============================================================

using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>How a <see cref="PayloadControl"/> drives its ArduPilot output.</summary>
    public enum PayloadKind
    {
        /// <summary>Servo with two endpoints — drop (with confirm) and retract.</summary>
        Drop,

        /// <summary>Servo exposed as a live PWM slider (aiming / nozzle servo).</summary>
        Slider,

        /// <summary>GPIO / relay output (pump, igniter, ...): momentary pulse or toggle.</summary>
        Relay,

        /// <summary>Strap-reel servo: hold-to-reel + timed full-spool in/out.</summary>
        Reel,

        /// <summary>Camera tilt servo: live slider with down/level/up presets.</summary>
        CamTilt,
    }

    /// <summary>
    /// One configurable payload output. Persisted in <c>NOMADConfig.Payloads</c>.
    /// </summary>
    public class PayloadControl
    {
        /// <summary>Operator-facing label shown on the panel and in settings.</summary>
        public string Name { get; set; } = "Payload";

        /// <summary>When false the payload is hidden from the panel.</summary>
        public bool Enabled { get; set; } = true;

        /// <summary>Which output type / UI this payload uses.</summary>
        public PayloadKind Kind { get; set; } = PayloadKind.Drop;

        /// <summary>ArduPilot servo output channel (Drop / Slider / Reel / CamTilt) or relay/GPIO number (Relay).</summary>
        public int Channel { get; set; } = 9;

        /// <summary>
        /// Servo low endpoint (Drop) / slider minimum (Slider) / reel-out PWM (Reel)
        /// / fully-down PWM (CamTilt), in microseconds.
        /// </summary>
        public int PwmMin { get; set; } = 1000;

        /// <summary>
        /// Servo high endpoint (Drop) / slider maximum (Slider) / reel-in PWM (Reel)
        /// / fully-up PWM (CamTilt), in microseconds.
        /// </summary>
        public int PwmMax { get; set; } = 2000;

        /// <summary>
        /// Slider default / resting position (Slider), reel stop PWM (Reel), or
        /// level / straight-ahead PWM (CamTilt), in microseconds.
        /// </summary>
        public int PwmNeutral { get; set; } = 1500;

        /// <summary>Drop only: when true the servo drops at <see cref="PwmMin"/> instead of <see cref="PwmMax"/>.</summary>
        public bool Reversed { get; set; }

        /// <summary>Relay only: pulse length in ms. &gt;0 fires a momentary pulse; 0 latches an on/off toggle.</summary>
        public int PulseMs { get; set; } = 500;

        /// <summary>
        /// Relay only: optional RC pass-through input channel (5–16, 0 = disabled). When set, the
        /// operator can hold a transmitter switch on this channel to fire the relay through the flight
        /// controller directly. Settings → Payloads writes the matching <c>RC{n}_OPTION</c>
        /// on the autopilot (any ArduPilot flight controller).
        /// </summary>
        public int RcChannel { get; set; }

        /// <summary>Reel only: max continuous hold-to-reel time in seconds before the safety cut-off stops the reel.</summary>
        public int HoldSafetyS { get; set; } = 10;

        /// <summary>Reel only: full-spool run duration in seconds for the armed "In Full" / "Out Full" buttons.</summary>
        public int FullDurationS { get; set; } = 80;

        /// <summary>CamTilt only: physical tilt range in degrees each way from level (pushed to the Jetson for angle conversion).</summary>
        public int AngleRangeDeg { get; set; } = 45;

        public PayloadControl Clone() => (PayloadControl)MemberwiseClone();

        /// <summary>A strap reel with the standard NOMAD defaults (out &lt;1000 us, in &gt;2000 us, stop 1500 us).</summary>
        public static PayloadControl NewReel(string name, int channel = 12) => new PayloadControl
        {
            Name = name,
            Kind = PayloadKind.Reel,
            Channel = channel,
            PwmMin = 900,      // reel out
            PwmMax = 2100,     // reel in
            PwmNeutral = 1500, // stop
        };

        /// <summary>The ZED camera tilt servo with the standard NOMAD calibration (700 down / 1250 level / 1450 up).</summary>
        public static PayloadControl NewCamTilt(string name = "Cam Tilt", int channel = 14) => new PayloadControl
        {
            Name = name,
            Kind = PayloadKind.CamTilt,
            Channel = channel,
            PwmMin = 700,      // fully down
            PwmNeutral = 1250, // level (mechanically offset arm, not 1500)
            PwmMax = 1450,     // fully up
        };
    }

    /// <summary>Lookup helpers over <c>NOMADConfig.Payloads</c>.</summary>
    public static class PayloadConfigExtensions
    {
        /// <summary>Enabled payloads in panel order.</summary>
        public static List<PayloadControl> EnabledPayloads(this NOMADConfig cfg)
            => cfg?.Payloads?.Where(p => p != null && p.Enabled).ToList() ?? new List<PayloadControl>();

        /// <summary>Enabled drop-servo payloads in order (joystick "payload 1" == index 0).</summary>
        public static List<PayloadControl> DropPayloads(this NOMADConfig cfg)
            => cfg?.Payloads?.Where(p => p != null && p.Enabled && p.Kind == PayloadKind.Drop).ToList()
               ?? new List<PayloadControl>();

        /// <summary>The water-pump / spray relay: the first enabled relay payload, or null.</summary>
        public static PayloadControl WaterPump(this NOMADConfig cfg)
            => cfg?.Payloads?.FirstOrDefault(p => p != null && p.Enabled && p.Kind == PayloadKind.Relay);

        /// <summary>Enabled strap-reel payloads in order (joystick "reel P1" == index 0).</summary>
        public static List<PayloadControl> ReelPayloads(this NOMADConfig cfg)
            => cfg?.Payloads?.Where(p => p != null && p.Enabled && p.Kind == PayloadKind.Reel).ToList()
               ?? new List<PayloadControl>();

        /// <summary>The 0-based n-th enabled reel payload, or null.</summary>
        public static PayloadControl ReelAt(this NOMADConfig cfg, int reelIdx)
        {
            var reels = cfg.ReelPayloads();
            return reelIdx >= 0 && reelIdx < reels.Count ? reels[reelIdx] : null;
        }

        /// <summary>The camera tilt servo: the first enabled CamTilt payload, or null.</summary>
        public static PayloadControl CameraTilt(this NOMADConfig cfg)
            => cfg?.Payloads?.FirstOrDefault(p => p != null && p.Enabled && p.Kind == PayloadKind.CamTilt);
    }
}
