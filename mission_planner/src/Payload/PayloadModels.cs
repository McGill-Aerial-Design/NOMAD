// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Payload Configuration Models
// ============================================================
// A payload is one configurable Cube Orange output. The control panel and the
// settings editor are both data-driven from NOMADConfig.Payloads, so operators
// can add / remove / reconfigure up to NOMADConfig.MaxPayloads of them without a
// code change. Three kinds are supported:
//
//   Drop   - a servo with two endpoints; three-click "Drop", click again to retract.
//   Slider - a servo exposed as a live PWM slider (e.g. an aiming / nozzle servo).
//   Relay  - a GPIO / relay output (e.g. the water pump); momentary pulse or toggle.
//
// Strap reels and the ZED camera tilt remain dedicated sections (they carry
// bespoke safety / joystick behaviour) and are configured separately.
// ============================================================

using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>How a <see cref="PayloadControl"/> drives its Cube output.</summary>
    public enum PayloadKind
    {
        /// <summary>Servo with two endpoints — drop (with confirm) and retract.</summary>
        Drop,

        /// <summary>Servo exposed as a live PWM slider (aiming / nozzle servo).</summary>
        Slider,

        /// <summary>GPIO / relay output (pump, igniter, ...): momentary pulse or toggle.</summary>
        Relay,
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

        /// <summary>Cube servo output channel (Drop / Slider) or relay/GPIO number (Relay).</summary>
        public int Channel { get; set; } = 9;

        /// <summary>Servo low endpoint (Drop) / slider minimum (Slider), in microseconds.</summary>
        public int PwmMin { get; set; } = 1000;

        /// <summary>Servo high endpoint (Drop) / slider maximum (Slider), in microseconds.</summary>
        public int PwmMax { get; set; } = 2000;

        /// <summary>Slider default / resting position, in microseconds.</summary>
        public int PwmNeutral { get; set; } = 1500;

        /// <summary>Drop only: when true the servo drops at <see cref="PwmMin"/> instead of <see cref="PwmMax"/>.</summary>
        public bool Reversed { get; set; }

        /// <summary>Relay only: pulse length in ms. &gt;0 fires a momentary pulse; 0 latches an on/off toggle.</summary>
        public int PulseMs { get; set; } = 500;

        /// <summary>
        /// Relay only: optional RC pass-through input channel (5–16, 0 = disabled). When set, the
        /// operator can hold a transmitter switch on this channel to fire the relay through the flight
        /// controller directly. Settings → Payloads writes the matching <c>RC{n}_OPTION</c> on the Cube.
        /// </summary>
        public int RcChannel { get; set; }

        public PayloadControl Clone() => (PayloadControl)MemberwiseClone();
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
    }
}
