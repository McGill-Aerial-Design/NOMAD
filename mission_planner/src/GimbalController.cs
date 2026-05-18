// ============================================================
// NOMAD Caddx Gimbal Controller (shared MAVLink sender)
// ============================================================
// Extracted from GimbalJoystickWindow so any input source — the
// floating joystick window, the NomadJoystickService driven by a
// physical DirectInput stick, or a future scripted automation —
// can drive the same target-angle integrator and serialize MAVLink
// access against PayloadControlPanel via CubeOutputController.MavlinkLock.
// ============================================================

using System;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Process-wide gimbal target state and MAVLink send helpers.
    /// All state is static so multiple input sources see the same target.
    /// </summary>
    public static class GimbalController
    {
        // Caddx mount limits.
        public const float PITCH_MIN_DEG = -90f;
        public const float PITCH_MAX_DEG =  90f;
        public const float ROLL_MIN_DEG  = -30f;
        public const float ROLL_MAX_DEG  =  30f;

        public enum MountMode
        {
            Retract = 0,
            Neutral = 1,
            MavlinkTargeting = 2,
            RcTargeting = 3,
        }

        // Integrated target angles in deg. Visible to UI components that want
        // to display the current target alongside their own controls.
        public static float TargetPitchDeg { get; private set; }
        public static float TargetRollDeg { get; private set; }

        public static MountMode CurrentMode { get; private set; } = MountMode.MavlinkTargeting;

        /// <summary>
        /// Fires whenever an input source moves the integrated target. Subscribers
        /// (e.g. GimbalJoystickWindow) update their on-screen readouts.
        /// </summary>
        public static event Action<float, float> TargetChanged;

        /// <summary>Fires when the mount mode preset changes.</summary>
        public static event Action<MountMode> ModeChanged;

        private static int _inflight; // 0/1 — Interlocked.Exchange guards.

        public static void SetTargetAngles(float pitchDeg, float rollDeg)
        {
            TargetPitchDeg = Clamp(pitchDeg, PITCH_MIN_DEG, PITCH_MAX_DEG);
            TargetRollDeg  = Clamp(rollDeg,  ROLL_MIN_DEG,  ROLL_MAX_DEG);
            TargetChanged?.Invoke(TargetPitchDeg, TargetRollDeg);
        }

        /// <summary>
        /// Integrate a normalized stick reading over dt and (optionally) push
        /// the new angle to the mount. The stick convention matches the
        /// floating joystick: +stickY = pitch up, +stickX = roll right.
        /// </summary>
        public static void ApplyStick(float stickX, float stickY, float maxRateDegSec, float dt, bool send)
        {
            float dPitch =  stickY * maxRateDegSec * dt;
            float dRoll  = -stickX * maxRateDegSec * dt;
            float p = Clamp(TargetPitchDeg + dPitch, PITCH_MIN_DEG, PITCH_MAX_DEG);
            float r = Clamp(TargetRollDeg  + dRoll,  ROLL_MIN_DEG,  ROLL_MAX_DEG);
            if (p == TargetPitchDeg && r == TargetRollDeg) return;
            TargetPitchDeg = p;
            TargetRollDeg = r;
            TargetChanged?.Invoke(p, r);
            if (send) SendPitchRollAngle(p, r);
        }

        public static void SetMode(MountMode mode)
        {
            CurrentMode = mode;
            ModeChanged?.Invoke(mode);
            SendMountConfigure(mode);
        }

        /// <summary>
        /// Send DO_MOUNT_CONTROL with absolute pitch/roll. Drops if a previous
        /// command is still in flight so fast stick motion never queues.
        /// </summary>
        public static void SendPitchRollAngle(float pitchDeg, float rollDeg)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return;
            if (System.Threading.Interlocked.Exchange(ref _inflight, 1) == 1) return;

            byte sysid  = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;

            Task.Run(async () =>
            {
                bool acquired = false;
                try
                {
                    acquired = await CubeOutputController.MavlinkLock
                        .WaitAsync(1000).ConfigureAwait(false);
                    if (!acquired) return;
                    await MainV2.comPort.doCommandAsync(
                        sysid, compid,
                        MAVLink.MAV_CMD.DO_MOUNT_CONTROL,
                        pitchDeg, rollDeg,
                        0f, 0f,
                        0f, 0f, 2f,
                        requireack: false, uicallback: null).ConfigureAwait(false);
                }
                catch { }
                finally
                {
                    if (acquired) CubeOutputController.MavlinkLock.Release();
                    System.Threading.Interlocked.Exchange(ref _inflight, 0);
                }
            });
        }

        public static void SendMountConfigure(MountMode mode)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return;

            byte sysid = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;

            Task.Run(async () =>
            {
                bool acquired = false;
                try
                {
                    acquired = await CubeOutputController.MavlinkLock
                        .WaitAsync(2000).ConfigureAwait(false);
                    if (!acquired) return;
                    await MainV2.comPort.doCommandAsync(
                        sysid, compid,
                        MAVLink.MAV_CMD.DO_MOUNT_CONFIGURE,
                        (float)mode,
                        1f, 1f, 1f,
                        2f, 2f, 2f,
                        requireack: false, uicallback: null).ConfigureAwait(false);
                }
                catch { }
                finally
                {
                    if (acquired) CubeOutputController.MavlinkLock.Release();
                }
            });
        }

        private static float Clamp(float v, float min, float max)
            => v < min ? min : v > max ? max : v;
    }
}
