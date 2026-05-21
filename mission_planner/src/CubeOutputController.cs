using System;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Centralized Cube Orange servo and relay command path.
    /// Uses the local Mission Planner MAVLink link first, then falls back to
    /// Edge Core, which sends the same MAVLink commands from the Jetson.
    /// </summary>
    internal static class CubeOutputController
    {
        private static readonly SemaphoreSlim s_mavlinkLock = new SemaphoreSlim(1, 1);
        private const int DroneCanBeepCommandDataTypeId = 1080;
        private const int DroneCanBeepPriority = 20;
        private static byte s_droneCanBeepTransferId;

        internal static SemaphoreSlim MavlinkLock => s_mavlinkLock;

        public static bool TrySendServoMavlink(int channel, int pwmUs, bool tryOnly = false)
        {
            if (channel <= 0) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            byte sysid = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;

            Task.Run(async () =>
            {
                bool acquired = tryOnly
                    ? await s_mavlinkLock.WaitAsync(0).ConfigureAwait(false)
                    : await s_mavlinkLock.WaitAsync(5000).ConfigureAwait(false);
                if (!acquired) return;
                try
                {
                    await MainV2.comPort.doCommandAsync(
                        sysid, compid,
                        MAVLink.MAV_CMD.DO_SET_SERVO,
                        channel, pwmUs, 0, 0, 0, 0, 0,
                        requireack: false, uicallback: null).ConfigureAwait(false);
                }
                catch { }
                finally
                {
                    s_mavlinkLock.Release();
                }
            });

            return true;
        }

        public static async Task<bool> SendServoPwmAsync(int channel, int pwmUs, bool tryOnly = false)
        {
            if (channel <= 0 || pwmUs < 500 || pwmUs > 2500) return false;
            if (TrySendServoMavlink(channel, pwmUs, tryOnly)) return true;

            try
            {
                var resp = await JetsonApiService.PostAsync(
                    $"/api/servo/channel/{channel}/pwm?pwm={pwmUs}").ConfigureAwait(false);
                return resp.IsSuccessStatusCode;
            }
            catch
            {
                return false;
            }
        }

        public static async Task<bool> SendMotorTestPwmAsync(int motorInstance, int pwmUs, double timeoutSeconds, bool tryOnly = false)
        {
            if (motorInstance <= 0) return false;
            if (pwmUs != 0 && (pwmUs < 500 || pwmUs > 2500)) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            bool acquired = false;
            try
            {
                byte sysid = MainV2.comPort.MAV.sysid;
                byte compid = MainV2.comPort.MAV.compid;

                acquired = tryOnly
                    ? await s_mavlinkLock.WaitAsync(0).ConfigureAwait(false)
                    : await s_mavlinkLock.WaitAsync(1500).ConfigureAwait(false);
                if (!acquired) return false;

                await MainV2.comPort.doCommandAsync(
                    sysid, compid,
                    MAVLink.MAV_CMD.DO_MOTOR_TEST,
                    motorInstance,
                    1, // MOTOR_TEST_THROTTLE_PWM
                    pwmUs,
                    (float)Math.Max(0.05, Math.Min(timeoutSeconds, 3.0)),
                    1,
                    0,
                    0,
                    requireack: false, uicallback: null).ConfigureAwait(false);

                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (acquired) s_mavlinkLock.Release();
            }
        }

        public static bool SendMotorTestPwm(int motorInstance, int pwmUs, double timeoutSeconds)
        {
            if (motorInstance <= 0) return false;
            if (pwmUs != 0 && (pwmUs < 500 || pwmUs > 2500)) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            bool acquired = false;
            try
            {
                byte sysid = MainV2.comPort.MAV.sysid;
                byte compid = MainV2.comPort.MAV.compid;

                acquired = s_mavlinkLock.Wait(1500);
                if (!acquired) return false;

                MainV2.comPort.doCommand(
                    sysid, compid,
                    MAVLink.MAV_CMD.DO_MOTOR_TEST,
                    motorInstance,
                    1,
                    pwmUs,
                    (float)Math.Max(0.05, Math.Min(timeoutSeconds, 3.0)),
                    1,
                    0,
                    0);

                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (acquired) s_mavlinkLock.Release();
            }
        }

        public static async Task<bool> EnableCanForwardAsync(int bus, bool enable)
        {
            if (bus < 1 || bus > 3) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            bool acquired = false;
            try
            {
                byte sysid = MainV2.comPort.MAV.sysid;
                byte compid = MainV2.comPort.MAV.compid;

                acquired = await s_mavlinkLock.WaitAsync(1500).ConfigureAwait(false);
                if (!acquired) return false;

                int busParam = enable ? bus : 0;

                await MainV2.comPort.doCommandIntAsync(
                    sysid, compid,
                    MAVLink.MAV_CMD.CAN_FORWARD,
                    0, 0, 0, 0,
                    busParam, 0, 0,
                    requireack: false, uicallback: null,
                    frame: MAVLink.MAV_FRAME.GLOBAL).ConfigureAwait(false);

                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (acquired) s_mavlinkLock.Release();
            }
        }

        public static bool EnableCanForward(int bus, bool enable)
        {
            if (bus < 1 || bus > 3) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            bool acquired = false;
            try
            {
                byte sysid = MainV2.comPort.MAV.sysid;
                byte compid = MainV2.comPort.MAV.compid;

                acquired = s_mavlinkLock.Wait(1500);
                if (!acquired) return false;

                int busParam = enable ? bus : 0;

                MainV2.comPort.doCommandInt(
                    sysid, compid,
                    MAVLink.MAV_CMD.CAN_FORWARD,
                    0, 0, 0, 0,
                    busParam, 0, 0,
                    false, null, MAVLink.MAV_FRAME.GLOBAL);

                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (acquired) s_mavlinkLock.Release();
            }
        }

        public static async Task<bool> SendDroneCanBeepAsync(int bus, int sourceNodeId, double frequencyHz, double durationSeconds, bool tryOnly = false)
        {
            return await Task.Run(() => SendDroneCanBeep(bus, sourceNodeId, frequencyHz, durationSeconds, tryOnly)).ConfigureAwait(false);
        }

        public static bool SendDroneCanBeep(int bus, int sourceNodeId, double frequencyHz, double durationSeconds, bool tryOnly = false)
        {
            if (bus < 1 || bus > 3) return false;
            if (sourceNodeId < 1 || sourceNodeId > 127) return false;
            if (frequencyHz < 20.0 || frequencyHz > 20000.0) return false;
            if (durationSeconds <= 0.0 || durationSeconds > 5.0) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            bool acquired = false;
            try
            {
                byte sysid = MainV2.comPort.MAV.sysid;
                byte compid = MainV2.comPort.MAV.compid;

                acquired = tryOnly
                    ? s_mavlinkLock.Wait(0)
                    : s_mavlinkLock.Wait(1500);
                if (!acquired) return false;

                byte transferId = s_droneCanBeepTransferId;
                s_droneCanBeepTransferId = (byte)((s_droneCanBeepTransferId + 1) & 0x1F);

                var frame = new MAVLink.mavlink_can_frame_t
                {
                    target_system = sysid,
                    target_component = compid,
                    bus = (byte)(bus - 1),
                    len = 8,
                    id = BuildDroneCanMessageId(DroneCanBeepPriority, DroneCanBeepCommandDataTypeId, sourceNodeId),
                    data = new byte[8],
                };

                WriteFloat16Le(frame.data, 0, frequencyHz);
                WriteFloat16Le(frame.data, 2, durationSeconds);
                frame.data[4] = (byte)(0xC0 | transferId);

                MainV2.comPort.sendPacket(frame, sysid, compid);
                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (acquired) s_mavlinkLock.Release();
            }
        }

        private static uint BuildDroneCanMessageId(int priority, int dataTypeId, int sourceNodeId)
        {
            return ((uint)(priority & 0x1F) << 24)
                | ((uint)(dataTypeId & 0xFFFF) << 8)
                | (uint)(sourceNodeId & 0x7F);
        }

        private static void WriteFloat16Le(byte[] buffer, int offset, double value)
        {
            ushort half = FloatToHalf((float)value);
            buffer[offset] = (byte)(half & 0xFF);
            buffer[offset + 1] = (byte)(half >> 8);
        }

        private static ushort FloatToHalf(float value)
        {
            uint bits = BitConverter.ToUInt32(BitConverter.GetBytes(value), 0);
            uint sign = (bits >> 16) & 0x8000;
            int exponent = (int)((bits >> 23) & 0xFF) - 127 + 15;
            uint mantissa = bits & 0x7FFFFF;

            if (exponent <= 0)
            {
                if (exponent < -10) return (ushort)sign;
                mantissa = (mantissa | 0x800000) >> (1 - exponent);
                return (ushort)(sign | ((mantissa + 0x1000) >> 13));
            }

            if (exponent >= 31)
            {
                return (ushort)(sign | 0x7C00);
            }

            return (ushort)(sign | ((uint)exponent << 10) | ((mantissa + 0x1000) >> 13));
        }

        public static bool TrySetRelayMavlink(int relayNumber, bool on)
        {
            if (relayNumber < 0) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            byte sysid = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;

            Task.Run(async () =>
            {
                await s_mavlinkLock.WaitAsync().ConfigureAwait(false);
                try
                {
                    await MainV2.comPort.doCommandAsync(
                        sysid, compid,
                        MAVLink.MAV_CMD.DO_SET_RELAY,
                        relayNumber, on ? 1 : 0, 0, 0, 0, 0, 0,
                        requireack: true, uicallback: null).ConfigureAwait(false);
                }
                catch { }
                finally
                {
                    s_mavlinkLock.Release();
                }
            });

            return true;
        }

        public static async Task<bool> FireRelayAsync(int relayNumber, int durationMs)
        {
            if (relayNumber < 0) return false;
            durationMs = Math.Max(50, Math.Min(durationMs, 5000));
            if (TrySetRelayMavlink(relayNumber, true))
            {
                await Task.Delay(durationMs).ConfigureAwait(false);
                TrySetRelayMavlink(relayNumber, false);
                return true;
            }

            try
            {
                var resp = await JetsonApiService.PostAsync(
                    $"/api/servo/shooter/trigger?duration_ms={durationMs}&relay_number={relayNumber}"
                ).ConfigureAwait(false);
                return resp.IsSuccessStatusCode;
            }
            catch
            {
                return false;
            }
        }
    }
}
