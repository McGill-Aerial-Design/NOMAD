using System;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    internal static class CubeOutputController
    {
        private static readonly SemaphoreSlim s_mavlinkLock = new SemaphoreSlim(1, 1);

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
                    1,
                    pwmUs,
                    (float)Math.Max(0.05, Math.Min(timeoutSeconds, 3.0)),
                    1, 0, 0,
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
                    1, 0, 0);

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
