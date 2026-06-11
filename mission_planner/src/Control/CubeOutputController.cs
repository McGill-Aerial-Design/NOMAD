// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
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

        internal sealed class MotorTestCommand
        {
            public readonly int MotorInstance;
            public readonly int PwmUs;
            public readonly double TimeoutSeconds;

            public MotorTestCommand(int motorInstance, int pwmUs, double timeoutSeconds)
            {
                MotorInstance = motorInstance;
                PwmUs = pwmUs;
                TimeoutSeconds = timeoutSeconds;
            }
        }

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
            if (!IsValidMotorTestCommand(motorInstance, pwmUs)) return false;
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

                await SendMotorTestPwmLockedAsync(sysid, compid, motorInstance, pwmUs, timeoutSeconds)
                    .ConfigureAwait(false);
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
            return SendMotorTestPwmAsync(motorInstance, pwmUs, timeoutSeconds).GetAwaiter().GetResult();
        }

        public static bool SendMotorTestPwmBatch(IEnumerable<MotorTestCommand> commands)
        {
            if (commands == null) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            var list = new List<MotorTestCommand>();
            foreach (var command in commands)
            {
                if (command == null) continue;
                if (!IsValidMotorTestCommand(command.MotorInstance, command.PwmUs)) return false;
                list.Add(command);
            }
            if (list.Count == 0) return true;

            bool acquired = false;
            try
            {
                byte sysid = MainV2.comPort.MAV.sysid;
                byte compid = MainV2.comPort.MAV.compid;

                acquired = s_mavlinkLock.Wait(1500);
                if (!acquired) return false;

                foreach (var command in list)
                {
                    SendMotorTestPwmLockedAsync(
                        sysid,
                        compid,
                        command.MotorInstance,
                        command.PwmUs,
                        command.TimeoutSeconds).GetAwaiter().GetResult();
                }

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

        private static bool IsValidMotorTestCommand(int motorInstance, int pwmUs)
        {
            if (motorInstance <= 0) return false;
            return pwmUs == 0 || (pwmUs >= 500 && pwmUs <= 2500);
        }

        private static Task SendMotorTestPwmLockedAsync(
            byte sysid,
            byte compid,
            int motorInstance,
            int pwmUs,
            double timeoutSeconds)
        {
            return MainV2.comPort.doCommandAsync(
                sysid, compid,
                MAVLink.MAV_CMD.DO_MOTOR_TEST,
                motorInstance,
                1,
                pwmUs,
                (float)Math.Max(0.05, Math.Min(timeoutSeconds, 3.0)),
                1, 0, 0,
                requireack: false, uicallback: null);
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
                // SR-PAY-03: direct GCS-to-FC relay output bypasses the Edge
                // Core interlock by design; the panel's armed click or the
                // transmitter switch is the operator interlock documented in
                // docs/safety/requirements.md and docs/safety/hazards.md.
                await Task.Delay(durationMs).ConfigureAwait(false);
                TrySetRelayMavlink(relayNumber, false);
                return true;
            }

            try
            {
                // The edge-side release interlock (SR-PAY-03) requires an
                // explicit arm immediately before the trigger; the operator's
                // confirm already happened in the UI / on the transmitter.
                var armResp = await JetsonApiService.PostAsync("/api/servo/shooter/arm").ConfigureAwait(false);
                if (!armResp.IsSuccessStatusCode) return false;
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
