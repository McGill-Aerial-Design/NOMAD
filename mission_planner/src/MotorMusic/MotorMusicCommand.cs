// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using MissionPlanner.ArduPilot.Mavlink;

namespace NOMAD.MissionPlanner
{
    internal enum MotorMusicOpcode
    {
        Note = 1,
        Stop = 2,
        Ping = 3,
    }

    internal sealed class MotorMusicCommandOptions
    {
        public int MotorCount { get; set; } = 4;
        public int MinOutputPwm { get; set; } = 1100;
        public int MaxOutputPwm { get; set; } = 1800;
        public int Transpose { get; set; } = -24;
        public double TempoScale { get; set; } = 1.0;
    }

    internal static class MotorMusicCommand
    {
        public const string ScriptPath = "APM/scripts/nomad_motor_music.lua";
        private const int CommandId = 31010; // MAV_CMD_USER_1

        public static bool IsConnected
        {
            get
            {
                try { return MainV2.comPort != null && MainV2.comPort.BaseStream.IsOpen; }
                catch { return false; }
            }
        }

        public static async Task<bool> SendNoteAsync(
            int motorSlot,
            int midiNote,
            int velocity,
            int durationMs,
            int maxOutputPwm,
            int minOutputPwm)
        {
            return await SendAsync(
                MotorMusicOpcode.Note,
                motorSlot,
                midiNote,
                velocity,
                durationMs,
                maxOutputPwm,
                minOutputPwm).ConfigureAwait(false);
        }

        public static async Task<bool> SendStopAsync()
        {
            return await SendAsync(MotorMusicOpcode.Stop, 0, 0, 0, 0, 0, 0).ConfigureAwait(false);
        }

        public static async Task<bool> SendPingAsync()
        {
            return await SendAsync(MotorMusicOpcode.Ping, 0, 0, 0, 0, 0, 0).ConfigureAwait(false);
        }

        public static async Task<string> InstallScriptAsync(CancellationToken token)
        {
            if (!IsConnected) return "Vehicle link is not open.";

            bool acquired = false;
            try
            {
                acquired = await CubeOutputController.MavlinkLock.WaitAsync(10000, token).ConfigureAwait(false);
                if (!acquired) return "MAVLink link is busy.";

                var sysid = MainV2.comPort.MAV.sysid;
                var compid = MainV2.comPort.MAV.compid;
                var ftp = new MAVFtp(MainV2.comPort, sysid, compid);
                using (var cts = CancellationTokenSource.CreateLinkedTokenSource(token))
                using (var stream = new MemoryStream(Encoding.ASCII.GetBytes(MotorMusicLuaScript.Text)))
                {
                    ftp.kCmdCreateDirectory("APM", cts);
                    ftp.kCmdCreateDirectory("APM/scripts", cts);
                    ftp.UploadFile(ScriptPath, stream, cts);
                }

                TrySetParam("SCR_ENABLE", 1);
                return "Installed APM/scripts/nomad_motor_music.lua and set SCR_ENABLE=1. " +
                       "Reboot or restart scripting before use.";
            }
            catch (Exception ex)
            {
                return "Install failed: " + ex.Message;
            }
            finally
            {
                if (acquired) CubeOutputController.MavlinkLock.Release();
            }
        }

        private static async Task<bool> SendAsync(
            MotorMusicOpcode opcode,
            int motorSlot,
            int midiNote,
            int velocity,
            int durationMs,
            int maxOutputPwm,
            int minOutputPwm)
        {
            if (!IsConnected) return false;

            bool acquired = false;
            try
            {
                byte sysid = MainV2.comPort.MAV.sysid;
                byte compid = MainV2.comPort.MAV.compid;

                acquired = await CubeOutputController.MavlinkLock.WaitAsync(500).ConfigureAwait(false);
                if (!acquired) return false;

                await MainV2.comPort.doCommandAsync(
                    sysid,
                    compid,
                    (MAVLink.MAV_CMD)CommandId,
                    (float)opcode,
                    motorSlot,
                    midiNote,
                    velocity,
                    durationMs,
                    maxOutputPwm,
                    minOutputPwm,
                    requireack: false,
                    uicallback: null).ConfigureAwait(false);
                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (acquired) CubeOutputController.MavlinkLock.Release();
            }
        }

        private static bool TrySetParam(string name, double value)
        {
            try
            {
                var methods = MainV2.comPort.GetType().GetMethods();
                foreach (var method in methods)
                {
                    if (method.Name != "setParam") continue;
                    var p = method.GetParameters();
                    if (p.Length < 2 || p[0].ParameterType != typeof(string)) continue;

                    var args = new object[p.Length];
                    args[0] = name;
                    args[1] = Convert.ChangeType(value, p[1].ParameterType);
                    for (int i = 2; i < p.Length; i++)
                    {
                        if (p[i].ParameterType == typeof(bool)) args[i] = true;
                        else if (p[i].HasDefaultValue) args[i] = p[i].DefaultValue;
                        else args[i] = p[i].ParameterType.IsValueType
                            ? Activator.CreateInstance(p[i].ParameterType)
                            : null;
                    }

                    method.Invoke(MainV2.comPort, args);
                    return true;
                }
            }
            catch (Exception ex)
            {
                Log.Error($"MotorMusic setParam({name}) failed - {ex.Message}");
            }
            return false;
        }
    }
}
