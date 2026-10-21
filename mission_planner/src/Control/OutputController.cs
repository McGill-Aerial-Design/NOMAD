// SPDX-License-Identifier: Apache-2.0
using System;
using System.Threading.Tasks;
using MissionPlanner;
namespace NOMAD.MissionPlanner
{
    internal enum GimbalRequestOutcome { NotSent, Submitted, Unknown }
    internal sealed class GimbalRequestResult
    {
        internal GimbalRequestOutcome Outcome
        {
            get;
        }
        internal string Message
        {
            get;
        }
        internal bool Submitted => Outcome == GimbalRequestOutcome.Submitted;
        internal GimbalRequestResult(GimbalRequestOutcome outcome, string message)
        {
            Outcome = outcome;
            Message = message;
        }
    }
    // The host owns the stream. NOMAD never opens, closes or reads it.
    internal static partial class OutputController
    {
        private static readonly object Gate = new object();
        // MP 1.3.83 event logging dereferences Delegate.Target, so these handlers must have an instance target.
        private static readonly EventHandler ConnectionChangedHandler = new ConnectionEvents().OnChanged;
        private static NOMADConfig _config;
        private static MAVLinkInterface _connection;
        private static object _stream;
        private static int _system, _component;
        private static bool _open;
        private static bool _writable;
        private static long _generation;
        private static long _modeGeneration = -1;
        private static int _submittedMode = -1;
        internal static Func<MAVLinkInterface> CurrentConnection = () => MainV2.comPort;
        internal static event Action ConnectionChanged;
        private static bool _payloadBusy;
        internal static long Generation
        {
            get
            {
                RefreshConnection();
                lock (Gate)
                {
                    return _generation;
                }
            }
        }
        internal static bool IsCurrentProfile(NOMADConfig config)
        {
            lock (Gate)
            {
                return _config != null && ReferenceEquals(config, _config);
            }
        }
        internal static void Initialize(NOMADConfig config)
        {
            lock (Gate)
            {
                _config = config;
                _generation++;
            }
            RefreshConnection();
        }
        internal static void RefreshConnection()
        {
            lock (Gate)
            {
                if (_config == null)
                {
                    return;
                }
                var connection = CurrentConnection();
                object stream = connection?.BaseStream;
                int system = connection?.sysidcurrent ?? 0;
                int component = connection?.compidcurrent ?? 0;
                bool open = connection?.BaseStream?.IsOpen == true;
                bool writable = connection != null && !connection.ReadOnly && !connection.logreadmode;
                if (connection == _connection && stream == _stream && system == _system &&
                    component == _component && open == _open && writable == _writable)
                    {
                        return;
                    }
                Unsubscribe();
                _connection = connection;
                _stream = stream;
                _system = system;
                _component = component;
                _open = open;
                _writable = writable;
                _generation++;
                if (_connection != null)
                {
                    _connection.MavChanged += ConnectionChangedHandler;
                    _connection.CommsClose += ConnectionChangedHandler;
                }
            }
            ConnectionChanged?.Invoke();
        }
        private sealed class ConnectionEvents
        {
            internal void OnChanged(object sender, EventArgs args)
            {
                lock (Gate)
                {
                    _generation++;
                }
                ConnectionChanged?.Invoke();
            }
        }
        private static void Unsubscribe()
        {
            if (_connection == null)
            {
                return;
            }
            _connection.MavChanged -= ConnectionChangedHandler;
            _connection.CommsClose -= ConnectionChangedHandler;
        }
        internal static void Shutdown()
        {
            lock (Gate)
            {
                Unsubscribe();
                _config = null;
                _connection = null;
                _stream = null;
                _generation++;
            }
        }
        internal static async Task<GimbalRequestResult> SendGimbalAsync(int mode, double? pitch, double? roll,
            long generation, Func<bool> inputStillCurrent)
        {
            RefreshConnection();
            MAVLinkInterface connection;
            byte system, component;
            Task<bool> configure;
            lock (Gate)
            {
                if (generation != _generation || _config == null || _payloadBusy || inputStillCurrent?.Invoke() == false)
                {
                    return Result(GimbalRequestOutcome.NotSent, "Gimbal input was retired; no command was sent.");
                }
                connection = _connection;
                if (connection == null || !_open || connection.ReadOnly || connection.logreadmode || connection.giveComport ||
                    _system < 1 || _system > 255 || _component < 1 || _component > 255)
                {
                    return Result(GimbalRequestOutcome.NotSent, "Select a live, writable Mission Planner aircraft.");
                }
                if (mode < 0 || mode > 3 || pitch.HasValue != roll.HasValue ||
                    (pitch.HasValue && (!Finite(pitch.Value) || !Finite(roll.Value) ||
                    pitch < GimbalCommand.PITCH_MIN_DEG || pitch > GimbalCommand.PITCH_MAX_DEG ||
                    roll < GimbalCommand.ROLL_MIN_DEG || roll > GimbalCommand.ROLL_MAX_DEG)))
                {
                    return Result(GimbalRequestOutcome.NotSent, "Invalid gimbal mode or target; no command was sent.");
                }
                system = (byte)_system;
                component = (byte)_component;
                try
                {
                    // The no-ACK host path emits synchronously before returning its completed task.
                    configure = _modeGeneration == generation && _submittedMode == mode
                        ? Task.FromResult(true)
                        : connection.doCommandAsync(system, component, MAVLink.MAV_CMD.DO_MOUNT_CONFIGURE,
                            mode, 1, 1, 0, 0, 0, 0, false);
                }
                catch (Exception error)
                {
                    return Result(GimbalRequestOutcome.Unknown,
                        "Gimbal outcome is unknown; do not retry blindly. " + error.Message);
                }
            }
            try
            {
                if (!await configure.ConfigureAwait(false))
                {
                    return Result(GimbalRequestOutcome.NotSent, "Mission Planner did not submit the mount command.");
                }
                lock (Gate)
                {
                    if (generation == _generation)
                    {
                        _modeGeneration = generation;
                        _submittedMode = mode;
                    }
                }
                if (pitch.HasValue)
                {
                    RefreshConnection();
                    lock (Gate)
                    {
                        if (generation != _generation || _payloadBusy || inputStillCurrent?.Invoke() == false ||
                            connection != CurrentConnection() || connection.BaseStream != _stream ||
                            connection.BaseStream?.IsOpen != true || connection.ReadOnly || connection.logreadmode || connection.giveComport)
                        {
                            return Result(GimbalRequestOutcome.Unknown, "Mount mode may have been sent; target input was retired.");
                        }
                        connection.setMountControl(system, component, pitch.Value * 100, roll.Value * 100, 0, false);
                    }
                }
                return Result(GimbalRequestOutcome.Submitted,
                    "Gimbal command submitted to Mission Planner. Vehicle acceptance and physical state are unverified.");
            }
            catch (Exception error)
            {
                return Result(GimbalRequestOutcome.Unknown, "Gimbal outcome is unknown; do not retry blindly. " + error.Message);
            }
        }
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        private static GimbalRequestResult Result(GimbalRequestOutcome outcome, string message) =>
            new GimbalRequestResult(outcome, message);
    }
}
