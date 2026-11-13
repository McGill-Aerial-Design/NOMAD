// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using MissionPlanner;
namespace NOMAD.MissionPlanner
{
    internal enum GimbalRequestOutcome { NotSent, Submitted, Unknown, Busy }
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
        private static long _servoTrackingGeneration = -1, _servoTrackingClaim = -1, _servoTrackingBinding = -1;
        private static string _servoTrackingInput;
        private static ServoGimbalAxis[] _servoTrackingAxes;
        internal static Func<MAVLinkInterface> CurrentConnection = () => MainV2.comPort;
        internal static event Action ConnectionChanged;
        internal static IReadOnlyList<ServoGimbalAxis> GimbalAxes
        { get { lock (Gate) { return _config?.SelectedGimbalAxes; } } }
        // Shared reservation for one admitted host helper, including its native retries.
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
        internal static async Task<GimbalRequestResult> SendGimbalTargetAsync(int mode, double? pitch, double? roll,
            double yaw, long generation, Func<bool> inputStillCurrent, string videoInputName = null,
            long? trackingClaim = null)
        {
            RefreshConnection();
            MAVLinkInterface connection;
            byte system, component;
            bool configure, acknowledgeServos;
            IReadOnlyList<ServoGimbalAxis> binding;
            ServoGimbalAxis[] axes;
            string inputName;
            long bindingRevision = GimbalController.BindingRevision;
            bool BindingCurrent() => _config != null && inputName == _config.SelectedVideoInputName &&
                bindingRevision == GimbalController.BindingRevision &&
                ReferenceEquals(binding, _config.SelectedGimbalAxes) && ServoGimbalAxis.Same(axes, _config.SelectedGimbalAxes);
            lock (Gate)
            {
                if (generation != _generation || _config == null || inputStillCurrent?.Invoke() == false)
                {
                    return Result(GimbalRequestOutcome.NotSent, "Gimbal input was retired; no command was sent.");
                }
                connection = _connection;
                if (connection == null || !_open || connection.ReadOnly || connection.logreadmode ||
                    _system < 1 || _system > 255 || _component < 1 || _component > 255)
                {
                    return Result(GimbalRequestOutcome.NotSent, "Select a live, writable Mission Planner aircraft.");
                }
                binding = _config.SelectedGimbalAxes;
                inputName = _config.SelectedVideoInputName;
                try
                {
                    ServoGimbalAxis.Validate(binding ?? Array.Empty<ServoGimbalAxis>());
                    axes = binding?.Select(axis => axis.Copy()).ToArray() ?? Array.Empty<ServoGimbalAxis>();
                }
                catch (Newtonsoft.Json.JsonException)
                { return Result(GimbalRequestOutcome.NotSent, "Review the servo gimbal calibration; no command was sent."); }
                bool custom = axes.Length != 0;
                if (custom && videoInputName != null && !string.Equals(videoInputName, inputName, StringComparison.OrdinalIgnoreCase))
                { return Result(GimbalRequestOutcome.NotSent, "The video input changed; no gimbal command was sent."); }
                if (mode < 0 || mode > 3 || pitch.HasValue != roll.HasValue || !Finite(yaw) ||
                    yaw < GimbalCommand.YAW_MIN_DEG || yaw > GimbalCommand.YAW_MAX_DEG ||
                    (!pitch.HasValue && yaw != 0) ||
                    (pitch.HasValue && mode != (int)MountMode.MavlinkTargeting) ||
                    (pitch.HasValue && (!Finite(pitch.Value) || !Finite(roll.Value))) ||
                    (!custom && pitch.HasValue && (
                    pitch < GimbalCommand.PITCH_MIN_DEG || pitch > GimbalCommand.PITCH_MAX_DEG ||
                    roll < GimbalCommand.ROLL_MIN_DEG || roll > GimbalCommand.ROLL_MAX_DEG)))
                {
                    return Result(GimbalRequestOutcome.NotSent, "Invalid gimbal mode or target; no command was sent.");
                }
                if (custom && (!pitch.HasValue ||
                    (!axes.Any(axis => axis.Axis == "Pitch") && pitch.Value != 0) ||
                    (!axes.Any(axis => axis.Axis == "Roll") && roll.Value != 0) ||
                    (!axes.Any(axis => axis.Axis == "Yaw") && yaw != 0) || axes.Any(axis =>
                    !axis.TryPwm(axis.Axis == "Pitch" ? pitch.Value : axis.Axis == "Roll" ? roll.Value : yaw, out _))))
                { return Result(GimbalRequestOutcome.NotSent, "Servo gimbals require calibrated angle targets; no command was sent."); }
                // Busy is returned only before invoking any native command helper.
                if (_payloadBusy || connection.giveComport)
                { return Result(GimbalRequestOutcome.Busy, "Mission Planner is busy; no gimbal command was sent."); }
                system = (byte)_system;
                component = (byte)_component;
                configure = _modeGeneration != generation || _submittedMode != mode;
                // Confirm each new tracking owner/binding once, then stream fresh absolute positions without ACK waits.
                acknowledgeServos = !trackingClaim.HasValue || _servoTrackingGeneration != generation ||
                    _servoTrackingClaim != trackingClaim.Value || _servoTrackingBinding != bindingRevision ||
                    _servoTrackingInput != inputName || !ServoGimbalAxis.Same(axes, _servoTrackingAxes);
                if (!trackingClaim.HasValue) { _servoTrackingClaim = -1; }
                _payloadBusy = true;
            }
            try
            {
                return await Task.Run(async () =>
                {
                    RefreshConnection();
                    lock (Gate)
                    {
                        if (generation != _generation || !BindingCurrent() || inputStillCurrent?.Invoke() == false ||
                            connection != _connection || connection != CurrentConnection() || connection.BaseStream != _stream ||
                            connection.BaseStream?.IsOpen != true || connection.ReadOnly || connection.logreadmode ||
                            connection.giveComport)
                        { return Result(GimbalRequestOutcome.NotSent, "Gimbal input was retired; no command was sent."); }
                    }
                    if (axes.Length != 0)
                    {
                        bool dispatched = false;
                        foreach (var axis in axes)
                        {
                            RefreshConnection();
                            lock (Gate)
                            {
                                if (generation != _generation || !BindingCurrent() || inputStillCurrent?.Invoke() == false ||
                                    connection != _connection || connection != CurrentConnection() || connection.BaseStream != _stream ||
                                    connection.BaseStream?.IsOpen != true || connection.ReadOnly || connection.logreadmode || connection.giveComport)
                                { return Result(dispatched ? GimbalRequestOutcome.Unknown : GimbalRequestOutcome.NotSent,
                                    "Servo gimbal input changed; already-dispatched commands may finish."); }
                            }
                            axis.TryPwm(axis.Axis == "Pitch" ? pitch.Value : axis.Axis == "Roll" ? roll.Value : yaw, out int pwm);
                            dispatched = true;
                            if (!await connection.doCommandAsync(system, component, MAVLink.MAV_CMD.DO_SET_SERVO,
                                axis.Channel, pwm, 0, 0, 0, 0, 0, acknowledgeServos).ConfigureAwait(false))
                            { return Result(GimbalRequestOutcome.Unknown, "Servo target not confirmed; check before retrying."); }
                        }
                        lock (Gate)
                        {
                            if (trackingClaim.HasValue && generation == _generation && BindingCurrent() &&
                                inputStillCurrent?.Invoke() != false)
                            {
                                _servoTrackingGeneration = generation; _servoTrackingClaim = trackingClaim.Value;
                                _servoTrackingBinding = bindingRevision; _servoTrackingInput = inputName;
                                _servoTrackingAxes = axes;
                            }
                        }
                        return Result(GimbalRequestOutcome.Submitted, acknowledgeServos
                            ? "Servo targets acknowledged; physical position not observed."
                            : "Servo targets sent; acceptance and physical position not observed.");
                    }
                    // Native helpers may emit before returning a Task; never invoke them on the UI or under Gate.
                    if (configure && !await connection.doCommandAsync(system, component,
                        MAVLink.MAV_CMD.DO_MOUNT_CONFIGURE, mode, 1, 1, 0, 0, 0, 0, false).ConfigureAwait(false))
                    { return Result(GimbalRequestOutcome.NotSent, "Mission Planner did not submit the mount command."); }
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
                            if (generation != _generation || !BindingCurrent() || inputStillCurrent?.Invoke() == false ||
                                connection != _connection || connection != CurrentConnection() || connection.BaseStream != _stream ||
                                connection.BaseStream?.IsOpen != true || connection.ReadOnly || connection.logreadmode ||
                                connection.giveComport)
                            { return Result(GimbalRequestOutcome.Unknown, "Mount mode may have been sent; target input was retired."); }
                        }
                        connection.setMountControl(system, component, pitch.Value * 100, roll.Value * 100, yaw * 100, false);
                    }
                    return Result(GimbalRequestOutcome.Submitted, "Command sent; acceptance not confirmed.");
                }).ConfigureAwait(false);
            }
            catch (Exception error)
            {
                return Result(GimbalRequestOutcome.Unknown, "Gimbal not confirmed. Check before retrying. " + error.Message);
            }
            finally { lock (Gate) { _payloadBusy = false; } }
        }
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        private static GimbalRequestResult Result(GimbalRequestOutcome outcome, string message) =>
            new GimbalRequestResult(outcome, message);
    }
}
