// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;

public class MAVLink
{
    public enum MAV_CMD { DO_MOUNT_CONFIGURE, DO_SET_SERVO, DO_SET_RELAY, DO_REPEAT_SERVO, DO_REPEAT_RELAY }
}
namespace MissionPlanner
{
    public static class MainV2 { public static MAVLinkInterface comPort; }
    public sealed class TestStream { public bool IsOpen = true; }
    public sealed class NativeCommand
    {
        public MAVLink.MAV_CMD Kind;
        public byte System, Component;
        public float First, Second, Third, Fourth;
        public bool Ack;
    }
    public sealed class MAVLinkInterface
    {
        public TestStream BaseStream = new TestStream();
        public int sysidcurrent = 4, compidcurrent = 2;
        public bool ReadOnly, logreadmode, giveComport;
        public event EventHandler MavChanged;
        public event EventHandler CommsClose;
        public void Close() { CommsClose?.Invoke(this, EventArgs.Empty); }
        public void Select() { MavChanged?.Invoke(this, EventArgs.Empty); }
        public readonly List<NativeCommand> Commands = new List<NativeCommand>();
        public Func<NativeCommand, Task<bool>> Send = command => Task.FromResult(true);
        public Func<Task<float>> Trim = () => Task.FromResult(1500f);
        public int Reads;
        public byte TrimSystem, TrimComponent;
        public string Parameter;
        public Task<float> GetParamAsync(byte system, byte component, string name)
        { Reads++; TrimSystem = system; TrimComponent = component; Parameter = name; return Trim(); }
        public Task<bool> setParamAsync(byte system, byte component, string name, double value, bool force) =>
            throw new InvalidOperationException("Payload fixtures do not write aircraft parameters.");
        public Task<bool> doCommandAsync(byte system, byte component, MAVLink.MAV_CMD kind,
            float p1, float p2, float p3, float p4, float p5, float p6, float p7, bool requireack)
        {
            var command = new NativeCommand { Kind = kind, System = system, Component = component,
                First = p1, Second = p2, Third = p3, Fourth = p4, Ack = requireack };
            lock (Commands) { Commands.Add(command); }
            return Send(command);
        }
        public void setMountControl(byte system, byte component, double pitch, double roll, double yaw, bool coordinates) { }
    }
}
namespace MissionPlanner.Joystick
{
    public interface IMyJoystickState
    {
        int X { get; } int Y { get; } int Z { get; } int Rx { get; } int Ry { get; } int Rz { get; }
        int[] GetSlider(); bool[] GetButtons();
    }
    public sealed class JoystickBase : IDisposable
    {
        public static IList<string> getDevices() => new List<string>();
        public static JoystickBase Create(Func<MissionPlanner.MAVLinkInterface> connection) => new JoystickBase();
        public bool AcquireJoystick(string device) => true;
        public IMyJoystickState GetCurrentState() => null;
        public void UnAcquireJoyStick() { }
        public void Dispose() { }
    }
}
namespace NOMAD.MissionPlanner
{
    public sealed partial class NOMADConfig
    {
        public JArray ActuatorDefinitions;
        public int GimbalMountMode = 2;
        public float JoystickGimbalMaxRateDegSec = 60;
        public JArray Inputs = new JArray();
        internal bool HasSameOutputSettings(NOMADConfig other) => JToken.DeepEquals(ActuatorDefinitions, other.ActuatorDefinitions);
    }
    public static class Log
    {
        public static void Warn(string text) { }
        public static void Error(string text) { }
        public static void Debug(string text) { }
    }
}

internal static class PayloadTests
{
    private static void Check(bool condition, string diagnostic)
    { if (!condition) { throw new Exception(diagnostic); } }
    private static JObject Definition(string id, string behavior, int channel, int safe = 1000, int confirmations = 0) =>
        new JObject { ["id"] = id, ["name"] = id, ["behavior"] = behavior, ["channel"] = channel,
            ["safe_pwm"] = safe, ["hazardous"] = confirmations >= 2, ["confirmation_count"] = confirmations,
            ["pulse_ms"] = 50, ["hold_ms"] = 50 };
    private static (NOMADConfig Config, MissionPlanner.MAVLinkInterface Connection, PayloadController Owner) Reset(params JObject[] definitions)
    {
        PayloadController.Shutdown(); OutputController.Shutdown();
        var config = new NOMADConfig { ActuatorDefinitions = new JArray(definitions) };
        var connection = new MissionPlanner.MAVLinkInterface();
        MissionPlanner.MainV2.comPort = connection;
        OutputController.Initialize(config); PayloadController.ApplyProfile(config);
        return (config, connection, PayloadController.Current);
    }
    private static string Act(PayloadController owner, string id, string operation, double? value = null) =>
        owner.ActAsync(id, operation, value).GetAwaiter().GetResult();
    private static void Wait(Task task)
    { Check(task.Wait(5000), "Native test deadline exceeded"); }
    private sealed class Sample : MissionPlanner.Joystick.IMyJoystickState
    {
        internal bool[] Buttons = new bool[6];
        internal int Raw = 32768;
        public int X => Raw;
        public int Y => 32768; public int Z => 32768; public int Rx => 32768; public int Ry => 32768; public int Rz => 32768;
        public int[] GetSlider() => new[] { 32768, 32768 };
        public bool[] GetButtons() => Buttons;
    }
    private static void Map(NOMADConfig config, string action, string other = "")
    {
        config.Inputs = new JArray(JObject.FromObject(new InputBinding { Id = "test_input", Name = "Test", Device = "test",
            Enabled = true, Source = other == "" ? "Button" : "ButtonPair", Action = action, OtherAction = other }));
    }
    private static void MapPosition(NOMADConfig config)
    {
        config.Inputs = new JArray(JObject.FromObject(new InputBinding { Id = "position_input", Name = "Position", Device = "test",
            Enabled = true, Source = "Axis", Action = "position:position" }));
    }
    private static void Buttons(NomadJoystickService service, bool[] buttons) => service.TranslateInputs(
        new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = buttons == null ? null : new Sample { Buttons = buttons } });
    private static void Position(NomadJoystickService service, bool valid, float value) => service.TranslateInputs(
        new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = valid ? new Sample { Raw = (int)Math.Round((value + 1) * 32767.5) } : null });
    private static void InvalidDefinitions()
    {
        var valid = Definition("servo", "ServoToggle", 8);
        var cases = new List<JToken> { new JObject(), new JArray(1), new JArray(valid, valid.DeepClone()) };
        foreach (var mutation in new Action<JObject>[] {
            item => item["channel"] = "8", item => item["channel"] = 0, item => item["safe_pwm"] = 1500,
            item => item["pwm_max"] = 3000, item => item["pulse_ms"] = 1501,
            item => item["confirmation_count"] = 1.5, item => item["unknown"] = 1,
            item => { item["hazardous"] = true; item["confirmation_count"] = 0; } })
        { var item = (JObject)valid.DeepClone(); mutation(item); cases.Add(new JArray(item)); }
        foreach (var item in cases)
        {
            bool rejected = false;
            try { PayloadDefinition.Parse(item); } catch (Newtonsoft.Json.JsonException) { rejected = true; }
            Check(rejected, "Invalid typed/calibrated payload definition was accepted: " + item);
        }
        var duplicate = Definition("other", "ServoPosition", 8);
        bool duplicateRejected = false;
        try { PayloadDefinition.Parse(new JArray(valid, duplicate)); } catch { duplicateRejected = true; }
        Check(duplicateRejected, "Duplicate physical output was accepted");
    }
    private static void NativeMappings()
    {
        var fixture = Reset(Definition("servo", "ServoToggle", 8), Definition("position", "ServoPosition", 9, 1500),
            Definition("winch", "ServoBidirectional", 10, 1500), Definition("relay", "RelayToggle", 0), Definition("pump", "RelayPulse", 1));
        Act(fixture.Owner, "servo", "activate"); Act(fixture.Owner, "servo", "safe");
        Act(fixture.Owner, "position", "position", 0.25); Act(fixture.Owner, "relay", "activate");
        Act(fixture.Owner, "winch", "negative");
        var commands = fixture.Connection.Commands;
        Check(commands.Count == 5 && commands[0].Kind == MAVLink.MAV_CMD.DO_SET_SERVO && commands[0].Second == 2000 &&
            commands[1].Second == 1000 && commands[2].Second == 1250 && commands[3].Kind == MAVLink.MAV_CMD.DO_SET_RELAY &&
            commands[3].First == 0 && commands[3].Second == 1, "Native SET/calibration mapping changed");
        Check(fixture.Connection.Reads == 1 && fixture.Connection.Parameter == "SERVO10_TRIM" && fixture.Connection.TrimSystem == 4 &&
            fixture.Connection.TrimComponent == 2 && commands[4].Kind == MAVLink.MAV_CMD.DO_REPEAT_SERVO && commands[4].Second == 1000 &&
            commands[4].Third == 1 && Math.Abs(commands[4].Fourth - 0.1f) < 0.0001, "Explicit trim query/repeat servo mapping changed");
        Act(fixture.Owner, "pump", "activate");
        Check(commands.Count == 5, "A second native repeat overwrote the global slot");
        Act(fixture.Owner, "winch", "safe"); Act(fixture.Owner, "pump", "activate");
        Check(commands.Count == 8 && commands[6].Kind == MAVLink.MAV_CMD.DO_SET_RELAY && commands[6].Second == 0 &&
            commands[7].Kind == MAVLink.MAV_CMD.DO_REPEAT_RELAY && commands[7].Second == 1 && Math.Abs(commands[7].Third - 0.1f) < 0.0001 &&
            commands.All(command => command.System == 4 && command.Component == 2 && command.Ack), "Relay OFF/repeat units or native ACK policy changed");
        Act(fixture.Owner, "pump", "safe");
        Check(Act(fixture.Owner, "position", "position", double.NaN).Contains("Invalid") && commands.Count == 9,
            "Invalid normalized position emitted a native command");
    }
    private static void TrimAndTargetFences()
    {
        var fixture = Reset(Definition("winch", "ServoBidirectional", 10, 1500));
        fixture.Connection.Trim = () => Task.FromResult(1400f);
        Check(Act(fixture.Owner, "winch", "positive").Contains("TRIM") && fixture.Connection.Commands.Count == 0,
            "Mismatched native trim energized a timed servo");
        var held = new TaskCompletionSource<float>(); fixture.Connection.Trim = () => held.Task;
        var pending = fixture.Owner.ActAsync("winch", "positive");
        fixture.Connection.sysidcurrent = 12;
        held.SetResult(1500); Wait(pending);
        Check(fixture.Connection.Commands.Count == 0, "Calibration query retargeted changed aircraft");
        var pulse = Reset(Definition("pump", "RelayPulse", 1));
        var off = new TaskCompletionSource<bool>(); pulse.Connection.Send = command => off.Task;
        pending = pulse.Owner.ActAsync("pump", "activate");
        pulse.Connection.compidcurrent = 3; off.SetResult(true); Wait(pending);
        Check(pulse.Connection.Commands.Count == 1 && pulse.Connection.Commands[0].Second == 0,
            "Relay repeat sent after target changed during OFF initialization");
    }
    private static void StopAndFaults()
    {
        var fixture = Reset(Definition("pump", "RelayPulse", 1));
        OutputController.SendGimbalTargetAsync(2, 0, 0, 0, OutputController.Generation, null).GetAwaiter().GetResult();
        fixture.Connection.Commands.Clear();
        var held = new TaskCompletionSource<bool>();
        fixture.Connection.Send = command => command.Kind == MAVLink.MAV_CMD.DO_REPEAT_RELAY ? held.Task : Task.FromResult(true);
        var pending = fixture.Owner.ActAsync("pump", "activate");
        var safe = fixture.Owner.ActAsync("pump", "safe");
        Check(!safe.IsCompleted && fixture.Connection.Commands.Count == 2 && PayloadController.HasPendingWork,
            "Safe raced unfinished native activation/retries or settings could retire it");
        var gimbal = OutputController.SendGimbalTargetAsync(2, 0, 0, 0, OutputController.Generation, null).GetAwaiter().GetResult();
        Check(!gimbal.Submitted && fixture.Connection.Commands.Count == 2, "Gimbal interfered with native payload ACK helper");
        held.SetResult(true); Wait(pending); Wait(safe);
        Check(fixture.Connection.Commands.Count == 3 && fixture.Connection.Commands[2].Kind == MAVLink.MAV_CMD.DO_SET_RELAY &&
            fixture.Connection.Commands[2].Second == 0, "Stop did not follow native helper completion with safe");
        fixture.Connection.Send = command => throw new TimeoutException("injected timeout");
        Act(fixture.Owner, "pump", "activate");
        int count = fixture.Connection.Commands.Count;
        Act(fixture.Owner, "pump", "activate");
        Check(fixture.Connection.Commands.Count == count, "Uncertain output was automatically replayed");
        fixture.Connection.Send = command => Task.FromResult(true); Act(fixture.Owner, "pump", "safe");
    }
    private static void SequentialSafeDrain()
    {
        var fixture = Reset(Definition("first", "ServoToggle", 8), Definition("second", "ServoToggle", 9));
        Act(fixture.Owner, "first", "activate"); Act(fixture.Owner, "second", "activate");
        var held = new TaskCompletionSource<bool>();
        fixture.Connection.Send = command => command.First == 8 ? held.Task : Task.FromResult(true);
        var first = fixture.Owner.ActAsync("first", "safe");
        var second = fixture.Owner.ActAsync("second", "safe");
        var drain = fixture.Owner.WaitForPendingAsync();
        Check(!drain.IsCompleted && fixture.Connection.Commands.Count == 3 && PayloadController.HasPendingWork,
            "Settings safe-drain returned before all original-target safe commands");
        held.SetResult(true); Wait(first); Wait(second); Wait(drain);
        OutputController.Initialize(new NOMADConfig());
        Check(fixture.Connection.Commands.Count == 4 && fixture.Connection.Commands[3].First == 9 &&
            fixture.Connection.Commands[3].System == 4 && fixture.Connection.Commands[3].Second == 1000,
            "A queued Safe was retired before settings finished the original context");
    }
    private static void SafeRejectionOutcomes()
    {
        foreach (bool hostBusy in new[] { true, false })
        {
            var fixture = Reset(Definition("winch", "ServoBidirectional", 10, 1500));
            string activation = Act(fixture.Owner, "winch", "negative");
            var commands = fixture.Connection.Commands;
            Check(activation.Contains("Command accepted") && commands.Count == 1 &&
                commands[0].Kind == MAVLink.MAV_CMD.DO_REPEAT_SERVO && commands[0].Second == 1000,
                "Safe rejection fixture did not establish an accepted negative repeat");
            Check(SpinWait.SpinUntil(() => !PayloadController.HasPendingWork, 5000),
                "Native repeat reservation did not expire before the Safe rejection check");

            fixture.Connection.giveComport = hostBusy;
            fixture.Connection.Send = command => Task.FromResult(false);
            string rejected = Act(fixture.Owner, "winch", "safe");
            int expectedCommands = hostBusy ? 1 : 2;
            Check(commands.Count == expectedCommands &&
                (hostBusy ? rejected.Contains("Safe outcome uncertain") && rejected.Contains("busy") :
                    rejected.Contains("Safe not confirmed")),
                "Busy admission and native nonconfirmation did not report distinct truthful Safe outcomes");
            if (!hostBusy)
            {
                Check(commands[1].Kind == MAVLink.MAV_CMD.DO_SET_SERVO && commands[1].First == 10 &&
                    commands[1].Second == 1500 && commands[1].System == 4 && commands[1].Component == 2 && commands[1].Ack,
                    "Unconfirmed Safe changed native neutral mapping, target or ACK policy");
            }

            fixture.Connection.giveComport = false;
            fixture.Connection.Send = command => Task.FromResult(true);
            Wait(fixture.Owner.WaitForPendingAsync());
            Check(commands.Count == expectedCommands && !PayloadController.HasPendingWork,
                "Clearing native busy/nonconfirmation automatically retried Safe or retained hidden work");
            Check(Act(fixture.Owner, "winch", "negative").Contains("explicit accepted Safe") &&
                commands.Count == expectedCommands,
                "Failed Safe did not retain recovery admission after the repeat window and busy state cleared");

            string accepted = Act(fixture.Owner, "winch", "safe");
            var neutral = commands.Last();
            Check(accepted.Contains("Safe accepted") && commands.Count == expectedCommands + 1 &&
                neutral.Kind == MAVLink.MAV_CMD.DO_SET_SERVO && neutral.First == 10 && neutral.Second == 1500 &&
                neutral.System == 4 && neutral.Component == 2 && neutral.Ack,
                "A new explicit Safe did not send exactly one accepted original-target neutral command");
        }
    }
    private static void ConfirmationAndHid()
    {
        var fixture = Reset(Definition("release", "ServoToggle", 8, confirmations: 2));
        Map(fixture.Config, "release:activate");
        var service = new NomadJoystickService(fixture.Config);
        Buttons(service, new[] { true, false, false, false, false, false });
        Buttons(service, new[] { true, false, false, false, false, false });
        Check(fixture.Connection.Commands.Count == 0, "Initially held HID activated payload");
        Buttons(service, new bool[6]);
        Buttons(service, new[] { true, false, false, false, false, false });
        Check(fixture.Connection.Commands.Count == 0, "Hazardous action ignored confirmation count");
        Buttons(service, new bool[6]);
        Buttons(service, new[] { true, false, false, false, false, false });
        Check(fixture.Connection.Commands.Count == 1, "Distinct neutral-separated HID confirmations failed");
        Buttons(service, null);
        Check(fixture.Connection.Commands.Count == 2 && fixture.Connection.Commands[1].Second == 1000,
            "HID loss did not request safe");
        fixture = Reset(Definition("servo", "ServoToggle", 8));
        Check(Act(fixture.Owner, "servo", "toggle").Contains("unknown") && fixture.Connection.Commands.Count == 0,
            "Unknown toggle inferred activation after restart");
        Act(fixture.Owner, "servo", "safe"); Act(fixture.Owner, "servo", "toggle"); Act(fixture.Owner, "servo", "toggle");
        Check(fixture.Connection.Commands.Select(command => command.Second).SequenceEqual(new[] { 1000f, 2000f, 1000f }),
            "Saved toggle mapping lost ACK-backed commanded state");
        fixture = Reset(Definition("position", "ServoPosition", 9, 1500));
        MapPosition(fixture.Config);
        service = new NomadJoystickService(fixture.Config);
        Position(service, true, 1); Position(service, false, 0); Position(service, true, 1);
        Check(fixture.Connection.Commands.Count == 0, "Invalid/held position input bypassed center fence");
        Position(service, true, 0); Position(service, true, 1);
        Check(fixture.Connection.Commands.Count == 2 && fixture.Connection.Commands.Last().Second == 2000,
            "Centered fresh continuous HID input failed");
        fixture.Connection.Select();
        Position(service, true, 1);
        Check(fixture.Connection.Commands.Count == 2, "Held continuous axis crossed selected-aircraft generation");
        var release = Definition("release", "ServoToggle", 8, confirmations: 2);
        release["confirmation_window_ms"] = 500;
        fixture = Reset(release);
        Act(fixture.Owner, "release", "activate"); Thread.Sleep(520);
        Act(fixture.Owner, "release", "activate");
        Check(fixture.Connection.Commands.Count == 0, "Expired confirmation was reused");
        Act(fixture.Owner, "release", "activate");
        Check(fixture.Connection.Commands.Count == 1, "Fresh confirmation sequence did not activate once");
        fixture = Reset(Definition("first", "ServoToggle", 8), Definition("second", "ServoToggle", 9));
        Map(fixture.Config, "first:activate", "second:activate");
        service = new NomadJoystickService(fixture.Config);
        Buttons(service, new bool[6]);
        Buttons(service, new[] { true, true, false, false, false, false });
        Wait(fixture.Owner.WaitForPendingAsync());
        Check(fixture.Connection.Commands.Count == 2 && fixture.Connection.Commands.All(command => command.Second == 1000),
            "Contradictory switch pair activated its different outputs instead of safe");
    }
    private static void ConflictSafetySurvivesNewInput()
    {
        var fixture = Reset(Definition("first", "ServoToggle", 8), Definition("second", "ServoToggle", 9));
        Act(fixture.Owner, "first", "activate");
        Act(fixture.Owner, "second", "activate");
        Map(fixture.Config, "first:activate", "second:activate");
        var service = new NomadJoystickService(fixture.Config);
        Buttons(service, new bool[6]);
        var held = new TaskCompletionSource<bool>();
        fixture.Connection.Send = command => command.First == 8 ? held.Task : Task.FromResult(true);
        Buttons(service, new[] { true, true, false, false, false, false });
        var drain = fixture.Owner.WaitForPendingAsync();
        Check(!drain.IsCompleted && fixture.Connection.Commands.Count == 3, "First native Safe was not held");
        Buttons(service, new bool[6]);
        held.SetResult(true);
        Wait(drain);
        Check(fixture.Connection.Commands.Count == 4 && fixture.Connection.Commands[3].First == 9 &&
            fixture.Connection.Commands[3].Second == 1000 && fixture.Connection.Commands[3].System == 4,
            "Input revision change discarded the second original-target Safe");
    }
    private static void RepeatSurvivesRetirement()
    {
        var fixture = Reset(Definition("winch", "ServoBidirectional", 10, 1500));
        var held = new TaskCompletionSource<bool>(); fixture.Connection.Send = command => held.Task;
        var pending = fixture.Owner.ActAsync("winch", "positive");
        var next = Reset(Definition("pump", "RelayPulse", 1));
        Act(next.Owner, "pump", "activate");
        Check(next.Connection.Commands.Count == 0, "New owner overwrote retired pending native repeat");
        held.SetException(new TimeoutException("repeat ACK lost")); Wait(pending);
        Act(next.Owner, "pump", "activate");
        Check(next.Connection.Commands.Count == 0, "Unknown repeat lost its bounded reservation across owner replacement");
        Thread.Sleep(80);
        Act(next.Owner, "pump", "activate");
        Check(next.Connection.Commands.Count == 2, "Expired native repeat reservation never released");
        Act(next.Owner, "pump", "safe");
    }
    private static void MixedBatchRetiresAcrossAircraftChange()
    {
        foreach (string retirement in new[] { "selection", "disconnect", "input" })
        {
            var fixture = Reset(Definition("first", "ServoToggle", 8), Definition("second", "ServoToggle", 9));
            Act(fixture.Owner, "first", "activate");
            Map(fixture.Config, "first:safe");
            fixture.Config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "second_input", Name = "Second", Device = "test", Enabled = true, Button = 2, Action = "second:activate" }));
            var service = new NomadJoystickService(fixture.Config);
            Buttons(service, new bool[6]);
            var held = new TaskCompletionSource<bool>();
            fixture.Connection.Send = command => command.First == 8 ? held.Task : Task.FromResult(true);
            var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            long revision = (long)typeof(NomadJoystickService).GetField("_inputRevision", flags).GetValue(service);
            var events = new List<(string Id, string Operation, string Input)> { ("first", "safe", "test_input"), ("second", "activate", "second_input") };
            var batch = (Task)typeof(NomadJoystickService).GetMethod("SendPayloadEvents", flags)
                .Invoke(service, new object[] { events, revision });
            Check(!batch.IsCompleted && fixture.Connection.Commands.Count == 2, "Mixed HID batch did not pause at native Safe");
            if (retirement == "disconnect") { fixture.Connection.Close(); }
            else if (retirement == "selection") { fixture.Connection.sysidcurrent = 12; }
            else { Buttons(service, new[] { true, false, false, false, false, false }); }
            // No HID poll occurs here: its input revision still belongs to the retired aircraft.
            held.SetResult(true);
            Wait(batch);
            Check(fixture.Connection.Commands.Count == 2 && fixture.Connection.Commands.All(command => command.First == 8 && command.System == 4),
                "Old mixed HID batch activated an output after " + retirement);
        }
    }
    private static void NamedInputSafety()
    {
        var fixture = Reset(Definition("first", "ServoToggle", 8, confirmations: 2), Definition("second", "ServoToggle", 9));
        Map(fixture.Config, "first:activate");
        Check(fixture.Owner.ActAsync("second", "activate", source: "hid", inputId: "test_input").Result.Contains("Invalid input") &&
            fixture.Owner.ActAsync("first", "negative", source: "hid", inputId: "test_input").Result.Contains("Invalid input") &&
            fixture.Connection.Commands.Count == 0, "Known binding authorized the wrong payload or operation");
        var binding = InputBinding.Parse(fixture.Config.Inputs)[0];
        var second = JObject.FromObject(binding); second["Id"] = "other_input"; second["Button"] = 1;
        fixture.Config.Inputs.Add(second);
        var service = new NomadJoystickService(fixture.Config);
        Buttons(service, new bool[6]); Buttons(service, new[] { true, false, false, false, false, false });
        Buttons(service, new bool[6]); Buttons(service, new[] { false, true, false, false, false, false });
        Check(fixture.Connection.Commands.Count == 0, "Different stable identities combined their confirmations");
        Buttons(service, new bool[6]);
        var first = fixture.Config.Inputs[0]; fixture.Config.Inputs.RemoveAt(0); fixture.Config.Inputs.Add(first);
        Buttons(service, new[] { false, true, false, false, false, false });
        Check(fixture.Connection.Commands.Count == 1, "Reordering lost the same binding's neutral-separated confirmation: " + fixture.Connection.Commands.Count + " " + fixture.Owner.Status("first"));
        Buttons(service, null); Buttons(service, new[] { false, true, false, false, false, false });
        Check(fixture.Connection.Commands.Count == 2, "Held reconnect activated without neutral");
        Check(NomadJoystickService.ResolveDeviceName("test", new[] { "test", "TEST" }) == null &&
            NomadJoystickService.ResolveDeviceName("missing", new[] { "test" }) == null, "Ambiguous or missing device was substituted");
        var state = new InputControlState();
        binding.Source = "AxisSwitch3"; binding.Action = "first:activate"; binding.OtherAction = "second:safe";
        Check(state.Sample(binding, new Sample { Raw = 65535 }) && !state.Ready, "Held three-position endpoint armed input");
        state.Sample(binding, new Sample()); state.Sample(binding, new Sample { Raw = 65535 });
        Check(state.Ready && state.Direction == 1, "Center did not unlock three-position switch");
        state.Sample(binding, new Sample { Raw = 0 });
        Check(!state.Ready, "Skipped axis center was accepted as neutral");
        state.Sample(binding, new Sample()); state.Sample(binding, new Sample { Raw = 65535 });
        state.Sample(binding, new Sample { Raw = 47000 });
        Check(state.Direction == 1 && !state.Changed, "Axis threshold jitter retriggered endpoint");
        binding.Source = "AxisSwitch2"; state.Reset();
        state.Sample(binding, new Sample { Raw = 65535 }); Check(!state.Ready, "Held two-position axis armed input");
        state.Sample(binding, new Sample { Raw = 0 }); state.Sample(binding, new Sample { Raw = 65535 });
        Check(state.Ready && state.Direction == 1, "Two-position inactive endpoint did not arm fresh edge");
        binding.Invert = true; state.Reset(); state.Sample(binding, new Sample { Raw = 65535 }); state.Sample(binding, new Sample { Raw = 0 });
        Check(state.Ready && state.Direction == 1, "Inverted two-position endpoint did not work");
        Check(NomadJoystickService.TryReadAxisNorm(new Sample { Raw = 32767 }, "X", false, 0, out float midpoint) && midpoint == 0 &&
            NomadJoystickService.TryReadAxisNorm(new Sample { Raw = 32768 }, "X", false, 0, out midpoint) && midpoint == 0,
            "Unsigned axis midpoint lost zero-deadzone center");
        var now = DateTime.UtcNow;
        var listen = new InputListenState(now);
        listen.Sample(new Sample { Buttons = new[] { true, false } }, false, now);
        listen.Sample(new Sample { Buttons = new[] { true, false }, Raw = 33000 }, false, now);
        Check(listen.Active && fixture.Connection.Commands.Count == 2, "Held/noisy Listen input dispatched or captured");
        listen.Sample(new Sample { Buttons = new[] { false, false } }, false, now);
        listen.Sample(new Sample { Buttons = new[] { true, false } }, false, now);
        Check(!listen.Active && listen.Button == 0 && fixture.Connection.Commands.Count == 2, "Fresh Listen press failed or dispatched output");
        listen = new InputListenState(now); listen.Sample(new Sample(), false, now); listen.Sample(new Sample { Raw = 65535 }, false, now);
        Check(!listen.Active && listen.Axis == "X", "Generic Listen failed to detect an axis without choosing its type");
        listen = new InputListenState(now); listen.Sample(new Sample(), false, now); listen.Sample(new Sample { Raw = 65535 }, true, now);
        Check(!listen.Active && listen.Axis == null && listen.Button == null, "New connection did not retire capture before detection");
        listen = new InputListenState(now); listen.Sample(new Sample(), false, now.AddSeconds(16));
        Check(!listen.Active, "Listen deadline did not retire capture");
        listen = new InputListenState(now); listen.Sample(null, false, now);
        Check(!listen.Active, "Device loss did not retire capture");
        fixture = Reset(Definition("gimbal", "ServoToggle", 8)); Map(fixture.Config, "gimbal:activate");
        service = new NomadJoystickService(fixture.Config); Buttons(service, new bool[6]); Buttons(service, new[] { true, false, false, false, false, false });
        Check(fixture.Connection.Commands.Count == 1, "Payload named gimbal could not be bound");
        fixture = Reset(Definition("first", "ServoToggle", 8));
        fixture.Config.Inputs = new JArray(JObject.FromObject(new InputBinding { Id = "pitch", Name = "Pitch", Device = "test", Enabled = true,
            Source = "Axis", Action = "gimbal:pitch" }));
        service = new NomadJoystickService(fixture.Config); GimbalController.SetTargetAngles(0, 0);
        Position(service, true, 1); Check(GimbalController.TargetPitchDeg == 0, "Held gimbal axis moved before center");
        Position(service, true, 0); Position(service, true, 1); Check(GimbalController.TargetPitchDeg > 0, "Centered gimbal axis did not move");
        float target = GimbalController.TargetPitchDeg; fixture.Connection.Select(); Position(service, true, 1);
        Check(GimbalController.TargetPitchDeg == target, "Held gimbal crossed generation");
    }
    private static void MixedInputArbitration()
    {
        var fixture = Reset(Definition("position", "ServoPosition", 9, 1500));
        MapPosition(fixture.Config);
        fixture.Config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "safe", Name = "Safe", Device = "test", Enabled = true,
            Button = 0, Action = "position:safe" }));
        var service = new NomadJoystickService(fixture.Config);
        Position(service, true, 0); fixture.Connection.Commands.Clear();
        service.TranslateInputs(new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = new Sample { Raw = 65535, Buttons = new[] { true, false } } });
        Check(fixture.Connection.Commands.Count == 1 && fixture.Connection.Commands[0].Second == 1500,
            "Same-sample position dispatched before its opposing Safe");
        foreach (int raw in new[] { 50000, 32768, 0, 32768, 65535 })
        {
            service.TranslateInputs(new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = new Sample { Raw = raw, Buttons = new[] { true, false } } });
        }
        Check(fixture.Connection.Commands.All(command => command.Second == 1500), "Changing axis overrode a held Safe");
        fixture = Reset(Definition("position", "ServoPosition", 9, 1500)); MapPosition(fixture.Config);
        fixture.Config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "axis2", Name = "Second axis", Device = "other", Enabled = true,
            Source = "Axis", Action = "position:position" }));
        service = new NomadJoystickService(fixture.Config);
        var both = new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = new Sample(), ["other"] = new Sample() };
        service.TranslateInputs(both); both["test"] = new Sample { Raw = 65535 }; both["other"] = new Sample { Raw = 0 }; service.TranslateInputs(both);
        Check(fixture.Connection.Commands.Count == 1 && fixture.Connection.Commands[0].Second == 1500,
            "Two continuous inputs dispatched position instead of conflict Safe");
        fixture = Reset(Definition("winch", "ServoBidirectional", 10, 1500)); Map(fixture.Config, "winch:positive");
        fixture.Config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "pitch", Name = "Pitch", Device = "test", Enabled = true,
            Source = "Axis", Action = "gimbal:pitch" }));
        var trim = new TaskCompletionSource<float>(); fixture.Connection.Trim = () => trim.Task;
        service = new NomadJoystickService(fixture.Config); Buttons(service, new bool[6]);
        Buttons(service, new[] { true, false, false, false, false, false });
        var pending = fixture.Owner.WaitForPendingAsync(); Check(!pending.IsCompleted, "Delayed trim fixture was not pending");
        service.TranslateInputs(new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = new Sample { Raw = 65535, Buttons = new[] { true, false, false, false, false, false } } });
        trim.SetResult(1500); Wait(pending);
        Check(fixture.Connection.Commands.Any(command => command.Kind == MAVLink.MAV_CMD.DO_REPEAT_SERVO),
            "Unrelated gimbal movement retired a legitimate pending payload action");
        Act(fixture.Owner, "winch", "safe");
        fixture = Reset(Definition("first", "ServoToggle", 8));
        fixture.Config.Inputs = new JArray(JObject.FromObject(new InputBinding { Id = "pitch", Name = "Pitch", Device = "test", Enabled = true,
            Source = "Axis", Action = "gimbal:pitch" }), JObject.FromObject(new InputBinding { Id = "roll", Name = "Roll", Device = "other", Enabled = true,
            Source = "Axis", Action = "gimbal:roll" }));
        service = new NomadJoystickService(fixture.Config); GimbalController.SetTargetAngles(0, 0);
        both = new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = new Sample(), ["other"] = null };
        service.TranslateInputs(both); both["test"] = new Sample { Raw = 65535 }; service.TranslateInputs(both);
        Check(GimbalController.TargetPitchDeg == 0, "One valid gimbal axis moved while the other was missing");
        both["test"] = new Sample(); both["other"] = new Sample { Raw = 65535 }; service.TranslateInputs(both);
        both["test"] = new Sample { Raw = 65535 }; service.TranslateInputs(both);
        Check(GimbalController.TargetPitchDeg == 0 && GimbalController.TargetRollDeg == 0, "One fresh gimbal center bypassed joint center");
        both["test"] = new Sample(); both["other"] = new Sample(); service.TranslateInputs(both);
        both["test"] = new Sample { Raw = 65535 }; service.TranslateInputs(both);
        Check(GimbalController.TargetPitchDeg > 0, "Joint centered gimbal axes failed to move");
        float target = GimbalController.TargetPitchDeg;
        OutputController.Initialize(new NOMADConfig()); service.TranslateInputs(both);
        Check(GimbalController.TargetPitchDeg == target, "Retired service profile continued gimbal movement");
    }
    private static void AbsentDeviceDoesNotRetireOtherInput()
    {
        foreach (bool loseActiveDevice in new[] { false, true })
        {
            var fixture = Reset(Definition("winch", "ServoBidirectional", 10, 1500), Definition("other", "RelayToggle", 0));
            Map(fixture.Config, "winch:positive");
            fixture.Config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "absent", Name = "Unplugged control",
                Device = "missing", Enabled = true, Action = "other:activate" }));
            var trim = new TaskCompletionSource<float>(); fixture.Connection.Trim = () => trim.Task;
            var service = new NomadJoystickService(fixture.Config);
            var samples = new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState> { ["test"] = new Sample() };
            service.TranslateInputs(samples);
            samples["test"] = new Sample { Buttons = new[] { true, false, false, false, false, false } };
            service.TranslateInputs(samples);
            var activation = fixture.Owner.WaitForPendingAsync();
            Check(!activation.IsCompleted && fixture.Connection.Reads == 1, "Delayed trim did not establish pending HID work");
            for (int poll = 0; poll < 5; poll++) { service.TranslateInputs(samples); }
            if (loseActiveDevice)
            {
                samples["test"] = null;
                for (int poll = 0; poll < 5; poll++) { service.TranslateInputs(samples); }
            }
            var drain = fixture.Owner.WaitForPendingAsync();
            trim.SetResult(1500); Wait(activation); Wait(drain);
            Check(fixture.Connection.Commands.Count == 1 && fixture.Connection.Commands[0].First == 10 &&
                fixture.Connection.Commands[0].Kind == (loseActiveDevice ? MAVLink.MAV_CMD.DO_SET_SERVO : MAVLink.MAV_CMD.DO_REPEAT_SERVO) &&
                fixture.Connection.Commands[0].Second == (loseActiveDevice ? 1500 : 2000),
                loseActiveDevice ? "Losing the active HID during trim did not refuse activation and send Safe" :
                    "An already absent unrelated HID retired unchanged pending trim input");
            if (!loseActiveDevice) { Act(fixture.Owner, "winch", "safe"); }
        }
    }
    private static void StableConflictDoesNotRetireOtherInput()
    {
        foreach (int conflictRaw in new[] { 32768, 65535 })
        {
            var fixture = Reset(Definition("position", "ServoPosition", 9, 1500), Definition("winch", "ServoBidirectional", 10, 1500));
            MapPosition(fixture.Config);
            fixture.Config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "safe", Name = "Safe", Device = "test", Enabled = true,
                Button = 0, Action = "position:safe" }));
            fixture.Config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "winch", Name = "Winch", Device = "other", Enabled = true,
                Button = 0, Action = "winch:positive" }));
            var service = new NomadJoystickService(fixture.Config);
            var samples = new Dictionary<string, MissionPlanner.Joystick.IMyJoystickState>
                { ["test"] = new Sample(), ["other"] = new Sample() };
            service.TranslateInputs(samples);
            int beforeConflict = fixture.Connection.Commands.Count;
            samples["test"] = new Sample { Raw = conflictRaw, Buttons = new[] { true, false } };
            service.TranslateInputs(samples); Wait(fixture.Owner.WaitForPendingAsync());
            Check(fixture.Connection.Commands.Count == beforeConflict + 1 && fixture.Connection.Commands.Last().First == 9 &&
                fixture.Connection.Commands.Last().Second == 1500, "Conflict entry did not admit exactly one position Safe");
            fixture.Connection.Commands.Clear();
            var trim = new TaskCompletionSource<float>(); fixture.Connection.Trim = () => trim.Task;
            samples["other"] = new Sample { Buttons = new[] { true, false } };
            service.TranslateInputs(samples);
            var pending = fixture.Owner.WaitForPendingAsync();
            Check(!pending.IsCompleted, "Unrelated winch did not begin blocked trim during established conflict");
            for (int poll = 0; poll < 5; poll++) { service.TranslateInputs(samples); }
            trim.SetResult(1500); Wait(pending);
            Check(fixture.Connection.Commands.Count == 1 && fixture.Connection.Commands[0].Kind == MAVLink.MAV_CMD.DO_REPEAT_SERVO &&
                fixture.Connection.Commands[0].First == 10 && fixture.Connection.Commands[0].Second == 2000,
                "Unchanged conflict retired unrelated trim or emitted position/repeated Safe");
            Act(fixture.Owner, "winch", "safe");
            if (conflictRaw == 65535)
            {
                fixture.Connection.Commands.Clear();
                samples["test"] = new Sample { Raw = 65535 };
                service.TranslateInputs(samples);
                Check(fixture.Connection.Commands.Count == 0, "Held axis resumed position after conflict without fresh center");
                samples["test"] = new Sample(); service.TranslateInputs(samples);
                samples["test"] = new Sample { Raw = 65535 }; service.TranslateInputs(samples);
                Check(fixture.Connection.Commands.Count == 2 && fixture.Connection.Commands.Last().First == 9 &&
                    fixture.Connection.Commands.Last().Second == 2000, "Fresh neutral did not restore position after conflict");
            }
        }
    }
    private static int Main()
    {
        StableConflictDoesNotRetireOtherInput(); AbsentDeviceDoesNotRetireOtherInput(); MixedInputArbitration(); NamedInputSafety(); InvalidDefinitions(); NativeMappings(); TrimAndTargetFences(); StopAndFaults(); SequentialSafeDrain(); SafeRejectionOutcomes(); ConfirmationAndHid(); ConflictSafetySurvivesNewInput(); RepeatSurvivesRetirement(); MixedBatchRetiresAcrossAircraftChange();
        Console.WriteLine("Payload tests passed (native mappings, strict calibration, ACK faults, HID edges, stale targets and repeat lifetime).");
        return 0;
    }
}
