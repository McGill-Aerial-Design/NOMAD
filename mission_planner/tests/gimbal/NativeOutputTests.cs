// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

// Native host doubles exercise dispatch and identity fences without aircraft or MP application state.
public class MAVLink
{
    public enum MAV_CMD { DO_MOUNT_CONFIGURE, DO_SET_SERVO }
}
namespace MissionPlanner
{
    public static class MainV2 { public static MAVLinkInterface comPort; }
    public sealed class TestStream { public bool IsOpen = true; }
    public sealed class MAVLinkInterface
    {
        public TestStream BaseStream = new TestStream();
        public int sysidcurrent = 4, compidcurrent = 2;
        public bool ReadOnly, logreadmode, giveComport;
        public event EventHandler MavChanged;
        public event EventHandler CommsClose;
        public Func<Task<bool>> Configure = () => Task.FromResult(true);
        public readonly ManualResetEventSlim ConfigureEntered = new ManualResetEventSlim();
        public int Modes, Targets;
        public readonly List<(int Channel, int Pwm)> ServoTargets = new List<(int Channel, int Pwm)>();
        public readonly List<bool> ServoAcknowledgements = new List<bool>();
        public byte LastSystem, LastComponent;
        public double Pitch, Roll, Yaw;
        public Action TargetDispatched;
        public bool AcknowledgementRequested;
        public void Close() { CommsClose?.Invoke(this, EventArgs.Empty); }
        public void Select() { MavChanged?.Invoke(this, EventArgs.Empty); }
        public Task<bool> doCommandAsync(byte system, byte component, MAVLink.MAV_CMD command,
            float p1, float p2, float p3, float p4, float p5, float p6, float p7, bool requireack)
        {
            if (command == MAVLink.MAV_CMD.DO_SET_SERVO)
            { ServoTargets.Add(((int)p1, (int)p2)); ServoAcknowledgements.Add(requireack); }
            else { Modes++; }
            LastSystem = system;
            LastComponent = component;
            AcknowledgementRequested = requireack;
            ConfigureEntered.Set();
            if (command == MAVLink.MAV_CMD.DO_SET_SERVO && !requireack) { return Task.FromResult(true); }
            return Configure();
        }
        public void setMountControl(byte system, byte component, double pitch, double roll, double yaw, bool coordinates)
        {
            Targets++;
            LastSystem = system;
            LastComponent = component;
            Pitch = pitch;
            Roll = roll;
            Yaw = yaw;
            TargetDispatched?.Invoke();
        }
    }
}
namespace NOMAD.MissionPlanner
{
    public sealed class NOMADConfig
    {
        public int GimbalMountMode = 2;
        public float JoystickGimbalMaxRateDegSec = 60;
        public string JoystickGimbalHorizontalAxis = "Yaw", JoystickGimbalVerticalAxis = "Pitch";
        public bool JoystickGimbalHorizontalInverted, JoystickGimbalVerticalInverted;
        public bool GimbalArrowKeysEnabled;
        public string SelectedVideoInputName = "";
        public IReadOnlyList<ServoGimbalAxis> SelectedGimbalAxes { get; set; }
    }
    public sealed class NOMADProfiles
    {
        public sealed class Profile { public NOMADConfig Config = new NOMADConfig(); }
        public Profile Selected = new Profile();
        public void Save() { }
    }
    public static class Log { public static void Warn(string message) { } }
}
internal static class NativeOutputTests
{
    private static MissionPlanner.MAVLinkInterface Reset()
    {
        OutputController.Shutdown();
        var connection = new MissionPlanner.MAVLinkInterface();
        MissionPlanner.MainV2.comPort = connection;
        var config = new NOMADConfig();
        OutputController.Initialize(config);
        GimbalController.ApplyProfile(config);
        return connection;
    }
    private static GimbalRequestResult Send(long? generation = null, Func<bool> current = null,
        double pitch = -12, double roll = 7) => Complete(OutputController.SendGimbalTargetAsync(2, pitch, roll, 0,
            generation ?? OutputController.Generation, current));
    private static void Require(bool condition, string diagnostic)
    {
        if (!condition) { throw new Exception(diagnostic); }
    }
    private static GimbalRequestResult Complete(Task<GimbalRequestResult> task)
    {
        Require(task.Wait(TimeSpan.FromSeconds(2)), "Gimbal request did not finish within two seconds");
        return task.GetAwaiter().GetResult();
    }
    private static void WaitForConfigure(MissionPlanner.MAVLinkInterface connection) =>
        Require(connection.ConfigureEntered.Wait(2000), "Native configure did not enter within two seconds");
    private static void DrainGimbal()
    {
        var inflight = typeof(GimbalController).GetField("_inflight",
            System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic);
        Require(SpinWait.SpinUntil(() => (int)inflight.GetValue(null) == 0, 2000),
            "Fire-and-forget gimbal request did not finish within two seconds");
    }
    private static void BlockingNativeHelpers()
    {
        foreach (bool mount in new[] { false, true })
        {
            var connection = Reset();
            int caller = Thread.CurrentThread.ManagedThreadId;
            using (var entered = new ManualResetEventSlim())
            using (var release = new ManualResetEventSlim())
            {
                Action block = () =>
                {
                    Require(Thread.CurrentThread.ManagedThreadId != caller, "Native helper ran on its caller thread");
                    entered.Set();
                    Require(release.Wait(5000), "Blocking native fixture was not released");
                };
                if (mount) { connection.TargetDispatched = block; }
                else { connection.Configure = () => { block(); return Task.FromResult(true); }; }
                var watch = Stopwatch.StartNew();
                var pending = Track();
                Require(watch.ElapsedMilliseconds < 500, "Native helper blocked the gimbal request caller");
                try
                {
                    Require(entered.Wait(2000), "Blocking native helper did not enter");
                    var state = Task.Run(() => { OutputController.RefreshConnection(); return OutputController.Generation; });
                    Require(state.Wait(500), "Native helper held the UI-visible output lock");
                    Require(Send().Outcome == GimbalRequestOutcome.Busy, "Blocked native helper allowed another request");
                    GimbalController.SetTargetAngles(0, 0);
                    OutputController.Shutdown();
                    OutputController.Initialize(new NOMADConfig());
                    Require(Send().Outcome == GimbalRequestOutcome.Busy,
                        "Retirement released a reservation still owned by a native helper");
                }
                finally { release.Set(); }
                var result = Complete(pending);
                Require(result.Outcome == (mount ? GimbalRequestOutcome.Submitted : GimbalRequestOutcome.Unknown) &&
                    connection.Targets == (mount ? 1 : 0) && GimbalController.LastSubmittedPose == null,
                    "Retired native completion replayed, published stale pose or claimed cancellation");
                Require(Send().Submitted, "Completed native helper retained its reservation");
            }
        }
    }
    private static void RefusesChangedTarget(Action<MissionPlanner.MAVLinkInterface> change, string scenario)
    {
        var connection = Reset();
        long captured = OutputController.Generation;
        change(connection);
        var result = Send(captured);
        Require(result.Outcome == GimbalRequestOutcome.NotSent && connection.Modes == 0 && connection.Targets == 0,
            scenario + " allowed retired input to dispatch");
    }
    private static Task<GimbalRequestResult> Track(float pitch = -12, float yaw = 7,
        Func<bool> current = null, long? generation = null, long? manualRevision = null) =>
        GimbalController.RequestTrackingTargetAsync(pitch, yaw, generation ?? OutputController.Generation,
            manualRevision ?? GimbalController.ManualRevision, current ?? (() => true));
    private static void TrackingAngles()
    {
        var connection = Reset();
        long revision = GimbalController.ManualRevision;
        var result = Complete(Track());
        Require(result.Submitted && connection.Pitch == -1200 && connection.Roll == 0 && connection.Yaw == 700 &&
            GimbalController.TargetPitchDeg == -12 && GimbalController.TargetRollDeg == 0 &&
            GimbalController.TargetYawDeg == 7 && GimbalController.ManualRevision == revision,
            "Tracking did not preserve explicit pitch/yaw centidegrees and zero roll");
        foreach (float limit in new[] { -90f, 90f })
        {
            Require(Complete(Track(limit, limit)).Submitted && connection.Pitch == limit * 100 &&
                connection.Yaw == limit * 100 && connection.Roll == 0, "Tracking boundary target was rejected or remapped");
        }
        foreach (float yaw in new[] { -180f, -170f, 170f, 180f })
        {
            Require(Complete(Track(yaw: yaw)).Submitted && connection.Yaw == yaw * 100,
                "Calibrated wider-travel mount yaw was rejected or remapped");
        }
        result = Complete(GimbalController.RequestPitchRollTargetAsync(-8, 4));
        Require(result.Submitted && connection.Pitch == -800 && connection.Roll == 400 && connection.Yaw == 0 &&
            GimbalController.TargetYawDeg == 0, "Manual roll was reinterpreted as tracking yaw");
        GimbalController.ApplyStick(1, 0, 10, 0.1f, true);
        DrainGimbal();
        Require(connection.Roll == 300 && connection.Yaw == 0, "Manual stick roll convention changed");
    }
    private static void MalformedTargets()
    {
        foreach (double invalid in new[] { double.NaN, double.NegativeInfinity, double.PositiveInfinity, -180.01, 180.01 })
        {
            var connection = Reset();
            var result = Complete(OutputController.SendGimbalTargetAsync(2, 0, 0, invalid,
                OutputController.Generation, null));
            Require(!result.Submitted && connection.Modes == 0 && connection.Targets == 0,
                "Invalid yaw reached a native command");
            Require(!Complete(Track(yaw: (float)invalid)).Submitted && connection.Modes == 0,
                "Tracking accepted invalid yaw");
        }
        foreach (double invalid in new[] { double.NaN, double.NegativeInfinity, double.PositiveInfinity, -30.01, 30.01 })
        {
            var connection = Reset();
            Require(!Send(roll: invalid).Submitted && connection.Modes == 0, "Invalid roll emitted a command");
        }
        foreach (var target in new (int Mode, double? Pitch, double? Roll, double Yaw)[]
        {
            (-1, 0, 0, 0), (4, 0, 0, 0), (3, 0, 0, 0), (2, null, 0, 0),
            (2, 0, null, 0), (2, null, null, 7)
        })
        {
            var connection = Reset();
            var result = Complete(OutputController.SendGimbalTargetAsync(target.Mode, target.Pitch, target.Roll, target.Yaw,
                OutputController.Generation, null));
            Require(!result.Submitted && connection.Modes == 0 && connection.Targets == 0,
                "Malformed gimbal target emitted a command");
        }
    }

    private static void ScreenAxisTargets()
    {
        var connection = Reset();
        Require(Complete(GimbalController.RequestScreenTargetAsync(-8, 4, 12)).Submitted,
            "Manual three-axis screen target failed");
        GimbalController.MaxRateDegSec = 10;
        GimbalController.ApplyScreenStick(1, 0, .1f, true);
        DrainGimbal();
        Require(connection.Pitch == -800 && connection.Roll == 400 && connection.Yaw == 1300,
            "Default screen horizontal input changed roll or lost the other target axes");
        GimbalArrowKeyFilter.TryHandleKey(System.Windows.Forms.Keys.Up);
        DrainGimbal();
        Require(connection.Pitch == -600 && connection.Roll == 400 && connection.Yaw == 1300,
            "Screen arrow mapping disagrees with the pad or erased an unmapped axis");
        Require(Complete(GimbalController.RequestScreenTargetAsync(-90, 4, 13)).Submitted &&
            connection.Pitch == -9000 && connection.Roll == 400 && connection.Yaw == 1300,
            "Look Down target erased roll or yaw");

        var config = new NOMADConfig { JoystickGimbalHorizontalAxis = "Roll", JoystickGimbalVerticalAxis = "Yaw",
            JoystickGimbalHorizontalInverted = true, JoystickGimbalVerticalInverted = true };
        OutputController.Initialize(config); GimbalController.ApplyProfile(config);
        GimbalController.MaxRateDegSec = 10;
        GimbalController.ApplyScreenStick(1, .5f, .1f, true);
        DrainGimbal();
        Require(connection.Pitch == 0 && connection.Roll == -100 && connection.Yaw == -50,
            "Configured screen axes or independent inversion was ignored");
        GimbalArrowKeyFilter.TryHandleKey(System.Windows.Forms.Keys.Left);
        DrainGimbal();
        Require(connection.Roll == 100 && connection.Yaw == -50,
            "Configured arrow axis or inversion was ignored");
        config.JoystickGimbalHorizontalInverted = config.JoystickGimbalVerticalInverted = false;
        GimbalController.ApplyProfile(config);
        Require(Complete(GimbalController.RequestScreenTargetAsync(89, 29, 179)).Submitted,
            "Screen boundary fixture did not initialize");
        GimbalController.ApplyScreenStick(1, 1, .5f, true);
        DrainGimbal();
        Require(connection.Pitch == 8900 && connection.Roll == 3000 && connection.Yaw == 18000,
            "Mapped screen target crossed an axis limit or changed the unselected pitch axis");

        config.JoystickGimbalHorizontalAxis = "None"; config.JoystickGimbalVerticalAxis = "Pitch";
        GimbalController.ApplyProfile(config);
        int targets = connection.Targets;
        GimbalController.ApplyScreenStick(1, 0, .1f, true);
        Require(connection.Targets == targets, "Disabled screen axis dispatched a native request");
        GimbalController.ApplyScreenStick(0, 1, .1f, true);
        DrainGimbal();
        Require(connection.Pitch == 600 && connection.Roll == 0 && connection.Yaw == 0,
            "One-axis gimbal configuration did not retain its enabled axis");
        foreach (float invalid in new[] { float.NaN, float.PositiveInfinity, 0, -.1f, .51f })
        {
            targets = connection.Targets;
            GimbalController.ApplyScreenStick(0, 1, invalid, true);
            Require(connection.Targets == targets, "Invalid screen dt dispatched");
        }
        connection = Reset(); connection.ReadOnly = true;
        GimbalController.ApplyScreenStick(1, 0, .1f, true);
        Require(connection.Targets == 0 && connection.Modes == 0, "Screen input bypassed native read-only admission");
    }
    private static void TrackingRetirement()
    {
        var connection = Reset();
        long revision = GimbalController.ManualRevision, generation = OutputController.Generation;
        GimbalController.SetTargetAngles(0, 0);
        Require(!Complete(Track(manualRevision: revision)).Submitted && connection.Modes == 0,
            "Stale manual revision admitted tracking");
        OutputController.Initialize(new NOMADConfig());
        Require(!Complete(Track(generation: generation)).Submitted && connection.Modes == 0,
            "Stale aircraft generation admitted tracking");
        Require(!Complete(Track(current: () => false)).Submitted && connection.Modes == 0,
            "Retired tracking source emitted a command");
        var noSource = Complete(GimbalController.RequestTrackingTargetAsync(0, 0, OutputController.Generation,
            GimbalController.ManualRevision, null));
        Require(!noSource.Submitted && connection.Modes == 0, "Tracking without a current-input fence emitted a command");
        foreach (string retired in new[] { "source", "profile", "readonly", "nativebusy", "stream", "system" })
        {
            connection = Reset();
            var held = new TaskCompletionSource<bool>();
            connection.Configure = () => held.Task;
            bool current = true;
            var pending = Track(current: () => current);
            WaitForConfigure(connection);
            if (retired == "source") { current = false; }
            if (retired == "profile") { OutputController.Initialize(new NOMADConfig()); }
            if (retired == "readonly") { connection.ReadOnly = true; }
            if (retired == "nativebusy") { connection.giveComport = true; }
            if (retired == "stream") { connection.BaseStream = new MissionPlanner.TestStream(); }
            if (retired == "system") { connection.sysidcurrent = 8; }
            held.SetResult(true);
            var result = Complete(pending);
            Require(result.Outcome == GimbalRequestOutcome.Unknown && connection.Modes == 1 && connection.Targets == 0 &&
                GimbalController.TargetYawDeg == 0, retired + " post-configure tracking sent or published a target");
        }
    }
    private static void ManualOverride(Action manual, string scenario)
    {
        var connection = Reset();
        var held = new TaskCompletionSource<bool>();
        connection.Configure = () => held.Task;
        var pending = Track();
        WaitForConfigure(connection);
        long revision = GimbalController.ManualRevision;
        Require(!Complete(Track()).Submitted && connection.Modes == 1 && connection.Targets == 0,
            "Concurrent tracking request dispatched while another was pending");
        int overrides = 0;
        Action onManual = () => overrides++;
        GimbalController.ManualInput += onManual;
        try { manual(); }
        finally { GimbalController.ManualInput -= onManual; }
        Require(overrides == 1 && GimbalController.ManualRevision > revision,
            scenario + " did not invalidate tracking before busy admission");
        held.SetResult(true);
        var result = Complete(pending);
        Require(result.Outcome == GimbalRequestOutcome.Unknown && connection.Modes == 1 && connection.Targets == 0,
            scenario + " allowed a pending tracking target through");
    }
    private static void NeutralAndDispatchedTracking()
    {
        var connection = Reset();
        var held = new TaskCompletionSource<bool>();
        connection.Configure = () => held.Task;
        var pending = Track();
        long revision = GimbalController.ManualRevision;
        int overrides = 0;
        Action onManual = () => overrides++;
        GimbalController.ManualInput += onManual;
        try { GimbalController.ApplyStick(0, 0, 0.05f, true); }
        finally { GimbalController.ManualInput -= onManual; }
        held.SetResult(true);
        Require(Complete(pending).Submitted && overrides == 0 &&
            GimbalController.ManualRevision == revision && connection.Targets == 1,
            "Neutral background stick sample retired or duplicated tracking");
        connection = Reset();
        bool current = true;
        connection.TargetDispatched = () => current = false;
        var result = Complete(Track(current: () => current));
        Require(result.Submitted && connection.Targets == 1 && GimbalController.TargetYawDeg == 0,
            "Already-dispatched native target was claimed cancelled or published after retirement");
    }
    private static void PendingManualRetirement()
    {
        var connection = Reset();
        var held = new TaskCompletionSource<bool>();
        connection.Configure = () => held.Task;
        var pending = GimbalController.RequestPitchRollTargetAsync(-10, 3);
        WaitForConfigure(connection);
        GimbalController.SetMode(MountMode.Neutral);
        held.SetResult(true);
        Require(Complete(pending).Outcome == GimbalRequestOutcome.Unknown && connection.Modes == 1 &&
            connection.Targets == 0 && GimbalController.SelectedMode == MountMode.MavlinkTargeting,
            "New manual mode input failed to retire a pending manual target");
    }
    private static void SharedTrackingClaim()
    {
        var connection = Reset();
        var held = new TaskCompletionSource<bool>();
        connection.Configure = () => held.Task;
        long first = GimbalController.ClaimTracking();
        var pending = Track(current: () => first == GimbalController.CurrentTrackingClaim);
        WaitForConfigure(connection);
        long second = GimbalController.ClaimTracking();
        held.SetResult(true);
        Require(second > first && Complete(pending).Outcome == GimbalRequestOutcome.Unknown && connection.Targets == 0,
            "New video owner allowed the previous owner's pending target through");
        Require(Complete(Track(current: () => second == GimbalController.CurrentTrackingClaim)).Submitted &&
            connection.Targets == 1 && GimbalController.CurrentTrackingClaim == second,
            "Retired owner completion invalidated the new tracking claim");
    }
    private static void ServoGimbalTargets()
    {
        var connection = Reset();
        var config = new NOMADConfig { SelectedVideoInputName = "Camera A", SelectedGimbalAxes = new[]
        {
            new ServoGimbalAxis { Axis = "Pitch", Channel = 14, MaximumPwm = 1800 },
            new ServoGimbalAxis { Axis = "Yaw", Channel = 15, MinimumAngleDeg = -180, MaximumAngleDeg = 180, Reversed = true },
            new ServoGimbalAxis { Axis = "Roll", Channel = 16, MinimumPwm = 1100, MaximumPwm = 1900 },
        } };
        OutputController.Initialize(config); GimbalController.ApplyProfile(config);
        var result = Complete(GimbalController.RequestScreenTargetAsync(45, -45, 90));
        Require(result.Submitted && connection.Modes == 0 && connection.Targets == 0 &&
            connection.ServoTargets.Count == 3 && connection.ServoTargets[0] == (14, 1600) &&
            connection.ServoTargets[1] == (15, 1250) && connection.ServoTargets[2] == (16, 1300) &&
            connection.AcknowledgementRequested && connection.LastSystem == 4 && connection.LastComponent == 2,
            "Custom gimbal did not send three calibrated, reversed, ACK-requested servo targets to the captured aircraft");
        int commands = connection.ServoTargets.Count;
        Require(!Complete(GimbalController.RequestScreenTargetAsync(180, 0, 0)).Submitted &&
            connection.ServoTargets.Count == commands, "An out-of-calibration gimbal target reached a servo");
        foreach (int count in new[] { 1, 2 })
        {
            config.SelectedGimbalAxes = new[]
            {
                new ServoGimbalAxis { Axis = "Pitch", Channel = 14 },
                new ServoGimbalAxis { Axis = "Yaw", Channel = 15 },
            }.Take(count).ToArray();
            GimbalController.ApplyProfile(config);
            commands = connection.ServoTargets.Count;
            Require(Complete(GimbalController.RequestScreenTargetAsync(0, 0, 0)).Submitted &&
                connection.ServoTargets.Count == commands + count, "One/two-axis gimbal dispatched the wrong outputs");
        }
        var input = new GimbalInputState();
        input.CanMove(true, 0, 0, 1, 1);
        Require(input.CanMove(true, 1, 0, 1, 1) && !input.CanMove(true, 1, 0, 1, 2),
            "Changing camera gimbal bindings retained a held joystick input");
        foreach (string change in new[] { "camera", "calibration", "slider" })
        {
            connection = Reset();
            config = new NOMADConfig { SelectedVideoInputName = "Camera A", SelectedGimbalAxes = new[]
            {
                new ServoGimbalAxis { Axis = "Pitch", Channel = 14 },
                new ServoGimbalAxis { Axis = "Yaw", Channel = 15 },
            } };
            OutputController.Initialize(config); GimbalController.ApplyProfile(config);
            var held = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            connection.Configure = () => held.Task;
            var pending = GimbalController.RequestScreenTargetAsync(10, 0, 20);
            WaitForConfigure(connection);
            if (change == "camera") { config.SelectedVideoInputName = "Camera B"; }
            else if (change == "calibration") { config.SelectedGimbalAxes[1].MaximumPwm = 1900; }
            else { GimbalController.NotifyServoInput(config, 14, 1800); }
            held.SetResult(true);
            result = Complete(pending);
            Require(!result.Submitted && connection.ServoTargets.Count == 1,
                "A changed " + change + " dispatched the remaining axes of a retired gimbal pose");
        }
    }
    private static void ContinuousManualServoTargets()
    {
        foreach (bool screen in new[] { false, true })
        foreach (string interruption in new[] { "none", "slider", "mode", "arrow" })
        {
            var connection = Reset();
            var config = new NOMADConfig { SelectedGimbalAxes = new[]
            {
                new ServoGimbalAxis { Axis = "Pitch", Channel = 14 },
                new ServoGimbalAxis { Axis = "Yaw", Channel = 15 },
                new ServoGimbalAxis { Axis = "Roll", Channel = 16 },
            } };
            OutputController.Initialize(config); GimbalController.ApplyProfile(config);
            var held = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            connection.Configure = () => held.Task;
            Action sample = () =>
            {
                if (screen) { GimbalController.ApplyScreenStick(1, 1, .05f, true); }
                else { GimbalController.ApplyStick(1, 1, .05f, true); }
            };
            sample(); WaitForConfigure(connection);
            try
            {
                sample(); sample();
                if (interruption == "slider") { GimbalController.NotifyServoInput(config, 14, 1800); }
                if (interruption == "mode") { GimbalController.SetMode(MountMode.Neutral); }
                if (interruption == "arrow") { GimbalController.NudgeScreenTarget(1, 0, 2); }
            }
            finally { held.SetResult(true); }
            DrainGimbal();
            if (interruption != "none")
            {
                Require(connection.ServoTargets.Count == 1,
                    interruption + " did not retire the remaining axes of a continuous manual pose");
                continue;
            }
            Require(connection.ServoTargets.Select(target => target.Channel).SequenceEqual(new[] { 14, 15, 16 }) &&
                connection.ServoAcknowledgements.All(ack => ack),
                "Continuous manual samples starved later servo axes while the first ACK was pending");
            Require(GimbalController.TargetPitchDeg == 9 && GimbalController.TargetYawDeg == (screen ? 9 : 0) &&
                GimbalController.TargetRollDeg == (screen ? 0 : -9),
                "An older manual command completion overwrote newer integrated input");
            connection.Configure = () => Task.FromResult(true);
            sample(); DrainGimbal();
            Require(connection.ServoTargets.Count == 6 && connection.ServoTargets[3].Pwm > connection.ServoTargets[0].Pwm &&
                (screen ? connection.ServoTargets[4].Pwm > connection.ServoTargets[1].Pwm :
                    connection.ServoTargets[5].Pwm < connection.ServoTargets[2].Pwm),
                "A fresh manual sample failed to send accumulated motion to later axes");
        }
    }
    private static void ServoTrackingStream()
    {
        var connection = Reset();
        var config = new NOMADConfig { SelectedGimbalAxes = new[]
        {
            new ServoGimbalAxis { Axis = "Pitch", Channel = 14 },
            new ServoGimbalAxis { Axis = "Yaw", Channel = 15, Reversed = true },
        } };
        OutputController.Initialize(config); GimbalController.ApplyProfile(config); GimbalController.ClaimTracking();
        Require(Complete(Track()).Submitted && connection.ServoAcknowledgements.SequenceEqual(new[] { true, true }),
            "First tracking pose did not confirm both configured servo outputs");
        // The link can continue writing while ACKs are delayed or absent.
        var held = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        connection.Configure = () => held.Task;
        for (int frame = 0; frame < 30; frame++)
        {
            var result = Complete(Track(-12 + frame * .1f, 7 + frame * .1f));
            Require(result.Submitted && !result.Message.Contains("acknowledged"),
                "Continuous servo tracking waited on an ACK or claimed acceptance");
        }
        Require(connection.ServoTargets.Count == 62 && connection.ServoAcknowledgements.Skip(2).All(ack => !ack),
            "Fresh servo targets were queued behind acknowledgements or emitted native retries");
        int commands = connection.ServoTargets.Count;
        Require(Complete(Track(current: () => false)).Outcome == GimbalRequestOutcome.NotSent &&
            connection.ServoTargets.Count == commands, "A retired tracking sample entered the servo stream");
        GimbalController.ClaimTracking(); connection.ConfigureEntered.Reset();
        var pending = Track(); WaitForConfigure(connection);
        GimbalController.SetTargetAngles(0, 0); held.SetResult(true);
        Require(Complete(pending).Outcome == GimbalRequestOutcome.Unknown &&
            connection.ServoTargets.Count == commands + 1 && connection.ServoAcknowledgements.Last(),
            "A new claim skipped confirmation or manual override let its remaining servo axes through");
        connection.Configure = () => Task.FromResult(true);
        Require(Complete(Track()).Submitted && connection.ServoAcknowledgements.Skip(commands).All(ack => ack),
            "Partially retired servo setup incorrectly enabled streaming");
        config.SelectedGimbalAxes[0].MaximumPwm = 1900;
        Require(Complete(Track()).Submitted && connection.ServoAcknowledgements.Last(),
            "Changed calibration reused a previous servo stream's confirmation");
        Require(Complete(Track()).Submitted && !connection.ServoAcknowledgements.Last(),
            "Confirmed replacement calibration did not enable fresh streaming");
        Require(Complete(GimbalController.RequestScreenTargetAsync(0, 0, 0)).Submitted &&
            connection.ServoAcknowledgements.Last(), "A manual servo target lost native ACK/retry semantics");
        Require(Complete(Track()).Submitted && connection.ServoAcknowledgements.Last(),
            "Manual control did not invalidate the continuous servo confirmation");
        GimbalController.ClaimTracking(); connection.Configure = () => Task.FromResult(false);
        for (int attempt = 0; attempt < 2; attempt++)
        {
            commands = connection.ServoTargets.Count;
            Require(Complete(Track()).Outcome == GimbalRequestOutcome.Unknown &&
                connection.ServoTargets.Count == commands + 1 && connection.ServoAcknowledgements.Last(),
                "A rejected setup enabled servo streaming or dispatched its remaining axes");
        }
        connection.Configure = () => Task.FromResult(true);
        Require(Complete(Track()).Submitted && Complete(Track()).Submitted && !connection.ServoAcknowledgements.Last(),
            "Fresh confirmation after a rejected setup did not restore the servo stream");
        connection.sysidcurrent = 8;
        Require(Complete(Track()).Submitted && connection.LastSystem == 8 && connection.ServoAcknowledgements.Last(),
            "New aircraft identity reused the previous servo stream's confirmation");
    }

    private static int Main()
    {
        var connection = Reset();
        var result = Send();
        Require(result.Submitted && result.Message.Contains("not confirmed"), "Submission claimed vehicle acceptance");
        Require(connection.Modes == 1 && connection.Targets == 1 && connection.LastSystem == 4 &&
            connection.LastComponent == 2 && connection.Pitch == -1200 && connection.Roll == 700 && connection.Yaw == 0 &&
            !connection.AcknowledgementRequested, "Host target IDs/centidegrees/ACK policy changed");
        Send();
        Require(connection.Modes == 1 && connection.Targets == 2, "Repeated target reconfigured mount mode");
        TrackingAngles();
        ScreenAxisTargets();
        MalformedTargets();
        TrackingRetirement();
        BlockingNativeHelpers();
        ManualOverride(() => GimbalController.RequestPitchRollTarget(0, 0), "Manual angle request");
        ManualOverride(() => GimbalController.SetMode(MountMode.Neutral), "Manual mode request");
        ManualOverride(() => GimbalController.NudgeTarget(2, 0), "Arrow nudge");
        ManualOverride(() => { _ = GimbalController.SetModeAndTargetAsync(MountMode.MavlinkTargeting, 0, 0); }, "Snap target");
        ManualOverride(() => GimbalController.ApplyStick(1, 0, 0.05f, true), "Sending manual stick");
        ManualOverride(() => GimbalController.ApplyStick(1, 0, 0.05f, false), "Local manual stick");
        ManualOverride(() => GimbalController.SetTargetAngles(0, 0), "Manual target edit");
        ManualOverride(() => GimbalController.ApplyScreenStick(1, 0, .05f, true), "Mapped screen stick");
        ManualOverride(() => GimbalController.NudgeScreenTarget(1, 0, 2), "Mapped screen arrow");
        ManualOverride(() => { _ = GimbalController.RequestScreenTargetAsync(0, 0, 0); }, "Mapped screen preset");
        NeutralAndDispatchedTracking();
        PendingManualRetirement();
        SharedTrackingClaim();
        ServoGimbalTargets();
        ContinuousManualServoTargets();
        ServoTrackingStream();
        RefusesChangedTarget(c => c.sysidcurrent = 8, "system selection");
        RefusesChangedTarget(c => c.compidcurrent = 9, "component selection");
        RefusesChangedTarget(c => c.BaseStream = new MissionPlanner.TestStream(), "stream replacement");
        RefusesChangedTarget(c => MissionPlanner.MainV2.comPort = new MissionPlanner.MAVLinkInterface(), "connection replacement");
        RefusesChangedTarget(c => OutputController.Initialize(new NOMADConfig()), "profile replacement");
        RefusesChangedTarget(c => c.Close(), "disconnect/reconnect on same stream");
        RefusesChangedTarget(c => c.Select(), "selection round trip");
        foreach (string unavailable in new[] { "closed", "readonly", "playback", "nativebusy" })
        {
            connection = Reset();
            connection.BaseStream.IsOpen = unavailable != "closed";
            connection.ReadOnly = unavailable == "readonly";
            connection.logreadmode = unavailable == "playback";
            connection.giveComport = unavailable == "nativebusy";
            Require(!Send().Submitted && connection.Modes == 0 && connection.Targets == 0,
                unavailable + " connection emitted a command");
            Require(!Complete(Track()).Submitted && connection.Modes == 0 && connection.Targets == 0,
                unavailable + " connection admitted tracking");
            if (unavailable == "nativebusy")
            {
                Require(Send().Outcome == GimbalRequestOutcome.Busy && connection.Modes == 0,
                    "Known native busy must be distinguished before any dispatch");
                connection.giveComport = false;
                Require(Send().Submitted && connection.Modes == 1 && connection.Targets == 1,
                    "A fresh request after native busy did not resume normally");
            }
        }
        foreach (double invalid in new[] { double.NaN, double.PositiveInfinity, -91, 91 })
        {
            connection = Reset();
            Require(!Send(pitch: invalid).Submitted && connection.Modes == 0, "Invalid pitch emitted a command");
        }
        connection = Reset();
        Require(!Send(current: () => false).Submitted && connection.Modes == 0, "Retired input callback dispatched");
        connection.Configure = () => Task.FromResult(false);
        Require(!Send().Submitted && connection.Targets == 0, "Rejected native configure allowed target");
        connection = Reset();
        var held = new TaskCompletionSource<bool>();
        connection.Configure = () => held.Task;
        var pending = OutputController.SendGimbalTargetAsync(2, -12, 7, 0, OutputController.Generation, null);
        WaitForConfigure(connection);
        var replacement = new MissionPlanner.MAVLinkInterface();
        MissionPlanner.MainV2.comPort = replacement;
        held.SetResult(true);
        result = Complete(pending);
        Require(result.Outcome == GimbalRequestOutcome.Unknown && connection.Targets == 0 && replacement.Modes == 0 &&
            replacement.Targets == 0, "Pending original command retargeted a new connection");
        connection = Reset();
        connection.Configure = () => throw new InvalidOperationException("injected write fault");
        result = Send();
        Require(result.Outcome == GimbalRequestOutcome.Unknown && connection.Modes == 1 && connection.Targets == 0,
            "Write exception was retried or presented as definite success");
        var input = new GimbalInputState();
        Require(!input.CanMove(false, 0, 0, 1), "Invalid sample was accepted");
        Require(!input.CanMove(true, 1, 0, 1), "Invalid sample falsely armed held deflection");
        Require(!input.CanMove(true, 0, 0, 1), "Centered stick moved the gimbal");
        Require(input.CanMove(true, 1, 0, 1), "Fresh motion after real center was blocked");
        Require(!input.CanMove(true, 1, 0, 2), "Held deflection crossed aircraft generation");
        connection = Reset();
        var repeated = System.Windows.Forms.Message.Create(IntPtr.Zero, 0x0100,
            (IntPtr)System.Windows.Forms.Keys.Up, (IntPtr)(1L << 30));
        OutputController.Initialize(new NOMADConfig());
        Require(!GimbalArrowKeyFilter.TryHandleMessage(ref repeated) && connection.Modes == 0,
            "Held arrow repeated into new profile");
        MissionPlanner.MainV2.comPort = new MissionPlanner.MAVLinkInterface();
        Require(!GimbalArrowKeyFilter.TryHandleMessage(ref repeated) && MissionPlanner.MainV2.comPort.Modes == 0,
            "Held arrow repeated into new selected aircraft");
        int reads = 0;
        OutputController.CurrentConnection = () => { reads++; return connection; };
        OutputController.Shutdown();
        OutputController.RefreshConnection();
        long inactive = OutputController.Generation;
        Require(reads == 0, "Shutdown output owner resurrected host subscriptions");
        Require(!Send(inactive).Submitted && reads == 0, "Shutdown owner allowed native dispatch");
        OutputController.CurrentConnection = () => MissionPlanner.MainV2.comPort;
        Console.WriteLine("Native output tests passed (identity, profile, tracking, manual override, units and faults).");
        return 0;
    }
}
