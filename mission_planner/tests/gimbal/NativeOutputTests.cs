// SPDX-License-Identifier: Apache-2.0
using System;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

// Native host doubles exercise dispatch and identity fences without aircraft or MP application state.
public class MAVLink
{
    public enum MAV_CMD { DO_MOUNT_CONFIGURE }
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
        public int Modes, Targets;
        public byte LastSystem, LastComponent;
        public double Pitch, Roll, Yaw;
        public Action TargetDispatched;
        public bool AcknowledgementRequested;
        public void Close() { CommsClose?.Invoke(this, EventArgs.Empty); }
        public void Select() { MavChanged?.Invoke(this, EventArgs.Empty); }
        public Task<bool> doCommandAsync(byte system, byte component, MAVLink.MAV_CMD command,
            float p1, float p2, float p3, float p4, float p5, float p6, float p7, bool requireack)
        {
            Modes++;
            LastSystem = system;
            LastComponent = component;
            AcknowledgementRequested = requireack;
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
        public bool GimbalArrowKeysEnabled;
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
        result = Complete(GimbalController.RequestPitchRollTargetAsync(-8, 4));
        Require(result.Submitted && connection.Pitch == -800 && connection.Roll == 400 && connection.Yaw == 0 &&
            GimbalController.TargetYawDeg == 0, "Manual roll was reinterpreted as tracking yaw");
        GimbalController.ApplyStick(1, 0, 10, 0.1f, true);
        Require(connection.Roll == 300 && connection.Yaw == 0, "Manual stick roll convention changed");
    }
    private static void MalformedTargets()
    {
        foreach (double invalid in new[] { double.NaN, double.NegativeInfinity, double.PositiveInfinity, -90.01, 90.01 })
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
        long second = GimbalController.ClaimTracking();
        held.SetResult(true);
        Require(second > first && Complete(pending).Outcome == GimbalRequestOutcome.Unknown && connection.Targets == 0,
            "New video owner allowed the previous owner's pending target through");
        Require(Complete(Track(current: () => second == GimbalController.CurrentTrackingClaim)).Submitted &&
            connection.Targets == 1 && GimbalController.CurrentTrackingClaim == second,
            "Retired owner completion invalidated the new tracking claim");
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
        MalformedTargets();
        TrackingRetirement();
        ManualOverride(() => GimbalController.RequestPitchRollTarget(0, 0), "Manual angle request");
        ManualOverride(() => GimbalController.SetMode(MountMode.Neutral), "Manual mode request");
        ManualOverride(() => GimbalController.NudgeTarget(2, 0), "Arrow nudge");
        ManualOverride(() => { _ = GimbalController.SetModeAndTargetAsync(MountMode.MavlinkTargeting, 0, 0); }, "Snap target");
        ManualOverride(() => GimbalController.ApplyStick(1, 0, 0.05f, true), "Sending manual stick");
        ManualOverride(() => GimbalController.ApplyStick(1, 0, 0.05f, false), "Local manual stick");
        ManualOverride(() => GimbalController.SetTargetAngles(0, 0), "Manual target edit");
        NeutralAndDispatchedTracking();
        PendingManualRetirement();
        SharedTrackingClaim();
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
