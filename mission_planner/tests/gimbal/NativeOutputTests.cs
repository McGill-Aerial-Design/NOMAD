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
        public double Pitch, Roll;
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
        OutputController.Initialize(new NOMADConfig());
        return connection;
    }
    private static GimbalRequestResult Send(long? generation = null, Func<bool> current = null,
        double pitch = -12, double roll = 7) => OutputController.SendGimbalAsync(2, pitch, roll,
            generation ?? OutputController.Generation, current).GetAwaiter().GetResult();
    private static void Require(bool condition, string diagnostic)
    {
        if (!condition) { throw new Exception(diagnostic); }
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
    private static int Main()
    {
        var connection = Reset();
        var result = Send();
        Require(result.Submitted && result.Message.Contains("not confirmed"), "Submission claimed vehicle acceptance");
        Require(connection.Modes == 1 && connection.Targets == 1 && connection.LastSystem == 4 &&
            connection.LastComponent == 2 && connection.Pitch == -1200 && connection.Roll == 700 &&
            !connection.AcknowledgementRequested, "Host target IDs/centidegrees/ACK policy changed");
        Send();
        Require(connection.Modes == 1 && connection.Targets == 2, "Repeated target reconfigured mount mode");
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
        var pending = OutputController.SendGimbalAsync(2, -12, 7, OutputController.Generation, null);
        var replacement = new MissionPlanner.MAVLinkInterface();
        MissionPlanner.MainV2.comPort = replacement;
        held.SetResult(true);
        result = pending.GetAwaiter().GetResult();
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
        Console.WriteLine("Native output tests passed (identity, profile, stale input, units and faults).");
        return 0;
    }
}
