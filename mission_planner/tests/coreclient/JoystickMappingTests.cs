// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Threading;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

namespace MissionPlanner.Joystick
{
    public interface IMyJoystickState { bool[] GetButtons(); int[] GetSlider(); int X { get; } }
}
namespace NOMAD.MissionPlanner
{
    public sealed partial class NomadJoystickService
    {
        private readonly NOMADConfig _config;
        private static readonly System.Collections.Generic.Dictionary<string, System.Reflection.PropertyInfo> AxisProps = BuildAxisMap();
        public NomadJoystickService(NOMADConfig config) { _config = config; }
        internal System.Threading.Tasks.Task TestButtons(bool[] buttons) => DriveActuatorButtons(new ButtonState(buttons));
        internal void TestReset() => ResetActuatorInput();
        private sealed class ButtonState : global::MissionPlanner.Joystick.IMyJoystickState
        {
            private readonly bool[] _buttons;
            internal ButtonState(bool[] buttons) { _buttons = buttons; }
            public bool[] GetButtons() => _buttons;
            public int[] GetSlider() => null;
            public int X => 32767;
        }
    }
}

internal static partial class NomadCoreClientTests
{
    private static bool[] HidButtons(int index = -1)
    {
        var buttons = new bool[10];
        if (index >= 0)
        {
            buttons[index] = true;
        }
        return buttons;
    }
    private static void WaitForHid(MockRuntime runtime, int count)
    {
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (runtime.CommandCount < count && DateTime.UtcNow < deadline)
        {
            System.Windows.Forms.Application.DoEvents(); Thread.Sleep(1);
        }
        Expect(runtime.CommandCount == count, "USB HID intent count: expected=" + count + " observed=" + runtime.CommandCount + " last=" + runtime.LastCommandType);
    }
    private static NOMADConfig HidConfig(int port, string operation = "activate") => new NOMADConfig
    {
        CoreRuntimePort = port, JoystickSw1UpAction = "release:" + operation,
        JoystickButtonIndices = new[] { 4, 5, 6, 7, 8, 9 }, JoystickKillSwitchEnabled = false
    };
    private static void Joystick_AuthorizationTests()
    {
        using (var runtime = new MockRuntime(2, semanticResponse: request => SemanticResponse(request)))
        {
            var config = HidConfig(runtime.Port);
            OutputController.Initialize(config);
            var service = new NomadJoystickService(config);
            service.TestButtons(HidButtons());
            WaitForHid(runtime, 1);
            service.TestButtons(HidButtons(4));
            WaitForHid(runtime, 2);
            service.TestButtons(HidButtons(4));
            service.TestButtons(HidButtons(4));
            runtime.Wait();
            Expect(runtime.Commands[0]["operation"].ToString() == "neutral", "positively observed HID neutral is sent to backend");
            Expect(runtime.Commands[1]["operation"].ToString() == "activate" && Convert.ToInt32(runtime.Commands[1]["input_slot"]) == 0,
                "first physical rising edge sends one semantic intent with stable input slot");
            Expect(!runtime.Commands[1].ContainsKey("channel") && !runtime.Commands[1].ContainsKey("pwm_microseconds"),
                "HID never chooses physical output mapping");
            Expect(runtime.CommandCount == 2, "held input does not multiply confirmation requests");
        }
        using (var runtime = new MockRuntime(8, semanticResponse: request => SemanticResponse(request)))
        {
            var config = HidConfig(runtime.Port, "positive");
            OutputController.Initialize(config);
            OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
            var service = new NomadJoystickService(config);
            service.TestButtons(HidButtons()); WaitForHid(runtime, 2);
            service.TestButtons(HidButtons(4)); WaitForHid(runtime, 3);
            var firstRelease = service.TestButtons(HidButtons()); WaitForHid(runtime, 5);
            Expect(firstRelease.Wait(5000), "first release/neutral input sequence completes");
            service.TestButtons(HidButtons(4)); WaitForHid(runtime, 6);
            var secondRelease = service.TestButtons(HidButtons()); WaitForHid(runtime, 8);
            Expect(secondRelease.Wait(5000), "second release/neutral input sequence completes");
            service.TestButtons(HidButtons());
            runtime.Wait();
            Expect(runtime.Commands.FindAll(c => c.ContainsKey("operation") && c["operation"].ToString() == "release_input").Count == 2,
                "two falling edges translate backend release_operation to exactly two backend-owned release intents");
            Expect(runtime.Commands.FindAll(c => c.ContainsKey("operation") && c["operation"].ToString() == "neutral").Count == 3,
                "initial neutral plus two return cycles emit three actual neutral observations");
        }
        using (var runtime = new MockRuntime(3, semanticResponse: request => SemanticResponse(request)))
        {
            var config = HidConfig(runtime.Port);
            OutputController.Initialize(config);
            var service = new NomadJoystickService(config);
            service.TestButtons(HidButtons()); WaitForHid(runtime, 1);
            service.TestButtons(HidButtons(4)); WaitForHid(runtime, 2);
            service.TestButtons(null); WaitForHid(runtime, 3);
            service.TestButtons(null);
            runtime.Wait();
            Expect(runtime.LastCommand["operation"].ToString() == "safe", "lost input sends a semantic safe/reset intent");
            Expect(runtime.Commands.FindAll(c => c["operation"].ToString() == "neutral").Count == 1,
                "lost input does not fabricate neutral authorization evidence");
        }
        using (var runtime = new MockRuntime(2, semanticResponse: request => SemanticResponse(request)))
        {
            var config = HidConfig(runtime.Port);
            config.JoystickButtonIndices = new[] { 0, 1, 2, 3, 4, 5 };
            config.JoystickSw1DownAction = "release:safe";
            OutputController.Initialize(config);
            var service = new NomadJoystickService(config);
            service.TestButtons(new[] { false, false }); WaitForHid(runtime, 1);
            service.TestButtons(new[] { false, true }); WaitForHid(runtime, 2);
            runtime.Wait();
            Expect(runtime.Commands[0]["operation"].ToString() == "neutral" &&
                Convert.ToInt32(runtime.Commands[0]["input_slot"]) == Convert.ToInt32(runtime.Commands[1]["input_slot"]),
                "same actuator UP/DOWN uses one physical-switch identity and one neutral event");
        }
        using (var runtime = new MockRuntime(2, semanticResponse: request => SemanticResponse(request)))
        {
            var config = HidConfig(runtime.Port);
            config.JoystickButtonIndices = new[] { 0, 99, 98, 97, 96, 95 };
            OutputController.Initialize(config);
            var service = new NomadJoystickService(config);
            service.TestButtons(new[] { true });
            Expect(runtime.CommandCount == 0, "initial held sample is not a rising edge");
            service.TestButtons(new[] { false }); WaitForHid(runtime, 1);
            service.TestButtons(new[] { true }); WaitForHid(runtime, 2);
            runtime.Wait();
            Expect(runtime.Commands[1]["operation"].ToString() == "activate", "single mapped button works despite unmapped indices beyond device range");
        }
        Axis_InvalidReadingsNeverSendMidpoint();
        Hid_OrdinaryTransitionCancelsUnsentSnapshot();
        Hid_DelayedPhysicalHelloDiscardsActivationAndNeutral();
        Hid_SnapshotChangeAfterWriteDoesNotCancelMutation();
        Hid_DelayedHelloDiscardsStaleIntent("activate");
        Hid_DelayedHelloDiscardsStaleIntent("neutral");
        Axis_DelayedHelloDiscardsStalePosition(false);
        Axis_DelayedHelloDiscardsStalePosition(true);
        Hid_LostInputCancelsUnsentSnapshot(false);
        Hid_LostInputCancelsUnsentSnapshot(true);
        using (var runtime = new MockRuntime(1, semanticResponse: request => SemanticResponse(request)))
        {
            var config = HidConfig(runtime.Port);
            OutputController.Initialize(config);
            var service = new NomadJoystickService(config);
            var contradictory = HidButtons(4); contradictory[5] = true;
            service.TestButtons(contradictory); WaitForHid(runtime, 1);
            service.TestButtons(contradictory);
            runtime.Wait();
            Expect(runtime.LastCommand["operation"].ToString() == "safe", "contradictory physical inputs send safe without a confirmation");
        }
    }
    private sealed class AxisState : global::MissionPlanner.Joystick.IMyJoystickState
    {
        public int X { get; set; } = 32767;
        public int[] Sliders { get; set; }
        public bool[] GetButtons() => new bool[6];
        public int[] GetSlider() => Sliders;
    }
    private static void Axis_InvalidReadingsNeverSendMidpoint()
    {
        var state = new AxisState();
        Expect(!NomadJoystickService.TryReadAxisNorm(state, "missing", false, 0.08f, out _), "unknown axis is invalid, not centered");
        Expect(!NomadJoystickService.TryReadAxisNorm(state, "Slider1", false, 0.08f, out _), "missing slider is invalid, not centered");
        state.X = 65536;
        Expect(!NomadJoystickService.TryReadAxisNorm(state, "X", false, 0.08f, out _), "out-of-range HID reading is invalid");
        state.X = 32767;
        Expect(!NomadJoystickService.TryReadAxisNorm(state, "X", false, float.NaN, out _), "invalid physical deadzone is rejected");
        Expect(NomadJoystickService.TryReadAxisNorm(state, "X", false, 0.08f, out var center) && center == 0,
            "positively observed center remains a valid axis reading");
        using var runtime = new MockRuntime(3, semanticResponse: request => SemanticResponse(request));
        var config = HidConfig(runtime.Port);
        config.JoystickPositionActuatorId = "release";
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        service.TranslatePositionInput(true, 0.75f); WaitForHid(runtime, 2);
        service.TranslatePositionInput(false, 0); WaitForHid(runtime, 3);
        service.TranslatePositionInput(false, 0);
        runtime.Wait();
        Expect(Convert.ToDouble(runtime.Commands[1]["value"]) == 0.875 && runtime.Commands[2]["operation"].ToString() == "safe",
            "invalid reading after valid position emits one safe intent and no midpoint command");
    }

    private static void Hid_LostInputCancelsUnsentSnapshot(bool releasedToNeutral)
    {
        using var releaseEntered = new ManualResetEventSlim();
        using var releaseResponse = new ManualResetEventSlim();
        using var runtime = new MockRuntime(5, semanticResponse: request =>
        {
            if (request.ContainsKey("operation") && request["operation"].ToString() == "release_input")
            {
                releaseEntered.Set();
                if (!releaseResponse.Wait(5000))
                {
                    throw new TimeoutException("Controlled HID release response was not allowed.");
                }
            }
            return SemanticResponse(request);
        });
        var config = HidConfig(runtime.Port, "positive");
        config.JoystickSw1DownAction = "release:negative";
        OutputController.Initialize(config);
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        var service = new NomadJoystickService(config);
        service.TestButtons(HidButtons()); WaitForHid(runtime, 2);
        service.TestButtons(HidButtons(4)); WaitForHid(runtime, 3);
        var pendingBatch = service.TestButtons(HidButtons(releasedToNeutral ? -1 : 5));
        Expect(releaseEntered.Wait(5000), "backend release response is deliberately pending");
        service.TestReset();
        releaseResponse.Set();
        WaitForHid(runtime, 5);
        runtime.Wait();
        Expect(pendingBatch.Wait(5000), "delayed physical input batch completes after reset");
        Expect(runtime.Commands.FindAll(c => c.ContainsKey("operation") && c["operation"].ToString() == "neutral").Count == 1,
            "lost input invalidates an unsent neutral observation behind a delayed release");
        Expect(runtime.Commands.FindAll(c => c.ContainsKey("operation") && c["operation"].ToString() == "negative").Count == 0,
            "lost input invalidates an unsent opposite-direction intent behind a delayed release");
        Expect(runtime.LastCommand["operation"].ToString() == "safe", "actual reset remains deliverable after stale input cancellation");
    }

    private static void Hid_OrdinaryTransitionCancelsUnsentSnapshot()
    {
        using var releaseEntered = new ManualResetEventSlim();
        using var releaseResponse = new ManualResetEventSlim();
        using var runtime = new MockRuntime(6, semanticResponse: request =>
        {
            if (request.ContainsKey("operation") && request["operation"].ToString() == "release_input")
            {
                releaseEntered.Set();
                if (!releaseResponse.Wait(5000))
                {
                    throw new TimeoutException("Delayed release was not allowed to return.");
                }
            }
            return SemanticResponse(request);
        });
        var config = HidConfig(runtime.Port, "positive");
        config.JoystickSw1DownAction = "release:negative";
        OutputController.Initialize(config);
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        var service = new NomadJoystickService(config);
        service.TestButtons(HidButtons()); WaitForHid(runtime, 2);
        service.TestButtons(HidButtons(4)); WaitForHid(runtime, 3);
        var capturedPress = service.TestButtons(HidButtons(5));
        Expect(releaseEntered.Wait(5000), "first direction release is deliberately delayed");
        var physicalFall = service.TestButtons(HidButtons());
        releaseResponse.Set();
        WaitForHid(runtime, 6);
        Expect(capturedPress.Wait(5000) && physicalFall.Wait(5000), "both physical snapshots finish dispatching");
        runtime.Wait();
        Expect(runtime.Commands.FindAll(c => c.ContainsKey("operation") && c["operation"].ToString() == "negative").Count == 0,
            "captured opposite press cannot be sent after that button physically fell");
        Expect(runtime.Commands.FindAll(c => c.ContainsKey("operation") && c["operation"].ToString() == "neutral").Count == 2,
            "fresh actual neutral remains deliverable after ordinary snapshot replacement");
    }

    private static void Hid_DelayedHelloDiscardsStaleIntent(string operation)
    {
        using var runtime = new AsyncRuntime(1, delayHello: true);
        runtime.ReleaseResponses();
        bool physicalSnapshotCurrent = true;
        var client = new NomadCoreClient("test-key", runtime.Port);
        var request = client.ActuatorActionAsync("release", operation, "hid", 0,
            inputStillCurrent: () => physicalSnapshotCurrent);
        runtime.WaitForHello();
        physicalSnapshotCurrent = false;
        runtime.ReleaseHello();
        var result1 = request.GetAwaiter().GetResult();
        runtime.Wait();
        Expect(result1.Outcome == NomadCoreRequestOutcome.NotAttempted && result1.ErrorCode == "stale_input" && runtime.Commands.Count == 0,
            "stale " + operation + " is discarded after hello before mutation transmission");
    }

    private static void Axis_DelayedHelloDiscardsStalePosition(bool reset)
    {
        using var runtime = new AsyncRuntime(3, delayHello: true, semanticResponse: request => SemanticResponse(request));
        runtime.ReleaseResponses();
        var config = HidConfig(runtime.Port);
        config.JoystickPositionActuatorId = "release";
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        runtime.ReleaseHello();
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        runtime.DelayHelloAgain();
        service.TranslatePositionInput(true, 0.75f);
        runtime.WaitForHello();
        if (reset)
        {
            service.TestReset();
        }
        service.TranslatePositionInput(false, 0);
        runtime.ReleaseHello();
        runtime.Wait();
        Expect(runtime.Commands.Count == 2 && runtime.Commands[1]["operation"].ToString() == "safe",
            "delayed old position is never transmitted after physical " + (reset ? "reset" : "loss") + "; safe intent remains available");
    }

    private static void Hid_DelayedPhysicalHelloDiscardsActivationAndNeutral()
    {
        using var runtime = new AsyncRuntime(3, delayHello: true, semanticResponse: request => SemanticResponse(request));
        runtime.ReleaseResponses();
        var config = HidConfig(runtime.Port);
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        var neutral = service.TestButtons(HidButtons());
        runtime.WaitForHello();
        var activation = service.TestButtons(HidButtons(4));
        service.TestReset();
        runtime.ReleaseHello();
        runtime.Wait();
        Expect(neutral.Wait(5000) && activation.Wait(5000), "delayed physical event requests finish without mutation retries");
        Expect(runtime.Commands.Count == 1 && runtime.Commands[0]["operation"].ToString() == "safe",
            "actual physical loss during hello transmits neither the old activation nor old neutral; safe remains deliverable");
    }

    private static void Hid_SnapshotChangeAfterWriteDoesNotCancelMutation()
    {
        using var runtime = new AsyncRuntime(1, semanticResponse: request => SemanticResponse(request));
        bool current = true;
        var request = new NomadCoreClient("test-key", runtime.Port).ActuatorActionAsync("release", "activate", "hid", 0,
            inputStillCurrent: () => current);
        runtime.WaitForCommands(1);
        current = false;
        runtime.ReleaseResponses();
        var result1 = request.GetAwaiter().GetResult();
        runtime.Wait();
        Expect(result1.Succeeded && runtime.Commands.Count == 1, "later physical change does not close or retry an already-transmitted mutation");
    }

}
