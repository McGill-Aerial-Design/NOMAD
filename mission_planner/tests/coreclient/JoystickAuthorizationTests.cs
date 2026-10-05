// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Threading;
using NOMAD.MissionPlanner;

namespace MissionPlanner.Joystick
{
    public interface IMyJoystickState { bool[] GetButtons(); }
}

namespace NOMAD.MissionPlanner
{
    internal static class AudioAlerts
    {
        public static void Speak(string message, string component) { }
    }

    // Hardware and unrelated axis polling are replaced; the production switch path is compiled unchanged.
    public sealed partial class NomadJoystickService
    {
        private readonly NOMADConfig _config;
        public NomadJoystickService(NOMADConfig config) { _config = config; }
        internal void TestButtons(bool[] buttons, long? nowMs = null)
        {
            DrivePayloadButtons(new ButtonState(buttons), nowMs ?? PayloadReleaseInterlock.NowMs);
        }
        internal void TestReset() => ResetPayloadInput();
        private sealed class ButtonState : global::MissionPlanner.Joystick.IMyJoystickState
        {
            private readonly bool[] _buttons;
            internal ButtonState(bool[] buttons) { _buttons = buttons; }
            public bool[] GetButtons() => _buttons;
        }
    }
}

internal static partial class NomadCoreClientTests
{
    private static void Joystick_AuthorizationTests()
    {
        Joystick_OneEdgeAndHeldButtonSendNothing();
        Joystick_ResetExpiryAndInvalidInputFailClosed();
        Joystick_AuthorizedDropSendsOnceAndSharesState();
        Joystick_AuthorizedPumpFiresOnce();
        Joystick_RetractsSuccessfulUiRelease();
        PayloadBoundary_RejectsMissingReusedAndExpiredAuthorization();
        Joystick_UncertainDropRequiresExplicitSafeRecovery();
        PayloadBoundary_ReservesOutputUntilUncertaintyRecorded();
        Joystick_LostInputSendsOneSafeReelStop(false);
        Joystick_LostInputSendsOneSafeReelStop(true);
    }

    private static bool[] Buttons(int slot = -1)
    {
        var buttons = new bool[7];
        if (slot >= 0)
        {
            buttons[slot] = true;
        }
        return buttons;
    }

    private static void SwitchEdge(NomadJoystickService service, int slot, long? nowMs = null)
    {
        service.TestButtons(Buttons(), nowMs);
        service.TestButtons(Buttons(slot), nowMs);
    }

    private static void Joystick_OneEdgeAndHeldButtonSendNothing()
    {
        using var runtime = new MockRuntime(0);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        service.TestButtons(Buttons(0));
        service.TestButtons(Buttons(0));
        SwitchEdge(service, 0);
        service.TestButtons(Buttons(0));
        SwitchEdge(service, 5);
        service.TestButtons(Buttons(5));
        Expect(runtime.CommandCount == 0, "startup press, one edge and held payload switch send zero mutations");
        runtime.Wait();
    }

    private static void Joystick_ResetExpiryAndInvalidInputFailClosed()
    {
        using var runtime = new MockRuntime(0);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        SwitchEdge(service, 0, 0);
        SwitchEdge(service, 0, 1);
        SwitchEdge(service, 0, 4000);
        service.TestReset();
        SwitchEdge(service, 0);
        SwitchEdge(service, 0);
        service.TestButtons(null);
        SwitchEdge(service, 0);
        SwitchEdge(service, 0);
        var invalid = Buttons(0);
        invalid[1] = true;
        service.TestButtons(invalid);
        SwitchEdge(service, 0);
        service.TestButtons(new bool[2]);
        SwitchEdge(service, 0);
        service.TestReset();
        SwitchEdge(service, 0, 200);
        SwitchEdge(service, 0, 201);
        SwitchEdge(service, 0, 100);
        service.TestReset();
        config.JoystickSw2UpAction = "DropToggleP1";
        SwitchEdge(service, 0);
        SwitchEdge(service, 2);
        SwitchEdge(service, 0);
        Expect(runtime.CommandCount == 0,
            "expiry, reset, lost input and malformed switch state discard pending authorization");
        Expect(runtime.CommandCount == 0,
            "clock reversal and different mapped slots cannot combine confirmation state");
        runtime.Wait();
    }

    private static void Joystick_AuthorizedDropSendsOnceAndSharesState()
    {
        using var runtime = new MockRuntime(2);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
        var service = new NomadJoystickService(config);
        SwitchEdge(service, 0);
        SwitchEdge(service, 0);
        Expect(runtime.CommandCount == 0, "two drop confirmations still send zero mutations");
        SwitchEdge(service, 0);
        WaitForCommandedRelease(true);
        Expect(runtime.CommandCount == 1, "three deliberate joystick confirmations send exactly one release request");
        Expect(Convert.ToInt32(runtime.LastCommand["pwm_microseconds"]) == 2000,
            "authorized request commands release PWM");
        SwitchEdge(service, 0);
        WaitForCommandedRelease(false);
        runtime.Wait();
        Expect(runtime.CommandCount == 2, "shared successful release makes next edge send exactly one retract");
        Expect(Convert.ToInt32(runtime.LastCommand["pwm_microseconds"]) == 1000,
            "retract remains available without release confirmation");
    }

    private static void Joystick_AuthorizedPumpFiresOnce()
    {
        using var runtime = new MockRuntime(2);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        config.Payloads.Add(new PayloadControl { Kind = PayloadKind.Relay, Channel = 3, PulseMs = 50 });
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        SwitchEdge(service, 5);
        Expect(runtime.CommandCount == 0, "one pump edge sends no ON mutation");
        SwitchEdge(service, 5);
        runtime.Wait();
        Expect(runtime.CommandCount == 2, "authorized pump sends one ON and its one safe OFF, without retry");
        Expect(runtime.Commands.FindAll(c => Convert.ToBoolean(c["on"])).Count == 1,
            "authorized pump sequence sends exactly one firing ON mutation");
        Expect(runtime.Commands.FindAll(c => !Convert.ToBoolean(c["on"])).Count == 1,
            "authorized pump sequence sends exactly one safe OFF mutation");
        Expect(!Convert.ToBoolean(runtime.LastCommand["on"]), "pulse finishes with an explicit OFF command");
    }

    private static PayloadReleaseAuthorization DropAuthorization(long? nowMs = null)
    {
        var interlock = new PayloadReleaseInterlock(PayloadReleaseInterlock.DropConfirmations,
            PayloadReleaseInterlock.ConfirmationWindowMs);
        long now = nowMs ?? PayloadReleaseInterlock.NowMs;
        interlock.RegisterClick(now);
        interlock.RegisterClick(now);
        return interlock.RegisterClick(now).Authorization;
    }

    private static void Joystick_RetractsSuccessfulUiRelease()
    {
        using var runtime = new MockRuntime(2);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
        using var panel = new PayloadControlPanel(config);
        panel.TestDropClick();
        panel.TestDropClick();
        panel.TestDropClick();
        WaitForCommandedRelease(true);
        var service = new NomadJoystickService(config);
        SwitchEdge(service, 0);
        WaitForCommandedRelease(false);
        runtime.Wait();
        Expect(runtime.CommandCount == 2, "successful UI release and joystick retract share one commanded state");
        Expect(Convert.ToInt32(runtime.Commands[0]["pwm_microseconds"]) == 2000 &&
            Convert.ToInt32(runtime.Commands[1]["pwm_microseconds"]) == 1000,
            "joystick responds to UI release with safe retract rather than another drop");
    }

    private static void PayloadBoundary_RejectsMissingReusedAndExpiredAuthorization()
    {
        using var runtime = new MockRuntime(2);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        using var panel = new PayloadControlPanel(config);
        WaitForPanel(panel.TestUnauthorizedDrop());
        var expired = DropAuthorization(PayloadReleaseInterlock.NowMs - 4000);
        var rejected = PayloadActions.Drop(config, 1, expired).GetAwaiter().GetResult();
        Expect(!rejected.Succeeded && runtime.CommandCount == 0,
            "UI and headless boundary cannot send without fresh authorization");
        PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
        panel.TestDropClick();
        panel.TestDropClick();
        Expect(runtime.CommandCount == 0, "actual UI input cannot bypass the shared boundary with one or two clicks");
        panel.TestDropClick();
        WaitForCommandedRelease(true);
        Expect(runtime.CommandCount == 1, "actual UI authorized sequence sends exactly one output");
        PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
        var token = DropAuthorization();
        var accepted = PayloadActions.Drop(config, 1, token).GetAwaiter().GetResult();
        var reused = PayloadActions.Drop(config, 1, token).GetAwaiter().GetResult();
        runtime.Wait();
        Expect(accepted.Succeeded && !reused.Succeeded && runtime.CommandCount == 2,
            "authorization is consumed for one output only");
        PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
    }

    private static void Joystick_UncertainDropRequiresExplicitSafeRecovery()
    {
        using var runtime = new MockRuntime(1, dropCommandResponse: true);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        SwitchEdge(service, 0);
        SwitchEdge(service, 0);
        SwitchEdge(service, 0);
        runtime.Wait();
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (!PayloadActions.RequiresSafeRecovery(8) && DateTime.UtcNow < deadline)
        {
            Thread.Sleep(1);
        }
        Expect(PayloadActions.RequiresSafeRecovery(8), "unknown release latches safe recovery across UI and joystick");
        var blocked = PayloadActions.Drop(config, 1, DropAuthorization()).GetAwaiter().GetResult();
        Expect(!blocked.Succeeded && runtime.CommandCount == 1,
            "even fresh authorization cannot blindly repeat unknown release");
        Expect(!PayloadControlPanel.IsPayloadReleaseCommanded(0),
            "unknown software release does not claim physical or commanded completion");
        using var safeRuntime = new MockRuntime(1);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = safeRuntime.Port });
        SwitchEdge(service, 0);
        safeRuntime.Wait();
        deadline = DateTime.UtcNow.AddSeconds(5);
        while (PayloadActions.RequiresSafeRecovery(8) && DateTime.UtcNow < deadline)
        {
            Thread.Sleep(1);
        }
        Expect(!PayloadActions.RequiresSafeRecovery(8),
            "next explicit edge sends safe retract and clears latch only on software success");
        Expect(Convert.ToInt32(safeRuntime.LastCommand["pwm_microseconds"]) == 1000,
            "recovery commands retract rather than retry release");
    }

    private static void WaitForCommandedRelease(bool commanded)
    {
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (PayloadControlPanel.IsPayloadReleaseCommanded(0) != commanded && DateTime.UtcNow < deadline)
        {
            Thread.Sleep(1);
        }
        Expect(PayloadControlPanel.IsPayloadReleaseCommanded(0) == commanded,
            "shared commanded release reaches expected software state");
    }

    private static void Joystick_LostInputSendsOneSafeReelStop(bool contradictory)
    {
        using var runtime = new AsyncRuntime(2);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port, JoystickSw1UpAction = "ReelInP1" };
        config.Payloads.Add(new PayloadControl { Kind = PayloadKind.Reel, Channel = 5, PwmNeutral = 1500 });
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        SwitchEdge(service, 0);
        runtime.WaitForCommands(1);
        var invalid = contradictory ? Buttons(0) : null;
        if (invalid != null)
        {
            invalid[1] = true;
        }
        service.TestButtons(invalid);
        Expect(runtime.Commands.Count == 1, "one safe reel stop waits behind the in-flight movement request");
        runtime.ReleaseResponse(5);
        runtime.WaitForCommands(2);
        service.TestButtons(null);
        service.TestReset();
        runtime.Wait();
        Expect(runtime.Commands.Count == 2, "repeated invalid input/reset sends one safe stop without retries");
        Expect(Convert.ToInt32(runtime.Commands[0]["pwm_microseconds"]) == 2000,
            "known initial command starts the reel");
        Expect(Convert.ToInt32(runtime.Commands[1]["pwm_microseconds"]) == 1500,
            "lost or contradictory input restores neutral PWM through the existing explicit-stop path");
    }

    private static void PayloadBoundary_ReservesOutputUntilUncertaintyRecorded()
    {
        using var runtime = new AsyncRuntime(1);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        config.Payloads[0].Channel = 3;
        OutputController.Initialize(config);
        var pending = PayloadActions.Drop(config, 1, DropAuthorization());
        runtime.WaitForCommands(1);
        var overlap = PayloadActions.Drop(config, 1, DropAuthorization()).GetAwaiter().GetResult();
        Expect(!overlap.Succeeded && runtime.Commands.Count == 1,
            "action reservation rejects concurrent authorization while output result is pending");
        runtime.ReleaseResponse(3);
        var interrupted = pending.GetAwaiter().GetResult();
        runtime.Wait();
        var blocked = PayloadActions.Drop(config, 1, DropAuthorization()).GetAwaiter().GetResult();
        Expect(interrupted.Outcome == NOMAD.MissionPlanner.Connectivity.NomadCoreRequestOutcome.Interrupted &&
            PayloadActions.RequiresSafeRecovery(3) && !blocked.Succeeded,
            "interrupted output atomically replaces pending reservation with safe-recovery latch");
        using var safeRuntime = new MockRuntime(1);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = safeRuntime.Port });
        var recovery = PayloadActions.Retract(config, 1).GetAwaiter().GetResult();
        safeRuntime.Wait();
        Expect(recovery.Succeeded && !PayloadActions.RequiresSafeRecovery(3),
            "explicit successful safe recovery clears interrupted-output latch");
    }
}
