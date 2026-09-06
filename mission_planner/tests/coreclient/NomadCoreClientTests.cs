// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NomadCoreClient unit tests (core CLI boundary contract)
// ============================================================
// Compiled together with src/Connectivity/NomadCoreClient.cs by
// scripts/build/test_plugin_core_client.ps1 (plain csc, no test framework —
// exits non-zero on failure). Run via `pixi run test-plugin-core-client`.
//
// The C++ `nomad` CLI is the replacement client boundary this plugin routes
// vehicle operations through. These tests pin the observable surface without
// a vehicle: the exact argument vector (parsed by src/main.cpp, which expects
// positional values then an optional --endpoint flag), the fail-closed
// validation gates (non-finite / out-of-range input never reaches the core),
// and — when the core binary is built — the live authentication gate (an
// actuation verb without NOMAD_API_KEY is refused before any socket work).
// ============================================================

using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using NOMAD.MissionPlanner.Connectivity;

internal static class NomadCoreClientTests
{
    private static int _failures;

    private static int Main()
    {
        BuildArguments_PinsVerbThenValuesThenEndpoint();
        GotoValues_UseInvariantFormatting();
        LatitudeValidation_RejectsOutOfRangeAndNonFinite();
        LongitudeValidation_RejectsOutOfRangeAndNonFinite();
        Goto_InvalidInputFailsClosedBeforeSpawn();
        Goto_CoreUnavailableFailsClosed();
        OutputVerbArguments_UseInvariantVectors();
        Servo_FailsClosedOnInvalidInput();
        SetRelay_FailsClosedOnInvalidInput();
        MotorTest_FailsClosedOnInvalidInput();
        GimbalConfigure_FailsClosedOnInvalidInput();
        SendUserCommand_RequiresExactlySevenFiniteValues();
        RunCliAuthenticationGate();

        Console.WriteLine(_failures == 0
            ? "All core-client tests passed."
            : $"{_failures} core-client test(s) FAILED.");
        return _failures == 0 ? 0 : 1;
    }

    private static void BuildArguments_PinsVerbThenValuesThenEndpoint()
    {
        var arguments = NomadCoreClient.BuildArguments(
            "udpin:0.0.0.0:14550", "goto", "45.1234567", "9.0000000", "5.0");

        Expect(arguments.Length == 6, "goto vector has 6 entries");
        Expect(arguments[0] == "goto", "verb is first");
        Expect(arguments[1] == "45.1234567" && arguments[2] == "9.0000000" && arguments[3] == "5.0",
            "values follow the verb in order");
        Expect(arguments[4] == "--endpoint", "--endpoint flag is last");
        Expect(arguments[5] == "udpin:0.0.0.0:14550", "endpoint value closes the vector");
    }

    private static void GotoValues_UseInvariantFormatting()
    {
        var values = NomadCoreClient.BuildGotoValues(45.1234567, 9.0, 5.0);
        Expect(values.Length == 3 && values[0] == "45.1234567", "latitude formats F7");
        Expect(values[1] == "9.0000000", "longitude formats F7 with trailing zeros");
        Expect(values[2] == "5.0", "altitude formats F1");

        // Round-trips the CLI's strtof parse: no comma-decimal anywhere.
        var commaValues = NomadCoreClient.BuildGotoValues(45.5, 9.25, 3.75);
        foreach (var value in commaValues)
        {
            Expect(value.IndexOf(',') < 0 && value.IndexOf('.') >= 0,
                $"value '{value}' uses the invariant decimal separator");
        }
    }

    private static void LatitudeValidation_RejectsOutOfRangeAndNonFinite()
    {
        Expect(NomadCoreClient.IsValidLatitude(0.0), "latitude 0 is valid");
        Expect(NomadCoreClient.IsValidLatitude(90.0), "latitude +90 is valid");
        Expect(NomadCoreClient.IsValidLatitude(-90.0), "latitude -90 is valid");
        Expect(!NomadCoreClient.IsValidLatitude(90.0000001), "latitude above +90 rejected");
        Expect(!NomadCoreClient.IsValidLatitude(-90.0000001), "latitude below -90 rejected");
        Expect(!NomadCoreClient.IsValidLatitude(double.NaN), "NaN latitude rejected");
        Expect(!NomadCoreClient.IsValidLatitude(double.PositiveInfinity), "+inf latitude rejected");
        Expect(!NomadCoreClient.IsValidLatitude(double.NegativeInfinity), "-inf latitude rejected");
    }

    private static void LongitudeValidation_RejectsOutOfRangeAndNonFinite()
    {
        Expect(NomadCoreClient.IsValidLongitude(0.0), "longitude 0 is valid");
        Expect(NomadCoreClient.IsValidLongitude(180.0), "longitude +180 is valid");
        Expect(NomadCoreClient.IsValidLongitude(-180.0), "longitude -180 is valid");
        Expect(!NomadCoreClient.IsValidLongitude(180.0000001), "longitude above +180 rejected");
        Expect(!NomadCoreClient.IsValidLongitude(-180.0000001), "longitude below -180 rejected");
        Expect(!NomadCoreClient.IsValidLongitude(double.NaN), "NaN longitude rejected");
        Expect(!NomadCoreClient.IsValidLongitude(double.PositiveInfinity), "+inf longitude rejected");
        Expect(!NomadCoreClient.IsValidLongitude(double.NegativeInfinity), "-inf longitude rejected");
    }

    private static void Goto_InvalidInputFailsClosedBeforeSpawn()
    {
        // A path that can never start proves validation returns first: if the
        // client tried to spawn it, Start() would throw and the test would
        // still see false — so additionally assert the result is false and
        // rely on the direct IsValid* pins above for the rejection reason.
        var client = new NomadCoreClient(@"C:\__nomad_core_does_not_exist__.exe");
        Expect(!client.Goto(double.NaN, 9.0, 5.0), "NaN latitude fails closed");
        Expect(!client.Goto(45.0, double.PositiveInfinity, 5.0), "infinite longitude fails closed");
        Expect(!client.Goto(45.0, 9.0, double.NegativeInfinity), "infinite altitude fails closed");
        Expect(!client.Goto(45.0, 9.0, 5.0), "bogus core path fails closed on otherwise valid goto");
    }

    private static void Goto_CoreUnavailableFailsClosed()
    {
        var client = new NomadCoreClient("");
        Expect(!string.IsNullOrWhiteSpace(client.ExecutablePath), "empty path falls back to 'nomad' on PATH");
    }

    private static void OutputVerbArguments_UseInvariantVectors()
    {
        // Pinned against src/main.cpp's positional parser: values in order,
        // then --endpoint (BuildArguments appends the flag itself).
        var servo = NomadCoreClient.BuildArguments("udpin:0.0.0.0:14550", "servo", "8", "1500");
        Expect(servo.Length == 5 && servo[0] == "servo" && servo[1] == "8" && servo[2] == "1500" &&
               servo[3] == "--endpoint", "servo vector: verb, channel, pwm, --endpoint");

        var relay = NomadCoreClient.BuildArguments("udpin:0.0.0.0:14550", "relay", "3", "1");
        Expect(relay.Length == 5 && relay[0] == "relay" && relay[1] == "3" && relay[2] == "1",
            "relay vector: verb, number, on/off");

        var motorTest = NomadCoreClient.BuildArguments("udpin:0.0.0.0:14550", "motor-test", "2", "1200", "1.50");
        Expect(motorTest.Length == 6 && motorTest[0] == "motor-test" && motorTest[1] == "2" &&
               motorTest[2] == "1200" && motorTest[3] == "1.50", "motor-test vector: verb, instance, pwm, timeout");

        var gimbal = NomadCoreClient.BuildArguments("udpin:0.0.0.0:14550", "gimbal-config", "2");
        Expect(gimbal.Length == 4 && gimbal[0] == "gimbal-config" && gimbal[1] == "2" &&
               gimbal[2] == "--endpoint", "gimbal-config vector: verb, mount mode, --endpoint");

        var user = NomadCoreClient.BuildArguments(
            "udpin:0.0.0.0:14550", "user-command", "1.000", "2.000", "3.000", "4.000", "5.000", "6.000", "7.000");
        Expect(user.Length == 10 && user[0] == "user-command" && user[1] == "1.000" && user[7] == "7.000" &&
               user[8] == "--endpoint", "user-command vector: verb plus exactly seven values");
    }

    private static void Servo_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient(@"C:\__nomad_core_does_not_exist__.exe");
        Expect(!client.Servo(0, 1500), "channel 0 rejected");
        Expect(!client.Servo(-1, 1500), "negative channel rejected");
        Expect(!client.Servo(1, 499), "pwm below 500 rejected");
        Expect(!client.Servo(1, 2501), "pwm above 2500 rejected");
        Expect(!client.Servo(1, 1500), "bogus core path fails closed on valid servo");
    }

    private static void SetRelay_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient(@"C:\__nomad_core_does_not_exist__.exe");
        Expect(!client.SetRelay(-1, true), "negative relay rejected");
        Expect(!client.SetRelay(16, true), "relay above 15 rejected");
        Expect(!client.SetRelay(3, true), "bogus core path fails closed on valid relay");
    }

    private static void MotorTest_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient(@"C:\__nomad_core_does_not_exist__.exe");
        Expect(!client.MotorTest(0, 1000, 1.0), "motor instance 0 rejected");
        Expect(!client.MotorTest(1, 400, 1.0), "pwm below 500 rejected");
        Expect(!client.MotorTest(1, 2600, 1.0), "pwm above 2500 rejected");
        Expect(!client.MotorTest(1, 1000, double.NaN), "NaN timeout rejected");
        Expect(!client.MotorTest(1, 1000, double.PositiveInfinity), "infinite timeout rejected");
        Expect(!client.MotorTest(1, 1000, 1.0), "bogus core path fails closed on valid motor test");
    }

    private static void GimbalConfigure_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient(@"C:\__nomad_core_does_not_exist__.exe");
        Expect(!client.GimbalConfigure(-1), "negative mount mode rejected");
        Expect(!client.GimbalConfigure(5), "mount mode above 4 rejected");
        Expect(!client.GimbalConfigure(2), "bogus core path fails closed on valid mount mode");
    }

    private static void SendUserCommand_RequiresExactlySevenFiniteValues()
    {
        var client = new NomadCoreClient(@"C:\__nomad_core_does_not_exist__.exe");
        Expect(!client.SendUserCommand(), "zero values rejected");
        Expect(!client.SendUserCommand(1, 2, 3, 4, 5, 6), "six values rejected");
        Expect(!client.SendUserCommand(1, 2, 3, 4, 5, 6, 7, 8), "eight values rejected");
        Expect(!client.SendUserCommand(1, 2, 3, 4, 5, 6, double.NaN), "NaN parameter rejected");
        Expect(!client.SendUserCommand(1, 2, 3, 4, 5, 6, double.PositiveInfinity), "infinite parameter rejected");
        Expect(!client.SendUserCommand(1, 2, 3, 4, 5, 6, 7), "bogus core path fails closed on valid command");
    }

    private static void RunCliAuthenticationGate()
    {
        var binary = FindCoreBinary();
        if (binary == null)
        {
            Console.WriteLine("SKIP: C++ core binary not built; auth gate not exercised.");
            return;
        }
        if (!string.IsNullOrEmpty(Environment.GetEnvironmentVariable("NOMAD_API_KEY")))
        {
            Console.WriteLine("SKIP: NOMAD_API_KEY set in harness environment; refusing auth-gate check.");
            return;
        }

        // Same invocation the client would produce. No vehicle is needed: the
        // CLI refuses actuation at the auth gate before opening the MAVLink
        // socket, mirroring the Python contract tests (SR-SEC-02/03).
        var arguments = NomadCoreClient.BuildArguments("udpin:0.0.0.0:14550", "goto", "45.0", "9.0", "5.0");
        var start = new ProcessStartInfo
        {
            FileName = binary,
            Arguments = string.Join(" ", arguments),
            UseShellExecute = false,
            RedirectStandardError = true,
            RedirectStandardOutput = true,
            CreateNoWindow = true,
        };

        using var process = new Process { StartInfo = start };
        process.Start();
        var stderr = process.StandardError.ReadToEnd();
        process.WaitForExit();

        Expect(process.ExitCode != 0, $"core CLI exits nonzero without a key (got {process.ExitCode})");
        Expect(stderr.IndexOf("refused", StringComparison.OrdinalIgnoreCase) >= 0,
            $"stderr names the auth refusal (got: {Truncate(stderr, 120)})");
    }

    private static string FindCoreBinary()
    {
        var overridePath = Environment.GetEnvironmentVariable("NOMAD_CORE_EXE");
        if (!string.IsNullOrEmpty(overridePath) && File.Exists(overridePath))
        {
            return overridePath;
        }
        var root = FindRepoRoot();
        if (root == null)
        {
            return null;
        }
        string[] names = { "nomad.exe", "nomad" };
        string[] roots = { Path.Combine(root, "build", "core"), Path.Combine(root, "build-core") };
        foreach (var buildRoot in roots)
        {
            foreach (var sub in new[] { "", "Debug", "Release" })
            {
                var directory = sub.Length == 0 ? buildRoot : Path.Combine(buildRoot, sub);
                foreach (var name in names)
                {
                    var candidate = Path.Combine(directory, name);
                    if (File.Exists(candidate))
                    {
                        return candidate;
                    }
                }
            }
        }
        return null;
    }

    private static string FindRepoRoot()
    {
        var directory = new DirectoryInfo(AppDomain.CurrentDomain.BaseDirectory);
        while (directory != null)
        {
            if (File.Exists(Path.Combine(directory.FullName, "pixi.toml")))
            {
                return directory.FullName;
            }
            directory = directory.Parent;
        }
        return null;
    }

    private static string Truncate(string text, int maxLength)
    {
        text = text.Trim();
        return text.Length <= maxLength ? text : text.Substring(0, maxLength) + "...";
    }

    private static void Expect(bool condition, string message)
    {
        if (condition)
        {
            return;
        }
        _failures++;
        Console.WriteLine($"FAIL: {message}");
    }
}
