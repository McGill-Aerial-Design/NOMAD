// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using NOMAD.MissionPlanner;

internal static class AdvisoryPositionTests
{
    private static int _failures;
    private static readonly MAVLink.MAVLINK_MSG_ID[] Sources =
    {
        MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT, MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY,
        MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY2, MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT
    };

    internal static int Run()
    {
        RunCase("all supported real packet formats and coordinate boundaries", FormatsAndCoordinates);
        RunCase("known horizontal GPS fixes and complete coordinate pairs", RawGpsValidity);
        RunCase("estimate preference and no unusable fallback", SourceSelection);
        RunCase("unorderable estimate receipts and explicit timestamp ties", AmbiguousReceipts);
        RunCase("receipt boundary and cached reread expiry", ReceiptExpiry);
        RunCase("sticky rejected receipts and stationary new receipts", RejectedReceipts);
        RunCase("UTC jumps and switching source slots", ClockAndSelectionChanges);
        RunCase("stale position while unrelated host traffic continues", StaleMonitor);
        RunCase("snapshot expiry and one fresh receipt recovery", SnapshotExpiryRecovery);
        RunCase("received outside point overrides cached inside coordinates", ReceivedPointIntegration);
        RunCase("sampling failure clears state before logging", SamplingFailure);
        RunCase("context replacements and retained caches", ContextReplacement);
        RunCase("closed/reopened and log replay contexts", ClosedAndReplayContexts);
        RunCase("context rechecked before publication", ChangedDuringRead);
        RunCase("sampling is nonreentrant and queue remains unconsumed", ReentrancyAndQueue);
        RunCase("stop/dispose fence pending host reads", PendingReadFences);
        RunCase("stopped subscriber cannot publish stale inside event", SubscriberStop);
        RunCase("subscriber context change fences later events and audio", SubscriberContextChange);
        RunCase("subscriber delay expires publication and current snapshot", SubscriberExpiry);
        return _failures;
    }

    private static void RunCase(string name, Action test)
    {
        int failuresBefore = _failures;
        try { test(); }
        catch (Exception ex) { Check(false, name + ": " + ex); }
        if (_failures == failuresBefore) { Console.WriteLine("PASS received-position: " + name); }
    }

    private static void Check(bool condition, string message)
    {
        if (!condition) { _failures++; Console.WriteLine("FAIL received-position: " + message); }
    }

    private sealed class TestClock
    {
        internal long Milliseconds;
        internal DateTime Utc = new DateTime(2026, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        internal RouterClock Clock => new RouterClock(() => Milliseconds / 1000.0, () => Utc);
        internal void Advance(int milliseconds)
        {
            Milliseconds += milliseconds;
            Utc = Utc.AddMilliseconds(milliseconds);
        }
    }

    private static MAVLink.MAVLinkMessage Packet(MAVLink.MAVLINK_MSG_ID source, DateTime received,
        int latitude = 15000000, int longitude = 15000000, byte fix = 3, byte systemId = 1, byte componentId = 1)
    {
        object payload;
        switch (source)
        {
            case MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT:
                payload = new MAVLink.mavlink_global_position_int_t { lat = latitude, lon = longitude };
                break;
            case MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY:
#pragma warning disable CS0612 // Retain the supported host's legacy high-latency position format.
                payload = new MAVLink.mavlink_high_latency_t { latitude = latitude, longitude = longitude };
#pragma warning restore CS0612
                break;
            case MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY2:
                payload = new MAVLink.mavlink_high_latency2_t
                    { latitude = latitude, longitude = longitude, failure_flags = ushort.MaxValue };
                break;
            default:
                payload = new MAVLink.mavlink_gps_raw_int_t { lat = latitude, lon = longitude, fix_type = fix };
                break;
        }
        var bytes = new MAVLink.MavlinkParse().GenerateMAVLinkPacket20(
            source, payload, false, systemId, componentId, 1);
        return new MAVLink.MAVLinkMessage(bytes, received);
    }

    private static MAVLink.MAVLinkMessage[] Slots(MAVLink.MAVLinkMessage packet, int index)
    {
        var slots = new MAVLink.MAVLinkMessage[4];
        slots[index] = packet;
        return slots;
    }

    private static void FormatsAndCoordinates()
    {
        var coordinates = new[] { (0, 0), (0, 15000000), (15000000, 0),
            (-900000000, -1800000000), (900000000, 1800000000) };
        for (int index = 0; index < Sources.Length; index++)
        {
            foreach (var pair in coordinates)
            {
                var clock = new TestClock();
                var packet = Packet(Sources[index], clock.Utc, pair.Item1, pair.Item2);
                var point = new AdvisoryPosition(clock.Clock).Read(Slots(packet, index), 1, 1);
                Check(point != null && point.Lat == pair.Item1 / 1.0e7 && point.Lon == pair.Item2 / 1.0e7,
                    Sources[index] + " preserves the complete received coordinate pair, including legal zeros/bounds");
            }
            foreach (var pair in new[] { (900000001, 0), (0, 1800000001), (int.MaxValue, 1), (1, int.MaxValue) })
            {
                var clock = new TestClock();
                var point = new AdvisoryPosition(clock.Clock).Read(
                    Slots(Packet(Sources[index], clock.Utc, pair.Item1, pair.Item2), index), 1, 1);
                Check(point == null, Sources[index] + " rejects an invalid axis without keeping the other axis");
            }
        }
    }

    private static void RawGpsValidity()
    {
        foreach (byte fix in new byte[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 255 })
        {
            var clock = new TestClock();
            var point = new AdvisoryPosition(clock.Clock).Read(
                Slots(Packet(Sources[3], clock.Utc, fix: fix), 3), 1, 1);
            Check((point != null) == (fix >= 2 && fix <= 8), "GPS horizontal fix " + fix + " eligibility");
        }
        var identityClock = new TestClock();
        Check(new AdvisoryPosition(identityClock.Clock).Read(
            Slots(Packet(Sources[0], identityClock.Utc, systemId: 2), 0), 1, 1) == null,
            "a packet for another observed system is not used");
        Check(new AdvisoryPosition(identityClock.Clock).Read(
            Slots(Packet(Sources[0], identityClock.Utc, componentId: 2), 0), 1, 1) == null,
            "a packet for another observed component is not used");
    }

    private static void SourceSelection()
    {
        var clock = new TestClock();
        var tracker = new AdvisoryPosition(clock.Clock);
        var slots = Slots(Packet(Sources[0], clock.Utc), 0);
        clock.Advance(10);
        slots[3] = Packet(Sources[3], clock.Utc, 17000000, 18000000);
        Check(tracker.Read(slots, 1, 1)?.Lat == 1.5, "new raw GPS cannot overrule an estimate");
        slots[1] = Packet(Sources[1], clock.Utc, 16000000, 17000000);
        Check(tracker.Read(slots, 1, 1)?.Lat == 1.6, "newest high-latency estimate is selected");
        clock.Advance(1);
        slots[2] = Packet(Sources[2], clock.Utc, int.MaxValue, 1);
        Check(tracker.Read(slots, 1, 1) == null,
            "invalid newest estimate suppresses older estimates and raw GPS");
        clock.Advance(4000);
        slots[3] = Packet(Sources[3], clock.Utc);
        Check(tracker.Read(slots, 1, 1) == null, "expired estimate presence does not fall back to fresh raw GPS");
    }

    private static void AmbiguousReceipts()
    {
        foreach (var timestamp in new[] { DateTime.MinValue,
            new DateTime(2026, 1, 1, 0, 0, 0, DateTimeKind.Unspecified) })
        {
            var clock = new TestClock();
            var tracker = new AdvisoryPosition(clock.Clock);
            var slots = Slots(Packet(Sources[0], clock.Utc), 0);
            slots[1] = Packet(Sources[1], timestamp);
            Check(tracker.Read(slots, 1, 1) == null, "unorderable estimate suppresses another valid estimate");
            slots[1] = null;
            Check(tracker.Read(slots, 1, 1) != null,
                "missing estimate is absent, rather than an invalid received packet");
        }
        var tieClock = new TestClock();
        var tieSlots = Slots(Packet(Sources[0], tieClock.Utc), 0);
        tieSlots[2] = Packet(Sources[2], tieClock.Utc, 16000000, 17000000);
        Check(new AdvisoryPosition(tieClock.Clock).Read(tieSlots, 1, 1) == null,
            "equal estimate receipt timestamps are deliberately unknown rather than arbitrary source preference");
        tieSlots[2] = Packet(Sources[2], tieClock.Utc, int.MaxValue, 1);
        Check(new AdvisoryPosition(tieClock.Clock).Read(tieSlots, 1, 1) == null,
            "a maximum-time tie cannot prefer the valid member over the invalid member");
        tieSlots[1] = Packet(Sources[1], tieClock.Utc, 16000000, 17000000);
        tieClock.Advance(1);
        tieSlots[2] = Packet(Sources[2], tieClock.Utc, 17000000, 18000000);
        Check(new AdvisoryPosition(tieClock.Clock).Read(tieSlots, 1, 1)?.Lat == 1.7,
            "tied older estimates do not hide a uniquely newer estimate");
        tieSlots[2] = Packet(Sources[2], tieClock.Utc.AddMilliseconds(100));
        Check(new AdvisoryPosition(tieClock.Clock).Read(tieSlots, 1, 1) == null,
            "a chosen future receipt suppresses another valid estimate");
    }

    private static void ReceiptExpiry()
    {
        var clock = new TestClock();
        var tracker = new AdvisoryPosition(clock.Clock);
        var slots = Slots(Packet(Sources[0], clock.Utc), 0);
        Check(tracker.Read(slots, 1, 1) != null, "a newly received point is available");
        clock.Advance(3500);
        Check(tracker.Read(slots, 1, 1) != null, "the exact 3500ms boundary is included");
        clock.Advance(1);
        Check(tracker.Read(slots, 1, 1) == null, "3501ms expires despite rereading the same packet");
        slots[0] = Packet(Sources[0], clock.Utc);
        Check(tracker.Read(slots, 1, 1) != null, "stationary coordinates with a new receipt can restore display");
        var cached = slots[0];
        slots[0] = null;
        Check(tracker.Read(slots, 1, 1) == null, "a missing packet does not display a previously admitted point");
        clock.Advance(3501);
        clock.Utc = cached.rxtime;
        slots[0] = cached;
        Check(tracker.Read(slots, 1, 1) == null, "a cache gap cannot readmit the same expired reference");
        var firstRead = new AdvisoryPosition(clock.Clock);
        slots[0] = Packet(Sources[0], clock.Utc.AddMilliseconds(-3000));
        Check(firstRead.Read(slots, 1, 1) != null, "first read accounts for the existing receipt age");
        clock.Advance(501);
        Check(firstRead.Read(slots, 1, 1) == null, "first read does not grant a new full budget");
    }

    private static void RejectedReceipts()
    {
        foreach (var age in new[] { -100, 3501 })
        {
            var clock = new TestClock();
            var tracker = new AdvisoryPosition(clock.Clock);
            var slots = Slots(Packet(Sources[0], clock.Utc.AddMilliseconds(-age)), 0);
            Check(tracker.Read(slots, 1, 1) == null, "future/old receipt is rejected at first sight");
            clock.Utc = slots[0].rxtime;
            Check(tracker.Read(slots, 1, 1) == null, "UTC catch-up cannot admit a rejected reference");
        }
        var missingClock = new TestClock();
        var missing = new AdvisoryPosition(missingClock.Clock);
        Check(missing.Read(new MAVLink.MAVLinkMessage[4], 1, 1) == null, "no packet is unknown, not geographic zero");
        Check(missing.Read(Slots(Packet(Sources[0], DateTime.MinValue), 0), 1, 1) == null,
            "default receipt timestamp is unknown");
    }

    private static void ClockAndSelectionChanges()
    {
        var clock = new TestClock();
        var tracker = new AdvisoryPosition(clock.Clock);
        var slots = Slots(Packet(Sources[0], clock.Utc), 0);
        Check(tracker.Read(slots, 1, 1) != null, "initial estimate admitted");
        clock.Utc = clock.Utc.AddDays(1);
        Check(tracker.Read(slots, 1, 1) != null, "forward UTC jump does not shorten accepted monotonic lifetime");
        clock.Advance(3501);
        clock.Utc = clock.Utc.AddDays(-2);
        Check(tracker.Read(slots, 1, 1) == null, "backward UTC jump cannot revive the old receipt");
        slots[1] = Packet(Sources[1], clock.Utc, 16000000, 16000000);
        Check(tracker.Read(slots, 1, 1)?.Lat == 1.6,
            "new receipt after UTC reversal is ordered by its frozen monotonic receipt");
        var switchClock = new TestClock();
        var switchTracker = new AdvisoryPosition(switchClock.Clock);
        var switchSlots = Slots(Packet(Sources[0], switchClock.Utc), 0);
        Check(switchTracker.Read(switchSlots, 1, 1)?.Lat == 1.5,
            "source-switch control establishes global point");
        switchClock.Advance(1000);
        switchSlots[1] = Packet(Sources[1], switchClock.Utc, 16000000, 16000000);
        Check(switchTracker.Read(switchSlots, 1, 1)?.Lat == 1.6,
            "source-switch control establishes distinct HL selection");
        switchClock.Milliseconds += 2501;
        Check(switchTracker.Read(switchSlots, 1, 1)?.Lat == 1.6,
            "HL remains admitted while the old global deadline expires");
        switchSlots[1] = null;
        Check(switchTracker.Read(switchSlots, 1, 1) == null,
            "switching back cannot readmit the expired global reference");
    }

    private sealed class Fixture : IDisposable
    {
        internal readonly TestClock Clock = new TestClock();
        internal readonly MAVLinkInterface Port = new MAVLinkInterface();
        internal readonly AdvisoryBoundaryMonitor Monitor;
        internal readonly List<AdvisoryOutlineStatus> Events = new List<AdvisoryOutlineStatus>();

        internal Fixture()
        {
            MainV2.comPort = Port;
            AudioAlerts.Calls = 0;
            var config = new GeofenceConfig();
            config.HardBoundary.Vertices = new List<GpsPoint>
                { new GpsPoint(1, 1), new GpsPoint(1, 2), new GpsPoint(2, 2), new GpsPoint(2, 1) };
            Monitor = new AdvisoryBoundaryMonitor(config, Clock.Clock);
            Monitor.StatusChanged += (s, e) => Events.Add(e.Status);
            Monitor.StartMonitoring(60000);
            Monitor.SamplePosition();
        }

        internal void Receive(int latitude = 15000000, int longitude = 15000000)
        {
            MainV2.comPort.MAV.Receive(Packet(Sources[0], Clock.Utc, latitude, longitude,
                systemId: MainV2.comPort.MAV.sysid, componentId: MainV2.comPort.MAV.compid));
        }

        internal void Inside()
        {
            Receive();
            Monitor.SamplePosition();
            Check(Monitor.CurrentStatus == AdvisoryOutlineStatus.InsideOutlines,
                "fixture establishes observed inside state");
        }

        public void Dispose()
        {
            Monitor.Dispose();
            Check(Port.Writes == 0, "position observation sends no vehicle command");
            MainV2.comPort = null;
            Log.OnError = null;
        }
    }

    private static void StaleMonitor()
    {
        using (var fixture = new Fixture())
        {
            fixture.Inside();
            fixture.Clock.Advance(3501);
            fixture.Port.MAV.lastvalidpacket = fixture.Clock.Utc;
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition,
                "fresh unrelated traffic does not renew the last position");
            Check(fixture.Monitor.CurrentObservation.Position == null, "unknown state has no displayed point");
            Check(fixture.Events.FindAll(s => s == AdvisoryOutlineStatus.InsideOutlines).Count == 1,
                "expiry creates no new inside notification");
            fixture.Receive();
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.InsideOutlines,
                "one fresh receipt restores display after snapshot expiry in an unchanged context");
        }
    }

    private static void ReceivedPointIntegration()
    {
        using (var fixture = new Fixture())
        {
            fixture.Port.MAV.ReadState = () =>
                throw new InvalidOperationException("cached cs must not supply position");
            fixture.Receive(25000000, 30000000);
            fixture.Monitor.SamplePosition();
            var observation = fixture.Monitor.CurrentObservation;
            Check(observation.Status == AdvisoryOutlineStatus.OutsideOuterOutline &&
                observation.Position?.Lat == 2.5 && observation.Position?.Lon == 3.0,
                "one received outside packet drives both classification and coordinates, while cached cs stays inside");
        }
    }

    private static void SnapshotExpiryRecovery()
    {
        using (var fixture = new Fixture())
        {
            fixture.Inside();
            fixture.Clock.Advance(3501);
            int reads = fixture.Port.MAV.Reads;
            var expired = fixture.Monitor.CurrentObservation;
            Check(expired.Status == AdvisoryOutlineStatus.NoPosition && expired.Position == null &&
                fixture.Port.MAV.Reads == reads, "snapshot expiry neither fetches nor renews a host packet");
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition,
                "polling the expired reference cannot restore display");
            fixture.Receive();
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.InsideOutlines,
                "one post-expiry receipt restores the unchanged live context");
        }
    }

    private static void SamplingFailure()
    {
        using (var fixture = new Fixture())
        {
            fixture.Inside();
            bool clearedBeforeLog = false;
            Log.OnError = () => clearedBeforeLog = fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition;
            fixture.Port.MAV.BeforeRead = () => throw new InvalidOperationException("controlled host read fault");
            fixture.Monitor.SamplePosition();
            fixture.Port.MAV.BeforeRead = null;
            Check(clearedBeforeLog && fixture.Monitor.CurrentObservation.Position == null,
                "sampling fault publishes unknown before logging and removes displayed coordinates");
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition,
                "old cache stays blocked after fault");
        }
    }

    private static void ContextReplacement()
    {
        using (var fixture = new Fixture())
        {
            fixture.Inside();
            var cached = fixture.Port.MAV.Queued.Peek();
            fixture.Port.MAV = new MAVState();
            fixture.Port.MAV.Receive(cached);
            RequireNewReceipt(fixture, "new MAV with the same IDs");
            fixture.Port.BaseStream = new TestStream();
            RequireNewReceipt(fixture, "new stream with the same MAV");
            MainV2.comPort = new MAVLinkInterface { MAV = fixture.Port.MAV, BaseStream = fixture.Port.BaseStream };
            RequireNewReceipt(fixture, "new interface with the same MAV/stream");
            MainV2.comPort.MAV.compid = 2;
            RequireNewReceipt(fixture, "changed observed identity");
        }
    }

    private static void RequireNewReceipt(Fixture fixture, string reason)
    {
        fixture.Monitor.SamplePosition();
        fixture.Monitor.SamplePosition();
        Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition,
            reason + " blocks retained packets");
        fixture.Inside();
    }

    private static void ClosedAndReplayContexts()
    {
        using (var fixture = new Fixture())
        {
            fixture.Inside();
            fixture.Port.BaseStream.IsOpen = false;
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition,
                "observed closed stream is unknown");
            fixture.Port.BaseStream.IsOpen = true;
            RequireNewReceipt(fixture, "observed close/reopen");
            fixture.Port.logreadmode = true;
            fixture.Receive();
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition,
                "log playback is not live receipt");
            fixture.Port.logreadmode = false;
            RequireNewReceipt(fixture, "return from log playback");
        }
    }

    private static void ChangedDuringRead()
    {
        using (var fixture = new Fixture())
        {
            fixture.Inside();
            fixture.Port.MAV.BeforeRead = () => MainV2.comPort = new MAVLinkInterface();
            fixture.Monitor.SamplePosition();
            fixture.Port.MAV.BeforeRead = null;
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition,
                "context replacement during sampling is rechecked before publication");
        }
    }

    private static void ReentrancyAndQueue()
    {
        using (var fixture = new Fixture())
        {
            fixture.Receive();
            var queued = fixture.Port.MAV.Queued.Peek();
            int before = fixture.Port.MAV.Reads;
            fixture.Port.MAV.BeforeRead = fixture.Monitor.SamplePosition;
            fixture.Monitor.SamplePosition();
            fixture.Port.MAV.BeforeRead = null;
            Check(fixture.Port.MAV.Reads - before == 4, "reentrant sampling is rejected rather than interleaved");
            Check(fixture.Port.MAV.Queued.Count == 1 && ReferenceEquals(fixture.Port.MAV.Queued.Peek(), queued),
                "sampling does not consume the host's queued packet");
            var copy = fixture.Monitor.CurrentObservation;
            copy.Position.Lat = 99;
            Check(fixture.Monitor.CurrentObservation.Position.Lat == 1.5, "callers cannot mutate the monitor's point");
        }
    }

    private static void PendingReadFences()
    {
        foreach (bool dispose in new[] { false, true })
        {
            using (var fixture = new Fixture())
            using (var entered = new ManualResetEventSlim())
            using (var release = new ManualResetEventSlim())
            {
                fixture.Inside();
                fixture.Port.MAV.BeforeRead = () =>
                {
                    entered.Set();
                    if (!release.Wait(TimeSpan.FromSeconds(2))) { throw new TimeoutException("host-read release"); }
                };
                var pending = Task.Run(fixture.Monitor.SamplePosition);
                Check(entered.Wait(TimeSpan.FromSeconds(2)), "pending sample reaches controlled host read");
                var stop = Task.Run(() =>
                {
                    if (dispose) { fixture.Monitor.Dispose(); }
                    else { fixture.Monitor.StopMonitoring(); }
                });
                Check(stop.Wait(TimeSpan.FromSeconds(1)), "stop/dispose does not wait for a blocked host read");
                release.Set();
                Check(pending.Wait(TimeSpan.FromSeconds(2)), "pending sample exits after release");
                Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition &&
                    fixture.Events.FindAll(s => s == AdvisoryOutlineStatus.InsideOutlines).Count == 1,
                    "pending callback cannot restore inside state or publish another inside event");
                fixture.Port.MAV.BeforeRead = null;
            }
        }
    }

    private static void SubscriberStop()
    {
        using (var fixture = new Fixture())
        {
            fixture.Monitor.StatusChanged += (s, e) =>
            {
                if (e.Status == AdvisoryOutlineStatus.InsideOutlines) { fixture.Monitor.StopMonitoring(); }
            };
            int laterInside = 0;
            fixture.Monitor.StatusChanged += (s, e) =>
            {
                if (e.Status == AdvisoryOutlineStatus.InsideOutlines) { laterInside++; }
            };
            fixture.Receive();
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition && laterInside == 0,
                "a subscriber stopping the monitor fences later stale-inside subscribers");
        }
    }

    private static void SubscriberContextChange()
    {
        using (var fixture = new Fixture())
        {
            fixture.Monitor.StatusChanged += (s, e) =>
            {
                if (e.Status == AdvisoryOutlineStatus.OutsideOuterOutline)
                {
                    fixture.Port.BaseStream = new TestStream();
                }
            };
            int laterOutside = 0;
            fixture.Monitor.StatusChanged += (s, e) =>
            {
                if (e.Status == AdvisoryOutlineStatus.OutsideOuterOutline) { laterOutside++; }
            };
            fixture.Receive(25000000, 30000000);
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.NoPosition && laterOutside == 0 &&
                AudioAlerts.Calls == 0, "subscriber context replacement invalidates state before later events/audio");
        }
    }

    private static void SubscriberExpiry()
    {
        using (var fixture = new Fixture())
        {
            fixture.Monitor.StatusChanged += (s, e) =>
            {
                if (e.Status == AdvisoryOutlineStatus.OutsideOuterOutline) { fixture.Clock.Advance(3501); }
            };
            int laterOutside = 0;
            fixture.Monitor.StatusChanged += (s, e) =>
            {
                if (e.Status == AdvisoryOutlineStatus.OutsideOuterOutline) { laterOutside++; }
            };
            fixture.Receive(25000000, 30000000);
            fixture.Monitor.SamplePosition();
            var observation = fixture.Monitor.CurrentObservation;
            Check(observation.Status == AdvisoryOutlineStatus.NoPosition && observation.Position == null &&
                laterOutside == 0 && AudioAlerts.Calls == 0,
                "expired chosen receipt cannot reach later subscribers, audio or current snapshot");
            fixture.Receive();
            fixture.Monitor.SamplePosition();
            Check(fixture.Monitor.CurrentStatus == AdvisoryOutlineStatus.InsideOutlines,
                "expiry retains live context so one fresh packet restores observation");
        }
    }
}
