// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// Host spies only: the production monitor and real MAVLink packet decoder are compiled unchanged.
using System;
using System.Collections.Generic;

namespace MissionPlanner
{
    internal static class MainV2
    {
        internal static MAVLinkInterface comPort;
    }

    internal sealed class TestStream
    {
        internal bool IsOpen = true;
    }

    internal sealed class MAVLinkInterface
    {
        internal MAVState MAV = new MAVState();
        internal TestStream BaseStream = new TestStream();
        internal bool logreadmode;
        internal int Writes;
        internal void sendPacket(object packet, byte systemId, byte componentId) { Writes++; }
    }

    internal sealed class CurrentState
    {
        internal double lat = 1.5, lng = 1.5;
        internal int gpsstatus = 0;
    }

    internal sealed class MAVState
    {
        internal byte sysid = 1, compid = 1;
        internal DateTime lastvalidpacket;
        private readonly CurrentState _state = new CurrentState();
        internal Action ReadState = null;
        internal CurrentState cs { get { ReadState?.Invoke(); return _state; } }
        internal Action BeforeRead;
        internal int Reads;
        private readonly Dictionary<uint, MAVLink.MAVLinkMessage> _last =
            new Dictionary<uint, MAVLink.MAVLinkMessage>();
        internal readonly Queue<MAVLink.MAVLinkMessage> Queued = new Queue<MAVLink.MAVLinkMessage>();

        internal MAVLink.MAVLinkMessage getPacketLast(uint messageId)
        {
            Reads++;
            BeforeRead?.Invoke();
            return _last.TryGetValue(messageId, out var packet) ? packet : null;
        }

        internal void Receive(MAVLink.MAVLinkMessage packet)
        {
            _last[packet.msgid] = packet;
            Queued.Enqueue(packet);
        }
    }
}

namespace NOMAD.MissionPlanner
{
    public sealed class GeofenceConfig
    {
        internal FlightBoundary SoftBoundary = new FlightBoundary();
        internal FlightBoundary HardBoundary = new FlightBoundary();
        internal bool AdvisoryAudioAlertsEnabled = true;
    }

    internal static class AudioAlerts
    {
        internal static int Calls;
        internal static void Speak(string text, string component = null) { Calls++; }
    }

    internal static class Log
    {
        internal static Action OnError;
        internal static int Errors;
        internal static void Debug(string text) { }
        internal static void Error(string text) { Errors++; OnError?.Invoke(); }
    }
}
