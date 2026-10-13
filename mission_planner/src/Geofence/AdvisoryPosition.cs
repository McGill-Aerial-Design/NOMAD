// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;

namespace NOMAD.MissionPlanner
{
    // Four fixed host packet slots. A cached packet is admitted once, including rejected receipts.
    internal sealed class AdvisoryPosition
    {
        internal const int ReceiptBudgetMs = 3500;
        private readonly RouterClock _clock;
        private readonly Receipt[] _receipts = { new Receipt(), new Receipt(), new Receipt(), new Receipt() };
        private Receipt _selected;

        private sealed class Receipt
        {
            internal MAVLink.MAVLinkMessage Packet;
            internal GpsPoint Position;
            internal double Deadline = RouterClock.Unset;
            internal double ReceivedAt = RouterClock.Unset;
            internal DateTime Timestamp;
        }

        internal AdvisoryPosition(RouterClock clock)
        {
            _clock = clock;
        }

        internal void Block(MAVLink.MAVLinkMessage[] packets)
        {
            double now = _clock.Seconds();
            var utc = _clock.UtcNow();
            for (int index = 0; index < _receipts.Length; index++)
            {
                if (packets[index] != null && !ReferenceEquals(_receipts[index].Packet, packets[index]))
                {
                    _receipts[index].Packet = packets[index];
                    SetReceiptTime(_receipts[index], now, utc);
                }
            }
            Reject();
        }

        internal void Reject()
        {
            _selected = null;
            foreach (var receipt in _receipts)
            {
                receipt.Position = null;
                receipt.Deadline = RouterClock.Unset;
            }
        }

        internal GpsPoint Read(MAVLink.MAVLinkMessage[] packets, byte systemId, byte componentId)
        {
            double now = _clock.Seconds();
            var utc = _clock.UtcNow();
            for (int index = 0; index < _receipts.Length; index++)
            {
                Observe(_receipts[index], packets[index], systemId, componentId, now, utc);
            }
            int selectedIndex = -1;
            for (int index = 0; index < 3; index++)
            {
                var estimate = _receipts[index];
                if (packets[index] == null) { continue; }
                if (estimate.ReceivedAt == RouterClock.Unset)
                {
                    _selected = null;
                    return null;
                }
                if (selectedIndex < 0 || estimate.ReceivedAt > _receipts[selectedIndex].ReceivedAt)
                {
                    selectedIndex = index;
                }
            }
            if (selectedIndex >= 0 && HasTimestampTie(packets, selectedIndex))
            {
                _selected = null;
                return null;
            }
            _selected = selectedIndex >= 0 ? _receipts[selectedIndex] : (packets[3] != null ? _receipts[3] : null);
            return HasFreshPosition ? _selected.Position : null;
        }

        internal bool HasFreshPosition => _selected?.Position != null && _clock.Seconds() <= _selected.Deadline;

        private bool HasTimestampTie(MAVLink.MAVLinkMessage[] packets, int index)
        {
            for (int other = 0; other < 3; other++)
            {
                if (other != index && packets[other] != null &&
                    (_receipts[other].Timestamp == _receipts[index].Timestamp ||
                     _receipts[other].ReceivedAt == _receipts[index].ReceivedAt))
                {
                    return true;
                }
            }
            return false;
        }

        private static void SetReceiptTime(Receipt receipt, double now, DateTime utc)
        {
            receipt.Timestamp = receipt.Packet.rxtime;
            receipt.ReceivedAt = receipt.Timestamp != DateTime.MinValue && receipt.Timestamp.Kind == DateTimeKind.Utc
                ? now - (utc - receipt.Timestamp).TotalSeconds : RouterClock.Unset;
        }

        private void Observe(Receipt receipt, MAVLink.MAVLinkMessage packet, byte systemId, byte componentId,
            double observedAt, DateTime utc)
        {
            if (packet == null)
            {
                receipt.Position = null;
                receipt.Deadline = RouterClock.Unset;
                return;
            }
            if (ReferenceEquals(receipt.Packet, packet))
            {
                return;
            }
            receipt.Packet = packet;
            receipt.Position = null;
            receipt.Deadline = RouterClock.Unset;
            SetReceiptTime(receipt, observedAt, utc);
            if (packet == null || packet.sysid != systemId || packet.compid != componentId ||
                packet.rxtime == DateTime.MinValue || packet.rxtime.Kind != DateTimeKind.Utc)
            {
                return;
            }
            double ageMs = (utc - receipt.Timestamp).TotalMilliseconds;
            if (ageMs < 0 || ageMs > ReceiptBudgetMs)
            {
                return;
            }
            receipt.Position = Decode(packet);
            if (receipt.Position != null)
            {
                receipt.Deadline = observedAt + (ReceiptBudgetMs - ageMs) / 1000.0;
            }
        }

        private static GpsPoint Decode(MAVLink.MAVLinkMessage packet)
        {
            double latitude;
            double longitude;
            switch ((MAVLink.MAVLINK_MSG_ID)packet.msgid)
            {
                case MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT:
                    var global = packet.ToStructure<MAVLink.mavlink_global_position_int_t>();
                    latitude = global.lat / 1.0e7;
                    longitude = global.lon / 1.0e7;
                    break;
                case MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY:
#pragma warning disable CS0612 // The supported host still receives this legacy position format.
                    var high = packet.ToStructure<MAVLink.mavlink_high_latency_t>();
#pragma warning restore CS0612
                    latitude = high.latitude / 1.0e7;
                    longitude = high.longitude / 1.0e7;
                    break;
                case MAVLink.MAVLINK_MSG_ID.HIGH_LATENCY2:
                    var high2 = packet.ToStructure<MAVLink.mavlink_high_latency2_t>();
                    latitude = high2.latitude / 1.0e7;
                    longitude = high2.longitude / 1.0e7;
                    break;
                case MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT:
                    var gps = packet.ToStructure<MAVLink.mavlink_gps_raw_int_t>();
                    if (gps.fix_type < 2 || gps.fix_type > 8)
                    {
                        return null;
                    }
                    latitude = gps.lat / 1.0e7;
                    longitude = gps.lon / 1.0e7;
                    break;
                default:
                    return null;
            }
            return GeoMath.IsValidCoordinate(latitude, longitude) ? new GpsPoint(latitude, longitude) : null;
        }
    }
}
