# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Diagnostic: list MAVLink message types arriving on the C++ core UDP port."""

from pymavlink import mavutil

# Listen on the same UDP port the C++ core uses (udpin:0.0.0.0:14570).
m = mavutil.mavlink_connection("udpin:0.0.0.0:14570")
m.wait_heartbeat(timeout=10)
print("heartbeat ok", flush=True)

seen = {}
for _ in range(400):
    msg = m.recv_match(blocking=True, timeout=8)
    if msg is None:
        break
    seen[msg.get_type()] = seen.get(msg.get_type(), 0) + 1

print("message types and counts:", flush=True)
for name in sorted(seen):
    print(f"  {name}: {seen[name]}", flush=True)

# Show GPS_RAW_INT detail if present.
if "GPS_RAW_INT" in seen:
    m2 = mavutil.mavlink_connection("udpin:0.0.0.0:14570")
    m2.wait_heartbeat(timeout=5)
    for _ in range(200):
        msg = m2.recv_match(blocking=True, timeout=8)
        if msg is None:
            break
        if msg.get_type() == "GPS_RAW_INT":
            print(f"GPS_RAW_INT fix_type={msg.fix_type} satellites={msg.satellites_visible}", flush=True)
            break
