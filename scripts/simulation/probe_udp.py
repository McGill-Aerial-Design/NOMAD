# SPDX-License-Identifier: Apache-2.0
"""Send five identified test datagrams to the Windows WSL-NAT gateway, not an aircraft."""

import ipaddress
import json
import socket
import subprocess
import sys

gateway = str(ipaddress.IPv4Address(subprocess.check_output(["ip", "route", "show", "default"], text=True).split()[2]))
payload = ("NOMAD_ROUTE_" + sys.argv[1]).encode("ascii")
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
    sent = [sender.sendto(payload, (gateway, 15600)) for _ in range(5)]
print(json.dumps({"gateway": gateway, "sent": len(sent), "bytes_each": sent}))
