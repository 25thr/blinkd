#!/usr/bin/env python3
"""
UDP listener for blinkd JSON event packets.

This script listens on UDP port 7777 for incoming JSON-formatted blink events,
typically sent by the `blinkd` C SDK.

Each packet is expected to contain:
    {
        "t_ms": <timestamp in ms>,
        "dur_ms": <duration in ms>,
        "flags": <bitmask indicating blink type>
    }

The script decodes the `flags` field into human-readable event names such as:
- BLINK
- LONG_BLINK
- DOUBLE_BLINK
- WINK_LEFT
- WINK_RIGHT

If a packet is malformed or not valid JSON, it will be printed as raw text.

Example incoming message:
    {"t_ms":12345,"dur_ms":100,"flags":1}

Example decoded output:
    [blinkd_event] t=12345ms  dur=100ms  flags=1 (BLINK)

Author: 03C0
"""

import socket
import json

UDP_IP = "127.0.0.1"
UDP_PORT = 7777
FLAGS = {
    1: "BLINK",
    2: "LONG_BLINK",
    4: "DOUBLE_BLINK",
    8: "WINK_LEFT",
    16: "WINK_RIGHT",
}

def decode_flags(f):
    return [name for bit, name in FLAGS.items() if f & bit]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"[blinkd_listener] Listening on {UDP_IP}:{UDP_PORT} ...")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode('utf-8', errors='ignore').strip()
        try:
            packet = json.loads(msg)
            names = decode_flags(packet["flags"])
            print(f"[blinkd_event] t={packet['t_ms']}ms  dur={packet['dur_ms']}ms  flags={packet['flags']} ({','.join(names)})")
        except json.JSONDecodeError:
            print(f"[raw] {msg}")
except KeyboardInterrupt:
    print("\n[blinkd_listener] Stopped.")
finally:
    sock.close()
