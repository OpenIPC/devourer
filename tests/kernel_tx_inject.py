#!/usr/bin/env python3
"""Flood MCS<idx>/20MHz 802.11 data frames out a monitor-mode netdev via a raw
AF_PACKET socket, as fast as the kernel driver accepts them. Apples-to-apples
with devourer's saturating MCS flood, for comparing kernel-driver vs devourer
on-air TX throughput (measure the air with tests/sdr_duty.py alongside).

  sudo python3 tests/kernel_tx_inject.py <monitor_iface> [mcs] [payload_bytes] [secs]
"""
import socket, struct, sys, time

iface = sys.argv[1]
mcs = int(sys.argv[2]) if len(sys.argv) > 2 else 7
payload = int(sys.argv[3]) if len(sys.argv) > 3 else 1500
secs = float(sys.argv[4]) if len(sys.argv) > 4 else 10.0

# radiotap with MCS field: present bit 19 (MCS). known=bw+mcs(0x03), flags bw20=0.
rt = struct.pack("<BBHI", 0, 0, 11, 1 << 19) + bytes([0x03, 0x00, mcs & 0xff])
# 802.11 data frame: FC=data, dur, A1=bcast, A2/A3 = canonical SA, seq.
sa = bytes([0x57, 0x42, 0x75, 0x05, 0xd6, 0x00])
dot11 = (bytes([0x08, 0x00, 0x00, 0x00]) + b"\xff\xff\xff\xff\xff\xff" +
         sa + sa + bytes([0x00, 0x00]))
frame = rt + dot11 + bytes(payload)

s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
s.bind((iface, 0))
n = 0
t0 = time.time()
while time.time() - t0 < secs:
    try:
        s.send(frame)
        n += 1
    except OSError:
        pass
dt = time.time() - t0
print(f"injected {n} frames in {dt:.1f}s ({n/dt:.0f} fps) mcs={mcs} payload={payload}")
