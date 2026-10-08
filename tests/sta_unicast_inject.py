#!/usr/bin/env python3
"""Inject unicast 802.11 data frames at a MAC from a monitor interface.

Companion to tests/mt7612u_sta_identity.sh and `mt7612uprobe sta`.

That gate counts what a managed station RECEIVES. Beacons alone only exercise
broadcast reception, and the question it exists to answer is whether the BSSID
registers gate UNICAST delivery to the station's own address. hostapd sends an
unassociated station no unicast at all, so without this the gate reads
to_us=0 in every arm and cannot answer it. This makes the traffic.

The monitor vif lives on the AP's own phy, so the injection rides the AP's
radio and lands on the AP's channel without needing a third adapter.

  sta_unicast_inject.py <mon-if> <dst-mac> <bssid> [seconds] [pps] [seq0]

seq0 (0..4095, default 0) is the first sequence number. Two injectors sharing
a transmitter address can be given disjoint ranges, so 802.11 duplicate
detection cannot merge the two streams. This script sets no Retry bit, but the
AP's hardware may retransmit a frame nobody acknowledges, with Retry set: in
tests/sta_client_onair.sh an unarmed station counts 10-30x as many frames to
an address nobody holds as were injected, while the stream to its own
address, which it acknowledges, arrives 1:1.

Note what this does NOT do: it never sees an acknowledgement, so it cannot
measure one. Acknowledgement is measured by tests/mt7612u_sta_autoack.sh,
which asks the transmitter (a Realtek peer's per-frame CCX reports).
"""
import signal
import socket
import struct
import sys
import time

# version 0, pad 0, length 8, present bitmap 0 - no fields, just the header.
RADIOTAP = struct.pack('<BBHI', 0, 0, 8, 0)

# pps is capped: this runs as root on a raw socket, and a rate the caller
# mistyped must not become a flood.
MAX_PPS = 2000.0


class Usage(Exception):
    pass


def mac(s):
    parts = s.split(':')
    if len(parts) != 6 or any(not 1 <= len(x) <= 2 for x in parts):
        raise Usage('not a MAC address: %r' % s)
    try:
        octets = [int(x, 16) for x in parts]
        if any(not 0 <= o <= 255 for o in octets):
            raise ValueError(s)
        return bytes(octets)
    except ValueError:
        raise Usage('not a MAC address: %r' % s)


def positive(name, s, cap=None):
    try:
        v = float(s)
    except ValueError:
        raise Usage('%s must be a number: %r' % (name, s))
    if not 0 < v < float('inf'):
        raise Usage('%s must be finite and > 0: %r' % (name, s))
    if cap is not None and v > cap:
        raise Usage('%s must be <= %g: %r' % (name, cap, s))
    return v


class Stop(Exception):
    pass


def on_term(signum, frame):
    raise Stop()


def main(argv):
    if len(argv) < 4 or len(argv) > 7:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        mon = argv[1]
        dst = mac(argv[2])
        bssid = mac(argv[3])
        secs = positive('seconds', argv[4]) if len(argv) > 4 else 120.0
        pps = positive('pps', argv[5], MAX_PPS) if len(argv) > 5 else 300.0
        seq0 = 0
        if len(argv) > 6:
            try:
                seq0 = int(argv[6])
            except ValueError:
                raise Usage('seq0 must be an integer: %r' % argv[6])
            if not 0 <= seq0 <= 4095:
                raise Usage('seq0 must be 0..4095: %r' % argv[6])
    except Usage as e:
        print('sta_unicast_inject: %s' % e, file=sys.stderr)
        print(__doc__, file=sys.stderr)
        return 2

    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
    sock.bind((mon, 0))
    # The harness stops this with SIGTERM once its gate is done; the count is
    # printed either way, because the harness requires a nonzero one.
    signal.signal(signal.SIGTERM, on_term)

    # Data frame, FromDS: addr1 = the station, addr2 = addr3 = the BSSID.
    # That is the shape an AP's downlink traffic has, which is what a station
    # would be filtering for.
    body = b'STA-UNICAST-PROBE' * 4
    gap = 1.0 / pps
    end = time.time() + secs
    sent = 0
    seq = seq0
    try:
        while time.time() < end:
            hdr = (struct.pack('<BBH', 0x08, 0x02, 0) + dst + bssid + bssid +
                   struct.pack('<H', (seq & 0x0fff) << 4))
            try:
                sock.send(RADIOTAP + hdr + body)
                sent += 1
            except OSError:
                # A busy or momentarily-down monitor vif is not fatal here;
                # the gate cares about the arms being comparable, not about
                # any single frame landing.
                time.sleep(0.01)
            seq += 1
            time.sleep(gap)
    except (Stop, KeyboardInterrupt):
        pass
    print('injected %d unicast frames at %s' % (sent, argv[2]), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
