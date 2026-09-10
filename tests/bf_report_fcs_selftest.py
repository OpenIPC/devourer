#!/usr/bin/env python3
"""FCS-presence handling in tools/bf_report_decode.py.

The C++ half of the FCS-presence change is covered by the bf_report_decode
ctest cell; this covers the Python half, where the real regressions have been.
Both defects found in review lived here: report_hex()'s return type change
silently dropped a None guard (turning a skippable malformed event into an
uncaught TypeError across three tools), and parse_mu_snr() reserved trailing
bytes unconditionally.

No hardware, no network.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import bf_report_decode as bf  # noqa: E402

# A real VHT MU compressed beamforming report, FCS included (same capture the
# C++ selftest uses).
FRAME = (
    "e000000056427505d60000e04c8822ce00000000000010001500088c04c2a98dad97b09fbe"
    "addaadc7bfccbde1c9e5cfe8cdf3cdf1d1eacfe5cff0d5eed9f0d9e6e1ffd70cd820e017d2"
    "25d61ef093f0b9daa1ec91e687dc83e093e058f417ecfbebf9ebe9e1f1db03dc08de0dda11"
    "d814de0fd808d60dd219c815c605b811b01bac20b62ac40f01f00fef00100100ff00111110"
    "01cbbf1bd5"
)
FRAME_NOFCS = FRAME[:-8]          # the same report as an FCS-less backend delivers

fails = 0


def check(cond, what):
    global fails
    if not cond:
        print(f"  FAIL {what}")
        fails += 1


def ev(body):
    return '{"ev":"bf.report_raw",' + body + "}"


# --- report_hex: the contract is (hex, fcs_present) or None ---------------
check(bf.report_hex('{"ev":"rx.pkt","n":1}') is None, "foreign event ignored")
check(bf.report_hex(ev('"fcs":1,"frame":"%s"' % FRAME)) == (FRAME, True), "fcs:1")
check(bf.report_hex(ev('"fcs":0,"frame":"%s"' % FRAME_NOFCS)) == (FRAME_NOFCS, False), "fcs:0")
check(bf.report_hex(ev('"frame":"%s"' % FRAME)) == (FRAME, True), "legacy event has no fcs -> assumed present")
check(bf.report_hex(FRAME) == (FRAME, True), "bare hex -> assumed present")

# The regression: an event with no `frame` must stay skippable. Before the
# guard was restored this returned (None, True), and bytes.fromhex(None) then
# raised TypeError, which parse_frame's `except ValueError` does not catch.
check(bf.report_hex(ev('"fcs":1')) is None, "event without a frame field is skippable")
try:
    got = bf.read_frames(iter([ev('"fcs":1'), '{"ev":"rx.pkt"}', ""]))
    check(got == [], "read_frames survives malformed input")
except Exception as exc:                                   # noqa: BLE001
    check(False, f"read_frames raised {type(exc).__name__} on malformed input")

# --- the equivalence the flag exists to produce ---------------------------
with_fcs = bf.parse_frame(FRAME, True)
no_fcs = bf.parse_frame(FRAME_NOFCS, False)
check(with_fcs is not None and no_fcs is not None, "both shapes parse")
if with_fcs and no_fcs:
    check(len(no_fcs["angle_bytes"]) == len(with_fcs["angle_bytes"]),
          "same logical report -> same angle block either way")
    check(no_fcs["snr"] == with_fcs["snr"], "same per-column SNR")

# Believing an FCS that is not there must cost exactly four bytes - that is
# the corruption this whole change exists to prevent.
wrong = bf.parse_frame(FRAME_NOFCS, True)
if wrong and no_fcs:
    check(len(wrong["angle_bytes"]) == len(no_fcs["angle_bytes"]) - 4,
          "assuming an absent FCS loses four angle bytes")

# --- parse_mu_snr: the FCS-present bound must not have moved --------------
if with_fcs:
    a = bf.parse_mu_snr(with_fcs, with_fcs["ns"] if "ns" in with_fcs else 52,
                        len(with_fcs["angle_bytes"]))
    legacy = dict(with_fcs)
    legacy.pop("fcs_present", None)        # a dict from before the field existed
    b = bf.parse_mu_snr(legacy, legacy["ns"] if "ns" in legacy else 52,
                        len(legacy["angle_bytes"]))
    check(a == b, "FCS-present MU-SNR identical with and without the new key")

print("bf_report_fcs: " + ("FAIL" if fails else "PASS"))
sys.exit(1 if fails else 0)
