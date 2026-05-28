#!/usr/bin/env python3
"""Unit tests for tools/usbmon_pcap_diff.py.

Synthesises a pcap stream of Linux usbmon (LINKTYPE_USB_LINUX_MMAPPED, 220)
records by hand-rolling the binary headers, writes it to a temp file, then
exercises the parser, aggregate, offload-probe, phase-split, and diff paths.

Run: PYTHONPATH=. python3 tests/test_usbmon_pcap_diff.py
"""

from __future__ import annotations

import hashlib
import os
import struct
import sys
import tempfile
import warnings
from pathlib import Path

# scapy emits a noisy "unknown LL type 220" warning every time it opens a
# pcap of LINKTYPE_USB_LINUX_MMAPPED — we parse the payload ourselves, so
# falling back to Raw packets is exactly what we want. Mute that line.
warnings.filterwarnings("ignore", message=".*unknown LL type.*")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.usbmon_pcap_diff import (  # noqa: E402
    PATH_A_LSSI_REG,
    REALTEK_VENQT_READ,
    REALTEK_VENQT_REQ,
    REALTEK_VENQT_WRITE,
    Urb,
    USBMON_MMAPPED_HDR_SIZE,
    _HDR_STRUCT,
    aggregate,
    diff,
    find_phase_boundary,
    offload_probe,
    read_urbs,
)

# pcap (classic, not pcapng) global + per-record header — easy to hand-roll.
# Magic 0xa1b2c3d4 (host byte order) with thiszone=0, sigfigs=0, snaplen=65535,
# network=220 (LINKTYPE_USB_LINUX_MMAPPED).
PCAP_GLOBAL = struct.pack(
    "<IHHiIII",
    0xA1B2C3D4,  # magic
    2, 4,         # version major/minor
    0, 0,         # thiszone, sigfigs
    65535,        # snaplen
    220,          # network
)

LINKTYPE_USB_LINUX_MMAPPED = 220
DIR_IN_BIT = 0x80
URB_SUBMIT = ord("S")
URB_COMPLETE = ord("C")
XFER_CTRL = 2
XFER_BULK = 3
XFER_INTR = 1


def _hdr(*, urb_id: int, event: int, xfer: int, ep: int, dir_in: bool,
         devnum: int, busnum: int, ts_us: int, status: int, length: int,
         setup: bytes = b"\x00" * 8, flag_setup: int = ord(" ")) -> bytes:
    """Build one 64-byte usbmon header."""
    assert len(setup) == 8
    epnum = (ep & 0x0F) | (DIR_IN_BIT if dir_in else 0)
    ts_sec = ts_us // 1_000_000
    ts_usec = ts_us % 1_000_000
    return _HDR_STRUCT.pack(
        urb_id, event, xfer, epnum, devnum, busnum,
        flag_setup, ord(" "),
        ts_sec, ts_usec, status, length, length,
        setup,
        0, 0, 0, 0,
    )


def _record(payload: bytes, ts_us: int) -> bytes:
    """Wrap one URB header+payload in a pcap-record header."""
    ts_sec = ts_us // 1_000_000
    ts_usec = ts_us % 1_000_000
    n = len(payload)
    return struct.pack("<IIII", ts_sec, ts_usec, n, n) + payload


def _ctrl_write(reg_num: int, data_bytes: bytes, ts_us: int,
                urb_id: int) -> tuple[bytes, bytes]:
    """One Realtek vendor write — emit Submit + Complete records.
    Returns (submit_record, complete_record) ready to concat into the pcap."""
    setup = struct.pack(
        "<B B H H H",
        REALTEK_VENQT_WRITE, REALTEK_VENQT_REQ,
        reg_num, 0, len(data_bytes),
    )
    submit_hdr = _hdr(
        urb_id=urb_id, event=URB_SUBMIT, xfer=XFER_CTRL, ep=0, dir_in=False,
        devnum=2, busnum=1, ts_us=ts_us, status=-115, length=len(data_bytes),
        setup=setup,
    )
    submit_payload = submit_hdr + data_bytes
    complete_hdr = _hdr(
        urb_id=urb_id, event=URB_COMPLETE, xfer=XFER_CTRL, ep=0, dir_in=False,
        devnum=2, busnum=1, ts_us=ts_us + 5, status=0, length=len(data_bytes),
    )
    complete_payload = complete_hdr  # OUT control: no data on complete
    return _record(submit_payload, ts_us), _record(complete_payload, ts_us + 5)


def _ctrl_read(reg_num: int, response: bytes, ts_us: int,
               urb_id: int) -> tuple[bytes, bytes]:
    setup = struct.pack(
        "<B B H H H",
        REALTEK_VENQT_READ, REALTEK_VENQT_REQ,
        reg_num, 0, len(response),
    )
    submit_hdr = _hdr(
        urb_id=urb_id, event=URB_SUBMIT, xfer=XFER_CTRL, ep=0, dir_in=True,
        devnum=2, busnum=1, ts_us=ts_us, status=-115, length=len(response),
        setup=setup,
    )
    complete_hdr = _hdr(
        urb_id=urb_id, event=URB_COMPLETE, xfer=XFER_CTRL, ep=0, dir_in=True,
        devnum=2, busnum=1, ts_us=ts_us + 5, status=0, length=len(response),
    )
    return _record(submit_hdr, ts_us), _record(complete_hdr + response, ts_us + 5)


def _bulk_out(ep: int, data: bytes, ts_us: int, urb_id: int) -> tuple[bytes, bytes]:
    submit_hdr = _hdr(
        urb_id=urb_id, event=URB_SUBMIT, xfer=XFER_BULK, ep=ep, dir_in=False,
        devnum=2, busnum=1, ts_us=ts_us, status=-115, length=len(data),
        flag_setup=ord("-"),
    )
    complete_hdr = _hdr(
        urb_id=urb_id, event=URB_COMPLETE, xfer=XFER_BULK, ep=ep, dir_in=False,
        devnum=2, busnum=1, ts_us=ts_us + 100, status=0, length=len(data),
        flag_setup=ord("-"),
    )
    return _record(submit_hdr + data, ts_us), _record(complete_hdr, ts_us + 100)


def _interrupt_in(ep: int, payload: bytes, ts_us: int, urb_id: int) -> tuple[bytes, bytes]:
    submit_hdr = _hdr(
        urb_id=urb_id, event=URB_SUBMIT, xfer=XFER_INTR, ep=ep, dir_in=True,
        devnum=2, busnum=1, ts_us=ts_us, status=-115, length=len(payload),
        flag_setup=ord("-"),
    )
    complete_hdr = _hdr(
        urb_id=urb_id, event=URB_COMPLETE, xfer=XFER_INTR, ep=ep, dir_in=True,
        devnum=2, busnum=1, ts_us=ts_us + 50, status=0, length=len(payload),
        flag_setup=ord("-"),
    )
    return _record(submit_hdr, ts_us), _record(complete_hdr + payload, ts_us + 50)


def _build_pcap(records: list[bytes]) -> bytes:
    return PCAP_GLOBAL + b"".join(records)


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------


def test_basic_parse(tmp: Path) -> None:
    rs = []
    rs += list(_ctrl_write(0x0100, b"\x05", 1_000_000, 1))
    rs += list(_ctrl_read(0x0100, b"\x05", 1_001_000, 2))
    rs += list(_bulk_out(0x02, b"\xff" * 100, 1_002_000, 3))
    rs += list(_interrupt_in(0x83, b"\x01\x02\x03", 1_003_000, 4))

    cap = tmp / "basic.pcap"
    cap.write_bytes(_build_pcap(rs))
    urbs = read_urbs(cap)
    assert len(urbs) == 8, f"expected 8 URBs, got {len(urbs)}"
    submits = [u for u in urbs if u.event == "S"]
    assert len(submits) == 4
    # First submit is the control write — must have setup decoded.
    write = submits[0]
    assert write.bmRequestType == REALTEK_VENQT_WRITE
    assert write.bRequest == REALTEK_VENQT_REQ
    assert write.wValue == 0x0100
    assert write.wLength == 1
    assert write.payload == b"\x05", f"payload was {write.payload!r}"
    # Read URB: payload arrives on the COMPLETE.
    read_complete = [
        u for u in urbs
        if u.event == "C" and u.xfer_type == 2 and u.dir_in
    ][0]
    assert read_complete.payload == b"\x05"
    # Bulk OUT submit carries 100 bytes of 0xff.
    bulk_submit = [u for u in urbs if u.xfer_type == 3 and u.event == "S"][0]
    assert bulk_submit.payload == b"\xff" * 100
    # Interrupt IN complete carries the response payload — must be first-class.
    intr_complete = [u for u in urbs if u.xfer_type == 1 and u.event == "C"][0]
    assert intr_complete.payload == b"\x01\x02\x03"
    print(f"  test_basic_parse: ok ({len(urbs)} URBs)")


def test_offload_probe_per_write(tmp: Path) -> None:
    # Simulate "devourer-style" capture: 2047 EP0 writes to path-A LSSI.
    rs = []
    for i in range(2047):
        rs += list(_ctrl_write(PATH_A_LSSI_REG, b"\xde\xad\xbe\xef",
                               1_000_000 + i * 200, 1000 + i))
    cap = tmp / "perwrite.pcap"
    cap.write_bytes(_build_pcap(rs))
    urbs = read_urbs(cap)
    probe = offload_probe(urbs)
    assert probe["path_a_writes"] == 2047
    assert "PER-WRITE" in probe["verdict"], probe["verdict"]
    print(f"  test_offload_probe_per_write: ok (2047 path-A writes detected)")


def test_offload_probe_offloaded(tmp: Path) -> None:
    # Simulate "kernel-FW-offload" capture: a handful of EP0 writes plus one
    # bulk-OUT carrying a fake H2C command.
    rs = []
    for i in range(10):
        rs += list(_ctrl_write(PATH_A_LSSI_REG, b"\x11\x22\x33\x44",
                               1_000_000 + i * 200, 2000 + i))
    rs += list(_bulk_out(0x03, b"\xa5" * 64, 1_100_000, 2100))
    cap = tmp / "offloaded.pcap"
    cap.write_bytes(_build_pcap(rs))
    urbs = read_urbs(cap)
    probe = offload_probe(urbs)
    assert probe["path_a_writes"] == 10
    assert "OFFLOAD" in probe["verdict"], probe["verdict"]
    assert probe["h2c_candidates"] >= 1
    print(f"  test_offload_probe_offloaded: ok ({probe['path_a_writes']} writes, "
          f"{probe['h2c_candidates']} H2C candidates)")


def test_phase_split_by_sentinel(tmp: Path) -> None:
    # Sentinel: write 0xDEAD to 0x01C0 before init, 0xBEEF after init.
    rs = []
    rs += list(_ctrl_write(0x01C0, b"\xad\xde", 1_000_000, 5000))  # init start
    for i in range(20):
        rs += list(_ctrl_write(0x0100 + i, b"\x42", 1_010_000 + i * 10, 5100 + i))
    rs += list(_ctrl_write(0x01C0, b"\xef\xbe", 1_020_000, 5200))  # init end
    rs += list(_bulk_out(0x02, b"tx" * 50, 1_030_000, 5300))
    cap = tmp / "phase.pcap"
    cap.write_bytes(_build_pcap(rs))
    urbs = read_urbs(cap)
    boundary = find_phase_boundary(urbs, sentinel_reg=0x01C0)
    # The init-end sentinel SUBMIT is at index 44 (22 ops * 2 each); boundary
    # returned is the index AFTER it.
    print(f"  test_phase_split_by_sentinel: boundary at {boundary}")
    assert boundary > 0 and boundary < len(urbs)
    # Everything after the boundary should be TX (bulk-OUT only).
    tail = urbs[boundary:]
    bulk_in_tail = [u for u in tail if u.xfer_type == 3]
    assert len(bulk_in_tail) == 2, f"expected 2 bulk records after boundary, got {len(bulk_in_tail)}"


def test_phase_split_by_gap(tmp: Path) -> None:
    rs = []
    for i in range(10):
        rs += list(_ctrl_write(0x0100 + i, b"\x10", 1_000_000 + i * 100, 6000 + i))
    # 50ms quiescent gap.
    rs += list(_bulk_out(0x02, b"x" * 40, 1_500_000, 6100))
    cap = tmp / "gap.pcap"
    cap.write_bytes(_build_pcap(rs))
    urbs = read_urbs(cap)
    boundary = find_phase_boundary(urbs, sentinel_reg=0x9999, min_gap_us=20_000)
    print(f"  test_phase_split_by_gap: boundary at {boundary}")
    # boundary should land just before the bulk-OUT submit.
    assert urbs[boundary].xfer_type == 3 and urbs[boundary].event == "S"


def test_diff_finds_extra_urb(tmp: Path) -> None:
    # Capture A: 5 writes. Capture B: same 5 + 1 extra in the middle.
    rs_a, rs_b = [], []
    for i in range(5):
        rs_a += list(_ctrl_write(0x0100 + i, b"\x01", 1_000_000 + i * 100, 7000 + i))
        rs_b += list(_ctrl_write(0x0100 + i, b"\x01", 1_000_000 + i * 100, 8000 + i))
        if i == 2:
            rs_b += list(_ctrl_write(0xC90, b"\xaa\xbb\xcc\xdd",
                                     1_000_250, 8500))
    cap_a = tmp / "a.pcap"
    cap_b = tmp / "b.pcap"
    cap_a.write_bytes(_build_pcap(rs_a))
    cap_b.write_bytes(_build_pcap(rs_b))
    a = read_urbs(cap_a)
    b = read_urbs(cap_b)
    res = diff(a, b)
    assert res["deltas"], "expected at least one delta"
    extra_b = [d for d in res["deltas"] if d["idx_a"] is None]
    assert extra_b, f"expected an 'extra in B' delta, got {res['deltas']}"
    print(f"  test_diff_finds_extra_urb: ok ({len(res['deltas'])} deltas)")


def test_diff_finds_payload_divergence(tmp: Path) -> None:
    rs_a, rs_b = [], []
    for i in range(5):
        rs_a += list(_ctrl_write(0xC90, b"\xaa\xaa\xaa\xaa",
                                 1_000_000 + i * 100, 9000 + i))
        # Capture B: same registers, different payload on entry 2 — simulates a
        # data-byte divergence that text usbmon would silently drop.
        payload = b"\xbb\xbb\xbb\xbb" if i == 2 else b"\xaa\xaa\xaa\xaa"
        rs_b += list(_ctrl_write(0xC90, payload,
                                 1_000_000 + i * 100, 9500 + i))
    cap_a = tmp / "a2.pcap"
    cap_b = tmp / "b2.pcap"
    cap_a.write_bytes(_build_pcap(rs_a))
    cap_b.write_bytes(_build_pcap(rs_b))
    a = read_urbs(cap_a)
    b = read_urbs(cap_b)
    res = diff(a, b)
    # Each ctrl write emits 2 URBs (Submit + Complete), so the diverged
    # payload sits at URB-index 4. The resync emits 2 deltas naming the
    # B-side records the walker couldn't pair (B[4] = Submit-bb, B[5] =
    # its Complete). SHA bb*4 = bfcd12432b17 — that string MUST surface
    # in at least one delta, otherwise the payload divergence was
    # silently dropped (the exact failure mode of the old text-format
    # tool).
    sha_bb = hashlib.sha256(b"\xbb\xbb\xbb\xbb").hexdigest()[:12]
    assert any(sha_bb in d["b"] for d in res["deltas"]), \
        f"expected delta naming SHA {sha_bb}, got {res['deltas']}"
    print(f"  test_diff_finds_payload_divergence: ok ({len(res['deltas'])} deltas, "
          f"sha bb*4={sha_bb} surfaced)")


def test_aggregate_classifies_in_urbs(tmp: Path) -> None:
    rs = []
    rs += list(_ctrl_write(0x100, b"\x01", 1_000_000, 10000))
    rs += list(_ctrl_read(0x100, b"\x01", 1_000_100, 10001))
    rs += list(_bulk_out(0x02, b"abc", 1_000_200, 10002))
    rs += list(_interrupt_in(0x83, b"\x99", 1_000_300, 10003))
    cap = tmp / "agg.pcap"
    cap.write_bytes(_build_pcap(rs))
    urbs = read_urbs(cap)
    agg = aggregate(urbs)
    # 4 submits.
    assert agg["submits"] == 4
    # 1 realtek write, 1 realtek read.
    assert agg["realtek_writes"] == 1
    assert agg["realtek_reads"] == 1
    # 1 IN URB (the interrupt; the control read is also IN-directed).
    assert agg["in_urbs_submitted"] == 2, \
        f"expected 2 IN submits (ctrl read + intr in), got {agg['in_urbs_submitted']}"
    print(f"  test_aggregate_classifies_in_urbs: ok")


def main() -> int:
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        print("running tests...")
        for fn in [
            test_basic_parse,
            test_offload_probe_per_write,
            test_offload_probe_offloaded,
            test_phase_split_by_sentinel,
            test_phase_split_by_gap,
            test_diff_finds_extra_urb,
            test_diff_finds_payload_divergence,
            test_aggregate_classifies_in_urbs,
        ]:
            fn(tmp)
        print("ALL OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
