#!/usr/bin/env python3
"""Binary-fidelity diff for USB captures collected by tshark on usbmonN.

Why this exists (the epistemic point that motivated it):

  If the URB sequence on the wire is truly identical between two captures and
  one produces a result the other doesn't, the diff has a gap. Prior
  `tools/usbmon_diff.py` (text-format usbmon, bulk-OUT only) cannot represent
  EP0 control transfers, full payload bytes, URB flags, IN URBs, or chip-side
  C2H interrupts. So "the wire matches" claims based on it were unfounded.

This tool reads pcapng captured via
    sudo modprobe usbmon
    sudo tshark -i usbmonN -s 0 -w cap.pcapng
and surfaces every URB axis that the text-format tool drops:

  * Control transfer setup packet: bmRequestType, bRequest, wValue, wIndex,
    wLength. For Realtek register writes this is bmRequestType=0x40,
    bRequest=5, wValue=<register addr>, wIndex=0, wLength={1,2,4}.
  * Full payload bytes (not truncated) + SHA-256 for fast equality checks.
  * URB status (0=OK, -ESHUTDOWN, -ETIMEDOUT, -EPIPE...).
  * Timestamps and inter-URB gaps at microsecond resolution.
  * IN URBs (control reads, bulk-IN, interrupt-IN) — first-class diff units,
    NOT filtered away.

Capture format expected: pcap link-layer type 220 = LINKTYPE_USB_LINUX_MMAPPED
(64-byte usbmon header in front of each URB) or type 189 = LINKTYPE_USB_LINUX
(48-byte header). Both are produced by tshark on different kernels; both are
handled here.

Modes:
  default       — full URB-stream diff between two captures.
  --offload-probe COUNT  — count Realtek control writes to path-A LSSI in a
                  single capture, classify as FW-offload vs per-write. This
                  answers the Phase-3 gate in the plan: if the kernel routes
                  the 2047-entry RF table through firmware (one bulk-OUT)
                  rather than 2047 EP0 control writes, the userspace driver's
                  wire shape is fundamentally divergent.
  --phase-split — detect "init done, TX starts" boundary by largest inter-URB
                  gap, report per-phase aggregates.
  --aggregate   — histograms only (EP, xfer_type, control bRequest, register
                  address pages).

Filtering:
  --busnum N    — limit to a specific USB bus (use `lsusb` to find).
  --devnum N    — limit to a specific device address on that bus.

The defaults are no filter. tshark already filters by interface; if multiple
devices share a bus, --devnum is what you want.
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import hashlib
import struct
import sys
from pathlib import Path
from typing import Iterable, Iterator, Optional

try:
    import logging
    from scapy.utils import PcapNgReader, PcapReader
    from scapy.error import Scapy_Exception
    # scapy logs "unknown LL type 220" every time we open a usbmon pcap.
    # We do the framing ourselves and want the Raw-packet fallback, so
    # raise its log level above WARNING.
    logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
except ImportError as e:  # pragma: no cover
    print(f"scapy is required: {e}", file=sys.stderr)
    sys.exit(2)


# Linux usbmon binary headers — kernel/Documentation/usb/usbmon.rst
# LINKTYPE_USB_LINUX_MMAPPED (220) is the 64-byte form produced since ~3.x.
# LINKTYPE_USB_LINUX (189) is the older 48-byte form.
USBMON_MMAPPED_HDR_SIZE = 64
USBMON_LEGACY_HDR_SIZE = 48

# Header format: id, type, xfer, epnum, devnum, busnum, flag_setup, flag_data,
# ts_sec, ts_usec, status, length, len_cap, setup[8], interval, start_frame,
# xfer_flags, ndesc.
_HDR_STRUCT = struct.Struct("<Q B B B B H b b q i i I I 8s i i I I")
assert _HDR_STRUCT.size == USBMON_MMAPPED_HDR_SIZE

# Endpoint direction bit: high bit of bEndpointAddress for OUT/IN URBs.
# In usbmon's epnum field this is preserved.
DIR_IN_BIT = 0x80

XFER_TYPE = {0: "ISO", 1: "INTR", 2: "CTRL", 3: "BULK"}
XFER_LETTER = {0: "Z", 1: "I", 2: "C", 3: "B"}

# Realtek vendor-request constants — kept in sync with src/RtlUsbAdapter.h.
REALTEK_VENQT_WRITE = 0x40  # bmRequestType: host->device, vendor, recipient device
REALTEK_VENQT_READ = 0xC0  # bmRequestType: device->host, vendor, recipient device
REALTEK_VENQT_REQ = 0x05  # bRequest

# Path-A LSSI register window — RTL8814AU jaguar baseband.
# Mirrors src/RadioManagementModule.cpp:308 (rA_LSSIWrite_Jaguar = 0xC90).
PATH_A_LSSI_REG = 0xC90
PATH_A_LSSI_WINDOW = (0xC90, 0xCDF)

# Standard errno strings for common URB completions on Linux.
ERRNO = {
    0: "OK",
    -2: "-ENOENT",
    -32: "-EPIPE",
    -71: "-EPROTO",
    -75: "-EOVERFLOW",
    -84: "-EILSEQ",
    -108: "-ESHUTDOWN",
    -110: "-ETIMEDOUT",
    -115: "-EINPROGRESS",
    -121: "-EREMOTEIO",
}


@dataclasses.dataclass
class Urb:
    """One usbmon record (a Submit, Complete, or Error event for a URB).

    A SUBMIT carries the host's intent (setup packet for control, the OUT
    payload for bulk/interrupt-OUT). A COMPLETE carries the device's reply
    (data for control/bulk/interrupt-IN, status code, actual length).
    Pair S/C by (id) to get a full transaction.
    """

    id: int
    event: str  # 'S', 'C', 'E'
    xfer_type: int  # 0..3 — see XFER_TYPE
    ep: int  # endpoint number (low 4 bits) — direction in `dir_in`
    dir_in: bool  # True for device->host
    devnum: int
    busnum: int
    ts_us: int  # microseconds since epoch (sec*1e6 + usec)
    status: int  # 0 on success, errno negative otherwise
    length: int  # requested length
    len_cap: int  # captured payload length
    setup: bytes  # 8 bytes; valid only when xfer_type==CTRL and event=='S'
    payload: bytes  # actual captured URB data

    # Decoded fields for control transfers (setup unpacked).
    bmRequestType: Optional[int] = None
    bRequest: Optional[int] = None
    wValue: Optional[int] = None
    wIndex: Optional[int] = None
    wLength: Optional[int] = None

    @property
    def payload_sha(self) -> str:
        return hashlib.sha256(self.payload).hexdigest()[:12] if self.payload else ""

    def is_realtek_write(self) -> bool:
        return (
            self.xfer_type == 2
            and self.event == "S"
            and self.bmRequestType == REALTEK_VENQT_WRITE
            and self.bRequest == REALTEK_VENQT_REQ
        )

    def is_realtek_read(self) -> bool:
        return (
            self.xfer_type == 2
            and self.event == "S"
            and self.bmRequestType == REALTEK_VENQT_READ
            and self.bRequest == REALTEK_VENQT_REQ
        )

    def short(self) -> str:
        kind = XFER_LETTER[self.xfer_type]
        d = "i" if self.dir_in else "o"
        s = f"{self.event} {kind}{d} dev={self.devnum} ep={self.ep:02x}"
        if self.xfer_type == 2 and self.event == "S" and self.bmRequestType is not None:
            s += (
                f" bmRT=0x{self.bmRequestType:02x} bReq={self.bRequest} "
                f"wVal=0x{self.wValue:04x} wIdx=0x{self.wIndex:04x} "
                f"wLen={self.wLength}"
            )
        s += f" status={ERRNO.get(self.status, str(self.status))}"
        s += f" len={self.length}/{self.len_cap}"
        if self.payload:
            s += f" sha={self.payload_sha}"
        return s


def _decode_urb(raw: bytes) -> Optional[Urb]:
    """Parse one pcap record (link-type 220) into a Urb. Returns None on
    truncated records — defensive against torn captures and unknown link
    types. Bytes past the 64-byte header are the URB payload (for OUT
    SUBMITs and IN COMPLETIONs)."""

    if len(raw) < USBMON_MMAPPED_HDR_SIZE:
        # Legacy 48-byte header — older kernels. Pad up so the unpack stays
        # consistent, then ignore the trailing 16 bytes we synthesised.
        if len(raw) < USBMON_LEGACY_HDR_SIZE:
            return None
        raw = raw[:USBMON_LEGACY_HDR_SIZE] + b"\x00" * 16 + raw[USBMON_LEGACY_HDR_SIZE:]

    (
        urb_id, etype, xfer, epnum, devnum, busnum, flag_setup, flag_data,
        ts_sec, ts_usec, status, length, len_cap, setup,
        _interval, _start_frame, _xfer_flags, _ndesc,
    ) = _HDR_STRUCT.unpack_from(raw, 0)

    payload = raw[USBMON_MMAPPED_HDR_SIZE:USBMON_MMAPPED_HDR_SIZE + len_cap]

    event = chr(etype) if etype in (ord("S"), ord("C"), ord("E")) else "?"
    dir_in = bool(epnum & DIR_IN_BIT)
    ep = epnum & 0x0F
    ts_us = ts_sec * 1_000_000 + ts_usec

    urb = Urb(
        id=urb_id, event=event, xfer_type=xfer, ep=ep, dir_in=dir_in,
        devnum=devnum, busnum=busnum, ts_us=ts_us, status=status,
        length=length, len_cap=len_cap, setup=setup, payload=payload,
    )

    # Decode control setup packet for every CTRL submit. Kernel-doc says the
    # flag_setup byte should be ' ' (0x20) when the setup is captured, but
    # current Linux kernels (verified 6.x) write 0 instead — so checking
    # against any specific magic byte is fragile. For a control submit the
    # setup bytes are valid by construction; decode unconditionally.
    if xfer == 2 and event == "S":
        (urb.bmRequestType, urb.bRequest, urb.wValue, urb.wIndex,
         urb.wLength) = struct.unpack("<B B H H H", setup)
        # Some captures still emit 'Z' / '-' / 'X' to indicate "no setup
        # packet captured" — surface that by leaving the fields None.
        if flag_setup not in (0, 0x20):
            urb.bmRequestType = urb.bRequest = None
            urb.wValue = urb.wIndex = urb.wLength = None

    return urb


def _open_reader(path: Path):
    """Open a pcap or pcapng file. tshark writes pcapng by default; we still
    accept legacy pcap for hand-rolled synthetic captures used in tests."""
    # PcapNgReader chokes on plain pcap with a magic mismatch — try it first
    # then fall back to PcapReader.
    try:
        return PcapNgReader(str(path))
    except Scapy_Exception:
        return PcapReader(str(path))


def read_urbs(
    path: Path,
    *,
    busnum: Optional[int] = None,
    devnum: Optional[int] = None,
) -> list[Urb]:
    """Read every URB record from a pcap/pcapng. Optionally filter to a
    single bus / device address."""

    urbs: list[Urb] = []
    with _open_reader(path) as r:
        for pkt in r:
            # scapy returns a Packet whose raw bytes start with the link-layer
            # header. For USB_LINUX_MMAPPED that's the 64-byte usbmon header.
            raw = bytes(pkt)
            urb = _decode_urb(raw)
            if urb is None:
                continue
            if busnum is not None and urb.busnum != busnum:
                continue
            if devnum is not None and urb.devnum != devnum:
                continue
            urbs.append(urb)
    return urbs


# --------------------------------------------------------------------------
# Phase split — finds the largest inter-URB gap and uses it as the init/TX
# boundary. Devourer-side captures should have a sentinel (DEAD/BEEF wValue);
# kernel-side captures rely on the deliberate quiescent gap the operator
# inserted between `modprobe` and `airodump-ng`.
# --------------------------------------------------------------------------


def find_phase_boundary(
    urbs: list[Urb], *, sentinel_reg: Optional[int] = None,
    min_gap_us: int = 20_000,
) -> int:
    """Return the index AT WHICH the TX phase begins (urbs[idx:] is TX,
    urbs[:idx] is init). If sentinel_reg is set and a control write with
    wValue=0xBEEF to that register is present, that wins. Otherwise the
    largest inter-URB gap exceeding min_gap_us is the marker.
    Returns len(urbs) if no boundary detected (all-init capture)."""

    if sentinel_reg is not None:
        for i, u in enumerate(urbs):
            if u.is_realtek_write() and u.wValue == sentinel_reg \
                    and u.payload == b"\xef\xbe":
                return i + 1
            if u.is_realtek_write() and u.wValue == sentinel_reg \
                    and u.payload == b"\xef\xbe\x00\x00":
                return i + 1

    submits = [u for u in urbs if u.event == "S"]
    if len(submits) < 2:
        return len(urbs)
    gaps = [
        (submits[i + 1].ts_us - submits[i].ts_us, i + 1)
        for i in range(len(submits) - 1)
    ]
    biggest, idx = max(gaps, key=lambda t: t[0])
    if biggest < min_gap_us:
        return len(urbs)
    # Translate the submits-list index back to the full urbs-list index.
    target = submits[idx]
    return urbs.index(target)


# --------------------------------------------------------------------------
# Offload probe — answers Phase-3 gate of the plan. Counts Realtek control
# writes whose wValue falls in the path-A LSSI window; if it's near 2047
# the loader is doing per-entry writes (devourer behaviour), if it's <50
# the table almost certainly went down via H2C FW offload.
# --------------------------------------------------------------------------


def offload_probe(
    urbs: list[Urb], *,
    lssi_window: tuple[int, int] = PATH_A_LSSI_WINDOW,
    threshold_offload: int = 50,
    threshold_walk: int = 1500,
) -> dict:
    """Classify a capture by counting per-entry path-A RF writes. Returns a
    dict with counts and a verdict string."""

    lo, hi = lssi_window
    writes_in_window = sum(
        1 for u in urbs
        if u.is_realtek_write() and lo <= (u.wValue or 0) <= hi
    )
    # Any bulk-OUT that LOOKS like an H2C command frame (small first byte
    # indicating a phydm offload opcode) — we don't know the cmd id yet, so
    # surface a count of suspicious bulk-OUTs for manual inspection.
    bulk_out_submits = [
        u for u in urbs
        if u.event == "S" and u.xfer_type == 3 and not u.dir_in
    ]
    suspect_h2c = [
        u for u in bulk_out_submits
        if 16 <= len(u.payload) <= 256
    ]

    if writes_in_window >= threshold_walk:
        verdict = "PER-WRITE (devourer-like). 2047-entry RF table walked one EP0 control transfer at a time."
    elif writes_in_window <= threshold_offload:
        verdict = (
            "OFFLOAD-LIKELY. Path-A LSSI window has very few EP0 writes; "
            "kernel almost certainly batched the RF table via an H2C bulk-OUT command. "
            "Devourer does not implement PHYDM_PHY_PARAM_OFFLOAD — that is the divergence."
        )
    else:
        verdict = "MIXED / inconclusive — inspect manually."

    return {
        "lssi_window": f"0x{lo:04x}..0x{hi:04x}",
        "path_a_writes": writes_in_window,
        "bulk_out_submits_total": len(bulk_out_submits),
        "h2c_candidates": len(suspect_h2c),
        "verdict": verdict,
    }


# --------------------------------------------------------------------------
# Aggregate summary — histograms, no diff. Useful to spot structural
# differences (e.g. one side has IN URBs the other side doesn't even read).
# --------------------------------------------------------------------------


def aggregate(urbs: list[Urb]) -> dict:
    submits = [u for u in urbs if u.event == "S"]
    completes = [u for u in urbs if u.event == "C"]
    by_kind = collections.Counter(
        (XFER_TYPE.get(u.xfer_type, "?"), "IN" if u.dir_in else "OUT")
        for u in submits
    )
    realtek_writes = [u for u in submits if u.is_realtek_write()]
    realtek_reads = [u for u in submits if u.is_realtek_read()]
    write_pages = collections.Counter(
        (u.wValue or 0) & 0xFF00 for u in realtek_writes
    )
    in_urbs = [u for u in submits if u.dir_in]
    statuses = collections.Counter(
        ERRNO.get(u.status, str(u.status)) for u in completes
    )
    return {
        "submits": len(submits),
        "completes": len(completes),
        "by_kind": dict(by_kind),
        "realtek_writes": len(realtek_writes),
        "realtek_reads": len(realtek_reads),
        "write_pages_top": write_pages.most_common(8),
        "in_urbs_submitted": len(in_urbs),
        "completion_status": dict(statuses),
    }


# --------------------------------------------------------------------------
# Diff — position-aligned URB comparison. Two captures, walked in parallel.
# Records every divergence and reports a normalized delta.
# --------------------------------------------------------------------------


def _semantic_tuple(u: Urb) -> tuple:
    """A tuple that should be equal across two captures of the SAME logical
    operation. Drops absolute timestamps and URB IDs (kernel pointers); keeps
    transfer kind, EP, direction, setup packet, payload hash, status."""
    return (
        u.event,
        u.xfer_type,
        u.ep,
        u.dir_in,
        u.bmRequestType,
        u.bRequest,
        u.wValue,
        u.wIndex,
        u.wLength,
        u.payload_sha,
        u.status,
    )


def diff(
    a: list[Urb], b: list[Urb], *,
    max_report: int = 50,
) -> dict:
    """Walk both URB streams in parallel. At each step, compare semantic
    tuples; on divergence record both sides and resynchronise by skipping
    the side that's ahead. This is intentionally a simple LCS-free walk —
    if the URB streams are truly equivalent the walk stays paired; if they
    diverge structurally the report will surface that immediately."""

    deltas = []
    i = j = 0
    while i < len(a) and j < len(b) and len(deltas) < max_report:
        ta, tb = _semantic_tuple(a[i]), _semantic_tuple(b[j])
        if ta == tb:
            i += 1
            j += 1
            continue
        # Try to resynchronise: look ahead a small window on either side.
        WINDOW = 8
        found = None
        for k in range(1, WINDOW + 1):
            if j + k < len(b) and _semantic_tuple(b[j + k]) == ta:
                found = ("B_AHEAD", k)
                break
            if i + k < len(a) and _semantic_tuple(a[i + k]) == tb:
                found = ("A_AHEAD", k)
                break
        if found is None:
            deltas.append({
                "idx_a": i, "idx_b": j,
                "a": a[i].short(), "b": b[j].short(),
            })
            i += 1
            j += 1
        elif found[0] == "B_AHEAD":
            # B has extra URBs at j..j+k-1 that A doesn't have.
            for k in range(found[1]):
                deltas.append({
                    "idx_a": None, "idx_b": j + k,
                    "a": "(missing)", "b": b[j + k].short(),
                })
                if len(deltas) >= max_report:
                    break
            j += found[1]
        else:  # A_AHEAD
            for k in range(found[1]):
                deltas.append({
                    "idx_a": i + k, "idx_b": None,
                    "a": a[i + k].short(), "b": "(missing)",
                })
                if len(deltas) >= max_report:
                    break
            i += found[1]
    return {
        "a_total": len(a),
        "b_total": len(b),
        "consumed_a": i,
        "consumed_b": j,
        "deltas": deltas,
    }


# --------------------------------------------------------------------------
# Output helpers
# --------------------------------------------------------------------------


def _print_aggregate(label: str, agg: dict) -> None:
    print(f"=== {label} ===")
    print(f"  submits={agg['submits']} completes={agg['completes']}")
    print(f"  by_kind={agg['by_kind']}")
    print(f"  realtek_writes={agg['realtek_writes']} realtek_reads={agg['realtek_reads']}")
    print(f"  write_pages_top={agg['write_pages_top']}")
    print(f"  in_urbs_submitted={agg['in_urbs_submitted']}")
    print(f"  completion_status={agg['completion_status']}")


def _print_offload(label: str, probe: dict) -> None:
    print(f"=== offload-probe: {label} ===")
    print(f"  path-A LSSI window: {probe['lssi_window']}")
    print(f"  EP0 writes targeting that window: {probe['path_a_writes']}")
    print(f"  bulk-OUT submits total:           {probe['bulk_out_submits_total']}")
    print(f"  H2C-candidate bulk-OUTs (16..256B): {probe['h2c_candidates']}")
    print(f"  verdict: {probe['verdict']}")


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Binary-fidelity USB capture diff (Linux usbmon pcapng).",
    )
    p.add_argument("cap_a", type=Path, help="First capture (pcap or pcapng).")
    p.add_argument("cap_b", type=Path, nargs="?", default=None,
                   help="Second capture; required unless --offload-probe is set.")
    p.add_argument("--busnum", type=int, default=None,
                   help="Restrict to this USB bus.")
    p.add_argument("--devnum-a", type=int, default=None,
                   help="Restrict cap_a to this device address.")
    p.add_argument("--devnum-b", type=int, default=None,
                   help="Restrict cap_b to this device address.")
    p.add_argument("--offload-probe", action="store_true",
                   help="Classify cap_a as FW-offload vs per-write for path-A RF table.")
    p.add_argument("--phase-split", action="store_true",
                   help="Detect init/TX boundary and report per-phase aggregates.")
    p.add_argument("--aggregate", action="store_true",
                   help="Print histograms only, skip the URB-by-URB diff.")
    p.add_argument("--sentinel-reg", type=lambda s: int(s, 0), default=0x04FC,
                   help="Devourer sentinel register; default 0x04FC (REG_DUMMY).")
    p.add_argument("--max-report", type=int, default=50,
                   help="Max divergences to print in default diff mode.")
    args = p.parse_args(argv)

    if not args.cap_a.exists():
        print(f"capture not found: {args.cap_a}", file=sys.stderr)
        return 2

    urbs_a = read_urbs(args.cap_a, busnum=args.busnum, devnum=args.devnum_a)
    print(f"loaded {len(urbs_a)} URBs from {args.cap_a}")

    if args.offload_probe:
        probe = offload_probe(urbs_a)
        _print_offload(str(args.cap_a), probe)
        return 0

    if args.cap_b is None or not args.cap_b.exists():
        print("cap_b required unless --offload-probe is set", file=sys.stderr)
        return 2
    urbs_b = read_urbs(args.cap_b, busnum=args.busnum, devnum=args.devnum_b)
    print(f"loaded {len(urbs_b)} URBs from {args.cap_b}")

    if args.aggregate:
        _print_aggregate(str(args.cap_a), aggregate(urbs_a))
        _print_aggregate(str(args.cap_b), aggregate(urbs_b))
        return 0

    if args.phase_split:
        ba = find_phase_boundary(urbs_a, sentinel_reg=args.sentinel_reg)
        bb = find_phase_boundary(urbs_b, sentinel_reg=args.sentinel_reg)
        print(f"phase boundary: cap_a@{ba}/{len(urbs_a)}  cap_b@{bb}/{len(urbs_b)}")
        _print_aggregate(f"{args.cap_a} INIT", aggregate(urbs_a[:ba]))
        _print_aggregate(f"{args.cap_a} TX",   aggregate(urbs_a[ba:]))
        _print_aggregate(f"{args.cap_b} INIT", aggregate(urbs_b[:bb]))
        _print_aggregate(f"{args.cap_b} TX",   aggregate(urbs_b[bb:]))
        return 0

    # Default: full diff.
    result = diff(urbs_a, urbs_b, max_report=args.max_report)
    print(f"a_total={result['a_total']} b_total={result['b_total']} "
          f"consumed_a={result['consumed_a']} consumed_b={result['consumed_b']}")
    if not result["deltas"]:
        print("no divergence within first matched windows.")
    else:
        print(f"first {len(result['deltas'])} divergences:")
        for d in result["deltas"]:
            print(f"  [a@{d['idx_a']}] {d['a']}")
            print(f"  [b@{d['idx_b']}] {d['b']}")
            print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
