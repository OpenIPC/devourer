#!/usr/bin/env python3
"""Hold src/mt76x0u/Mt76x0uUsbIds.h to the vendor table it was transcribed from.

The sibling of tests/mt7612u_usb_ids_vs_mt76.py, for the same reason: the C++
selftest proves the ids we list never collide with a Realtek one, but an id we
FORGOT is invisible to it - that adapter falls through to the Realtek path and
is misdetected. So this compares the header with mt76x0_device_table[] directly
and pins the SHA-256 of the vendor file.

Two things the MT7612U table does not have are checked too:
  * USB_DEVICE_AND_INTERFACE_INFO entries (MT7630U, MT7650U) count as ids;
  * an entry with `.driver_info = 1` (the Archer T1U no_2ghz quirk) must carry
    kFlagNo2Ghz in the header, and no other entry may.

Bench cell: SKIPs (exit 77) when the reference/mt76 submodule is absent.
"""

from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path

UPSTREAM = "openwrt/mt76 commit be5ce79"
VENDOR_REL = "reference/mt76/mt76x0/usb.c"
VENDOR_SHA256 = "045a5b45a2ae7c45098b9e6272c82cc5ac5c9867a6c80d9174decf00ea3deb2e"
HEADER_REL = "src/mt76x0u/Mt76x0uUsbIds.h"
SKIP = 77

# { USB_DEVICE(0x148F, 0x7610) },
# { USB_DEVICE(0x2357, 0x0105), .driver_info = 1, },
# { USB_DEVICE_AND_INTERFACE_INFO(0x0E8D, 0x7630, 0xff, 0x2, 0xff)},
VENDOR_ENTRY = re.compile(
    r"\{\s*USB_DEVICE(?:_AND_INTERFACE_INFO)?\(\s*0x([0-9a-fA-F]{4})\s*,"
    r"\s*0x([0-9a-fA-F]{4})[^)]*\)\s*(,\s*\.driver_info\s*=\s*(\d+))?"
)
# {0x148f, 0x7610, 0},  /  {0x2357, 0x0105, kFlagNo2Ghz},
HEADER_ENTRY = re.compile(
    r"\{\s*0x([0-9a-fA-F]{4})\s*,\s*0x([0-9a-fA-F]{4})\s*,\s*([^}]*)\}"
)


def vendor_ids(text: str) -> dict[tuple[int, int], int]:
    table = re.search(r"mt76x0_device_table\[\]\s*=\s*\{(.*?)\n\};", text, re.S)
    if not table:
        raise SystemExit(f"mt76x0_device_table[] not found in {VENDOR_REL}")
    out: dict[tuple[int, int], int] = {}
    for v, p, _, info in VENDOR_ENTRY.findall(table.group(1)):
        out[(int(v, 16), int(p, 16))] = int(info) if info else 0
    return out


def header_ids(text: str) -> list[tuple[tuple[int, int], str]]:
    table = re.search(r"kUsbIds\[\]\s*=\s*\{(.*?)\n\};", text, re.S)
    if not table:
        raise SystemExit(f"kUsbIds[] not found in {HEADER_REL}")
    return [
        ((int(v, 16), int(p, 16)), flags.strip())
        for v, p, flags in HEADER_ENTRY.findall(table.group(1))
    ]


def fmt(ids) -> str:
    return ", ".join(f"{v:04x}:{p:04x}" for v, p in ids)


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    vendor_path = root / VENDOR_REL
    if not vendor_path.exists():
        print(f"SKIP: missing {VENDOR_REL}; git submodule update --init reference/mt76")
        return SKIP

    raw = vendor_path.read_bytes()
    actual = hashlib.sha256(raw).hexdigest()
    if actual != VENDOR_SHA256:
        print(f"{VENDOR_REL} is not the pinned revision ({UPSTREAM}):\n"
              f"  sha256   {actual}\n  expected {VENDOR_SHA256}", file=sys.stderr)
        return 1

    vendor = vendor_ids(raw.decode("utf-8"))
    entries = header_ids((root / HEADER_REL).read_text(encoding="utf-8"))
    ours = [i for i, _ in entries]

    rc = 0
    dupes = sorted({i for i in ours if ours.count(i) > 1})
    if dupes:
        print(f"{HEADER_REL} lists id(s) twice: {fmt(dupes)}", file=sys.stderr)
        rc = 1
    missing = [i for i in vendor if i not in ours]
    extra = [i for i in ours if i not in vendor]
    if missing:
        print(f"{HEADER_REL} is MISSING id(s) the vendor driver claims: "
              f"{fmt(missing)}", file=sys.stderr)
        rc = 1
    if extra:
        print(f"{HEADER_REL} claims id(s) the vendor driver does not: "
              f"{fmt(extra)}", file=sys.stderr)
        rc = 1

    for ident, flags in entries:
        want = vendor.get(ident, 0) == 1
        have = "kFlagNo2Ghz" in flags
        if want != have:
            print(f"{fmt([ident])}: driver_info={vendor.get(ident, 0)} in the "
                  f"vendor table but kFlagNo2Ghz {'set' if have else 'unset'} "
                  f"in {HEADER_REL}", file=sys.stderr)
            rc = 1

    if rc == 0:
        print(f"mt76x0u_usb_ids_vs_mt76: {len(ours)} ids match {VENDOR_REL} "
              f"({UPSTREAM}), quirk flags included")
    return rc


if __name__ == "__main__":
    sys.exit(main())
