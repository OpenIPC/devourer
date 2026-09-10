#!/usr/bin/env python3
"""Hold src/mt7612u/Mt7612uUsbIds.h to the vendor table it was transcribed from.

The header claims to be "the complete mt76x2u_device_table, transcribed from
reference/mt76 @ be5ce79". That claim is load-bearing in a way the C++ cell
cannot see: Mt7612uUsbIdsSelftest proves the ids we DO list never collide with
a Realtek one, but an id we forgot is invisible to it — the device simply falls
through to the Realtek path and is misdetected as a Jaguar1, which is the exact
failure the gate exists to prevent. The first draft of that header had 11 of
the 16 entries, taken from the host's kernel tree rather than from the pinned
reference, and nothing caught it.

So this compares the two tables directly, and pins the SHA-256 of the vendor
file the same way tools/extract_mt7612u_tables.py pins its sources — otherwise
"matches the vendor table" would only mean "matches whatever happens to be
checked out".

Bench cell, not a CI cell: reference/ is a git submodule and CI checks the repo
out without submodules, so this SKIPS (exit 77) there. It runs for anyone who
has fetched the reference tree, which is everyone who can regenerate initvals.h.
"""

from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path

UPSTREAM = "openwrt/mt76 commit be5ce79"
VENDOR_REL = "reference/mt76/mt76x2/usb.c"
VENDOR_SHA256 = "6e6292552a4b3a8f9a9e773d226c0255042d84203ff19e8621fb7650e60c02fd"
HEADER_REL = "src/mt7612u/Mt7612uUsbIds.h"

SUBMODULE_HINT = (
    f"reference/mt76 is a pinned git submodule ({UPSTREAM}); fetch it with\n"
    f"  git submodule update --init reference/mt76"
)

SKIP = 77

# { USB_DEVICE(0x0b05, 0x1833) },  /* Asus USB-AC54 */
VENDOR_ENTRY = re.compile(
    r"USB_DEVICE\(\s*0x([0-9a-fA-F]{4})\s*,\s*0x([0-9a-fA-F]{4})\s*\)"
)
# {0x0b05, 0x1833}, /* Asus USB-AC54 */
HEADER_ENTRY = re.compile(
    r"\{\s*0x([0-9a-fA-F]{4})\s*,\s*0x([0-9a-fA-F]{4})\s*\}"
)


def vendor_ids(text: str) -> list[tuple[int, int]]:
    """The ids inside mt76x2u_device_table[] only. Taking every USB_DEVICE() in
    the file would silently absorb a second table if one is ever added."""
    table = re.search(
        r"mt76x2u_device_table\[\]\s*=\s*\{(.*?)\n\};", text, re.S
    )
    if not table:
        raise SystemExit(f"mt76x2u_device_table[] not found in {VENDOR_REL}")
    return [
        (int(v, 16), int(p, 16)) for v, p in VENDOR_ENTRY.findall(table.group(1))
    ]


def header_ids(text: str) -> list[tuple[int, int]]:
    table = re.search(r"kUsbIds\[\]\s*=\s*\{(.*?)\n\};", text, re.S)
    if not table:
        raise SystemExit(f"kUsbIds[] not found in {HEADER_REL}")
    return [
        (int(v, 16), int(p, 16)) for v, p in HEADER_ENTRY.findall(table.group(1))
    ]


def fmt(ids) -> str:
    return ", ".join(f"{v:04x}:{p:04x}" for v, p in ids)


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    vendor_path = root / VENDOR_REL
    header_path = root / HEADER_REL

    if not vendor_path.exists():
        print(f"SKIP: missing {VENDOR_REL}\n{SUBMODULE_HINT}")
        return SKIP

    raw = vendor_path.read_bytes()
    actual = hashlib.sha256(raw).hexdigest()
    if actual != VENDOR_SHA256:
        print(
            f"{VENDOR_REL} is not the pinned revision:\n"
            f"  sha256   {actual}\n"
            f"  expected {VENDOR_SHA256} ({UPSTREAM})\n"
            f"The id comparison below would be against a different tree than "
            f"{HEADER_REL} names. Re-verify the table against the new "
            f"revision, then update VENDOR_SHA256 here and the provenance "
            f"comment in the header.",
            file=sys.stderr,
        )
        return 1

    vendor = vendor_ids(raw.decode("utf-8"))
    ours = header_ids(header_path.read_text(encoding="utf-8"))

    # Before the set comparison, which is blind to multiplicity: a duplicated
    # entry would otherwise pass, and be reported as "17 ids match" a 16-entry
    # table. Harmless to is_usb_id(), but this cell is the thing that says the
    # table is a faithful transcription, so it has to notice.
    seen: set[tuple[int, int]] = set()
    dupes: list[tuple[int, int]] = []
    for i in ours:
        if i in seen:
            dupes.append(i)
        seen.add(i)
    if dupes:
        print(
            f"{HEADER_REL} lists {len(dupes)} id(s) twice: {fmt(dupes)}",
            file=sys.stderr,
        )
        return 1

    missing = [i for i in vendor if i not in ours]
    extra = [i for i in ours if i not in vendor]

    if missing:
        print(
            f"{HEADER_REL} is MISSING {len(missing)} id(s) the vendor driver "
            f"claims: {fmt(missing)}\n"
            f"Each is an MT7662-MAC adapter that would fall through to the "
            f"Realtek path and be misdetected.",
            file=sys.stderr,
        )
    if extra:
        print(
            f"{HEADER_REL} claims {len(extra)} id(s) the vendor driver does "
            f"not: {fmt(extra)}\n"
            f"The gate runs BEFORE the Realtek SYS_CFG2 read, so an id we "
            f"wrongly own is refused outright with no second chance.",
            file=sys.stderr,
        )
    if missing or extra:
        return 1

    note = ""
    if vendor != ours:
        # Same set, different order. Not a defect — the header is read by
        # humans next to the vendor file, so say it and carry on.
        note = "  (same ids, different order to the vendor file)\n"

    print(
        f"mt7612u_usb_ids_vs_mt76: {len(vendor)} ids match {VENDOR_REL} "
        f"at {UPSTREAM}\n{note}"
        f"  covers only which backend gets to LOOK at a device; the library's "
        f"MT_ASIC_VERSION identify stays authoritative after the handle opens",
        end="\n",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
