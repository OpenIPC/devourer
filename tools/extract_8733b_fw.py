#!/usr/bin/env python3
"""Extract the RTL8733B WLAN NIC firmware blobs from libc0607/rtl8733bu.

Only the normal and cut-2 (CCV) NIC images are retained. WoWLAN images and the
rest of the out-of-tree kernel driver are not part of devourer's bring-up.

The default input path mirrors the other firmware extractors. During initial
bring-up, --input can point at a separately cloned source tree.
"""

from __future__ import annotations

import argparse
import hashlib
import re
from pathlib import Path

UPSTREAM = "libc0607/rtl8733bu-20230626 commit 9e5f684"
DEFAULT_INPUT = "reference/rtl8733bu-20230626/hal/rtl8733b/hal8733b_fw.c"
OUTPUT_C = "hal/hal8733b_fw.c"
OUTPUT_H = "hal/hal8733b_fw.h"
ARRAYS = ("array_mp_8733b_fw_nic", "ccv_array_mp_8733b_fw_nic")
SOURCE_SHA256 = "6cb766e5271ce941ed1b2b7a4c35404f282152554f9a24d80cf38f9958325744"
EXPECTED = {
    "array_mp_8733b_fw_nic": (
        122008,
        "bf377a5c1ba09b458bd81514f46376441ab8524241cba77cac318b50dbee03c0",
    ),
    "ccv_array_mp_8733b_fw_nic": (
        122384,
        "f4ea32a4495f8384731688fd9f2fac18642830747f25dbe9d7dd8764a905cd63",
    ),
}


def extract(text: str, name: str) -> tuple[list[str], bytes]:
    match = re.search(rf"(?m)^u8\s+{name}\[\]\s*=\s*\{{", text)
    if not match:
        raise SystemExit(f"could not find {name}[]")
    end = text.find("};", match.end())
    if end < 0:
        raise SystemExit(f"unterminated {name}[]")
    body = text[match.end() : end]
    rows = [
        "\t" + line.strip()
        for line in body.splitlines()
        if line.strip()
    ]
    if not rows:
        raise SystemExit(f"empty {name}[]")
    blob = bytes(int(value, 16) for value in re.findall(r"0x([0-9a-fA-F]{1,2})", body))
    expected_size, expected_hash = EXPECTED[name]
    actual_hash = hashlib.sha256(blob).hexdigest()
    if len(blob) != expected_size or actual_hash != expected_hash:
        raise SystemExit(
            f"unexpected {name}: size={len(blob)} sha256={actual_hash}; "
            f"expected size={expected_size} sha256={expected_hash}"
        )
    return rows, blob


def render_c(blocks: dict[str, tuple[list[str], bytes]]) -> str:
    banner = (
        "/* RTL8733B (RTL8731BU / RTL8733BU) WLAN NIC firmware blobs.\n"
        f" * Extracted from {UPSTREAM} by tools/extract_8733b_fw.py.\n"
        f" * Source file SHA-256: {SOURCE_SHA256}.\n"
        " * The source file carries a GPL-2.0 notice; its repository also has\n"
        " * a top-level GPL-3.0 license file. Preserve both when auditing use. */\n"
    )
    c_parts = [banner, '#include "drv_types.h"\n#include "hal8733b_fw.h"\n\n']
    for name in ARRAYS:
        rows, blob = blocks[name]
        c_parts += [
            f"/* {len(blob)} bytes; SHA-256 {hashlib.sha256(blob).hexdigest()} */\n",
            f"const u8 {name}[] = {{\n",
            "\n".join(rows),
            "\n};\n",
            f"const u32 {name}_len = sizeof({name});\n\n",
        ]
    return "".join(c_parts).rstrip() + "\n"


def render_h() -> str:
    declarations = "".join(
        f"extern const u8 {name}[];\nextern const u32 {name}_len;\n"
        for name in ARRAYS
    )
    return (
        "/* Auto-extracted RTL8733B NIC firmware. See hal8733b_fw.c. */\n"
        "#ifndef HAL8733B_FW_H\n#define HAL8733B_FW_H\n"
        '#include "drv_types.h"\n#ifdef __cplusplus\nextern "C" {\n#endif\n'
        + declarations
        + '#ifdef __cplusplus\n}\n#endif\n#endif /* HAL8733B_FW_H */\n'
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path)
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify that checked-in outputs exactly match the pinned input",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    source = args.input or root / DEFAULT_INPUT
    if not source.exists():
        raise SystemExit(
            f"missing input: {source}\n"
            f"reference/rtl8733bu-20230626 is a pinned git submodule "
            f"({UPSTREAM}); fetch it with\n"
            "  git submodule update --init reference/rtl8733bu-20230626\n"
            "or pass --input"
        )
    source_bytes = source.read_bytes()
    source_hash = hashlib.sha256(source_bytes).hexdigest()
    if source_hash != SOURCE_SHA256:
        raise SystemExit(
            f"unexpected source SHA-256: {source_hash}; expected {SOURCE_SHA256}"
        )
    blocks = {
        name: extract(source_bytes.decode("utf-8"), name) for name in ARRAYS
    }
    outputs = {OUTPUT_C: render_c(blocks), OUTPUT_H: render_h()}
    # Explicit encoding + newline="": the generated files are byte-defined
    # artifacts, so --check must compare bytes identically on every platform.
    # Without newline="", Windows would write CRLF on generate and translate it
    # back on read, making the round-trip pass while the checked-in file differs.
    if args.check:
        stale = [
            path
            for path, expected in outputs.items()
            if not (root / path).exists()
            or (root / path).read_text(encoding="utf-8", newline="") != expected
        ]
        if stale:
            raise SystemExit("stale generated output: " + ", ".join(stale))
        verb = "verified"
    else:
        for path, output in outputs.items():
            (root / path).write_text(output, encoding="utf-8", newline="")
        verb = "wrote"
    sizes = ", ".join(f"{name}={len(blob)}" for name, (_, blob) in blocks.items())
    print(f"{verb} {OUTPUT_C} and {OUTPUT_H}: {sizes}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
