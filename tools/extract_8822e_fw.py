#!/usr/bin/env python3
"""Extract the RTL8822E (Jaguar3 EU) NIC firmware blob from the upstream
OpenHD/rtl88x2eu source into devourer's hal/ tree, dropping the AP/SPIC/WoWLAN
images devourer doesn't use. The rtl8822e parts (RTL8812EU / RTL8822EU) share
the rtl8822c PHY generation but ship a different MAC/DLFW firmware; this is the
only mandatory chip-specific blob for EU support.

Mirror of how hal/hal8822c_fw.c was vendored. Edit this generator, not the
generated output.

Input:  reference/rtl88x2eu/hal/rtl8822e/hal8822e_fw.c   (vendored, gitignored)
Output: hal/hal8822e_fw.{c,h}                            (committed)

The blob is consumed by src/jaguar3/Halmac8822cFw via the shared DLFW state
machine — the 8822c and 8822e DLFW registers are byte-identical, so only the
firmware image differs.
"""

from __future__ import annotations

import sys
from pathlib import Path

UPSTREAM_TAG = "OpenHD/rtl88x2eu (Realtek FW v5.15.0.1-197)"
INPUT_FILE = "reference/rtl88x2eu/hal/rtl8822e/hal8822e_fw.c"
OUTPUT_C = "hal/hal8822e_fw.c"
OUTPUT_H = "hal/hal8822e_fw.h"
ARRAY = "array_mp_8822e_fw_nic"


def extract_block(repo_root: Path) -> list[str]:
    path = repo_root / INPUT_FILE
    if not path.exists():
        sys.exit(
            f"missing input: {INPUT_FILE}\n"
            "  -> vendor it first: git clone --depth 1 "
            "https://github.com/OpenHD/rtl88x2eu reference/rtl88x2eu"
        )
    # The NIC image is one of several arrays in the file (ap / nic / spic /
    # wowlan). Capture only the byte rows of array_mp_8822e_fw_nic[].
    start = f"{ARRAY}[]={{".replace(" ", "")
    body: list[str] = []
    in_array = False
    for line in path.read_text().splitlines():
        if not in_array:
            if start in line.replace(" ", ""):
                in_array = True
            continue
        if line.strip().startswith("};"):
            break
        if line.strip():
            body.append("\t" + line.strip())
    if not body:
        sys.exit(f"could not find {ARRAY}[] block in {INPUT_FILE}")
    return body


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    body = extract_block(repo_root)

    header = (
        "/* RTL8822E (Jaguar3 EU: RTL8812EU / RTL8822EU) WLAN NIC firmware blob.\n"
        f" * Extracted (NIC image only) from {UPSTREAM_TAG}\n"
        " * hal/rtl8822e/hal8822e_fw.c by tools/extract_8822e_fw.py.\n"
        " * Consumed by src/jaguar3/Halmac8822cFw (shared 8822c/8822e DLFW path). */\n"
    )
    c_text = (
        header
        + '#include "drv_types.h"\n'
        + '#include "hal8822e_fw.h"\n\n'
        + f"const u8 {ARRAY}[] = {{\n"
        + "\n".join(body)
        + "\n};\n"
        + f"const u32 {ARRAY}_len = sizeof({ARRAY});\n"
    )
    h_text = (
        "/* Auto-extracted RTL8822E NIC firmware blob. See hal8822e_fw.c. */\n"
        "#ifndef HAL8822E_FW_H\n"
        "#define HAL8822E_FW_H\n"
        '#include "drv_types.h"\n'
        "#ifdef __cplusplus\n"
        'extern "C" {\n'
        "#endif\n"
        f"extern const u8 {ARRAY}[];\n"
        f"extern const u32 {ARRAY}_len;\n"
        "#ifdef __cplusplus\n"
        "}\n"
        "#endif\n"
        "#endif /* HAL8822E_FW_H */\n"
    )

    (repo_root / OUTPUT_C).write_text(c_text)
    (repo_root / OUTPUT_H).write_text(h_text)
    # rough byte count (8 hex bytes per row, minus trailing commas variance)
    nbytes = sum(ln.count("0x") for ln in body)
    print(f"wrote {OUTPUT_C} ({len(body)} rows, ~{nbytes} bytes) and {OUTPUT_H}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
