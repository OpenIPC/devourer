#!/usr/bin/env python3
"""Extract the RTL8852C (Kestrel C8852C: RTL8852CU / RTL8832CU) WLAN firmware
blobs from the morrownr rtl8852cu vendor source into devourer's hal/ tree.

Mirror of tools/extract_8852b_fw.py (edit this generator, not the output). Two
Kestrel-C twists vs the 8852B:
  - the 8852C tree has no separate NIC-CE image; it ships plain NIC images
    (array_8852c_u{1,2}_nic) — that is what devourer loads (the mainline rtw89
    8852C driver loads the same rtw8852c_fw.bin family).
  - the cut selection is u1 (cut CAV / cut 1) vs u2 (cut CBV and later),
    per the fwdl.h INTERNAL_FW_CONTENT_8852C_{CAV,CBV} mapping. Both are
    extracted; the Kestrel FW module picks by the cut it read at identity time
    (the RTL8832CU on the bench is die 0x52 cut 1 -> u1).

Input:  reference/rtl8852cu/phl/hal_g6/mac/fw_ax/rtl8852c/hal8852c_fw.c
Output: hal/hal8852c_fw.{c,h}                          (committed)

Consumed by src/kestrel (mac_ax fwdl path, C8852C variant).
"""

from __future__ import annotations

import sys
from pathlib import Path

UPSTREAM_TAG = (
    "morrownr/rtl8852cu-20251113 (Realtek v1.19.22-103)"
)
INPUT_FILE = "reference/rtl8852cu/phl/hal_g6/mac/fw_ax/rtl8852c/hal8852c_fw.c"
OUTPUT_C = "hal/hal8852c_fw.c"
OUTPUT_H = "hal/hal8852c_fw.h"
# vendor array -> (devourer array, comment)
ARRAYS = {
    "array_8852c_u1_nic": "NIC image for cut CAV (u1)",
    "array_8852c_u2_nic": "NIC image for cut CBV and later (u2)",
}


def extract_blocks(repo_root: Path) -> dict[str, list[str]]:
    path = repo_root / INPUT_FILE
    if not path.exists():
        sys.exit(
            f"missing input: {INPUT_FILE}\n"
            "  -> fetch the submodule first: git submodule update --init "
            "reference/rtl8852cu"
        )
    blocks: dict[str, list[str]] = {}
    current: list[str] | None = None
    for line in path.read_text().splitlines():
        squeezed = line.replace(" ", "")
        if current is None:
            for name in ARRAYS:
                if squeezed.startswith(f"u8{name}[]={{"):
                    current = blocks.setdefault(name, [])
                    break
            continue
        if line.strip().startswith("};"):
            current = None
            continue
        if line.strip():
            current.append("\t" + line.strip())
    missing = [n for n in ARRAYS if n not in blocks or not blocks[n]]
    if missing:
        sys.exit(f"could not find array block(s) {missing} in {INPUT_FILE}")
    return blocks


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    blocks = extract_blocks(repo_root)

    header = (
        "/* RTL8852C (Kestrel C8852C: RTL8852CU / RTL8832CU, Wi-Fi 6) WLAN "
        "firmware blobs.\n"
        f" * Extracted (NIC images only) from {UPSTREAM_TAG}\n"
        " * phl/hal_g6/mac/fw_ax/rtl8852c/hal8852c_fw.c by "
        "tools/extract_8852c_fw.py.\n"
        " * Two images, selected at runtime by chip cut (CAV -> u1, CBV+ -> "
        "u2);\n"
        " * consumed by src/kestrel (mac_ax fwdl path, C8852C variant). */\n"
    )
    c_parts = [header, '#include "drv_types.h"\n#include "hal8852c_fw.h"\n']
    h_decls = []
    for name, why in ARRAYS.items():
        body = blocks[name]
        c_parts.append(
            f"\n/* {why} */\nconst u8 {name}[] = {{\n"
            + "\n".join(body)
            + "\n};\n"
            + f"const u32 {name}_len = sizeof({name});\n"
        )
        h_decls.append(f"extern const u8 {name}[];\nextern const u32 {name}_len;\n")
    h_text = (
        "/* Auto-extracted RTL8852C NIC firmware blobs (u1 = cut CAV, u2 = "
        "cut CBV+).\n * See hal8852c_fw.c. */\n"
        "#ifndef HAL8852C_FW_H\n"
        "#define HAL8852C_FW_H\n"
        '#include "drv_types.h"\n'
        "#ifdef __cplusplus\n"
        'extern "C" {\n'
        "#endif\n" + "".join(h_decls) + "#ifdef __cplusplus\n"
        "}\n"
        "#endif\n"
        "#endif /* HAL8852C_FW_H */\n"
    )

    (repo_root / OUTPUT_C).write_text("".join(c_parts))
    (repo_root / OUTPUT_H).write_text(h_text)
    for name in ARRAYS:
        nbytes = sum(ln.count("0x") for ln in blocks[name])
        print(f"  {name}: ~{nbytes} bytes ({len(blocks[name])} rows)")
    print(f"wrote {OUTPUT_C} and {OUTPUT_H}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
