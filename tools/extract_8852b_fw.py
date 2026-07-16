#!/usr/bin/env python3
"""Extract the RTL8852B (Kestrel, Wi-Fi 6 / 802.11ax) WLAN firmware blobs from
the morrownr rtl8852bu vendor source into devourer's hal/ tree.

Mirror of tools/extract_8821c_fw.py (edit this generator, not the output),
with one Kestrel twist: the G6 trees ship one image per chip *cut*, selected
at runtime by the cut nibble at R_AX_SYS_CFG1[15:12] (mac_ax get_chip_info):
CBV (cut 1) loads the u2 image, CCV+ (cut >= 2) the u3 image — the
fwdl.h INTERNAL_FW_CONTENT_8852B_{CBV,CCV}_NICCE mapping. Both are extracted;
the Kestrel FW module picks by the cut it read at identity time.

Of the vendor categories (nic / nicce / nicce_bplus / wowlan...) devourer
takes NICCE only — the image the Linux CE driver itself loads
(rtl8852b_halinit.c: fw_type = RTW_FW_NIC_CE). The *_bplus images are the
RTL8852BP special case, not our silicon; wowlan is out of scope.

Input:  reference/rtl8852bu/phl/hal_g6/mac/fw_ax/rtl8852b/hal8852b_fw.c
Output: hal/hal8852b_fw.{c,h}                          (committed)

Consumed by src/kestrel (mac_ax fwdl path, milestone M1).
"""

from __future__ import annotations

import sys
from pathlib import Path

UPSTREAM_TAG = (
    "morrownr/rtl8852bu-20250826 (Realtek v1.19.21-86-g85f01e54fa.20250826)"
)
INPUT_FILE = "reference/rtl8852bu/phl/hal_g6/mac/fw_ax/rtl8852b/hal8852b_fw.c"
OUTPUT_C = "hal/hal8852b_fw.c"
OUTPUT_H = "hal/hal8852b_fw.h"
# vendor array -> (devourer array, comment)
ARRAYS = {
    "array_8852b_u2_nicce": "NICCE image for cut CBV (u2)",
    "array_8852b_u3_nicce": "NICCE image for cut CCV and later (u3)",
}


def extract_blocks(repo_root: Path) -> dict[str, list[str]]:
    path = repo_root / INPUT_FILE
    if not path.exists():
        sys.exit(
            f"missing input: {INPUT_FILE}\n"
            "  -> fetch the submodule first: git submodule update --init "
            "reference/rtl8852bu"
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
        "/* RTL8852B (Kestrel: RTL8852BU / RTL8832BU, Wi-Fi 6) WLAN firmware "
        "blobs.\n"
        f" * Extracted (NICCE images only) from {UPSTREAM_TAG}\n"
        " * phl/hal_g6/mac/fw_ax/rtl8852b/hal8852b_fw.c by "
        "tools/extract_8852b_fw.py.\n"
        " * Two images, selected at runtime by chip cut (CBV -> u2, CCV+ -> "
        "u3);\n"
        " * consumed by src/kestrel (mac_ax fwdl path). */\n"
    )
    c_parts = [header, '#include "drv_types.h"\n#include "hal8852b_fw.h"\n']
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
        "/* Auto-extracted RTL8852B NICCE firmware blobs (u2 = cut CBV, u3 = "
        "cut CCV+).\n * See hal8852b_fw.c. */\n"
        "#ifndef HAL8852B_FW_H\n"
        "#define HAL8852B_FW_H\n"
        '#include "drv_types.h"\n'
        "#ifdef __cplusplus\n"
        'extern "C" {\n'
        "#endif\n" + "".join(h_decls) + "#ifdef __cplusplus\n"
        "}\n"
        "#endif\n"
        "#endif /* HAL8852B_FW_H */\n"
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
