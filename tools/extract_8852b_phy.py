#!/usr/bin/env python3
"""Extract the RTL8852B halbb (BB) + halrf (RF) register tables from the
morrownr rtl8852bu vendor source into devourer's hal/ tree.

Mirror of tools/extract_8822c_phy_tables.py, but for the Kestrel G6 halbb/halrf
"raw_data" arrays (`const u32 array_mp_8852b_<name>[] = { addr, val, ... }`).
Both halbb and halrf use the same conditional encoding as phydm (IF/ELSE/END/
CHK opcodes in the address word's top nibble, plus a headline selector), walked
at runtime by src/kestrel/PhyTableLoaderKestrel — only the DATA is copied here.

Tables (the minimum for RX/TX bring-up):
  phy_reg        BB core registers          (halbb_hwimg_raw_data_8852b.h)
  phy_reg_gain   BB RX gain table           (halbb_hwimg_raw_data_8852b_gain.h)
  radioa/radiob  RF path-A/B registers      (halrf_hwimg_raw_data_8852b.h)

Input:  reference/rtl8852bu/phl/hal_g6/phy/{bb/halbb_8852b,rf/halrf_8852b}/...
Output: hal/hal8852b_phy.{c,h}   (committed)

Consumed by src/kestrel (PHY table apply, milestone M3).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

UPSTREAM_TAG = "morrownr/rtl8852bu-20250826 (halbb/halrf 8852B raw_data)"
BB = "reference/rtl8852bu/phl/hal_g6/phy/bb/halbb_8852b"
RF = "reference/rtl8852bu/phl/hal_g6/phy/rf/halrf_8852b"
# array name -> source file
TABLES = {
    "array_mp_8852b_phy_reg": f"{BB}/halbb_hwimg_raw_data_8852b.h",
    "array_mp_8852b_phy_reg_gain": f"{BB}/halbb_hwimg_raw_data_8852b.h",
    "array_mp_8852b_radioa": f"{RF}/halrf_hwimg_raw_data_8852b.h",
    "array_mp_8852b_radiob": f"{RF}/halrf_hwimg_raw_data_8852b.h",
}
OUT_C = "hal/hal8852b_phy.c"
OUT_H = "hal/hal8852b_phy.h"


def extract_array(text: str, name: str) -> list[str]:
    # const u32 <name>[] = { ... };  — capture the body rows.
    m = re.search(
        r"\bconst\s+u32\s+" + re.escape(name) + r"\s*\[\]\s*=\s*\{(.*?)\};",
        text,
        re.S,
    )
    if not m:
        sys.exit(f"could not find {name}[] in source")
    rows = []
    for line in m.group(1).splitlines():
        s = line.strip().rstrip(",")
        if s:
            rows.append("\t" + line.strip())
    return rows


def main() -> int:
    repo = Path(__file__).resolve().parent.parent
    file_cache: dict[str, str] = {}
    blocks: dict[str, list[str]] = {}
    for name, rel in TABLES.items():
        path = repo / rel
        if not path.exists():
            sys.exit(
                f"missing input: {rel}\n  -> git submodule update --init "
                "reference/rtl8852bu"
            )
        text = file_cache.setdefault(rel, path.read_text(errors="replace"))
        blocks[name] = extract_array(text, name)

    header = (
        "/* RTL8852B (Kestrel) halbb/halrf register tables — {addr, value} u32\n"
        f" * pairs with phydm-style IF/ELSE/END/CHK conditional opcodes in the\n"
        f" * address word's top nibble. Extracted from {UPSTREAM_TAG}\n"
        " * by tools/extract_8852b_phy.py. Walked by PhyTableLoaderKestrel. */\n"
    )
    c_parts = [header, '#include "drv_types.h"\n#include "hal8852b_phy.h"\n']
    h_decls = []
    for name, rows in blocks.items():
        c_parts.append(
            f"\nconst u32 {name}[] = {{\n" + "\n".join(rows) + "\n};\n"
            + f"const u32 {name}_len = sizeof({name}) / sizeof(u32);\n"
        )
        h_decls.append(
            f"extern const u32 {name}[];\nextern const u32 {name}_len;\n"
        )
    h_text = (
        "/* Auto-extracted RTL8852B halbb/halrf tables. See hal8852b_phy.c. */\n"
        "#ifndef HAL8852B_PHY_H\n#define HAL8852B_PHY_H\n"
        '#include "drv_types.h"\n#ifdef __cplusplus\nextern "C" {\n#endif\n'
        + "".join(h_decls)
        + "#ifdef __cplusplus\n}\n#endif\n#endif /* HAL8852B_PHY_H */\n"
    )

    (repo / OUT_C).write_text("".join(c_parts))
    (repo / OUT_H).write_text(h_text)
    for name, rows in blocks.items():
        print(f"  {name}: {len(rows)} rows (~{len(rows)//2} entries)")
    print(f"wrote {OUT_C} and {OUT_H}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
