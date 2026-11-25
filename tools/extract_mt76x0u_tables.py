#!/usr/bin/env python3
"""Generate src/mt76x0u/initvals.h from the pinned mt76x0 tables.

The MT76x0U sibling of tools/extract_mt7612u_tables.py, for the same reason:
mt76 writes these tables with symbolic register names and macros (MT_BBP(AGC,
4), MT_RF(0, 3), MT_BCN_OFFSET(1), RF_G_BAND | RF_BW_20), and transcribing ~800
entries by hand is a chance to typo an address into a plausible one. Every
expression here is evaluated against the pinned headers instead.

Sources (reference/mt76 @ be5ce79, each pinned by SHA-256):
  mt76x0/initvals.h, initvals_init.h, initvals_phy.h  - the tables
  mt76x0/phy.h                                          - RF_* band/width flags
  mt76x02_regs.h                                        - MAC/BBP register names

The evaluator understands integer literals, object-like defines, the three
function-like macros the tables use (MT_BBP, MT_RF, MT_BCN_OFFSET), and
| + << >> & ~ with parentheses. Anything else is an error, not a guess.

  tools/extract_mt76x0u_tables.py          regenerate
  tools/extract_mt76x0u_tables.py --check  verify the checked-in header
Exits 77 (ctest SKIP) when the reference/mt76 submodule is absent.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

UPSTREAM = "openwrt/mt76 commit be5ce79"
DEFAULT_ROOT = "reference/mt76"
OUTPUT_H = "src/mt76x0u/initvals.h"

SOURCES = {
    "initvals": "mt76x0/initvals.h",
    "initvals_init": "mt76x0/initvals_init.h",
    "initvals_phy": "mt76x0/initvals_phy.h",
    "phy": "mt76x0/phy.h",
    "regs": "mt76x02_regs.h",
}
SOURCE_SHA256 = {
    "initvals": "396bd681bc7f7d3a04f0a04aa38f9419c3483ef9fe7e075e78c4db799002e1fc",
    "initvals_init": "8c93c4d814f4b09102dc413fc882d923e96aca1dd1df397edf8aa9013c64c7b7",
    "initvals_phy": "5a1f239a47c22f2d5ac58c806fbf93d36a4f0467dea9d4faee6e9bab4be5d0f6",
    "phy": "78684d608144054cf1f2803a3db20845c465fe4a1164352481f23fa96bd5cdf7",
    "regs": "a75b36645b29e69db0627d9a16f776cad0e4000b6944a31183ef644d0398aefb",
}

# (source key, array name, kind). Kinds: pair = {reg, value};
# bbp = {bw_band, {reg, value}}; rf = {rf_bank_reg, bw_band, value};
# freq = struct mt76x0_freq_item (20 fields); u8 = flat byte list.
TABLES = [
    ("initvals_init", "common_mac_reg_table", "pair"),
    ("initvals_init", "mt76x0_mac_reg_table", "pair"),
    ("initvals_init", "mt76x0_bbp_init_tab", "pair"),
    ("initvals_init", "mt76x0_dcoc_tab", "pair"),
    ("initvals", "mt76x0_bbp_switch_tab", "bbp"),
    ("initvals_phy", "mt76x0_rf_central_tab", "pair"),
    ("initvals_phy", "mt76x0_rf_2g_channel_0_tab", "pair"),
    ("initvals_phy", "mt76x0_rf_5g_channel_0_tab", "pair"),
    ("initvals_phy", "mt76x0_rf_vga_channel_0_tab", "pair"),
    ("initvals_phy", "mt76x0_rf_bw_switch_tab", "rf"),
    ("initvals_phy", "mt76x0_rf_band_switch_tab", "rf"),
    ("initvals_phy", "mt76x0_frequency_plan", "freq"),
    ("initvals_phy", "mt76x0_sdm_frequency_plan", "freq"),
    ("initvals_phy", "mt76x0_sdm_channel", "u8"),
    ("initvals_phy", "mt76x0_rf_ext_pa_tab", "rf"),
]
# Row counts, so a parse regression cannot pass silently.
EXPECTED_COUNTS: dict[str, int] = {
    "common_mac_reg_table": 31,
    "mt76x0_mac_reg_table": 35,
    "mt76x0_bbp_init_tab": 58,
    "mt76x0_dcoc_tab": 9,
    "mt76x0_bbp_switch_tab": 48,
    "mt76x0_rf_central_tab": 44,
    "mt76x0_rf_2g_channel_0_tab": 68,
    "mt76x0_rf_5g_channel_0_tab": 38,
    "mt76x0_rf_vga_channel_0_tab": 35,
    "mt76x0_rf_bw_switch_tab": 41,
    "mt76x0_rf_band_switch_tab": 43,
    "mt76x0_frequency_plan": 125,
    "mt76x0_sdm_frequency_plan": 125,
    "mt76x0_sdm_channel": 26,
    "mt76x0_rf_ext_pa_tab": 44,
}

FREQ_FIELDS = 20


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def load_defines(text: str) -> tuple[dict[str, str], dict[str, tuple[list[str], str]]]:
    """Object-like and function-like #defines (single line after joining
    continuations)."""
    objects: dict[str, str] = {}
    functions: dict[str, tuple[list[str], str]] = {}
    text = strip_comments(text.replace("\\\n", " "))
    for line in text.splitlines():
        m = re.match(r"^\s*#\s*define\s+([A-Za-z_]\w*)(\(([^)]*)\))?\s+(.+?)\s*$", line)
        if not m:
            continue
        name, params, body = m.group(1), m.group(3), m.group(4)
        if m.group(2):
            functions[name] = ([p.strip() for p in params.split(",")], body)
        else:
            objects[name] = body
    return objects, functions


def split_args(text: str) -> list[str]:
    args, depth, start = [], 0, 0
    for i, c in enumerate(text):
        if c in "({":
            depth += 1
        elif c in ")}":
            depth -= 1
        elif c == "," and depth == 0:
            args.append(text[start:i])
            start = i + 1
    args.append(text[start:])
    return [a.strip() for a in args]


class Evaluator:
    def __init__(self, objects, functions):
        self.objects = objects
        self.functions = functions

    def expand(self, expr: str, depth: int = 0) -> str:
        if depth > 32:
            raise SystemExit(f"macro recursion too deep at {expr!r}")
        out, i = [], 0
        while i < len(expr):
            rest = expr[i:]
            num = re.match(r"0[xX][0-9a-fA-F]+[uUlL]*|\d+[uUlL]*", rest)
            if num:
                out.append(f"({int(num.group(0).rstrip('uUlL'), 0)})")
                i += len(num.group(0))
                continue
            ident = re.match(r"[A-Za-z_]\w*", rest)
            if ident:
                name = ident.group(0)
                after = rest[len(name):]
                if name in self.functions and after.lstrip().startswith("("):
                    params, body = self.functions[name]
                    open_at = len(name) + (len(after) - len(after.lstrip()))
                    depth_p, j = 0, open_at
                    while True:
                        if rest[j] == "(":
                            depth_p += 1
                        elif rest[j] == ")":
                            depth_p -= 1
                            if depth_p == 0:
                                break
                        j += 1
                    args = split_args(rest[open_at + 1:j])
                    if len(args) != len(params):
                        raise SystemExit(f"{name} takes {len(params)} args: {rest[:j+1]!r}")
                    sub = body
                    # Token pasting first (MT_BBP_##_type##_BASE), then plain
                    # parameter substitution.
                    for p, a in zip(params, args):
                        sub = re.sub(rf"##\s*{p}\s*##", a, sub)
                        sub = re.sub(rf"##\s*{p}\b", a, sub)
                        sub = re.sub(rf"\b{p}\s*##", a, sub)
                    for p, a in zip(params, args):
                        sub = re.sub(rf"\b{p}\b", f"({a})", sub)
                    out.append(f"({self.expand(sub, depth + 1)})")
                    i += j + 1
                    continue
                if name in self.objects:
                    out.append(f"({self.expand(self.objects[name], depth + 1)})")
                    i += len(name)
                    continue
                raise SystemExit(f"unknown symbol {name!r} in {expr!r}")
            out.append(rest[0])
            i += 1
        return "".join(out)

    def __call__(self, expr: str) -> int:
        py = self.expand(strip_comments(expr).strip())
        if not re.fullmatch(r"[0-9()|+\-<>&~\s]*", py):
            raise SystemExit(f"refusing to evaluate {expr!r} -> {py!r}")
        try:
            return int(eval(py, {"__builtins__": {}}, {})) & 0xFFFFFFFF
        except Exception as exc:  # noqa: BLE001
            raise SystemExit(f"could not evaluate {expr!r}: {exc}") from exc


def parse_braces(text: str):
    """C initialiser -> nested lists of expression strings."""
    text = text.strip()
    if not text.startswith("{"):
        return text
    depth, items, start = 0, [], 1
    for i, c in enumerate(text):
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                inner = text[1:i]
                return [parse_braces(a) for a in split_args(inner) if a.strip()]
    raise SystemExit("unbalanced braces")


def flatten(node) -> list[str]:
    if isinstance(node, str):
        return [node]
    out = []
    for n in node:
        out.extend(flatten(n))
    return out


def extract(text: str, name: str):
    m = re.search(rf"\b{name}\[\]\s*=\s*(\{{.*?\n\}});", strip_comments(text), re.S)
    if not m:
        raise SystemExit(f"{name}[] not found")
    return parse_braces(m.group(1))


def load(source_root: Path, check_hashes: bool):
    texts, hashes = {}, {}
    for key, rel in SOURCES.items():
        path = source_root / rel
        if not path.exists():
            print(f"missing {path}; git submodule update --init {DEFAULT_ROOT}",
                  file=sys.stderr)
            raise SystemExit(77)
        raw = path.read_bytes()
        hashes[key] = hashlib.sha256(raw).hexdigest()
        if check_hashes and SOURCE_SHA256[key] and hashes[key] != SOURCE_SHA256[key]:
            raise SystemExit(f"unexpected {rel} SHA-256 {hashes[key]}; expected "
                             f"{SOURCE_SHA256[key]} ({UPSTREAM})")
        texts[key] = raw.decode("utf-8")

    objects, functions = load_defines(texts["regs"])
    o2, f2 = load_defines(texts["phy"])
    objects.update(o2)
    functions.update(f2)
    ev = Evaluator(objects, functions)

    tables = []
    for key, name, kind in TABLES:
        entries = extract(texts[key], name)
        rows = []
        if kind == "u8":
            rows = [(ev(e), e) for e in entries]
        else:
            for entry in entries:
                fields = flatten(entry)
                want = {"pair": 2, "bbp": 3, "rf": 3, "freq": FREQ_FIELDS}[kind]
                if len(fields) != want:
                    raise SystemExit(f"{name}: entry with {len(fields)} fields, want {want}: {fields}")
                rows.append(([ev(f) for f in fields], fields))
        tables.append((name, kind, rows))
    return tables, hashes


def render(tables, hashes) -> str:
    out = [
        "/* SPDX-License-Identifier: GPL-2.0-only */\n"
        "/* GENERATED - do not hand-edit.\n"
        f" * Source: {UPSTREAM}, mt76x0/initvals.h, initvals_init.h and\n"
        " * initvals_phy.h, with register names, MT_BBP()/MT_RF()/MT_BCN_OFFSET()\n"
        " * and the RF_* flags evaluated against mt76x02_regs.h and mt76x0/phy.h.\n"
        " * Copyright as the source: (c) 2002-2010 Ralink Technology, Inc.;\n"
        " * (C) 2015 Jakub Kicinski; (C) 2018 Stanislaw Gruszka; (C) 2018 Lorenzo\n"
        " * Bianconi.\n"
        " * Regenerate:  tools/extract_mt76x0u_tables.py\n"
        " * Verify:      tools/extract_mt76x0u_tables.py --check\n"
        " * Source SHA-256:\n"
    ]
    for key, rel in SOURCES.items():
        out.append(f" *   {rel}  {hashes[key]}\n")
    out.append(
        " *\n"
        " * Include after internal.h (struct mt0_reg_pair) and phy.h (the item\n"
        " * structs). Array names are the kernel's, for grep.\n"
        " */\n"
        "#ifndef MT76X0U_INITVALS_H\n#define MT76X0U_INITVALS_H\n"
    )
    for name, kind, rows in tables:
        ctype = {"pair": "struct mt0_reg_pair", "bbp": "struct mt0_bbp_switch_item",
                 "rf": "struct mt0_rf_switch_item", "freq": "struct mt0_freq_item",
                 "u8": "uint8_t"}[kind]
        out.append(f"\nstatic const {ctype} {name}[] = {{\n")
        if kind == "u8":
            out.append("\t" + ", ".join(str(v) for v, _ in rows) + ",\n")
        for vals, src in ([] if kind == "u8" else rows):
            if kind == "pair":
                out.append(f"\t{{ 0x{vals[0]:08x}, 0x{vals[1]:08x} }},  /* {src[0]} */\n")
            elif kind == "bbp":
                out.append(f"\t{{ 0x{vals[0]:04x}, {{ 0x{vals[1]:08x}, 0x{vals[2]:08x} }} }},"
                           f"  /* {src[1]} */\n")
            elif kind == "rf":
                out.append(f"\t{{ 0x{vals[0]:08x}, 0x{vals[1]:04x}, 0x{vals[2]:02x} }},"
                           f"  /* {src[0]} */\n")
            else:
                out.append("\t{ " + ", ".join(f"0x{v:x}" for v in vals) + " },\n")
        out.append("};\n")
    out.append("\n#endif /* MT76X0U_INITVALS_H */\n")
    return "".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source-root", type=Path)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--print-pins", action="store_true",
                    help="print source hashes and row counts (to update the pins)")
    args = ap.parse_args()
    root = Path(__file__).resolve().parent.parent
    tables, hashes = load(args.source_root or root / DEFAULT_ROOT,
                          check_hashes=not args.print_pins)
    counts = {name: len(rows) for name, _, rows in tables}
    if args.print_pins:
        for k, h in hashes.items():
            print(f'    "{k}": "{h}",')
        for n, c in counts.items():
            print(f'    "{n}": {c},')
        return 0
    if any(not h for h in SOURCE_SHA256.values()) or not EXPECTED_COUNTS:
        raise SystemExit("pins not set; run with --print-pins and fill them in")
    if counts != EXPECTED_COUNTS:
        raise SystemExit(f"unexpected row counts: {counts}")

    output = render(tables, hashes)
    path = root / OUTPUT_H
    if args.check:
        existing = None
        if path.exists():
            with open(path, encoding="utf-8", newline="") as fh:
                existing = fh.read()
        if existing != output:
            raise SystemExit(f"stale generated output: {OUTPUT_H}")
        verb = "verified"
    else:
        with open(path, "w", encoding="utf-8", newline="") as fh:
            fh.write(output)
        verb = "wrote"
    print(f"{verb} {OUTPUT_H}: {sum(counts.values())} entries in {len(tables)} tables")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
