#!/usr/bin/env python3
"""Extract the pinned MT7612U MAC init-value table.

mt76 writes these as symbolic register names plus four FIELD_PREP macros, so a
transcription is a chance to typo a register address into a plausible-looking
one. This resolves the names against mt76x02_regs.h and evaluates the macros,
so the checked-in header is mechanical rather than hand-copied.

Unlike the Realtek extractors here, the source is C initialiser syntax rather
than a vendor parameter blob, so the symbol table and the small expression
evaluator below are the bulk of the work. Both are deliberately narrow: they
understand only the constructs these definitions actually use, and raise on
anything else instead of guessing.
"""

from __future__ import annotations

import argparse
import hashlib
import re
from pathlib import Path

UPSTREAM = "openwrt/mt76 commit be5ce79"
DEFAULT_ROOT = "reference/mt76"
SUBMODULE_HINT = (
    f"{DEFAULT_ROOT} is a pinned git submodule ({UPSTREAM}); fetch it with\n"
    f"  git submodule update --init {DEFAULT_ROOT}"
)
OUTPUT_H = "src/mt7612u/initvals.h"

SOURCES = {
    "init": (
        "mt76x2/init.c",
        "d356f6d90cb2171272a4885fb4d5f4e9a607f06ec084babb9c5025b8f3f83e77",
    ),
    "regs": (
        "mt76x02_regs.h",
        "a75b36645b29e69db0627d9a16f776cad0e4000b6944a31183ef644d0398aefb",
    ),
}

# The two arrays mt76_write_mac_initvals() writes, in the order it writes them.
ARRAYS = ("vals", "prot_vals")

# Expected shape of the result, so a silent parse regression cannot pass.
EXPECTED_COUNT = 60
EXPECTED_SHA256 = "ba381217626e876b27a2aa9150440ff7c6af86740a3c64afbb36e81347df3719"


# --- a very small C constant-expression evaluator -------------------------

_DEFINE = re.compile(r"^\s*#\s*define\s+(MT_[A-Za-z0-9_]+)\s+(.+?)\s*(?:/\*.*)?$")


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def load_symbols(regs_text: str) -> dict[str, str]:
    """Object-like MT_* defines only. Function-like ones (MT_BBP(x, y)) are
    not constants and are never referenced by the tables below."""
    symbols: dict[str, str] = {}
    text = regs_text.replace("\\\n", " ")
    for line in text.splitlines():
        match = _DEFINE.match(line)
        if not match:
            continue
        name, body = match.group(1), match.group(2).strip()
        if name.endswith("(") or "(" in line.split(name, 1)[0]:
            continue
        symbols[name] = body
    return symbols


def _ffs(mask: int) -> int:
    if mask == 0:
        raise SystemExit("FIELD_PREP with a zero mask")
    return (mask & -mask).bit_length() - 1


def _split_args(text: str) -> list[str]:
    args, depth, start = [], 0, 0
    for index, char in enumerate(text):
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
        elif char == "," and depth == 0:
            args.append(text[start:index])
            start = index + 1
    args.append(text[start:])
    return [arg.strip() for arg in args]


def _match_call(expr: str, name: str) -> tuple[str, int] | None:
    """If expr starts with name(...), return its argument text and the index
    just past the closing parenthesis."""
    if not expr.startswith(name + "("):
        return None
    depth, index = 0, len(name)
    while index < len(expr):
        if expr[index] == "(":
            depth += 1
        elif expr[index] == ")":
            depth -= 1
            if depth == 0:
                return expr[len(name) + 1:index], index + 1
        index += 1
    raise SystemExit(f"unbalanced parentheses in {expr!r}")


def evaluate(expr: str, symbols: dict[str, str], depth: int = 0) -> int:
    """Evaluate one C constant expression built from the constructs these
    tables use: integer literals, MT_* symbols, BIT, GENMASK, FIELD_PREP,
    parentheses and | + << & ~. Anything else is an error, not a guess."""
    if depth > 32:
        raise SystemExit(f"macro recursion too deep at {expr!r}")

    expr = _strip_comments(expr).strip()
    while expr.endswith(","):
        expr = expr[:-1].strip()

    # Rewrite the calls we understand into plain Python, innermost first.
    out, index = [], 0
    while index < len(expr):
        rest = expr[index:]
        for name in ("FIELD_PREP", "GENMASK", "BIT"):
            call = _match_call(rest, name)
            if call is None:
                continue
            args_text, consumed = call
            args = [evaluate(a, symbols, depth + 1) for a in _split_args(args_text)]
            if name == "BIT":
                value = 1 << args[0]
            elif name == "GENMASK":
                high, low = args
                value = ((1 << (high - low + 1)) - 1) << low
            else:
                mask, field = args
                value = (field << _ffs(mask)) & mask
            out.append(f"({value})")
            index += consumed
            break
        else:
            # Numbers first: "0x0400" would otherwise tokenise as 0 followed
            # by an identifier "x0400".
            number = re.match(r"0[xX][0-9a-fA-F]+[uUlL]*|\d+[uUlL]*", rest)
            if number:
                literal = number.group(0).rstrip("uUlL")
                out.append(f"({int(literal, 0)})")
                index += len(number.group(0))
                continue
            match = re.match(r"[A-Za-z_][A-Za-z0-9_]*", rest)
            if match:
                name = match.group(0)
                if name not in symbols:
                    raise SystemExit(f"unknown symbol {name!r} in {expr!r}")
                out.append(f"({evaluate(symbols[name], symbols, depth + 1)})")
                index += len(name)
            else:
                out.append(rest[0])
                index += 1

    python = "".join(out)
    python = re.sub(r"\b(0[xX][0-9a-fA-F]+|\d+)[uUlL]+\b", r"\1", python)
    if not re.fullmatch(r"[0-9xXa-fA-F()|+\-*<>&~^\s]*", python):
        raise SystemExit(f"refusing to evaluate {expr!r} -> {python!r}")
    try:
        return int(eval(python, {"__builtins__": {}}, {})) & 0xFFFFFFFF
    except Exception as exc:  # noqa: BLE001 - report the expression, not a trace
        raise SystemExit(f"could not evaluate {expr!r}: {exc}") from exc


# --- extraction -----------------------------------------------------------

def local_macros(function_text: str) -> dict[str, str]:
    """The DEFAULT_PROT_CFG_* macros are defined inside the function and are
    not visible in the register header."""
    macros = {}
    text = function_text.replace("\\\n", " ")
    for match in re.finditer(r"^\s*#\s*define\s+(DEFAULT_\w+)\s+(.+)$", text, re.M):
        macros[match.group(1)] = match.group(2).strip()
    return macros


def extract_array(function_text: str, name: str) -> list[tuple[str, str]]:
    match = re.search(
        rf"\b{name}\[\]\s*=\s*\{{(.*?)\n\t\}};", function_text, re.S
    )
    if not match:
        raise SystemExit(f"could not find {name}[] in mt76_write_mac_initvals()")
    body = _strip_comments(match.group(1))
    pairs = re.findall(r"\{\s*([^,]+?)\s*,\s*([^}]+?)\s*\}", body)
    if not pairs:
        raise SystemExit(f"{name}[] parsed to zero entries")
    return [(reg.strip(), val.strip()) for reg, val in pairs]


def load(source_root: Path) -> list[tuple[int, int, str]]:
    texts = {}
    for key, (relative, expected_hash) in SOURCES.items():
        path = source_root / relative
        if not path.exists():
            raise SystemExit(f"missing {path}\n{SUBMODULE_HINT}")
        source = path.read_bytes()
        actual_hash = hashlib.sha256(source).hexdigest()
        if actual_hash != expected_hash:
            raise SystemExit(
                f"unexpected {relative} SHA-256: {actual_hash}; "
                f"expected {expected_hash}"
            )
        texts[key] = source.decode("utf-8")

    function = re.search(
        r"void mt76_write_mac_initvals\(.*?\n\}", texts["init"], re.S
    )
    if not function:
        raise SystemExit("mt76_write_mac_initvals() not found in mt76x2/init.c")
    function_text = function.group(0)

    symbols = load_symbols(texts["regs"])
    symbols.update(local_macros(function_text))

    rows: list[tuple[int, int, str]] = []
    for array in ARRAYS:
        for reg_expr, val_expr in extract_array(function_text, array):
            reg = evaluate(reg_expr, symbols)
            val = evaluate(val_expr, symbols)
            # Keep the symbolic name as a comment when there is one, so the
            # generated file stays readable against the register header.
            label = reg_expr if reg_expr.startswith("MT_") else ""
            rows.append((reg, val, label))
    return rows


def table_hash(rows: list[tuple[int, int, str]]) -> str:
    digest = hashlib.sha256()
    for reg, val, _ in rows:
        digest.update(reg.to_bytes(4, "little"))
        digest.update(val.to_bytes(4, "little"))
    return digest.hexdigest()


def render_h(rows: list[tuple[int, int, str]]) -> str:
    lines = [
        "/* SPDX-License-Identifier: BSD-3-Clause-Clear */\n"
        "/* GENERATED - do not hand-edit.\n"
        f" * Source: {UPSTREAM}, mt76_write_mac_initvals() in mt76x2/init.c,\n"
        " * with register names and the DEFAULT_PROT_CFG_* macros resolved\n"
        " * against mt76x02_regs.h.\n"
        " * Regenerate:  tools/extract_mt7612u_tables.py\n"
        " * Verify:      tools/extract_mt7612u_tables.py --check\n"
        f" * Table SHA-256 (LE reg,val stream): {table_hash(rows)}\n"
        " */\n"
        "#ifndef MT7612U_INITVALS_H\n"
        "#define MT7612U_INITVALS_H\n"
        "\n"
        "static const struct { uint32_t reg; uint32_t val; } "
        "mt7612u_mac_initvals[] = {\n"
    ]
    for reg, val, label in rows:
        comment = f"  /* {label} */" if label else ""
        lines.append(f"\t{{ 0x{reg:04x}, 0x{val:08x} }},{comment}\n")
    lines.append("};\n#endif\n")
    return "".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    source_root = args.source_root or root / DEFAULT_ROOT
    rows = load(source_root)

    actual_hash = table_hash(rows)
    if len(rows) != EXPECTED_COUNT or actual_hash != EXPECTED_SHA256:
        raise SystemExit(
            f"unexpected table: count={len(rows)} sha256={actual_hash}; "
            f"expected count={EXPECTED_COUNT} sha256={EXPECTED_SHA256}"
        )

    output = render_h(rows)
    # Explicit encoding + newline="": the generated file is a byte-defined
    # artifact, so --check must compare identically on every platform.
    if args.check:
        path = root / OUTPUT_H
        # NOT Path.read_text(newline=""): that keyword only exists on Python
        # 3.13+, and this script has to run on whatever the CI image ships
        # (3.12 today). open() has taken `newline` since forever.
        if not path.exists():
            existing = None
        else:
            with open(path, encoding="utf-8", newline="") as fh:
                existing = fh.read()
        if existing != output:
            raise SystemExit(f"stale generated output: {OUTPUT_H}")
        verb = "verified"
    else:
        (root / OUTPUT_H).write_text(output, encoding="utf-8", newline="")
        verb = "wrote"
    print(f"{verb} {OUTPUT_H}: {len(rows)} register writes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
