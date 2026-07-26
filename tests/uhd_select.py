#!/usr/bin/env python3
"""One place that decides WHICH UHD radio a test harness opens.

A bench with two B210-class radios (they share USB id 2500:0020, so lsusb
cannot tell them apart) makes `MultiUSRP("")` a coin flip: half your runs
measure the wrong antenna and nothing in the log says so. Every tool here
routes its device-args through `device_args()`, which resolves in order:

  1. an explicit `--args` on the command line;
  2. the DEVOURER_UHD_ARGS environment variable;
  3. `tests/.uhd_args`, an untracked one-line file — the reliable one on this
     suite, because nearly every harness invokes its SDR tool through `sudo`,
     which scrubs the environment: an exported variable does NOT survive
     `sudo python3 tests/sdr_duty.py`, but a file on disk does;
  4. nothing — allowed only when exactly one device is present. With more than
     one it is an error listing the serials, not a silent pick.

Set it once per bench:

    echo serial=XXXXXXX > tests/.uhd_args

`uhd_find_devices` prints the serials. A tool that genuinely wants to address
several radios at once passes `--args` explicitly and bypasses the guard.
"""
from __future__ import annotations

import os
import sys

ENV_VAR = "DEVOURER_UHD_ARGS"
ARGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".uhd_args")


def _from_file() -> str:
    try:
        with open(ARGS_FILE) as fh:
            for line in fh:
                line = line.split("#", 1)[0].strip()
                if line:
                    return line
    except OSError:
        pass
    return ""


def _find(args: str = ""):
    """Enumerated UHD devices, or None when the bindings can't be imported."""
    try:
        import uhd  # noqa: F401  (import cost is paid once, at call time)

        return list(uhd.find(args))
    except Exception:
        return None


def device_args(cli: str | None = None, *, allow_ambiguous: bool = False) -> str:
    """Resolve the UHD device-args string a tool should open with."""
    if cli:
        return cli
    env = os.environ.get(ENV_VAR, "").strip()
    if env:
        return env
    from_file = _from_file()
    if from_file:
        return from_file
    if allow_ambiguous:
        return ""
    found = _find()
    if found is None or len(found) <= 1:
        return ""  # single radio (or no bindings): nothing to disambiguate
    lines = []
    for d in found:
        # str(device_addr) is a "Device Address:\n    key: value\n..." block
        keys = {}
        for row in str(d).splitlines():
            if ":" in row and not row.startswith("Device"):
                k, _, v = row.partition(":")
                keys[k.strip()] = v.strip()
        lines.append(
            "    serial={:<10} name={}".format(
                keys.get("serial", "?"), keys.get("name", keys.get("product", "?"))
            )
        )
    sys.exit(
        "error: {} UHD devices present and no device selected.\n".format(len(found))
        + "\n".join(lines)
        + "\n  Pick one with --args serial=..., or (survives sudo):"
        + "\n    echo serial=... > {}".format(ARGS_FILE)
    )


def add_args_argument(ap, help_extra: str = "") -> None:
    """Register the standard --args option, defaulting from the environment."""
    ap.add_argument(
        "--args",
        default=os.environ.get(ENV_VAR, ""),
        help="UHD device args, e.g. serial=XXXXXXX (default: ${}){}".format(
            ENV_VAR, help_extra
        ),
    )


if __name__ == "__main__":  # tiny CLI: print what would be opened
    print(device_args(sys.argv[1] if len(sys.argv) > 1 else None) or "<any>")
