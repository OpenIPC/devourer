#!/usr/bin/env python3
"""End-to-end smoke test: pcapng → urbscript emitter → C replay (dry-run).

Synthesises a small pcapng with a mix of control writes, control reads, a
bulk OUT, and an interrupt IN. Runs tools/pcapng_to_urbscript.py to produce
a .urbs file, then runs build/usbmon_replay --dry-run on it and verifies
the emitted submit counts match the script.

Requires build/usbmon_replay to have been compiled
(cc -O2 -Wall -Wextra -o build/usbmon_replay tools/usbmon_replay.c).
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tests.test_usbmon_pcap_diff import (  # noqa: E402
    _build_pcap,
    _bulk_out,
    _ctrl_read,
    _ctrl_write,
    _interrupt_in,
)


def main() -> int:
    replay_bin = ROOT / "build" / "usbmon_replay"
    if not replay_bin.exists():
        print(f"FAIL: {replay_bin} not built", file=sys.stderr)
        print("  cc -O2 -Wall -Wextra -o build/usbmon_replay tools/usbmon_replay.c",
              file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        pcap = tmp / "in.pcap"
        urbs = tmp / "in.urbs"

        records = []
        # 3 control writes, 1 control read, 1 bulk OUT, 1 interrupt IN.
        records += list(_ctrl_write(0x0100, b"\x01", 1_000_000, 1))
        records += list(_ctrl_write(0x0102, b"\xab\xcd", 1_000_500, 2))
        records += list(_ctrl_write(0x0C90, b"\x55\xaa\x55\xaa", 1_001_000, 3))
        records += list(_ctrl_read(0x0F00, b"\x12\x34", 1_001_500, 4))
        records += list(_bulk_out(0x02, b"hello world " * 8, 1_002_000, 5))
        records += list(_interrupt_in(0x83, b"\xc2\xc2\xc2", 1_002_500, 6))
        pcap.write_bytes(_build_pcap(records))

        # Stage 1 — pcap → urbscript.
        r = subprocess.run(
            [sys.executable, str(ROOT / "tools" / "pcapng_to_urbscript.py"),
             str(pcap), "-o", str(urbs)],
            check=True, capture_output=True, text=True,
        )
        assert urbs.exists(), "urbscript not emitted"
        out = r.stdout + r.stderr
        # 6 submits total.
        assert "wrote 6 URB records" in out, f"unexpected stage-1 output:\n{out}"
        print(f"  stage 1 (pcap → urbs): ok")

        # Stage 2 — replay --dry-run.
        r = subprocess.run(
            [str(replay_bin), "--device", "/dev/null",
             "--urbs", str(urbs), "--dry-run", "-v"],
            check=True, capture_output=True, text=True,
        )
        out = r.stderr
        # Expect 6 submits, 6 ok, by kind: ctrl=4 bulk=1 intr=1, in=2 out=4
        assert "6 submits, ok=6" in out, f"summary missing or wrong:\n{out}"
        assert "ctrl=4 bulk=1 intr=1" in out, f"by-kind wrong:\n{out}"
        assert "in=2 out=4" in out, f"by-dir wrong:\n{out}"
        print(f"  stage 2 (urbs → replay --dry-run): ok")
        print("ALL OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
