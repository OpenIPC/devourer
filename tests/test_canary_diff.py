"""Unit tests for tests/canary_diff.py.

Runs on any hosted CI runner — no USB hardware, no VM. Exercises the
parser, mask logic, channel-aware capture-state filter, and exit-code
contract. Hardware-side canary capture stays in regress.py / the
self-hosted rig.
"""

from __future__ import annotations

import sys
import subprocess
import textwrap
from pathlib import Path

HERE = Path(__file__).parent
SCRIPT = HERE / "canary_diff.py"


def run_diff(kernel: str, devourer: str, *extra_args: str,
             tmp_path: Path) -> subprocess.CompletedProcess:
    kf = tmp_path / "kernel.canary"
    df = tmp_path / "dev.canary"
    kf.write_text(kernel)
    df.write_text(devourer)
    return subprocess.run(
        [sys.executable, str(SCRIPT), str(kf), str(df), *extra_args],
        capture_output=True, text=True,
    )


CANARY_HEADER = "=== DEVOURER_DUMP_CANARY (post channel-set ch=6) ==="
CANARY_FOOTER = "=== END DEVOURER_DUMP_CANARY ==="


def wrap(body: str) -> str:
    return f"{CANARY_HEADER}\n{textwrap.dedent(body).strip()}\n{CANARY_FOOTER}\n"


def test_clean_diff_exits_zero(tmp_path: Path) -> None:
    canary = wrap("""
        BB 0x808 = 0x3E028233
        MAC 0x040 = 0x000C0000
        RF[A] 0x00 = 0x33EA9
    """)
    res = run_diff(canary, canary, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_real_divergence_exits_one(tmp_path: Path) -> None:
    kernel = wrap("BB 0x808 = 0x3E028233")
    devourer = wrap("BB 0x808 = 0x3E028299")
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 1, res.stdout + res.stderr
    assert "BB 0x808" in res.stdout
    assert "FAIL: 1 real divergence" in res.stdout


def test_ephemeral_mac_counter_masked(tmp_path: Path) -> None:
    """MAC 0x550 is a beacon-window counter; differences should be
    masked out and exit 0."""
    kernel = wrap("MAC 0x550 = 0x00001019")
    devourer = wrap("MAC 0x550 = 0x00001010")
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_ephemeral_rf_thermal_masked(tmp_path: Path) -> None:
    """RF[A] 0x42 is the thermal-meter sample register."""
    kernel = wrap("RF[A] 0x42 = 0x0B160")
    devourer = wrap("RF[A] 0x42 = 0x098F8")
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_bb_swing_thermal_bits_masked(tmp_path: Path) -> None:
    """BB 0xc1c bits 31:21 are thermal-tracked; the rest (e.g. AGC
    table select bits 11:8) should still diff. Kernel 0x23E in
    high bits = 0x47C00003, devourer 0x200 in high bits =
    0x40000003 — same low bits, diff masked → clean."""
    kernel = wrap("BB 0xc1c = 0x47C00003")
    devourer = wrap("BB 0xc1c = 0x40000003")
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_bb_swing_lower_bits_still_diffed(tmp_path: Path) -> None:
    """BB 0xc1c bits 0-20 are NOT masked — a real divergence
    in the AGC-table-select bits should fail the diff."""
    # Same upper bits, different lower bits.
    kernel = wrap("BB 0xc1c = 0x47C00003")
    devourer = wrap("BB 0xc1c = 0x47C00103")  # bit 8 differs
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 1, res.stdout + res.stderr


def test_strict_disables_masking(tmp_path: Path) -> None:
    kernel = wrap("MAC 0x550 = 0x00001019")
    devourer = wrap("MAC 0x550 = 0x00001010")
    res = run_diff(kernel, devourer, "--strict", tmp_path=tmp_path)
    assert res.returncode == 1, res.stdout + res.stderr


def test_5g_capture_state_artifact_masked(tmp_path: Path) -> None:
    """BB 0xc20 is CCK-only — never written at 5G by either side, but
    kernel iface (long-lived) retains a 2.4G value while devourer
    starts fresh. At ch100 the diff should be masked."""
    kernel = wrap("BB 0xc20 = 0x2F2F2F2F")
    devourer = wrap("BB 0xc20 = 0x12121212")
    res = run_diff(kernel, devourer, "--channel", "100", tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr
    # The masked-artifact note should mention 0xc20.
    assert "0xc20" in res.stdout


def test_5g_capture_state_artifact_NOT_masked_at_2g(tmp_path: Path) -> None:
    """Same divergence at ch6 is treated as a real difference (CCK is
    active at 2.4G; if it diverges there it's a real init drift)."""
    kernel = wrap("BB 0xc20 = 0x2F2F2F2F")
    devourer = wrap("BB 0xc20 = 0x12121212")
    res = run_diff(kernel, devourer, "--channel", "6", tmp_path=tmp_path)
    assert res.returncode == 1, res.stdout + res.stderr


def test_register_set_mismatch_exits_two(tmp_path: Path) -> None:
    kernel = wrap("""
        BB 0x808 = 0x3E028233
        BB 0x80c = 0x12131113
    """)
    devourer = wrap("BB 0x808 = 0x3E028233")  # 0x80c missing
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 2, res.stdout + res.stderr
    assert "BB 0x80c" in res.stderr


def test_empty_file_exits_two(tmp_path: Path) -> None:
    res = run_diff("", wrap("BB 0x808 = 0x3E028233"), tmp_path=tmp_path)
    assert res.returncode == 2
    assert "no canary readings parsed" in res.stderr


def test_lines_outside_envelope_ignored(tmp_path: Path) -> None:
    """Devourer logs `<devourer>...` lines that the awk extractor
    strips. If extra noise leaks in via a different path, the parser
    should still only consider the envelope."""
    kernel = wrap("BB 0x808 = 0x3E028233")
    # Extra junk before + after the envelope.
    devourer = (
        "spurious log line that should be ignored\n"
        "BB 0x808 = 0xDEADBEEF\n"      # outside envelope — ignored
        "MAC 0x040 = 0xCAFEBABE\n"     # outside envelope — ignored
        + wrap("BB 0x808 = 0x3E028233")
        + "trailing junk\n"
    )
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_show_clean_emits_clean_line(tmp_path: Path) -> None:
    canary = wrap("BB 0x808 = 0x3E028233")
    res = run_diff(canary, canary, "--show-clean", tmp_path=tmp_path)
    assert res.returncode == 0
    assert "CLEAN" in res.stdout
