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
    """Kernel-style capture: bare `KIND 0xADDR = 0xVALUE` lines."""
    return f"{CANARY_HEADER}\n{textwrap.dedent(body).strip()}\n{CANARY_FOOTER}\n"


def dwrap(body: str) -> str:
    """Devourer-style capture: raw stderr with the `devourer [I] `
    human-diagnostic prefix on every line (dump + envelope markers)."""
    lines = [CANARY_HEADER, *textwrap.dedent(body).strip().splitlines(),
             CANARY_FOOTER]
    return "".join(f"devourer [I] {l.strip()}\n" for l in lines)


def test_clean_diff_exits_zero(tmp_path: Path) -> None:
    body = """
        BB 0x808 = 0x3E028233
        MAC 0x040 = 0x000C0000
        RF[A] 0x00 = 0x33EA9
    """
    res = run_diff(wrap(body), dwrap(body), tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_real_divergence_exits_one(tmp_path: Path) -> None:
    kernel = wrap("BB 0x808 = 0x3E028233")
    devourer = dwrap("BB 0x808 = 0x3E028299")
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 1, res.stdout + res.stderr
    assert "BB 0x808" in res.stdout
    assert "FAIL: 1 real divergence" in res.stdout


def test_ephemeral_mac_counter_masked(tmp_path: Path) -> None:
    """MAC 0x550 is a beacon-window counter; differences should be
    masked out and exit 0."""
    kernel = wrap("MAC 0x550 = 0x00001019")
    devourer = dwrap("MAC 0x550 = 0x00001010")
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_ephemeral_rf_thermal_masked(tmp_path: Path) -> None:
    """RF[A] 0x42 is the thermal-meter sample register."""
    kernel = wrap("RF[A] 0x42 = 0x0B160")
    devourer = dwrap("RF[A] 0x42 = 0x098F8")
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


def test_dig_igi_byte0_masked(tmp_path: Path) -> None:
    """BB 0xc50 bits 7:0 are the DIG IGI — kernel's phydm walks
    them; devourer writes the floor once. Diffs in byte 0 must
    be masked, but upper bytes still diff."""
    kernel = wrap("BB 0xc50 = 0x69b80022")
    devourer = wrap("BB 0xc50 = 0x69b8001c")  # only byte 0 differs
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_dig_igi_upper_bits_still_diffed(tmp_path: Path) -> None:
    """BB 0xc50 bits 31:8 are static AGC config — a real divergence
    there should still fail the diff."""
    kernel = wrap("BB 0xc50 = 0x69b8001c")
    devourer = wrap("BB 0xc50 = 0x69b8011c")  # bit 8 differs
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 1, res.stdout + res.stderr


def test_iqk_output_regs_masked(tmp_path: Path) -> None:
    """IQK output coefficients vary run-to-run; the canonical
    set (0x8b0, 0xc10/0xc14/0xc90/0xc94, path-B mirrors) is
    masked entirely."""
    body = "\n".join(
        f"BB 0x{addr:x} = 0xAAAAAAAA" for addr in
        (0x8b0, 0xc10, 0xc14, 0xc90, 0xc94, 0xe10, 0xe14, 0xe90, 0xe94)
    )
    body_alt = "\n".join(
        f"BB 0x{addr:x} = 0x55555555" for addr in
        (0x8b0, 0xc10, 0xc14, 0xc90, 0xc94, 0xe10, 0xe14, 0xe90, 0xe94)
    )
    res = run_diff(wrap(body), wrap(body_alt), tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_iqk_output_unmasked_under_strict(tmp_path: Path) -> None:
    """--strict bypasses the IQK output mask; functional checks
    that want bit-exact match (e.g. replay testing) can opt in."""
    kernel = wrap("BB 0xc90 = 0xAAAAAAAA")
    devourer = wrap("BB 0xc90 = 0x55555555")
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


def test_mac_opmode_artifacts_masked(tmp_path: Path) -> None:
    """MAC 0x100 / 0x420 / 0x4c8 / 0x522 / 0x610 / 0x614 are kernel-vs-
    devourer monitor-mode structural differences. Kernel programs the
    iface MAC, TX queue control, etc; devourer monitor-mode doesn't.
    These should be masked entirely so the diff doesn't fail on them."""
    body_kernel = "\n".join([
        "MAC 0x100 = 0x000006ff",
        "MAC 0x420 = 0x03311f80",
        "MAC 0x4c8 = 0x363608ff",
        "MAC 0x522 = 0x4f000000",
        "MAC 0x610 = 0xc7b00d20",
        "MAC 0x614 = 0x0000b3e4",
    ])
    body_devourer = "\n".join([
        "MAC 0x100 = 0x000000c5",
        "MAC 0x420 = 0x00310f00",
        "MAC 0x4c8 = 0x1f1f08ff",
        "MAC 0x522 = 0x470f0000",
        "MAC 0x610 = 0x00000000",
        "MAC 0x614 = 0x00000000",
    ])
    res = run_diff(wrap(body_kernel), wrap(body_devourer), tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_5g_capture_state_artifact_8814_path_c_d_masked(tmp_path: Path) -> None:
    """8814 path-C/D CCK TX-AGC mirrors (0x1820 / 0x1a20) — same
    asymmetry as path-A/B, should also be masked at 5G."""
    body_k = "BB 0x1820 = 0x2A2A2A2A\nBB 0x1a20 = 0x3B3B3B3B"
    body_d = "BB 0x1820 = 0x00000000\nBB 0x1a20 = 0x00000000"
    res = run_diff(wrap(body_k), wrap(body_d),
                   "--channel", "100", tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "0x1820" in res.stdout
    assert "0x1a20" in res.stdout


def test_8814_pathcd_bb_swing_thermal_masked(tmp_path: Path) -> None:
    """8814 path-C/D TX scaling (0x181c / 0x1a1c) bits 31:21 are
    the same pwrtrk-tracked field as path-A/B 0xc1c / 0xe1c."""
    kernel = wrap("BB 0x181c = 0x47C00003\nBB 0x1a1c = 0x47C00003")
    devourer = wrap("BB 0x181c = 0x40000003\nBB 0x1a1c = 0x40000003")
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_8814_pathcd_bb_swing_lower_bits_still_diffed(tmp_path: Path) -> None:
    """Lower bits of 0x181c / 0x1a1c are checked."""
    kernel = wrap("BB 0x181c = 0x47C00003")
    devourer = wrap("BB 0x181c = 0x47C00103")  # bit 8 differs
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 1, res.stdout + res.stderr


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
    """A raw devourer stderr capture carries other `devourer [X] `
    diagnostic lines around the dump envelope. The parser should
    strip the prefix but still only consider the envelope."""
    kernel = wrap("BB 0x808 = 0x3E028233")
    # Extra junk before + after the envelope.
    devourer = (
        "devourer [I] spurious diagnostic that should be ignored\n"
        "devourer [W] BB 0x808 = 0xDEADBEEF\n"   # outside envelope — ignored
        "MAC 0x040 = 0xCAFEBABE\n"               # outside envelope — ignored
        + dwrap("BB 0x808 = 0x3E028233")
        + "trailing junk\n"
    )
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_prefix_stripped_any_level_letter(tmp_path: Path) -> None:
    """Prefix stripping is liberal: any `devourer [X] ` level letter
    (T/D/I/W/E) parses, and pre-stripped devourer captures still work."""
    kernel = wrap("BB 0x808 = 0x3E028233")
    devourer = (
        "devourer [D] === DEVOURER_DUMP_CANARY (post channel-set ch=6) ===\n"
        "devourer [T] BB 0x808 = 0x3E028233\n"
        "devourer [D] === END DEVOURER_DUMP_CANARY ===\n"
    )
    res = run_diff(kernel, devourer, tmp_path=tmp_path)
    assert res.returncode == 0, res.stdout + res.stderr


def test_show_clean_emits_clean_line(tmp_path: Path) -> None:
    canary = wrap("BB 0x808 = 0x3E028233")
    res = run_diff(canary, canary, "--show-clean", tmp_path=tmp_path)
    assert res.returncode == 0
    assert "CLEAN" in res.stdout
