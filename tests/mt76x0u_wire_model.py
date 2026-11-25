#!/usr/bin/env python3
"""A scripted MT7610U for transfer-level checks of the MT76x0U port.

FakeChip answers EP0 vendor requests and bulk transfers the way the chip's
register and MCU interface would for a bring-up and channel sets, and logs
every transaction as one canonical line. Two drivers can be pointed at it and
their logs diffed: the C++ library through tests/mt76x0u_wire_harness (pipe
protocol below), and - as a bench comparison, not a test - wifit3's Python
driver (tools/mt76x0u_wire_vs_wifit3.py).

The model is deliberately simple: registers are a dict, a few have behaviour
(chip id, MAC version, firmware-running flag, EFUSE kick, PLL/XTAL ready,
temperature sensor), firmware chunks are recognised by the FCE DMA writes that
precede them, MCU commands are decoded and answered by sequence number, and
CMD_RANDOM_READ is answered from the RF/MAC state the random writes built. It
proves sequence and encoding, not that the real chip behaves like this.

As a ctest (no arguments but the harness path) it runs three cases against a
synthetic firmware image, so it needs nothing from linux-firmware:
  cold  - firmware not running: uploaded, chip comes up, five channels tune;
  warm  - firmware already running: NOT re-uploaded (the kernel's path);
  force - firmware running, re-upload forced: refused (bench 2026-11-19: it
          jams RX DMA on hardware), firmware kept as in the warm case.

  tests/mt76x0u_wire_model.py HARNESS                 run the cases
  tests/mt76x0u_wire_model.py HARNESS --log OUT FWDIR [force] CH...
                                                      one run, log to OUT
"""

from __future__ import annotations

import hashlib
import os
import struct
import subprocess
import sys
import tempfile
from collections import deque

LIBUSB_ERROR_TIMEOUT = -7

# --- registers the model gives behaviour to (mt76x02_regs.h) -----------------
ASIC_VERSION = 0x0000
CMB_CTRL = 0x0020
EFUSE_CTRL = 0x0024
EFUSE_DATA0 = 0x0028
WPDMA_GLO_CFG = 0x0208
FCE_DMA_ADDR = 0x0230
FCE_DMA_LEN = 0x0234
USB_DMA_CFG = 0x0238
COM_REG0 = 0x0730
MAC_CSR0 = 0x1000
MAC_STATUS = 0x1200
BBP_CORE0 = 0x2000
BBP_CORE34 = 0x2088
BBP_CORE35 = 0x208C

MEMMAP_WLAN = 0x410000
MEMMAP_RF = 0x80000000

MCU_CMDS = {1: "FUN_SET", 10: "RANDOM_READ", 12: "RANDOM_WRITE", 20: "POWER_SAVING",
            31: "CALIBRATION"}
FUNCS = {1: "Q_SELECT", 2: "BW_SETTING", 5: "GET_FW_VERSION"}
CALS = {1: "R", 2: "RXDCOC", 3: "LC", 4: "LOFT", 5: "TXIQ", 6: "BW", 7: "DPD",
        8: "RXIQ", 9: "TXDCOC", 10: "RX_GROUP_DELAY", 11: "TX_GROUP_DELAY",
        12: "VCO", 0xFE: "NO_SIGNAL", 0xFF: "FULL"}


def default_eeprom() -> bytes:
    """A plausible MT7610U EFUSE image: dual band, internal PAs, 1T1R, no TSSI,
    the LNA gains wifit3's tests use."""
    ee = bytearray(512)

    def w16(off, v):
        ee[off:off + 2] = struct.pack("<H", v)
    w16(0x000, 0x7610)                       # chip id (wifit3 checks it; mt76 does not)
    w16(0x002, 0x0C01)                       # version 0x0c, fae 0x01
    ee[0x004:0x00A] = bytes([0x00, 0x0E, 0x8D, 0x76, 0x10, 0x01])
    w16(0x034, 0x0311)                       # NIC_CONF_0: 1T1R, PA_INT 2G+5G, board 0
    w16(0x03A, 0x0040)                       # FREQ_OFFSET
    w16(0x044, 0x0804)                       # LNA: 2g 4, 5g[0] 8
    w16(0x048, 0x0A00)                       # 5g[1] 10
    w16(0x04C, 0x0C00)                       # 5g[2] 12
    w16(0x0D0, 0x8500)                       # 2G target power: temp offset +5
    return bytes(ee)


def default_usage_map() -> bytes:
    """Physical-read view of 0x1e0..0x1ff: some blocks used (non-zero)."""
    return bytes([1] * 8 + [0] * 24)


class FakeChip:
    def __init__(self, fw_running=False, eeprom=None):
        self.regs: dict[int, int] = {}
        self.rf: dict[int, int] = {}
        self.fce: dict[int, int] = {}
        self.fw_running = fw_running
        self.fw_pending_len = False
        self.efuse_addr = 0
        self.efuse_mode = 0
        self.eeprom = eeprom or default_eeprom()
        self.usage = default_usage_map()
        self.responses: deque[bytes] = deque()
        self.log: list[str] = []
        self.fw_chunks = 0
        self.fw_bytes = 0

    # --- registers -----------------------------------------------------
    def read(self, addr: int) -> int:
        v = self.regs.get(addr, 0)
        if addr == ASIC_VERSION:
            return 0x76100044
        if addr == MAC_CSR0:
            return 0x76502000
        if addr == COM_REG0:
            return 1 if self.fw_running else 0
        if addr == CMB_CTRL:
            return v | (1 << 22) | (1 << 23)          # XTAL_RDY | PLL_LD
        if addr == EFUSE_CTRL:
            return v & ~((1 << 30) | 0x3F)            # KICK done, AOUT 0 (burned)
        if EFUSE_DATA0 <= addr < EFUSE_DATA0 + 16:
            src = self.usage if self.efuse_mode == 1 else self.eeprom
            off = self.efuse_addr - (0x1E0 if self.efuse_mode == 1 else 0)
            i = off + (addr - EFUSE_DATA0)
            block = src[i:i + 4].ljust(4, b"\xff")
            return struct.unpack("<I", block)[0]
        if addr == WPDMA_GLO_CFG:
            return v & ~0xA                           # TX/RX DMA idle
        if addr == USB_DMA_CFG:
            return v & ~0xC0000000                    # not busy
        if addr == MAC_STATUS:
            return 0
        if addr == BBP_CORE0:
            return 0x00000075                         # a BBP version: neither 0 nor ~0
        if addr == BBP_CORE34:
            return v & ~0x10                          # temperature sample done
        if addr == BBP_CORE35:
            return 0x0E
        return v

    def write(self, addr: int, val: int) -> None:
        self.regs[addr] = val & 0xFFFFFFFF
        if addr == EFUSE_CTRL and val & (1 << 30):
            self.efuse_addr = (val >> 16) & 0x3FF
            self.efuse_mode = (val >> 6) & 0x3

    # --- EP0 -----------------------------------------------------------
    def ctrl(self, type_: int, req: int, val: int, idx: int, length: int,
             data: bytes) -> tuple[int, bytes]:
        is_in = bool(type_ & 0x80)
        addr = (val << 16) | idx
        if req == 0x07 and is_in:                     # MULTI_READ
            v = self.read(addr)
            self.log.append(f"R 0x{addr:04x} -> 0x{v:08x}")
            return length, struct.pack("<I", v)[:length]
        if req == 0x06 and not is_in:                 # MULTI_WRITE
            if len(data) == 4:
                v = struct.unpack("<I", data)[0]
                self.write(addr, v)
                self.log.append(f"W 0x{addr:04x} = 0x{v:08x}")
            else:
                for i in range(0, len(data) - len(data) % 4, 4):
                    self.write(idx + i, struct.unpack_from("<I", data, i)[0])
                kind = "zero" if not any(data) else hashlib.sha256(data).hexdigest()[:12]
                self.log.append(f"WCOPY 0x{idx:04x} len={len(data)} {kind}")
            return len(data), b""
        if req == 0x42 and not is_in:                 # WRITE_FCE (16-bit halves)
            reg, hi = idx & ~3, bool(idx & 2)
            cur = self.fce.get(reg, 0)
            cur = (cur & 0xFFFF) | (val << 16) if hi else (cur & 0xFFFF0000) | val
            self.fce[reg] = cur
            if reg == FCE_DMA_LEN and hi:
                self.fw_pending_len = True
            self.log.append(f"FCE 0x{reg:04x}.{'hi' if hi else 'lo'} = 0x{val:04x}")
            return 0, b""
        if req == 0x01 and not is_in:                 # DEV_MODE
            if val == 0x01:
                self.fw_running = False
                self.log.append("DEV_MODE fw_reset")
            elif val == 0x12:
                self.fw_running = True                # the IVB starts the firmware
                self.log.append(f"DEV_MODE start ivb len={len(data)} "
                                f"{hashlib.sha256(data).hexdigest()[:12]}")
            else:
                self.log.append(f"DEV_MODE 0x{val:02x} len={len(data)}")
            return len(data), b""
        self.log.append(f"CTRL type=0x{type_:02x} req=0x{req:02x} val=0x{val:04x} "
                        f"idx=0x{idx:04x} len={length}")
        return (length if is_in else len(data)), (b"\0" * length if is_in else b"")

    # --- bulk ----------------------------------------------------------
    def _addr_name(self, a: int) -> str:
        if a & MEMMAP_RF:
            r = a & ~MEMMAP_RF
            return f"RF {r >> 16}/{r & 0xFF}"
        if MEMMAP_WLAN <= a < MEMMAP_WLAN + 0x10000:
            return f"MAC 0x{a - MEMMAP_WLAN:04x}"
        return f"0x{a:08x}"

    def bulk_out(self, ep: int, data: bytes) -> int:
        if ep != 0x08:
            self.log.append(f"BULK_OUT ep=0x{ep:02x} len={len(data)}")
            return len(data)
        info = struct.unpack_from("<I", data, 0)[0]
        length = info & 0xFFFF
        if self.fw_pending_len:
            # A firmware chunk: the FCE DMA address/length writes came first.
            self.fw_pending_len = False
            dst = self.fce.get(FCE_DMA_ADDR, 0)
            payload = data[4:4 + length]
            self.fw_chunks += 1
            self.fw_bytes += length
            self.log.append(f"FW dst=0x{dst:05x} len={length} "
                            f"{hashlib.sha256(payload).hexdigest()[:12]}")
            return len(data)
        seq = (info >> 16) & 0xF
        cmd = (info >> 20) & 0x7F
        payload = data[4:4 + length]
        wait = "wait" if seq else "nowait"
        name = MCU_CMDS.get(cmd, f"cmd{cmd}")
        reply_pairs = b""
        if cmd in (10, 12):
            pairs = [struct.unpack_from("<II", payload, i)
                     for i in range(0, len(payload) - len(payload) % 8, 8)]
            self.log.append(f"MCU {name} n={len(pairs)} {wait}")
            for a, v in pairs:
                if cmd == 12:
                    if a & MEMMAP_RF:
                        self.rf[a & ~MEMMAP_RF] = v & 0xFF
                    elif MEMMAP_WLAN <= a < MEMMAP_WLAN + 0x10000:
                        self.write(a - MEMMAP_WLAN, v)
                    self.log.append(f"  {self._addr_name(a)} = 0x{v:x}")
                else:
                    if a & MEMMAP_RF:
                        rv = self.rf.get(a & ~MEMMAP_RF, 0)
                    else:
                        rv = self.read(a - MEMMAP_WLAN)
                    reply_pairs += struct.pack("<II", a, rv)
                    self.log.append(f"  read {self._addr_name(a)}")
        elif cmd == 1 and len(payload) >= 8:
            f, v = struct.unpack_from("<II", payload)
            self.log.append(f"MCU FUN_SET {FUNCS.get(f, f)} 0x{v:x} {wait}")
        elif cmd == 31 and len(payload) >= 8:
            c, v = struct.unpack_from("<II", payload)
            self.log.append(f"MCU CAL {CALS.get(c, c)} 0x{v:x} {wait}")
        else:
            self.log.append(f"MCU {name} len={length} {payload.hex()} {wait}")
        if seq:
            # mt76x02u_mcu_wait_resp: info word (seq, EVT_CMD_DONE), the
            # random-read pairs, a 4-byte trailer.
            rinfo = (seq << 16) | (len(reply_pairs) + 4)
            self.responses.append(struct.pack("<I", rinfo) + reply_pairs + b"\0" * 4)
        return len(data)

    def bulk_in(self, ep: int, max_len: int) -> tuple[int, bytes]:
        if ep == 0x85 and self.responses:
            r = self.responses.popleft()
            return 0, r[:max_len]
        return LIBUSB_ERROR_TIMEOUT, b""   # EP 0x84 carries nothing here

    def marker(self, label: str) -> None:
        self.log.append(f"# --- {label} ---")


# --- the C++ library, over the harness pipe ---------------------------------

def run_harness(harness: str, chip: FakeChip, fw_dir: str, args: list[str]) -> int:
    p = subprocess.Popen([harness, fw_dir, *args], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stderr_lines: list[str] = []
    for line in p.stdout:
        f = line.rstrip("\n").split(" ")
        if f[0] == "M":
            chip.marker(" ".join(f[1:]))
            continue
        if f[0] == "C":
            type_, req, val, idx, ln = (int(f[1], 16), int(f[2], 16), int(f[3], 16),
                                        int(f[4], 16), int(f[5]))
            data = b"" if f[6] == "-" else bytes.fromhex(f[6])
            rc, out = chip.ctrl(type_, req, val, idx, ln, data)
            p.stdin.write(f"{rc} {out.hex() or '-'}\n")
        elif f[0] == "B":
            ep, ln = int(f[1], 16), int(f[2])
            if ep & 0x80:
                rc, out = chip.bulk_in(ep, ln)
                p.stdin.write(f"{rc} {len(out)} {out.hex() or '-'}\n")
            else:
                n = chip.bulk_out(ep, bytes.fromhex(f[4]))
                p.stdin.write(f"0 {n} -\n")
        p.stdin.flush()
    rc = p.wait()
    stderr_lines = p.stderr.read().splitlines()
    if rc:
        sys.stderr.write("\n".join(stderr_lines[-15:]) + "\n")
    return rc


def synthetic_firmware(dirpath: str) -> None:
    """A structurally valid mt7610e.bin: 32-byte header, ILM (with its 64-byte
    IVB), DLM. Contents are arbitrary; only the shape matters to the loader."""
    ilm, dlm = 64 + 2 * 0x38F0, 300            # ILM = IVB + exactly two chunks
    hdr = struct.pack("<IIHH4s16s", ilm, dlm, 0x7640, 0x0100, b"\0" * 4, b"2014040110180000")
    body = bytes((i * 7) & 0xFF for i in range(ilm + dlm))
    with open(os.path.join(dirpath, "mt7610e.bin"), "wb") as fh:
        fh.write(hdr + body)


# --- the ctest cases ---------------------------------------------------------

def check(cond: bool, what: str, failures: list[str]) -> None:
    if not cond:
        failures.append(what)


def cases(harness: str) -> int:
    failures: list[str] = []
    with tempfile.TemporaryDirectory() as fw:
        synthetic_firmware(fw)
        chans = ["1", "6", "36", "100", "157", "36+", "40-"]

        cold = FakeChip(fw_running=False)
        rc = run_harness(harness, cold, fw, chans)
        log = cold.log
        check(rc == 0, "cold: bring-up and every channel set succeed", failures)
        check(cold.fw_chunks == 2 + 1, f"cold: ILM in 2 chunks + DLM in 1 (got {cold.fw_chunks})",
              failures)
        check(any(l.startswith("FW dst=0x00040 ") for l in log), "cold: ILM lands at 0x40", failures)
        check(any(l.startswith("FW dst=0x80000 ") for l in log), "cold: DLM lands at 0x80000",
              failures)
        check(any(l.startswith("DEV_MODE start ivb len=64") for l in log),
              "cold: firmware started with the 64-byte IVB", failures)
        qsel = next((i for i, l in enumerate(log) if "Q_SELECT" in l), None)
        first_mcu = next((i for i, l in enumerate(log) if l.startswith("MCU ")), None)
        check(qsel is not None and qsel == first_mcu, "cold: Q_SELECT is the first MCU command",
              failures)
        pse = [i for i, l in enumerate(log) if l == "W 0x0800 = 0x00000001"]
        check(len(pse) >= 2, "cold: MT_FCE_PSE_CTRL written before and after the upload", failures)
        check(sum(1 for l in log if l.startswith("MCU FUN_SET BW_SETTING 0x1")) == 2 * 2,
              "cold: two 40 MHz tunes, each run twice, set BW_SETTING 1", failures)
        check(any(l.startswith("MCU CAL VCO") for l in log) is False,
              "cold: no power-on VCO cal before a start (none was requested)", failures)
        check(sum(1 for l in log if l.startswith("MCU CAL FULL")) == 2 * len(chans),
              "cold: one full calibration per pass, two passes per channel set (the "
              "kernel's double invocation)", failures)

        once = FakeChip(fw_running=False)
        rc = run_harness(harness, once, fw, ["once", "6"])
        check(rc == 0 and sum(1 for l in once.log if l.startswith("MCU CAL FULL")) == 1,
              "once: a single pass when asked", failures)
        check(any("MCU CAL FULL 0x701" in l for l in log) and
              any("MCU CAL FULL 0x801" in l for l in log) and
              any("MCU CAL FULL 0x901" in l for l in log) and
              any("MCU CAL FULL 0x600" in l for l in log),
              "cold: calibration value by band/channel (0x600/0x701/0x801/0x901)", failures)

        warm = FakeChip(fw_running=True)
        rc = run_harness(harness, warm, fw, ["6"])
        check(rc == 0, "warm: bring-up succeeds", failures)
        check(warm.fw_chunks == 0, "warm: running firmware kept, nothing uploaded", failures)
        check(not any(l == "DEV_MODE fw_reset" for l in warm.log), "warm: no firmware reset",
              failures)

        forced = FakeChip(fw_running=True)
        rc = run_harness(harness, forced, fw, ["force", "6"])
        check(rc == 0, "force: bring-up succeeds", failures)
        check(forced.fw_chunks == 0, "force: refused, firmware not re-uploaded", failures)
        check(not any(l == "DEV_MODE fw_reset" for l in forced.log),
              "force: refused, no firmware reset", failures)

    if failures:
        for f in failures:
            print(f"mt76x0u_wire: FAIL {f}", file=sys.stderr)
        return 1
    print(f"mt76x0u_wire: ok (cold {len(cold.log)} transactions, warm {len(warm.log)}, "
          f"forced {len(forced.log)})")
    return 0


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    harness = argv[1]
    if len(argv) > 2 and argv[2] == "--log":
        out, fw, rest = argv[3], argv[4], argv[5:]
        chip = FakeChip(fw_running=False)
        rc = run_harness(harness, chip, fw, rest)
        with open(out, "w") as fh:
            fh.write("\n".join(chip.log) + "\n")
        print(f"{len(chip.log)} transactions -> {out} (rc {rc})")
        return rc
    return cases(harness)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
