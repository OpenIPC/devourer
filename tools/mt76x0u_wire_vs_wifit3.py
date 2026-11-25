#!/usr/bin/env python3
"""Run wifit3's MT7610U driver against tests/mt76x0u_wire_model.py's FakeChip
and write the same canonical transfer log the C++ harness produces, so the two
bring-ups and channel sets can be diffed transaction by transaction.

A bench comparison, not a ctest: it needs a wifit3 checkout
(github.com/derv82/wifit3) and runs that project's code, so run it somewhere
isolated (the plan ran it in a container with --network=none and the checkout
mounted read-only). pyusb is NOT needed: a minimal stub of the three names
wifit3 uses (usb.core.Device/USBError, usb.util) stands in, and the fake
device answers every control/bulk transfer from the model.

  tools/mt76x0u_wire_vs_wifit3.py WIFIT3_SRC OUT.log CH [CH ...]

WIFIT3_SRC is the checkout's src/ directory. wifit3 uses its own bundled
firmware (assets/mt7610e_linux-firmware.bin); compare with the C++ run on the
same mt7610e.bin.
"""

from __future__ import annotations

import asyncio
import os
import sys
import tempfile
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tests"))
from mt76x0u_wire_model import FakeChip  # noqa: E402


def install_usb_stub() -> None:
    usb = types.ModuleType("usb")
    core = types.ModuleType("usb.core")
    util = types.ModuleType("usb.util")

    class USBError(IOError):
        def __init__(self, strerror, error_code=None, errno=None):
            super().__init__(errno, strerror)
            self.backend_error_code = error_code
            self.errno = errno

    class Device:  # only a type for annotations
        pass

    core.USBError = USBError
    core.Device = Device
    for name in ("claim_interface", "release_interface", "dispose_resources"):
        setattr(util, name, lambda *a, **k: None)
    usb.core, usb.util = core, util
    sys.modules.update({"usb": usb, "usb.core": core, "usb.util": util})


class FakeUsbDevice:
    """pyusb Device surface wifit3 touches, backed by FakeChip."""

    def __init__(self, chip: FakeChip):
        self.chip = chip
        self.lock = threading.Lock()   # wifit3's RX drainer reads from a thread

    def ctrl_transfer(self, bmRequestType, bRequest, wValue=0, wIndex=0,
                      data_or_wLength=None, timeout=None):
        is_in = bool(bmRequestType & 0x80)
        if is_in:
            length, data = int(data_or_wLength), b""
        else:
            data = bytes(data_or_wLength or b"")
            length = len(data)
        with self.lock:
            rc, out = self.chip.ctrl(bmRequestType, bRequest, wValue, wIndex, length, data)
        return bytearray(out) if is_in else rc

    def write(self, ep, data, timeout=None):
        with self.lock:
            return self.chip.bulk_out(ep, bytes(data))

    def read(self, ep, length, timeout=None):
        with self.lock:
            rc, out = self.chip.bulk_in(ep, length)
        if rc == 0:
            return bytearray(out)
        if ep == 0x84:
            time.sleep(0.005)          # don't let the drainer spin flat out
        import usb.core
        raise usb.core.USBError("timeout", error_code=-7, errno=110)

    def reset(self):
        pass


async def run(src: str, out: str, channels: list[int]) -> int:
    sys.path.insert(0, src)
    from wifit3.chips.mt76x0u.driver import MT76x0UDriver
    from wifit3.models.device_id import DeviceID

    chip = FakeChip(fw_running=False)
    dev = FakeUsbDevice(chip)
    drv = MT76x0UDriver(dev, DeviceID(0x0E8D, 0x7610, "MT7610U"))
    for name in ("claim", "release", "dispose"):
        setattr(drv.transport, name, lambda *a, **k: None)

    rc = 0
    chip.marker("bring-up")
    try:
        await drv.connect()
        for ch in channels:
            chip.marker(f"set_channel {ch}")
            await drv.set_channel(ch)
    except Exception as e:  # noqa: BLE001 - report and still write the log
        print(f"wifit3 run stopped: {type(e).__name__}: {e}", file=sys.stderr)
        rc = 1
    chip.marker("close")
    try:
        await drv.close()
    except Exception:  # noqa: BLE001
        pass
    with open(out, "w") as fh:
        fh.write("\n".join(chip.log) + "\n")
    print(f"{len(chip.log)} transactions -> {out} (rc {rc}, firmware chunks {chip.fw_chunks})")
    return rc


def main(argv: list[str]) -> int:
    if len(argv) < 4:
        print(__doc__, file=sys.stderr)
        return 2
    install_usb_stub()
    os.environ.pop("WIFIT3_WIRE_LOG_FILE", None)
    return asyncio.run(run(argv[1], argv[2], [int(c) for c in argv[3:]]))


if __name__ == "__main__":
    sys.exit(main(sys.argv))
