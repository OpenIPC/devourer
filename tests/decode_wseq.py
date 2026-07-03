#!/usr/bin/env python3
"""Decode Realtek USB vendor register writes from a usbmon pcap.

Realtek reg write = control-OUT, bmRequestType=0x40, bRequest=0x05,
wValue=address, data=value (1/2/4 bytes, little-endian). Emits the ordered
(addr, len, val) write list so the kernel's cold-init sequence can be diffed
against devourer's bring-up. Input: a DLT_USB_LINUX_MMAPPED pcap (tcpdump -i
usbmonN -w). Stdlib only.
"""
import struct
import sys

DLT_USB_LINUX_MMAPPED = 220


def main(path):
    with open(path, "rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            print("empty pcap")
            return
        magic = struct.unpack("<I", gh[:4])[0]
        le = magic in (0xA1B2C3D4, 0xA1B23C4D)
        endian = "<" if le else ">"
        dlt = struct.unpack(endian + "I", gh[20:24])[0]
        if dlt != DLT_USB_LINUX_MMAPPED:
            print(f"warning: DLT={dlt}, expected {DLT_USB_LINUX_MMAPPED}")

        writes = []
        while True:
            rh = f.read(16)
            if len(rh) < 16:
                break
            _, _, incl, _orig = struct.unpack(endian + "IIII", rh)
            data = f.read(incl)
            if len(data) < 64:
                continue
            # usbmon_packet: type@8, xfer_type@9, flag_setup@14, setup[8]@40
            pkt_type = data[8]
            xfer_type = data[9]
            flag_setup = data[14]
            setup = data[40:48]
            bm, breq = setup[0], setup[1]
            wValue = struct.unpack("<H", setup[2:4])[0]
            wLength = struct.unpack("<H", setup[6:8])[0]
            # control(2) submit('S'=0x53) with setup present (flag_setup==0)
            if xfer_type != 2 or pkt_type != 0x53 or flag_setup != 0:
                continue
            if bm != 0x40 or breq != 0x05:
                continue
            payload = data[64:64 + wLength]
            if len(payload) < wLength or wLength not in (1, 2, 4):
                # value may be truncated in capture; keep addr+len
                val = int.from_bytes(payload, "little") if payload else None
            else:
                val = int.from_bytes(payload, "little")
            writes.append((wValue, wLength, val))

    print(f"# {len(writes)} vendor reg writes (addr len val)")
    for addr, ln, val in writes:
        vs = f"0x{val:0{ln*2}x}" if val is not None else "??"
        print(f"0x{addr:04x} {ln} {vs}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/j2_tx_wseq/wseq.pcap")
