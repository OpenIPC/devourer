#!/usr/bin/env python3
"""kestrel_capture_diff.py — usbmon-text analyzer for the Kestrel (RTL8852BU)
bring-up deviation audit: turn a `cat /sys/kernel/debug/usb/usbmon/Nu` capture
into a canonical register/H2C op stream and diff two captures (vendor golden
vs devourer) over the init windows.

The Realtek USB register plane is vendor request 0x05: wValue = low 16 addr
bits, wIndex = high bits (0 = MAC space, 1 = BB/RF window). usbmon `=` data is
a BYTE stream — dwords must be LE-decoded. usbmon text shows at most 32 data
bytes, which for a bulk-OUT H2C is exactly WD(24) + fwcmd hdr(8): enough to
classify every H2C (cat/class/func/seq/total_len) but not its content.

Subcommands:
  h2c FILE                 the runtime-H2C ledger (FWDL sections summarized)
  writes FILE [--window W] ordered control-write stream for a window
  reads FILE --addr HEX    completions of control reads of one register
  ledger FILE              milestones + per-window op counts
  diff VENDOR OURS [--window W]  register-write diff over a window
Windows: `prefwdl` (start → FWDL header), `fwdl` (FWDL header → last section),
`boot2ofld` (last section → first cmd_ofld), `ofld` (first cmd_ofld → end),
`all` (default).
"""
import argparse
import re
import sys
from collections import Counter, OrderedDict

CTRL_RE = re.compile(
    r'^(?P<tag>\S+) (?P<ts>\d+) (?P<ev>[SC]) (?P<typ>C[io]):(?P<bus>\d+):'
    r'(?P<dev>\d+):0 (?P<rest>.*)$')
BULK_RE = re.compile(
    r'^(?P<tag>\S+) (?P<ts>\d+) (?P<ev>[SC]) (?P<typ>B[io]):(?P<bus>\d+):'
    r'(?P<dev>\d+):(?P<ep>\d+) (?P<st>\S+) (?P<len>\d+)(?P<rest>.*)$')
SETUP_RE = re.compile(
    r's (?P<rt>[0-9a-f]{2}) (?P<rq>[0-9a-f]{2}) (?P<val>[0-9a-f]{4}) '
    r'(?P<idx>[0-9a-f]{4}) (?P<wlen>[0-9a-f]{4})')
DATA_RE = re.compile(r'= ([0-9a-f ]+)$')


def le(hexbytes: str) -> int:
    return int.from_bytes(bytes.fromhex(hexbytes), 'little')


def parse_data(rest: str):
    m = DATA_RE.search(rest)
    if not m:
        return b''
    return bytes.fromhex(m.group(1).replace(' ', ''))


class Op:
    __slots__ = ('kind', 'ts', 'line', 'addr', 'wlen', 'data', 'ep', 'h2c')

    def __init__(self, kind, ts, line, **kw):
        self.kind = kind          # 'W', 'R', 'H2C', 'FWDL', 'BI'
        self.ts = ts
        self.line = line
        self.addr = kw.get('addr')
        self.wlen = kw.get('wlen')
        self.data = kw.get('data')   # bytes for W; int value for R completion
        self.ep = kw.get('ep')
        self.h2c = kw.get('h2c')     # dict for H2C kinds

    def val(self):
        return le(self.data.hex()) if isinstance(self.data, (bytes, bytearray)) and self.data else None


def decode_h2c(data: bytes):
    """First 32 bytes of a CH12 bulk-OUT: WD body (24) + fwcmd hdr (8)."""
    if len(data) < 24:
        return None
    d0 = int.from_bytes(data[0:4], 'little')
    out = {'wd0': d0, 'fwdl_en': bool((d0 >> 20) & 1),
           'ch_dma': (d0 >> 16) & 0xF}
    if len(data) >= 32:
        h0 = int.from_bytes(data[24:28], 'little')
        h1 = int.from_bytes(data[28:32], 'little')
        out.update(cat=h0 & 3, cls=(h0 >> 2) & 0x3F, func=(h0 >> 8) & 0xFF,
                   del_type=(h0 >> 16) & 0xF, seq=(h0 >> 24) & 0xFF,
                   total_len=h1 & 0x3FFF, rec_ack=bool(h1 & (1 << 14)),
                   done_ack=bool(h1 & (1 << 15)), hdr1=h1)
    return out


def parse(path: str):
    """Yield Op records; pair control-read submissions with completions."""
    pending_reads = {}   # urb tag -> (addr, wlen, ts, line)
    ops = []
    with open(path, errors='replace') as f:
        for lineno, line in enumerate(f, 1):
            m = CTRL_RE.match(line)
            if m:
                if m.group('ev') == 'S':
                    s = SETUP_RE.search(m.group('rest'))
                    if not s or s.group('rq') != '05':
                        continue
                    addr = (int(s.group('idx'), 16) << 16) | int(s.group('val'), 16)
                    wlen = int(s.group('wlen'), 16)
                    if s.group('rt') == '40':
                        ops.append(Op('W', int(m.group('ts')), lineno,
                                      addr=addr, wlen=wlen,
                                      data=parse_data(m.group('rest'))))
                    elif s.group('rt') == 'c0':
                        pending_reads[m.group('tag')] = (addr, wlen,
                                                         int(m.group('ts')), lineno)
                elif m.group('ev') == 'C' and m.group('typ') == 'Ci':
                    pend = pending_reads.pop(m.group('tag'), None)
                    if pend:
                        addr, wlen, ts, sl = pend
                        ops.append(Op('R', ts, sl, addr=addr, wlen=wlen,
                                      data=parse_data(m.group('rest'))))
                continue
            m = BULK_RE.match(line)
            if m and m.group('ev') == 'S' and m.group('typ') == 'Bo':
                data = parse_data(m.group('rest'))
                h = decode_h2c(data) if data else None
                if h is not None:
                    kind = 'FWDL' if h['fwdl_en'] else 'H2C'
                    ops.append(Op(kind, int(m.group('ts')), lineno,
                                  ep=int(m.group('ep')), h2c=h,
                                  wlen=int(m.group('len'))))
            elif m and m.group('ev') == 'C' and m.group('typ') == 'Bi':
                if int(m.group('len') or 0) > 0:
                    ops.append(Op('BI', int(m.group('ts')), lineno,
                                  ep=int(m.group('ep')), wlen=int(m.group('len'))))
    return ops


def milestones(ops):
    """Indices of window boundaries in the op list, for the LAST bring-up
    attempt in the capture (rxdemo's libusb_reset can leave an earlier partial
    attempt in the whole-bus stream). Anchor on the last FWDL header (cls 3),
    take the last FWDL section at or after it, and the first cmd_ofld (cls 9)
    after that section."""
    fwdl_hdrs = [i for i, op in enumerate(ops)
                 if op.kind == 'H2C' and op.h2c.get('cls') == 3]
    ms = {'start': 0, 'fwdl_hdr': None, 'last_sec': None, 'first_ofld': None}
    ms['fwdl_hdr'] = fwdl_hdrs[-1] if fwdl_hdrs else None
    anchor = ms['fwdl_hdr'] if ms['fwdl_hdr'] is not None else 0
    for i in range(anchor, len(ops)):
        if ops[i].kind == 'FWDL':
            ms['last_sec'] = i
    lo = ms['last_sec'] if ms['last_sec'] is not None else anchor
    for i in range(lo, len(ops)):
        if ops[i].kind == 'H2C' and ops[i].h2c.get('cls') == 9:
            ms['first_ofld'] = i
            break
    return ms


def window_slice(ops, window):
    ms = milestones(ops)
    n = len(ops)
    bounds = {
        'all': (0, n),
        'prefwdl': (0, ms['fwdl_hdr'] if ms['fwdl_hdr'] is not None else n),
        'fwdl': (ms['fwdl_hdr'] or 0, (ms['last_sec'] + 1) if ms['last_sec'] is not None else n),
        'boot2ofld': ((ms['last_sec'] + 1) if ms['last_sec'] is not None else 0,
                      ms['first_ofld'] if ms['first_ofld'] is not None else n),
        'ofld': (ms['first_ofld'] if ms['first_ofld'] is not None else n, n),
    }
    lo, hi = bounds[window]
    return ops[lo:hi], ms


def fmt_write(op):
    v = op.val()
    return (f"W{op.wlen} 0x{op.addr:06x} = 0x{v:0{op.wlen * 2}x}"
            if v is not None else f"W{op.wlen} 0x{op.addr:06x} = <nodata>")


def cmd_h2c(args):
    ops = parse(args.file)
    fwdl = sum(1 for o in ops if o.kind == 'FWDL')
    h2cs = [o for o in ops if o.kind == 'H2C']
    print(f"{len(h2cs)} runtime H2C, {fwdl} FWDL section pkts")
    for i, o in enumerate(h2cs):
        h = o.h2c
        if 'cls' not in h:
            print(f"{i:4d} L{o.line}: ep{o.ep} len={o.wlen} (short data)")
            continue
        flags = ''.join(x for x, c in (('R', h['rec_ack']), ('D', h['done_ack'])) if c)
        print(f"{i:4d} L{o.line}: ep{o.ep} cat={h['cat']} cls={h['cls']:2d} "
              f"func=0x{h['func']:02x} seq={h['seq']:3d} tlen={h['total_len']:4d} "
              f"pkt={o.wlen} {flags}")
    hist = Counter((o.h2c.get('cat'), o.h2c.get('cls'), o.h2c.get('func')) for o in h2cs)
    print("--- histogram cat/cls/func ---")
    for k, v in sorted(hist.items(), key=lambda kv: (kv[0][0] or 0, kv[0][1] or 0, kv[0][2] or 0)):
        print(f"  cat={k[0]} cls={k[1]:2d} func=0x{k[2] or 0:02x}: {v}")


def cmd_writes(args):
    ops, ms = window_slice(parse(args.file), args.window)
    for op in ops:
        if op.kind == 'W':
            print(f"L{op.line}: {fmt_write(op)}")


def cmd_reads(args):
    addr = int(args.addr, 16)
    ops = parse(args.file)
    for op in ops:
        if op.kind == 'R' and op.addr == addr:
            v = op.val()
            print(f"L{op.line}: R{op.wlen} 0x{op.addr:06x} -> "
                  f"{'0x%0*x' % (op.wlen * 2, v) if v is not None else '<nodata>'}")


def cmd_ledger(args):
    ops = parse(args.file)
    ms = milestones(ops)
    print(f"ops={len(ops)} milestones: " +
          ", ".join(f"{k}={v if v is not None else '-'}" for k, v in ms.items()))
    for w in ('prefwdl', 'fwdl', 'boot2ofld', 'ofld'):
        sl, _ = window_slice(ops, w)
        c = Counter(o.kind for o in sl)
        print(f"  {w:9s}: {dict(c)}")


def cmd_trace(args):
    """Interleaved W/R/H2C/BI stream for a window — places register writes
    relative to the cmd_ofld batches."""
    ops, _ = window_slice(parse(args.file), args.window)
    for op in ops:
        if op.kind == 'W':
            print(f"L{op.line}: {fmt_write(op)}")
        elif op.kind == 'R' and args.reads:
            v = op.val()
            print(f"L{op.line}:   R 0x{op.addr:06x} -> "
                  f"{'0x%x' % v if v is not None else '?'}")
        elif op.kind in ('H2C', 'FWDL'):
            h = op.h2c
            if 'cls' in h:
                n = (h['total_len'] - 8) // 16 if h['cls'] == 9 else 0
                extra = f" [{n} cmds]" if h['cls'] == 9 else ''
                print(f"L{op.line}: >>> {op.kind} ep{op.ep} cat={h['cat']} "
                      f"cls={h['cls']} func=0x{h['func']:02x} seq={h['seq']} "
                      f"tlen={h['total_len']} pkt={op.wlen}{extra}")
            else:
                print(f"L{op.line}: >>> {op.kind} ep{op.ep} pkt={op.wlen}")
        elif op.kind == 'BI':
            print(f"L{op.line}:   <BI ep{op.ep} {op.wlen}B")


def write_seq(ops):
    """OrderedDict (addr, wlen) -> [values in order]."""
    seq = OrderedDict()
    for op in ops:
        if op.kind != 'W':
            continue
        key = (op.addr, op.wlen)
        seq.setdefault(key, []).append(op.val())
    return seq


def cmd_diff(args):
    vops, _ = window_slice(parse(args.vendor), args.window)
    oops, _ = window_slice(parse(args.ours), args.window)
    vw, ow = write_seq(vops), write_seq(oops)
    vaddrs = {a for a, _ in vw}
    oaddrs = {a for a, _ in ow}

    print(f"=== window '{args.window}': vendor {sum(len(v) for v in vw.values())} writes "
          f"({len(vw)} uniq addr+len), ours {sum(len(v) for v in ow.values())} "
          f"({len(ow)} uniq) ===")

    print("--- MISSING (vendor writes, we never touch the addr) ---")
    for (a, wl), vals in vw.items():
        if a not in oaddrs:
            vs = '/'.join(f"0x{v:0{wl*2}x}" if v is not None else '?' for v in vals)
            print(f"  0x{a:06x} (w{wl} x{len(vals)}): {vs}")

    print("--- EXTRA (we write, vendor never touches the addr) ---")
    for (a, wl), vals in ow.items():
        if a not in vaddrs:
            vs = '/'.join(f"0x{v:0{wl*2}x}" if v is not None else '?' for v in vals)
            print(f"  0x{a:06x} (w{wl} x{len(vals)}): {vs}")

    print("--- VALUE MISMATCH (same addr, different final value; width-merged) ---")
    def final_by_addr(w):
        fin = {}
        for (a, wl), vals in w.items():
            if vals and vals[-1] is not None:
                fin[a] = (wl, vals[-1], len(vals))
        return fin
    vf, of = final_by_addr(vw), final_by_addr(ow)
    for a in sorted(vf.keys() & of.keys()):
        (vwl, vv, vn), (owl, ov, on) = vf[a], of[a]
        # compare on the overlapping width
        w = min(vwl, owl)
        mask = (1 << (w * 8)) - 1
        if (vv & mask) != (ov & mask):
            print(f"  0x{a:06x}: vendor w{vwl}=0x{vv:0{vwl*2}x} (x{vn})  "
                  f"ours w{owl}=0x{ov:0{owl*2}x} (x{on})")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('h2c'); p.add_argument('file'); p.set_defaults(fn=cmd_h2c)
    p = sub.add_parser('writes'); p.add_argument('file')
    p.add_argument('--window', default='all',
                   choices=['all', 'prefwdl', 'fwdl', 'boot2ofld', 'ofld'])
    p.set_defaults(fn=cmd_writes)
    p = sub.add_parser('reads'); p.add_argument('file'); p.add_argument('--addr', required=True)
    p.set_defaults(fn=cmd_reads)
    p = sub.add_parser('ledger'); p.add_argument('file'); p.set_defaults(fn=cmd_ledger)
    p = sub.add_parser('trace'); p.add_argument('file')
    p.add_argument('--window', default='ofld',
                   choices=['all', 'prefwdl', 'fwdl', 'boot2ofld', 'ofld'])
    p.add_argument('--reads', action='store_true')
    p.set_defaults(fn=cmd_trace)
    p = sub.add_parser('diff'); p.add_argument('vendor'); p.add_argument('ours')
    p.add_argument('--window', default='boot2ofld',
                   choices=['all', 'prefwdl', 'fwdl', 'boot2ofld', 'ofld'])
    p.set_defaults(fn=cmd_diff)
    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    sys.exit(main())
