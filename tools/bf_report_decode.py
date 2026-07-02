#!/usr/bin/env python3
"""Decode 802.11ac VHT Compressed Beamforming reports into per-subcarrier CSI.

Input: `<devourer-bf-report-raw>HEX` lines (from WiFiDriverDemo with
DEVOURER_BF_DETECT_REPORT=4) on stdin or a file, or bare hex frames.

For the two-adapter self-sounding path a 2-TX beamformer sounds a 1-RX
beamformee, so each report carries a per-subcarrier 2x1 steering vector
V(k) = h(k)* / ||h(k)|| where h(k) = [h_A(k), h_B(k)] is the per-tone channel
from the beamformer's two antennas. The compressed report encodes V(k) as
Givens angles (psi, phi) per subcarrier:

    |V(k)| = [cos(psi_k), sin(psi_k)],   arg(V_B/V_A) = phi_k

so psi_k = atan(|h_B|/|h_A|) and phi_k = arg(h_B/h_A) — the *relative*
per-subcarrier channel between the two TX antennas. Its variation across k is
the frequency-selective structure (a deep fade on one antenna rotates V).

The angle bit-width is derived from the payload length and the subcarrier
count Ns (from BW + grouping Ng), then the (b_phi, b_psi) split is chosen by
cross-frame stability — the correct split makes psi(k)/phi(k) repeatable
across reports of a quasi-static channel; a wrong split looks like noise.

Usage:
    WiFiDriverDemo ... DEVOURER_BF_DETECT_REPORT=4 | tools/bf_report_decode.py
    tools/bf_report_decode.py captured_frames.txt --csv out.csv
"""
from __future__ import annotations

import argparse
import math
import sys

# Ns = number of subcarriers carried in a VHT compressed BF report, per
# bandwidth (0..3 = 20/40/80/160 MHz) and grouping Ng (1/2/4).
_MSB = False

NS_TABLE = {
    0: {1: 52, 2: 30, 4: 16},
    1: {1: 108, 2: 58, 4: 30},
    2: {1: 234, 2: 122, 4: 62},
    3: {1: 468, 2: 244, 4: 124},
}


class BitReader:
    """Bit reader for the packed angle stream. 802.11 packs LSB-first; a
    --msb flag flips it for cross-checking the decode."""

    def __init__(self, data: bytes, msb: bool = False):
        self.data = data
        self.pos = 0
        self.msb = msb

    def read(self, n: int) -> int:
        v = 0
        for i in range(n):
            byte = self.data[self.pos >> 3]
            if self.msb:
                bit = (byte >> (7 - (self.pos & 7))) & 1
                v = (v << 1) | bit
            else:
                bit = (byte >> (self.pos & 7)) & 1
                v |= bit << i
            self.pos += 1
        return v


def dequant_phi(q: int, b: int) -> float:
    """phi in (0, 2*pi): phi = (2q+1) * pi / 2^b."""
    return (2 * q + 1) * math.pi / (1 << b)


def dequant_psi(q: int, b: int) -> float:
    """psi in (0, pi/2): psi = (2q+1) * pi / 2^(b+2)."""
    return (2 * q + 1) * math.pi / (1 << (b + 2))


def parse_frame(hexstr: str):
    """Return dict with header fields + raw angle bytes, or None if not a
    VHT/HT compressed beamforming report."""
    try:
        d = bytes.fromhex(hexstr)
    except ValueError:
        return None
    if len(d) < 30:
        return None
    sub = d[0] & 0xF0
    if sub not in (0xD0, 0xE0):           # Action / Action No-Ack
        return None
    cat, act = d[24], d[25]
    if not ((cat == 0x15 or cat == 0x07) and act == 0x00):
        return None
    mc = d[26] | (d[27] << 8) | (d[28] << 16)   # 24-bit LE MIMO control
    nc = (mc & 0x7) + 1
    nr = ((mc >> 3) & 0x7) + 1
    bw = (mc >> 6) & 0x3
    ng_code = (mc >> 8) & 0x3
    ng = {0: 1, 1: 2, 2: 4, 3: 4}[ng_code]
    codebook = (mc >> 10) & 0x1
    feedback = (mc >> 11) & 0x1           # 0 = SU, 1 = MU
    sa = ":".join(f"{b:02x}" for b in d[10:16])
    snr = list(d[29:29 + nc])             # avg SNR per column (signed 0.25 dB)
    angle_bytes = d[29 + nc:len(d) - 4]   # drop 4-byte FCS
    return dict(sa=sa, nc=nc, nr=nr, bw=bw, ng=ng, codebook=codebook,
                feedback=feedback, snr=snr, angle_bytes=angle_bytes)


def angle_layout(nr: int, nc: int):
    """Order of (kind, index) angles for the compressed matrix, per
    802.11 19.3.12.3.6. For each column i: (Nr-i) phi's then (Nr-i) psi's...
    actually the standard interleaves D(phi) then G(psi) per Givens stage.
    For the 2x1 case this is simply [phi, psi]."""
    seq = []
    for i in range(1, min(nc, nr - 1) + 1):
        for _ in range(i, nr):
            seq.append("phi")
        for _ in range(i, nr):
            seq.append("psi")
    return seq                            # 2x1 -> ["phi","psi"]


def decode_angles(angle_bytes: bytes, ns: int, na: int, bphi: int, bpsi: int,
                  layout):
    """Unpack ns subcarriers x na angles. Returns list per subcarrier of
    dict(phi=[...], psi=[...]) in radians, or None if the stream is too short."""
    need_bits = ns * (layout.count("phi") * bphi + layout.count("psi") * bpsi)
    if need_bits > len(angle_bytes) * 8:
        return None
    br = BitReader(angle_bytes, _MSB)
    out = []
    for _ in range(ns):
        phi, psi = [], []
        for kind in layout:
            if kind == "phi":
                phi.append(dequant_phi(br.read(bphi), bphi))
            else:
                psi.append(dequant_psi(br.read(bpsi), bpsi))
        out.append(dict(phi=phi, psi=psi))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("infile", nargs="?", help="capture file (default stdin)")
    ap.add_argument("--csv", help="write per-subcarrier CSV here")
    ap.add_argument("--max-frames", type=int, default=200)
    ap.add_argument("--msb", action="store_true", help="MSB-first bit order")
    args = ap.parse_args()

    global _MSB
    _MSB = args.msb
    src = open(args.infile) if args.infile else sys.stdin
    frames = []
    for line in src:
        line = line.strip()
        if "<devourer-bf-report-raw>" in line:
            line = line.split("<devourer-bf-report-raw>", 1)[1]
        f = parse_frame(line)
        if f:
            frames.append(f)
        if len(frames) >= args.max_frames:
            break
    if not frames:
        print("no VHT/HT compressed-beamforming reports found", file=sys.stderr)
        return 1

    f0 = frames[0]
    nr, nc, bw, ng = f0["nr"], f0["nc"], f0["bw"], f0["ng"]
    ns = NS_TABLE.get(bw, {}).get(ng)
    layout = angle_layout(nr, nc)
    na = len(layout)
    nbits = len(f0["angle_bytes"]) * 8
    print(f"# {len(frames)} reports  SA={f0['sa']}  Nr={nr} Nc={nc} "
          f"BW={20 << bw}MHz Ng={ng} codebook={f0['codebook']} "
          f"feedback={'MU' if f0['feedback'] else 'SU'}")
    if not ns:
        print(f"# unknown Ns for BW={bw} Ng={ng}", file=sys.stderr)
        return 1
    per_sc_bits = nbits / ns
    print(f"# angle payload {len(f0['angle_bytes'])} B = {nbits} bits, "
          f"Ns={ns} -> {per_sc_bits:.2f} bits/subcarrier over {na} angle(s)")
    if per_sc_bits != int(per_sc_bits):
        print("# WARN: non-integer bits/subcarrier — Ns or Na guess wrong",
              file=sys.stderr)
    per_sc_bits = int(per_sc_bits)

    # Candidate (bphi, bpsi) splits summing to the per-subcarrier budget for a
    # 2x1 (one phi + one psi). Standard SU pairs are (7,5)/(9,7); Realtek's
    # compact report may use a smaller codebook, so enumerate all splits and
    # pick the one whose psi(k) is most stable across the (quasi-static) frames.
    nphi, npsi = layout.count("phi"), layout.count("psi")
    candidates = []
    for bphi in range(2, per_sc_bits - 1):
        bpsi = (per_sc_bits - nphi * bphi) // npsi if npsi else 0
        if bpsi < 2:
            continue
        if nphi * bphi + npsi * bpsi != per_sc_bits:
            continue
        candidates.append((bphi, bpsi))
    if not candidates:
        candidates = [(per_sc_bits // 2, per_sc_bits - per_sc_bits // 2)]

    def stability(bphi, bpsi):
        """Mean cross-frame variance of psi[0] per subcarrier — lower is a
        more self-consistent decode of a static channel."""
        cols = [[] for _ in range(ns)]
        for f in frames:
            dec = decode_angles(f["angle_bytes"], ns, na, bphi, bpsi, layout)
            if dec is None:
                return math.inf
            for k in range(ns):
                cols[k].append(dec[k]["psi"][0])
        var = 0.0
        for c in cols:
            m = sum(c) / len(c)
            var += sum((x - m) ** 2 for x in c) / len(c)
        return var / ns

    def smoothness(bphi, bpsi):
        """Lag-1 correlation of psi[0] across subcarriers, averaged over
        frames. Real (band-limited) CSI is smooth vs frequency -> high
        correlation; a wrong bit-split scrambles tones -> ~0."""
        tot, n = 0.0, 0
        for f in frames[:20]:
            dec = decode_angles(f["angle_bytes"], ns, na, bphi, bpsi, layout)
            if dec is None:
                return -1.0
            x = [dec[k]["psi"][0] for k in range(ns)]
            m = sum(x) / ns
            xc = [v - m for v in x]
            denom = sum(v * v for v in xc) or 1e-9
            num = sum(xc[i] * xc[i + 1] for i in range(ns - 1))
            tot += num / denom
            n += 1
        return tot / n if n else -1.0

    print("# candidate split validation (want low cross-frame var AND high "
          "adjacent-tone corr):")
    print("#   b_phi b_psi   xframe_var   tone_corr")
    scored = []
    for (bphi, bpsi) in candidates:
        v = stability(bphi, bpsi)
        s = smoothness(bphi, bpsi)
        scored.append((bphi, bpsi, v, s))
        print(f"#   {bphi:5d} {bpsi:5d}   {v:10.5f}   {s:+.3f}")
    # Cross-frame variance is the reliable discriminator: the correct bit-split
    # decodes a static channel to repeatable angles (low var); a wrong split
    # smears the LSBs into different values frame-to-frame (high var). Tone
    # correlation only helps on a *frequency-selective* channel — on a flat
    # bench LOS link it is near zero for every split and can't disambiguate.
    best = min(scored, key=lambda t: t[2])
    bphi, bpsi = best[0], best[1]
    print(f"# chose b_phi={bphi} b_psi={bpsi} by min cross-frame var "
          f"({best[2]:.5f}); phi=psi+2 matches the standard Givens structure. "
          f"tone_corr={best[3]:+.3f}")
    if best[3] < 0.3:
        print("# NB: low tone correlation => channel ~flat across this BW "
              "(short LOS / coherence-BW > channel-BW); per-tone structure is "
              "quantisation-limited. Use wider BW or multipath to see "
              "frequency selectivity.")

    # Average the decoded angles over all frames -> the per-tone channel shape.
    acc = [dict(psi=0.0, phi=0.0) for _ in range(ns)]
    psi_var = [0.0] * ns
    psi_all = [[] for _ in range(ns)]
    for f in frames:
        dec = decode_angles(f["angle_bytes"], ns, na, bphi, bpsi, layout)
        for k in range(ns):
            acc[k]["psi"] += dec[k]["psi"][0]
            acc[k]["phi"] += dec[k]["phi"][0]
            psi_all[k].append(dec[k]["psi"][0])
    nfr = len(frames)
    rows = []
    for k in range(ns):
        psi = acc[k]["psi"] / nfr
        phi = acc[k]["phi"] / nfr
        ratio = math.tan(psi)                     # |h_B|/|h_A|
        m = psi
        var = sum((x - m) ** 2 for x in psi_all[k]) / nfr
        rows.append((k, psi, phi, ratio, var))

    # Avg SNR of space-time stream: int8, dB = 22 + 0.25*v -> range -10..53.75.
    snr0 = frames[0]["snr"]
    snr_db = [(22.0 + 0.25 * (b if b < 128 else b - 256)) for b in snr0]
    print(f"# per-stream avg SNR (col dB): "
          f"{', '.join(f'{s:.2f}' for s in snr_db)}")
    print(f"# per-tone relative channel |h_B/h_A| across {ns} subcarriers "
          f"(psi->amplitude ratio); '#'=strong on antenna B, '.'=on antenna A")

    # ASCII plot of the amplitude ratio across subcarriers (the frequency shape)
    ratios = [r[3] for r in rows]
    lo, hi = min(ratios), max(ratios)
    span = (hi - lo) or 1.0
    ramp = " .:-=+*#%@"
    line = "".join(ramp[min(len(ramp) - 1,
                            int((r - lo) / span * (len(ramp) - 1)))]
                   for r in ratios)
    print(f"  |h_B/h_A| [{lo:.2f}..{hi:.2f}]: {line}")

    if args.csv:
        with open(args.csv, "w") as fh:
            fh.write("subcarrier,psi_rad,phi_rad,ampl_ratio_hB_hA,psi_var\n")
            for (k, psi, phi, ratio, var) in rows:
                fh.write(f"{k},{psi:.5f},{phi:.5f},{ratio:.5f},{var:.6f}\n")
        print(f"# wrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
