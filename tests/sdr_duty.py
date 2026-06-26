#!/usr/bin/env python3
"""Ceiling-free on-air TX measurement via USRP energy/duty-cycle.

A monitor-mode sniffer caps how many frames/s it can count (~2900 fps on an
RTL8814AU here), so it silently undercounts a fast transmitter. This measures
the **channel occupancy (duty cycle)** directly with the USRP: on a clean
channel the only signal is the device under test, so the fraction of time the
received power is above the noise floor is exactly the TX's airtime occupancy.

  on_air_Mbps ~= duty * phy_rate_mbps        (raw PHY bits on air)

Reports duty + an estimated on-air rate for a given HT MCS/bandwidth/GI. Run it
while a transmitter floods a clean 5 GHz channel; compare devourer vs wfb_tx vs
inject_beacon at the same config — higher duty = more on air.

  sudo python3 sdr_duty.py --freq 5745e6 --secs 4 --mcs 7 --bw 20
"""
from __future__ import annotations
import argparse, math, sys, time
import numpy as np

try:
    import uhd
except ImportError:
    sys.stderr.write("need UHD python (system pkg) — run with system python3\n")
    raise

# HT single-stream PHY rates (Mbps): [20MHz long GI, 20 short GI, 40 long, 40 short]
HT_RATE = {
    0: (6.5, 7.2, 13.5, 15.0), 1: (13.0, 14.4, 27.0, 30.0),
    2: (19.5, 21.7, 40.5, 45.0), 3: (26.0, 28.9, 54.0, 60.0),
    4: (39.0, 43.3, 81.0, 90.0), 5: (52.0, 57.8, 108.0, 120.0),
    6: (58.5, 65.0, 121.5, 135.0), 7: (65.0, 72.2, 135.0, 150.0),
}


def phy_rate(mcs, bw, short_gi):
    col = (0 if bw == 20 else 2) + (1 if short_gi else 0)
    return HT_RATE[mcs][col]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--freq", type=float, default=5745e6)
    ap.add_argument("--rate", type=float, default=25e6, help="USRP sample rate")
    ap.add_argument("--gain", type=float, default=40.0)
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--mcs", type=int, default=7)
    ap.add_argument("--bw", type=int, default=20)
    ap.add_argument("--short-gi", action="store_true")
    ap.add_argument("--win", type=int, default=20, help="samples per energy bin")
    ap.add_argument("--margin-db", type=float, default=8.0,
                    help="threshold = noise_floor + margin")
    ap.add_argument("--noise-db", type=float, default=None,
                    help="fixed idle noise-floor (dB) for the threshold. Auto "
                         "10th-pct fails when the channel is ~saturated (the "
                         "low tail becomes signal), so calibrate once on an idle "
                         "channel and pass it here.")
    args = ap.parse_args()

    usrp = uhd.usrp.MultiUSRP("")
    usrp.set_rx_rate(args.rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(args.freq))
    usrp.set_rx_gain(args.gain)
    st = uhd.usrp.StreamArgs("fc32", "sc16"); st.channels = [0]
    rx = usrp.get_rx_stream(st)
    buf = np.zeros((1, rx.get_max_num_samps()), dtype=np.complex64)
    md = uhd.types.RXMetadata()
    cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    cmd.stream_now = True
    rx.issue_stream_cmd(cmd)

    # Collect per-bin power (mean |x|^2 over `win` samples).
    powers = []
    t_end = time.monotonic() + args.secs
    try:
        while time.monotonic() < t_end:
            n = rx.recv(buf, md, 1.0)
            if md.error_code != uhd.types.RXMetadataErrorCode.none or n <= 0:
                continue
            x = buf[0, :n]
            p = (x.real.astype(np.float32) ** 2 + x.imag.astype(np.float32) ** 2)
            nb = (n // args.win) * args.win
            if nb:
                powers.append(p[:nb].reshape(-1, args.win).mean(axis=1))
    finally:
        rx.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))
    if not powers:
        print("sdr-duty: no samples"); return 1
    pw = np.concatenate(powers)
    pdb = 10 * np.log10(pw + 1e-12)
    if args.noise_db is not None:
        noise = args.noise_db
    else:
        # Auto: works only when the channel has clear gaps (duty < ~90%). The
        # 1st percentile is the noise floor on a clean, gappy channel.
        noise = np.percentile(pdb, 1)
    thr = noise + args.margin_db
    duty = float(np.mean(pdb > thr))
    pr = phy_rate(args.mcs, args.bw, args.short_gi)
    print(f"sdr-duty: duty={duty*100:.1f}%  noise={noise:.1f}dB thr={thr:.1f}dB  "
          f"phy={pr}Mbps(MCS{args.mcs}/{args.bw}MHz{'/sgi' if args.short_gi else ''})  "
          f"on_air~={duty*pr:.1f}Mbps  bins={len(pw)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
