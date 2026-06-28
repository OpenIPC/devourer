#!/usr/bin/env python3
"""Build the adaptive-link channel table from on-air measurements -> link_calib.json.

link_model.py keys delivery on a per-(MCS, SNR-bucket) channel
    {corrupt_rate, n_sub, survivor_hist}
— exactly what ~/git/sdr2wifi/survivor_hist.py and fused_fec_link.FusedFecReceiver
already emit. The measurement procedure (per MCS):
    for each tests/sdr_interferer.py --tx-gain setpoint:
        run the chip TX at that MCS + the VRX (chip or SDR);
        read the VRX-reported mean SNR and the per-corrupt-frame survivor histogram.
Append each as one JSON line to a measurements file:
    {"mcs": 5, "snr_db": 18.3, "corrupt_rate": 0.31, "n_sub": 10,
     "survivor_hist": {"0": 15, ... "10": 195}}

This consumes that JSONL, buckets by SNR, and writes the channel table + per-MCS
waterfall centres (back-fit from the corrupt_rate=0.5 crossing) that link_model
overlays.

  ./calibrate_link.py --measurements meas.jsonl --out link_calib.json --bucket 1.0
"""
import argparse
import json


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--measurements", required=True, help="JSONL of on-air channel measurements")
    p.add_argument("--out", default="link_calib.json")
    p.add_argument("--bucket", type=float, default=1.0, help="SNR bucket width (dB)")
    a = p.parse_args()

    channels = {}                 # "mcs:snr_bucket" -> channel dict
    by_mcs = {}                   # mcs -> list[(snr, corrupt_rate)]
    with open(a.measurements) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = json.loads(line)
            mcs = int(m["mcs"]); snr = float(m["snr_db"])
            b = round(snr / a.bucket) * a.bucket
            channels[f"{mcs}:{b:g}"] = {
                "corrupt_rate": float(m["corrupt_rate"]),
                "n_sub": int(m.get("n_sub", 10)),
                "survivor_hist": {int(k): int(v) for k, v in m["survivor_hist"].items()},
            }
            by_mcs.setdefault(mcs, []).append((snr, float(m["corrupt_rate"])))

    # Back-fit each MCS's waterfall centre = SNR where corrupt_rate crosses 0.5
    centers = {}
    for mcs, pts in by_mcs.items():
        pts.sort()
        center = None
        for (s0, c0), (s1, c1) in zip(pts, pts[1:]):
            if (c0 - 0.5) * (c1 - 0.5) <= 0 and c1 != c0:
                center = s0 + (0.5 - c0) * (s1 - s0) / (c1 - c0)
                break
        if center is not None:
            centers[str(mcs)] = round(center, 2)

    out = {"source": "measured", "bucket_db": a.bucket,
           "centers": centers, "channels": channels}
    with open(a.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[calibrate-link] wrote {a.out}: {len(channels)} channels, "
          f"{len(centers)} MCS centres {centers}")


if __name__ == "__main__":
    main()
