#!/usr/bin/env bash
# Kernel-driver ground truth for 8812EU narrowband: measure the ACTUAL
# libc0607 5mhz_bw driver's on-air spectrum at 5/10/20 MHz with the B210, from
# the same host harness used for devourer. Driver source: the
# reference/rtl88x2eu-5mhz submodule (libc0607/rtl88x2eu-20230815 @ 5mhz_bw).
#
# The driver runs inside the pinned-kernel VM (Ubuntu 22.04 / 5.15) with the
# 8812EU virsh-attached; injection is tests/inject_beacon.py over ssh. This is
# the experiment that settles whether the "mirror" leakage in the OpenHD 5 MHz
# notes reproduces on the kernel driver on our unit — devourer's port of the
# same register sequence measures clean, but that is NOT evidence about the
# kernel driver itself.
#
# Prerequisites (manual, one-time per session):
#   - VM running, 8812EU attached (virsh attach-device), 8812eu.ko loaded
#   - iface in monitor mode: iw set type monitor; ip link up
#   - /tmp/inject_beacon.py copied into the VM
#
#   sudo VM_SSH=josephnef@10.216.129.144 IFACE=wlx9803cfcfa449 \
#       tests/jaguar3_eu_kernel_5mhz_sdr.sh
set -u
cd "$(dirname "$0")/.."

VM_SSH=${VM_SSH:?set VM_SSH=user@vm-ip}
IFACE=${IFACE:?set IFACE=<monitor iface in VM>}
PROCDIR=${PROCDIR:-rtl88x2eu}   # rtl88x2eu (libc0607) / rtl88x2eu_ohd (OpenHD)
FREQ=${FREQ:-5220e6}     # ch44 — same channel the devourer runs used
CHAN=${CHAN:-5220}
RATE=46.08e6
OUT=/tmp/j3_eu_kernel_nb
rm -rf "$OUT" 2>/dev/null || sudo rm -rf "$OUT"
mkdir -p "$OUT"

vmssh() { ssh -o BatchMode=yes "$VM_SSH" "$@"; }

probe() { # label
  python3 tests/sdr_tx_probe.py --freq "$FREQ" --rate "$RATE" --gain 50 \
      --nsamps 12e6 --label "$1" --psd-out "$OUT/$1.npy" 2>&1 \
      | grep '\[sdr' || true
}

cleanup() { vmssh "sudo pkill -f inject_beacon" 2>/dev/null || true; }
trap cleanup EXIT

echo "=== baseline: kernel driver idle (ambient PSD) ==="
probe baseline

for BW in 20 10 5; do
  echo "=== kernel TX at ${BW} MHz ==="
  IWBW="${BW}MHz"; [ "$BW" = 20 ] && IWBW="HT20"   # iw has no "20MHz" token
  vmssh "sudo iw dev $IFACE set freq $CHAN $IWBW" || {
    echo "  !! set freq $CHAN $IWBW failed"; continue; }
  # ground truth from the driver, not cfg80211
  vmssh "grep -o 'ch=.*' /proc/net/$PROCDIR/$IFACE/monitor" | sed 's/^/    /'
  vmssh "sudo nohup python3 /tmp/inject_beacon.py --iface $IFACE \
      --duration 45 --interval 0.002 --rate 6 >/tmp/inject_${BW}.log 2>&1 &"
  sleep 4
  probe "tx${BW}"
  sleep 3
  probe "tx${BW}_b"
  cleanup
  sleep 2
done

echo "=== differential-PSD analysis (1 MHz bins, contiguous -10dB width) ==="
python3 - "$OUT" <<'PY'
import sys, numpy as np
out = sys.argv[1]
base = np.load(f"{out}/baseline.npy"); bp = base[1]

def analyse(label):
    a = np.load(f"{out}/{label}.npy"); f, p = a[0], a[1]
    diff = np.clip(p - bp, 0, None)
    sm = np.convolve(diff, np.ones(33)/33, mode='same')
    if sm.max() <= 0:
        return None
    ipk = int(np.argmax(sm)); pk = sm[ipk]; thr = pk/10.0
    lo = ipk
    while lo > 0 and sm[lo-1] >= thr: lo -= 1
    hi = ipk
    while hi < len(sm)-1 and sm[hi+1] >= thr: hi += 1
    fctr = (f[0]+f[-1])/2
    # 1 MHz-binned profile, printed for spur inspection
    nb = 46
    edges = np.linspace(f[0], f[-1], nb+1)
    binp = np.array([diff[(f>=edges[i])&(f<edges[i+1])].sum() for i in range(nb)])
    mx = binp.max() if binp.max() > 0 else 1
    prof = [(float((edges[i]+edges[i+1])/2 - fctr),
             float(10*np.log10(binp[i]/mx)) if binp[i] > 0 else -99.0)
            for i in range(nb)]
    return dict(w=(f[hi]-f[lo]), off=(f[ipk]-fctr), prof=prof, total=diff.sum())

for bw in (20, 10, 5):
    for lab in (f"tx{bw}", f"tx{bw}_b"):
        r = analyse(lab)
        if r is None:
            print(f"{lab}: no signal"); continue
        print(f"--- {lab}: contiguous -10dB width {r['w']/1e6:.2f} MHz, "
              f"peak at {r['off']/1e6:+.2f} MHz, total {r['total']:.2e} ---")
        for c, db in r["prof"]:
            if db > -25:
                print(f"  {c/1e6:+7.2f} MHz  {db:6.1f} dB {'#'*max(0,int(25+db))}")
PY
echo "PSDs kept in $OUT"
