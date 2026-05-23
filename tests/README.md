# devourer regression test rig

Cross-driver matrix test that compares this project's userspace stack
against the kernel-driver baseline (aircrack-ng / mainline `rtw88`) for
both TX and RX on plugged-in USB Wi-Fi adapters.

```
                  TX = devourer       TX = kernel
RX = devourer     end-to-end devourer  does dvr RX a kernel-TX frame?
RX = kernel       does dvr emit valid  baseline / rig sanity check
                  frames?
```

Each cell injects/receives the canonical beacon (SA `57:42:75:05:d6:00`,
matching `txdemo/main.cpp`) for `--duration` seconds and counts hits.
The baseline cell runs first — if it fails the rig itself is broken
(channel busy, antennas, kernel driver mismatch) and the remaining cells
are skipped.

## Prerequisites

- 2 supported USB Wi-Fi adapters plugged into the same host
- devourer built (`build/WiFiDriverDemo`, `build/WiFiDriverTxDemo`)
- Kernel driver(s) for the adapter(s) installed and `modprobe`-able
  (rtl8812au/rtl8814au from aircrack-ng or your distro's `rtw88` for
  mainline). The script doesn't care which — it queries sysfs for whatever
  is bound.
- Python 3.9+ with `scapy` available (`pip install scapy` or your distro's
  `python3-scapy`)
- `iw`, `tcpdump`, `ip` on PATH
- Passwordless `sudo`, or run the script directly as root
- NetworkManager users: stop NM for the duration of the test, or
  `nmcli device set <iface> managed no` on the test interfaces before
  running. (NM will fight you for the monitor-mode wlan iface otherwise.)

The script does a preflight check and prints distro-agnostic install
hints for anything missing.

## Usage

```bash
sudo python3 tests/regress.py --channel 100
```

Auto-detects the first two supported adapters via sysfs. To pick
specific ones:

```bash
sudo python3 tests/regress.py \
    --tx-pid 0x8812 --rx-pid 0x8813 --channel 100 --duration 20
```

Output is a markdown table printed to stdout — paste into PR comments
or save with `tee`:

```
## Regression matrix — channel 100, 2026-05-23 12:34:56

- TX adapter: `0bda:8812` (RTL8812AU)
- RX adapter: `0bda:8813` (RTL8814AU)
- Cell duration: 15s
- Pass threshold: ≥ 5 hits

|   | TX = devourer | TX = kernel |
|---|---|---|
| RX = devourer | 42 hits / 7500 TX / 15s ✓ | 35 hits / 7500 TX / 15s ✓ |
| RX = kernel | 31 hits / 7500 TX / 15s ✓ | 47 hits / 7500 TX / 15s ✓ |
```

For debugging a specific cell that failed, re-run with `--keep-logs` —
per-cell stdout/stderr logs are symlinked at
`/tmp/devourer-regress-last/`.

## Supported adapters

Listed in `SUPPORTED_DUTS` at the top of `regress.py`. Extend the dict
to add new chipsets — the rest of the script is chipset-agnostic.

## Channel selection

The default `--channel 36` is a 5GHz channel that's typically quiet,
which means hit counts will be low but stable. For high-confidence
runs, pick a channel where your nearest AP is actively transmitting
(check via `iw dev wlan0 scan | grep -E "freq|SSID"` on a separate
device).

## VM-readiness

The kernel-cell shell-outs go through `run_kernel_cmd()` in `regress.py`.
Today it's `subprocess.run` (local). To migrate the kernel side into a
pinned-kernel VM — recommended once host-kernel upgrades start breaking
the out-of-tree aircrack-ng driver — replace `run_kernel_cmd` with an
`ssh user@trainer-vm sudo` wrapper and arrange USB hot-plug passthrough
into the VM via libvirt (`virsh attach-device` with a `<hostdev>` USB
spec). The matrix orchestrator doesn't need to change.

## Known gaps

- Tests "signal of life", not throughput. Hit counts vary 5-20× run-over-
  run depending on ambient RF — thresholds are deliberately generous.
- Per-cell startup time is ~10s (devourer fwdl + warmup). 4 cells × ~25s
  ≈ 100s per matrix run. Fine for manual runs, would be annoying for CI.
- No support yet for >2 adapters. To extend, add a pairing loop in
  `main()` that runs the 4-cell matrix per chipset pair.
- Kernel TX side uses scapy at 500 fps. If your kernel driver's
  injection rate is the bottleneck on a given chip, lower
  `--interval` in `inject_beacon.py`.
