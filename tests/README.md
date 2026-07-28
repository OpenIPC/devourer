# devourer regression test rig

Cross-driver matrix test that compares this project's userspace stack
against the kernel-driver baseline for both TX and RX on plugged-in USB
Wi-Fi adapters. An optional on-air sniffer (`--sniffer-iface`) can
attend each cell and report what actually flew on the air — adding
unambiguous TX-vs-RX failure attribution on top of the bare hit count.

```
                  TX = devourer       TX = kernel
RX = devourer     end-to-end devourer  does dvr RX a kernel-TX frame?
RX = kernel       does dvr emit valid  baseline / rig sanity check
                  frames?
```

Each cell injects/receives the canonical beacon (SA `57:42:75:05:d6:00`,
matching `examples/tx/main.cpp`) for `--duration` seconds and counts hits.
The baseline cell runs first — if it fails the rig itself is broken and
the remaining cells are skipped (override with `--no-baseline-abort`).

## Failure attribution via the sniffer

With `--sniffer-iface` set (see [On-air sniffer](#--sniffer-iface--on-air-encoding-verification--attribution)
below), every cell gains a third independent witness:

| sniffer caught N? | RX counted N? | diagnosis                          |
|-------------------|---------------|------------------------------------|
| no                | no            | TX side broken (frame never flew)  |
| yes               | no            | RX side broken (frame flew, missed) |
| yes               | yes           | cell passes                         |
| no                | yes           | rig fault (impossible)              |

This is the primary reason to attach a sniffer: it turns the 2×2 matrix
from "did the pair work end-to-end?" into "which side broke?". The same
attribution holds whether the other side is `devourer` or `kernel` — so
**a sniffer-equipped run does not need a working kernel driver for the
DUT** to diagnose a devourer-side regression. The VM mode (below) is
only needed when you specifically want the kernel-vs-kernel and
kernel-vs-devourer **interop** rows for a chip whose host kernel driver
doesn't build.

## Two run modes

These describe where the *kernel-side* cells run. With a sniffer
attached, the kernel side is needed for interop rows but not for
attributing devourer-side failures — see
[Failure attribution via the sniffer](#failure-attribution-via-the-sniffer).

### Local mode

The kernel-side cells run against whatever driver is bound to the DUTs on
the **host** (mainline `rtw88_*` or whatever's loaded). Cheap to set up
but limited to drivers that build cleanly against the host kernel — that's
a moving target as kernels evolve, especially for the out-of-tree
`aircrack-ng/rtl8812au` driver.

```bash
sudo python3 tests/regress.py
# default --channel 6; pass --channel 36 / --channel 100 to exercise 5GHz
```

### VM mode (recommended)

The kernel-side cells run inside a **pinned-kernel libvirt VM** that has
the OOT `aircrack-ng/rtl8812au` driver built and loaded. DUTs are
transferred between host and VM per cell via `virsh attach-device` /
`detach-device`. The VM's kernel never moves so the driver never breaks.

Provision the VM once with the included script (Ubuntu 22.04 LTS,
kernel 5.15 — where aircrack-ng's driver builds without patches):

```bash
sudo tests/setup_vm.sh                    # provision; ~5-10 min
sudo tests/setup_vm.sh --status           # show VM IP, ssh hint
```

Then run the matrix in VM mode:

```bash
sudo python3 tests/regress.py \
    --vm-name devourer-testrig \
    --vm-ssh <user>@<VM-IP-from-status>
# Defaults to --channel 6 (2.4GHz). Re-run with --channel 36 / 100 to
# also exercise 5GHz.
```

VM mode is what unblocks chipsets where the host kernel driver doesn't
work — e.g. RTL8814AU, where mainline `rtw88_8814au` currently fails to
probe on kernels 6.15+ (`failed to download firmware`, `error -22`), but
`aircrack-ng/rtl8812au` claims it cleanly on the pinned kernel 5.15.

## Prerequisites

### On the host (both modes)

- 2 supported USB Wi-Fi adapters plugged in
- devourer built (`build/rxdemo`, `build/txdemo`)
- Python 3.9+ with `scapy` (`pip install scapy` or `python3-scapy`)
- `iw`, `tcpdump`, `ip` on PATH
- Passwordless `sudo`, or run directly as root

### Adaptive-hopset validation (`hopset_adaptive_jammer.sh`)

Three radios — a transmitting authority, a lockstep receiver running the
exclusion policy, and a B210 interferer. `MODE` selects the scenario
(`parked`, `sense`, `failsafe`, `herding`, `hidden`, `mirror`, `fusionmatrix`,
`policycost`, `prepost`, `overhead`); the mode arms whatever it needs, so
`MODE=sense` really does sense and `MODE=fusionmatrix` gives every row the same
sensing windows.

Two things about how it measures, because both changed what the numbers said:

- Every mode reports **FEC delivery** — real RS-coded bodies from
  `tools/precoder` through `DEVOURER_TX_STDIN`, decoded by the salvage path —
  beside the marker-derived dwell proxy. `tests/fec_metrics.py` owns that
  arithmetic and is shared with `jammer_resilience.py`; it also prints the
  fraction of the window the receiver held lockstep, which is the number that
  catches a receiver scoring 1.0 on the half of a window it was present for.
- A measurement window opens only once the receiver reports tracking, and
  closes against the transmitter's own body count at both instants. A window
  that opens at process start is mostly chip bring-up and acquisition: one
  70 s phase read 0.30 FEC delivery whose settled part ran at 0.90.

`FEC=0` falls back to the proxy alone. The payload is generated once per size
and cached in `/tmp`; it must outlast the longest phase, since looping it would
re-use the outer code's block ids and the decoder would merge two passes into
one block.

### Choosing the SDR (benches with more than one radio)

Every UHD tool here (`sdr_duty.py`, `sdr_interferer.py`, `hop_rx_probe.py`,
`sdr_follower_jammer.py`, …) resolves its radio through `uhd_select.py`:
an explicit `--args`, else `DEVOURER_UHD_ARGS`, else the untracked
`tests/.uhd_args`, else the only device present. With two or more radios and
no selection it **fails with the serial list** rather than picking one — B210
variants share USB id `2500:0020`, so a silent pick means half your runs
measure the wrong antenna and nothing in the log says so.

Set it once per bench. Use the file, not the variable: nearly every harness
invokes its SDR tool through `sudo`, which scrubs the environment.

```sh
uhd_find_devices                          # read off the serials
echo serial=XXXXXXX > tests/.uhd_args     # survives sudo; gitignored
python3 tests/uhd_select.py               # prints what would be opened
```

### For local mode

- Kernel driver(s) installed and `modprobe`-able for your DUTs (rtw88 or
  aircrack-ng — script auto-detects whatever's bound via sysfs)
- NetworkManager users: stop NM, or `nmcli device set <iface> managed no`
  on the test interfaces

### For VM mode (in addition)

- `libvirtd` + `virsh` + `virt-install` on the host
- `xorriso` (for the cloud-init seed ISO that `setup_vm.sh` generates)
- An Ubuntu 22.04 cloud image at `/var/lib/libvirt/images/jammy-base.qcow2`
  (download from <https://cloud-images.ubuntu.com/jammy/current/>)
- Working USB hot-plug on libvirt (`xhci` controller; `setup_vm.sh` adds it)
- The host user's SSH key in `~/.ssh/id_rsa.pub` (or set `SSH_PUBKEY=...`
  before `setup_vm.sh`) — gets baked into the VM's user account
  (defaults to your invoking user; override with `VM_USER=foo`)

The script does a preflight check and prints distro-agnostic install
hints for anything missing.

## Output

Markdown table to stdout, ready to paste into PR comments:

```
## Regression matrix — channel 100, 2026-05-23 13:22:14

- TX adapter: `0bda:8812` (RTL8812AU)
- RX adapter: `0bda:8813` (RTL8814AU)
- Kernel host: VM devourer-testrig via <user>@<VM-IP>
- Cell duration: 10s
- Pass threshold: ≥ 3 hits

|   | TX = devourer | TX = kernel |
|---|---|---|
| RX = devourer | 0 hits / 4500 TX ✗ | 0 hits / 258 TX ✗ |
| RX = kernel | 4172 hits / 4500 TX ✓ | 229 hits / 259 TX ✓ |
```

Pass/fail per cell on hit-count threshold (default ≥ 1 — generous because
air interference makes absolute counts unreliable). Bump for higher-
confidence runs on a quieter channel.

For debugging a specific cell that failed, re-run with `--keep-logs` —
per-cell stdout/stderr logs end up at `/tmp/devourer-regress-last/`.

## CLI knobs

- `--channel N` — Wi-Fi channel for both adapters (default `6`). Band
  behaviour can differ per chip — don't assume a single-channel matrix is
  comprehensive; re-run with `--channel 36` / `--channel 100` to cover
  5GHz. NB: with the RTL8814AU on the kernel-TX side, kernel-TX cells
  read 0 at every channel — `88XXau` host-push beacon injection (what
  `inject_beacon.py` does) doesn't emit on that driver; judge 8814 TX by
  the devourer-TX cells.
- `--duration SECONDS` — per-cell injection/measurement window (default 15)
- `--pass-threshold N` — min hits to pass (default 1)
- `--tx-pid 0xNNNN` / `--rx-pid 0xNNNN` — pick specific DUTs (defaults to
  the first two auto-detected)
- `--no-baseline-abort` — run all 4 cells even if kernel-kernel fails
  (useful when one chipset has no working kernel driver on the host)
- `--no-rf-reset` — skip the per-cell USB port-level authorize-cycle.
  Default behaviour is to deauthorize / reauthorize each DUT's USB port
  before every cell, forcing a true chip-power cycle. Required because
  `libusb_reset_device` and sysfs unbind/rebind do NOT reset Realtek RF
  analog state — RF reg writes survive both. Without the cycle, canary
  captures and matrix cells inherit RF state (band-select bits, IQK
  coefficients, AGC indices) from the previous run on the same DUT.
  Adds ~3 s per DUT per cell; disable only for trivial smoke runs.
- `--vm-name NAME` / `--vm-ssh USER@HOST` — enter VM mode
- `--keep-logs` — symlink the temp log dir at `/tmp/devourer-regress-last`

Environment variable equivalents: `DEVOURER_VM_NAME`, `DEVOURER_VM_SSH`,
`DEVOURER_SNIFFER_IFACE`.

### `--sniffer-iface` — on-air encoding verification + attribution

When set, each matrix cell captures on a third (host-local) monitor
iface and reports the decoded radiotap encoding distribution alongside
the hit count. The sniffer also gives unambiguous TX-vs-RX failure
attribution (see [Failure attribution via the sniffer](#failure-attribution-via-the-sniffer)
above). Composes with `--full-matrix` and `--encoding-matrix`.

#### Per-band sniffer assignment

A single sniffer chip can't cover the whole spectrum. Pick by band:

| Band                 | Sniffer chip            | Why                                  |
|----------------------|-------------------------|--------------------------------------|
| 2.4 GHz (ch 1–13)    | **AR9271** (`ath9k_htc`)| Vanilla radiotap, no driver-side flag filtering. Canonical kernel-MAC reference for HT/legacy capture. 2.4 G only. |
| 5 GHz (UNII-1/2/3)   | **RTL8832AU** (`8852au`, lwfinger OOT) | Covers UNII-1 (ch36–48), UNII-2 DFS (ch52–144), UNII-3 (ch149+). Out-of-tree DKMS; recent kernels may need build patches. **Caveat — host stall observed once** when used as `--sniffer-iface` for `--full-matrix --channel 36` on kernel 6.18 with three concurrent Realtek DUTs on the same xhci bus; hard CPU stall, no oops/NMI trace. Cause not isolated (xhci bus contention vs. lwfinger 5 GHz monitor path); if you see a stall, drop `--sniffer-iface` for 5 G runs and use compositional attribution across cells. |

Set the iface per run — there is no auto-band switching:

```bash
# 2.4 GHz channel — use AR9271
sudo python3 tests/regress.py --full-matrix --channel 6 \
    --sniffer-iface wlp0s20f0u14

# 5 GHz channel — use RTL8832AU
sudo python3 tests/regress.py --full-matrix --channel 36 \
    --sniffer-iface wlp0s20f0u13
sudo python3 tests/regress.py --full-matrix --channel 100 \
    --sniffer-iface wlp0s20f0u13
```

(Replace iface names with `iw dev` output on your rig.) `regress.py`
puts the sniffer iface into monitor mode and tunes it to `--channel`
itself — pre-configuration is not required.

Per-cell output gains a `↪ sniffer: N frames — <encoding>=N, ...` line
showing what actually flew. If the `--ldpc` injection comes back tagged
as BCC, mac80211 / the OOT driver stripped the flag before the air —
the chip-side RX never had to refuse an LDPC frame, so any
`k/d`-row-flat result for LDPC means the test setup, not the chip.

The sniffer is host-only and never moved through the VM USB
passthrough. The same helper, run standalone outside the matrix, is
`tests/sniff_air.py` (see below).

#### Why this displaces VM mode for most devourer work

The sniffer attributes failure to TX or RX without needing the *other*
side to be a working kernel driver. So if you're investigating a
devourer-side regression (TX or RX), pair-with-anything + sniffer is
sufficient — even pair-with-devourer cells stay diagnosable because the
sniffer is the third-party witness. VM mode is now only required when
you specifically need the cross-driver **interop** rows on a chip whose
host kernel driver doesn't build (notably RTL8814AU on kernels 6.15+
where mainline `rtw88_8814au` fails to download firmware).

## Specialized modes

The default invocation runs the 4-cell matrix on one ordered (TX, RX)
pair. Two additional modes extend coverage along different axes.

### `--full-matrix`: cross-chipset interop

Iterates every ordered (TX, RX) pair of plugged DUTs across all four
driver-side combinations and emits four NxN tables. For N adapters,
N×(N-1)×4 cells; ~16 min for N=3 in VM mode. Useful for catching
cross-chipset regressions in PRs that touch shared HAL code.

```bash
# 5 GHz UNII-2 — sniffer attribution + VM-side kernel interop
sudo python3 tests/regress.py --full-matrix --channel 100 \
    --sniffer-iface wlp0s20f0u13 \
    --vm-name devourer-testrig --vm-ssh <user>@<VM-IP>
```

For devourer-only regressions (skip the cross-driver interop rows,
keep attribution), `--no-baseline-abort` lets the matrix proceed
without a working kernel side; the sniffer column still attributes
every cell.

### `--encoding-matrix`: chip-specific radiotap encoding asymmetries

For one ordered TX→RX pair, iterates (driver mode × radiotap encoding
flags). 16 cells per run (~10 min in VM mode). Designed to surface
chip-specific RX asymmetries that the default and full matrices miss
because they only exercise the chip's default encoding.

```bash
sudo python3 tests/regress.py --encoding-matrix \
    --tx-pid 0x8813 --rx-pid 0x0120 --channel 100 \
    --vm-name devourer-testrig --vm-ssh <user>@<VM-IP>
```

`--modes` restricts the driver-mode rows to a comma list of `txside:rxside`
entries — LDPC/STBC truth-table runs want `--modes devourer:devourer`, since
the kernel-TX rows are never authoritative for those bits (the kernel driver
strips them; see the caveat below) and the kernel rows double the runtime:

```bash
sudo python3 tests/regress.py --encoding-matrix --modes devourer:devourer \
    --tx-pid 0x8812 --rx-pid 0x012d --channel 6 --duration 8
```

Each devourer-RX cell's `rx.txhit` events carry the received frame's
`rate`/`bw`/`stbc`/`ldpc` so a pass proves the encoding actually flew (a pass
with `ldpc:0` means the TX fell back to BCC, not that the RX decoded LDPC).
Exception: the RTL8814AU reports `ldpc:0` even on LDPC frames it decoded —
the chip has no per-frame LDPC indicator (`AdapterCaps.ldpc_rx_flag=0`);
judge its RX by hit count, its TX by the paired 8812AU's flag.

Encoding combos iterated by default — 6 cells per driver mode × 4 driver
modes = 24 cells total per run:

| Combo | Radiotap bit | Notes |
|---|---|---|
| `HT-BCC` | 19 (MCS info) | MCS 1, BCC FEC, 20 MHz |
| `HT-LDPC` | 19 | MCS 1, FEC=LDPC |
| `HT-STBC=1` | 19 | MCS 1, 1 STBC stream |
| `HT-LDPC+STBC` | 19 | MCS 1, FEC=LDPC + 1 STBC stream |
| `VHT-BCC` | 21 (VHT info) | VHT MCS 0, NSS 1, BCC, 20 MHz |
| `VHT-LDPC` | 21 | VHT MCS 0, NSS 1, FEC=LDPC |

VHT (802.11ac) coverage exists because some chips' LDPC decoder limitation
is on the VHT path only — the silicon's HT-LDPC and VHT-LDPC are separate
blocks. RTL8821AU is the motivating case (HT-LDPC tests pass, VHT-LDPC
reportedly fails at RX). Add more in `ENCODING_COMBOS` at the top of
`regress.py` if you need MCS 7, 40/80 MHz, or higher NSS.

The underlying knobs are also usable standalone for one-off targeted TX:

- **Devourer TX:** `DEVOURER_TX_RATE=<rate>[/<bw>][/SGI][/LDPC][/STBC]` read by
  `txdemo` (parsed into a `devourer::TxMode`, applied via
  `RtlJaguarDevice::SetTxMode`). `<rate>` = `6M`..`54M` (legacy) | `MCS0`..`MCS31`
  (HT) | `VHT1SS_MCS0`..`VHT4SS_MCS9` (VHT); `<bw>` = `20|40|80|160`. Examples:
  `MCS7/40/SGI`, `VHT2SS_MCS3/80/LDPC`.
- **Kernel-side scapy TX:** `--mcs N` / `--ldpc` / `--stbc N` / `--bandwidth
  20|40|80|160` on `tests/inject_beacon.py`, plus `--vht` / `--vht-mcs N`
  / `--vht-nss N` for VHT mode.

#### Caveat: kernel-TX encoding flags may not always reach the air

mac80211 + the `aircrack-ng/88XXau` driver don't necessarily honour the
radiotap MCS flags on TX — the chip's own rate-selection logic may
override them. So the `k/k` and `k/d` rows of the table reflect what
the *kernel* driver chose to transmit, which may collapse encoding
columns onto the chip's default.

To prove what actually flew, run `tests/sniff_air.py` on a third
adapter (AR9271 is the canonical sniffer — it speaks vanilla radiotap
without driver-side filtering) on the same channel, in parallel with
the matrix. Output reports each captured frame's decoded radiotap
MCS / VHT info, including LDPC bit + STBC streams. If a `--ldpc`
injection comes back tagged as BCC, the kernel stripped it before the
air.

```bash
sudo python3 tests/sniff_air.py --iface wlan0mon --channel 100 \
    --duration 60
```

The `d/k` and `d/d` rows are not affected by the kernel-TX caveat —
`txdemo` writes the radiotap header directly into the
chip's bulk-OUT buffer, so the `DEVOURER_TX_*` env vars are ground
truth for what flies on devourer TX.

### `ldpc_waterfall.sh`: measured LDPC coding gain

Traces delivery-vs-TX-power waterfalls for the same MCS with BCC vs LDPC
coding and reads the horizontal shift = coding gain in dB. The emitter steps
a flat TXAGC index per point (`DEVOURER_TX_PWR`, 0.5 dB/step Jaguar1/2,
0.25 dB Jaguar3); the ground counts canonical-SA FCS-clean frames per
`rx.energy` window (`DEVOURER_RX_AGG_SA=canon`); PER denominators come from
the emitter's exact final `tx.stats`.

```bash
sudo tests/ldpc_waterfall.sh --emit-pid 0x8812 --ground-pid 0x012d \
    --ground-vid 0x2357 --channel 6 --rate MCS7 --pwr-start 22 --pwr-stop 52
python3 tests/ldpc_waterfall.py /tmp/devourer-ldpc-waterfall/points.jsonl \
    --step-qdb 2 --thresholds 0.1,0.5
```

Sweep the noise-limited rising edge — at high index near-field, both curves
distort (the BCC MCS7 curve rolls over and dies while LDPC keeps decoding;
compare curves only on the rising edge). Bench-measured on an 8812AU→8822BU
pair at MCS7/20 MHz: **+3.0 dB LDPC gain at the 10%-delivery crossing**, and
in the saturation regime LDPC delivered ~80% where BCC delivered 0–19%.

### `nitroqam_waterfall.sh`: VHT on the 2.4 GHz band

VHT is a 5 GHz standard; airing it on 2.4 GHz is the vendor extension marketed
as NitroQAM/TurboQAM. Same waterfall method as `ldpc_waterfall.sh` plus a
**modulation gate**: every sampled `rx.txhit` is decoded back to a rate, so a
cell that delivers frames which arrive as HT is reported as a fallback rather
than as a pass.

```bash
sudo tests/nitroqam_waterfall.sh --emit-vid 0x0bda --emit-pid 0x8812 \
    --ground-vid 0x2357 --ground-pid 0x012d --channel 6 --bw 20
python3 tests/nitroqam_waterfall.py report \
    /tmp/devourer-nitroqam-waterfall/points.jsonl
```

The power axis defaults to `--pwr-mode offset` (`SetTxPowerOffsetQdb`, which
preserves the calibrated per-rate shape). Do not substitute the flat
`DEVOURER_TX_PWR` index: it forces both paths to one level and zeroes the
per-rate diffs, and measurably degrades the link — the same pair that reaches
MCS4 at calibrated power tops out at MCS1 under a flat index.

`nitroqam_waterfall.py --self-test` is the headless half, wired into `ctest` as
`nitroqam_decode_math`: it covers the DESC_RATE decode and threshold-crossing
math that a bench link topping out below 256-QAM never reaches on air.

**Cold-cycle or measure nothing.** The harness VBUS-cycles both ends before
every cell (`REGRESS_VBUS_MAP` + uhubctl) because high-order constellation TX
degrades across warm re-inits: the same 8812AU delivered 0% at MCS7 warm and
58% cold, same channel/power/gap, with no error logged anywhere. Without it a
rate ceiling measures accumulated chip state, not the link.

Measured: VHT confirmed on 2.4 GHz from an 8812AU, 8822BU and 8812CU, and
**256-QAM (VHT1SS_MCS8) confirmed on the 8812AU**, modulation-verified by an
8822BU peer. Note VHT MCS9 is illegal at 20 MHz for 1-2 streams — hardware
emits MCS8 and the modulation gate catches it. Full results:
`docs/vht-on-2g4.md`.

### `nitroqam_kernel_ab.sh`: devourer vs the vendor driver, same dongle

The control that decides whether a delivery wall is ours or the rig's: runs
devourer and the vendor driver from `reference/rtl8812au` on the same adapter,
interleaved per rate, judged by an AR9271 sniffer (different vendor's radio, no
shared silicon or code).

```bash
sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3" tests/nitroqam_kernel_ab.sh 6
```

Measured at the top of the HT ladder: devourer 58.2% vs vendor 64.3% at MCS7,
61.7% vs 68.4% at MCS5 — on par, no devourer-side high-MCS defect. Do **not**
substitute the in-tree `rtw88` for the vendor driver: it accepts frames on a
monitor netdev and reports them injected while airing nothing decodable, which
reads as a kernel-side failure at every rate.

### `ground_station_qualify.sh`: is your receiver fit to measure this rate?

Run before trusting any delivery number, and before believing any story about a
transmitter. Sweeps the rate ladder on the ground station and refuses the pairing
(exit 1) when the test rate is off the flat part of the curve.

```bash
sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;0bda:8813=4-2.3,2" \
     GND_VID=0x0bda GND_PID=0x8813 tests/ground_station_qualify.sh MCS7/20
```

A receiver sitting on its cliff at the test rate measures **itself**, not the
transmitter, and swings between fine and zero on a few dB of ambient while the
robust rates stay pinned — indistinguishable from a transmitter that degrades and
recovers. Same TX, same channel, minutes apart:

| ground | MCS3 | MCS4 | MCS5 | MCS6 | MCS7 |
| --- | --- | --- | --- | --- | --- |
| Archer T3U (8822BU, **internal** antennas) | 97.8 | 98.3 | 79.1 | 49.0 | **2.9** |
| RTL8814AU (two **external** antennas) | 80.6 | 80.2 | 80.8 | 80.6 | **80.4** |

Both adapters are healthy. Prefer external-antenna adapters as ground stations
for high-MCS work, and cross-check against a second independent receiver
(`rx_vendor_ab.sh` runs the vendor driver on the *receive* side).

### Warm-session TX degradation: four harnesses, and start with the first

**`probe_repeatability.sh` — run this before believing any delivery
difference.** N identical back-to-back probes with nothing changed between them,
reporting the spread. On an 8812AU a single MCS7/20 probe has sd 1.8 and a
5.7-point range (the MCS1 control: 98.0 ± 0.2), so **effects under ~4 points are
not detectable one-shot**. Several plausible-looking decay curves in this
investigation turned out to sit inside that band.

```bash
sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
     tests/probe_repeatability.sh
```

`warm_tx_degradation_repro.sh` holds one ground receiver up for a whole run,
then alternates probes with warm devourer sessions, recording delivery at a test
and a control rate alongside the chip thermal meter and the receiver's RSSI/EVM.
Two controls close it: an idle with no power cycle, and a VBUS cycle.
`WARM_PWR=63 WARM_GAP=0` turns the warm sessions into max-power/max-duty
heating.

`thermal_offtime_sweep.sh` and `thermal_causation_probe.sh` exist to separate
*cooling* from *state reset*, which no ordinary power-cycle experiment can do —
the first varies only how long the chip stays powered down, the second measures
delivery inside a single uninterrupted session where nothing but temperature can
move. Both currently argue **against** a thermal explanation; the mechanism is
open. Results and what did not survive testing: `docs/warm-tx-degradation.md`.

Reading any of them: falling delivery with **flat RSSI** is signal quality;
falling RSSI would be a transmitter losing power, a different bug.

### `tx_teardown_asan.sh` / `teardown_gen_sanity.sh`: lifetime bugs on real hardware

A sanitizer only sees what runs, and the interesting lifetime bugs live in the
device/libusb teardown, which no headless test reaches. Both scripts want an
ASan build:

```bash
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DDEVOURER_SANITIZE=address && cmake --build build-asan -j
sudo -v && tests/tx_teardown_asan.sh            # Jaguar1 DUT, max-duty TX
sudo -v && tests/teardown_gen_sanity.sh         # every plugged generation
```

`tx_teardown_asan.sh` floods at `DEVOURER_TX_GAP_US=0` and kills the demo
mid-stream (`SIG=INT` for the other signal), because a saturated async TX queue
at exit is the condition that exposes teardown ordering — a gap-2000 run drains
between frames and looks clean either way. It also covers the aggregation URB
(`DEVOURER_TX_USB_AGG`) and, given a hub with per-port power switching
(`WEDGE_HUB`/`WEDGE_PORT`, uhubctl `ppps`), a real VBUS cut mid-flood: the
quiesce must still return, and a `tx.quiesce_timeout` event in the log says it
never drained. `BUILD=<dir>` points at another tree, which is how a fix is
shown to fix something rather than asserted to.

### `kestrel_prich_onair.sh`: Kestrel per-channel BB programming

Counts delivered frames across the configurations that the vendored
`halbb_ctrl_bw_ch` treats differently — 2.4 GHz CCK (its SCO threshold tables),
20 MHz OFDM as the control, and RX at 40/80 MHz (the primary sub-band index).
`KESTREL_TX=1` swaps the roles to check the transmit side, and `REF_ONLY=1`
takes the Kestrel out of the path entirely, which is the first thing to run
when a wide-bandwidth cell scores zero — the emitter's own channel width comes
from `DEVOURER_HOP_BW`, not from the `/40` in `DEVOURER_TX_RATE` (that only
fills the descriptor field), and getting that wrong zeroes a cell for reasons
that have nothing to do with the DUT.

## Supported DUTs

Listed in `SUPPORTED_DUTS` at the top of `regress.py`. Extend the dict
to add new chipsets — the rest of the script is chipset-agnostic.

## Architecture notes

- All kernel-side operations (modprobe / sysfs reads / `iw` / `tcpdump` /
  scapy) go through one abstraction (`KernelHost`). Local mode runs them
  via `subprocess.run`; VM mode wraps them in `ssh ... sudo`. Adding a
  third backend (e.g. remote bare-metal box) is a new class.
- DUT routing in VM mode uses `virsh attach-device` (USB hot-plug). The
  matrix moves DUTs between host and VM per cell as needed, restoring all
  DUTs to the host on exit so the next cell starts from a clean baseline.
- `inject_beacon.py` is shipped to the VM via `scp` each run (small file)
  and exits when its `--duration` elapses — orchestrator waits rather
  than killing, so the final "sent N frames" line is captured.

## Known limitations

- Tests "signal of life", not throughput — air noise makes absolute
  counts unreliable; pass-threshold is deliberately generous.
- Per matrix run: ~100s in local mode, ~3-4 min in VM mode (USB hot-plug
  adds ~5s per cell transition).
- Two-adapter scope today. To extend to >2, add a pairing loop in
  `main()` that runs the 4-cell matrix per chipset pair.
- VM mode assumes a single libvirt host running both `virsh` (locally)
  and the VM. Pulling the VM onto a different host is a `--vm-ssh
  user@vmhost` away on the kernel cell side, but `virsh attach-device`
  still runs locally; if the VM is on a different host, run virsh there
  (via your own wrapper).
- Cell 4 (`devourer-TX → devourer-RX`) requires both DUTs to be on the
  host and devourer-claimable simultaneously. Both chipsets need working
  devourer RX — an RX-side failure shows as 0 hits in that cell
  regardless of the TX side.
- **Channel / band asymmetry.** A single-channel matrix run doesn't tell
  the full story — chip behaviour can differ per band. Run 2.4GHz
  (`--channel 6`) plus at least one 5GHz channel (`--channel 36` /
  `--channel 100`) before calling a configuration good.

## Kestrel (RTL8852BU / RTL8832CU) suite

The 11ax generation has its own `kestrel_*` on-air scripts (regress.py covers
it as a DUT too, but these exercise Kestrel-specific surfaces). All expect the
adapters cold (`uhubctl` VBUS-cycle; the in-tree rtw89 modules must be
temp-blacklisted — they auto-probe and warm-initialize the chip at every
enumeration, masking cold-boot behavior):

- `kestrel_rx_smoke.sh` — ambient monitor-RX decode per channel (VBUS-cycles
  first).
- `kestrel_tx_sdr.sh`, `kestrel_narrowband_sdr.sh`, `kestrel_streamtx_sdr.sh`
  — B210 SDR duty / occupied-bandwidth measurements of the TX path.
- `kestrel_tx_onair.sh`, `kestrel_inject_sanity.sh`, `kestrel_bw_rx.sh` —
  two-adapter TX→RX decode, per-bandwidth.
- `kestrel_he_onair.sh`, `kestrel_two_radio_he.sh` — HE-rate on-air witness
  (the two-radio variant needs no vendor kernel module).
- `kestrel_beacon_onair.sh`, `kestrel_txreport_onair.sh` — MAC-plane canaries
  (HW beacon engine, TX-report C2H).
- `kestrel_8832cu_*` — the 8832CU family: `rx`, `tx_sdr`, `tx_matrix`
  (rate×bw duty matrix), `evm`/`evm_stream` (8812AU-monitor TX-EVM; `TX_ID`
  env retargets the DUT), `6g_txrx` (6 GHz TX+RX), `6g160_txrx` (the
  documented 6G-160 TX limitation witness), `ab`/`kernel_rx_ab` (vendor-ko
  A/B ground truth).
- `kestrel_vendor_ko_smoke.sh`, `kestrel_vendor_rx_check.sh`,
  `kestrel_env_calibrate.sh` — kernel-driver ground-truth harnesses.
- `kestrel_txpwr_sweep.sh` — TXAGC dBm sweep vs SDR duty.
- `kestrel_fwlog_c2h_probe.sh` — exercises the `debug.kestrel_fw_log` knob.
- Selftests (`ctest`): `kestrel_rxparse_selftest.cpp`,
  `kestrel_txreport_selftest.cpp`.
