# Startup time: devourer vs the kernel drivers

How long from "process starts" until the radio actually moves frames?
Devourer reaches ready-to-RX/TX faster than the kernel driver on every
supported chip, in both directions. This doc carries the measured numbers
and the methodology — including the two traps (cold-vs-warm chips, and
RF-quiet rooms) that make naive startup benchmarks lie.

"Ready" is measured, not inferred:

- **devourer RX** — `exec → first 802.11 frame delivered` (`WiFiDriverDemo`,
  its `init-timing: demo.first_rx_frame` line).
- **devourer TX** — `exec → first bulk-OUT submitted` (`WiFiDriverTxDemo`,
  `init-timing: txdemo.first_tx_submit`; includes the demo's settle sleep —
  that's the user-visible latency).
- **kernel** — `insmod → netdev registered → monitor-mode setup → first
  tcpdump frame`. Same finish line: a frame in hand.

## The numbers

From a **true cold plug** (VBUS power-cycled before every rep, 2.4 GHz
ch 6, median of 2, rep spread < 1.5%), against the vendor out-of-tree
drivers built from the trees under `reference/` for the same host kernel
— every cell on host silicon and host USB, no VM (virtualized USB adds
latency that would contaminate either column):

| chip | devourer | vendor kernel driver |
|---|---|---|
| RTL8814AU | **5.7 s** | 6.9 s (88XXau) |
| RTL8821AU (Archer T2U+) | **2.2 s** | 5.3 s (88XXau) |
| RTL8822BU (Archer T3U) | **7.2 s** | 11.1 s (rtl88x2bu) |
| RTL8821CU | **3.4 s** | 5.1 s (8821cu) |
| RTL8812CU | **1.2 s** | 3.1 s (rtl88x2cu) |
| RTL8812EU | **1.0 s** | 3.0 s (rtl88x2eu) |

TX-ready tracks RX-ready within ~1% (the TX number includes everything
the RX number does except the RX loop itself; bring-up dominates both).
The PID/branding variants ride the same measured silicon: RTL8811AU is
the 1T1R cut of the 8812AU path, RTL8812BU rides the 8822B path,
RTL8822CU/8822EU are the same chips as the 8812CU/8812EU rows.

## Cold chip vs warm chip — measure the one you mean

A Realtek chip keeps calibration state across a soft re-open. The vendor
drivers exploit that: re-initialized **warm** (chip already brought up once
since power), their startup drops sharply, while from **true cold** they
pay the full bring-up + calibration bill. Same harness, warm
(`authorized`-toggle between reps instead of VBUS, median of 3–5):

| chip | devourer warm | kernel warm | kernel cold |
|---|---|---|---|
| RTL8814AU | 5.9 s | 6.1 s | 6.9 s |
| RTL8821AU | 2.5 s | 2.4 s | 5.3 s |
| RTL8822BU | 7.4 s | 11.2 s | 11.1 s |
| RTL8821CU | 3.7 s | 3.9 s | 5.1 s |
| RTL8812CU | 1.5 s | 1.7 s | 3.1 s |
| RTL8812EU | 1.1 s | 1.8 s | 3.0 s |

The 8821AU's kernel driver even edges ahead by ~60 ms once the chip is
warm — but warm is not what a user plugging in an adapter experiences.
A benchmark that only `rmmod`s or unbinds between reps (never cutting
VBUS) flatters the kernel driver and *understates* the devourer
advantage at first plug, where the margins run 1.2–2.9×. Devourer's own
init never runs slower than its cold number. The rtl88x2bu driver is
the outlier: ~11 s flat regardless of chip state.

## Where the time goes

Kernel-side, the split is `probe` (insmod → netdev: USB probe + efuse) vs
`monitor_setup` (ifup — where these vendor drivers run hal init + firmware
download) — e.g. the T3U's 11.2 s is 5.5 s probe + 5.6 s ifup; the 8812CU's
1.7 s is 0.4 s probe + 1.1 s ifup. Devourer-side, open/claim/USB-reset cost
~0.3–0.5 s and the rest is `InitWrite` — firmware download, MAC/BB/RF
tables, and calibration (LCK + IQK is most of the 8822B's ~7 s; the
Jaguar3 halrf chain is far cheaper).

Module load itself is noise: stripping an 87 MB debug build of
rtl88x2bu down to 11 MB moved `kernel.probe` by nothing — the cost is
the driver's probe work, not insmod.

## Reproducing

```sh
# Kernel cells from the reference/ vendor modules on the host
# (88XXau for Jaguar1, rtl88x2bu/8821cu/rtl88x2cu/rtl88x2eu for Jaguar2/3):
sudo python3 tests/bench_init.py --kernel-host --traffic-from 8812
```

(The harness also has a `--vm-name`/`--vm-ssh` mode that runs kernel
cells in a pinned-kernel VM — useful for driver-behaviour comparisons,
but not for startup timing: virtualized USB adds latency to every stage.)

Two things the harness needs to be honest:

- **A traffic source.** Every "first frame" marker — devourer's and the
  kernel cell's tcpdump alike — needs a frame on the bench channel. In an
  RF-quiet room there are *zero* ambient ch-6 frames and every RX cell
  times out looking healthy-but-dead. `--traffic-from PID` dedicates a
  plugged non-DUT adapter (e.g. an 8812AU) to a ~500 fps devourer beacon
  flood, so first-frame latency measures RX-path readiness, not
  beacon-interval luck.
- **A real cold cycle when comparing drivers.** The harness's per-rep
  `authorized`-toggle re-enumerates the device but leaves calibration
  state in the chip (= the "warm" table above). For first-plug numbers,
  VBUS-cycle the port between reps (`uhubctl -l HUB -p PORT -a off/on`)
  — that's what the headline table uses.

Per-stage numbers come from the `init-timing: <scope>.<stage> = N ms`
lines the library emits (`src/InitTimer.h`); `bench_init.py` aggregates
medians and writes a markdown report. A/B variants (`--variants`)
isolate libusb log level, USB reset, and the TX-power loop.
