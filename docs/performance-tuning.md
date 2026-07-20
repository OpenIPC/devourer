# Performance: devourer vs the kernel drivers

Three axes decide whether a userspace driver is "fast enough": how quickly it
comes up, how much of the channel it fills, and how much host CPU it burns to
do it. Devourer wins or ties the vendor drivers on all three. This doc carries
the measured numbers, the methodology, and the traps that make naive benchmarks
lie.

All numbers below are on host silicon and host USB (no VM — virtualized USB
adds latency and CPU that would contaminate either column), against the vendor
out-of-tree drivers built from the trees under `reference/` for the same host
kernel.

## On-air throughput — the channel, not the host

At a saturating MCS flood, devourer fills the channel as fully as the vendor
driver: on the RTL8812AU at MCS7/20 MHz, ch 149, **~80% SDR duty vs the vendor
88XXau's ~78%**. The ceiling here is the *air* (and below it the USB link:
480 Mbps USB2 / 5 Gbps USB3), both far above Wi-Fi on-air rates — so the host
software path is never the throughput bottleneck, and a userspace driver has no
inherent disadvantage.

Measure on-air throughput as **SDR duty × PHY rate** (`tests/bench_onair.py`),
never sniffer frame counts: a sensitive monitor receiver decodes weak frames
and masks a real drop. Keep a known-good control adapter, re-check it each
session, and take one clean SDR read per session (a second back-to-back read
can fail to reacquire and report ~0).

## Host CPU cost — where userspace actually wins

Throughput being equal, the interesting question is CPU-per-frame. Devourer
burns **3–4× less** host CPU than the kernel driver it re-implements, at the
same on-air work. Same RTL8812AU, MCS7/20 MHz, ch 149, single-thread flood,
total system CPU (all cores, idle-subtracted):

| frame size | devourer (libusb) | kernel (vendor 88XXau) | devourer advantage |
|-----------:|------------------:|-----------------------:|:------------------:|
| 1500 B | **0.17 core** (48 core-ms/1000 fr) | 0.97 core (205 core-ms/1000 fr) | **4.2×** |
| 512 B | ~0.29 core (45 core-ms/1000 fr) | 1.02 core (164 core-ms/1000 fr) | **3.6×** |
| 128 B | ~0.33 core (37 core-ms/1000 fr) | 1.03 core (116 core-ms/1000 fr) | **3.2×** |
| 64 B | 0.34 core (36 core-ms/1000 fr) | 1.06 core (112 core-ms/1000 fr) | **3.1×** |

The reason is structural. The kernel driver drags every frame through the full
networking stack — **mac80211 → cfg80211 → qdisc → skb alloc/free → driver
xmit** — and that generality it can't skip is the extra ~0.7 core. Devourer's
raw injection builds one TX descriptor and hands the frame to usbfs; it does
none of that stack work. The cost that remains splits roughly evenly between
userspace libusb and the usbfs submit syscall (kernel-in-syscall `stime`), and
neither dominates.

This matters most on the **air side of a link (the camera / transmitter)**,
where the host is an embedded SoC and every spare core counts.

### Measuring CPU honestly

`tests/cpu_per_mbit.py` produces the table above. Three things it does to keep
the number trustworthy:

- **Total system CPU, idle-subtracted, with a paired baseline** — captures both
  userspace and kernel-side work (usbfs/xhci softirq for devourer, mac80211 +
  driver `kworker`/softirq for the kernel), so neither side hides cost in a
  kernel thread. On a shared host it also subtracts identifiable background
  load deterministically rather than hoping it averages out.
- **A `utime`/`stime` split** on the flooding process — separates the userspace
  feeder/interpreter cost from the kernel-in-syscall cost. This is what proves
  the kernel driver's ~1 core is real per-frame kernel work, not the injector.
- **A compiled C injector** (`tests/raw_inject.c`, a raw AF_PACKET `sendto`
  loop) as the kernel-side feeder, so the feeder's own overhead is provably
  negligible (a few milli-cores) and matches devourer's compiled feeder. It
  also runs on Python-less targets, so the same CPU test can be taken on an
  embedded camera SoC directly.

Caveat: "kernel" here is the **vendor 88XXau** — the exact driver devourer
re-implements, so the delta is purely userspace-vs-kernel architecture on
identical chip programming. Mainline rtw88 was not a usable control (its
monitor mode does not reliably transmit injected frames).

## TX submission mode — async on Jaguar1, synchronous on Jaguar2/Jaguar3

Devourer submits a TX frame to USB one of two ways, and the choice is
deliberate and per-generation — not two code paths that drifted apart. Both
solve the same problem (per-frame USB round-trip time capping single-thread
throughput) with opposite strategies, and the right strategy tracks USB speed.

- **Asynchronous** (`tx_async`, fire-and-forget with caller-thread completion
  reaping): many frames sit in flight at once, so a single feeder thread keeps
  the pipe full. **Jaguar1 uses this.**
- **Synchronous** (`bulk_send_sync_ep`, blocking with a finite timeout): each
  send completes-or-fails before the next. **Jaguar2/Jaguar3 use this** for the
  data path (and every generation uses it for firmware download).

Single-thread flood, MCS7/20 MHz, ch 149 (SDR duty × PHY rate):

| generation | USB link | measured |
|---|---|---|
| Jaguar1 (RTL8812AU) | USB2 High-Speed | **sync ≈ 45%** duty · **async ≈ 80%** duty |
| Jaguar3 (RTL8812CU) | USB3 SuperSpeed | sync ≈ 79% duty |
| Jaguar2 (RTL8822BU) | USB3 SuperSpeed | sync ≈ 79% duty (deep-feed and A-MPDU do not lift it) |

- On **USB2**, the per-frame round-trip is long enough that a blocking send
  leaves the channel idle between frames — one sync thread cannot saturate.
  Async pipelining hides the round-trip and nearly doubles the on-air duty.
  Jaguar1 therefore needs async.
- On **USB3**, the round-trip is short enough that a single blocking thread
  already saturates (~79%, matching Jaguar1's async ceiling). Async would
  recover no throughput — deep-feeding or A-MPDU doesn't raise it either.

Jaguar2/Jaguar3 also *stay* synchronous for two reasons beyond "async buys
nothing on USB3": the blocking path gives the HalMAC bring-up a clean per-send
backoff (until the TX-path enable registers are programmed the chip NAKs every
frame; a blocking send with a finite timeout completes-or-fails each frame and
stays responsive to the stop flag, whereas fire-and-forget into a non-draining
endpoint is exactly what wedges the chip's USB core), and it avoids contending
with the Jaguar3 coex runtime thread that shares the same libusb context.

**Do not unify the two onto one mode.** Forcing Jaguar1 onto sync halves its
single-thread throughput; forcing Jaguar2/Jaguar3 onto async yields no
throughput gain and gives up the bring-up backoff and coex isolation. The
shared machinery already lives one level down (both modes are methods on the
USB transport, the descriptor build stays in each HAL); the generation picks
only the transfer mode.

## Tuning levers and their measured reality

Two levers are often assumed to cut host CPU. Measured on the 8812AU, neither
does much — the structural win above is already the dominant effect:

- **USB TX aggregation** (`DEVOURER_TX_USB_AGG`, packs N frames per bulk-OUT
  URB to amortize the submit syscall) is **silicon-disabled on the RTL8812A**:
  the chip overflows its OQT beyond one descriptor per bulk window, so the
  aggregation planner never packs and every frame takes the single-frame path.
  Enabling it there only adds the batch layer's userspace packing overhead —
  CPU goes flat-to-worse, not down. It genuinely packs only on the 8814A (3),
  8821A (6), and the HalMAC generations (3) — a modest depth, where the
  syscall saved can still be offset by the packing cost. Do not expect a
  host-CPU reduction from it on the 8812A.
- **Zerocopy RX DMA ring** (`usb.rx_zerocopy`, default on): allocates the async
  RX ring from kernel DMA memory (`libusb_dev_mem_alloc`) so a received frame
  DMAs straight into the userspace buffer and usbfs skips the copy-on-reap.
  Correct and safe (per-buffer heap fallback when the HCD doesn't support it),
  but the copy it removes is tiny at realistic RX rates — a 20 Mbps video link
  is ~2.5 MB/s, negligible against the per-URB reap. It's kept as a
  structurally-correct, low-risk path (and a base to develop further), not a
  measured CPU win at these rates.

- **RX URB size** (`rx.urb_bytes` / `DEVOURER_RX_URB_BYTES`, default 16 KB on
  the 11ac generations) is a compatibility constraint, not a throughput lever.
  Some MediaTek Android xhci/usbfs stacks never complete a bulk-IN read larger
  than 16 KB — `LIBUSB_ERROR_TIMEOUT` forever, zero RX with a green init
  (OpenIPC/PixelPilot#6; fixed by floppyhammer's 16 KB reads in #19, regressed
  to 32 KB by the #213 transport split, restored since). Bigger URBs buy
  nothing on healthy hosts either: the device-side RX aggregation thresholds
  (Jaguar1 `rxagg_usb_size` 0x3/0x1, Jaguar2/3 halmac agg page-th 0x03) cap
  every aggregate at ≤16 KB, so each aggregate ends its URB via short packet
  and a 32 KB buffer never filled past 16 KB anyway. If you raise the URB
  size, raise the aggregation thresholds with it — but never let an aggregate
  exceed the URB size, or a block spans two URBs and the RND8/next_offset
  parse walk breaks. Kestrel keeps its own 32 KB ring (8852C RXAGG LEN_TH is
  ~20 KB) and does not consume this knob.

The honest summary: devourer's CPU advantage is the kernel stack it *doesn't*
run, and that is already captured by the raw-injection design. There is no
large additional CPU lever to pull on the 8812A on top of it.

## Startup time — ready-to-move-frames

How long from "process starts" until the radio actually moves frames? Devourer
reaches ready-to-RX/TX faster than the kernel driver on every supported chip,
in both directions. "Ready" is measured, not inferred:

- **devourer RX** — `exec → first 802.11 frame delivered` (`rxdemo`, its
  `init.timing` event, stage `demo.first_rx_frame`).
- **devourer TX** — `exec → first bulk-OUT submitted` (`txdemo`, the
  `txdemo.first_tx_submit` stage; includes the demo's settle sleep — that's the
  user-visible latency).
- **kernel** — `insmod → netdev registered → monitor-mode setup → first
  tcpdump frame`. Same finish line: a frame in hand.

From a **true cold plug** (VBUS power-cycled before every rep, 2.4 GHz ch 6,
median of 2–4 reps):

| chip | devourer | vendor kernel driver |
|---|---|---|
| RTL8812AU | **1.7 s** | 2.1 s (88XXau) |
| RTL8814AU | **5.7 s** | 6.9 s (88XXau) |
| RTL8821AU (Archer T2U+) | **2.2 s** | 5.3 s (88XXau) |
| RTL8822BU (Archer T3U) | **7.2 s** | 11.1 s (rtl88x2bu) |
| RTL8821CU | **3.4 s** | 5.1 s (8821cu) |
| RTL8812CU | **1.2 s** | 3.1 s (rtl88x2cu) |
| RTL8812EU | **1.0 s** | 3.0 s (rtl88x2eu) |

TX-ready tracks RX-ready within ~1% (the TX number includes everything the RX
number does except the RX loop itself; bring-up dominates both). The
PID/branding variants ride the same measured silicon: RTL8811AU is the 1T1R cut
of the 8812AU path, RTL8812BU rides the 8822B path, RTL8822CU/8822EU are the
same chips as the 8812CU/8812EU rows.

### Cold chip vs warm chip — measure the one you mean

A Realtek chip keeps calibration state across a soft re-open. The vendor drivers
exploit that: re-initialized **warm** (chip already brought up once since
power), their startup drops sharply, while from **true cold** they pay the full
bring-up + calibration bill. Same harness, warm (`authorized`-toggle between
reps instead of VBUS, median of 3–5):

| chip | devourer warm | kernel warm | kernel cold |
|---|---|---|---|
| RTL8812AU | 1.7 s | 2.0 s | 2.1 s |
| RTL8814AU | 5.9 s | 6.1 s | 6.9 s |
| RTL8821AU | 2.5 s | 2.4 s | 5.3 s |
| RTL8822BU | 7.4 s | 11.2 s | 11.1 s |
| RTL8821CU | 3.7 s | 3.9 s | 5.1 s |
| RTL8812CU | 1.5 s | 1.7 s | 3.1 s |
| RTL8812EU | 1.1 s | 1.8 s | 3.0 s |

The 8821AU's kernel driver even edges ahead by ~60 ms once the chip is warm —
but warm is not what a user plugging in an adapter experiences. One more
`authorized`-toggle caveat, seen on the 8812AU: an init from the toggled-warm
state can come up RX-deaf with a fully green init (1 of 8 warm reps; 0 of 8
VBUS-cold reps) — a stuck-chip-state artifact of the toggle, so treat a deaf
warm rep as suspect before blaming the driver or the unit. A benchmark that only
`rmmod`s or unbinds between reps (never cutting VBUS) flatters the kernel driver
and *understates* the devourer advantage at first plug, where the margins run
1.2–2.9×. Devourer's own init never runs slower than its cold number. The
rtl88x2bu driver is the outlier: ~11 s flat regardless of chip state.

### Where the time goes

Kernel-side, the split is `probe` (insmod → netdev: USB probe + efuse) vs
`monitor_setup` (ifup — where these vendor drivers run hal init + firmware
download) — e.g. the T3U's 11.2 s is 5.5 s probe + 5.6 s ifup; the 8812CU's
1.7 s is 0.4 s probe + 1.1 s ifup. Devourer-side, open/claim/USB-reset cost
~0.3–0.5 s and the rest is `InitWrite` — firmware download, MAC/BB/RF tables,
and calibration (LCK + IQK is most of the 8822B's ~7 s; the Jaguar3 halrf chain
is far cheaper). Module load itself is noise: stripping an 87 MB debug build of
rtl88x2bu down to 11 MB moved `kernel.probe` by nothing — the cost is the
driver's probe work, not insmod.

## Reproducing

```sh
# Startup time — kernel cells from the reference/ vendor modules on the host
# (88XXau for Jaguar1, rtl88x2bu/8821cu/rtl88x2cu/rtl88x2eu for Jaguar2/3):
sudo python3 tests/bench_init.py --kernel-host --traffic-from 8812

# On-air throughput — SDR duty × PHY rate per plugged chip and band:
sudo python3 tests/bench_onair.py --bands 149

# Host CPU per frame — devourer vs vendor kernel driver, swept over frame size:
cc -O2 -o build/raw_inject tests/raw_inject.c   # camera-portable C injector
sudo python3 tests/cpu_per_mbit.py --sizes 1500,512,128,64
```

(`bench_init.py` also has a `--vm-name`/`--vm-ssh` mode that runs kernel cells
in a pinned-kernel VM — useful for driver-behaviour comparisons, but not for
startup timing: virtualized USB adds latency to every stage.)

Three things the startup/CPU harnesses need to be honest:

- **A traffic source.** Every "first frame" marker — devourer's and the kernel
  cell's tcpdump alike — needs a frame on the bench channel. In an RF-quiet room
  there are *zero* ambient ch-6 frames and every RX cell times out looking
  healthy-but-dead. `--traffic-from PID` dedicates a plugged non-DUT adapter
  (e.g. an 8812AU) to a ~500 fps devourer beacon flood, so first-frame latency
  measures RX-path readiness, not beacon-interval luck.
- **A real cold cycle when comparing drivers.** The per-rep `authorized`-toggle
  re-enumerates the device but leaves calibration state in the chip (= the
  "warm" table above). For first-plug numbers, VBUS-cycle the port between reps.
  `REGRESS_VBUS_MAP="<sysfs_id>=<hubloc>:<port>"` (comma list) makes
  `bench_init.py`/`regress.py` do it automatically for DUTs on
  uhubctl-switchable hub ports; `tests/bench_8812au_row.sh` is the worked
  example. Never map an xhci root port — uhubctl on a root port has wedged a
  device here beyond anything but a machine power-off.
- **No in-tree kernel driver racing the timed window.** Modern kernels ship
  in-tree rtw88 USB drivers for every chip devourer supports; udev auto-loads
  them by modalias at every (re-)enumeration, and their probe runs a firmware
  download into the chip before the driver under test ever opens it. `modprobe
  -r` does not survive the next re-enumeration — write a temp blacklist
  (`/etc/modprobe.d/zz-temp-blacklist-*.conf`) for the DUT's module for the
  session. This also biases any "first init from cold" debugging, not just
  timing: a failed rtw88 probe (e.g. on a marginal unit) leaves the chip
  half-wedged mid-fwdl, and whatever the next driver does runs from that state.

Per-stage startup numbers come from the
`{"ev":"init.timing","stage":"<scope>.<stage>","ms":N}` lines the library emits
(`src/InitTimer.h`); `bench_init.py` aggregates medians and writes a markdown
report. A/B variants (`--variants`) isolate libusb log level, USB reset, and the
TX-power loop.
