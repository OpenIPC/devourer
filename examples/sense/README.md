# WiFiSenseDemo — Wi-Fi motion sensing you can run on your own dongles

A runnable example built on the `devourer` library that turns two cheap Realtek
Wi-Fi adapters into a **motion / presence sensor**. No SDR, no special AP — one
adapter sounds, the other answers, and the demo reads human motion out of the
802.11ac beamforming feedback the second adapter returns.

This document is written so you can **reproduce it, move it into your own
environment, and change the maths**. If you only want the theory of *why* a
beamforming report senses motion, read
[`docs/beamforming-victim-sensing.md`](../../docs/beamforming-victim-sensing.md);
this file is the hands-on half.

---

## 1. The one-paragraph idea

An 802.11ac beamformer (the *sounder*) asks a *beamformee* to measure the
per-subcarrier channel between the sounder's two transmit antennas and report it
back, compressed as **Givens rotation angles** (a phase `phi` and an amplitude
angle `psi` per subcarrier). That report is a measurement *taken at the
beamformee* of the radio channel between the two devices. When a person moves in
that channel, the multipath changes, so the reported angles **jitter frame to
frame**. Measure that jitter and you have a motion detector. This is the
mechanism behind published work such as Wi-BFI, BeamSense and BFMSense.

The demo drives *both* ends itself — a sounder and a beamformee on the same host
— so you don't need a cooperating AP. The sounder injects a short NDPA frame; the
chip hardware-generates the sounding NDP; the beamformee answers with a
compressed beamforming report; the sounder self-captures it on a concurrent RX
loop. All of that is the `devourer` beamforming self-sounding path (see
[`docs/beamforming-self-sounding.md`](../../docs/beamforming-self-sounding.md)).

---

## 2. Hardware you need

**Two USB adapters** that `devourer` supports, plugged into the same host:

| Role | Requirement | Good choice |
|------|-------------|-------------|
| **Sounder** | Can transmit + self-capture (TX-with-RX). Any supported generation works; Jaguar3 (8822CU `0bda:c812`, 8822EU `0bda:a81a`) is the best-tested. | RTL8822CU |
| **Beamformee** | Must answer a VHT sounding with a compressed report. **Two receive antennas (2T2R) give a far cleaner signal** than a 1T1R part. | RTL8822BU / RTL8822CU (2T2R) |

A 1T1R beamformee (e.g. an 8811AU) *works* but produces a much noisier, weaker
signal — its "relative channel between two antennas" is degenerate. If your
readout is jumpy, suspect the beamformee first.

**Placement matters more than anything else** — see §6.

The demo is built on macOS/Linux (libusb). It does not need root on macOS; on
Linux you may need to run as root or add a udev rule so libusb can claim the
interface.

---

## 3. Build

From the repo root:

```sh
cmake -S . -B build
cmake --build build -j --target WiFiSenseDemo
```

The decoder has a headless self-test that runs under `ctest` (no radio needed):

```sh
ctest --test-dir build -R bf_report_decode
```

---

## 4. Run it

```sh
./build/WiFiSenseDemo --channel 6 \
  --sounder    0x0bda:0xc812 \
  --beamformee 0x0bda:0xb82c
```

Both selectors take `[VID:]PID` (VID defaults to `0x0bda`). Find your adapters'
IDs with `lsusb` (Linux) or `system_profiler SPUSBDataType` / `ioreg -p IOUSB`
(macOS).

Startup takes a few seconds: bring up both radios → calibrate the decoder split
(~48 reports) → acquire the noise floor (~2.5 s). Then you get a live line:

```
  [  CLEAR  ]   0.4σ  |                    |  now 0.0003 base 0.0003  1310/s
  [ MOTION  ]   9.2σ  |####################|  now 0.0016 base 0.0003  1298/s
```

Wave your hand near the adapters and the `σ` reading climbs; the verdict flips to
`MOTION` and holds for ~1 s after you stop. `Ctrl-C` to quit.

### Reading the line

| Field | Meaning |
|-------|---------|
| `CLEAR` / `MOTION` | verdict. `MOTION·nb` = the rise is concentrated on a few tones (looks like a narrowband interferer, not broadband motion). |
| `σ` | how many noise-floor standard deviations the current energy sits above the self-calibrated floor. The detector fires at `σ ≈ k` (see §7). |
| bar | `σ` as a bar, saturating at 10σ. |
| `now` | current motion energy (mean per-tone circular variance of `phi`). |
| `base` | the self-calibrated still-floor it is measured against. |
| `N/s` | beamforming reports captured per second (health indicator; a fast Jaguar3 sounder gives ~1000–1500/s). |

---

## 5. All the knobs

**Command-line flags**

| Flag | Default | Purpose |
|------|---------|---------|
| `--channel N` | `6` | Wi-Fi channel to sound on. Try 5 GHz (36, 149) for a different multipath environment. |
| `--sounder [VID:]PID` | `0bda:8812` | sounder adapter selector |
| `--beamformee [VID:]PID` | `0bda:c812` | beamformee adapter selector |
| `--vid 0xNNNN` | `0x0bda` | default VID for both selectors (for OEM-rebadged dongles) |
| `--sensitivity low\|med\|high` | `med` | detector threshold `k` = 6 / 4 / 2.5 sigmas |
| `-v`, `--verbose` | off | show the library's bring-up logs (otherwise quieted to warnings) |

**Environment variables**

| Var | Purpose |
|-----|---------|
| `DEVOURER_SENSE_K=<sigmas>` | override the CFAR threshold `k` directly (finer than `--sensitivity`). |
| `DEVOURER_SENSE_DUMP=1` | print every captured report as a `<devourer-bf-report-raw>HEX` line on **stderr** — the input for offline analysis (§8). |
| `DEVOURER_SENSE_DEBUG=1` | print the first few captured reports' geometry (SA, Nc/Nr, MU, Ns) for a sanity check. |

---

## 6. Environment — the single biggest lever

The signal strength depends almost entirely on **how much a moving person changes
the channel between the two adapters, relative to the static part of that
channel.**

- **Separate the two adapters.** Two dongles side by side on the same hub share a
  strong, short, line-of-sight channel that a hand barely perturbs. Put one on a
  **USB extension a metre or more away**, ideally across the space you want to
  sense. This can turn a marginal signal into an obvious one.
- **Put the person in the path.** Motion *between* or *near* the two antennas
  moves the needle most.
- **Multipath helps.** A reflective room (walls, furniture) gives the channel more
  structure to perturb than an anechoic free-space shot.
- **Channel width / band.** 20 MHz on 2.4 GHz is the default. 5 GHz and wider
  channels change the coherence bandwidth and the per-tone structure; worth
  experimenting.

Reference numbers from one indoor 20 MHz / channel-6 setup (2T2R↔1T1R, adapters
~30 cm apart):

| condition | motion energy (mean per-tone circular variance of `phi`) |
|-----------|----------------------------------------------------------|
| still     | 0.0003 (floor), window-to-window jitter ~0.00002 |
| hand wave next to a dongle | 0.0006 – 0.002 (2–6× the floor) |

Your absolute numbers **will differ** — that is exactly why the detector
self-calibrates rather than using a fixed threshold. Use these only as a rough
scale.

---

## 7. How it works inside (so you can change the maths)

Everything below lives in two files:

- **`src/BfReportDecode.h`** — the report decoder + the `MotionMeter` metric.
- **`examples/sense/main.cpp`** — the `AdaptiveDetector` + display.

### 7a. Decoding the report → angles

`parse_report()` matches a VHT/HT compressed beamforming report and locates the
packed angle bits. `decode_angles()` unpacks, per subcarrier, one `phi` (phase)
and one `psi` (amplitude) angle, LSB-first, and dequantises them:

```
phi = (2q + 1) · π / 2^b_phi          # dequant_phi
psi = (2q + 1) · π / 2^(b_psi + 2)    # dequant_psi
```

**The bit split `(b_phi, b_psi)` matters and is easy to get wrong.** The 802.11
compressed-Givens codebook always pairs `b_phi = b_psi + 2`. For the common
10-bits-per-subcarrier 2×1 report that uniquely means **`(6, 4)`**. `pick_split()`
enforces that relationship — do **not** replace it with a naive "minimise
cross-frame variance" search: a too-coarse `psi` (e.g. 2 bits) is trivially
constant, so an unconstrained search picks a bit-*misaligned* split whose `phi`
decodes to garbage that jitters on a static channel and reads as constant motion.
This was a real bug; the self-test now pins the split to `(6,4)`.

### 7b. The motion metric (`MotionMeter`)

For each report we keep the first `phi` of every subcarrier in a sliding window
(`kWindow = 512` reports, ~0.3–0.5 s). The per-tone motion signal is the
**circular variance** of that phase over the window:

```
var[k] = 1 − |mean_over_window( e^{i·phi_k} )|        # in [0, 1]
motion_energy = mean_k var[k]
```

Circular variance is 0 when a tone's phase is constant (static channel) and rises
toward 1 as it scatters (a changing channel). It is CFO-robust in the sense that
a *constant* phase offset cancels; what survives is frame-to-frame change.

Why `phi` (phase) and not `psi` (amplitude)? Measured: a hand wave moves the
phase but barely moves the coarse 4-bit amplitude ratio, so `phi` is the sensitive
signal. `psi` is decoded too, and the "broadband vs localized" flag
(`MotionMeter::localized()`) uses the per-tone shape to distinguish human motion
from a narrowband interferer.

**Things to try here:** a different aggregation (median/percentile instead of
mean over tones), a frame-to-frame `|Δphi|` "velocity" metric, a shorter window
for faster response, or weighting tones by their SNR.

### 7c. The detector (`AdaptiveDetector`)

A fixed threshold is wrong because the still-floor varies with hardware, channel
and geometry. Instead the detector self-calibrates (a CFAR-style test):

- Track the **floor** (mean still energy) and its **jitter** `dev` with a
  rate-independent EMA (`kTauFloor = 4 s`; faster `kWarmTau = 0.4 s` during a
  `kWarmSec = 2.5 s` warm-up). The floor **freezes while motion is held**, so a
  moving subject can't drag it up and blind the detector.
- Threshold: `floor + k · max(dev, kDevFloor)`. `k` is the sensitivity
  (`--sensitivity`/`DEVOURER_SENSE_K`); `kDevFloor` is a minimum jitter so the
  threshold can't collapse onto a perfectly-still floor.
- **Hysteresis:** arm only after the energy stays over threshold for
  `kArmSec = 0.15 s` (a lone spike can't trigger), then **hold** `kHoldSec = 1.2 s`
  after it drops (no flicker; bridges the gaps in an intermittent wave).
- `σ` shown in the UI is `(energy − floor) / max(dev, kDevFloor)`.

The tunable constants are all `static constexpr` at the top of
`AdaptiveDetector` in `examples/sense/main.cpp`:

```
kWindow    = 512      # metric window (reports)      [in main.cpp top-level]
kWarmSec   = 2.5 s    # floor acquisition
kWarmTau   = 0.4 s    # fast tracking during warm-up
kTauFloor  = 4.0 s    # floor/jitter tracking constant
kArmSec    = 0.15 s   # dwell over threshold before MOTION
kHoldSec   = 1.2 s    # hold MOTION after last trigger
kDevSeed   = 0.00005  # initial jitter estimate
kDevFloor  = 0.00004  # minimum jitter → minimum sensitivity margin
```

If your still-floor sits at a different scale than the reference in §6, the two
numbers most likely to need adjusting are **`kDevFloor`** (raise it if you get
false alarms at rest, lower it if real motion never crosses the threshold) and
**`k`** (via `DEVOURER_SENSE_K`, no rebuild needed).

---

## 8. Experiment with your own formulas (capture → analyse loop)

You do not have to edit C++ to try new maths. Capture the raw reports and analyse
them offline:

```sh
# capture ~30 s of reports to a file (stderr carries the raw dump)
DEVOURER_SENSE_DUMP=1 ./build/WiFiSenseDemo --channel 6 \
  --sounder 0x0bda:0xc812 --beamformee 0x0bda:0xb82c \
  2> capture.txt

# decode + inspect with the reference Python tool
grep '<devourer-bf-report-raw>' capture.txt | tools/bf_report_decode.py
```

`tools/bf_report_decode.py` prints the header (Nc/Nr/BW/Ng), the chosen split,
per-stream SNR and the per-tone `|h_B/h_A|`. From the same `capture.txt` you can
compute anything you like in a few lines of Python — per-tone variance, a
different window, a spectrogram of `phi[k]` over time, a doppler estimate — and
label a run by doing a clean **still segment then a moving segment** and comparing
the two. That is exactly how the metric, window and thresholds in this demo were
chosen.

> **A note on the reference tool:** `tools/bf_report_decode.py` currently selects
> the split by cross-frame stability and can land on the degenerate `(8,2)` for a
> 10-bit report. When comparing against the C++ path, force the correct Givens
> split `(6,4)` (see §7a).

---

## 9. Limitations and honest caveats

- **Weak coupling on close adapters.** Side-by-side dongles barely see motion. §6
  is not optional advice — it is the difference between working and not.
- **Coarse amplitude.** The Realtek compact codebook gives `psi` only ~4 bits, so
  amplitude-based sensing is limited; this demo leans on phase.
- **Second-adapter flakiness.** Some cheap beamformees' firmware crashes on long
  runs and the adapter drops off the USB bus. The demo has a stall watchdog (§10);
  if it fires, replug that adapter.
- **A single-radio (`--mode self`) variant** — the sounder acting as its own
  beamformee — is plausible and would remove the flaky second adapter, but is not
  implemented here.
- **No passive/AP-sniffing mode.** Sensing an existing AP↔client sounding exchange
  (the Wi-BFI approach) is a natural extension but is deliberately **not** shipped:
  it needs an AP actively sounding VHT beamforming, which we could not validate
  against. Contributions welcome.

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `could not open VVVV:PPPP` | Adapter not found. Check `lsusb`. A prior hang can drop an adapter off the bus — **unplug/replug it**. |
| Stuck on `calibrating decoder… 0 reports (0/s)` | The beamformee isn't answering: wrong PID, it doesn't support VHT sounding, or it fell off the bus. Try `-v` to see bring-up, and confirm both adapters enumerate. |
| `report stream stalled Ns — beamformee stopped responding` | The beamformee firmware crashed. The demo stops cleanly; replug that adapter before re-running. |
| Constant `MOTION` at rest | Threshold too low for your floor: raise `DEVOURER_SENSE_K`, or `--sensitivity low`. If `base` reads `0.000` and never rises, that's the old split bug — make sure you're on a build with the `(6,4)` `pick_split`. |
| Never triggers even on a strong wave | Threshold too high, or coupling too weak. `--sensitivity high`, and **separate the adapters** (§6). |
| Needs root on Linux | libusb can't claim the interface. Run as root or add a udev rule for the adapter's VID:PID. |

---

## 11. Files

| File | What |
|------|------|
| `examples/sense/main.cpp` | the `WiFiSenseDemo` binary: adapter bring-up, sounding loop, `AdaptiveDetector`, display, watchdog. |
| `src/BfReportDecode.h` | header-only report decoder + `MotionMeter` (reusable; no libusb dependency). |
| `examples/sense/bf_report_decode_selftest.cpp` | headless `ctest` that guards the decoder against a real captured report. |
| `docs/beamforming-victim-sensing.md` | the theory: why a beamforming report measures the channel and senses motion. |
| `tools/bf_report_decode.py` | reference Python decoder for offline analysis. |
