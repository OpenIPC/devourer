# Victim-oriented sensing — reading the client's channel *and* interference from its beamforming report

A beamforming report is a measurement taken **at the beamformee**. When a
beamformer (the AP / ground station) sounds a client and the client sends back a
VHT Compressed Beamforming report, that report describes the RF world as the
*client* sees it — not the beamformer. Two things fall out of that, and the
beamformer can read both off the air without the client cooperating beyond
normal 802.11ac sounding:

1. **the client's per-tone channel** — where it is, how its multipath is shaped;
2. **the client's per-tone interference** — which frequencies are being jammed
   *at the client's location*, which the beamformer's own receiver cannot see.

The second one is the interesting one. It is **victim-oriented sensing**: the
ground station learns that the drone (or the client) is being interfered on a
particular slice of spectrum even though the ground station's own receiver hears
a clean channel — and it can act on that (hop, or puncture).

This document is the write-up of a three-adapter, no-SDR experiment that proves
both, on the devourer stack. It builds directly on the self-sounding pipeline in
[beamforming-self-sounding.md](beamforming-self-sounding.md).

## It is measurement locality, not reciprocity

The tempting framing — *"the channel is reciprocal, interference is not, so the
report tells you about the far-end interference"* — is a **category error** for
802.11ac. That reciprocity argument belongs to *implicit* / TDD beamforming,
where the transmitter measures the *uplink* sounding and assumes the downlink is
the same (and needs TX/RX calibration to do it). 802.11ac dropped implicit
beamforming entirely and kept only **explicit** feedback: the beamformee measures
the *downlink* directly and sends it back. No reciprocity is assumed, so there is
no "interference non-reciprocity" to invoke.

The correct statement is just **measurement locality**:

> An explicit beamforming report is a beamformee-side measurement. Its two parts
> carry different information: the unit-norm **V angles** describe the downlink
> **channel** (interference-blind by construction — normalization discards
> absolute power), while the report's per-tone **stability** and **SNR** carry
> the beamformee's **local noise-plus-interference** — the part the beamformer
> cannot observe from its own side.

The IEEE framework text is explicit that the reported per-tone SNR's denominator
is *"the noise plus interference power measured at the beamformee"* — so
interference is in there by definition. Prior sensing work (Wi-BFI, BeamSense,
LeakyBeam) mines beamforming feedback off the air to sense *humans*; using it as
a per-tone *interference* sensor appears to be new.

## The setup

Three Realtek USB adapters, no SDR, no lab instruments. Only the chip and its
role in the sounding exchange bear on the measurement — the host driving libusb
does not, so none is named.

```
  RTL8812AU — beamformer / sounder          RTL8821AU — beamformee (client)
  ┌──────────────────────────┐  NDPA + NDP  ┌──────────────────────────┐
  │ injects NDPA, HW-gen NDP; │ ───────────▶ │ estimates H(k), returns  │
  │ self-captures the report  │ ◀─────────── │ the VHT CBF report        │
  └──────────────────────────┘    report    └──────────────────────────┘
                                             RTL8822CU — 5 MHz interferer
                                             (coupled into the beamformee's RX)
```

The RF geometry is the whole content of the setup; the two constraints that
matter:

- the **sounder ↔ beamformee** link must carry both the NDP and the report — in
  these runs at ~36 dB SNR, giving the beamformee headroom to keep decoding
  *under* interference (below);
- the **interferer** must couple into the **beamformee's** receiver, not the
  sounder's — that is what makes the returned report, rather than the sounder's
  own reception, the thing that sees it.

Roles, driven through devourer's demos:

- **Sounder** — `DEVOURER_BF_ARM_SOUNDER=1 DEVOURER_TX_NDPA=1
  DEVOURER_TX_WITH_RX=thread DEVOURER_BF_DETECT_REPORT=4`: injects the NDPA, the
  MAC hardware-generates the NDP, and it self-captures the returned report on the
  same handle (see [beamforming-self-sounding.md](beamforming-self-sounding.md)).
- **Beamformee** — `DEVOURER_BF_ARM_BFEE=<sounder SA>`; replies with no association.
- **Interferer** — a Jaguar3 part transmitting **5 MHz narrowband OFDM**,
  `DEVOURER_NB_BW=5 DEVOURER_TX_GAP_US=0`, at controllable `DEVOURER_TX_PWR`.

Reports are decoded with `tools/bf_report_decode.py`, which recovers the per-tone
Givens angles `(psi, phi)` — and hence the per-subcarrier channel and its
frame-to-frame stability.

## Two readouts

The compressed **V** matrix is *unitary* — it is normalized to unit column norm,
so it encodes only the channel's per-tone spatial *direction*, never absolute
power. That gives two distinct, complementary readouts:

| readout | quantity | senses |
|---|---|---|
| **V shape** | per-tone `|h_B/h_A|` (from `psi`), averaged over frames | the **channel** (multipath, position) |
| **V stability** | per-tone cross-frame **variance** of the angle | **interference** (a jammer corrupts the *estimate*, so the angle jitters on the hit tones) |

The second is the trick. Interference does not change the channel, and it cannot
show up as a power notch in the normalized V. But it *corrupts the beamformee's
least-squares channel estimate* on the tones it overlaps, differently every
frame — so those tones' angles become **noisy**. A per-tone spike in cross-frame
variance is a reliable, decoder-robust interference footprint that sidesteps both
the coarse MU Delta-SNR field and the all-or-nothing behaviour of a CW tone.

## Result 1 — the channel (position fingerprint)

Fixed sounder, fixed frequency (2.4 GHz ch6, 20 MHz), beamformee moved 30 cm and
back. Correlation of the 52-tone `|h_B/h_A|` shape (200 reports per capture):

| comparison | correlation |
|---|---|
| within position A (still) | **+0.92** |
| within position B (still) | **+0.99** |
| A ↔ B (30 cm move) | **−0.04** |
| return ↔ A | **+0.64** |
| return ↔ B | **−0.37** |

The fingerprint is rock-stable while the beamformee sits still, a 30 cm move
**decorrelates it completely**, and it **returns toward A** when the beamformee
returns (positively correlated with A, negatively with B) — so it is caused by
*position, not elapsed time*. The return is +0.64 rather than +0.92 for a
physical reason: at 2.4 GHz λ ≈ 12.5 cm, so the fingerprint decorrelates over
~6 cm and hand-repositioning cannot reproduce the exact multipath. That cm-scale
sensitivity is *why* the report works as a channel sensor.

## Result 2 — the interference (per-tone footprint)

Same rig, beamformee held still, the c812 5 MHz interferer switched on at ch6
centre (2437 MHz), then retuned, then off. Per-tone cross-frame **variance** of
the V angle across the 52 subcarriers (`.`=low … `@`=high):

```
  baseline OFF     :                                            (flat)
  interferer ch6   :                     :::-::::::::            (centre spike)
  interferer ch8   :                                     .**@    (spike moves high)
  OFF again        :                                            (flat)
```

- centre-band variance rose **12.6×** at the interferer's frequency (edges only 1.8×);
- retuning the interferer ch6 → ch8 **moved the spike** centre → high tones;
- switching it off returned the profile to flat.

Reports still flowed the whole time (the link had ~36 dB SNR of headroom, and a
5 MHz OFDM interferer is gentle — spread power, no AGC/sync catastrophe). So the
report reveals **which frequencies are jammed at the client**, and the footprint
tracks the real interferer frequency. This is the victim-oriented signal.

## Result 3 — the closed adaptive loop

`detect → decide → hop → verify`, fully autonomous, driven only by the report's
own interference readout. The jammer sits on ch6; the link senses its own channel
and, on detecting interference, hops to escape it:

```
[round 1] link ch6, jammer ch6  → report centre/edge = 12.1 → JAMMED → HOP ch6→ch1
[round 2] link ch1, jammer ch6  → report centre/edge =  0.4 → CLEAN → escaped, verified
```

The "act" is a **channel hop**, deliberately — an RX-side CSI mask
(`DEVOURER_RX_CSI_MASK`) is *inert* against real jamming, because a jammer's loss
is pre-FCS (AGC / sync), upstream of the equalizer where the mask acts. Hopping
away is the mitigation the sensing enables.

## Boundary conditions

Three measured limits define where the method applies:

- **The interferer must be bandwidth-limited, not a CW tone.** A single-carrier
  tone from a Wi-Fi adapter is an all-or-nothing jammer: even at the lowest gain
  index it lands tens of dB over receiver sensitivity and kills the *whole*
  receiver (AGC capture, preamble-sync and pilot corruption — all pre-FFT and
  global), rather than notching one slice. Measured: it blacked out the
  beamformee at every gain **and** jammed the sounder too (its own RX fell
  2000 → 1 frames) — no locality. A resolvable per-tone footprint needs a
  bandwidth-limited interferer well below the signal — hence the 5 MHz OFDM
  source, not a tone.
- **The detection floor is set by channel stability.** Sweeping the interferer
  power down, the low end is dominated by ambient channel drift (the same
  cm-scale sensitivity that makes Result 1 work) and by the coarse,
  split-dependent report readout, so the power → detectability curve is not
  monotonic near the floor. Detection is firm at strong interference and sinks
  into drift noise as it weakens; a calibrated floor needs a static (cabled /
  anechoic) channel.
- **The variance detector fires on the interference *transient*.** It keys on the
  *onset* of estimate corruption; as the beamformee adapts to a *steady*
  interferer the angle re-stabilizes and the variance fades (measured: three
  back-to-back sounds gave 14× → 1.7× → 2.0× with the jammer transmitting
  throughout). For the adaptive loop this is fine — every hop lands on a *fresh*
  channel, which is exactly when detection is strongest — but it means a slow
  poll-then-sound can miss a standing jammer.

## Reproduce

Sounder (beamformer + self-capture), on the ground-station adapter:

```sh
DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 DEVOURER_TX_RATE=VHT2SS_MCS0 \
  DEVOURER_TX_NDPA_RA=<beamformee-MAC> DEVOURER_TX_NDPA=1 \
  DEVOURER_BF_ARM_SOUNDER=1 DEVOURER_TX_WITH_RX=thread \
  DEVOURER_BF_DETECT_REPORT=4 DEVOURER_TX_GAP_US=4000 \
  ./build/txdemo 2>/dev/null | grep '<devourer-bf-report-raw>' > cap.raw
```

Beamformee (the client), on its adapter:

```sh
DEVOURER_CHANNEL=6 DEVOURER_BF_ARM_BFEE=57:42:75:05:d6:00 \
  ./build/rxdemo
```

5 MHz narrowband interferer (for the interference experiment), on a Jaguar3 part:

```sh
DEVOURER_PID=0xc812 DEVOURER_CHANNEL=6 DEVOURER_NB_BW=5 \
  DEVOURER_TX_GAP_US=0 DEVOURER_TX_PWR=8 ./build/txdemo
```

Decode + per-tone readouts:

```sh
tools/bf_report_decode.py cap.raw --csv cap.csv
# channel shape:      ampl_ratio_hB_hA column (Result 1)
# interference:       per-tone cross-frame variance of psi (Result 2/3)
```

## Try it — `sense`

`sense` turns the above into a runnable motion/presence sensor: one binary
drives two adapters — a sounder and a beamformee — sounds continuously,
self-captures the reports, decodes the per-tone angles in C++
(`src/BfReportDecode.h`), and prints a live readout (the mean per-tone cross-frame
variance of the phase angle) against a **self-calibrating noise floor** — a
CFAR-style test shown as `σ`, with `CLEAR` / `MOTION` verdicts.

```sh
sense --channel 6 \
  --sounder 0x0bda:0x8812 --beamformee 0x0bda:0xc812
```

Give it a few seconds to calibrate, then move near the adapters — the `σ` reading
climbs and the verdict flips to `MOTION`. The effect is strongest with the two
adapters **physically separated**; side by side they share an almost-static short
channel a hand barely perturbs.

**Full hands-on guide — hardware, placement, all the knobs, the decoding/detector
maths, and a capture→analyse loop for trying your own formulas — is in
[`examples/sense/README.md`](../examples/sense/README.md).**

## Related

- [beamforming-self-sounding.md](beamforming-self-sounding.md) — the sounding
  pipeline and register recipe this builds on.
- [pseudo-preamble-puncturing.md](pseudo-preamble-puncturing.md) — the per-tone
  *mitigation* half (`DEVOURER_RX_CSI_MASK` / `src/ToneMask.h`); note the RX mask
  targets in-band spurs on decodable frames, not a jammed slice.
- [frequency-hopping.md](frequency-hopping.md) — `FastRetune` / radiotap-driven
  hopping, the "act" in the adaptive loop.
