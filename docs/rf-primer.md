# A visual primer on the RF machinery

devourer talks to a Wi-Fi radio at a very low level — subcarriers, constellations,
gain control, the transmit and receive chains. If you're new to that machinery,
the terms in the other docs (per-tone SNR, EVM, CCA, AGC, occupied bandwidth) can
feel like jargon. This page is a picture book: nine short animations, each
built in the DEVOURER live-monitor style, that show what the machinery actually
looks like — from a single subcarrier all the way to a hopping, diversity-combined,
bandwidth-hopping link. Read it top to bottom and the rest of the docs will click.

Everything here is grounded in what devourer measures — the constellation noise
follows the textbook AWGN model, the spectrum levels are from a real USRP B210
capture, and the AGC behaviour is the very effect the energy sensor keeps seeing.

## 1. The channel — what a "subcarrier" is

![OFDM channel anatomy](img/ofdm_anatomy.gif)

A Wi-Fi channel isn't one frequency — it's a comb of many narrow **subcarriers**.
A 20 MHz channel is 64 of them, spaced 312.5 kHz apart: a **DC null** left empty
in the middle, a handful of **pilot** tones the receiver uses to track drift,
dozens of **data** tones that carry the bits, and empty **guard** bins at the
edges so the signal doesn't spill into the neighbours. Wider channels (40/80 MHz)
just add more of the same 312.5 kHz tones; the narrowband 5/10 MHz modes re-clock
to *closer* spacing to fit a thin channel (more robust, less throughput).

This comb is the coordinate system everything per-tone lives in — the
[per-subcarrier SNR waterfall](beamforming-self-sounding.md), the
[NHM power buckets](rx-spectrum-sensing.md), and the tone mask all index into it.

## 2. Modulation — how bits ride the signal, and why SNR matters

![IQ constellation vs SNR](img/constellation.gif)

Each subcarrier carries bits by taking one of a set of points on the I/Q plane —
the **constellation**. QPSK has 4 points (2 bits each); 256-QAM packs 256 points
(8 bits each). More points = more bits per symbol = more throughput. The catch:
noise nudges each received point away from its ideal spot (that displacement is
the **EVM**), and if it drifts across the boundary into a neighbouring point's
cell, that's a **bit error**. The animation holds one channel and climbs the
modulation: QPSK's points are far apart so there's margin to spare, but 256-QAM
packs them so tight the *same* noise smears the clusters together and the link
breaks. That boundary — the highest modulation a given SNR can hold — is exactly
what the [MCS-headroom probe](adaptive-link-building-blocks.md) measures.

## 3. Building the waveform — the transmit pipeline

![TX pipeline](img/tx_pipeline.gif)

So how does a block of bits actually *become* those constellation points on those
subcarriers? A short assembly line: the bits are **scrambled** (whitened so there
are no long runs), **forward-error-coded** (redundancy added so the receiver can
repair errors), **interleaved** (spread out so a fade damages many codewords a
little rather than one a lot), **mapped** onto the subcarrier constellations, run
through an **inverse FFT** that turns all those subcarriers into one time-domain
OFDM symbol, given a **cyclic-prefix** guard (the tail copied to the front, so
echoes off walls don't smear the next symbol), and finally up-converted and
radiated. That last waveform is exactly what the spectrum analyzer below shows.

## 4. On the air — a bare tone vs a modulated carrier

![CW tone vs modulated spectrum](img/spectrum_compare.gif)

Look at the same signals on a **spectrum analyzer** (power vs frequency). A
**CW tone** puts all its energy at one frequency — a single tall spike, nearly
zero bandwidth; it's a clean narrowband probe or interferer
(`DEVOURER_CW_TONE`). A **modulated carrier** spreads its energy across every
subcarrier — a flat block filling the whole 20 MHz (`DEVOURER_CONT_TX`); it's what
real traffic looks like, and the realistic stimulus for link probing. Same
transmitter, two completely different spectral footprints. (Levels here are a real
B210 capture on ch100: a −25 dB floor, the tone ~+18 dB above it, the block ~+28.)

## 5. At the receiver — gain control, and why a strong signal goes deaf

![AGC gain and saturation](img/agc_saturation.gif)

The receiver can't handle every signal level directly, so an **AGC** (automatic
gain control) turns its gain up for weak signals and down for strong ones, aiming
to keep the ADC in its sweet spot. But the gain has a floor. When a signal is
*too* strong — a transmitter co-located inches away — the AGC runs out of
attenuation, the ADC input exceeds full scale, and the waveform **clips** flat
against the rails. A clipped waveform can't be demodulated: the receiver goes
deaf. That's why, in the [sensing docs](rx-spectrum-sensing.md), a *moderate*
interferer makes the CCA counter **spike** while a *strong* co-located one makes
frames and CCA **collapse** toward zero — the AGC saturating is the collapse.

## 6. Measuring the channel — beamforming self-sounding

![Beamforming self-sounding sequence](img/bf_sequence.gif)

To know *how good* each subcarrier is, you have to measure the channel between two
radios. The sequence: the **sounder** announces (NDPA), sends a **known waveform**
on every subcarrier (NDP), the **beamformee** compares what arrived to what it
knows was sent — that's the per-subcarrier channel `H(k)` — and sends back a
compressed **CSI report**. With two adapters you own, you play both roles yourself
(*self-sounding*). That report is the source of the per-subcarrier SNR waterfall,
the per-tone interference localizer, and the motion sensor.

## 7. Combining two antennas — diversity under motion

![MRC antenna diversity](img/mrc_diversity.gif)

Multipath makes a signal **fade** — deep dips that come and go. Two antennas help,
but only when they see *different* fades. Held **still**, closely-spaced antennas
see almost the same channel: they dip together, so combining them (maximal-ratio
combining) barely fills the holes and the second chain is mostly wasted power.
Under **motion** the antennas decorrelate — when one is in a fade the other
usually isn't — so the combined signal fills the deep fades and outages drop
sharply. That's why the number of active receive chains is a *fade-state* lever,
not a range lever, and why a motion signal tells the controller when to open them.

## 8. Spreading across the band — frequency hopping

![Frequency-hopping pattern](img/hop_pattern.gif)

Instead of parking on one channel, the link can **hop** channel to channel every
dwell, spreading its energy across the band. A narrowband interferer sitting on
one channel then only clips the occasional hop that lands on it — every other hop
escapes. Done per-packet (`DEVOURER_HOP_*`), hopping doubles as a
frequency-diversity interleaver for the outer FEC: losses are spread thin across
frequencies instead of wiping out a run of packets on one.

## 9. Trading robustness for throughput in time — bandwidth TDMA

![Bandwidth TDMA — two stations flipping together under a shared clock](img/tdma_schedule.gif)

Narrowband (section 1) is more robust but slower; a wide channel is faster but
needs a healthier link. You don't have to pick one for the whole session — you
can **alternate them in time**. The link runs a schedule of bursts: a narrowband
burst carries the frames you cannot lose (a keyframe, a control message) at a
robust rate, then a wide burst carries the bulk at a fast rate, then back — so
the occupied width **breathes** burst to burst. What makes this practical is that
switching bandwidth is nearly free (~0.2 ms — a single baseband re-clock register
via [`FastSetBandwidth`](narrowband.md)), so the schedule can flip many times a
second. The catch, and the reason it's *bursts* and not per-frame: narrowband and
20 MHz are different sample-clock domains, so a receiver decodes exactly one width
at a time — both ends have to flip **together**. Either the receiver switches in
lockstep with the transmitter (synced by a shared clock or by the transmitter's
own marker frames), or a second receiver camps permanently on the narrowband band
as an independent, always-listening robust link for the critical frames. The
runnable version is the [`tdma`](narrowband.md) example; the switch machinery it
rides on is in [`narrowband.md`](narrowband.md).

---

## Where to go next

With the machinery in hand, the rest reads straight:

- [`rx-spectrum-sensing.md`](rx-spectrum-sensing.md) — reading energy, noise, and
  interference off that channel comb, frame-free (includes the animated NHM
  monitor).
- [`beamforming-self-sounding.md`](beamforming-self-sounding.md) — measuring the
  per-subcarrier channel with two adapters (the animated SNR waterfall).
- [`adaptive-link-building-blocks.md`](adaptive-link-building-blocks.md) — the
  levers, sensors, and probes that turn all of the above into an adaptive link,
  and [`adaptive-link.md`](adaptive-link.md) — the objective they serve.
- [`narrowband.md`](narrowband.md) — the 5/10 MHz re-clock machinery, the cheap
  bandwidth switch, and the burst-TDMA example from section 9.
