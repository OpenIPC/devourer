# Adaptive-link building blocks

The [energy-minimizing adaptive link](adaptive-link.md) sets out *what* to
optimize — the fewest Joules per delivered bit under a video-quality floor, ridden
by pushing the highest modulation the link bears and spending only the power that
clears it. This document is the companion: *what* devourer actually exposes to
build that loop with. It catalogs the concrete primitives — the **levers** an
adaptive controller can pull, the **sensors** it can read, the **active stimuli**
it can emit to probe the link, and how they compose into a decision.

Devourer is the **mechanism, not the policy.** It gives a ground station a clean
view of the link and a drone a set of runtime-changeable transmit parameters; the
closed loop that scores the link and picks the operating point lives upstream (in
wfb-ng / OpenIPC / an adaptive sidecar). Everything below is a building block that
loop consumes or drives.

## The loop, in one line

**Sense → decide → act.** The ground senses link quality, a policy decides the
operating point, the drone acts on its transmit parameters (with local safety
overrides). The sections map onto that: *sensors* feed the sense step, *levers*
are the act step, and *active stimuli / probes* let the controller measure a
candidate operating point **before** committing video to it — turning a reactive
loop (discover the ceiling by dropping frames) into a proactive one.

## Levers — what the link can change at runtime

All of these move without a re-init; most move per-packet or within a few
milliseconds, so a controller can retune between frames.

| Lever | Effect on the link | Effect on energy/bit | How it moves |
|---|---|---|---|
| **Modulation / MCS** (time-on-air) | strong | **strong** — less airtime, fewer Joules/bit | per-packet radiotap rate, or the device TX-mode default (`DEVOURER_TX_RATE`); immediate |
| **FEC strength** | strong | strong | application-layer (the outer code / per-layer ladder), not a PHY register |
| **Channel / bandwidth** | strong | moderate | per-packet retune (~1–2 ms intra-band fast retune, longer on a band change); the channel-agility lever |
| **Transmit power** | **strongest** | **weak** — the always-on baseline draw dominates | runtime TXAGC override, re-applied without a channel switch |
| **Active receive chains** | conditional | **conditional** — pays only when the antennas decorrelate (motion) | RX-path enable mask; a fade-state lever, not a range lever |
| **Duty cycle** | — | direct | inter-frame gap; back-to-back for maximum airtime, idle to save it |

The energy asymmetry is the crux the design leans on: modulation and airtime are
strong energy levers, transmit power is a strong *link* lever but a weak *energy*
lever. So the reflex is to ride the fastest modulation the link tolerates and
spend the minimum power that clears it — which is precisely a **boundary search**,
and the active probes below exist to find that boundary cheaply.

## Sensors — how the link is measured

Two classes, differing in whether a frame has to arrive.

**Frame-driven** (ride ambient traffic — a received frame carries them):

- **per-chain RSSI / SNR / EVM** — link-quality scalars averaged over the channel,
  per receive chain. The primary "how good is the link right now" signal, but only
  as fast as frames arrive.

**Frame-free** (no received frame required — the receiver reads them off the
baseband directly; see [`rx-spectrum-sensing.md`](rx-spectrum-sensing.md)):

- **false-alarm (FA) + CCA counters** — in-band energy and channel-busy, read as a
  delta over a poll interval. Spike when a carrier appears.
- **DIG initial gain (IGI)** — a noise-floor proxy.
- **NHM noise histogram** — a 12-bucket, IGI-referenced in-band **power
  distribution**. Its mass shifts into higher buckets under a rising interferer —
  a coarse spectrum-free "how much energy, and how high" without a sweep.
- **per-tone interference localizer** — from a self-sounded beamforming report,
  per-subcarrier SNR / phase-variance that locates a narrowband interferer to a
  fraction of the channel. The finest frequency-resolution sensor the silicon
  offers (no raw per-subcarrier CSI is exported to the host).

**Thermal** (a local safety input, not a link sensor):

- **thermal meter + baseline** — the PA's relative temperature and its trend. The
  drone's *local safety override* (thermal back-off) reads this; it also bounds
  how much power/duty the controller may request.

## Active stimuli — probing the link on purpose

Passive sensing waits for the link to reveal itself. An active stimulus makes the
link reveal itself on demand.

- **CW tone** (`DEVOURER_CW_TONE`) — a bare, unmodulated RF carrier at the channel
  center. A controllable **narrowband** probe / interferer: park it on a channel
  and a second adapter's energy sensor detects it. Useful for reciprocity checks
  and for injecting a known interferer to validate the sensors — but it occupies a
  single frequency and doesn't stress the amplifier the way real traffic does.
- **Modulated continuous TX** (`DEVOURER_CONT_TX`) — the modulated sibling: a true
  100%-duty **full-channel** OFDM carrier at a real rate, on all three chip
  generations (Realtek's MP hardware continuous-TX mode). Because it fills the
  whole 20/40/80 MHz and loads the PA like real traffic, it is the *realistic*
  stimulus — what you want for spectral-occupancy, power, and thermal-duty
  characterisation. It idle-holds the carrier until stopped, then restores the
  chip. 100% duty is the worst-case PA heat, so it is a debug / characterisation
  stimulus — not for sustained use; pair it with the thermal telemetry and watch
  the drift. (SDR spectrum-shape check: `tests/sdr_spectrum.py` distinguishes a
  full-channel modulated block from a bare tone by occupied bandwidth.)

The two are complementary: the tone probes *one frequency* narrowly; the modulated
carrier probes *the whole channel* realistically.

## Active probing — turning a stimulus into a decision

The **active link-probe** (`tests/link_probe.sh` + `tests/link_probe.py`) composes
a stimulus and a sensor into an operating-point recommendation. One adapter emits
a modulated feed and **sweeps a lever** in steps (marking each step); the ground
station reads its per-step SNR and NHM; the analyzer aligns the two by time and
reports the **margin-vs-lever curve** plus the operating point that meets a target.

The probe deliberately uses a **beacon feed** (fresh, gap-separated frames), not
the 100%-duty continuous carrier: the receiver needs decodable per-frame SNR at
each step, and the continuous carrier — a spectral/thermal stimulus — is not a
clean frame source (its looped payload isn't FCS-valid, and a gapless carrier
offers no frame boundaries to lock onto). Stimulus and probe are thus decoupled:
the continuous carrier characterises the spectrum/power/thermal; the beacon feed
carries the per-frame link quality.

- **Power↔margin** — sweep transmit power, read the ground SNR at each level, and
  pick the *minimum power that clears the margin*. This is the energy-min reflex
  made measurable: rather than guess the power or discover it by degrading video,
  measure the cheapest power that holds the link. (Sweep the noise-limited,
  lower-power regime for a clean monotonic curve; very high power into a strong
  link just saturates the receiver.)
- **MCS-headroom** — the same harness with the rate as the swept axis: does the
  next modulation still clear the SNR/EVM floor? A proactive rate-adaptation input.

This is the concrete building block an energy-min controller uses to place the
operating point *before* committing the video stream to it.

## Composing the blocks into the energy-min loop

The design's decisions map onto the blocks above:

- **Ride the highest MCS, spend the least power that clears it** — the
  power↔margin and MCS-headroom probes measure that boundary; the rate and power
  levers act on it.
- **Pick a clean channel / bandwidth** — sweep the frame-free energy sensor across
  candidate channels (a coarse spectrum map), or localize an interferer per-tone,
  then retune the channel lever there. A modulated continuous burst on a candidate
  channel lets the far end confirm it carries the target rate.
- **Adapt receive chains to the fade state** — combine more chains under motion
  (decorrelated antennas fill fades), collapse to one on a still, strong link; the
  RX-path lever, driven by the per-chain sensors.
- **Hold the thermal / regulatory ceiling** — the thermal telemetry bounds the
  power/duty the controller may request; the continuous-TX stimulus is the
  worst-case duty for characterising that ceiling.
- **Re-find each other when feedback drops** — a modulated continuous (or periodic)
  carrier as the ground's cheap re-find beacon, detected by the drone's low-duty
  energy sensor — the asymmetric-duty rendezvous the design describes.

None of these is a policy in itself; each is a lever to pull or a number to read.
The controller that weighs them against the energy objective and the per-layer
quality floor is the subject of [`adaptive-link.md`](adaptive-link.md), and it
rides upstream of devourer.
