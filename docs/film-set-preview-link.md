# A wireless multi-camera preview link for a film set — what the measured primitives buy, and what is missing

A film unit wants what a Teradek Bolt, a SWIT CREW or a Hollyland Pyro sells:
every camera's picture on the monitors in the video village, live. The
requirement that landed on our side reads: HDMI (later 3G-SDI) in at each
camera, two or three to **twenty-five cameras at once** into one tent with a
wall of monitors and a playback recorder, **about 100 ms** camera-to-screen,
**one to one and a half kilometres**, Full HD up to 60 fps.

This page is the conceptual map of how far the measured primitives in this
project reach toward that, and where the real gaps are. It is deliberately
paired: every favourable number is quoted next to what it does *not* prove.
Everything below was measured on a bench at near-field range unless it says
otherwise, and nothing here has ever been measured at a kilometre.

## 1. Where the three commercial classes actually differ

The three product families solve the problem with three different physical
layers, and the differences matter more than the brochures suggest.

- **Bolt-class** systems carry the picture essentially uncompressed on a
  proprietary wideband PHY (an Amimon-heritage design). That is why the
  brochure says "under a millisecond": there is no codec in the loop. The
  price is a very wide channel per camera, a hard range cliff, a fixed
  transmitter-to-receiver pairing, and a bill of materials nobody can
  replicate with commodity Wi-Fi silicon.
- **CREW/Pyro-class** systems compress (H.265, tens of Mbps) and ride a
  Wi-Fi-derived or proprietary OFDM link in the 5 GHz band. Their honest
  latency figures are tens of milliseconds, their range figures are
  line-of-sight with the receiver's panel antenna, and they pair one
  transmitter with a handful of receivers.
- **Remote-production class** (encoder + cellular modem + studio decoder) is a
  different product: seconds of buffering budget, a network operator in the
  loop, no relevance to a tent 300 m from the camera.

An open build on commodity 802.11 silicon lives in the second class. Anyone
promising Bolt latency from capture → codec → Wi-Fi → decode is promising a
different physical layer. The requirement's own research document already
says this; it is repeated here because it decides every downstream number.

## 2. Airtime: twenty-five cameras is a channel plan, not a bigger dongle

The single most important reframing. What one radio can carry on one channel
is measured, repeatedly, with an SDR duty-cycle method that has no receiver
ceiling:

| Operating point | On-air throughput (occupancy × PHY rate) | After a typical 8-of-12 erasure code |
|---|---|---|
| 20 MHz, HT MCS7 | ~52 Mbps on the common 2T2R USB parts, 60–65 on the best of them, 33–43 on the 802.11ax parts over USB 2.0 | ~35 Mbps |
| 40 MHz, HT MCS7 | ~85 Mbps (one part measured) | ~57 Mbps |
| 80 / 160 MHz | tunes and airs; **no throughput has been measured** | — |

Those are single-transmitter, no-contention, bench-range figures. At a
kilometre with an omnidirectional camera antenna, MCS7 is not the operating
point; field experience from FPV links puts sustained rates at MCS1–3, which
is 13–26 Mbps of PHY rate at 20 MHz and roughly **8–18 Mbps useful per
20 MHz channel** after coding. A watchable 1080p60 H.265 preview is 4–6 Mbps;
a good one is 8–12 Mbps.

| Per-camera bitrate | 25 cameras, coded | 20 MHz channels at MCS3 | 20 MHz channels at MCS7 |
|---|---|---|---|
| 5 Mbps | ~190 Mbps | ~11 | ~6 |
| 10 Mbps | ~375 Mbps | ~21 | ~11 |

The 5 GHz band offers about two dozen non-overlapping 20 MHz channels if the
DFS ranges are usable where the shoot happens; 6 GHz adds more on the one
tri-band part — that part does 160 MHz at 5 GHz, but its 6 GHz transmit path
tops out at 80 MHz today — and with no range evidence at all. So
twenty-five cameras at a kilometre is a **ten-to-twenty-five-channel system
with one receiving radio per channel**. Not because the software is weak, but
because a receiving USB adapter has the same ~50–60 Mbps airtime ceiling as a
transmitting one, and because twenty-five uncoordinated transmitters sharing
a channel under carrier-sense do not add up — the measured behaviour of
carrier-sense against a co-channel transmitter is deferral, not capacity.

Adversarial counterpart: the per-channel figures are bench range, the
cameras-per-channel figure rests on FPV folklore about what MCS survives a
kilometre, and no experiment in this project has ever run more than two
transmitters into one receiver.

## 3. Latency: the budget is lost at the HDMI socket, not in the radio

The 100 ms budget decomposes into legs, and the ecosystem has a measured or
reported number for most of them:

| Leg | What is known | Nature of the evidence |
|---|---|---|
| Camera sensor → HDMI out | 1–2 frames (17–33 ms at 60 fps); outside our control | vendor behaviour |
| **USB HDMI capture dongle** | **+66–99 ms on its own**, no camera and no radio in the path | one user report on the OpenIPC low-latency copy audit |
| Hardware encoder, sensor input | ~6 ms capture-to-wire, ~3 ms projected with a realtime ring | measured on a SigmaStar encoder, MIPI sensor only |
| Radio: host submit → on air | 0.8 ms (USB 2) to 3.2 ms (USB 3 / PCIe) at the 99.9th percentile, carrier-sense deferral being the tail | measured on four transports |
| Erasure-coding block | a design choice: 8 × 1.5 kB at 10 Mbps ≈ 10 ms | arithmetic |
| Whole system, camera → wfb-ng → GStreamer on a PC | ~60–80 ms at 1080p60; ~100 ms in another player; ~300 ms windowed | user reports, same audit |
| Display | half a refresh plus panel: 8–16 ms at 60 Hz, more on slow panels | measured elsewhere, panel part unknowable from software |

The radio and codec legs together are under 20 ms in measured pieces. The
budget is won or lost at two places we do not usually think of as "the link":
the **HDMI ingest** and the **monitor refresh**. A USB capture dongle by itself
spends most of the budget; the only ingest shape that fits is a bridge chip
(HDMI or SDI deserialiser) feeding the encoder SoC's video input port
directly, so that the encoder sees the frame as it arrives. And the wall of
monitors must be fast panels, or the same system reads 50 ms slower for no
reason the link can fix.

Adversarial counterpart: the whole-system figures are from IP-camera sensor
paths, not from an HDMI ingest, and the dongle figure is a single report. The
one glass-to-glass instrument the ecosystem has built is not yet online, so a
p50/p95 over a thousand samples — which the requirement rightly asks for —
cannot be produced today.

## 4. Range: the dominant risk, and unmeasured

Kilometre-scale line-of-sight at 5 GHz with legal power is routine for FPV
links at low MCS with a directional ground antenna. A film set is not that:
the tent is not necessarily in view of all twenty-five cameras, the crew's
own Wi-Fi, wireless focus, comms and other departments' preview links share
the band, and the cameras move between setups.

What this project has measured that bears on it: with a parked interferer
on the channel, a static link delivered nothing while a hopping link
delivered ~95% of coded blocks; a *following* interferer could still deny the
hopping link down to ~10%. Adaptive channel exclusion moved delivery from
0.86 to 0.93 without changing the delivered rate. Narrowband 5/10 MHz buys a
theoretical 3–6 dB; the 802.11ax extended-range mode a nominal 8–10 dB; in
both cases the on-air tests prove delivery and correct classification at
bench range, not gain at distance.

Adversarial counterpart: there is **no** kilometre measurement in this
project or in the team's memory. The one range-shaped constant in the code
(an ACK timeout sized for ~15 km round trip) is a sizing statement, not a
test. Any range figure quoted to the customer before a field day with a
vendor-driver control link is a guess.

## 5. What this project adds over the usual open stack

An open build today would be wfb-ng or OpenHD on top of a patched kernel
driver. This project's userspace driver sits at throughput parity with the
wfb-ng kernel driver (SDR-verified) and adds things that matter specifically
to a many-camera set:

- **No kernel module on either end.** The camera box can be any encoder SoC
  on its vendor kernel and the receiving nodes any x86 or ARM host, running the
  same code.
- **A hardware timebase across all radios.** Beacon-stamped hardware time is
  held to a fraction of a microsecond between nodes with carrier-sense off and
  to about a hundred microseconds software-stamped. On a set that is a
  common clock under every preview stream and the shared timebase a channel
  scheduler needs — the foundation for timecode alignment, not timecode itself:
  nothing maps this clock to SMPTE timecode or an LTC output yet (§7).
  Commercial systems sell timecode passthrough as a feature; here the clock
  falls out of the link and the carriage remains to be built.
- **Per-frame rate, power and channel choice**, layered (temporal-SVC)
  unequal error protection, and an erasure code that salvages partially
  corrupt frames (about +13% recovered blocks at the highest HT rate on a
  chip-to-chip bench).
- **Evidence-driven whole-link channel migration** with a passive scout
  radio, authenticated, plus keyed frequency hopping for the jammed or crowded
  case. On a set where another department powers something up on your
  channel, this is the difference between a producer seeing a frozen monitor
  and not noticing.
- **Hardware acknowledgement and per-frame transmit reports** as delivery
  feedback and a transmit-side link sensor — the raw material for a
  control/return plane (camera control, tally), not a reliable one by
  themselves. Measured: two of the three families acknowledge 100% of
  solicited frames, the third only 64–91% depending on session shape; report
  coverage is 86–100%, and two families deliver no reports at all in a
  transmit-only session. Reliable control needs an end-to-end retry and
  acknowledgement layer on top; the video plane does not use any of this.

What it does **not** have for this job, plainly: no packaged low-latency
video pipeline (the production video path today is wfb-ng on top of it); no
encryption of the video plane; no multi-receiver ground-station story beyond
two transmitters into one receiver; no range data.

## 6. Where this meets the scheduled-RAN roadmap

The project's multi-cell roadmap (a 5G-NR-inspired scheduled network of
time-synchronised access-point cells serving many stations) was written for
robots in a warehouse. The film set is the same architecture with the traffic
direction reversed: almost everything is **uplink**, camera to village.

- The measured submit-to-air guard is 0.8–3.2 ms at the 99.9th percentile,
  and the scheduled-MAC contract sizes a slot at about twice the guard — so
  roughly 6–7 ms on the worst measured transport, or about 5 ms only if a
  bounded deadline-miss rate (~1%) is accepted. With one or two cameras per
  channel, a cell of two to four stations has a 13–26 ms superframe,
  comfortably inside the budget. With twenty-five stations on one channel it
  would be a 150–160 ms superframe — which independently confirms the
  channel-plan conclusion of §2.
- The single-cell scheduled MAC milestone is the per-channel cell: one ground
  radio, one or two camera stations, collision-free uplink where carrier-sense
  between two mutually hidden cameras would not be.
- The network-time and central-scheduler milestones are the ground station:
  ten to twenty-five co-located cells on one timebase, the controller owning
  the channel plan, admission and per-camera rate and power. That layer is
  what turns twenty-five independent links into a product, and it is unbuilt.
- The mobility milestone has a literal customer: a Steadicam operator walking
  out of one sector antenna's lobe into another.
- The ARM cell-node feasibility spike is the camera box; the film case only
  needs the coarse tier (millisecond guards, frequency separation), not the
  microsecond PCIe tier.

Multi-user OFDMA on the 802.11ax parts would be the "right" way to put more
uplinks on fewer channels; the trigger-based uplink is not available on the
client firmware those parts ship, so it is out of reach.

## 7. The missing pieces, ranked by risk

1. **HDMI / SDI ingest — nothing exists in the ecosystem.** No camera
   streamer or encoder in the OpenIPC family supports an HDMI-to-MIPI bridge
   or an SDI deserialiser; they are sensor-only. The only HDMI-receive work on
   record is an RK3588 board whose native HDMI input works once enabled in the
   device tree, and a Zynq-7010 board without a video codec that was a dead
   end. A camera-box board choice and an encoder input backend for it gate
   everything, including the one-camera MVP.
2. **Low-latency encoder mode on an external video input.** The measured
   ~6 ms encoder path and the layered-SVC-with-per-layer-FEC design are the
   right shape, but sensor-bound; nothing has been measured with a bridge chip
   in front.
3. **Multi-stream ground station.** Nothing in the ecosystem decodes more
   than one stream or composites a wall. Two shapes are possible: one central
   decoder (an RK3588 tops out around sixteen 1080p60 streams, so two boards,
   or an x86 iGPU), or **one small receive-and-decode box per monitor**, each
   on its camera's channel, with recording pulled over the tent's Ethernet.
   The per-monitor shape matches the physical wall and sidesteps putting
   twenty-five radios on one host.
4. **Multi-receiver, multi-channel coordination.** No test, document or tool
   exists for many radios or many nodes receiving many channels on one
   timebase. The PTP-synchronised test rig is the nearest scaffold; the
   roadmap's Phase B is this problem in different clothes.
5. **Range evidence.** A field day with directional ground antennas and a
   vendor-driver control link, before any number is promised.
6. **The glass-to-glass instrument online.** The requirement's p50/p95 over a
   thousand samples needs the photodiode meter published and running; today
   the ecosystem's latency figures are user reports.
7. **Video-plane encryption and authentication.** The customer asks for
   AES-256 with per-pair keys; the alternative open transport has it, this one
   does not.
8. **Timecode carriage.** The hardware timebase exists; nothing maps it to
   SMPTE timecode or an LTC output on the receive box. A genuine
   differentiator, unbuilt.
9. **Licensing for a registry-listed product.** Every open transport in
   this space is GPL — the alternatives GPL-3, this project GPL-2 — and the
   customer's own research flags GPL as a blocker for closed firmware. The
   userspace shape helps with kernel-module obligations, not with the licence
   itself. A question for the customer's counsel, not an engineering gap.

## 8. A staged build that does not promise what is unmeasured

1. **One camera, one monitor.** A camera box with a bridge-chip HDMI input,
   hardware H.265, and the userspace radio; one receive-and-decode box;
   instrumented end-to-end with the photodiode meter from day one. Pass/fail
   on the customer's 80 ms p95, measured, before anything else is built.
2. **Four cameras, four channels, four monitors** on one hardware timebase.
   This is the single-cell scheduled MAC and network-time milestones in
   product clothing, and the first point at which the channel plan is real.
3. **A range day.** Directional ground antennas, a vendor-driver control link
   on identical hardware, cold-cycled between runs, delivery versus distance at
   each MCS. The cameras-per-channel figure in §2 gets replaced by a measured
   one here.
4. **Scale to the wall** through the central channel planner, and only then
   quote twenty-five.

Every number on this page is a bench figure at near-field range unless
labelled otherwise; the customer should be told so in the same sentence as
the number.
