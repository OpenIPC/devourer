# HE trigger-based uplink (Kestrel)

This document describes devourer's 802.11ax trigger-based uplink support on the
Kestrel generation (RTL8852BU/8832BU and RTL8852CU/8832CU) — how a Trigger frame
is put on the air, what the shipped firmware does and does not do with it, the
device API, and how the mechanism is used to build an adaptive link. Pre-AX
generations (Jaguar1/2/3, 802.11n/ac) have no Trigger-frame concept and report
`AdapterCaps.trigger_ul_ok = false`.

## What a Trigger frame is

802.11ax moves uplink scheduling into the MAC/PHY. An access point sends a
**Trigger frame** — an 802.11 control frame (frame-control byte `0x24`) whose
Common Info carries the uplink bandwidth, GI/LTF and AP TX power, and whose
per-user Info fields each grant one station a **resource unit**, an MCS, a
spatial-stream allocation, and a **target RSSI**. A compliant station that finds
its AID in a Trigger answers a fixed SIFS (~16 µs) later with an **HE TB PPDU**
(trigger-based PPDU) on exactly the granted resource — hardware-scheduled and
contention-free. This is the standard's mechanism for coordinated, low-jitter
uplink, and the forward path to cellular-style scheduled access on Wi-Fi.

A **BFRP** (Beamforming Report Poll) is a Trigger-frame variant that solicits a
compressed-beamforming report; the sounding sequence (NDPA → NDP → BFRP) that
carries it is the firmware's own use of the Trigger primitive.

## Two ways to put a Trigger on the air

There are two distinct transmit paths, and on the shipped RTL8852 *client*
firmware they behave very differently.

### Firmware-scheduled

The driver hands the firmware a command and the firmware assembles and schedules
the Trigger itself:

| Command | Class / func | Role |
|---|---|---|
| F2P trigger | `CL_FR_EXCHG` / `F2P_TEST` | one-shot Basic Trigger |
| UL_FIXINFO | `CL_FR_EXCHG` / `TBLUD` | autonomous periodic UL-OFDMA scheduler table |
| Sounding | `CL_SOUND` / `SET_SND_PARA` | NDPA → NDP → BFRP sequence, arms RX for the report |
| TWT | `CL_TWT` / `*` | wake/service-period agreements |

These are byte-exact ports of the vendor `mac_ax` command encoders and are
firmware-accepted (no SER). **On the shipped client firmware, none of them put a
Trigger on the air.** The F2P command is a manufacturing/hardware-verification
entry with no production caller and is silently dropped. The sounding command is
accepted and *engaged* — the firmware holds a resource per one-shot sounding, and
the command queue stalls after seven un-completed soundings — but it does not
transmit the NDPA/BFRP. This was confirmed on-air: a monitor that decoded 139,532
frames from the AP and station (beacons, RTS, auth/assoc, data, ACKs) decoded
zero Trigger frames while the sounding command was firing. The firmware exposes
the command surface but not the trigger-transmit scheduler that AP-class firmware
runs.

### Host-injected

Instead of asking the firmware to build the Trigger, the driver builds the raw
802.11ax Trigger-frame bytes on the host and transmits them through the ordinary
packet path — the same path that airs beacons, authentication/association
responses and data frames:

1. `build_basic_trigger()` (`src/TriggerTwt.h`) assembles the frame from a
   `TriggerConfig`: frame-control `0x24`, duration, RA/TA, the HE Common Info
   field, and one User Info field per granted user (AID, RU allocation, MCS,
   spatial streams, target RSSI).
2. `IRtlDevice::SendTrigger()` prepends a radiotap header (legacy OFDM, so any
   monitor decodes it) and calls the normal `send_packet()` transmit path. The
   firmware's trigger scheduler is bypassed entirely.

This works on the client firmware because it rides the plain management-frame
transmit path, which functions normally. On-air validation: with an associated
station granted by AID, the AP injected Basic Triggers and an independent monitor
decoded them with the exact commanded parameters (AP power, AID, RU, MCS, NSS),
confirming well-formed HE Trigger frames on the air.

A group-addressed (broadcast) Trigger is marked BMC in the transmit descriptor so
the firmware transmits it once rather than retrying an unacknowledged frame to the
retry limit; unicast frames are unaffected and still retried normally.

## What closes and what does not

Host-injection puts a real, correctly-decodable HE Trigger on the air. It does
**not**, on its own, elicit an HE TB PPDU from a standard station: a host-injected
frame does not arm the MAC's SIFS-scheduled receive window, so the receiving
station gets no hardware timing cue to answer at trigger+SIFS. The
hardware-scheduled response requires a firmware-scheduled Trigger, which the
client firmware does not air. In short:

- **Trigger on the air** — delivered (host-injection).
- **Hardware SIFS-timed TB PPDU response** — not available on client firmware.

## Device API

The surface is on `IRtlDevice`; each call returns `false` on pre-AX generations.

- `SendTrigger(const TriggerConfig&)` — air one Basic Trigger. Default path is
  host-injection; set `DeviceConfig.debug.kestrel_trigger_f2p` (env
  `DEVOURER_KESTREL_TRIGGER_F2P=1`) to route through the firmware F2P command
  instead, for comparison.
- `RegisterPeerSta(mac, macid, addr_cam_idx)` — register an associated station so
  a Trigger's per-user grant resolves against it.
- `ConfigureUlOfdma`, `ConfigureTwt` / `TwtBindSta` / `ConfigureTwtOfdma`,
  `StartSounding` / `RegisterBeamformee` — the firmware-scheduled commands above.

`TriggerConfig` (`src/TriggerTwt.h`) holds the uplink bandwidth, GI/LTF, AP TX
power and up to eight `TrigUser` grants; `he_ru_alloc()` builds the RU-allocation
subfield and `he_tgt_rssi_enc()` the target-RSSI subfield. A frame carrying its
own rate radiotap overrides the mode per packet, so rate, bandwidth and channel
are per-Trigger selectable like any injected frame.

## Adaptive-link use

The Trigger frame is a per-user control channel: every grant independently
carries the RU, MCS, spatial-stream count and target RSSI the transmitter should
use. That makes it the natural carrier for a closed-loop adaptive uplink.

- **Rate and RU adaptation.** The AP tracks each transmitter's link quality (RSSI,
  SNR and EVM are available per received frame, and `GetRxQuality()` /
  `cell::UeRxAttribution` aggregate them per station) and encodes the chosen MCS
  and RU width into the next Trigger's User Info. A degrading link is granted a
  lower MCS or a narrower RU; a recovering one is stepped back up.
- **Uplink power control.** The target-RSSI subfield asks the transmitter to hit a
  specific receive level, moving the AGC operating point off the near-field
  saturation region (strong RSSI with poor EVM) or up out of the noise — the same
  distinction `LinkHealth` classifies from the RX sensor tuple.
- **Per-packet steering.** Because injection is radiotap-driven, the Trigger's own
  PHY rate, bandwidth and channel are chosen per packet, and combine with
  `SetTxPowerOffsetQdb()` and `FastRetune()` for a link that adapts rate, power and
  frequency together.

Two link topologies use this today:

- **Software-timed devourer↔devourer link.** Both ends run devourer. The AP injects
  a Trigger with the desired uplink parameters; the peer decodes it (the RX path
  emits an `rx.trigger` event with the full Common/User info) and adapts its next
  uplink transmission accordingly. The response is software-scheduled rather than
  hardware-SIFS, which fits the millisecond-scale cadence of a long-range video
  link and sidesteps the client-firmware TB-response gap entirely.
- **Signaling to standard stations.** A broadcast Trigger is a low-overhead way for
  the AP to publish per-user uplink parameters that a cooperating station reads and
  applies, independent of whether that station's hardware answers at SIFS.

For a fixed-schedule beacon-timed link, the beacon engine (`StartBeacon`,
`PinBeaconTbtt`) remains the hardware-timed primitive; the Trigger path adds the
per-user, per-packet uplink-parameter channel on top of it. See
`docs/scheduled-mac.md` for the beacon-timed scheduling contracts and
`docs/time-distribution.md` for the TSF/timestamp primitives.

## Validation

`tests/ul_sounding_e2e.sh` brings up a devourer HE software-AP on a Kestrel
adapter, associates a real `rtw89` station, and drives either the sounding
command (default) or host-injection (`INJECT=1`), with a separate-bus monitor and
a USRP B210 as on-air witnesses. It reports the injected-vs-decoded Trigger
counts, the B210 preamble-classifier delta, and `ul_tb` (the station's TB-PPDU
response, observed via the AP's own RX). The headless golden-byte encoder checks
run under `ctest` (`kestrel_sched`).
