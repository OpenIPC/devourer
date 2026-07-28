# VHT on the 2.4 GHz band

802.11ac (VHT) is a 5 GHz standard. Running a VHT PPDU in the 2.4 GHz band is a
vendor extension — the thing marketed as **NitroQAM** or **TurboQAM** on
"AC1300"-class adapters. Its selling point is the top of the VHT rate ladder:
256-QAM at MCS8/MCS9, which standards-only 802.11n does not have.

## Selecting it

Nothing gates this. Rate resolution never reads the band, so a VHT rate on a
2.4 GHz channel is accepted end to end — `DEVOURER_TX_RATE=VHT1SS_MCS8/20` on
channel 6, or a radiotap VHT field on a per-packet basis. The receiver must be
able to decode it; a standards-only 802.11n station sees nothing at all.

## Read this first: qualify the receiver, or measure the receiver

Delivery is a property of a *link*. A ground station whose modulation cliff sits
at the rate under test reports its own limits, not the transmitter's — and it
swings between "fine" and "zero" on a few dB of ambient while the robust rates
stay pinned. Two RTL8822BU adapters, identical silicon and identical code, same
transmitter and channel, differing only in antennas:

| ground (both 8822BU) | MCS1 | MCS4 | MCS5 | MCS6 | MCS7 |
| --- | --- | --- | --- | --- | --- |
| TP-Link Archer T3U — **internal** antennas | 98.1 | 97.1 | 79.1 | 49.0 | **2.9** |
| Comfast CF-924AC V2 — **two external** | 95.8 | 95.8 | 95.7 | 94.4 | **95.9** |

Run `tests/ground_station_qualify.sh` before trusting any number here; it refuses
the pairing when the test rate is off the flat part of the ladder. Every figure
below was taken through a qualified receiver.

## Cold-cycle too

**High-order constellation TX degrades across warm re-inits.** After a run of
back-to-back devourer sessions on the same adapter, 64-QAM and up stop decoding
entirely while 16-QAM and below keep working. There is no error anywhere: the
transmitter reports every frame submitted, the receiver reports a strong RSSI
and a clean EVM on the frames it does decode, and the result looks exactly like
a link too weak to carry a denser constellation.

Measured on an RTL8812AU, same channel, same power, same inter-frame gap, the
only difference being a VBUS power cycle:

| MCS7 (64-QAM 5/6) | delivery |
| --- | --- |
| warm (many prior sessions) | 0% |
| after VBUS cold cycle | 58% |

This is the general "chip retains state across soft re-init" trap in the root
`CLAUDE.md`, and it is severe enough here to invert a conclusion. An
`authorized` toggle is **not** sufficient — it re-enumerates without removing
power. Use a real VBUS cycle (`REGRESS_VBUS_MAP` + uhubctl, hub ports with
per-port power switching only). `tests/nitroqam_waterfall.sh` does this before
every cell by default, on both ends.

The degradation itself is an open bug, tracked with its own reproduction in
`tests/warm_tx_degradation_repro.sh` — a deployed link cannot power-cycle its
adapter, so it needs a software-only recovery path.

## What is measured

Injection on 2.4 GHz at 20 MHz, delivery counted from FCS-clean canonical-SA
frames, and every sampled `rx.txhit` decoded back to a rate so the receiver
confirms *which* modulation arrived. A frame commanded as VHT that decodes as
HT is a fallback, not a pass.

Measured from an RTL8812AU into a qualified CF-924AC (external antennas):

| rate | delivery | modulation verified |
| --- | --- | --- |
| VHT 1SS MCS8 — **256-QAM** | **77.1%** | 37/37 decoded as `vht1ss_mcs8` |
| VHT 2SS MCS0 | 95.3% | 54/54 |
| VHT 2SS MCS4 | 90.8% | 43/43 |
| VHT 2SS MCS7 | 15.5% | 7/7 |
| VHT 2SS MCS8 — 256-QAM, 2 streams | 0.6% | not reached |

| Transmitter | Generation | Peer | Confirmed on 2.4 GHz |
| --- | --- | --- | --- |
| RTL8812AU | Jaguar1 | CF-924AC (8822BU) | VHT 1SS to **MCS8 (256-QAM)**, 2SS to MCS7 |
| RTL8822BU | Jaguar2 | RTL8814AU | VHT 1SS MCS0, 2SS MCS0 |
| RTL8812CU | Jaguar3 | RTL8814AU | VHT 1SS MCS0, MCS4 |

`AdapterCaps::vht_2g4_ok` records the VHT format as a transmit claim for those
three dies. The Jaguar2/Jaguar3 rows are floors, not limits — they were measured
through an unqualified receiver and have not been re-run.

**VHT MCS9 is not a legal rate at 20 MHz for 1 or 2 spatial streams.** Asking
for `VHT1SS_MCS9/20` gets an MCS8 PPDU: the run delivered 311 frames and every
sampled one decoded as `vht1ss_mcs8`. That is correct hardware behaviour, and
the harness's modulation gate is what caught it rather than reporting 311
frames as an MCS9 pass. Exercising MCS9 at all requires 40 MHz.

## Cross-check against the vendor driver

Whether a high-MCS wall is ours or the rig's cannot be answered from delivery
alone. `tests/nitroqam_kernel_ab.sh` runs devourer and the vendor driver
(`reference/rtl8812au`, the OpenHD fork) on the **same dongle**, same channel,
same rates, interleaved, judged by an AR9271 sniffer — a different vendor's
radio sharing no silicon, firmware or driver code with either side.

| rate | devourer | vendor 88XXau |
| --- | --- | --- |
| MCS5 (64-QAM 2/3) | 61.7% | 68.4% |
| MCS7 (64-QAM 5/6) | 58.2% | 64.3% |

devourer is on par with the vendor driver at the top of the HT ladder. There is
no devourer-side high-MCS defect.

Two traps that harness encodes, each of which produced a wrong answer first:

- **The in-tree `rtw88` is not a usable control.** It accepts frames on a
  monitor netdev and reports them injected while airing nothing the sniffer
  decodes — it reads as a kernel-side failure at every rate. Use the vendor
  driver from `reference/`.
- **Do not run all of one side then all of the other.** Ambient 2.4 GHz moved
  one side's delivery 2× between runs. Interleave per rate, and bracket every
  cell with a robust-rate sanity cell so a wedged half is reported invalid
  rather than as a zero.

## What is still not measured

- **256-QAM on 2 spatial streams** (`VHT2SS_MCS8`) reads 0.6% where 2SS MCS7
  reaches 15.5% — the ladder is rolling off, so this is the link running out of
  margin rather than anything specific to the mode. It is the one genuine
  remaining gap to the 2×2 headline.
- **40 MHz, hence the headline 400 Mbps rate.** `rxdemo` with
  `DEVOURER_BW=40` delivers zero on both Jaguar1 and Jaguar2 — including for a
  20 MHz transmitter on the primary channel, at either primary-channel offset,
  in both role directions, and including the most robust 40 MHz rate (MCS0/40).
  Re-tested on a **qualified** receiver and still zero, so this is a real
  receiver-side gap and not a margin artifact. Note the existing 40 MHz TX
  validation (`tests/jaguar2_tx_bw40.sh`) uses a *kernel* sniffer, so the
  devourer 40 MHz receive path may never have been exercised.
- **Link asymmetry.** 8812AU → 8822BU reaches MCS7 at 68%; the reverse
  direction is dead at MCS7 even with both ends cold. Pick the direction
  deliberately when measuring, and do not read a dead direction as a chip
  limit.

## Scope

This is a close-range, strong-link throughput mode — the opposite direction
from the low-rate narrowband work that long-range FPV links want. It costs SNR
to buy PHY rate.

There is no regulatory enforcement, and the 2.4 GHz regulatory limit tables
have no VHT rows to clamp against: on Jaguar2/8822B the TXAGC walk bounds
2.4 GHz VHT by the HT limit instead of letting the missing row fall through
unclamped, but on Jaguar1 that fallthrough is not bounded. The caller owns
compliance.

## Harnesses

- `tests/nitroqam_waterfall.sh` — delivery-versus-power sweep with the
  modulation gate, cold-cycling both ends per cell. Its power axis defaults to
  `SetTxPowerOffsetQdb` (`--pwr-mode offset`), which preserves the chip's
  calibrated per-rate shape; the flat `DEVOURER_TX_PWR` index
  (`--pwr-mode flat`) is not a neutral substitute.
- `tests/nitroqam_kernel_ab.sh` — the devourer-vs-vendor control above.
- `nitroqam_waterfall.py --self-test` — headless decode/crossing math, wired
  into `ctest` as `nitroqam_decode_math`.

An SDR duty-cycle comparison was tried as a transmitter-only substitute for a
receiver and **does not work** on this bench at any payload from 1400 to 4000
bytes: the host feed rather than the channel sets the occupancy, so every
encoding reads the same duty. It was dropped rather than shipped as a harness
that cannot return a valid answer.
