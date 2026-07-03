# Pseudo preamble puncturing — using a wide channel with a dirty slice

Wi-Fi 7 preamble puncturing lets a transmitter keep an 80/160 MHz channel
while skipping one dirty 20 MHz slice: the EHT preamble carries a per-PPDU
puncturing bitmap, the TX IFFT zeroes the punctured tones, and the receiver
skips them during equalization. This document is the answer to "how close can
the 802.11ac-era Jaguar silicon get?" — what the chips offer, what devourer
now exposes, and what a jammed-slice experiment measured about each
mechanism's real worth.

## Why true puncturing is out of reach

Puncturing is **PHY signaling, not a driver trick**. Three independent walls:

- **The VHT preamble must be contiguous full-band.** L-STF/L-LTF/L-SIG/
  VHT-SIG are wideband training sequences; there is no 802.11ac encoding for
  "this 80 MHz PPDU skips 20 MHz in the middle", so even a hypothetically
  tone-nulled transmission could not tell a receiver which tones to skip.
- **No TX tone nulling exists on this silicon.** Neither the vendor drivers
  nor the firmware expose per-subcarrier TX amplitude control on any Jaguar
  generation. Every tone-level knob the hardware has lives in the RX path.
- **The nearest 11ac concept, non-contiguous 80+80, is absent too** — none of
  the supported USB parts implement it (register-level plumbing for it exists
  only on later silicon).

What the silicon *does* offer composes into a three-part approximation:
per-tone **detection** (find the dirty slice), an RX-side **tone mask**
(de-weight it in the equalizer), and TX-side **avoidance** (place a narrower
transmission on the clean sub-channels). All three are now in devourer; the
measurements below say which ones carry their weight.

## The RX tone mask (`DEVOURER_RX_CSI_MASK`, `DEVOURER_RX_NBI`)

All three generations carry two receive-path tone-level mechanisms, ported
from vendor phydm into `src/ToneMask.h` and exposed on both demos:

- **CSI mask** — a per-tone receive-equalizer de-weight table: the BB treats
  masked subcarriers' channel estimates as garbage. On 11ac silicon
  (Jaguar-1: 8812/8811/8821/8814; Jaguar-2: 8822B) it is a plain bit array —
  enable `0x874[0]`, positive tones `0x880-0x88F`, negative `0x890-0x89F`,
  one bit per 312.5 kHz subcarrier. On Jaguar-3 (8822C/E) it is an indexed
  table behind `0x1D94` (bracketed by `0x1EE8[1:0]`, two 4-bit entries per
  byte: enable bit + 3-bit weight), enable `0xC0C[3]`. The vendor wrapper
  only ever masks a handful of tones around one interferer frequency;
  devourer's `DEVOURER_RX_CSI_MASK=<f_lo>-<f_hi>[/wgt]` (MHz) loops the
  low-level setter across an arbitrary range, so one env var masks a whole
  20 MHz slice. On Jaguar-3 note the sharing trap: adjacent tones share a
  table byte, so a range must be accumulated host-side before writing —
  the vendor's one-tone-at-a-time sequence would clobber neighbours.
- **NBI notch** — a single narrowband notch filter, LUT-quantized
  (`0x87C[19:14]` + enable `0x87C[13]` on 11ac, with `0xC20[28]`/`0xE20[28]`
  added on 8822B; `0x1944/0x4044[20:12]` per path + `0x818[3]` (inverted
  polarity on 8822C/E), `0x818[11]`, `0x1D3C[30:27]` on Jaguar-3).
  `DEVOURER_RX_NBI=<f_mhz>` — vendor-parity, one interferer.

Both knobs are applied when the RX loop starts, after the channel set (same
contract as `DEVOURER_RX_PATHS`: the final word for a single-channel capture;
a channel switch reverts them). The tone math (`phydm_find_fc` port +
subcarrier enumeration) is unit-tested headlessly (`ctest -R tone_mask_math`);
the register encodings are asserted on hardware by
`tests/tone_mask_regcheck.sh` — validated on 8821AU, 8814AU, 8822BU, 8812CU
and 8822EU, including an 80 MHz cell (masking 5230-5250 of the ch36 block
sets exactly tones +64..+128).

The mask is provably *active*, not just a register that reads back:
`tests/tone_mask_rx_sanity.sh` shows an off-frequency mask leaves a live link
untouched (29 → 30 hits) while masking the tuned channel collapses it
(29 → 6).

## What the jammed-slice experiment measured

`tests/pseudo_puncture_onair.sh`: 8822BU transmits VHT80 beacons on the ch36
block (fc 5210) to an 8814AU receiver at 80 MHz, while a USRP B210 jams
exactly one 20 MHz slice (5230-5250) with band-limited AWGN
(`tests/sdr_interferer.py` — the TX gain is the reproducible damage knob).
Hits per 15 s cell:

| cell                                   | hits | vs clean |
|----------------------------------------|------|----------|
| A — clean VHT80                        | 61   | 100%     |
| B — jammed VHT80                       | 31   | 50%      |
| C — jammed VHT80 + CSI mask on slice   | 33   | 54%      |
| D — jammed, TX drops to clean-half 40  | 48   | 78%      |

**The CSI mask is inert against a jammed slice** (+2 hits ≈ run-to-run
noise; a higher jam gain repeats the same picture), and the mechanism behind
that is the central finding: re-running the jammed cell with
`DEVOURER_RX_KEEP_CORRUPTED=1` delivers **zero FCS-failed frames** — the
missing 50% never reach the FCS stage at all. The jam kills wide frames at
the detection/AGC/sync level, *upstream* of the equalizer the CSI mask
programs, and equally upstream of the sub-block salvage layer
(`docs/fused-fec.md`), which needs a delivered-but-corrupt body to work on.
The mask remains the right tool for what the vendor built it for — a narrow
in-band interferer riding on top of decodable frames — but it cannot emulate
punctured *reception* of a wide PPDU.

**TX-side avoidance is the lever that works.** Dropping the transmission to
the clean 40 MHz half recovers most of the delivery (78% of clean-80). It is
also cheap and per-packet-capable: bandwidth and rate ride in the radiotap
header, the primary-channel offset in `SelectedChannel.ChannelOffset`, and an
intra-band retune costs ~1.5 ms (`docs/frequency-hopping.md`). Wi-Fi 7 keeps
the punctured channel's full clean tones where this fallback pays the
next-narrower-bandwidth quantization (80 → 40 loses the clean ch44 slice
too) — that gap, [clean][clean][dirty][clean] → 60 MHz usable vs 40 MHz
usable, is the real cost of not having puncturing signaling.

**And it is unilateral, at zero receiver cost.** A receiver tuned at 80 MHz
decodes narrower frames sent on its primary sub-channels *without retuning*
(`tests/rx80_narrow_tx_probe.sh`), and pays no measurable sensitivity price
for staying wide: the differential delivery-vs-noise sweep puts the wide-RX
penalty at 0 dB (≤ 1 dB, bounded by the capture-cliff width —
`tests/wide_rx_penalty_sweep.sh`). So the RX parks at 80 MHz and the TX
shrinks or grows per-packet along the **primary-nested ladder**
`20 ⊂ 40 ⊂ 80` with no coordination — the one geometric constraint being
that every rung must contain the RX's primary 20 MHz (the upper-40 is
unreachable without a coordinated primary move). The adaptive-link
controller consumes exactly this: bandwidth as an energy-ranked op-table
dimension (`ControllerConfig.bw_set`), a shared seq-derived probe schedule
that senses per-rung delivery with zero extra wire fields
(`rc_proto.probe_bw`, `score.RungWindow`), and a `primary_dirty` escalation
for the one case no unilateral bandwidth choice can fix.

## Finding the dirty slice

Two detectors measured (`tests/pseudo_puncture_detect.sh`):

- **Per-tone SNR self-sounding** (`sound` mode) — the MU-report machinery
  from `docs/beamforming-self-sounding.md`, sounding each 20 MHz slice in
  turn. Measured caveat: the report describes the channel **at the
  beamformee**. On a bench where the sounding pair sits point-blank (NDP SNR
  ~50 dB) the jam neither suppresses the report count nor moves the per-tone
  SNR beyond its ±5 dB wobble — the detector does not discriminate the
  jammed slice. Use it when the sounding peer *is* the far end of the link
  whose spectrum you care about.
- **Victim-link probe scan** (`probe` mode) — hop the actual TX/RX pair
  across the four 20 MHz slices and compare delivery. Picks the jammed slice
  correctly (29 hits on ch48 vs 35-37 elsewhere). Crude, but it senses the
  only channel that matters — the victim's own — and needs no extra
  hardware. `FastRetune` makes the scan cheap enough to repeat.

## Verdict

| mechanism                            | works? | notes                                    |
|--------------------------------------|--------|------------------------------------------|
| TX tone nulling / true puncturing    | no     | no silicon hook, no VHT signaling         |
| RX CSI mask against a jammed slice   | no     | loss is pre-FCS (sync/AGC), mask is later |
| RX CSI mask against in-band spurs    | yes    | the vendor's original use; knob now open  |
| Sub-block salvage of jammed frames   | no     | nothing delivered to salvage              |
| TX avoidance (narrower, clean half)  | yes    | 78% of clean delivery; per-packet capable |
| Detection: self-sounding per-tone SNR| partly | senses the peer's channel, not the victim's |
| Detection: victim-link probe scan    | yes    | picks the jammed slice                    |

So the honest Jaguar rendition of "[clean][clean][skip][clean]" is:
**probe-scan the slices, then transmit around the dirty one at the widest
clean bandwidth, re-scanning on a cadence** — puncturing by primary-channel
selection, at the cost of the bandwidth quantization Wi-Fi 7 avoids.
