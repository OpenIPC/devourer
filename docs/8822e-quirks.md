# RTL8822E (RTL8812EU / RTL8822EU) — definitive quirks

New to the vocabulary used here (DMAC/CMAC, IQK, DIG, coex, efuse…)? The
[visual driver primer](driver-primer.md) defines all of it with animations.

The rtl8822e family (WiFi-only **RTL8812EU** `0bda:a81a`, BT-combo **RTL8822EU**
`0bda:e822`, chip-id `0x17`, Jaguar3) as devourer drives it today. Every entry
states what the chip needs, what devourer does about it, the residual cost if
any, and the reproducer that proves it. Reference hardware: LB-LINK
BL-M8812EU2 (`0bda:a81a`, rfe-type 21); the BT-combo part and other board
variants are outside the characterized set (see **Coverage**).

The methodology that closed most of these: vendor-kernel ground truth on the
same unit (`tests/eu_kernel_mcs_probe.sh`), full MAC+BB end-state diff
kernel-vs-devourer (`tests/eu_bb_endstate_diff.sh` — kernel via `/proc`
`read_reg` live, devourer via `DEVOURER_BB_DUMP`), then subset bisection with
the Jaguar3 golden-replay hook (`DEVOURER_REPLAY_WSEQ`).

## Front-end pin-mux is load-bearing (DPDT / PAPE / pads)

The chip's RFE control lines (DPDT antenna transfer switch, PA-enable, pad
routing) ride MAC GPIO/LED pin-mux registers that the halmac "Config PIN Mux"
family programs. Three of these must hold, or TX breaks in distinctive ways:

- **The DPDT transfer switch must be under HARDWARE TX/RX control — via the
  full eFEM GPIO pin-mux, not a static route.** The kernel's
  `_efem_pinmux_config` (rfe 21–24) claims GPIO2/3/4/13/14/EECS for the RFE
  engine (`RFE_CTRL_3/5/7/8/9/11`, halmac `pinmux_set_func_8822e`); GPIO13
  becomes `WL_DPDT_SEL` driven by RFE_CTRL_9, so the antenna transfer switch
  follows the TX/RX state — PA on TX, both LNAs on RX. Devourer ports this as
  `RtlJaguar3Device::efem_pinmux_8822e()` (post-coex; coex GPIO_MUXCFG writes
  would mask it). An earlier static approximation — set `0x4c[24]`
  (`BIT_DPDT_WLBT_SEL`), clear `[22]` — fixed TX (without WL DPDT ownership,
  every PPDU faster than ~26 Mbps PHY airs as garbage, and MCS0–3 leak
  through at a ~−30 dB EVM floor) but parked the switch TX-favoring: **RX
  path B lost its antenna on every 8812EU** — chain-B pwdb pinned at the
  noise floor (~10 raw / −99 dBm) regardless of antenna, invisible to
  total-frame-count validation because chain A carries the stream (bench
  2026-07-14, per-chain RSSI A/B: static write = TX 9.9M MCS7 / RX-B dead;
  `[24]`-only = RX-B alive / TX airs nothing; eFEM pin-mux = TX 9.7M MCS7
  AND RX A/B −70/−77 dBm concurrently). The eFEM pin-mux is gated on
  `rfe_type` 21–24 (kernel parity with `_efem_pinmux_config` / `config_rfe`);
  other boards keep the reset-default pins. Legacy behavior remains selectable
  via `DEVOURER_DPDT_MODE=efem|legacy|bit24|skip` (default `efem`) for board
  A/B. NB `0x204c` is a
  readable mirror of `0x4c` — end-state diffs surface it at either address.
- **OFDM 1SS TX path must be single-path** (`0x820[7:0]`: `0x32` = 1ss-B at
  5 GHz, `0x31` at 2.4 GHz; `0x1e2c=0x0400`): kernel semantics give 1SS frames
  ONE chain (2SS gets both). The old 1ss-on-both mapping (`0x33`) interacts
  with the DPDT fix — MCS0 TX stalls (bulk NAK wedge) and MCS7 delivery
  collapses. Path B is selected at 5 GHz on measured merit (MCS7 ~2.5× the
  delivery of 1ss-A at ~3 dB better EVM on this module); the 2.4 GHz choice
  follows the kernel default — on-air comparison is blocked by the 2.4 GHz TX
  quirk (see **2.4 GHz TX**).
- **`PAD_CTRL1 0x64[29:28]`**: halmac pre-init sets both; the bring-up's
  FW/coex steps clear bit 29 — re-asserted post-coex.

Reproducers: `tests/eu_mcs7_txagc_fix.sh` (parametrized TX-rate/power cell
bench, EU→CU), `tests/eu_bb_endstate_diff.sh`. Bench state: full ladder clean
at ch36 — MCS0/MCS4/MCS7/54M ≈ 5k frames / 12 s at EVM −26/−32/−48/−45.

## TX+RX mode and the path-B TXAGC reference (0x41e8)

Path-B OFDM TXAGC (`0x41e8`) is written unconditionally in every mode,
including TX+RX. (An earlier structural skip existed because any nonzero write
appeared to near-deafen the EU's RX. The register is exonerated — a direct A/B
with per-chain RSSI shows no RX effect from the write — but the historical
observation was pointing at something real: the static DPDT route disconnects
RX path B's antenna outright (see the pin-mux section above), and every
recheck counted TOTAL frames, which chain A dominates —
`tests/eu_41e8_desense_recheck.sh`'s −2% "noise" verdict and the 24k-frame
full-duplex proof (`tests/eu_fullduplex_pathb_check.sh`) were both blind to a
dead chain B. Per-chain RSSI is the only honest RX-health metric on a 2T2R
part.) Full-duplex holds under the eFEM pin-mux with path-B power applied.

`DEVOURER_TX_WITH_RX=thread` must still be set **before** `InitWrite`
(bring-up keeps the RX filters open; retrofitting RX later is unreliable).

## RF register 0x0 needs the legacy FON write port

RF reg `0x0` (mode register) writes through the direct `0x3c00`/`0x4c00` BB
window silently no-op — the kernel special-cases it through the FON port
`0x1808`/`0x4108` (`addr<<20 | data`). Devourer does the same in the halrf
code and the radio-table loader (the vendor tables carry reg-0 entries).
Reads stay direct-window for every register. Same rule on the 8822C.

## Channel-switch obligations (run at EVERY switch)

Ported straight from the kernel's `switch_channel` tail; skipping any of these
was a real failure on the bench:

- **IGI toggle** (`0x1d70` −0x202/restore): forces the BB to send the 3-wire
  command so the RF re-enters RX mode — the BB does not do this on its own
  after path/channel/BW changes. Symptoms when missing: intermittent
  post-switch RX deafness and a first-cell TX wedge (~50 frames then bulk
  NAK). Applies to both Jaguar3 variants. Liveness reproducer:
  `tests/cu_2g_ground_liveness.sh`.
- **Spur elimination**: per-channel NBI notch + CSI mask + packet-detection
  for the 14 channel/BW combos whose synthesizer harmonics land in-band
  (153/161/169@20, 151/159/167@40, 155/171@80, 54/102/118@40, 58/106/122@80),
  explicit spur-free default elsewhere. `FastRetune` declines hops into or
  out of these combos (the lean hop can't reprogram the notch state).
- **CCK TX shaping filter** (2.4 GHz, per channel; ch14 special set) + per-band
  TX backoff/scaling.
- 2.4 GHz RX needs the RF-write force-update brackets (`0x1830[29]`/
  `0x4130[29]`) around the RF18 tune — the BB-window write path does not push
  the analog front-end shadow on its own.

## Runtime obligations (the ~2 s watchdog cadence)

devourer runs the vendor watchdog's monitor-mode dynamic mechanisms
(`PhydmRuntimeJaguar3`, both Jaguar3 variants) — from the coex thread in
TX/TX+RX sessions, from a dedicated housekeeping thread in RX-only sessions
(register I/O must not run on the bulk-IN event thread):

- **FA/CCA window statistics** (read + reset per tick) feeding
- **DIG**: unlinked IGI stepping by false-alarm level (thresholds
  2000/4000/5000 per window; DFS channels pin `0x20`), written to `0x1d70`
  per path. The coverage window floors at `0x1e`, NOT phydm's generic
  `DIG_MIN_COVERAGE 0x1c`: at IGI `0x1c` the 8822CU's MCS4+/dense-QAM RX
  decodes nothing (0 of 65k kernel-injected MCS7 frames) while `0x1e` is
  transparent (65.9k/65k) — hardware-bisected, value-specific, the same
  class of per-IC "For HW setting" floor exception the kernel carries for
  other ICs.
- **CCK packet detection** (type4, 2.4 GHz): CCK-FA moving average drives the
  PD/CS level ladder in `0x1ac8/0x1acc/0x1ad0`.
- **EDCCA tracking**: `th_l2h = max(IGI+8, 48)`, `th_h2l = th_l2h − 8` into
  `0x84c` (matches the kernel end state). `SetCcaMode`'s carrier-sense-disable
  knob (primary CCA + EDCCA) suppresses this tracking.

The kernel's remaining watchdog mechanisms self-disable without a link and
are intentionally absent: CFO tracking (returns unless associated), rate
adaptation and the beamforming watchdog (no station entries), antenna
diversity (off on 2T2R).

## Coex / thermal (the same tick)

- **Coex re-apply + FW heartbeats**: without the WiFi-only coex re-apply the
  combo firmware silences the antenna during sustained 5 GHz TX (shared
  Jaguar3 behavior).
- **Thermal swing tracking**: RF `0x42[6:1]` meter → delta-swing table →
  `0x18a0/0x41a0[7:0]`. The chip runs THERMAL mode (the kernel's TSSI
  machinery is compiled in but its efuse mode-select is forced to thermal;
  `0x1e7c[30]`=0 on both drivers — the 2600-line TSSI subsystem is *not* a
  devourer gap).
- **LCK synthesizer re-lock**: when the averaged thermal drifts ≥4 units from
  the LCK baseline, re-run AACK+RTK (RF-A `0xca[0]` / `0xcc[18]` pulse+poll —
  the SYN lives on path A). Validated live: 22-minute max-duty soak
  (`tests/eu_heat_soak_lck.sh`) — 1.10 M clean MCS7 frames, re-lock fired at
  drift=4, EVM drift +3 dB.
- The FW power-mode/coex H2Cs **rewrite the OFDM TXAGC refs wholesale** during
  bring-up — devourer's authoritative TXAGC apply runs after them (the coex
  runtime ticks do not re-clobber; watchable via the coex-tick register trace
  under `DEVOURER_LOG_LEVEL=debug`).

## 2.4 GHz TX: undecodable on this module — kernel parity

The module airs 2.4 GHz TX energy (SDR duty ≈ expected) but **no receiver
decodes any of it** — CCK and OFDM alike, at any power, under **both**
devourer and the vendor kernel driver (`tests/eu_kernel_2g_verified.sh`: the
same verified ground decodes an 8812AU control at −63 EVM while 51k
kernel-injected EU frames yield zero). Treat 2.4 GHz TX as unusable on the
BL-M8812EU2 until a second board proves it module-specific vs family-wide.
2.4 GHz **RX** works. (The old README "8 Mbps @ ch6" figure was SDR
duty×rate — energy, not decodable throughput.)

## EFUSE / bring-up constraints

- **OTP/efuse is only reliably readable at init** (burst mode + sw-power-cut,
  one-shot); post-bring-up probes are refused by design.
- Efuse supplies: per-channel-group per-path TXAGC base refs (`0x22`/`0x4C`
  logical), rfe-type (`0xca`), thermal baselines (`0xd0`/`0xd1`).
- **DPK is force-bypassed** on rfe 21/22 — by the kernel too; both drivers run
  DPK-bypassed.
- **IQK clears the 40 MHz `TX_CCK_IND` RF bit** — calibration-order fix-up in
  the channel path.
- The TXAGC ref→power transfer is **not a constant 0.25 dB/step** on the E
  (accelerating ~0.3→0.9 dB/idx up the range; TSSI/kfree trims reshape it) —
  `GetTxPowerCaps().step_measured` stays false; controllers should calibrate
  their own dB-per-step or use ground RSSI.

## Coverage

Everything above is characterized on one board: the LB-LINK BL-M8812EU2
(WiFi-only RTL8812EU, rfe 21). Outside that set:

- **RTL8822EU (`0bda:e822`, BT combo)**: uncharacterized — including whether
  it needs more than the WiFi-only coex handling the CU-style combo parts get.
- **Other rfe types**: rfe 22–24 boards differ in RFE pin mapping and DPK
  policy; the 2.4 GHz TX verdict in particular is single-board evidence.
