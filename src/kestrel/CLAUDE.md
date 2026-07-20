# src/kestrel/ — Kestrel (RTL8852, Wi-Fi 6) working context

Deep per-generation facts for this subtree, loaded alongside the root
CLAUDE.md. Chips: RTL8852BU/8832BU (variant C8852B) and RTL8852CU/8832CU
(C8852C), one HAL serving both dies on all platforms (MSVC included).

## Two-plane architecture

The **mac_ax plane** (power-on / FWDL / efuse / MAC TRX / H2C-C2H / USB /
descriptors) is hand-ported C++ from Realtek's "G6 phl" vendor trees
(`reference/rtl8852bu`, `reference/rtl8852cu` — none of the 11ac
loaders/parsers apply). The **BB/RF plane is Realtek's halbb/halrf C compiled
VERBATIM** for both chips: one chip-agnostic core + per-chip backends under
`hal/halbb/g6/` + `hal/halrf/g6/` (`vendor/` = untouched vendor files;
`shim/` + `hal_headers_le.h` = the minimal PHL surface they compile against;
`kestrel_halbb_glue.c` / `kestrel_halrf_glue.c` = the C entry points devourer
drives; `tools/vendor_hal{bb,rf}_*.sh` re-vendor).

The vendored loaders own the BB/RF/gain tables (`halbb_init_reg`,
`halrf_config_radio` — radio pages reach the fw via the bridge `send_h2c`
into the mac_ax H2C encoder), the per-channel BB config (`halbb_ctrl_bw_ch`),
the RF tune (`halrf_ctl_ch/bw` on the B, `halrf_ctl_band_ch_bw_8852c` on the
C), and the cals: DACK + RX-DCK both dies (wrapped in an RF 0x0/0x1/0x5
save/restore on the B — the vendor relies on later TRX flow to rewrite them),
IQK on the 8852C only. The RFK prologue (NCTL microcode / a-die SI reset /
LCK / RCK / efuse trim + TSSI DE) runs at `phy_bb_rf_init` in the vendor's
`halrf_dm_init` position — that position is load-bearing: run after the tune,
the SI reset deafens the radio both ways. **Evidence-gated cals**: TSSI/DPK
(both dies, via `support_ability`) and IQK on the 8852B degrade TX under the
fixed-dBm power model — gates fall with a TSSI-referenced power model.

## Dispatch and cold boot

Dispatched **PID-first** (`kestrel/KestrelUsbIds.h`), not from the 0x00FC
byte: on AX silicon that register is R_AX_SYS_CHIPINFO (die-id 0x51/0x52; the
8852A's 0x50 collides with the 8822B cold transient). Cold-boot facts (mac_ax
plane, both dies): a USB chip from real cold reads WLMAC_PWR_STE=MAC_ON (its
AFSM auto-powers to enumerate) and power_on must force the MAC off first
(`pwr.c` prologue) or the bootrom never raises H2C_PATH_RDY; `B_AX_ENBT` must
be set before any LTE-space access (arms the LTE indirect interface) and the
GNT arbitration is SW-forced to WiFi (`mac_cfg_gnt_8852b` stance) — a kernel
driver pre-initializing the chip masks all of this, so blacklist rtw89 when
testing.

USB open: Kestrel adapters skip the pre-claim `libusb_reset_device`
unconditionally (`claim_interface_then_reset` guard): `power_on` forces the
MAC off from any retained state, while a USB reset on running firmware drops
the chip to ROM — a stale-handle re-enumeration that can land in the dead
ZeroCD DISK id (`0bda:1a2b`).

## Validated capability surface

On-air-validated: monitor RX (both dies, 2.4/5 GHz; the RX front-end needs
the halbb per-band LNA/TIA gain-error cache, without which 5 GHz is deaf), TX
(mgmt injection — the OpenIPC video path via `streamtx`; legacy/HT/VHT/HE
rates), channel/BW **5/10/20/40/80 MHz on both dies + 160 MHz on the 8852C**
(the 8852B die has no 160 MHz — vendor bw_sup; caps report accordingly).
40 MHz tunes to the block center (primary ±2); 80/160 MHz derive
center/pri_ch from the channel plan (160 = an 8-wide block, center =
block_start+14). **6 GHz TX tops out at 80 MHz**: the 6G 160 MHz TX does not
radiate on the C8852C (the RF synth locks and RX-160 works, but the 6G+160
TX-enable path is un-ported — B210-confirmed 0% duty vs 45% at 6G-80 / 40% at
5G-160; a MAC TXAGC-max / RF-TX-path gap, not a chip limit — the vendor
drives it). 5/10 MHz narrowband is the BB "small BW" field with the RF left
in 20 MHz mode (no ADC re-clock, unlike Jaguar). RX bulk-IN delivery requires
the USB RXAGG engine enabled (`B_AX_RXAGG_EN`). TX airs with the CMAC
EDCCA/CCA gate disabled (`sch_tx_en`, TX path only) — the intended
injection/monitor-link mode. `ReadTsf` reads the per-port MAC TSF;
`StartBeacon` drives the AX HW beacon engine.

**HE ER SU + DCM extended range** (both dies): per-packet via radiotap-HE
FORMAT=EXT_SU or `DEVOURER_TX_RATE=.../ER[/DCM]`; RX classifies the format in
`RxAtrib.ppdu_type` (7=HE_SU, 8=HE_ERSU) — `docs/he-extended-range.md`.

Async packet-C2H (bulk-IN rpkt_type=10) delivery works — routed by
`handle_c2h` on the C2H class/func — so the #236 C2H surface (TWT/F2P
reports) is reachable. Not working: the fw's USR_TX_RPT TX-egress-timestamp
C2H (gated on a full BSS association, not just the registered NO_LINK role).

## FastRetune (frequency hopping)

`HalKestrel::fast_retune` is the lean same-band 20 MHz hop: RF channel set +
a per-sub-band gain re-apply + a BB reset + fixed TX power — skipping the BB
bandwidth config and the ~1.2 ms RX-DCK. **~9 ms/hop same-sub-band, ~13 ms
cross-sub-band** (vs ~2 ms on the 8822B, ~44 ms unoptimized). The channel set
is `fast_rf_channel_8852b`, a **compose-cache write-only** port of the vendored
`halrf_ctrl_ch_8852b`. Three cuts (all on-air + soak validated):

- **Compose-cache**: the vendored path reads RF18 ×4 (path A/B × DAV 0x18 / DDV
  0x10018) and RMW-toggles RF 0xcf ×8 every hop; the lean path primes those
  dwords once per epoch (`_kfr_*`, invalidated by every full `set_channel`) and
  writes whole dwords — ~12 reads gone.
- **No DDV writes**: the d-die 0x10018 window is unpopulated on the single-die
  8852B; skipping it costs no channel accuracy (soak-confirmed).
- **Relock only on a sub-band bucket change** (`fast_rf_channel_8852b(...,
  relock)`, `bucket_changed = gain_bucket != _last_gain_bucket`). A
  same-sub-band hop moves the synth little and it settles during the caller's
  admission window, so the ~13 ms path-A LCK poll is pure blocking — a plain
  RF18 write holds channel accuracy (soak: 2000 hops, zero wrong-channel,
  ~97 % delivery). The full `halrf_set_s0_arfc18` relock (RF 0xd3[8] hold +
  RF18 write + RF 0xb7[8] LCK poll) + `fast_lck_check_8852b` (RF 0xc5[15]
  verify + MMD-reset lock recovery) is kept for a sub-band crossing — a bigger
  VCO jump, and the only path with the recovery. NOTE: the relock guarantees a
  verified lock, so a `SetMonitorChannel` (full path) always runs it; the fast
  hop trades that guarantee for latency within a sub-band, empirically safe.

The synth-lock *diagnostic* reads are gated off the hot path
(`vnd_rf_tune(..., diag=false)`), and the gain-error re-applies only on a
bucket move. 8852C keeps the vendored `ctl_band_ch_bw` tune
(`fast_rf_channel_8852b` returns false). No firmware channel-switch H2C exists
here (H2C 0x1D is 11ac HalMAC; rtw89 has only `SCAN_OFFLOAD` + MCC). The
dwell-1 / N-channel data plane (`examples/dwelltx`) runs on Kestrel at ~30 ms
slots (soak: 2000 hops, zero wrong-channel, ~97 % delivery).

### Firmware IO-offload hop (`DEVOURER_KFR_OFLD`, default on)

`fast_retune_ofld_8852b` ships the *entire* same-sub-band hop — the six RF18/
0xcf channel-set writes, `halbb_bb_reset_all_8852b`, and the fixed-dBm TX-power
target — as ONE `FW_OFLD`/`CMD_OFLD_REG` H2C (mac_ax `fwofld.c`) the on-chip
firmware replays locally. `KestrelFw::reg_write_ofld` packs the 16-byte
`fwcmd_cmd_ofld` command buffer (src RF a-die / BB / MAC, `WRITE`/`DELAY`
types, LC on the last, `cmd_num` running); the firmware applies each masked
write as `(reg & ~mask) | ((val << ctz(mask)) & mask)` — so BB/MAC entries
carry the **raw** field value (pre-shifting double-shifts and mis-tunes),
while RF writes use the full `MASKRF` (shift 0). This collapses ~20 per-hop USB
register round-trips into a single bulk-OUT: host-side hop cost **~9.3 ms →
~0.15 ms (~45×)**, zero wrong-channel over a 6000-hop soak.

The offload does not speed up the RF synth — it only frees the host. The
direct path's ~5 ms of `bb_reset` USB round-trips incidentally covered the VCO
settle; collapsing them to ~0.15 ms exposes the true **~1.5 ms synth-settle
floor** (below it, a frame airs mid-retune → wrong-channel; ≥1.5 ms is clean,
`DEVOURER_DWELL_SETTLE_US` on the demo). Net time-to-usable-channel ~1.7 ms
(~5.5×), with the host free to prep the next frame / service RX during the
settle. The hop returns before the synth settles, so this changes the
`FastRetune` timing contract — the caller honours the settle (the demos'
admission window). Default on; `DEVOURER_KFR_OFLD=0` forces the self-pacing
direct path (for A/B or a suspect chip). A bucket crossing (verified synth
relock) and the 8852C fall through to the direct steps.

## TX power

A fixed BB dBm (`halbb_set_txpwr_dbm`, default 20 dBm, `DEVOURER_TX_PWR`
override — whole dBm on this family) with the runtime `SetTxPowerOffsetQdb`
lever. Per-packet radiotap `DBM_TX_POWER`: no WD-descriptor field exists —
`send_packet` honours the radiotap delta by rewriting the fixed-dBm BB target
between frames (2 BB RMWs on value change, free while constant; 0.25 dB
steps, clamped to the 0..23 dBm PA window around the `DEVOURER_TX_PWR` base;
global, so a HW beacon airing between frames follows the last-written level;
a field-less frame restores the `SetTxPowerOffsetQdb` session offset).

## 8852C vs 8852B divergences

Almost every 8852C divergence is a **`_V1` register-bank** move
(USB/HFC/RXAGG/HCI at 0x5xxx/0x1700/0x6000/0x7880 vs the 8852B
0x1xxx/0x8Axx/0x8900/0x8380) plus a different descriptor: FWDL/H2C use a
16-byte `rxd_short_t` (not the 24-byte WD body), data/mgmt TX uses the
32-byte `wd_body_t_v1` (not 24), and the RX-descriptor drv_info unit is
16 bytes (not 8 — `FrameParserKestrel.h`; with the B's unit the C "receives"
frames with correct lengths but unreadable bodies, i.e. looks deaf to any
SA-matching harness). The CBV-cut die loads the `u2_nic` fw image (CAV→u1).
Two 8852C-specific TX pieces the 8852B path lacks: the BB IFFT→TX-chain
routing (`halbb_ctrl_tx_path_tmac_8852c`, the 0xD800..0xD82C "path-com" block
the 8852B replaces with its per-STA CMAC antenna model), and the V1
descriptor's rate word — `USERATE_SEL`/BW/GI/DATARATE live in wd_body dword7,
not wd_info dword0 as on the 8852B (mis-placing it left `f_rate=0` so the
scheduler stalled and the bulk-OUT NAKed). Behavioural quirks beyond the
`_V1` bank moves (e.g. the frame-free NHM absolute floor is 2.4 GHz-only —
the firmware DIG biases the 5 GHz idle-noise measurement):
`docs/8852c-quirks.md`.

## Scope

The capstone is 11ax trigger-based UL + TWT (issue #236 — the v1.19 vendor
fwcmd surface exposes TWT-OFDMA + F2P trigger H2Cs that mainline rtw89
lacks). The 8852A-family (RTL8832AU) is deliberately excluded (frozen 2021
v1.15 vendor drop only).
