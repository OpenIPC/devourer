# RTL8814AU port audit — devourer vs aircrack-ng/rtl8814au

Systematic source-level comparison of devourer against its kernel reference
(`aircrack-ng/rtl8814au` @ `8926414`, `CONFIG_RTL8814A=y`, USB, BT/WoWLAN off,
module-param defaults), motivated by 8814 TX being silent on-air and by the
suspicion that the port carries mistakes. Ten work packages covered every
devourer 8814 code path; each finding below was verified against both sources
at the cited lines before classification. Several were found independently by
two blind audit passes.

Classes: **C1** unambiguous porting error (fixed) · **C2** intentional
adaptation (kept, documented) · **C3** kernel feature not ported (risk-assessed)
· **C4** ambiguous (documented experiment, no speculative fix).

Method: static function-pair diff with kernel `#ifdef`s resolved per the real
build config, plus table-parity regeneration. usbmon/canary lenses and on-air
validation are listed under *Hardware validation* — **none of the fixes below
have been hardware-validated yet**.

## C1 fixes (27 commits, each independently buildable)

Ranked by suspected relevance to the motivating "USB accepts, 0 frames on air"
symptom.

| # | Commit | Finding | Kernel ref |
|---|---|---|---|
| 1 | `c5bb4ad` | 8812 `_InitBurstPktLen` body ran on 8814: its `0x456=0x70` (AMPDU_MAX_TIME on 8812) clobbered `REG_TXPKTBUF_BCNQ1_BDNY_8814A` — the TX-buffer boundary programmed to 0x7F6 moments earlier. Ported the 8814 body (FAST_EDCA, RXDMA burst mode, `0xf002=0` "avoid usb 3.0 H2C fail", LDPC-pre-TX off); removed PIFS-zeroing, USTIME, MAX_AGGR 0x1f1f, RSV_CTRL/ARFR/0xf050/0x288/0x289 8812-isms | `usb_halinit.c:122-166` |
| 2 | `f3188ea` | The 5GHz CCK→OFDM TX clamp gated on `_channel.Channel` — a member **never assigned anywhere** (read of indeterminate memory; the 5G-TX fix fired nondeterministically per build/run). Channel now stored in `SetMonitorChannel`; member value-initialised | `core/rtw_mlme_ext.c:1058` (kernel keys off maintained `cur_channel`) |
| 3 | `2931ad6` | FW-boot poll was vacuous: accepted `byte0==0x78` of REG_MCUFWDL — which devourer itself writes in the 0x6078 kick — so a never-booted 3081 looked like success. Now requires chip-set `CPU_DL_READY` (BIT15) and errors loudly on timeout. **A failure here on a virgin chip is the smoking gun for the TX silence** | `rtl8814a_hal_init.c:649-656` |
| 4 | `dbeb720` | 8814 RF reads used the 8812 3-wire serial mechanism; the kernel reads via per-path direct BB shadow blocks (`0x2800/0x2C00/0x3800/0x3C00 + reg*4`). Paths C/D returned garbage from the serial path, so every masked RF RMW (channel + BW writes of RF 0x18) corrupted C/D tuning on **every channel set**. Overturns the old "C/D write-only by design" note | `rtl8814a_phycfg.c:86-122` |
| 5 | `1a8ea83` | Crystal-cap trim wrote 8812 bit positions on every chip — on 8814 the XTAL trim landed 4 bits high (real field untouched, `0x2C[30:27]` clobbered) ⇒ uncorrected carrier-frequency offset on TX+RX. Also fixed: 8821's distinct mask, the `(uint8_t)` truncation of the 12-bit pattern (hurt 8812 too), and the missing RCK1 A→B/C/D RC-trim sync | `phydm_cfotracking.c:230-249`, `rtl8814a_rf6052.c:143-146` |
| 6 | `993e0d9` | `USTIME_TSF/EDCA` forced to 0x50 (8812's 80MHz tick); 8814 MAC runs 100MHz, table value 0x64 — all µs-derived TX timing ran ~25% fast | `usb_halinit.c:577-579` (writes commented out) |
| 7 | `cffeba5` | `NAV_UPPER` left 0 after the init zero-write (kernel restores `ceil(30000/128)=0xEB`) — MAC honoured arbitrarily long NAV; can defer TX indefinitely on busy air | `rtl8814a_hal_init.c:3794-3807` |
| 8 | `8b45e23` | USB TX-agg block-descriptor config never armed (`TDECTRL[7:4]=3`, `0x20B=0x06` — kernel always does) | `usb_halinit.c:654-676` |
| 9 | `0cbeea5` | Per-byte aggregation limits `0x4CA/0x4CB=0x36` from `_InitMacConfigure_8814A` tail (devourer ported the old 8812 pair the kernel folded away) | `usb_halinit.c:487-554` |
| 10 | `e834ce5` | Bulk-IN host buffer 16KB vs the 20KB chip-side RX-aggregation threshold (kernel: 32KB × 8 async URBs). Oversize aggregates split with no short packet → tail parsed as a descriptor, remainder dropped. *Companion to #1, which introduced the kernel 20K threshold* | `rtl8814a_recv.h:25` |
| 11 | `f3b8c8c` | 5G EFUSE PG diff block parsed with the 2.4G two-bytes-per-Ntx shape — every field from relative byte 16 wrong-sourced (BW20/40 3S/4S ↔ OFDM bytes; BW80 diffs read BW160 nibbles). Found independently by two passes | `hal_com_phycfg.c:848-953` |
| 12 | `b3929ef` | Per-Ntx power diffs are **cumulative** upstream (MCS16-31 = `[0]+[1]+[2]`; VHT2SS+ adds `[1]`); devourer used exclusive windows, and 5G had no VHT clauses at all | `hal_com_phycfg.c:2490-2601` |
| 13 | `95d1b97` | Kernel 8814 RFE decision tree ported: unburnt/BIT7 `0xCA` → rfe_type **1** on AU (was 0 via 8812 parse), `&0x7F`, amplifier state derived from rfe_type. Removed the now-wrong `GetPhyContext` 0→1 patch. *Runtime-inert on the CF-938AC (0xCA=0x01 → 1 either way)* | `rtl8814a_hal_init.c:1474-1568` |
| 14 | `c9394a4` | 8812 TX-power-training writes ran on 8814 each channel set (kernel 8814 has none; paths B/C/D collapsed last-writer-wins onto 0xE54) | `rtl8814a_phycfg.c:636-673` |
| 15 | `559f39a` | IGI floor wrote paths A/B only — 4dB initial-gain imbalance vs C/D on the 4-path chip (RX/MRC skew) | phydm `ODM_IC_AC_4SS` DIG |
| 16 | `217fb76` | Power-seq poll retry 10ms vs kernel `udelay(10)` — failing poll cost ~50s instead of ~50ms | `HalPwrSeqCmd.c:134` |
| 17 | `be22303` | Deinit-before-init scaffold made faithful to `hal_carddisable_8814`: `REG_CR=0` stop-rx prologue, gated on the warm-boot detect (kernel never feeds `ACT_TO_CARDEMU` to a cold chip), kernel's constant `0xFE` cut mask | `usb_halinit.c:1386-1455` |
| 18 | `5dde6cc` | `0x577=0x03` secondary-CCA control port | `usb_halinit.c:1250` |
| 19 | `abe4988` | `USB_AGG_EN` (0x283 bit7) now explicitly cleared like the kernel RX-agg setup | `usb_halinit.c:705-727` |
| 20 | `9c38f80` | VHT radiotap bandwidth never reached the descriptor (MHz literals compared against `CHANNEL_WIDTH` enums) — every VHT frame TXed at 20MHz | `rtl8814a_xmit.c:443` |
| 21 | `e280b92` | rfe_type-2 5G pinmux constant `0x37173717` (copy slip; latent — rfe-2 boards only) | `PHY_SetRFEReg8814A` |
| 22 | `e20afaa` | `REG_USB_HRPWM` init write skipped on 8814 (kernel has it commented out) | `usb_halinit.c:1354` |
| 23 | `6021880` | fw version logged from double-offset header read (always 0; blob is v33, md5-identical to kernel's) | `rtl8814a_hal.h:76` |
| 24 | `10927e6` | RX parse hardening: PHY-status memcpy gated (payload bytes were decoded as RSSI/EVM/SNR on physt-less frames + tail over-read), short-fragment guard, 8814 DWORD4 trap documented, spurious log demoted | `usb_ops_linux.c:179` |
| 25 | `3c958ce` | fwdl comment register labels corrected (0x0230 is FIFOPAGE_INFO_1/HPQ count, 0x0210 TXDMA_STATUS, …) — debug-risk only | `rtl8814a_spec.h` |
| 26 | `3597aef` | Hygiene: `0x670` labeled REG_CAMCMD clear-all (was "NAV-related"); dead BIT0 LLT port removed with the adjudication recorded | — |
| 27 | `334d04f` | Docs: stale `DEVOURER_FORCE_TXPWR` removed (only `DEVOURER_SKIP_TXPWR` exists) | — |

## Key negative results (settled — don't re-chase)

- **The kernel sends ZERO H2C commands in monitor bring-up/channel-set/inject**
  under this config (exhaustive caller-traced inventory; `RegFWOffload` is
  never assigned). Devourer's zero-H2C is *not* a divergence; missing H2C
  cannot explain "kernel TX works, devourer silent". The surviving FW axes are
  boot verification (#3, now instrumented) and the rtw88-trace download
  protocol itself.
- **IQK-off in monitor is parity**: the kernel's init IQK block is commented
  out, the channel-set trigger requires `bNeedIQK` (join/AP/DFS only), and the
  watchdog IQK is `#if 0`.
- **PHY tables are byte-identical** (vendored inputs match the kernel tree;
  generated arrays in parity) and the table walker + `check_positive` are
  semantically exact — including both drivers ignoring cond2/3/4 (GLNA/GPA/
  ALNA/APA never select 8814 table branches in either tree).
- **PWR_SEQ arrays + parser semantics** are byte-identical/faithful. The
  historical "cut-mask filter broke fwdl" mystery has a mechanism: the kernel
  passes the chip-independent **constant** `~PWR_CUT_TESTCHIP_MSK` (0xFE) —
  deriving the mask from EEPROM cut_version (cut ≥ H overflows past BIT7 →
  mask 0) silently no-ops the whole flow.
- **Auto-LLT trigger is BIT16** of 0x208 (the only structured in-tree
  definition; HW-verified 2ms self-clear). The vendor BIT0 function polls a
  stale variable and verifies nothing. Invariant: FIFOPAGE_INFO/RQPN latch
  immediately precedes the BIT16 trigger, both after fwdl (devourer's
  rsvd-page fwdl transport requires post-fwdl LLT, unlike the kernel's DDMA
  position).
- **TX descriptor field map**: every field devourer sets has identical
  (dword, bit, width) in the `_8812` and `_8814A` macro sets; checksum domain
  (16 LE16 words, dwords 8-9 excluded) exact; QSEL/MACID/EP mapping exact.
- Kernel `ip link down` does **not** HW-deinit (call commented out, IPS off);
  virsh surprise-removal also skips card-disable — only rmmod/clean-unbind
  powers the chip down. Devourer has no teardown at all (C3): the env-gated
  deinit-before-init scaffold is the mitigation.

## Residual C3 risks (not ported; assessed)

| Area | Risk |
|---|---|
| Thermal power-tracking + LCK (8814) — kernel watchdog compensates TX AGC/BB-swing per thermal delta with **no link gating** | TX power drifts as the chip heats during sustained injection (long-range video duty cycles); LCK never re-runs. Highest-value C3 to port next |
| DIG / CCK-PD / EDCCA / FA-reset watchdog slices | IGI frozen at floor; CCK PD fixed; EDCCA thresholds stay at table defaults — RX adaptation in noisy environments (watchdog exists but is env-gated off for measured USB-contention reasons; ports only FA+DIG) |
| `phy_SpurCalibration_8814A` (skip premise was wrong) | ch153 NBI/CSI (any rfe) and ch140 swap (rfe 0) are live at 20MHz — RX spur masking missing on those channels |
| Vendor-request single-shot (kernel: 10× retry + io-error escalation) | One EP0 hiccup aborts a session (read) or silently skips a register write — credible #36 (passthrough-cycle) contributor |
| Sync 1×32KB polling read (kernel: 8×32KB async URBs) | IN-token gaps while parsing + 50ms error sleep → RX loss under load only |
| `Index5G_BW80_Base` (kernel averages adjacent BW40 groups) | 80MHz TXAGC base off by up to half the inter-group delta — 80MHz only |
| 0xC9 TRX-antenna option / USB2→2T4R policy | TX nss policy on USB2 ports / non-0xFF 0xC9 boards |
| TX-power validity check (wrong offset, throws on blank maps; kernel falls back to default tables) | Crash on blank-TX-power EFUSE boards |
| Skip-fwdl-if-running (`MCUFWDL==0x78`) has no kernel counterpart | Warm runs skip ALL fw arming. Entangled with the deinit experiment (scaffold makes the chip cold → skip not taken) |
| Bulk-boundary padding (kernel pads 8B via PKT_OFFSET when `(40+sz)%512==0`; devourer sends exact-multiple + ZLP) | Deterministic per frame size; a 472/984-byte payload sweep settles chip ZLP tolerance |
| EFUSE physical walk 512 vs 1024; EFUSE mask unapplied; 0x8129 ID-check (kernel dropped it) | Odd/heavily-reprogrammed boards only |

## C4 experiments (no speculative fixes made)

1. **HWSEQ for injected frames**: this kernel does `HWSEQ_EN=0` + copies the
   frame seqnum (`monitor_overwrite_seqnum=0`); devourer does `HWSEQ_EN=1`
   (matches the 88XXau byte-match instead — two GPL trees disagree, same story
   for `DATA_RETRY_LIMIT` 12-vs-0). HW currently overwrites injected seqnums —
   matters for wfb-ng-style consumers. Experiment: flip to kernel behaviour,
   verify on-air seq == injected seq, watch for TX regressions.
2. **HWSEQ_CTRL byte3**: devourer forces 0xFF via 32-bit RMW (source intent);
   the working kernel chip ends at 0x03 (its 8-bit write no-ops on silicon).
   Experiment: drop the force, byte-match the kernel chip state.
3. **0x283 bit7 reset value** — one vendor-read on live HW settles whether the
   new explicit clear ever mattered.
4. **RX-wedge falsification**: re-run 8814 RX under dense traffic with the
   32KB buffer (and optionally the old ≤12K threshold) to see whether the
   split-aggregate mechanism was the "~10 frames then bulk-IN timeout" wedge.
5. **rtw88-mimic ops with no vendor counterpart** (`0x0064/0x004C/0x00EC/
   0x1103/0x1330/0x01A0/0x0009` + `0x10C2` BIT5): resolvable only against the
   rtw88 source; candidates for byte-state diffing if the FW-boot check (#3)
   fails on a virgin chip.

## Coverage map

| Devourer 8814 path | Audited in |
|---|---|
| Init flow order + warm-boot detect (`HalModule::rtl8812au_hal_init`) | WP3 (45-item ordered checklist) |
| Power-on / fwdl mimic / FW vars / H2C (`FirmwareManager`) | WP2, WP7 |
| Queue/page/EP/boundary, MAC config, EDCA/retry/agg/beacon/burst | WP1 |
| Deinit scaffold + PWR_SEQ arrays + parser | WP4 |
| TX descriptor + radiotap mapping + USB xmit rules (`RtlJaguarDevice`, `FrameParser` TX) | WP5 |
| Channel/BW/band/RFE/TXAGC/power chain (`RadioManagementModule`) | WP6 |
| EFUSE machinery + derived config (`EepromManager`) | WP7 |
| PHY tables + walker (`PhyTableLoader`, `hal/phydm/rtl8814a/*`) | Lens D + WP8 |
| DM defaults, watchdog slices, IQK (`PhydmWatchdog`, `Iqk8814a`) | WP9 |
| RX descriptor/aggregation/filters + USB primitives (`FrameParser` RX, `RtlUsbAdapter`) | WP10 |

## Hardware validation (pending — user-assisted)

1. `cmake --build build -j` — done for every commit.
2. **Virgin-chip ch6 on-air check** (AR9271 sniffer, fresh Vbus power-cycle):
   watch the new log lines — `8814A firmware boot NOT confirmed` = FW-arming
   axis confirmed; frames on air = the BCNQ1/USTIME/NAV cluster was the gate.
3. Canary re-diff at ch6 + ch36 (extend set with 0x2C, 0x456, 0x55C/0x638,
   0x652, 0x577, 0x4CA/B, 0x208/0x20B, 0x283, RF 0x18 on C/D via the new
   direct read).
4. `sudo python3 tests/regress.py --vm-name devourer-testrig --vm-ssh …`
   default matrix; `--channel 36` spot-check; full matrix + encoding-matrix if
   8814 TX goes green.

## Resolution: FW-boot / TX-silence axis (2026-06-11)

The smoking gun fired exactly as predicted by finding #3: on a virgin chip the
`CPU_DL_READY` poll failed (`REG_MCUFWDL` stuck at `0x00606078`) and TX was
0 on-air. Replacing the rtw88-mimic fwdl *bracket* with a verbatim port of the
vendor kernel's `FirmwareDownload8814A` + `HalROMDownloadFWRSVDPage8814A`
(keeping the mimic's power-on prefix, which remains devourer's only 8814
power-on) makes the 3081 boot: trajectory `0x...2079` (per-section
DL_RDY/CHKSUM_OK RMW) → `0x...6079` (FW_DW_RDY) → `0x...E078`
(`CPU_DL_READY`, asserted instantly on `_3081Enable`). With the FW running,
TX emits on-air: ch6 12.8k / ch36 10.7k / ch100 10.8k canonical-SA frames at
the kernel-monitor witness; RX unaffected. The legacy sequence is preserved
bit-for-bit behind `DEVOURER_8814_FWDL=rtw88` and still reproduces the
failure — a clean causal A/B. Residual-risk items #1 (blanket `0x79/0x6078`
kick) and the FW-boot arm of #5 are thereby resolved.
