# RTL8822B firmware channel-switch offload (H2C 0x1D)

Isolation and benchmark of Realtek's firmware-offloaded channel switch on
the RTL8822BU — the primitive nominated by the
[kernel channel-switch baseline](kernel-channel-switch-baseline.md) —
against devourer's own `FastRetune` on the same silicon, channels and
on-air oracles. Harness: the kchansw bench (`tests/kchansw_bench.py
phase-b-vm --pairs ...` with `VM_MODPARAMS=rtw_ch_switch_offload=1`) plus
`tests/kchansw_fastretune_arm.py` for the devourer arm.

## The mechanism (source-isolated, vendor rtl88x2bu fork)

Two distinct firmware channel-switch families exist in the driver; the
8822B uses the second:

- **H2C 0x1C `CHNL_SWITCH_OPER_OFFLOAD`** — the TDLS-V1 path
  (`rtw_hal_ch_sw_oper_offload`, 4-byte payload: channel, bw, 40/80 MHz
  SC, RFE type; completion = C2H 0x10 `FW_CHNL_SWITCH_COMPLETE` →
  `rtw_tdls_chsw_oper_done`, 10 ms wait). Compiled only when
  `CONFIG_TDLS_CH_SW` **without** V2 — on the 8822B, `hal_ic_cfg.h`
  selects `CONFIG_TDLS_CH_SW_V2`, whose TDLS path is plain
  `set_channel_bwmode`. 0x1C is therefore dead code on this chip.
- **H2C 0x1D `SINGLE_CHANNELSWITCH_V2`** — the 8822B's real firmware
  switch (`rtw_hal_switch_chnl_and_set_bw_offload`, `hal/hal_com.c`).
  3-byte payload: `b0[7:0]` central channel, `b1[3:0]` primary-channel
  index, `b1[7:4]` bandwidth, `b2[0]` PWR_IDX_UPDATE_EN, `b2[1]`
  IQK_UPDATE_EN (the driver sets IQK_UPDATE only). Completion: C2H
  `CMD_ID_C2H_CUR_CHANNEL` → `rtw_sctx_done(hal->chsw_sctx)`; the host
  waits ≤ 10 ms. Wired into the *normal* set-channel path
  (`rtl8822b_switch_chnl_and_set_bw` → `switch_chnl_and_set_bw_by_fw`)
  behind `hal->ch_switch_offload` — the stock module parameter
  `rtw_ch_switch_offload` (default 0). No TDLS peer, association, or
  MAC-ID is involved; `iw set channel` on a monitor interface exercises
  it directly.

So the "research harness outside TDLS" the experiment called for ships in
the driver: `insmod 88x2bu_ohd.ko rtw_wifi_spec=1 rtw_ch_switch_offload=1`.

## Results (VM venue, kernel 5.15; oracles + dead-air on the host)

Dead air = first frame decoded on the destination minus last frame
decoded on the source (chip-clock fits, σ 28–330 µs). All rows RF-
evidenced; ≥ 10 warm-ups excluded.

| arm (36↔40, 5 GHz, 20 MHz) | n | med | p90 | p99 | max | host cost med |
|---|---|---|---|---|---|---|
| fw offload H2C 0x1D | 1000/1000 | **1.03 ms** | 2.43 | 4.33 | 1978.6 | 13.5 ms (nl80211) |
| driver path (by_drv, same module) | 998/1000 | 1.08 ms | 1.68 | 4.42 | 32.9 | 66.3 ms (nl80211) |
| devourer `FastRetune` | 1099 | 2.75 ms | 3.59 | 5.44 | **7.1** | 1.82 ms (register writes) |

| arm (36↔44, HT40) | n | med | p99 | gate |
|---|---|---|---|---|
| fw offload | 296/300 | 0.98 ms | 4.26 | advance |
| by_drv | 300/300 | 0.86 ms | 4.08 | advance |

| arm (cross-band / 2.4 GHz) | n | med | p99 | note |
|---|---|---|---|---|
| fw offload 36↔6 (per-switch band change) | 100/100 | **1.95 ms** | 11.4 | slow-slot; host-cmd only 15.6 ms |
| fw offload 1↔6 | 300/300 | 4.16 ms | 23.3 | decode-limited (see below) |
| by_drv 1↔6 | 300/300 | 4.53 ms | 27.2 | ditto; host-cmd 265 ms both arms |
| `FastRetune` 1↔6 | 299 | ≤ 20.7 ms | ≤ 45.2 | instrument-limited upper bound |

Reading:

- **The firmware executes the switch in ~2 ms and proves it**: the
  `kc_fw_ch_sw` kprobe pair (H2C submit → C2H CUR_CHANNEL) spans 1.94 ms
  median, matching the on-air 1.03 ms dead-air plus completion-report
  overhead. This is a genuine firmware execution with a hardware-side
  completion event, not a fire-and-forget.
- **Against `FastRetune`**: the offload roughly halves the median RF dark
  time (1.03 vs 2.75 ms) — the headroom the baseline predicted — and it
  handles **per-switch cross-band retunes in ~2 ms**, which `FastRetune`
  cannot (band changes fall back to the ~90 ms full path). `FastRetune`
  wins the tail: max 7.1 ms over 1099 switches versus three ~2 s
  whole-driver stalls in the fw run (below).
- **Direction asymmetry** (fw, 36↔40): 36→40 median 1.52 ms, 40→36
  0.43 ms — the upward retune pays PLL settle the downward one doesn't.
- **2.4 GHz cells are instrument-limited**: the ch-1 oracle decodes
  sparsely near-field (in-dwell decode gap p90 ≈ 9 ms), so every 2.4 GHz
  dead-air row is an upper bound. `FastRetune`'s own register-write time
  there is 1.26 ms — the 20.7 ms row is the oracle, not the radio.

## Failure classification (issue taxonomy)

- **C2H timeout**: 21/1010 completion waits exceeded the 10 ms sctx
  timeout. The driver logs and continues; all 21 switches still produced
  destination-channel RF and the session stayed consistent — the timeout
  path is benign here.
- **Whole-driver stalls**: 3/1010 switches (i=374/524/606) went dark
  ~2 s, `kc_fw_ch_sw` span 2.3–2.4 s — the H2C submit itself blocked on
  the USB layer (periodic driver housekeeping), not a firmware refusal.
  These dominate the fw arm's max and are the reason its tail loses to
  `FastRetune`.
- **Silent TX after interface restart** (fork bug, both paths equally):
  the first monitor session after insmod airs; any later `ip link down →
  set type monitor → up` cycle leaves TX accepted-but-silent until
  re-insmod. Isolated by cells: fresh-start runs all air (including
  per-switch cross-band); every post-restart config is silent regardless
  of `rtw_ch_switch_offload`. Benchmarks here therefore re-insmod per
  configuration.
- **Firmware feature absent / H2C rejected / wrong destination /
  automatic return**: none observed on this chip+firmware — 1696
  RF-evidenced fw switches across four configurations, every one decoded
  on the commanded channel, none returned to base autonomously.

## The devourer port (`DEVOURER_FASTRETUNE_FW`)

The offload is now a `FastRetune` fast path in the Jaguar2 HAL
(`HalJaguar2::fw_channel_switch`, 8822B only): the H2C rides the classic
HMEBOX mailboxes devourer already drives, and completion is
**fire-and-confirm-later** — polling RF18 during the switch measurably
*stretched* it (the PI reads contend with the firmware's RF-bus writes;
on-air dead time tripled), and the vendor's C2H wait needs an RX drain a
TX-only session doesn't have. Instead the next hop corroborates the
previous switch with one RF18 read (which doubles as the compose-cache
re-prime); a miss resyncs through the full path. `DEVOURER_FASTRETUNE_FW=1`
enables it for intra-band 20/40 MHz hops, `=2` also hands **cross-band**
hops to the firmware (band block reprogrammed by fw; active TX-power knobs
re-fold host-side).

Same rig, same oracles, same segmentation as the arms above (n≥399 each):

| devourer arm (dead air) | med | p90 | p99 | max |
|---|---|---|---|---|
| `FastRetune` software path, 36↔40 | 2.75 ms | 3.59 | 5.44 | 7.10 |
| fw path (`=1`), 36↔40 | **1.44 ms** | 2.72 | 5.05 | 9.05 |
| fw path (`=2`), 36↔6 cross-band | **1.86 ms** | 5.95 | 10.76 | 18.23 |

The fw path halves the median dark time and turns the cross-band hop from
a ~90 ms full-path event into ~2 ms — with none of the vendor driver's
2 s stalls (1499 devourer fw switches, max 18 ms), and sustained TX
across 400 consecutive band changes (no silent-TX). Hopping RX is
unaffected: identical decode counts over identical hop-RX runs with the
path on and off.

**Jaguar3** wires the same H2C in both vendor drivers
(`rtl8822c/e_phy.c`), and the port covers it through the same knob (the
H2C rides `HalJaguar3`'s HMEBOX counter, shared with the coex thread's
H2Cs under the same lock). On the 8822C (8812CU, same rig/oracles):
36↔40 fw 2.28 ms med / p99 5.94 versus sw 2.40 / 5.19 — a tie on air,
because the 8822C's RF settle dominates its dark time whichever engine
sequences it; the fw win there is the per-hop host cost (~0.6 ms H2C
submit vs ~1.9 ms of composed USB writes). Cross-band 36↔6 through the
firmware: **2.63 ms median** (n=379, p99 11.0) versus the ~90 ms full
path. Hopping-RX decode identical fw vs sw. The 8822E shares the code
path (not yet on-air-validated — no E-die on the rig this session); its
spur channels conservatively decline the fast path, fw included.

## Go/no-go for the series

**Go — and the port target is devourer, not the kernel.** The offload is
a 3-byte H2C plus a C2H completion on machinery devourer's Jaguar2 HAL
already has (H2C submit, C2H over the RX path). Adopting it as a
`FastRetune` fast-path would:

- cut the median hop dark time ~1 ms → ~2.6× (measured headroom, same
  silicon);
- add the ~2 ms cross-band hop `FastRetune` lacks;
- keep devourer's tail discipline (the 2 s stalls are vendor-driver USB
  housekeeping, absent from devourer's TX path).

Caveats to carry into the port: verify per-rate TX power after fw
switches (`PWR_IDX_UPDATE_EN` semantics), and the C2H completion must be
drained — TX-only sessions need `DEVOURER_TX_WITH_RX=thread` or a poll.
For experiment 3, MCC/FCS remains the scheduler-grade candidate (dwell
schedules, two contexts); 0x1D is the retune primitive underneath it.
