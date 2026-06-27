# Fused FEC for the long-range video link

This document describes the **fused forward-error-correction (FEC)** stack: a
cross-layer error-correction architecture for the OpenIPC / wifibroadcast-style
one-way video downlink, and the tooling that implements and measures it.

## The problem

A long-range Wi-Fi video link has three error-correction resources that, by
default, never cooperate:

1. **PHY-layer FEC** — the convolutional (BCC) or LDPC inner code baked into
   every 802.11 MCS. Its code rate is fixed by the MCS choice. devourer already
   selects it per packet from the radiotap header / `TxMode`
   (`src/RadiotapBuilder.cpp`, `src/RtlJaguarDevice.cpp`), and the SVC ladder
   (`txdemo/svc_tx_demo/svc_tx.h`) already picks a robust MCS for important
   video layers.
2. **Application-layer FEC** — a packet-erasure code spread across radio frames
   (Reed-Solomon / RaptorQ / RLC). It recovers whole frames that are *lost*.
3. **Corrupted-frame retention** — `DEVOURER_RX_KEEP_CORRUPTED` keeps frames
   that fail the 802.11 FCS instead of dropping them
   (`src/RadioManagementModule.cpp` sets RCR `ACRC32|AICV`; `demo/main.cpp`
   surfaces the body with `crc_err`/`icv_err` + per-frame RSSI/EVM/SNR).

**The gap:** a frame that fails the FCS is normally dropped wholesale, so the
outer code treats it as a *whole-frame erasure* and throws away the
mostly-correct bytes. Real 802.11 corruption is usually localized — a frame that
fails the FCS is mostly correct ("All Bits Are Not Equal", INFOCOM 2009). The
fused FEC converts unknown-position byte errors into known-position erasures at
sub-frame granularity, so the outer code recovers from far fewer erasures. This
is befinitiv's unbuilt wifibroadcast proposal, realized.

## Architecture — three concatenated codes

```
 IP / NAL data ─▶ OUTER erasure code ─▶ symbols ─▶ SUB-BLOCK INTEGRITY ─▶ radio body ─▶ PHY-MCS FEC ─▶ air
                  (RaptorQ / RLC / RS)             (CRC per sub-block)                  (BCC / LDPC)
```

The keystone is the middle layer. The outer erasure decoders already recover
from *per-symbol* erasures; today one symbol maps to one radio body. The
sub-block-integrity (SBI) layer instead packs several **fixed-size, individually
CRC-guarded** sub-blocks into one radio body, with one sub-block == one
outer-code symbol. Then:

- a clean frame delivers all its sub-blocks as good symbols (as before);
- a CRC-failed frame (kept via `KEEP_CORRUPTED`) delivers its *surviving*
  sub-blocks and erases only the corrupted ones;
- the outer decoder is **unchanged** — a dropped sub-block is just a symbol that
  didn't arrive, which is exactly what an erasure code recovers.

Sub-blocks are fixed-size and **not** self-delimiting on purpose: a corrupted
length byte must never desync the other sub-blocks in the same body. The
receiver partitions the body at offsets it computes from its own configured
sub-block size, never trusting the (corruptible) header.

### Where the gain comes from — and its limit

SBI salvage only helps when corruption is **localized within a frame**. This is
the central empirical result of the project:

- The **Realtek chip RX** (8821) uses a soft-decision *hardware* decoder, which
  on a marginal frame leaves a few localized residual byte errors. SBI salvages
  the rest. **On-air gain measured: +129 RS blocks recovered in a 15 s run that
  the drop-whole-frame baseline lost** (chip↔chip, see Results).
- An **SDR receiver running a hard-decision Viterbi** (the gr-ieee802-11 fork),
  on a marginal frame, *diverges* and produces frame-wide errors — most
  sub-blocks bad. SBI then has little to salvage. The over-air pipeline works
  end-to-end, but the gain is small until the SDR decoder is made
  soft-decision (see "Future work: soft-decision SDR").

## Outer codes

`tools/precoder/stream_fec.py` is a thin dispatcher selecting one of three outer
schemes by `FecConfig.scheme`, each in its own module with a MAGIC for wire
dispatch:

| scheme | module | MAGIC | character |
|--------|--------|-------|-----------|
| `raptorq` | `stream_fec_raptorq.py` | `0xF52E` | RFC 6330 fountain block code; high throughput, high latency floor |
| `rlc` | `stream_fec_rlc.py` | `0xF534` | RFC 8681 sliding-window; low latency (wraps Inria swif-codec) |
| `rs` | `stream_fec_rs.py` | `0xF540` | **MDS Reed-Solomon over GF(2⁸)**, systematic Vandermonde (Rizzo/zfec construction, poly 0x11d), matches wfb-ng's `k=8/n=12` |

Reed-Solomon is the best fit for the small, low-latency blocks a video downlink
wants: it is MDS (any *k* of *n* symbols reconstruct, zero overhead), exactly
where RaptorQ's probabilistic overhead is worst.

## Per-SVC-layer FEC-rate UEP

`tools/precoder/svc_uep_fec.py` is the application-FEC half of cross-layer
unequal error protection (Abdel-Khalek & Heath, JSAC 2012). The PHY-MCS half
already lives in C++ (`txdemo/svc_tx_demo/svc_tx.h`, `DEVOURER_SVC_LADDER`).
This module maps each HEVC temporal layer to its own RS redundancy — heavy on
base/IDR, light on enhancement — routed by an SBI `stream_id`:

```
critical (IDR / VPS/SPS/PPS) : RS overhead 1.00  (N=16, tolerates 8/16 loss)
T0 base                      : RS overhead 0.75  (N=14)
T1                           : RS overhead 0.50  (N=12)
T2                           : RS overhead 0.25  (N=10)
```

Verified graceful-degradation staircase at 30 % loss: blocks decoded
critical 17 / T0 16 / T1 12 / T2 7 (of 20). Together with `svc_tx.h`'s MCS
ladder, base/IDR layers get the most robust MCS **and** the heaviest FEC.

## Two receive scenarios, one shared framing

The SBI framing + outer code are identical for both receivers. Only the erasure
decision differs:

1. **chip↔chip** (`fused_fec_link.py`, `fused_fec_tx.py`, `fused_fec_rx.py`):
   devourer 8812 TX → devourer 8821 RX. The chip gives only hard corrupted
   bytes; localization is the per-sub-block CRC. `fused_fec_rx` runs a baseline
   and an SBI decoder in lockstep and reports the gain.
2. **SDR-RX** (`~/git/sdr2wifi/fused_fec_rung3.py`): devourer 8812 TX →
   USRP B210 RX via the `~/git/gr-ieee802-11` fork. Same SBI framing, same
   `FusedFecReceiver`. The fork surfaces FCS-failed frames via `GR_KEEP_CORRUPTED`
   (mirror of devourer's env). The SDR can, in principle, also use per-bit soft
   information — see Future work.

## Module map (`tools/precoder/`)

| file | role |
|------|------|
| `fec_subblock.py` | the SBI layer — pack/unpack fixed-size CRC-guarded sub-blocks; `stream_id` multiplexing; numpy-free so it imports into the GNU Radio env |
| `stream_fec_rs.py` | Reed-Solomon outer scheme (GF(2⁸), systematic Vandermonde) |
| `stream_fec.py` | dispatcher (adds the `rs` scheme + `FEC_MAGIC_RS`) |
| `svc_uep_fec.py` | per-SVC-layer FEC-rate UEP (HEVC NAL → layer → RS config) |
| `fused_fec_link.py` | chip-path `FusedFecSender` / `FusedFecReceiver` (baseline-vs-SBI) |
| `fused_fec_tx.py` / `fused_fec_rx.py` | chip-path CLIs (bytes ↔ `StreamTxDemo` / `<devourer-stream>`) |
| `fec_fusion_sim.py` | offline simulation: quantify SBI gain, size sub-blocks, no hardware |
| `test_*.py` | unit tests for each module (130 in the suite) |

## Running it

### Offline simulation (no hardware)

```sh
cd tools/precoder
uv run python fec_fusion_sim.py --scheme rs --model slope --ber 3e-4 --sweep
```

Quantifies the SBI gain under uniform/slope BER + frame-loss and sweeps the
sub-block size. In the post-PHY-FEC residual-BER regime (small symbols, BER
3e-5..3e-4) SBI lifts message success from 23–73 % → 97–100 %; the knee is
≈32 B sub-blocks (≈6 % framing overhead).

### chip↔chip on-air

```sh
sudo bash tests/fused_fec_onair.sh        # SKIP_RAIL=1 after a fresh boot
```

8812 TX at a fragile high MCS (the robust BPSK preamble/SIG keep the frame
detected while the 64-QAM data fails the FCS → corrupt-but-received) → 8821 RX
with `KEEP_CORRUPTED`. Reports `received / corrupt / FUSED-FEC GAIN`. See
[Reproducible corruption](#reproducible-corruption) for the recipe details.

### SDR-RX over real air (rung-3)

In `~/git/sdr2wifi` (needs the gr-ieee802-11 fork + GNU Radio 3.10):

```sh
sudo bash run_fused_rung3.sh    # 8812 TX HT MCS7 → B210 RX → SBI salvage
```

## Results

- **chip↔chip, real silicon, 15 s, MCS7, ch6:** 4086 frames, 1072 corrupt
  surfaced, 6013 sub-blocks salvaged, **FUSED-FEC GAIN = 129 RS blocks** the
  baseline lost (≈13 % lift in delivered blocks).
- **SDR-RX, over real air, HT MCS7:** the pipeline runs end-to-end — 4044
  devourer HT frames decoded over the air, corrupt frames surfaced, SBI bridge
  runs. The gain is small because the fork's **hard-decision** Viterbi produces
  frame-wide corruption (see below), not because of a code defect.

## Reproducible corruption

To exercise the salvage path reproducibly on a bench without an attenuator
(fixed antennas, adapters inches apart):

- **Use normal TX power.** Lowering chip TX power kills *detection* (preamble
  too weak → zero frames received), it does not make frames partially corrupt.
  Power controls reception; MCS (and an interferer) control data corruption.
- **A high MCS corrupts naturally.** MCS7's fragile 64-QAM data fails the FCS on
  ~13–26 % of frames at normal power while the robust preamble keeps them
  detected — no interferer required.
- **`tests/sdr_interferer.py`** turns a USRP B210 into a calibrated co-channel
  AWGN/CW source. A given `--tx-gain` reproduces the same interference (fixed
  RNG seed), so the FCS-failure rate becomes a function of one number — the
  reproducible knob when the natural rate is too low.
- **`DEVOURER_TX_PWR_OVERRIDE`** (StreamTxDemo) forces an absolute per-rate
  TXAGC index 0–63 for fine control.

Note: a `uhubctl` power-cycle of a **root-hub** port (e.g. the 8812 on root hub
9) drops the device but does not re-enumerate it on power restore — it wedges
until a full host reboot. `tests/fused_fec_onair.sh` supports `SKIP_RAIL=1` to
use the current rail as-is after a clean boot.

## SDR receiver changes (sister repos)

The SDR-RX path required two changes to the `~/git/gr-ieee802-11` fork, both
mirroring devourer's `KEEP_CORRUPTED`:

- **`lib/decode_mac.cc`** — `GR_KEEP_CORRUPTED` surfaces FCS-failed *legacy*
  PSDUs tagged `crc_ok=#f` instead of dropping them.
- **`lib/frame_equalizer_impl.cc`** — HT/VHT/MIMO frames are decoded here (not
  in `decode_mac`) and previously only printed their CRC result. A new `pdu`
  message port publishes them (with `crc_ok` + `GR_KEEP_CORRUPTED`); the hier
  block forwards it to `mac_out`.

`~/git/sdr2wifi` holds the over-air receiver (`fused_fec_rung3.py`), the
software-loopback capstone (`fused_fec_rung1.py`), and validation tools
(`keep_corrupted_check.py`, `ht_hier_check.py`).

## Future work: soft-decision SDR

The SDR over-air gain is gated by the fork's **hard-decision** Viterbi: on a
marginal frame it diverges and corrupts the whole frame, leaving SBI nothing to
salvage. The fix is a **soft-decision** receive path — a soft demapper
(per-coded-bit LLRs for BPSK/QPSK/16/64-QAM) feeding a soft-input Viterbi —
so marginal frames carry a few *localized* residual errors instead of
frame-wide garbage, restoring the structure SBI exploits. The fork already has a
soft min-sum LDPC decoder, but only for R=1/2 n=648 (the robust regime that does
not corrupt over air); the fragile BCC rates (MCS1-7) are where the soft path is
needed.

## References

- Jamieson & Balakrishnan, "PPR: Partial Packet Recovery for Wireless Networks", SIGCOMM 2007.
- Han et al., "All Bits Are Not Equal — A Study of IEEE 802.11 Communication Bit Errors", INFOCOM 2009.
- Han et al., "Maranello: Practical Partial Packet Recovery for 802.11", NSDI 2010.
- Abdel-Khalek & Heath, "A Cross-Layer Design for Perceptual Optimization of H.264/SVC with UEP", JSAC 2012.
- Shokrollahi, "Raptor Codes", IEEE Trans. IT 2006 (RaptorQ: RFC 6330). Roca et al., RLC: RFC 8681.
- Rizzo, "Effective Erasure Codes for Reliable Computer Communication Protocols", 1997 (the GF(2⁸) Vandermonde RS).
