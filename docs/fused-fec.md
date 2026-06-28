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

### Where the gain comes from

SBI salvage only helps when corruption is **localized within a frame**, so the
inner decoder's failure mode determines the gain:

- a **soft-decision** inner decoder, on a marginal frame, leaves a few
  *localized* residual byte errors; SBI salvages the surviving sub-blocks;
- a **hard-decision** inner decoder *diverges* on a marginal frame and smears
  errors frame-wide, leaving SBI little to salvage.

The **Realtek chip RX** (8821) decodes soft in *hardware*, landing in the first
regime natively. The **SDR RX** (gr-ieee802-11 fork) supports both: its default
hard-decision Viterbi lands in the second; a soft-decision path
(`GR_SOFT_VITERBI`) restores the first. Soft information helps here, at the inner
decoder — not at the outer code; see
[Soft information](#soft-information-inner-decoder-vs-outer-code).

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

## Frequency diversity (a complementary erasure source)

The outer erasure code recovers symbols that don't arrive, whatever the cause —
including a whole channel lost to a narrowband fade or interferer. devourer's
per-packet frequency hopping (see **`docs/frequency-hopping.md`**) turns that
into a frequency-diversity code almost for free: with one hop per packet the
outer code's symbols, emitted in order, land on the hop channels round-robin, so
a block of *N* symbols is spread across *N_ch* channels. A single dead channel
then erases only ⌈*N* / *N_ch*⌉ of each block's symbols, which the MDS
Reed-Solomon outer code recovers as long as

```
repair_count  ≥  ⌈N / N_ch⌉
```

— converting a catastrophic single-channel outage (which otherwise loses whole
blocks regardless of how much parity you add) into an ordinary erasure. The
receive side is the existing decoder: it dedups by `(block_id, symbol index)`,
so symbols recovered on different channels — from several adapters, a wideband
SDR, or a lockstep-hopping radio — simply add up. `hop_diversity_sim.py` proves
the recovery threshold against the real codec, and `hop_rx_combine.py` is the
front-end-agnostic combiner.

## Two receive scenarios, one shared framing

The SBI framing, outer code, and per-sub-block CRC erasure decision are identical
for both receivers. Only the receiver — and thus the inner decode — differs:

1. **chip↔chip** (`fused_fec_link.py`, `fused_fec_tx.py`, `fused_fec_rx.py`):
   devourer 8812 TX → devourer 8821 RX. The chip gives only hard corrupted
   bytes; localization is the per-sub-block CRC. `fused_fec_rx` runs a baseline
   and an SBI decoder in lockstep and reports the gain.
2. **SDR-RX** ([`sdr2wifi`](https://github.com/josephnef/sdr2wifi)'s
   `fused_fec_rung3.py`): devourer 8812 TX → USRP B210 RX via the
   [`gr-ieee802-11` fork](https://github.com/josephnef/gr-ieee802-11). Same SBI framing, same
   `FusedFecReceiver`. The fork surfaces FCS-failed frames via `GR_KEEP_CORRUPTED`
   (mirror of devourer's env) and can decode soft (`GR_SOFT_VITERBI`). The bridge
   keeps only encoding ≥ HT PDUs: an 802.11n frame's legacy L-SIG cover is also
   decoded as a garbage legacy frame, which `KEEP_CORRUPTED` would otherwise
   surface as a spurious corrupt frame.

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
| `soft_erasure_fec.py` | errors-and-erasures Reed-Solomon (BCH form) + soft-reliability GMD; the reference that quantifies the inner-vs-outer soft-information question |
| `fec_ab_sim.py` | the SBI-vs-plain-block-FEC A/B over measured channels (does SBI beat just adding parity, at equal overhead?) |
| `hop_diversity_sim.py` | frequency-diversity recovery vs the real RS codec — the single-channel-loss threshold (see `docs/frequency-hopping.md`) |
| `hop_rx_combine.py` | diversity-RX combiner — merge per-channel symbol feeds into one erasure decode |
| `test_*.py` | unit tests for each module (215 in the suite) |

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

In [`sdr2wifi`](https://github.com/josephnef/sdr2wifi) (needs the
[`gr-ieee802-11` fork](https://github.com/josephnef/gr-ieee802-11) + GNU Radio 3.10):

```sh
sudo bash run_fused_rung3.sh    # 8812 TX HT MCS7 → B210 RX → SBI salvage
```

## Results

- **chip↔chip, real silicon, 15 s, MCS7, ch6:** 4086 frames, 1072 corrupt
  surfaced, 6013 sub-blocks salvaged, **FUSED-FEC GAIN = 129 RS blocks** the
  baseline lost (≈13 % lift in delivered blocks).
- **SDR-RX, over real air, marginal HT MCS7** (8812 TX → B210, deterministic
  capture-replay so no USRP-overflow confound):
  - *hard-decision* Viterbi: 5 of 1795 HT frames pass the FCS clean; SBI salvages
    3.6 sub-blocks per corrupt frame and lifts 4 baseline RS blocks to 21
    (**FUSED-FEC GAIN = 17**).
  - *soft-decision* Viterbi (`GR_SOFT_VITERBI`): 489 of 1809 HT frames pass clean
    and ≈6.9 sub-blocks survive per corrupt frame (≈2× the hard path); the
    baseline alone reaches the full 21 blocks, so SBI has nothing left to add.

  The two paths reach the same delivered payload by complementary routes: hard
  decode leans on SBI salvage, soft decode recovers the frames outright.
- **chip RX vs SDR RX, matched operating point.** The fair cross-receiver metric
  is *conditional sub-block survival* — of a corrupt frame's sub-blocks, the
  fraction that still pass their CRC (what SBI salvages) — compared at equal
  corrupt-frame rate. (Matched on corrupt rate, not SNR: the chip's pilot-based
  SNR meter reads ~31–34 dB even at 80 %+ corrupt while the SDR's EVM-based one
  reads ~21 dB, and without an RF splitter the two antennas are not on one
  channel.) Measured: the Realtek 8821 (hardware soft decode) survives 81 % at
  42 % corrupt, 69 % at 83 %, 55 % at 97 %; the B210 fork's **soft** Viterbi
  survives 69 % at 73 % corrupt — on the chip's curve (parity); its **hard**
  Viterbi survives 36 % at ~100 % corrupt — far below. Soft decode brings the
  SDR's localization to chip parity; hard decode does not.

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

The SDR-RX path carries three changes in the
[`gr-ieee802-11` fork](https://github.com/josephnef/gr-ieee802-11):

- **`lib/decode_mac.cc`** — `GR_KEEP_CORRUPTED` surfaces FCS-failed *legacy*
  PSDUs tagged `crc_ok=#f` instead of dropping them (mirrors devourer's env).
- **`lib/frame_equalizer_impl.cc`** — HT/VHT/MIMO frames are decoded here (not in
  `decode_mac`); a `pdu` message port publishes them (with `crc_ok` under
  `GR_KEEP_CORRUPTED`) and stamps per-frame `snr`/`encoding`, which the hier
  block forwards to `mac_out`.
- **`lib/frame_equalizer_impl.cc` + `lib/viterbi_decoder/`** — the soft-decision
  path under `GR_SOFT_VITERBI` (default off, hard path bit-identical): a max-log
  soft demapper (per-coded-bit LLRs for BPSK/QPSK/16/64-QAM) feeds a soft-input
  Viterbi (scalar + SSE2).

[`sdr2wifi`](https://github.com/josephnef/sdr2wifi) holds the over-air receiver (`fused_fec_rung3.py`), the
software-loopback capstone (`fused_fec_rung1.py`), and validation tools
(`keep_corrupted_check.py`, `ht_hier_check.py`, `iq_diag.py`, `iq_fec_diag.py`).

## Soft information: inner decoder vs outer code

An SDR computes a per-coded-bit LLR at the demapper that a Wi-Fi chip never
exposes. That soft information has exactly one place it helps, and the
distinction sets where an SDR earns its place on RX.

**Inner decoder — soft helps.** Feeding real LLRs to a soft-input Viterbi
(`GR_SOFT_VITERBI`) keeps a marginal frame's residual errors *localized* instead
of letting the hard decoder diverge frame-wide — the structure SBI exploits. The
[Results](#results) show this over real air. The Realtek chip already decodes
soft in hardware, so the soft Viterbi brings the SDR to parity with the chip, it
does not surpass it.

**Outer code — soft does not help.** The shipped outer RS is erasure-only and
takes its erasures from the per-sub-block CRC. `soft_erasure_fec.py` tests the
alternative — a BCH-form errors-and-erasures RS whose erasures are chosen by soft
per-symbol reliability (min |LLR| over a symbol's bits), swept GMD-style — and
the result is negative for this link:

- Against the per-sub-block CRC's *perfect* erasure flag, binary CRC-erasure
  beats soft-reliability GMD: a CRC locates erasures exactly, soft only ranks
  them and can be fooled by a confidently-wrong symbol. Soft beats only a
  no-side-information errors-only decode.
- Soft's one structural edge is dropping the CRC bytes for extra parity, but that
  wins only at large per-symbol overhead: at the link's 32-byte symbols
  (CRC ≈12 %) CRC-erasure wins decisively; soft pulls ahead only near 50 %
  overhead (≈4-byte symbols), which the video link never uses.

So at the outer code the chip and SDR are equivalent — the chip's
`KEEP_CORRUPTED` supplies the same CRC erasure flag — and the SDR's soft
advantage is confined to the inner decoder. `soft_erasure_fec.py --mode
genie|overhead` is the reproducible reference for both regimes.

## Is SBI worth it? Fused vs. plain block FEC

The architectural question for the whole initiative: does SBI's per-sub-block
machinery beat simply spending the same overhead on more Reed-Solomon parity (the
wfb-ng model)? `fec_ab_sim.py` decides it by running both over the MEASURED
channels (real survivor distributions) at equal on-air overhead — plain pays no
CRC tax and so buys ~6 % more parity; SBI pays the tax but recovers surviving
sub-blocks from corrupt frames.

The split is structural. Plain treats a CRC-failed frame as a whole-frame
erasure, so the overhead it needs scales with the corrupt-frame *rate*: it
tolerates a corrupt rate up to `ov/(1+ov)` (33 % at 50 % overhead) and is useless
beyond it. SBI loses only the bad sub-blocks, so its overhead scales with the
per-frame corruption *fraction*: with localized corruption (measured survivor
fraction f ≈ 0.69) it tolerates ≈ `1/(1−f)` ≈ 3× the corrupt rate at the same
overhead. Measured source-delivery crossover (localized corruption, 25 %
overhead):

| corrupt-frame rate | plain | SBI |
|--------------------|-------|-----|
| 5 %  | 0.98 | 1.00 |
| 10 % | 0.89 | 0.99 |
| 20 % | 0.56 | 0.93 |
| 30 % | 0.26 | 0.80 |
| 50 % | 0.02 | 0.41 |

Below ~10 % loss the two are close and SBI's CRC tax is not worth it; above ~15 %
loss SBI wins decisively, and the deeper the link the larger the gap. That is the
high-loss regime a long-range link runs in — where fused FEC earns its
complexity.

The win is contingent on corruption staying localized. With frame-wide corruption
(the SDR hard-Viterbi channel, survivor fraction 0.36 at ~100 % corrupt) neither
inter-frame scheme recovers — so the soft inner decoder (which keeps corruption
localized) and SBI are complementary, not independent.

## References

- Jamieson & Balakrishnan, "PPR: Partial Packet Recovery for Wireless Networks", SIGCOMM 2007.
- Han et al., "All Bits Are Not Equal — A Study of IEEE 802.11 Communication Bit Errors", INFOCOM 2009.
- Han et al., "Maranello: Practical Partial Packet Recovery for 802.11", NSDI 2010.
- Abdel-Khalek & Heath, "A Cross-Layer Design for Perceptual Optimization of H.264/SVC with UEP", JSAC 2012.
- Shokrollahi, "Raptor Codes", IEEE Trans. IT 2006 (RaptorQ: RFC 6330). Roca et al., RLC: RFC 8681.
- Rizzo, "Effective Erasure Codes for Reliable Computer Communication Protocols", 1997 (the GF(2⁸) Vandermonde RS).
- Forney, "Generalized Minimum Distance Decoding", IEEE Trans. IT 1966 (the soft-reliability erasure ladder in `soft_erasure_fec.py`).
