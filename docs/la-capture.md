# LA-mode IQ capture — per-tone CSI from Jaguar silicon

No Jaguar-class chip exports per-subcarrier CSI through a normal host
path. phydm's **LA (logic-analyzer / ADC-sampling) mode** is the debug
escape hatch: the baseband DMAs a raw sample bus into the top of the TX
packet buffer, and the host reads it back through the `0x0140`-page +
`0x8000`-window. An offline FFT of a captured L-LTF **is** the full
per-tone channel estimate H(k) — one-shot and slow, but it turns a dongle
into a per-subcarrier channel sounder. Implementation:
`src/LaCapture.{h,cpp}` (one algorithm, per-family `LaRegs` — the
NhmReader pattern); vendored source: `reference/rtl88x2bu` (11AC dialect)
and `reference/rtl88x2cu`/`rtl88x2eu` (JGR3) `hal/phydm/phydm_adc_sampling.c`
+ `phydm_debug.c`.

## Silicon support

`PHYDM_IC_SUPPORT_LA_MODE` (phydm_pre_define.h): **8814A, 8822B, 8821C,
8822C, 8822E**. The **8812A and 8821A have no LA block** — `0x7c0` never
latches the arm bits there and the module reports `no_la_block`
(`la.nosupport` event). Bench-validated on all five supported chips
(USB) plus the **8821CE over PCIe** (same module through `RtlAdapter`;
readback 8 ms via MMIO vs ~2 s over USB control transfers).

| chip | TXFF window | size (HALF mode) |
|---|---|---|
| 8814A | 0x30000..0x40000 | 64 KB |
| 8822B / 8822C / 8822E | 0x20000..0x40000 | 128 KB |
| 8821C / 8821CE | 0x8000..0x10000 | 32 KB (`0x7cc[30]` doubles it) |

## Usage

```sh
# one-shot capture, CCA-triggered, 20 Msps, ADC bus path A:
DEVOURER_LA_CAPTURE=cca/20M/dma0/port:0x880/t200 \
DEVOURER_LA_OUT=/tmp/la.bin DEVOURER_LA_MAX=8192 build/rxdemo

python3 tools/la_decode.py score /tmp/la.bin --offset-mhz 4  # layout scoring
python3 tools/la_decode.py plot  /tmp/la.bin                 # PSD + |x|
python3 tools/la_csi.py /tmp/la.bin --plot h.png             # per-tone H(k)
```

Spec grammar: `<trig>[/<rate>M][/dma<N>][/port:0xNNN[.hdr<H>][.bit<B>]]`
`[/edge<0|1>][/t<us>][/all]` — `<trig>` ∈ `manual|crcok|crcfail|cca|bb|mac`.
`t<us>` is the post-trigger time; the rest of the ring is pre-trigger, so
a `crcok` capture (trigger = frame **end**) holds the whole frame
including the preamble. `DEVOURER_LA_MAX` caps readback and keeps the
**newest** samples (the trigger vicinity). `DEVOURER_LA_DELAY_MS`
(default 2000) is the arm delay after RX is live. The demo writes a
32-byte `DVLA` header + LE u64 records; events: `la.capture`,
`la.timeout`, `la.nosupport`, `la.wedged`.

## Sample format (established empirically — tests/la_cw_score.sh)

At `dma0/port:0x880`, every chip packs **one complex sample per 64-bit
word**: 12-bit two's complement, **I=[11:0], Q=[23:12]** of the low dword
(`qi12_l` in tools/la_decode.py — the naive I/Q order is spectrally
inverted), at exactly the configured sample rate. JGR3 (8822C/E)
additionally carries the **second RX path** in the high dword
(I=[55:44], Q=[43:32], `iq12_h`) — a coherent two-chain capture in one
trigger. CW line scores (peak/median): 8822CU 50.8 dB, 8821C 36.8 dB,
8822BU 23.6 dB, 8814AU 19.1 dB.

## Hardware findings the vendor code hides

- **Manual REG trigger ordering**: the `0x7c0[5]` 0→1 edge must be pulsed
  **after** the dump-enable bit `0x7c0[1]` is set. The vendor sets it
  earlier in `phydm_la_set_mac_iq_dump`; that order never completes on
  real 8822BU silicon (edge consumed before arm).
- **dma_type alone doesn't select a live bus** — the BB dbg-port value
  routes the sampled bus, and `dbg_port=0` is a dead bus (clean poll
  timeout). ADC pairing: `dma0`+`0x880` (path A), `dma1`+`0xa80` (path B).
- **8821C cut A** has no LA clock bit (`0x95c[23]` skipped, like vendor
  `phydm_la_clk_en`); expect a poll timeout there.
- **TX-buffer conflict**: capture windows occupy the top of the TXFF and
  the smoke test's capture→TX→capture cell passes, but don't run a
  capture concurrent with heavy TX. The vendor flow leaks `0x0106=0x69`
  (TXFF debug access); LaCapture restores it.

## Validation

- `tests/la_capture_smoke.sh <PID> [ch] [tx_pid] [vid]` — per-chip:
  manual trigger, crcok trigger, capture→TX→capture. 3/3 PASS on
  8822BU, 8814AU, 8821C(8811CU), 8822CU, 8822EU.
- `tests/la_cw_score.sh` — layout scoring against a B210 CW at a known
  offset (layout-scoring methodology).
- `tests/la_sdr_crosscheck.sh <PID> [ch] [vid] [min_notch]` — the
  **notch protocol**: the B210 transmits an L-LTF train with an
  asymmetric tone set zeroed; the chip's H(k) must reproduce the notch at
  exactly those tones (local-neighbour depth). 8822BU: 21–31 dB on all
  six tones. **Chip-vs-SDR |H(k)| correlation was refuted as a
  methodology**: receivers at different antennas see independent fading
  (measured ~0 correlation with both sides self-consistent).
- `ctest` `la_csi_math` — synthetic L-LTF through a 2-tap channel + CFO +
  AWGN; |H(k)| recovery correlation > 0.99.

## Per-chip CSI quality caveats

- **8822CU (Jaguar3)**: ~8 dB adjacent-tone leakage floor on **both** RX
  paths (common LO — phase-noise/capture-path spreading; unchanged by FFT
  window backoff or multi-period CFO refinement). Single-tone features
  bound at ~5–8 dB at their edges; features ≥2 tones wide resolve at
  15–23 dB. Pass the crosscheck with `MIN_NOTCH=5`.
- The B210 helper (`tests/la_sdr_compare.py txltf`) transmits the LTF
  train as long precomputed buffers — per-burst `send()` calls underrun
  from python and nothing airs.

## Risks

Same class as `BbDbgportReader` (0x8fc/0x95c/0x1c3c writes on a live
demod): every touched register is snapshotted and restored, chip liveness
is checked between phases, and a dead read latches the module wedged
(`la.wedged`). Recovery ladder: `libusb_reset_device` → port-level
usbreset → VBUS power-cycle. Readback cost: two vendor-control reads per
8-byte sample (~1 s per 1 K samples on USB2; MMIO-fast on PCIe).
