# precoder — pre-modulator subcarrier encoder

Proof-of-concept that turns devourer's TX path into a coarse, packet-bounded
frequency-domain transmitter: choose what each OFDM **data subcarrier** carries
(±1 BPSK), and the encoder solves for the PSDU bytes that make the chip emit it
— no 8051 firmware change. The chip's pipeline is fixed but fully deterministic:

```
PSDU bytes → scrambler (x⁷+x⁴+1) → BCC K=7 (133,171) r=½
           → interleaver → BPSK map (0→+1, 1→−1) → pilots → IFFT → CP → DAC
```

Every stage above the constellation is invertible (or, for the rate-½ code,
a linear map onto a 2ᵏ subspace), so we run it backwards.

Scope (locked by the plan): RTL8812AU / RTL8821AU / RTL8811AU, single-stream
BPSK, BCC (not LDPC), long GI, 20 MHz. RTL8814AU is out of scope (its TX is
unreliable after USB passthrough cycles).

## Setup (uv)

```sh
cd tools/precoder
uv venv
uv sync                 # numpy + pytest (dev)
uv run pytest           # 31 DSP known-answer + round-trip tests
```

Phase-B SDR extra (optional): `uv sync --extra sdr`.

## Files

| file | what |
|------|------|
| `encode_subcarriers.py` | the encoder: forward model **and** exact inverse, plus CLI |
| `seed_probe.py` | discover / characterise the chip scrambler seed (rx + bruteforce) |
| `fft_capture.py` | Phase-B per-subcarrier IQ verification (+ runnable `--self-test`) |
| `test_pipeline.py` | pytest: scrambler/BCC/interleaver KATs + pipeline round-trips |

The repo-level end-to-end smoke is `tests/precoder_smoke.py` (skips without numpy).

## End-to-end recipe

```sh
# 0. Build the C++ side from the repo root.
cmake -S . -B build && cmake --build build -j      # -> build/PrecoderDemo

# 1. Characterise the chip's scrambler seed (does it re-seed per frame?).
#    Run an RX adapter at the precoder's TX channel:
uv run python seed_probe.py --mode rx --rx-pid 0x8813 --channel 6
#    CONSTANT seed -> one shaped PSDU works; VARYING -> use bruteforce (below).

# 2. Encode a target subcarrier pattern (one ±1 per data subcarrier per line;
#    48 values/symbol for legacy). Default --phy legacy matches PrecoderDemo.
uv run python encode_subcarriers.py --pattern target.txt \
    --scrambler-seed 0x5d --psdu-out shaped.bin

# 3. Transmit it.
DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 ./build/PrecoderDemo --psdu shaped.bin

# 4. Phase A — two adapters, no SDR (transport + PHY rate + byte round-trip):
sudo python3 ../../tests/precoder_roundtrip.py \
    --tx-pid 0x8812 --rx-pid 0x8813 --psdu shaped.bin --channel 6
# 5. Phase B (rigorous, gated on A): capture 20 MHz IQ on a >=20 Msps SDR and
uv run python fft_capture.py --iq frame.cf32 --pattern target.txt
```

## Verification tiers — what each one actually proves

The sorting question for **any** observable is: *does it expose the received
**data-symbol** value `Y(k)` per subcarrier?* Only things that do can confirm
per-subcarrier control. Everything else is computed *after* the chip already
decided what each subcarrier was, so it's blind to our shaping.

| Tier | Observable | Exposes `Y(k)`? | Proves |
|------|-----------|:---:|--------|
| 0 | offline KATs | n/a | model vs **spec** (software) |
| 1 | decoded bytes | ❌ | transport + PHY rate |
| 2 | RSSI/EVM/SNR | ❌ | link health only |
| 3 | CSI `H(k)` | ❌ | the **preamble** channel (not data) |
| 4 | BB dbgport post-FFT IQ | ✅ | **per-subcarrier control** (no SDR) |
| 5 | IQK TX→RX loopback | ✅* | clean `Y(k)` feed into Tier 4 |
| — | SDR | ✅ | per-subcarrier control (off-chip) |

**Tier 0 — offline KATs** (`test_pipeline.py`, no hardware). The encoder's
`inverse ∘ forward = identity` and the scrambler/BCC/interleaver match the
*standard*. **This is where the model is validated — against the spec, in
software.** The chip is not involved.

**Tier 1 — bit-level loopback, two Wi-Fi adapters, no SDR**
(`tests/precoder_roundtrip.py`). TX one devourer adapter, RX a second; checks
(1) transport, (2) PHY rate `0x04` = 6M OFDM not `0x00` = 1M CCK, (3) bytes
round-trip, and *(optional, 8814 RX only)* the scrambler seed matches.

> ⚠️ Tier 1 does **not** validate the model or prove per-subcarrier content.
> The RX inverts the chip with the **chip's own** tables — never ours — so a
> wrong interleaver/BCC/scrambler table in our code leaves the air subcarriers
> wrong **but the bytes still round-trip**. A wrong table is invisible to a
> byte mirror. Tier 1 proves the *precondition* (it's really 6M OFDM, bytes
> survive), not the shaping.

**Tier 2 — RX-status health diagnostics, free with Tier 1**
(`DEVOURER_DUMP_BODY=1`: rate + per-stream RSSI/EVM/SNR + CRC). EVM is already
parsed (`FrameParser.cpp`), so this is ~free. ⚠️ **Link-health only, never a
content signal:** BPSK ±1 are equal-power and equal-power-allocated, so
per-subcarrier SNR is flat for *any* pattern, and EVM is measured against the
*decoded* symbol — both stay good whether or not our intended value landed.
Also, RX phy-status EVM/SNR is **per-stream, not per-subcarrier**. Useful for
"is the link clean / is it really 6M OFDM", not for shaping.

**Tier 3 — CSI `H(k)`** (PicoScenes-style H2C dump). ❌ Not on the control path:
`H(k)` is estimated from the **L-LTF preamble**, which the chip generates and we
can't shape, so CSI is *invariant* to our data shaping and never yields `Y(k)`.
A channel-characterisation tool, not a mirror. (Also verify the "RTL8812AU is a
first-class PicoScenes target" claim before investing — Realtek CSI support is
far less mature than Intel/Atheros/SDR.)

**Tier 4 — BB debug-port post-FFT IQ (the real no-SDR path).** Routes the
post-FFT per-subcarrier IQ to a readable register → the actual `Y(k)` on the
leading data symbol(s); with a flat/known rig channel ≈ `X(k)`. Grounded in the
tree: `0x8FC` dbgport is already poked at `HalModule.cpp:2167`, and `0x95C`
(`rDMA_trigger_Jaguar2`, "ADC sample mode") is in `Hal8814PhyReg.h:190`.
**Research spike** — undocumented register dance, BB-state brick risk.

**Tier 5 — IQK TX→RX loopback** (`*`if it carries a full PPDU). The path-enable
dance already lives in `Iqk8812a.cpp`/`Iqk8814a.cpp` (the `0x77…` writes). The
risk isn't the regs — it's that IQK loopback carries the calibration *tone* via
the cal state machine, not a descriptor-driven PPDU through normal
preamble/AGC/sync. If a data PPDU survives it (cheap 30-min experiment), 5→4 is
the cleanest channel- and noise-free mirror.

**SDR** (`fft_capture.py` + ≥20 Msps SDR). The lowest-effort *reliable* `Y(k)`
observer; the off-chip counterpart to Tier 4.

## Things the plan got slightly wrong (and how this resolves them)

* **HT MCS 0 doesn't reach the air (decisive).** The plan selected the rate via
  `DEVOURER_TX_MCS=0`, but `RtlJaguarDevice::send_packet` only sets the TX rate
  from the radiotap RATE field (legacy) or the VHT field — it **never reads the
  HT MCS index**. An HT-MCS frame with no RATE field therefore transmits at the
  `MGN_1M` default = **1 Mbps CCK** (DSSS, no OFDM subcarriers). So this PoC uses
  **legacy 6 Mbps OFDM** (BPSK r=½), which `send_packet` honours and which the
  chip really modulates as OFDM. (Fixing the HT path is a one-liner in
  `send_packet`, but it changes shared core TX behaviour the regression matrix
  depends on — deliberately out of scope here.)

* **Numerology.** Consequently the on-air rate is legacy 802.11a/g 6 Mbps:
  **48 data subcarriers, 24 info bits, a 16×3 interleaver** — exactly the plan's
  prose. `--phy legacy` (default) matches it; `--phy ht` (52 SD, 13×4) is correct
  math kept for when the HT path is wired up.

* **Scrambler RX read is 8814-specific.** `GET_RX_STATUS_DESC_RX_SCRAMBLER_8814A`
  reads `desc+16 bits 9–15`; on the 8812/8821 RX descriptor that region holds
  SPLCP/LDPC/STBC/BW, **not** the seed. Since the PoC *targets* 8812/8821, the
  `--mode rx` seed read is only trustworthy with an RTL8814AU sniffer;
  `--mode bruteforce` is the reliable on-air path.

* **Per-PPDU re-seeding.** 802.11 picks a fresh scrambler seed per frame. If the
  chip does that (seed_probe tells you), a single pre-descrambled PSDU can't
  pin the pattern — brute-force 128 seed variants and the matching frames carry
  the target. Only a chip that re-uses a constant seed lets one PSDU work.

* **RTL-SDR can't do 20 MHz.** It tops out ~2.4 MS/s; an 802.11 20 MHz capture
  needs a ≥20 Msps SDR (HackRF/USRP/Lime/bladeRF). `fft_capture.py` reads an IQ
  file or a SoapySDR device, not an RTL-SDR at 20 MHz.

* **"Data frame".** Data/ToDS frames get NAK'd by the chip in monitor mode
  (`txdemo/main.cpp`), so `PrecoderDemo` injects a **probe-request** with the
  canonical SA — the SA matcher recognises it identically.

## Real-frame placement (offset + entry_state)

The encoder's contract is purely bit-domain: given a target for a contiguous run
of OFDM data symbols, starting from BCC `entry_state` at scrambler phase
`offset`, it emits the bytes that reproduce it. In a real frame the scrambled
stream is `[SERVICE(16b) | MAC header | body | tail | pad]`, so the body does
**not** start at the head:

* `PrecoderDemo` prepends a 24-byte MAC header → body starts at scrambled-bit
  offset `16 + 24·8 = 208`. For legacy (`N_DBPS=24`), 208 is **not** a symbol
  boundary (208 = 8·24 + 16), so the body starts mid-symbol-8; the first fully
  controllable symbol begins at bit 216 (= 9·24), i.e. one byte into the body.
* For **exact** per-subcarrier control of a body symbol, encode with the
  matching `--offset` and `entry_state` (the BCC state the SERVICE+header
  leave). Use `encode_subcarriers.bcc_final_state(scrambled_prefix_bits)` to
  compute the state; pad the body so the target lands on a 24-bit boundary.
* For the **byte-level** round-trip (Phase A) the offset is irrelevant — the
  bytes come back verbatim regardless.
