# Narrowband (5 and 10 MHz) channels

This document describes how devourer runs a Realtek Jaguar adapter in 5 MHz and
10 MHz channel bandwidths — half and quarter of a standard 20 MHz channel — and
the machinery that makes it work across three chip generations. Narrowband
trades throughput for link budget: halving the bandwidth roughly halves the
noise power the receiver integrates, which is worth ~3 dB per octave of range
on a clean link. It doubles as a porting guide, because the underlying trick is
the same everywhere and the walls are chip-specific.

## The one idea: underclock the baseband, leave the RF alone

A narrowband channel is **not** a different RF tune. The RF synthesizer stays in
its ordinary 20 MHz mode — same LO, same channel, same 20 MHz analog filter
front end. What changes is the **baseband sample-clock rate**: the ADC and DAC
run at a divided-down clock, so the same OFDM machine emits and captures a
proportionally narrower signal. 10 MHz halves the ADC/DAC clocks; 5 MHz quarters
them.

Two consequences fall out of this and are worth internalizing before touching a
register:

- **Radiotap stays 20 MHz.** The modulation the frame carries is unchanged — it
  is a 20 MHz-shaped waveform played out of a slower DAC. A monitor sniffer
  reports 20 MHz; the *only* witness that anything narrowed is a wideband SDR
  measuring occupied bandwidth, or another narrowband receiver decoding it. Any
  validation that relies on a second Wi-Fi card's reported bandwidth is blind
  here.
- **The MAC still thinks it is 20 MHz.** `REG_WMAC_TRXPTCL_CTL` (0x668) BW bits
  stay cleared, exactly as for 20 MHz. Narrowband is a pure PHY re-clock for
  monitor/injection use; the vendor drivers' AP/STA-mode narrowband additionally
  scales SIFS and strips CCK, which devourer does not need for injection.

Because the RF is untouched, narrowband is applied as an **end-of-bring-up
retune** on an already-tuned channel: bring the chip up at 20 MHz, then re-clock.

## Per-generation register machinery

All three generations do the same thing — divide the ADC and DAC clocks and tell
the baseband it is "small BW" — but the register block differs.

### Jaguar1 (RTL8812AU / RTL8811AU / RTL8814AU) — the shared `0x8ac` block

The 8812A drives its bandwidth through BB register `0x8ac` (`rRFMOD_Jaguar`)
under mask `0x003003C3`, whose fields are:

| Field | Bits | Role |
|-------|------|------|
| rf mode | [1:0] | 0 = 20 MHz (kept for narrowband) |
| small BW | [7:6] | 0 normally; 1 = 5 MHz, 2 = 10 MHz |
| ADC clock | [9:8] | receive sample clock divider |
| DAC clock | [21:20] | transmit sample clock divider |

The `[9:8]` field is unambiguously the ADC clock: the vendor's own
`phy_FixSpur_8812A` pokes it and comments it "ADC clock 160M / 80M". Normal
20 MHz sits at ADC `[9:8]=2`, DAC `[21:20]=3`. Narrowband steps both down one
notch per octave:

| Bandwidth | DAC `[21:20]` | ADC `[9:8]` | small BW `[7:6]` | SDR lobe |
|-----------|---------------|-------------|------------------|----------|
| 20 MHz | 3 | 2 | 0 | 16.8 MHz |
| 10 MHz | 2 | 1 | 2 | 8.2 MHz |
| 5 MHz | 1 | 0 | 1 | 4.1 MHz |

`0x8c4[30]` (ADC buffer clock) is held at 0, and `phy_FixSpur_8812A` is skipped
for narrowband (it would overwrite `[9:8]`). Bench-characterized fact worth
knowing for porting: **the DAC code sets the transmitted lobe width, the ADC
code sets receive sensitivity** — sweeping ADC while transmitting does nothing
to the emission, while at 10 MHz an ADC of 1 receives roughly 2.5× better than 0.
The codes are overridable via `DEVOURER_NB_ADC` / `DEVOURER_NB_DAC` for
uncharacterized cuts.

The **RTL8814AU (4T4R)** reaches the same block but with a different field
encoding, and it is instructive. Its `phy_SetBwRegAdc_8814A` only pokes
`0x8ac[1:0]` (a 20/40/80 mode selector with no sub-20 value), so the divider
looks absent — but the vendor's own comment documents the full field set as
`[28, 21:20, 16, 9:6, 1:0]`, which is the **Jaguar2 (8822B) layout**: the ADC
clock is `[9:8]+[16]` and the DAC clock is `[21:20]+[28]`, with the `[16]`/`[28]`
high bits dominating (a divide touching only `[9:8]`/`[21:20]` does nothing on
the 8814). Clear those high bits and write the 8821C/8822B divide codes verbatim
(10 MHz → 3/3, 5 MHz → 2/2, small-BW 2/1) and the 8814 narrows exactly as an
8822B does — TX and RX. The lesson: the same register block can carry the same
fields with a *different* value encoding per die; a divide that reads dead may
just be landing in the wrong sub-field.

### Jaguar2 (RTL8822BU / RTL8821CU) — `0x8ac` packed, plus an RF re-latch edge

Jaguar2 uses the same `0x8ac` register with the small-BW field at `[7:6]` and
the ADC/DAC clock fields; it also sets `0x8c4[30]=0` and `0x8c8[31]=1`, and the
RF18 bandwidth bits stay in their 20 MHz encoding. The bit values differ from
the 8812A's — the block is shared, the divide encoding is not.

The Jaguar2 wall: **the 8822B RF synthesizer only re-latches its internal
channel state on an RF18 *value edge*.** Narrowband is applied as an
end-of-bring-up retune, and the RF18 value it wants is the same 20 MHz-mode
value the chip already holds — so a plain rewrite is a no-op and the RF stays
latched to its pre-re-clock internals, leaving the receiver deaf and the
transmitter emitting nothing decodable (band-independent). The kernel driver
never trips this because its flow always tunes *through* a channel change (an
RF18 edge) around any bandwidth switch. The fix is to force the edge: write RF18
once with a substitute channel byte, then the real value. (The 8821C variant
does not share this dependence.)

### Jaguar3 (RTL8822CU / RTL8822EU) — the `0x9b0`/`0x9b4` dividers

Jaguar3 moved the block: small-BW lives in `0x9b0[7:6]`, the ADC/DAC dividers in
`0x9b4`, with the RF18 bandwidth bits again at the 20 MHz encoding. The 8822E
additionally needs a MAC-clock reconfiguration (`REG_AFE_CTRL1 0x24[21:20]` plus
the TSF/EDCA microsecond-tick dividers `0x55c`/`0x638`) and CFR/TX-shaping
tweaks, and a DAC-FIFO soft reset to suppress a 5 MHz image; the 8822C narrowband
works without them. The 8822E's vendor 5 MHz DAC divide code is dead on real
silicon — only the 8822C code emits the lobe, a fact the OpenHD 8812EU ecosystem
converged on independently.

### Common to all generations

- **A baseband reset relatches the divided clock tree.** Toggling MAC `0x0`
  BIT16 (the `_iqk_bb_reset` mechanism) after the re-clock makes the DAC/DFE
  pick up the new sample rate. Jaguar3 needs it; the Jaguar2 narrowband path
  performs it; the 8812A happens not to require it.
- **TX power folds to the 20 MHz column.** The per-rate regulatory `txpwr_lmt`
  tables carry no narrowband rows. A raw 5/10 bandwidth index reads as HT40 and
  misses the table entirely — leaving TX power unclamped at the rail. Narrowband
  must fold to the 20 MHz power column (the RF is a 20 MHz tune anyway).

## The walls (in case you hit the same one)

Narrowband looks like a two-register change and is mostly a collection of
chip-specific traps. The ones this port paid for, current-state:

1. **8822B RF18 re-latch edge** (above). Symptom: SDR shows a perfectly narrow
   lobe, but nothing decodes and no valid frame airs — the classic "the TX side
   works, the whole thing is dead" signature of a stale RF latch.

2. **8821A cannot divide its DAC clock.** The RTL8821AU is healthy at 20 MHz but
   its 1T1R clock tree couples the DAC clock to the TX DMA/USB path. Dividing it
   starves TX: bulk-out submissions fail (libusb `rc -2`) at a rate that scales
   with divide depth — ~35 % failures at the 10 MHz DAC code, ~72 % at 5 MHz,
   0 % at full clock. This is a per-die difference *within* Jaguar1: the 8812AU's
   2T2R baseband is unaffected. devourer gates narrowband to the 8812 die and
   falls the 8821A back to a 20 MHz baseband. The lesson generalizes: a shared
   register block does not guarantee shared behavior; validate each die on
   hardware.

3. **5 MHz at 5 GHz is CFO-limited.** Quartering the clock quarters the OFDM
   subcarrier spacing (78.125 → ~19.5 kHz), so the carrier-frequency-offset
   budget shrinks 4×, while the absolute crystal offset between a TX/RX pair is
   ~2.2× larger at 5.2 GHz than at 2.4 GHz for the same ppm. A far-offset crystal
   pair is therefore **bimodal per bring-up** at 5 MHz/5 GHz — it syncs on one
   power-up and is deaf on the next — while a closer-crystal peer decodes the
   same transmitter and the same pair is stable at 2.4 GHz. This is physics, not
   a driver bug; the durable fix is a crystal-cap trim lever (tracked
   separately).

4. **The same die-family can encode the divider differently.** The 8814A's
   bandwidth register looked like a mode selector (`0x8ac[1:0]`) with no
   narrowband value, and the 8812A divide (which writes `[9:8]`/`[21:20]`)
   narrows nothing on it. The trap is assuming "no divide field" when the field
   is there under a different encoding — the 8814 uses the 8822B's `[16]`/`[28]`
   high bits (see the Jaguar1 section). Read the vendor's field comments, not
   just the code it executes.

## Validation methodology

Because radiotap lies about bandwidth, narrowband is validated two ways, and
both are needed — the first proves the emission narrowed, the second proves it
is still coherent.

### SDR occupied bandwidth

Transmit continuously and measure the occupied bandwidth of the emission with a
wideband SDR, isolating the DUT's footprint from ambient with a differential PSD
(TX PSD minus a silent-baseline PSD). A working narrowband TX halves (10 MHz) or
quarters (5 MHz) the −10 dB lobe width relative to 20 MHz. This is the only test
that catches a wrong divide code — but it *cannot* distinguish a clean narrow
lobe from a narrowed-but-garbage one (an aliased or image-corrupted spectrum can
read the right width and decode nothing).

### Cross-RX hit count

Run a narrowband transmitter on one adapter and a narrowband receiver on
another, counting delivered frames. This is the discriminator: it proves the
divided ADC path actually demodulates a real narrowband signal, not just that
the lobe is narrow. Pair a chip under test with a known-good narrowband peer of
another generation to isolate TX-side from RX-side faults. When a cross-RX cell
reads zero, an arbiter receiver (a third, closer-crystal adapter) separates a
real fault from CFO bimodality: if the arbiter decodes the same transmitter, the
zero is the crystal pair, not the code.

### The divide-code sweep

`DEVOURER_NB_ADC` / `DEVOURER_NB_DAC` expose the ADC/DAC clock fields for a grid
sweep against the SDR (TX lobe) and cross-RX (decode quality). This is how the
8812AU codes above were pinned rather than guessed, and how the 8821A DAC-starve
was characterized (the TX failure rate versus divide depth).

## Using it

`DEVOURER_NB_BW=5` or `=10` on the demos selects narrowband; the library exposes
it as `CHANNEL_WIDTH_5` / `CHANNEL_WIDTH_10` on `SelectedChannel`, and
`IRtlDevice::GetAdapterCaps().narrowband_ok` reports whether the running chip
supports it. Support today: **Jaguar2 (8822B/8821C) and Jaguar3 (8822C/8822E)**
fully, and **Jaguar1 on the 8812AU/8811AU and the 8814AU** — every generation.
The 8821A is the one exclusion (its DAC-clock divide starves TX; see the walls).

Test scripts: `tests/jaguar2_narrowband_sdr.sh` and
`tests/jaguar1_nb_divide_sweep.sh` (SDR occupied-bandwidth), and
`tests/narrowband_cross_rx.sh` (cross-generation decode).
