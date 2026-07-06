# Bench testing without lying to yourself: near-field saturation

The most common way a bench test misleads you: you put the TX and RX inches (or
a few feet) apart, run high power, and the link is *worse* than it was at range
— low throughput, dropped frames, garbage video — and you start blaming the NIC,
the antennas, or interference. Usually none of those. The signal is simply **too
strong for the receiver**, and adding power or swapping antennas makes it worse.

Two mechanisms, both power problems that masquerade as sensitivity problems:

- **Front-end saturation.** The LNA and ADC clip. The chip's AGC drops its gain
  to the floor and *still* can't keep the signal in the linear range, so the
  constellation smears — even though the raw signal level is enormous.
- **Multipath self-jamming.** Your own strong signal bounces off nearby walls and
  arrives delayed. To the demodulator those delayed copies are in-band noise.
  Past some power level, adding TX power adds as much reflected noise as signal,
  and the link jams itself. This is why a link that is fine outdoors falls apart
  in a room at ten feet.

## The tell: EVM, not SNR

The counterintuitive part — and the reason people miss it — is that **SNR can
look perfectly healthy while the link is saturating.** Measured on this bench,
sweeping an 8812AU's TX power from low to full while a second adapter reported
per-frame metrics (`tests/saturation_knee_sweep.sh`):

| TX power | RSSI (raw) | SNR | EVM |
|---|---|---|---|
| low | 50 | 18 dB | −28 dB (clean) |
| **knee** | ~60 | 18 dB | **−28 dB (best)** |
| high | 70 | 18 dB | −20 dB (degrading) |
| full | 73 | **18 dB (unchanged)** | **−13 dB (collapsed)** |

SNR sat flat at 18 dB across the whole sweep and told you nothing. **EVM
improved as power rose, then reversed at the saturation knee** — the moment more
signal started making the constellation *worse*. That turnover is the signature.

## Is 25 mW too much next to each other?

Yes. The numbers, at 5.8 GHz:

- Free-space loss is only ~28 dB at 10 cm, ~57 dB at 3 m (10 ft).
- 25 mW = +14 dBm. At 10 cm the receiver sees **+14 − 28 = −14 dBm**.
- The RTL88xx front end starts compressing around **−10 to −20 dBm** input.

So 25 mW at 10 cm lands you squarely *in* compression — which is exactly the
"RSSI −10, link dead" report. These receivers are linear and happy around **−40
to −70 dBm** at the input. To bench two adapters safely you want **≥ 40 dB of
loss between them**: drop TX to ~1 mW (0 dBm), add a 30–40 dB attenuator, or
both. Ten feet in a reflective room is past hard clipping but squarely in the
multipath-desense regime.

## How to bench without the trap

1. **Back the power off.** With the runtime TX-power API (`src/TxPower.h`) this
   is one call — no low-power firmware needed:
   `dev->SetTxPowerOffsetQdb(-80)` drops 20 dB (25 mW → ~0.25 mW). Sweep the
   offset down until EVM stops improving; that knee is your linear operating
   point. `tests/saturation_knee_sweep.sh` does exactly this and prints it.
2. **Add loss instead of distance** when you can't move things apart: a
   30–40 dB SMA attenuator on the conducted path, or just physical separation
   and cross-polarised antennas.
3. **Watch EVM, not SNR.** A strong RSSI with poor EVM is the saturation
   fingerprint. `DEVOURER_RX_ENERGY_MS=500 DEVOURER_LINKHEALTH=1` on
   `WiFiDriverDemo` classifies each window and says so in plain language:

   ```
   <devourer-linkhealth>verdict=SATURATED rssi_dbm=-33 snr_db=19.5 evm_db=-22.5
     cause="strong RSSI but poor EVM — receiver front-end overload and/or the
     strong signal self-jamming via reflections (near-field). SNR alone can
     look fine here"
     fix="REDUCE TX power (SetTxPowerOffsetQdb, e.g. -40..-80 qdB), add an
     attenuator, or increase distance — do NOT add power/antenna"
   ```

4. **Only trust a "weak link" verdict at range.** If the link doctor says
   `WEAK` (low RSSI, low SNR, AGC wide open) *then* more power or a better
   antenna is the answer. If it says `SATURATED`, the answer is the opposite —
   and no amount of antenna tuning will fix a receiver that is clipping.

## The link doctor (`<devourer-linkhealth>`)

`src/LinkHealth.h` maps the RX sensor tuple to one of: `SATURATED`,
`INTERFERENCE`, `WEAK`, `MARGINAL`, `HEALTHY`, `NO_SIGNAL`, each with a
one-line cause and the fix. The discriminators, all from telemetry devourer
already exposes:

- **SATURATED** — strong peak RSSI **and** poor EVM (SNR can look fine). Back
  off power / attenuate / add distance.
- **INTERFERENCE** — not-strong signal, degraded, **and** a high false-alarm
  rate: external energy raising the floor. Hop channels.
- **WEAK** — low RSSI, low SNR, AGC at its ceiling. Genuine range limit: more
  power / antenna helps here.
- **HEALTHY** — good SNR + EVM in the linear range.

It rides the `DEVOURER_RX_ENERGY_MS` cadence (so set both). The thresholds are
calibrated from on-air measurement (the sweep above and the AWGN sweep in
`tests/j3_dig_penalty_sweep.sh`) and unit-guarded in
`tests/link_health_selftest.cpp`.
