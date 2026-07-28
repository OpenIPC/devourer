# The "warm TX degradation" that wasn't — a measurement post-mortem

A field report described an RTL8812AU whose 64-QAM rates died across repeated
sessions while the robust rates stayed perfect, recovering only on a USB power
cycle. Investigating it produced, in order: a latched-chip-state root cause, a
thermal root cause, a band-switching root cause, and a degraded-adapter root
cause. **All four were wrong, and all four had the same origin.**

The ground station was a TP-Link Archer T3U — an RTL8822BU with *internal*
antennas. Its modulation cliff sits inside the 64-QAM rates, so it could not
measure what it was being asked to measure.

## The measurement that ends it

Two RTL8822BU adapters. Identical silicon, identical code path, same
transmitter, same channel, minutes apart. The only difference is antennas:

| ground station | MCS1 | MCS3 | MCS4 | MCS5 | MCS6 | MCS7 |
| --- | --- | --- | --- | --- | --- | --- |
| Archer T3U — **internal** | 98.1 | 97.8 | 98.3 | 79.1 | 49.0 | **2.9** |
| Comfast CF-924AC V2 — **two external** | 95.8 | 95.4 | 95.8 | 95.7 | 94.4 | **95.9** |

One is a textbook waterfall rolling off through 64-QAM. The other is flat with
margin to spare. Both adapters are healthy.

## Why a marginal receiver imitates a degrading transmitter

A receiver perched on its cliff swings between "fine" and "zero" on a few dB of
ambient, while the rates below the cliff stay pinned at ~98%. That is precisely
the reported symptom: high-order modulation dies, low rates unaffected,
apparently recovering at random.

Earlier in one session that same T3U delivered MCS7 at 68%; later, 0.6%. The
cliff never moved — the operating point did (co-channel ambient rose from ~19 to
~58 frames/3 s). Power-cycling appeared to "fix" it only because a cycle takes
long enough for conditions to drift back.

## What each wrong answer was, and what killed it

- **Latched chip state.** Refuted by reading the registers: the BB TX-swing is
  rewritten unconditionally at every init by `phy_SetBBSwingByBand_8812A`.
- **Thermal.** Refuted by `tests/thermal_offtime_sweep.sh` — vary only the
  power-off duration, keeping the reset identical, and delivery does not follow
  temperature (best result came from the *shortest* off-time and the *hottest*
  chip). Also by `tests/thermal_causation_probe.sh`: inside one uninterrupted
  session the thermal meter is pinned while delivery still drifts.
- **A transmitter defect from band switching.** A stressor matrix did reproduce
  a 32-point collapse — but judged by an AR9271 sniffer the same stressor moved
  decoded MCS7 from 3721 to 3915 frames, i.e. not at all. The transmitter also
  matched the vendor driver (62.5% vs 69.8%).
- **A degraded 8822BU.** The vendor driver failed on it identically, which
  looked conclusive — but a link-budget shortfall predicts exactly that too. TX
  power moved nothing (front end already near-field saturated at RSSI 67) and
  both RX chains read healthy. Then the CF-924AC settled it.

## Rules this produced

1. **Qualify the receiver before believing a delivery number.**
   `tests/ground_station_qualify.sh` sweeps the ladder and refuses the pairing
   (exit 1) when the test rate is off the flat part. Prefer external-antenna
   adapters as ground stations for high-MCS work.
2. **Measure the noise floor first.** `tests/probe_repeatability.sh` — a single
   MCS7/20 probe here has sd 1.8 and a 5.7-point spread, so effects under ~4
   points are not findings. Several convincing "decay curves" sat inside it.
3. **Corroborate with an independent receiver** before attributing a delivery
   change to either end. An AR9271 and the `reference/` vendor drivers were on
   this bench the whole time; either would have caught this on day one.
4. **A control that fails under two drivers does not prove hardware damage.**
   Run the vendor driver on the *receive* side too (`tests/rx_vendor_ab.sh`),
   and check the physical setup — antennas, placement, gain.
5. **Never attribute causation to an intervention that changes two variables.**
   Removing power both resets chip state and cools the die; no amount of
   power-cycling separates them.

## The one code change that survived

Jaguar1 was the only generation that neither reset the chip at init nor
de-initialised it at teardown, so a session left the chip in ACT with its RF
front end live indefinitely after the owning process exited — and an
autonomously-airing beacon kept transmitting.

`RtlJaguarDevice::Stop()` and the destructor now run
`HalModule::rtw_hal_deinit()`: halt the MAC engines, then the die's card-disable
power sequence, using tables that had sat in `hal/Hal88*PwrSeq.c` since the port
began with nothing calling them.

**It carries no performance claim.** The delivery recovery an earlier revision
cited was measured through the unqualified T3U. The change stands on the
architecture alone: every other generation already does this, and it makes a
stray beacon stop. `tuning.teardown_power_down=0`
(`DEVOURER_TEARDOWN_POWER_DOWN=0`) disables it, which post-mortems need — a
powered-down chip answers every register read with the CARDEMU fill.

## Is the original report real?

Unknown, and deliberately left open. Every measurement taken here was through an
instrument that could not support the conclusion, so nothing gathered so far is
evidence either way. A re-report is worth taking seriously if it comes from a rig
where the ground station passes rule 1, a second receiver corroborates, and the
noise floor is established.

## Harnesses

| script | question |
| --- | --- |
| `ground_station_qualify.sh` | can this receiver measure this rate at all? |
| `probe_repeatability.sh` | how big must an effect be to be real here? |
| `rx_vendor_ab.sh` | is a bad receiver our code, or the hardware? |
| `band_stressor_sniffer.sh` | does a stressor hurt the transmitter, judged independently? |
| `ground_rx_degradation.sh` | which end of the link is losing? |
| `severe_form_hunt.sh` | which stressor, if any, reproduces the collapse? |
| `thermal_offtime_sweep.sh` | cooling, or state reset? |
| `thermal_causation_probe.sh` | does delivery track temperature with nothing else changing? |
| `examples/chipstate` | what state did the last session leave, read without disturbing it? |
