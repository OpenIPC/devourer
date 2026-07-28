# Warm-session TX degradation on Jaguar1

A devourer session used to leave a Jaguar1 chip powered up. Not just enumerated
— in the ACT power state with the RF front end live, indefinitely, long after
the process that brought it up had exited. Nothing else in the tree does this:
Jaguar2 resets the chip at init and Jaguar3 has run a full card-disable at
teardown for a long time.

Alongside that, high-rate delivery was observed to decay across back-to-back
sessions and to recover only when power was removed. This document records what
is measured, what was fixed, and — importantly — **which parts of the causal
story did not survive testing**.

## First: know your noise floor

Ten identical back-to-back MCS7/20 probes on an RTL8812AU, nothing changed
between them:

| | mean | sd | range |
| --- | --- | --- | --- |
| MCS7/20 | 67.3% | **1.8** | 65.0 – 70.7 |
| MCS1/20 control | 98.0% | 0.2 | 97.5 – 98.3 |

**A single probe is worth about ±3 points.** Any claimed effect smaller than
~4 points is not detectable one-shot, and several plausible-looking "decay
curves" in this investigation sat inside that band. `tests/probe_repeatability.sh`
measures the floor for a given pair; run it before believing a delivery
difference.

## What is measured

The fix is that `Stop()` and the destructor now power the chip down. Its effect
on an idle gap, same repro before and after:

| stage | before fix | after fix |
| --- | --- | --- |
| 60 s idle, **no** power cycle | thermal 48, MCS7 73.0% | thermal 40, MCS7 **85.8%** |

That is ~13 points, around 7 sd — comfortably real. Before the fix no amount of
idling recovered anything and only pulling VBUS helped; afterwards an ordinary
gap between sessions restores the part.

Delivery also correlates with the chip's thermal meter *within* one probe
sequence: across the ten repeats above, thermal climbed 39 → 45 while MCS7 fell
70.7% → 65.7%, with the control flat and RSSI unchanged (so nothing is losing
transmit *power* — only quality).

## What did NOT survive testing

**"It is thermal" is not proven, and two experiments argue against it.**

Every recovery that works removes power, and removing power does two things at
once: it resets chip state *and* it lets the die cool. No power-cycle experiment
can separate them. Two attempts to separate them failed to support cooling:

- **Off-time sweep** (`tests/thermal_offtime_sweep.sh`) — degrade the part, then
  power it down for 5/15/30/60/120 s before an otherwise identical probe. The
  reset is bit-identical in every arm; only cooling time varies. Temperature
  fell cleanly with off-time (47 → 40), but delivery scattered 63–83% with **no
  relationship to off-time or temperature** — the best result came from the
  *shortest* off-time and the *hottest* chip.
- **Within a single uninterrupted session**
  (`tests/thermal_causation_probe.sh`) — 240 s of continuous max-duty TX, one
  bring-up, no re-init, no state change. The thermal meter stayed **pinned**
  while MCS7 delivery still drifted down ~10% and the MCS1 control held flat.

So heat is a real correlate of delivery in some conditions and demonstrably not
the driver in others. The mechanism behind the recovery-on-power-removal is
**open**. Do not cite this document as evidence that the effect is thermal.

## The fix

`RtlJaguarDevice::Stop()` and the destructor run
`HalModule::rtw_hal_deinit()` — halt the MAC engines (`REG_CR`, `REG_RCR`) then
the die's card-disable power sequence, through the same `HalPwrSeqCmdParsing`
and the same three-way chip dispatch `InitPowerOn` already uses. The sequence
tables (`rtl8812_card_disable_flow` and the 8814A/8821A equivalents) had been
carried in `hal/Hal88*PwrSeq.c` since the port began with nothing calling them.

It is justified on its own terms regardless of the mechanism question: leaving a
radio powered and transmitting-capable after its owning process has exited is
wrong, it is what every other generation already avoids, and it is what makes an
autonomously-airing beacon stop at session end.

## What it does not fix

- **Continuous transmission.** Inside one uninterrupted high-duty session the
  part still drifts down; power-down at teardown cannot help a session that
  never ends.
- **`SIGKILL`.** Neither `Stop()` nor the destructor runs, so the chip stays
  powered. An init-side power-off was considered and rejected: it would run
  immediately before power-on, buying nothing, while adding risk to a bring-up
  path that works.
- **There is no in-flight recovery API.** Until the mechanism is understood
  there is nothing principled to implement, and a call that restored the rate by
  dropping the link would not be a recovery for an airborne link anyway.

## Per-chip

- **RTL8812AU** — where the effect is characterized and the fix validated.
- **RTL8814AU** — barely heats at the same duty (34 → 37) and shows no
  degradation; re-inits cleanly after the power-down.
- **RTL8821AU** — re-inits cleanly; not separately characterized.
- **Jaguar2 (RTL8822BU)** — audited, inconclusive: thermal did not move and the
  available link was too marginal at MCS7 to resolve a small effect. No teardown
  power-down added there on the strength of a non-measurement.

## Harnesses

| script | question it answers |
| --- | --- |
| `probe_repeatability.sh` | how big must an effect be to be real on this pair? |
| `warm_tx_degradation_repro.sh` | does delivery decay across sessions, and what do the sensors say? |
| `thermal_offtime_sweep.sh` | is the recovery cooling, or the state reset? |
| `thermal_causation_probe.sh` | does delivery track temperature with nothing else changing? |

Reading the repro: falling delivery with **flat RSSI** means signal quality, not
transmit power. Falling RSSI would be a different bug.
